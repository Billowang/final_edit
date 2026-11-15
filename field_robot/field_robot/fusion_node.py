"""5. Data Fusion Node: 輪子圈數(平移) + MPU yaw(朝向) -> Pose2D。
馬達節點在原地旋轉時不送資料，所以位置 x,y 不動、朝向 theta 仍由 yaw 更新；
姿態以固定頻率發佈，確保原地旋轉時控制節點也能拿到最新的 theta。

朝向基準（與 map_node 雙向握手）：
  map_node 收到網頁地圖資料後送 /zero_heading_req (Bool True) -> 本節點把當下收到的 yaw 存成基準，
  之後 yaw = 目前 yaw - 基準，也就是收到要求當下的車頭方向 = 0 度（+X）；
  校正完成後回 /heading_zeroed (Bool True) 給 map_node（還沒收到任何 yaw 則回 False）。
校正時 x,y 也一併歸零（參數 zero_position_on_start，預設 True）：路徑座標以本壘為原點，
每次開始工作都視為「車子在本壘、車頭朝一壘(+X)」，跟開機第一次一樣，不會帶著上一趟的舊位置。
開機到第一次校正之前，先用開機第一筆 yaw 當暫時基準，讓姿態有值可用。
中途要重新對位（位置和朝向都歸零）：  ros2 topic pub --once /reset_pose std_msgs/msg/Empty "{}"

GNSS 航向檢查（gnss_node -> /gnss_fix）：
  只在「RTK 固定解 + 直線前進 + 位移 >= gnss_min_baseline_m」時，用 RTK 位移算出實際行進方向，
  跟 MPU 朝向比對：偏差 = GNSS 航向 - MPU 朝向（正 = 實際比 MPU 以為的更偏逆時針）。
  偏差 >= gnss_deadband_deg 且 <= gnss_max_correct_deg 時，每次只修正 gain 倍、最多 gnss_max_step_deg，
  慢慢把 yaw 拉回去；偏差超過 gnss_max_correct_deg 只警告、不動 yaw（多半是 MPU 或方位參數有大問題）。
  結果發佈在 /gnss_check = [偏差deg, 本次修正deg, GNSS距離m, 里程距離m, GNSS/里程]。
  場地 +X（本壘->一壘）的真實方位角：field_x_bearing_deg >= 0 時用它（從北順時針，度）；
  設 -1 = 自動：每次校正朝向後，用第一段直線學出來（只能檢查跑的過程中有沒有飄）。
  GNSS 資料有問題時不使用：定位不是 RTK Fixed、資料中斷、或「GNSS 位移與里程比值不合理」
  （GNSS 卡在原地、跳點都會被這條擋掉），這時只靠 MPU + 輪子里程，yaw 不被 GNSS 動到。
  沒接 GNSS（沒有 /gnss_fix）時等於關閉。
所有 gnss_* 參數都是即時讀取，可隨時  ros2 param set /fusion_node gnss_gain 0.1
"""
import math

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32, Float32MultiArray, Float64MultiArray



def wrap(a):
    """角度(度)包到 [-180, 180)。"""
    return (a + 180.0) % 360.0 - 180.0


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('wheel_radius', 0.10)    # 輪徑 0.20 m -> 半徑 0.10 m
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('zero_position_on_start', True)   # 校正朝向時 x,y 一起歸零
        self.declare_parameter('yaw_timeout_s', 0.5)             # 超過這麼久沒收到 /mpu_yaw -> 不再發佈姿態（控制端會停車）
        self.declare_parameter('yaw_jump_deg', 10.0)             # 相鄰兩筆 yaw 跳超過 (此值 + 300 deg/s * min(間隔, 0.1s)) -> 視為 ESP32 重開
        # ---- GNSS 航向檢查（全部即時生效）----
        self.declare_parameter('gnss_enable', True)              # False = 完全忽略 GNSS
        self.declare_parameter('gnss_correct', True)             # False = 只監測、不修正 yaw
        self.declare_parameter('gnss_accept_float', False)       # True = RTK Float(5) 也接受；預設只收 Fixed(4)
        self.declare_parameter('gnss_min_baseline_m', 1.5)       # 位移至少多少公尺才比對
        self.declare_parameter('gnss_straight_tol_deg', 5.0)     # 這段路 yaw 變化超過就視為不是直線
        self.declare_parameter('gnss_gain', 0.3)                 # 每次修正偏差的比例 (0~1)
        self.declare_parameter('gnss_max_step_deg', 2.0)         # 每次最多修正幾度
        self.declare_parameter('gnss_deadband_deg', 1.0)         # 偏差小於此不修
        self.declare_parameter('gnss_max_correct_deg', 30.0)     # 偏差大於此只警告、不自動修
        self.declare_parameter('gnss_stale_s', 2.0)              # 超過幾秒沒資料 -> 這段作廢重來
        self.declare_parameter('gnss_ratio_min', 0.6)            # GNSS 位移 / 里程 的合理範圍，超出就整段不採用
        self.declare_parameter('gnss_ratio_max', 1.5)
        self.declare_parameter('field_x_bearing_deg', -1.0)      # 本壘->一壘方位角(北=0,順時針)；-1=自動學習
        self.x = self.y = 0.0
        self.raw_yaw = None
        self.yaw = None
        self.yaw0 = None
        self.seg = None                 # GNSS 直線比對的起點 {lat, lon, yaw, odom}
        self.bearing_learned = None     # 自動學到的場地 +X 方位角 (rad)
        self.gnss_t = None              # 上一筆有效 GNSS 的時間
        self.yaw_t = None               # 最近一次收到 /mpu_yaw 的時間 (s)
        self.pub = self.create_publisher(Pose2D, '/robot_pose', 10)
        self.pub_check = self.create_publisher(Float32MultiArray, '/gnss_check', 10)
        self.pub_zeroed = self.create_publisher(Bool, '/heading_zeroed', 10)      # fusion -> map
        self.create_subscription(Float32, '/mpu_yaw', self.on_yaw, 10)
        self.create_subscription(Twist, '/wheel_feedback', self.on_wheel, 10)
        self.create_subscription(Empty, '/reset_pose', self.on_reset, 10)
        self.create_subscription(Bool, '/zero_heading_req', self.on_zero_req, 10)  # map -> fusion
        self.create_subscription(Float64MultiArray, '/gnss_fix', self.on_gnss, 10)  # gnss_node -> fusion
        self.create_timer(1.0 / self.get_parameter('publish_hz').value, self.publish_pose)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_yaw(self, msg):
        t = self.now_s()
        if self.raw_yaw is not None and self.yaw_t is not None:
            jump = wrap(msg.data - self.raw_yaw)
            limit = self.get_parameter('yaw_jump_deg').value + 300.0 * min(t - self.yaw_t, 0.1)   # 度
            if abs(jump) > limit:
                # 物理上不可能轉這麼快 -> ESP32 重開（yaw 從 0 重算）或封包錯亂。
                # 把基準一起位移，讓 fusion 的朝向保持連續，不要讓車以為方向突然變了。
                self.yaw0 += jump
                self.get_logger().warn(f'MPU yaw 突然跳 {jump:+.1f} deg（ESP32 重開？），'
                                       '已補償，朝向保持連續；請檢查 ESP32 供電/接線')
        self.raw_yaw = msg.data
        self.yaw_t = t
        if self.yaw0 is None:
            self.yaw0 = msg.data          # 開機當下朝向 = 0（面向 +X）
        self.yaw = msg.data - self.yaw0

    def yaw_fresh(self):
        return self.yaw_t is not None and self.now_s() - self.yaw_t <= self.get_parameter('yaw_timeout_s').value

    def on_zero_req(self, msg):
        if msg.data:
            self.zero_heading()

    def zero_heading(self):
        if self.raw_yaw is None:
            self.get_logger().warn('收到朝向校正要求但還沒收到 yaw，回報失敗')
            self.pub_zeroed.publish(Bool(data=False))
            return
        self.yaw0 = self.raw_yaw          # 收到要求當下的 yaw 當基準，之後 = 目前 - 基準
        self.yaw = 0.0
        if self.get_parameter('zero_position_on_start').value:
            self.x = self.y = 0.0
        self.seg = self.bearing_learned = None      # GNSS 比對重新開始
        self.get_logger().info(f'heading zeroed (baseline {self.yaw0:.2f} deg)')
        self.publish_pose()               # 先把校正後的姿態送出去
        self.pub_zeroed.publish(Bool(data=True))   # 再回報 map_node：校正完成

    def on_reset(self, _):
        self.x = self.y = 0.0
        self.seg = self.bearing_learned = None
        if self.raw_yaw is not None:
            self.yaw0 = self.raw_yaw
            self.yaw = 0.0
        self.get_logger().info('pose reset to (0, 0, 0)')

    def on_wheel(self, msg):
        if self.yaw is None or not self.yaw_fresh():
            return                      # 沒有可信朝向就不累加位置（累加了也是錯的方向）
        c = 2.0 * math.pi * self.get_parameter('wheel_radius').value   # 圈數 -> 距離
        ds = (msg.linear.x + msg.linear.y) * c / 2.0
        yaw_r = math.radians(self.yaw)      # 三角函數需要弧度
        self.x += ds * math.cos(yaw_r)
        self.y += ds * math.sin(yaw_r)
        if self.seg is not None:
            if ds < -0.001:                 # 倒車：行進方向跟車頭相反，這段不能用
                self.seg = None
            else:
                self.seg['odom'] += max(ds, 0.0)

    # ---------------- GNSS 航向檢查 ----------------
    def start_seg(self, lat, lon):
        self.seg = dict(lat=lat, lon=lon, yaw=self.yaw, odom=0.0)

    def on_gnss(self, msg):
        gp = lambda k: self.get_parameter(k).value
        if not gp('gnss_enable') or self.yaw is None or len(msg.data) < 3:
            self.seg = None
            return
        lat, lon, q = msg.data[0], msg.data[1], int(msg.data[2])
        now = self.get_clock().now().nanoseconds * 1e-9
        if not (q == 4 or (q == 5 and gp('gnss_accept_float'))) or not (math.isfinite(lat) and math.isfinite(lon)):
            self.seg = None                 # 不是 RTK 固定解 -> 不用
            return
        gap = None if self.gnss_t is None else now - self.gnss_t
        self.gnss_t = now
        if self.seg is None or gap > gp('gnss_stale_s'):    # 剛開始 / 資料中斷過 -> 重新取起點
            self.start_seg(lat, lon)
            return
        s = self.seg
        dyaw = wrap(self.yaw - s['yaw'])
        if abs(dyaw) > gp('gnss_straight_tol_deg'):
            self.start_seg(lat, lon)        # 這段有轉彎，不是直線
            return
        dn = (lat - s['lat']) * 111320.0                                  # 往北位移 (m)
        de = (lon - s['lon']) * 111320.0 * math.cos(math.radians(s['lat']))   # 往東位移 (m)
        dist = math.hypot(dn, de)
        if dist < gp('gnss_min_baseline_m'):
            return                          # 走得不夠遠，繼續累積
        if s['odom'] < 0.2:                 # 里程計沒有往前走（被推動/倒車）-> 不可信
            self.start_seg(lat, lon)
            return

        ratio = dist / s['odom']
        if not (gp('gnss_ratio_min') <= ratio <= gp('gnss_ratio_max')):
            self.get_logger().warn(f'GNSS 位移 {dist:.2f} m 與里程 {s["odom"]:.2f} m 差太多 (比值 {ratio:.2f})，'
                                   '這段不採用、不修正 yaw')
            self.start_seg(lat, lon)
            return

        b = math.degrees(math.atan2(de, dn))   # 實際行進方位角（度，北=0，順時針）
        yaw_mid = wrap(s['yaw'] + 0.5 * dyaw)
        bearing_param = gp('field_x_bearing_deg')
        if bearing_param >= 0.0:
            field_b = bearing_param
        elif self.bearing_learned is None:
            self.bearing_learned = wrap(yaw_mid + b)      # 假設此刻 MPU 朝向正確，學出 +X 的方位角
            self.get_logger().info(f'GNSS 學到場地 +X 方位角 = {self.bearing_learned % 360:.1f} deg')
            self.start_seg(lat, lon)
            return
        else:
            field_b = self.bearing_learned

        # 場地 +Y 在 +X 逆時針 90 度，所以 車頭朝向 theta = 場地+X方位角 - 行進方位角
        err = wrap((field_b - b) - yaw_mid)               # GNSS 航向 - MPU 朝向
        ed = err                                          # 度
        step = 0.0
        if abs(ed) > gp('gnss_max_correct_deg'):
            self.get_logger().warn(f'GNSS 航向偏差 {ed:+.1f} deg 超過 {gp("gnss_max_correct_deg"):.0f} deg，'
                                   '不自動修正，請檢查 MPU 或 field_x_bearing_deg')
        elif abs(ed) >= gp('gnss_deadband_deg') and gp('gnss_correct'):
            max_step = gp('gnss_max_step_deg')
            step = max(-max_step, min(max_step, gp('gnss_gain') * err))
            self.yaw0 -= step               # yaw = raw - yaw0，所以 yaw0 減 step => yaw 加 step
            self.yaw += step
        self.get_logger().info(f'GNSS 比對：航向偏差 {ed:+.1f} deg，修正 {step:+.2f} deg，'
                               f'GNSS {dist:.2f} m / 里程 {s["odom"]:.2f} m = {ratio:.3f}')
        self.pub_check.publish(Float32MultiArray(
            data=[float(ed), float(step), float(dist), float(s['odom']), float(ratio)]))
        self.start_seg(lat, lon)            # 下一段從這裡重新累積

    def publish_pose(self):
        if self.yaw is None:
            self.get_logger().warn('還沒收到任何 /mpu_yaw（檢查 mpu_node 的埠與 ESP32），沒有姿態可發佈', throttle_duration_sec=5.0)
            return
        if self.yaw is not None and not self.yaw_fresh():
            # MPU 資料中斷：不再發佈（發舊的 theta 會讓 control 以為朝向沒變而一直自轉）
            self.get_logger().warn('MPU yaw 中斷，暫停發佈 /robot_pose（control 會停車）', throttle_duration_sec=2.0)
            return
        if self.yaw is not None:
            self.pub.publish(Pose2D(x=self.x, y=self.y, theta=wrap(self.yaw)))


def main():
    rclpy.init()
    rclpy.spin(FusionNode())
    rclpy.shutdown()
