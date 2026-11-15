"""3. Control Node: mode switch, PID waypoint tracking, arrival detection.

切到手動時：目標、PID、噴粉、輪速立即全部清掉，之後不再處理任何自動資料。
手動輪速需要持續收到 /manual_cmd（網頁每 0.5 s 心跳），超過 manual_timeout 自動歸零。

ESP32 GPIO17 輸出（畫線，經 mpu_node 的序列埠轉給 ESP32）：
  /esp_gpio_cmd (Float32MultiArray) = [啟用 0/1, PWM 工作比 %, PWM 頻率 Hz]，每個控制週期(20 Hz)都發。
  只有「自動模式、這一段 line=1、且正在前進」時啟用 = 1，其餘情況一律 0。
  工作比 / 頻率可即時調整：ros2 param set /control_node esp_pwm_duty 60.0
  esp_pwm_duty = 100.0 時為持續 3.3V（不是 PWM）。

畫線 / 丟壘包（由 map_node 的 /goal_cmd = [x, y, theta, line, drop] 帶進來）：
  line：只有「這一段 line=1 且正在前進（不是原地轉向）」時才讓 ESP32 GPIO17 輸出（畫線）。
  drop：到達這個點後車子停住，送出 /drop_cmd (Bool True)（mpu_node 轉成 ESP32 丟壘包封包），
        然後一直等 /drop_done（ESP32 回報丟完，經 mpu_node 轉來）才回報 /reached，map_node 才送下一個點。
        drop_ack_timeout 秒內沒等到回報 -> 任務中止（/mission_failed），不會假裝丟成功繼續走。
  切到手動 / 任務取消 / 中止時，等待中的丟壘包流程一併清掉（已送出的那一下無法收回）。
"""
import math

import rclpy
import rclpy.duration
from geometry_msgs.msg import Pose2D, Vector3
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String


def wrap(a):
    """角度(度)包到 [-180, 180)。"""
    return (a + 180.0) % 360.0 - 180.0


class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp, self.ki, self.kd, self.limit = kp, ki, kd, limit
        self.i = 0.0
        self.prev = None

    def reset(self):
        self.i, self.prev = 0.0, None

    def step(self, err, dt):
        self.i = max(-self.limit, min(self.limit, self.i + err * dt))
        # 角度誤差(度)在 ±180 跳變時，用 wrap 取真正的變化量，避免微分尖峰
        # PID 輸入單位=度，輸出單位=度/秒（所以 kp 單位是 (度/秒)/度）
        d = 0.0 if self.prev is None else wrap(err - self.prev) / dt
        self.prev = err
        out = self.kp * err + self.ki * self.i + self.kd * d
        return max(-self.limit, min(self.limit, out))


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        for k, v in dict(wheel_radius=0.10, track_width=0.50,
                         v_max=0.6, v_min=0.15, w_max=70.0,                  # w_max: 最大角速度 (度/秒)
                         goal_tol=0.15, kp_dist=0.8,
                         overshoot_tol=0.5,                 # 離目標 < 此距離且目標已在車後 -> 視為越過目標，直接算到達；
                                                            # 此範圍內也不再啟動原地轉向（避免在目標點旁邊自轉）
                         turn_in=30.0, turn_out=6.0,        # 原地轉向遲滯 (度)：誤差 < turn_out 才算對準、開始前進
                         rpm_deadzone=15.0,                 # 馬達死區補償：原地轉向時每輪最低 rpm（0 = 關閉）
                         heading_tol=3.0,                   # 前進時的角度死區 (度)，誤差更小就不修正
                         goal_timeout=120.0,                # 單一航點逾時 (s)，逾時中止任務
                         pose_timeout=0.5,                  # 超過這麼久沒收到 /robot_pose -> 自動模式停車
                         manual_timeout=1.0,                # 手動指令逾時 (s)
                         spray_while_turning=False,
                         drop_ack_timeout=8.0,              # 丟壘包後等 ESP32 回報完成的最長時間 (s)，要大於 mpu_node 的 drop_ms
                         esp_pwm_duty=100.0,                # ESP32 GPIO17 輸出工作比 (%)，0~100，100 = 持續高電位 3.3V
                         esp_pwm_freq=1000.0).items():      # ESP32 GPIO17 PWM 頻率 (Hz)，限制 100~20000
            self.declare_parameter(k, v)
        self.p = lambda k: self.get_parameter(k).value

        self.pub_motor = self.create_publisher(Vector3, '/motor_rpm_cmd', 10)  # x=left, y=right
        self.pub_reached = self.create_publisher(Bool, '/reached', 10)
        self.pub_failed = self.create_publisher(Bool, '/mission_failed', 10)
        self.pub_drop = self.create_publisher(Bool, '/drop_cmd', 10)         # control -> mpu_node -> ESP32
        self.pub_drop_state = self.create_publisher(Bool, '/drop_state', 10) # True = 正在等丟壘包完成（給網頁顯示）
        self.pub_fail_reason = self.create_publisher(String, '/mission_fail_reason', 10)   # 中止原因（給網頁顯示）
        self.pub_esp = self.create_publisher(Float32MultiArray, '/esp_gpio_cmd', 10)   # control -> mpu_node -> ESP32
        self.create_subscription(Bool, '/drop_done', self.on_drop_done, 10)    # mpu_node(ESP32) -> control
        self.create_subscription(Float32MultiArray, '/goal_cmd', self.on_goal, 10)   # [x, y, theta, line, drop]
        self.create_subscription(Pose2D, '/robot_pose', self.on_pose, 10)
        self.create_subscription(String, '/mode_cmd', self.on_mode, 10)
        self.create_subscription(Vector3, '/manual_cmd', self.on_manual, 10)
        self.create_subscription(Bool, '/goal_cancel', self.on_cancel, 10)    # map -> control

        self.mode = 'manual'
        self.goal = None
        self.goal_t0 = None
        self.pose = None
        self.pose_t = None
        self.turning = False
        self.goal_line = True
        self.goal_drop = False
        self.drop_until = None
        self.manual = (0.0, 0.0)
        self.last_manual_time = self.get_clock().now()
        self.pid_w = PID(1.5, 0.0, 0.1, self.p('w_max'))
        self.dt = 0.05
        self.create_timer(self.dt, self.loop)

    def publish_esp(self, active):
        """通知 ESP32 GPIO17：active=True 輸出 PWM（工作比/頻率取自參數），False 輸出 0V。"""
        duty = max(0.0, min(100.0, float(self.p('esp_pwm_duty'))))
        freq = max(100.0, min(20000.0, float(self.p('esp_pwm_freq'))))
        self.pub_esp.publish(Float32MultiArray(data=[1.0 if active else 0.0, duty, freq]))

    def clear_auto(self):
        """清掉一切自動模式的狀態，並立刻停車、關噴粉。"""
        self.goal = None
        self.goal_t0 = None
        if self.drop_until is not None:
            self.pub_drop_state.publish(Bool(data=False))
        self.drop_until = None
        self.goal_line = True
        self.goal_drop = False
        self.turning = False
        self.pid_w.reset()
        self.send_rpm(0.0, 0.0)
        self.publish_esp(False)

    def on_mode(self, msg):
        if msg.data not in ('auto', 'manual') or msg.data == self.mode:
            return
        self.mode = msg.data
        self.manual = (0.0, 0.0)
        self.last_manual_time = self.get_clock().now()
        self.clear_auto()
        self.get_logger().info(f'mode -> {self.mode}')

    def on_cancel(self, msg):
        if msg.data and self.mode == 'auto':
            self.clear_auto()           # 任務開始/停止/中止：丟掉舊目標並停車

    def on_manual(self, msg):
        self.manual = (msg.x, msg.y)
        self.last_manual_time = self.get_clock().now()

    def on_drop_done(self, msg):
        if msg.data and self.drop_until is not None:     # 只有正在等丟壘包回報時才收；其餘一律忽略
            self.drop_until = None
            self.pub_drop_state.publish(Bool(data=False))
            self.get_logger().info('ESP32 回報丟壘包完成')
            self.pub_reached.publish(Bool(data=True))

    def on_goal(self, msg):
        if len(msg.data) < 5:
            self.get_logger().error(f'/goal_cmd 長度不對（要 [x,y,theta,line,drop]）：{list(msg.data)}')
            return
        if self.mode == 'auto':                 # 手動模式下一律丟棄自動目標
            self.goal = Pose2D(x=float(msg.data[0]), y=float(msg.data[1]), theta=float(msg.data[2]))
            self.goal_line = msg.data[3] >= 0.5
            self.goal_drop = msg.data[4] >= 0.5
            self.drop_until = None
            self.goal_t0 = self.get_clock().now()
            self.turning = True                 # 新目標：先原地轉到對準（前進速度 0），再前進
            self.pid_w.reset()

    def on_pose(self, msg):
        self.pose = msg
        self.pose_t = self.get_clock().now()

    def send_rpm(self, left, right):
        self.pub_motor.publish(Vector3(x=float(left), y=float(right)))

    def fail(self, why):
        self.get_logger().error(f'goal failed: {why} -> mission aborted')
        self.pub_fail_reason.publish(String(data=str(why)))     # 先送原因，再送 /mission_failed
        self.clear_auto()
        self.pub_failed.publish(Bool(data=True))

    def loop(self):
        now = self.get_clock().now()

        if self.mode == 'manual':
            if (now - self.last_manual_time).nanoseconds / 1e9 > self.p('manual_timeout'):
                self.manual = (0.0, 0.0)        # 網頁/心跳中斷 -> 停車
            self.send_rpm(*self.manual)
            self.publish_esp(False)           # 手動模式一律不輸出
            return

        # ---- auto ----
        if self.drop_until is not None:                  # 正在丟壘包：停車，等 /drop_done 才繼續（on_drop_done 回報到達）
            self.send_rpm(0.0, 0.0)
            self.publish_esp(False)
            if now >= self.drop_until:
                self.fail('drop ack timeout（ESP32 沒回報丟壘包完成）')
            return
        if self.goal is not None and (now - self.goal_t0).nanoseconds / 1e9 > self.p('goal_timeout'):
            self.fail('timeout')
            return
        if self.goal is not None and self.pose is None:
            self.get_logger().warn('自動模式有目標，但還沒收到 /robot_pose（fusion/mpu 沒資料），不動作', throttle_duration_sec=3.0)
        if self.goal is None or self.pose is None:
            self.send_rpm(0.0, 0.0)
            self.publish_esp(False)
            return

        if (now - self.pose_t).nanoseconds / 1e9 > self.p('pose_timeout'):
            # 定位中斷（MPU/fusion 當掉）：停車等待，不拿舊姿態繼續算（會原地一直轉）；恢復後自動繼續
            self.send_rpm(0.0, 0.0)
            self.publish_esp(False)
            self.pid_w.reset()
            self.get_logger().warn('/robot_pose 中斷，自動模式暫停', throttle_duration_sec=2.0)
            return

        dx, dy = self.goal.x - self.pose.x, self.goal.y - self.pose.y
        dist = math.hypot(dx, dy)
        err = wrap(math.degrees(math.atan2(dy, dx)) - self.pose.theta)   # 度
        near = dist < self.p('overshoot_tol')
        passed = near and abs(err) > 90.0           # 已越過目標點（目標在車後方）
        if dist < self.p('goal_tol') or passed:
            self.send_rpm(0.0, 0.0)
            self.publish_esp(False)
            self.goal = None
            if self.goal_drop:
                self.pub_drop.publish(Bool(data=True))
                self.drop_until = now + rclpy.duration.Duration(seconds=float(self.p('drop_ack_timeout')))
                self.goal_drop = False
                self.pub_drop_state.publish(Bool(data=True))
                self.get_logger().info('到達：丟壘包')
            else:
                self.pub_reached.publish(Bool(data=True))
            return

        if self.turning:                                   # 遲滯：>turn_in 開始原地轉，<turn_out 才結束
            if abs(err) < self.p('turn_out'):
                self.turning = False
        elif abs(err) > self.p('turn_in') and not near:    # 目標附近不再新開原地轉向
            self.turning = True

        if self.turning:
            v = 0.0
        else:
            v = max(self.p('v_min'), min(self.p('v_max'), self.p('kp_dist') * dist)) * math.cos(math.radians(err))
        w = self.pid_w.step(err, self.dt)
        if not self.turning and abs(err) < self.p('heading_tol'):
            w = 0.0                                        # 角度死區：誤差很小就直線走，避免來回抖動

        # 只在前進時畫線（原地轉彎不畫），要改回舊行為把 spray_while_turning 設 true
        # 這一段要畫線（goal_line）且正在前進才畫；不畫線的路段 GPIO17 關閉
        draw = self.goal_line and (v > 0.0 or bool(self.p('spray_while_turning')))

        L, r = self.p('track_width'), self.p('wheel_radius')
        to_rpm = 60.0 / (2.0 * math.pi * r)
        w_rad = math.radians(w)                            # 差速運動學需要 rad/s
        left = (v - w_rad * L / 2.0) * to_rpm
        right = (v + w_rad * L / 2.0) * to_rpm
        if self.turning:
            # 死區補償：誤差變小時 PID 輸出會低於馬達的啟動轉速，車子轉不動、卡在 turn_out 之外。
            # 原地轉向時每輪至少給 rpm_deadzone，轉進 turn_out 容許範圍後就會開始前進。
            dz = self.p('rpm_deadzone')
            sgn = 1.0 if err >= 0.0 else -1.0
            left = -sgn * max(abs(left), dz)
            right = sgn * max(abs(right), dz)
        self.send_rpm(left, right)
        # ESP32 GPIO17（畫線輸出）：只在「這一段要畫線」且正在前進時輸出（與噴粉同條件）
        self.publish_esp(draw)


def main():
    rclpy.init()
    rclpy.spin(ControlNode())
    rclpy.shutdown()
