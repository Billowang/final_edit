"""9. GNSS Node: 序列埠讀 RTK 接收機的 NMEA GGA -> /gnss_fix (Float64MultiArray)

  data = [lat_deg, lon_deg, fix_quality, num_sats, hdop]
  fix_quality: 0 未定位 / 1 單點 / 2 DGPS / 4 RTK Fixed / 5 RTK Float
  沒有定位時 lat、lon 為 nan。

只負責讀取與解析（含 NMEA 校驗碼），不做融合；資料可不可信由 fusion_node 判斷。

沒接 GNSS 時直接忽略：
  - 埠不存在 -> 只記一次 log，之後每 retry_s 秒靜靜檢查一次，什麼都不發佈（fusion 就等於沒有 GNSS）。
  - 之後才插上去也會自動開始。要完全停用：ros2 param set /gnss_node enable false
防止跟 MPU / 馬達搶同一個序列埠（會讓 MPU 封包被偷走、yaw 亂掉）：
  - 預設埠改為 /dev/mygnss（請用 udev 規則固定名稱，見說明）；不再預設 /dev/ttyACM0。
  - 若 port 解析後跟 reserved_ports（預設 /dev/mympu、/dev/mymotor）是同一個裝置，拒絕開啟。
  - Linux 以獨占模式開啟（exclusive），別的程式已在用就開不起來，不會偷資料。
  - 開啟後 no_nmea_timeout_s 秒內收不到任何有效 NMEA（代表不是 GNSS 裝置）-> 關閉，30 秒後才再試。
Windows 測試用 port:=COM4。
"""
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

try:
    import serial
except ImportError:
    serial = "/dev/mygnss"


def nmea_ok(line: str) -> bool:
    """驗 NMEA 校驗碼：'$' 與 '*' 之間所有字元 XOR == '*' 後兩位十六進位"""
    if not line.startswith('$') or '*' not in line:
        return False
    body, _, chk = line[1:].partition('*')
    try:
        want = int(chk[:2], 16)
    except ValueError:
        return False
    x = 0
    for ch in body:
        x ^= ord(ch)
    return x == want


def dm_to_deg(v: str, hemi: str) -> float:
    """NMEA 的 (d)ddmm.mmmm -> 十進位度；S / W 為負"""
    f = float(v)
    deg = int(f // 100)
    out = deg + (f - deg * 100) / 60.0
    return -out if hemi in ('S', 'W') else out


class GnssNode(Node):
    def __init__(self):
        super().__init__('gnss_node')
        for k, v in dict(enable=True, port='/dev/mygnss', baud=115200,
                         retry_s=5.0, no_nmea_timeout_s=10.0,
                         reserved_ports=['/dev/mympu', '/dev/mymotor']).items():
            self.declare_parameter(k, v)
        self.p = lambda k: self.get_parameter(k).value
        self.pub = self.create_publisher(Float64MultiArray, '/gnss_fix', 10)
        self.ser = None
        self.buf = bytearray()
        self.next_try = 0.0
        self.opened_at = 0.0
        self.last_nmea = 0.0            # 最近一次有效 NMEA（任何句型）
        self.last_gga = 0.0
        self.said_missing = False
        self.create_timer(0.02, self.read_serial)
        self.create_timer(1.0, self.tick)
        self.tick()

    # ---------------- 開埠 / 監看 ----------------
    def tick(self):
        now = time.monotonic()
        if self.ser is None:
            self.try_open(now)
            return
        if not self.p('enable'):
            self.close('enable=false，關閉 GNSS 序列埠')
            return
        if now - self.last_nmea > self.p('no_nmea_timeout_s'):
            self.close('收不到任何有效 NMEA，這個埠可能不是 GNSS，30 秒後再試')
            self.next_try = now + 30.0
        elif now - self.last_gga > self.p('no_nmea_timeout_s'):
            self.get_logger().warn('有 NMEA 但沒有 GGA 句，請確認接收機有輸出 GGA',
                                   throttle_duration_sec=30.0)

    def try_open(self, now):
        if not self.p('enable') or now < self.next_try:
            return
        self.next_try = now + self.p('retry_s')
        port, baud = self.p('port'), self.p('baud')
        if serial is None:
            self.get_logger().error('找不到 pyserial（python3-serial）', throttle_duration_sec=30.0)
            return
        posix = os.name == 'posix'
        if posix and not os.path.exists(port):
            if not self.said_missing:                       # 只講一次，不洗版
                self.get_logger().info(f'找不到 {port}，視為沒接 GNSS，忽略（插上後會自動開始）')
                self.said_missing = True
            return
        self.said_missing = False
        if posix:
            real = os.path.realpath(port)
            for r in self.p('reserved_ports'):
                if os.path.exists(r) and os.path.realpath(r) == real:
                    self.get_logger().error(
                        f'{port} 跟 {r} 是同一個裝置，GNSS 不開這個埠（會搶走 MPU/馬達資料）。'
                        '請用 udev 固定 /dev/mygnss，或用 -p port:= 指到真正的 GNSS', throttle_duration_sec=30.0)
                    return
        try:
            kw = {'exclusive': True} if posix else {}
            self.ser = serial.Serial(port, baud, timeout=0, **kw)
        except Exception as e:
            self.get_logger().warn(f'GNSS 序列埠開不了 ({e})，稍後重試', throttle_duration_sec=10.0)
            self.ser = None
            return
        self.buf.clear()
        self.opened_at = self.last_nmea = self.last_gga = time.monotonic()
        self.get_logger().info(f'GNSS serial opened {port} ({baud})')

    def close(self, why):
        self.get_logger().warn(why)
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.buf.clear()

    # ---------------- 讀取 ----------------
    def read_serial(self):
        if self.ser is None:
            return
        try:
            n = self.ser.in_waiting
            if n:
                self.buf += self.ser.read(n)
        except Exception as e:
            self.close(f'GNSS serial read failed: {e}')
            return
        while b'\n' in self.buf:
            raw, _, rest = self.buf.partition(b'\n')
            self.buf = bytearray(rest)
            self.handle(raw.decode('ascii', errors='ignore').strip())
        if len(self.buf) > 4096:                      # 沒有換行的雜訊，避免無限長大
            self.buf.clear()

    def handle(self, line):
        if not nmea_ok(line):
            return
        self.last_nmea = time.monotonic()
        f = line[1:line.index('*')].split(',')
        if not f[0].endswith('GGA') or len(f) < 9:    # $GNGGA / $GPGGA 都吃
            return
        try:
            quality = int(f[6]) if f[6] else 0
            sats = int(f[7]) if f[7] else 0
            hdop = float(f[8]) if f[8] else float('nan')
            if f[2] and f[4]:
                lat, lon = dm_to_deg(f[2], f[3]), dm_to_deg(f[4], f[5])
            else:
                lat = lon = float('nan')
        except ValueError:
            return
        self.last_gga = time.monotonic()
        self.pub.publish(Float64MultiArray(data=[lat, lon, float(quality), float(sats), hdop]))


def main():
    rclpy.init()
    rclpy.spin(GnssNode())
    rclpy.shutdown()
