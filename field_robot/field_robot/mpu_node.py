"""MPU Node -> /mpu_yaw (度, 逆時針為正, 範圍 -180~180)、/mpu_rate (度/秒)。
讀取邏輯直接使用 COM6 測試腳本（找 0xAA -> 0x55 -> 讀 9 bytes -> '<ff'），
只把 print 換成發佈 topic。
ESP32 封包 (11 bytes，小端序，Float32)：AA 55 | seq u8 | yaw_deg float32 | rate_dps float32

反向（樹莓派 -> ESP32，控制 ESP32 GPIO17 輸出）：訂閱 /esp_gpio_cmd (Float32MultiArray=[啟用, 工作比%, 頻率Hz])，
轉成 7 bytes 封包寫回同一個序列埠：A5 5A | enable u8 | duty u8 (0~100) | freq u16 小端 | checksum u8
checksum = enable ^ duty ^ freq_low ^ freq_high。ESP32 超過 500 ms 沒收到指令會自動關閉輸出。

丟壘包：訂閱 /drop_cmd (Bool True)，轉成 6 bytes 封包寫給 ESP32（GPIO 輸出一個脈衝）：
  A5 5B | seq u8 | dur u8 (單位 10 ms，脈衝寬度) | checksum u8，checksum = seq ^ dur
  ESP32 以 seq 去重：同一個 seq 只丟一次。指令每 0.1 s 重送，直到 ESP32 回報「已開始」（最多 2 s）。
  脈衝寬度由參數 drop_ms 設定（10~2550 ms，預設 1000）。

ESP32 -> 樹莓派 丟壘包回報（4 bytes，約 10 Hz）：AA 56 | seq u8 | state u8 | checksum u8 (= seq ^ state)
  state：0 = 開機後還沒丟過、1 = 丟壘包中、2 = 已完成。
  收到「seq 等於剛送出的 seq 且 state == 2」時發佈 /drop_done (Bool True)，control_node 才會繼續下一個點。
"""
import math
import struct
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray


class MPUNode(Node):
    def __init__(self):
        super().__init__('mpu_node')
        self.declare_parameter('port', '/dev/mympu')   # 請依實際連接埠修改
        self.declare_parameter('baud', 115200)
        self.declare_parameter('drop_ms', 1000)        # 丟壘包輸出脈衝寬度 (ms)，10~2550
        self.declare_parameter('yaw_sign', 1.0)        # 逆時針(從上往下看)必須是 yaw 增加；若相反設 -1.0（不用重燒 ESP32）
        self.ser = None
        self._wlock = threading.Lock()      # 序列埠開關/寫入鎖（讀取在另一條執行緒）
        self._open()                        # 開不了不會讓節點掛掉，reader 執行緒會每秒重試

        self.pub_yaw = self.create_publisher(Float32, '/mpu_yaw', 10)
        self.pub_rate = self.create_publisher(Float32, '/mpu_rate', 10)

        self.create_subscription(Float32MultiArray, '/esp_gpio_cmd', self.on_esp_cmd, 10)   # control -> ESP32

        self._drop_seq = int(time.time()) & 0xFF   # 起始值用時間，避免重啟後 seq 剛好等於 ESP32 記住的上一次
        self._drop_pending = False                  # 已送出、還在等 ESP32 回報完成
        self._drop_started = False                  # ESP32 已回報收到（state 1 或 2）-> 不再重送
        self._drop_tries = 0
        self.pub_drop_done = self.create_publisher(Bool, '/drop_done', 10)
        self.create_subscription(Bool, '/drop_cmd', self.on_drop, 10)    # control -> ESP32
        self.create_timer(0.1, self._drop_tick)

        self._last_frame = time.monotonic()
        self.create_timer(1.0, self._watch)     # 沒資料時明確報錯，不要靜靜地什麼都沒有
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _open(self):
        port, baud = self.get_parameter('port').value, self.get_parameter('baud').value
        try:
            ser = serial.Serial(port, baud, timeout=1.0)    # 跟原本、跟 COM6 測試腳本完全相同的開法
        except Exception as e:
            self.get_logger().warn(f'MPU 序列埠開不了 ({e})，1 秒後重試', throttle_duration_sec=5.0)
            return False
        with self._wlock:
            self.ser = ser
        self.get_logger().info(f'Listening on {port} at {baud} baud (Float32 mode)...')
        return True

    def _watch(self):
        if self.ser is not None and time.monotonic() - self._last_frame > 2.0:
            self.get_logger().warn(
                f"已開啟 {self.get_parameter('port').value} 但 2 秒以上沒收到有效 MPU 封包："
                '請確認埠是不是 ESP32、baud、ESP32 有沒有在跑', throttle_duration_sec=5.0)

    def _close(self):
        with self._wlock:
            ser, self.ser = self.ser, None
        try:
            if ser is not None:
                ser.close()
        except Exception:
            pass

    def on_esp_cmd(self, msg):
        if len(msg.data) < 3:
            return
        en = 1 if msg.data[0] >= 0.5 else 0
        duty = int(round(max(0.0, min(100.0, msg.data[1]))))
        freq = int(round(max(100.0, min(20000.0, msg.data[2]))))
        self._send_esp(en, duty, freq)

    def on_drop(self, msg):
        if not msg.data:
            return
        self._drop_seq = (self._drop_seq + 1) & 0xFF
        self._drop_pending, self._drop_started, self._drop_tries = True, False, 0
        self.get_logger().info(f'丟壘包 seq={self._drop_seq}')
        self._drop_tick()

    def _drop_tick(self):
        if not self._drop_pending or self._drop_started or self._drop_tries >= 20:
            return                                  # 已確認收到就不再重送；重送滿 2 s 放棄（control 會逾時中止任務）
        self._drop_tries += 1
        dur = int(round(max(10, min(2550, float(self.get_parameter('drop_ms').value))) / 10.0))
        seq = self._drop_seq
        try:
            with self._wlock:
                if self.ser is None:
                    self.get_logger().warn('丟壘包：ESP32 序列埠未連線，指令未送出', throttle_duration_sec=2.0)
                    return
                self.ser.write(b'\xa5\x5b' + bytes([seq, dur, seq ^ dur]))
        except Exception as e:
            self.get_logger().warn(f'寫入 ESP32 失敗: {e}', throttle_duration_sec=5.0)

    def _on_drop_status(self, seq, state):
        if not self._drop_pending or seq != self._drop_seq:
            return
        if state >= 1:
            self._drop_started = True
        if state == 2:
            self._drop_pending = False
            self.get_logger().info(f'ESP32 回報丟壘包完成 seq={seq}')
            self.pub_drop_done.publish(Bool(data=True))

    def _send_esp(self, en, duty, freq):
        body = bytes([en & 0xFF, duty & 0xFF]) + struct.pack('<H', freq)
        chk = body[0] ^ body[1] ^ body[2] ^ body[3]
        try:
            with self._wlock:
                if self.ser is None:
                    return
                self.ser.write(b'\xa5\x5a' + body + bytes([chk]))
        except Exception as e:
            self.get_logger().warn(f'寫入 ESP32 失敗: {e}', throttle_duration_sec=5.0)

    def _reader(self):
        while not self._stop.is_set():
            ser = self.ser
            if ser is None:                         # 沒開成功 / 斷線後：每秒重開
                time.sleep(1.0)
                if not self._stop.is_set():
                    self._open()
                continue
            try:
                b = ser.read(1)                             # 尋找同步標頭 AA 55
                while b == b"\xaa":
                    b2 = ser.read(1)
                    if b2 == b"\x56":                       # 丟壘包回報：seq | state | chk
                        st = ser.read(3)
                        if len(st) == 3 and st[2] == (st[0] ^ st[1]):
                            self._on_drop_status(st[0], st[1])
                        break
                    if b2 != b"\x55":
                        b = b2                              # 連續 AA 時，後一個 AA 當標頭，不漏封包
                        continue
                    rest = ser.read(9)                      # seq(1) + yaw(4) + rate(4)
                    if len(rest) != 9:
                        self.get_logger().warn("警告: 讀取資料逾時或長度不足 (期待 9 bytes)")
                        break
                    seq = rest[0]
                    yaw_deg, rate_dps = struct.unpack("<ff", rest[1:])   # 小端序兩個 float32
                    if not (math.isfinite(yaw_deg) and math.isfinite(rate_dps)):
                        break                               # 壞資料（失去同步）直接丟掉
                    self._last_frame = time.monotonic()
                    sg = float(self.get_parameter('yaw_sign').value)
                    self.get_logger().info(
                        f"Seq: {seq:3d} | Yaw: {sg * yaw_deg:8.3f} deg | Rate: {sg * rate_dps:6.2f} dps",
                        throttle_duration_sec=1.0)
                    self.pub_yaw.publish(Float32(data=sg * yaw_deg))
                    self.pub_rate.publish(Float32(data=sg * rate_dps))
                    break
            except Exception as e:
                if self._stop.is_set():
                    break
                self.get_logger().warn(f"讀取發生錯誤: {e}，關閉序列埠並重連", throttle_duration_sec=5.0)
                self._close()                               # USB 掉線/重新枚舉：關掉，上面迴圈會重開


def main():
    rclpy.init()
    node = MPUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send_esp(0, 0, 1000)         # 結束前先關閉 ESP32 輸出
        node._stop.set()
        node._thread.join(timeout=1.5)
        node._close()
        node.destroy_node()
        rclpy.shutdown()
