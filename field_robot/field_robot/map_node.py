"""2. Map Node: 依「地圖型號」(field_library) 展開航點，逐點送出 Pose2D。

/mission_cmd (String) 兩種格式：
  JSON（建議，地圖資料跟著指令一起到，不會有跨 topic 的先後問題）：
    {"cmd":"start","map":{"preset":"成棒（國際規格）","spacing":27.43,"path":"perimeter_full"}}
  純文字 "start" / "stop"：start 時使用最近一次 /map_select 的內容（相容舊流程）
/mode_cmd 變成 manual 時：任務、路徑、進度全部清掉，不再送任何目標。
/mission_failed：控制節點回報航點逾時 -> 任務中止。

與 fusion_node 的雙向通道（朝向校正握手）：
  map_node -> fusion_node  /zero_heading_req (Bool)  收到 start 且路徑建好後送 True，
                                                     要求把「當下車頭」設為 +X（yaw = 目前 - 收到當下的值）
  fusion_node -> map_node  /heading_zeroed   (Bool)  校正完成回 True（沒有 yaw 資料則回 False）
收到 True 才送第一個目標；沒回應或回 False 會每 0.5 s 重送要求，超過 max_zero_retries 次就中止任務。

送給 control_node 的目標 /goal_cmd (Float32MultiArray) = [x, y, theta_deg, line, drop]：
  line = 1：從上一個點走到這個點的路上要畫線；drop = 1：到達這個點後要丟壘包。
  （同一則訊息帶完整資訊，沒有跨 topic 的先後問題；/goal_pose 仍會同步送出供除錯）
與 control_node：任務開始 / 停止 / 中止時送 /goal_cancel (Bool True)，control 立刻清掉舊目標並停車，
避免上一趟（例如全場）沒走完的目標還留在 control 裡。
"""
import json
import math

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Int32, String

from .field_library import list_presets, resolve_waypoints


class MapNode(Node):
    def __init__(self):
        super().__init__('map_node')
        self.declare_parameter('max_zero_retries', 6)   # 朝向校正要求最多重送次數（間隔 0.5 s）
        self.pub_goal = self.create_publisher(Pose2D, '/goal_pose', 10)
        self.pub_goal_cmd = self.create_publisher(Float32MultiArray, '/goal_cmd', 10)   # map -> control（含畫線/丟壘包旗標）
        self.pub_done = self.create_publisher(Bool, '/mission_done', 10)
        self.pub_zero_req = self.create_publisher(Bool, '/zero_heading_req', 10)   # map -> fusion
        self.pub_progress = self.create_publisher(Int32, '/mission_progress', 10)   # 目前目標的索引（0 起算），給網頁畫進度
        self.pub_cancel = self.create_publisher(Bool, '/goal_cancel', 10)          # map -> control
        self.create_subscription(String, '/map_select', self.on_map, 10)
        self.create_subscription(String, '/mission_cmd', self.on_cmd, 10)
        self.create_subscription(Bool, '/reached', self.on_reached, 10)
        self.create_subscription(Bool, '/mission_failed', self.on_failed, 10)
        self.create_subscription(String, '/mode_cmd', self.on_mode, 10)
        self.create_subscription(Bool, '/heading_zeroed', self.on_zeroed, 10)     # fusion -> map
        self.waiting_zero = False
        self.zero_tries = 0
        self.zero_timer = self.create_timer(0.5, self.on_zero_timeout)   # 等不到 fusion 時重送要求
        self.zero_timer.cancel()
        self.map_req = {}
        self.path = []
        self.idx = 0
        self.running = False

    def cancel_goal(self):
        self.pub_cancel.publish(Bool(data=True))

    def reset_mission(self):
        self.cancel_goal()
        self.running = False
        self.waiting_zero = False
        self.zero_timer.cancel()
        self.path = []
        self.idx = 0

    def on_map(self, msg):
        try:
            self.map_req = json.loads(msg.data)
        except json.JSONDecodeError:
            self.map_req = {'preset': msg.data}

    def on_mode(self, msg):
        if msg.data == 'manual' and (self.running or self.path):
            self.reset_mission()
            self.get_logger().info('mode -> manual: mission cleared')

    def on_failed(self, msg):
        if msg.data and self.running:
            self.reset_mission()
            self.get_logger().error('mission aborted by control_node')

    def on_cmd(self, msg):
        cmd, req = msg.data, None
        if msg.data.lstrip().startswith('{'):
            try:
                d = json.loads(msg.data)
            except json.JSONDecodeError:
                self.get_logger().error(f'bad mission_cmd JSON: {msg.data!r}')
                return
            cmd, req = d.get('cmd'), d.get('map')

        if cmd == 'start':
            req = req if req is not None else self.map_req
            try:
                self.path = self.build_path(req)
            except Exception as e:
                self.get_logger().error(f'bad map model {req}: {e}')
                return
            self.map_req = req
            self.idx, self.running = 0, True
            self.cancel_goal()          # 先清掉 control 裡上一趟遺留的目標
            # 先請 fusion 把朝向校正為 +X，等它回 /heading_zeroed=True 才送第一個目標
            self.waiting_zero = True
            self.zero_tries = 0
            self._request_zero()
            self.zero_timer.reset()
        elif cmd == 'stop':
            self.cancel_goal()
            self.running = False
            self.waiting_zero = False
            self.zero_timer.cancel()

    def build_path(self, req):
        spacing = req.get('spacing')
        if spacing is None:
            spacing = list_presets()[req['preset']]['base_spacing_m']
        spacing = float(spacing)
        if not 2.0 <= spacing <= 50.0:
            raise ValueError(f'spacing {spacing} out of range')
        wps = resolve_waypoints(req.get('path', 'perimeter_full'), spacing)
        out = []
        for i, w in enumerate(wps):
            if i + 1 < len(wps):
                th = math.degrees(math.atan2(wps[i + 1]['y'] - w['y'], wps[i + 1]['x'] - w['x']))   # 度
            else:
                th = out[-1][2] if out else 0.0
            out.append((w['x'], w['y'], th, w['line'], w['drop']))
        return out

    def _request_zero(self):
        self.zero_tries += 1
        self.pub_zero_req.publish(Bool(data=True))
        self.get_logger().info(f'請 fusion 校正朝向 (第 {self.zero_tries} 次)')

    def on_zeroed(self, msg):
        if not (self.waiting_zero and self.running):
            return
        if msg.data:
            self.get_logger().info('朝向校正完成')
            self._release_start()
        else:
            self.get_logger().warn('fusion 回報校正失敗（尚未收到 yaw），稍後重試')

    def on_zero_timeout(self):
        if not (self.waiting_zero and self.running):
            self.zero_timer.cancel()
            return
        if self.zero_tries >= self.get_parameter('max_zero_retries').value:
            self.get_logger().error('朝向校正一直沒完成，任務中止')
            self.reset_mission()
            return
        self._request_zero()

    def _release_start(self):
        self.waiting_zero = False
        self.zero_timer.cancel()
        self._send_goal()

    def on_reached(self, msg):
        if not (msg.data and self.running) or self.waiting_zero:
            return
        self.idx += 1
        if self.idx >= len(self.path):
            self.running = False
            self.pub_done.publish(Bool(data=True))
            self.get_logger().info('mission done')
        else:
            self._send_goal()

    def _send_goal(self):
        x, y, th, line, drop = self.path[self.idx]
        self.pub_goal.publish(Pose2D(x=x, y=y, theta=th))
        self.pub_progress.publish(Int32(data=self.idx))
        self.pub_goal_cmd.publish(Float32MultiArray(data=[x, y, th, 1.0 if line else 0.0, 1.0 if drop else 0.0]))
        self.get_logger().info(f'goal {self.idx + 1}/{len(self.path)}: ({x:.2f},{y:.2f}) line={int(line)} drop={int(drop)}')


def main():
    rclpy.init()
    rclpy.spin(MapNode())
    rclpy.shutdown()
