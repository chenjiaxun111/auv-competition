#!/usr/bin/env python3
"""Read-only mission/heading terminal view; no RPC, publisher or arm service."""
import argparse
import json
from datetime import datetime
import math
from pathlib import Path
import time


class StatusView:
    def __init__(self):
        self.state = '等待状态'
        self.state_at = -math.inf
        self.diagnostic = {}
        self.diagnostic_at = -math.inf
        self.targets = {}
        self.progress = {}
        self.progress_at = -math.inf

    def progress_received(self, text, now):
        try:
            payload = json.loads(text)
            if not isinstance(payload, dict) or payload.get('schema') != 'auv.mission_progress.v1':
                raise ValueError('invalid progress schema')
            self.progress = payload
        except (ValueError, TypeError):
            self.progress = {}
        self.progress_at = now

    def startup_line(self, now):
        if now - self.state_at > 2 or self.state not in {'START_DELAY', 'DEPARTURE_FORWARD'}:
            return ''
        label = '出发等待' if self.state == 'START_DELAY' else '直行中'
        progress = self.progress
        if now - self.progress_at > 0.5 or progress.get('state') != self.state:
            return f' | {label}：倒计时未收到/过期'
        remaining, duration = progress.get('remaining_s'), progress.get('duration_s')
        if (self.vector([remaining, duration], 2) is None
                or remaining < 0 or duration < 0 or remaining > duration):
            return f' | {label}：倒计时无效'
        # Do not start a local timer: reconnect/rearm resets come from controller.
        text = f' | {label}：剩余 {math.ceil(remaining)} 秒（设定 {duration:g} 秒）'
        if self.state == 'START_DELAY':
            return text + '，零指令等待'
        surge = progress.get('departure_surge')
        if self.vector([surge], 1) is not None:
            text += f'，前进指令 surge={surge:g}'
        return text + '（非实测速度）'

    def observations_received(self, kind, text, now):
        # A new empty/malformed frame clears old detections immediately.
        try:
            payload = json.loads(text)
            expected = f'auv.{"ball" if kind == "ball" else "gate_pose"}_observations.v1'
            if not isinstance(payload, dict) or payload.get('schema') != expected:
                raise ValueError('invalid observation schema')
            observations = payload.get('observations')
            if not isinstance(observations, list):
                raise ValueError('observations must be a list')
            frame = str(payload.get('frame_id', '未知相机坐标系')).replace('\n', ' ')[:100]
        except (ValueError, TypeError):
            observations, frame = [], '数据格式无效'
        self.targets[kind] = (now, frame, observations)

    @staticmethod
    def vector(value, length):
        if not isinstance(value, (list, tuple)) or len(value) != length:
            return None
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) for v in value):
            return None
        return tuple(float(v) for v in value)

    def target_line(self, now):
        if now - self.state_at > 2:
            return '状态过期，隐藏目标'
        if self.state in {'SEARCH_BALL', 'ALIGN_BALL', 'APPROACH_BALL'}:
            kind = 'ball'
        elif self.state in {'SEARCH_GATE', 'ALIGN_GATE', 'APPROACH_GATE'}:
            kind = 'gate'
        else:
            return '当前阶段不进行视觉伺服'
        received, frame, observations = self.targets.get(kind, (-math.inf, '', []))
        if now - received > 0.35:
            return '未收到/已过期（隐藏旧坐标）'
        candidates = []
        for item in observations:
            if not isinstance(item, dict):
                continue
            score = item.get('confidence')
            if (isinstance(score, bool) or not isinstance(score, (int, float))
                    or not math.isfinite(score) or not 0 <= score <= 1):
                continue
            if kind == 'ball':
                position = self.vector(item.get('position_m'), 3)
                if position is not None and position[2] > 0:
                    candidates.append((score, -position[2], item))
            elif item.get('geometry_valid') is True:
                center = self.vector(item.get('center_error_norm'), 2)
                size = self.vector(item.get('size_norm'), 2)
                if center is not None and size is not None and min(size) > 0:
                    candidates.append((score, size[0] * size[1], item))
        if not candidates:
            return '未检测到有效目标（球需有效深度，门需有效四角）'
        item = max(candidates, key=lambda entry: entry[:2])[2]
        position = self.vector(item.get('position_m' if kind == 'ball' else 'translation_m'), 3)
        age = now - received
        if position is not None and position[2] > 0:
            x, y, z = position
            source = 'ZED双目表面深度' if kind == 'ball' else '门框PnP估计'
            name = str(item.get('class_name', 'gate')).replace('\n', ' ')[:40]
            return (f'{name} [{source}] 相机坐标x={x:+.3f} y={y:+.3f} z={z:.3f}m'
                    f' 前向距离={z:.3f}m 直线距离={math.hypot(x,y,z):.3f}m'
                    f' frame={frame} 接收age={age:.2f}s')
        center = self.vector(item['center_error_norm'], 2)
        return (f'gate [Pose二维] 中心误差=({center[0]:+.3f},{center[1]:+.3f})'
                '（归一化） 三维坐标/距离不可用（PnP未启用或无有效解）'
                f' 接收age={age:.2f}s')

    def line(self, now, stage):
        state = self.state if now - self.state_at <= 2 else '状态未收到/过期'
        values = self.diagnostic
        fresh = now - self.diagnostic_at <= 2
        connected = fresh and values.get('connected') == 'true'
        try:
            age = float(values.get('telemetry_age_s', 'inf'))
            yaw = float(values.get('yaw_deg', 'nan'))
        except (TypeError, ValueError):
            age, yaw = math.inf, math.nan
        if not connected or not math.isfinite(age) or age > 0.5 or age < 0:
            heading = '无数据/过期'
        elif values.get('yaw_valid') != 'true':
            heading = '无效（传感器未就绪）'
        elif not math.isfinite(yaw):
            heading = '无效数值'
        else:
            heading = f'{yaw:+.2f}° 有效 age={age:.2f}s'
        permit = ('已解锁' if values.get('motion_enabled') == 'true' else '未解锁')
        if not connected:
            permit = '未知'
        return (f'阶段={stage} | 状态={state} | NanoPi={"在线" if connected else "离线/过期"}'
                f' | 运动={permit} | 陀螺仪[NanoPi航向yaw]={heading}'
                f'{self.startup_line(now)}'
                f'\n  感知目标={self.target_line(now)}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--supervisor-pid', type=int, required=True)
    parser.add_argument('--state-file', required=True)
    args = parser.parse_args()
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import String

    rclpy.init(args=[])
    node = Node('competition_terminal_monitor')
    view = StatusView()
    # Same reliable/volatile contract as controller state and bridge diagnostics.
    qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE)

    def state_received(msg):
        view.state = msg.data.replace('\n', ' ')[:80]
        view.state_at = time.monotonic()

    def diagnostic_received(msg):
        for status in msg.status:
            if status.name == '/auv/rov_bridge':
                view.diagnostic = {item.key: item.value for item in status.values}
                view.diagnostic_at = time.monotonic()

    subscriptions = [
        node.create_subscription(String, '/auv/mission_state', state_received, qos),
        node.create_subscription(String, '/auv/mission_progress',
                                 lambda msg: view.progress_received(
                                     msg.data, time.monotonic()), qos),
        node.create_subscription(DiagnosticArray, '/auv/bridge_diagnostics',
                                 diagnostic_received, qos),
        node.create_subscription(String, '/auv/perception/ball_observations',
                                 lambda msg: view.observations_received(
                                     'ball', msg.data, time.monotonic()), qos),
        node.create_subscription(String, '/auv/perception/gate_pose_observations',
                                 lambda msg: view.observations_received(
                                     'gate', msg.data, time.monotonic()), qos),
    ]

    def display():
        try:
            stage = Path(args.state_file).read_text().strip()
        except OSError:
            stage = '未知'
        print(f'[{datetime.now().astimezone().isoformat(timespec="seconds")}] '
              + view.line(time.monotonic(), stage), flush=True)

    timer = node.create_timer(1.0, display)
    print('实时状态：每秒刷新；yaw 是航向角，不是角速度。'
          'Ctrl+C 仅关闭显示，停车请另执行 auv_competition.sh stop。', flush=True)
    try:
        while rclpy.ok():
            try:
                command = Path(f'/proc/{args.supervisor_pid}/cmdline').read_bytes()
            except OSError:
                break
            if b'auv_competition.sh' not in command or b'__run' not in command:
                break
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException, BrokenPipeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    # Keep timer/subscription references alive for the duration of spin.
    del timer, subscriptions


if __name__ == '__main__':
    main()
