#!/usr/bin/env python3
"""Replay detector/Pose/controller against a recorded competition video.

NanoPi output is disabled by default. It is enabled only when both
``--enable-nanopi`` and the explicit physical-safety acknowledgement
``--allow-pwm`` are supplied.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


class RosCompetitionOutput:
    """Publish planner output through the real competition ROS interface."""

    def __init__(self) -> None:
        import rclpy
        from geometry_msgs.msg import Wrench
        from std_msgs.msg import Bool, String

        self.rclpy = rclpy
        self.Wrench = Wrench
        self.Bool = Bool
        rclpy.init()
        self.node = rclpy.create_node('video_competition_replay')
        self.command_pub = self.node.create_publisher(
            Wrench, '/auv/control_command', 1)
        self.enable_pub = self.node.create_publisher(
            Bool, '/auv/motion_enable', 1)
        self.connected = False
        self.last_power = None
        self.node.create_subscription(
            Bool, '/auv/connected', self._connected, 5)
        self.node.create_subscription(
            String, '/auv/propeller_power', self._power, 5)

    def _connected(self, message) -> None:
        self.connected = bool(message.data)

    def _power(self, message) -> None:
        try:
            self.last_power = json.loads(message.data)
        except json.JSONDecodeError:
            self.last_power = None

    def wait_ready(self, timeout_s: float = 10.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
            if (self.connected and self.command_pub.get_subscription_count() == 1
                    and self.enable_pub.get_subscription_count() == 1):
                return
        raise RuntimeError('competition bridge is not uniquely connected')

    def publish(self, output) -> None:
        command = self.Wrench()
        command.force.x = float(output.surge)
        command.force.y = float(output.sway)
        command.force.z = float(output.heave)
        command.torque.z = float(output.yaw)
        enabled = self.Bool()
        enabled.data = bool(output.motion_enable)
        self.command_pub.publish(command)
        self.enable_pub.publish(enabled)
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def stop(self) -> None:
        for _ in range(8):
            command = self.Wrench()
            enabled = self.Bool()
            enabled.data = False
            self.command_pub.publish(command)
            self.enable_pub.publish(enabled)
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


HERE = Path(__file__).resolve().parent
PACKAGE_ROOT = (
    HERE.parent / 'AUV-Connection-Available' / 'AUV_ws' / 'src'
    / 'auv_competition_control'
)
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from auv_competition_control.controller_core import (  # noqa: E402
    BallObservation,
    CompetitionPlanner,
    GateKeypoint,
    GatePoseObservation,
    MissionState,
    PlannerConfig,
)
from auv_competition_control.pose_geometry import gate_image_geometry  # noqa: E402
from test_dual_model_video_realtime import (  # noqa: E402
    DEFAULT_DETECTOR,
    DEFAULT_POSE,
    DEFAULT_VIDEO,
    draw_detector,
    draw_gate_pose,
)


class SharedState:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.jpeg: bytes | None = None
        self.sequence = 0
        self.client_connected = threading.Event()
        self.stop_event = threading.Event()
        self.finished = False
        self.error = ''
        self.stats = {
            'frame': 0,
            'total_frames': 0,
            'fps': 0.0,
            'state': 'STARTING',
            'surge': 0.0,
            'sway': 0.0,
            'heave': 0.0,
            'yaw': 0.0,
            'target_depth_m': None,
            'planned_motion_enable': False,
            'nanopi_transmission': False,
            'target_kind': 'none',
            'target_detail': '等待模型启动',
            'state_changes': 0,
            'output_video': '',
            'output_csv': '',
        }


STATE = SharedState()


def page_html() -> bytes:
    return """<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<title>AUV 离线闭环调试</title><style>
body{margin:0;background:#101216;color:#eee;font-family:Arial,"Microsoft YaHei",sans-serif}
.wrap{max-width:1500px;margin:auto;padding:12px}.warning{background:#7a1717;padding:10px;
font-weight:bold;text-align:center;border-radius:6px}.grid{display:grid;grid-template-columns:minmax(0,3fr) minmax(310px,1fr);gap:12px;margin-top:12px}
img{width:100%;background:#000;border:1px solid #47505d}.panel{background:#1a1f27;padding:14px;border-radius:7px}
.state{font-size:26px;color:#6ee7ff;font-weight:bold;margin:8px 0 16px}.row{display:flex;justify-content:space-between;border-bottom:1px solid #303844;padding:8px 0}
.value{font-family:monospace;font-size:17px}.off{color:#ff7777;font-weight:bold}.target{color:#f5df7b;line-height:1.5;margin-top:12px}
button{width:100%;margin-top:16px;padding:10px;font-size:16px;background:#a52626;color:white;border:0;border-radius:5px}
@media(max-width:900px){.grid{grid-template-columns:1fr}}
</style></head><body><div class="wrap">
<div id="warning" class="warning">正在读取输出链状态</div>
<div class="grid"><div><img src="/stream" alt="等待视频流"></div><div class="panel">
<div>比赛状态机</div><div id="state" class="state">STARTING</div>
<div class="row"><span>前后 surge</span><span id="surge" class="value">0.00</span></div>
<div class="row"><span>横移 sway</span><span id="sway" class="value">0.00</span></div>
<div class="row"><span>升沉 heave</span><span id="heave" class="value">0.00</span></div>
<div class="row"><span>转向 yaw</span><span id="yaw" class="value">0.00</span></div>
<div class="row"><span>目标深度</span><span id="depth" class="value">0.00 m</span></div>
<div class="row"><span>控制器运动许可</span><span id="enable" class="value">否</span></div>
<div class="row"><span>NanoPi 下发</span><span id="nanopi" class="value off">读取中</span></div>
<div id="target" class="target">等待目标</div><div id="progress" class="target"></div>
<button onclick="fetch('/stop',{method:'POST'});this.disabled=true">停止离线测试</button>
</div></div></div><script>
function n(v){return Number(v).toFixed(2)}
setInterval(async()=>{try{const s=await (await fetch('/stats')).json();
document.getElementById('state').textContent=s.state;
document.getElementById('surge').textContent=n(s.surge);
document.getElementById('sway').textContent=n(s.sway);
document.getElementById('heave').textContent=n(s.heave);
document.getElementById('yaw').textContent=n(s.yaw);
document.getElementById('depth').textContent=(s.target_depth_m===null?'视觉模式：禁用':n(s.target_depth_m)+' m');
document.getElementById('enable').textContent=s.planned_motion_enable?'是（仅计算）':'否';
const tx=Boolean(s.nanopi_transmission);
document.getElementById('nanopi').textContent=tx?'已启用':'已关闭';
document.getElementById('warning').textContent=tx
  ?'实链回放：控制指令正在发送到 NanoPi，请确认推进器动力已物理断开'
  :'离线仿真：NanoPi 发送路径未加载，推进器不会收到指令';
document.getElementById('target').textContent=s.target_kind+'：'+s.target_detail;
document.getElementById('progress').textContent=`帧 ${s.frame}/${s.total_frames} · ${n(s.fps)} FPS · 状态切换 ${s.state_changes} 次`;
}catch(e){}},300)</script></body></html>""".encode('utf-8')


class Handler(BaseHTTPRequestHandler):
    def log_message(self, _format: str, *_args) -> None:
        pass

    def _send(self, content_type: str, body: bytes) -> None:
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path == '/':
            self._send('text/html; charset=utf-8', page_html())
            return
        if self.path == '/stats':
            payload = dict(STATE.stats, finished=STATE.finished, error=STATE.error)
            self._send('application/json; charset=utf-8', json.dumps(
                payload, ensure_ascii=False).encode('utf-8'))
            return
        if self.path != '/stream':
            self.send_error(404)
            return
        STATE.client_connected.set()
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        last_sequence = -1
        try:
            while not STATE.stop_event.is_set():
                with STATE.condition:
                    STATE.condition.wait_for(
                        lambda: STATE.sequence != last_sequence or STATE.finished,
                        timeout=2.0)
                    jpeg = STATE.jpeg
                    last_sequence = STATE.sequence
                    finished = STATE.finished
                if jpeg:
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
                if finished:
                    break
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self) -> None:
        if self.path != '/stop':
            self.send_error(404)
            return
        STATE.stop_event.set()
        self._send('text/plain; charset=utf-8', '正在停止'.encode('utf-8'))


def ball_observations(result, now_s: float, width: int, height: int,
                      target_class: str, ball_diameter_m: float,
                      horizontal_fov_deg: float) -> list[BallObservation]:
    """Approximate metric ball positions for MP4-only controller replay.

    Live competition code does not use this approximation; it uses ZED
    registered depth.  This is only enough to exercise controller transitions.
    """
    observations = []
    if result.boxes is None:
        return observations
    fx = width / (2.0 * math.tan(math.radians(horizontal_fov_deg) / 2.0))
    fy = fx
    for box, confidence, class_id in zip(
            result.boxes.xyxy.cpu().numpy(),
            result.boxes.conf.cpu().numpy(),
            result.boxes.cls.cpu().numpy().astype(int)):
        name = str(result.names[int(class_id)])
        if name != target_class:
            continue
        x1, y1, x2, y2 = (float(value) for value in box)
        diameter_px = max(1.0, 0.5 * ((x2 - x1) + (y2 - y1)))
        z = fx * ball_diameter_m / diameter_px
        centre_x = 0.5 * (x1 + x2)
        centre_y = 0.5 * (y1 + y2)
        observations.append(BallObservation(
            class_name=name,
            confidence=float(confidence),
            x=(centre_x - width / 2.0) * z / fx,
            y=(centre_y - height / 2.0) * z / fy,
            z=z,
            received_s=now_s,
        ))
    return observations


def gate_observations(result, now_s: float, width: int, height: int,
                      horizontal_fov_deg: float,
                      keypoint_threshold: float) -> list[GatePoseObservation]:
    observations = []
    if result.boxes is None or result.keypoints is None:
        return observations
    fx = width / (2.0 * math.tan(math.radians(horizontal_fov_deg) / 2.0))
    cx = width / 2.0
    points_all = result.keypoints.xy.cpu().numpy()
    confidences = result.keypoints.conf
    scores_all = (confidences.cpu().numpy() if confidences is not None else
                  np.ones((len(points_all), 4), dtype=np.float32))
    for box_conf, points, scores in zip(
            result.boxes.conf.cpu().numpy(), points_all, scores_all):
        if len(points) != 4 or len(scores) != 4:
            continue
        geometry = gate_image_geometry(points, width, height, fx, cx)
        valid = geometry.valid and float(np.min(scores)) >= keypoint_threshold
        keypoints = tuple(GateKeypoint(
            x=float(point[0] / width), y=float(point[1] / height),
            confidence=float(score)) for point, score in zip(points, scores))
        observations.append(GatePoseObservation(
            confidence=float(box_conf),
            keypoints=keypoints,
            center_x_norm=geometry.center_x_norm,
            center_y_norm=geometry.center_y_norm,
            width_norm=geometry.width_norm,
            height_norm=geometry.height_norm,
            bearing_x_rad=geometry.bearing_x_rad,
            received_s=now_s,
            geometry_valid=valid,
        ))
    return observations


def target_text(output) -> tuple[str, str]:
    target = output.selected_target
    if isinstance(target, BallObservation):
        return ('球（MP4 视觉估距）',
                f'x={target.x:+.2f}m y={target.y:+.2f}m z≈{target.z:.2f}m')
    if isinstance(target, GatePoseObservation):
        return ('门框 Pose',
                f'中心=({target.center_x_norm:+.2f},{target.center_y_norm:+.2f}) '
                f'宽度={target.width_norm:.2f} 方位={target.bearing_x_rad:+.2f}rad')
    return '无目标', '状态机正在搜索或执行定时动作'


def draw_control_panel(frame: np.ndarray, output, phase: str,
                       nanopi_enabled: bool) -> None:
    height, width = frame.shape[:2]
    panel_height = min(175, max(130, height // 4))
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (width, panel_height), (12, 16, 22), -1)
    cv2.addWeighted(overlay, 0.86, frame, 0.14, 0.0, frame)
    link_text = ('RECORDED VIDEO - NANOPI PWM ENABLED' if nanopi_enabled
                 else 'OFFLINE CLOSED LOOP - NANOPI TX DISABLED')
    link_color = ((80, 220, 255) if nanopi_enabled else (80, 80, 255))
    cv2.putText(frame, link_text, (16, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.70, link_color, 2, cv2.LINE_AA)
    cv2.putText(frame, f'STATE: {output.state.value}   phase={phase}', (16, 64),
                cv2.FONT_HERSHEY_SIMPLEX, 0.76, (0, 240, 255), 2, cv2.LINE_AA)
    depth_text = ('disabled' if output.target_depth_m is None
                  else f'{output.target_depth_m:.2f}m')
    cv2.putText(frame,
                f'PLAN surge={output.surge:+6.2f} sway={output.sway:+6.2f} '
                f'heave={output.heave:+6.2f} yaw={output.yaw:+6.2f} '
                f'depth={depth_text}',
                (16, 99), cv2.FONT_HERSHEY_SIMPLEX, 0.64,
                (245, 245, 245), 2, cv2.LINE_AA)
    kind, detail = target_text(output)
    cv2.putText(frame, f'{kind}: {detail}', (16, 134),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (110, 230, 180), 2, cv2.LINE_AA)


def create_planner(args: argparse.Namespace, now_s: float) -> CompetitionPlanner:
    config = PlannerConfig(
        target_ball_color=args.target_ball_color,
        require_bridge_connection=False,
        vertical_control_mode='visual_heave',
        initial_depth_m=args.simulated_depth_m,
        detection_confirmations=args.detection_confirmations,
    )
    planner = CompetitionPlanner(config)
    planner.set_depth(args.simulated_depth_m)
    planner.set_armed(True, now_s)
    if args.start_phase == 'gate':
        # Test-only phase injection. The production mission still reaches this
        # state through HIT_BALL -> RETREAT -> SEARCH_GATE.
        planner._enter(MissionState.SEARCH_GATE, now_s)
    return planner


def inference_worker(args: argparse.Namespace) -> None:
    capture = writer = csv_file = ros_output = None
    try:
        for path in (args.video, args.detector, args.pose):
            if not path.is_file():
                raise FileNotFoundError(path)
        detector = YOLO(str(args.detector))
        pose_model = YOLO(str(args.pose))
        capture = cv2.VideoCapture(str(args.video))
        if not capture.isOpened():
            raise RuntimeError(f'无法打开视频：{args.video}')
        source_fps = float(capture.get(cv2.CAP_PROP_FPS) or 30.0)
        total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        start_frame = max(0, int(args.start_seconds * source_fps))
        capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
        args.output_video.parent.mkdir(parents=True, exist_ok=True)
        args.output_csv.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(args.output_video),
                                 cv2.VideoWriter_fourcc(*'mp4v'),
                                 source_fps, (width, height))
        if not writer.isOpened():
            raise RuntimeError(f'无法创建输出视频：{args.output_video}')
        csv_file = args.output_csv.open('w', newline='', encoding='utf-8')
        log = csv.writer(csv_file)
        log.writerow(['frame', 'video_s', 'state', 'surge', 'sway', 'heave', 'yaw',
                      'target_depth_m', 'planned_motion_enable',
                      'nanopi_transmission', 'target_kind', 'target_detail'])
        STATE.stats.update(total_frames=total,
                           output_video=str(args.output_video),
                           output_csv=str(args.output_csv))
        if not args.headless:
            while not STATE.client_connected.wait(0.2):
                if STATE.stop_event.is_set():
                    return

        if args.enable_nanopi:
            ros_output = RosCompetitionOutput()
            ros_output.wait_ready()

        planner = create_planner(args, start_frame / source_fps)
        processed = 0
        state_changes = 0
        previous_state = planner.state
        wall_started = time.perf_counter()
        while not STATE.stop_event.is_set():
            ok, frame = capture.read()
            if not ok:
                break
            frame_number = int(capture.get(cv2.CAP_PROP_POS_FRAMES))
            video_s = frame_number / source_fps
            detector_result = detector.predict(
                frame, imgsz=args.imgsz, conf=args.det_conf,
                device=args.device, verbose=False, half=args.half)[0]
            balls = ball_observations(
                detector_result, video_s, width, height,
                f'{args.target_ball_color}_ball', args.ball_diameter_m,
                args.horizontal_fov_deg)
            planner.update_ball_observations(balls, video_s)
            gate_proposal, _counts = draw_detector(frame, detector_result)

            gates = []
            pose_detected = pose_valid = 0
            if gate_proposal or planner.state in (
                    MissionState.SEARCH_GATE, MissionState.ALIGN_GATE,
                    MissionState.APPROACH_GATE):
                pose_result = pose_model.predict(
                    frame, imgsz=args.imgsz, conf=args.pose_conf,
                    device=args.device, verbose=False, half=args.half)[0]
                pose_detected, pose_valid = draw_gate_pose(
                    frame, pose_result, args.keypoint_conf)
                gates = gate_observations(
                    pose_result, video_s, width, height,
                    args.horizontal_fov_deg, args.keypoint_conf)
            planner.update_gate_observations(gates, video_s)
            output = planner.tick(video_s)
            if ros_output is not None:
                ros_output.publish(output)
            if output.state != previous_state:
                state_changes += 1
                previous_state = output.state
            draw_control_panel(
                frame, output, args.start_phase, ros_output is not None)
            kind, detail = target_text(output)
            processed += 1
            actual_fps = processed / max(time.perf_counter() - wall_started, 1e-6)
            log.writerow([
                frame_number, f'{video_s:.3f}', output.state.value,
                f'{output.surge:.6f}', f'{output.sway:.6f}',
                f'{output.heave:.6f}', f'{output.yaw:.6f}',
                ('' if output.target_depth_m is None
                 else f'{output.target_depth_m:.6f}'),
                int(output.motion_enable), int(ros_output is not None), kind, detail,
            ])
            if processed % 30 == 0:
                csv_file.flush()
            writer.write(frame)
            encoded, buffer = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
            if encoded:
                with STATE.condition:
                    STATE.jpeg = buffer.tobytes()
                    STATE.sequence += 1
                    STATE.condition.notify_all()
            STATE.stats.update(
                frame=frame_number, fps=actual_fps, state=output.state.value,
                surge=output.surge, sway=output.sway,
                heave=output.heave, yaw=output.yaw,
                target_depth_m=output.target_depth_m,
                planned_motion_enable=output.motion_enable,
                nanopi_transmission=ros_output is not None, target_kind=kind,
                target_detail=detail, state_changes=state_changes,
                pose_detected=pose_detected, pose_valid=pose_valid,
            )
            if args.max_frames and processed >= args.max_frames:
                break
            target_elapsed = processed / source_fps
            delay = target_elapsed - (time.perf_counter() - wall_started)
            if args.realtime and delay > 0:
                time.sleep(delay)
    except Exception as error:
        STATE.error = f'{type(error).__name__}: {error}'
    finally:
        try:
            # Zero and disable real output before closing potentially slow
            # video resources. This also runs after inference/write errors.
            if ros_output is not None:
                ros_output.stop()
        except Exception as stop_error:
            suffix = f'{type(stop_error).__name__}: {stop_error}'
            STATE.error = f'{STATE.error}; stop failed: {suffix}'.strip('; ')
        finally:
            if capture is not None:
                capture.release()
            if writer is not None:
                writer.release()
            if csv_file is not None:
                csv_file.close()
            STATE.finished = True
            with STATE.condition:
                STATE.condition.notify_all()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', type=Path, default=DEFAULT_VIDEO)
    parser.add_argument('--detector', type=Path, default=DEFAULT_DETECTOR)
    parser.add_argument('--pose', type=Path, default=DEFAULT_POSE)
    output = HERE / 'video_test_results'
    parser.add_argument('--output-video', type=Path,
                        default=output / 'preview_closed_loop.mp4')
    parser.add_argument('--output-csv', type=Path,
                        default=output / 'preview_closed_loop_commands.csv')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--start-phase', choices=('ball', 'gate'), default='gate')
    parser.add_argument('--start-seconds', type=float, default=90.0)
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--device', default='0')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--det-conf', type=float, default=0.45)
    parser.add_argument('--pose-conf', type=float, default=0.50)
    parser.add_argument('--keypoint-conf', type=float, default=0.50)
    parser.add_argument('--target-ball-color', choices=('red', 'blue'), default='red')
    parser.add_argument('--ball-diameter-m', type=float, default=0.20)
    parser.add_argument('--horizontal-fov-deg', type=float, default=110.0)
    parser.add_argument('--simulated-depth-m', type=float, default=0.60)
    parser.add_argument('--detection-confirmations', type=int, default=3)
    parser.add_argument('--jpeg-quality', type=int, default=85)
    parser.add_argument('--half', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--realtime', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--enable-nanopi', action='store_true')
    parser.add_argument('--allow-pwm', action='store_true')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.enable_nanopi and not args.allow_pwm:
        raise SystemExit(
            'NanoPi output refused: physically disconnect thruster power and '
            'add --allow-pwm')
    worker = threading.Thread(target=inference_worker, args=(args,), daemon=True)
    worker.start()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.timeout = 1.0
    print(f'闭环调试页面：http://{args.host}:{args.port}', flush=True)
    print('NANOPI_TRANSMISSION=' +
          ('ENABLED' if args.enable_nanopi else 'DISABLED'), flush=True)
    try:
        while not STATE.stop_event.is_set() and not STATE.finished:
            server.handle_request()
    except KeyboardInterrupt:
        STATE.stop_event.set()
    finally:
        server.server_close()
        worker.join(timeout=5.0)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
