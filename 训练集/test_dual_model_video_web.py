#!/usr/bin/env python3
"""在浏览器中实时预览普通检测模型 + 门框 Pose 模型的协同结果。"""

from __future__ import annotations

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
from ultralytics import YOLO

from test_dual_model_video_realtime import (
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
        self.error = ""
        self.stats = {
            "frame": 0,
            "total_frames": 0,
            "fps": 0.0,
            "gate_proposals": 0,
            "pose_valid": 0,
            "output": "",
        }


STATE = SharedState()


def page_html() -> bytes:
    return """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>AUV 双模型实时检测</title>
<style>
body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}
h2{margin:14px 0 5px}.hint{color:#aaa;margin:5px 0 12px}
img{max-width:96vw;max-height:80vh;border:1px solid #555;background:#000}
button{margin:12px;padding:8px 22px;font-size:16px}
#status{margin:8px;color:#8ee}
</style></head><body>
<h2>AUV 双模型实时检测</h2>
<div class="hint">球：普通检测模型；门框四角：Pose 模型</div>
<img src="/stream" alt="正在等待检测画面">
<div id="status">正在启动模型……</div>
<button onclick="fetch('/stop',{method:'POST'});this.disabled=true">停止测试</button>
<script>
setInterval(async()=>{try{let r=await fetch('/stats');let s=await r.json();
document.getElementById('status').textContent = s.error ? ('错误：'+s.error) :
`帧 ${s.frame}/${s.total_frames}　速度 ${s.fps.toFixed(1)} FPS　有效门框姿态 ${s.pose_valid}`;
}catch(e){}},500);
</script></body></html>""".encode("utf-8")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, _format: str, *_args) -> None:
        pass

    def do_GET(self) -> None:
        if self.path == "/":
            body = page_html()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/stats":
            payload = dict(STATE.stats)
            payload["finished"] = STATE.finished
            payload["error"] = STATE.error
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/stream":
            STATE.client_connected.set()
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last_sequence = -1
            try:
                while not STATE.stop_event.is_set():
                    with STATE.condition:
                        STATE.condition.wait_for(
                            lambda: STATE.sequence != last_sequence or STATE.finished,
                            timeout=2.0,
                        )
                        jpeg = STATE.jpeg
                        last_sequence = STATE.sequence
                        finished = STATE.finished
                    if jpeg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                    if finished:
                        break
            except (BrokenPipeError, ConnectionResetError):
                pass
            return
        self.send_error(404)

    def do_POST(self) -> None:
        if self.path != "/stop":
            self.send_error(404)
            return
        STATE.stop_event.set()
        body = b"stopping"
        self.send_response(200)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def inference_worker(args: argparse.Namespace) -> None:
    cap = None
    writer = None
    try:
        detector = YOLO(str(args.detector))
        pose_model = YOLO(str(args.pose))
        cap = cv2.VideoCapture(str(args.video))
        if not cap.isOpened():
            raise RuntimeError(f"无法打开视频：{args.video}")

        source_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), source_fps, (width, height)
        )
        if not writer.isOpened():
            raise RuntimeError(f"无法创建输出视频：{args.output}")

        STATE.stats.update(total_frames=total, output=str(args.output))
        # 等浏览器真正打开流后再开始，避免用户错过视频开头。
        while not STATE.client_connected.wait(0.2):
            if STATE.stop_event.is_set():
                return

        frame_index = 0
        started = time.perf_counter()
        while not STATE.stop_event.is_set():
            ok, frame = cap.read()
            if not ok:
                break
            frame_index += 1
            detector_result = detector.predict(
                frame, imgsz=args.imgsz, conf=args.det_conf, device=args.device,
                verbose=False, half=args.half,
            )[0]
            annotated = frame.copy()
            gate_proposal, _frame_counts = draw_detector(annotated, detector_result)

            pose_valid = 0
            pose_detected = 0
            if gate_proposal:
                pose_result = pose_model.predict(
                    frame, imgsz=args.imgsz, conf=args.pose_conf, device=args.device,
                    verbose=False, half=args.half,
                )[0]
                pose_detected, pose_valid = draw_gate_pose(
                    annotated, pose_result, args.keypoint_conf
                )

            elapsed = max(time.perf_counter() - started, 1e-6)
            actual_fps = frame_index / elapsed
            cv2.putText(
                annotated,
                f"dual-model | frame {frame_index}/{total} | {actual_fps:.1f} FPS",
                (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2,
                cv2.LINE_AA,
            )
            writer.write(annotated)
            encoded, buffer = cv2.imencode(
                ".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
            )
            if encoded:
                with STATE.condition:
                    STATE.jpeg = buffer.tobytes()
                    STATE.sequence += 1
                    STATE.condition.notify_all()
            STATE.stats.update(
                frame=frame_index,
                fps=actual_fps,
                gate_proposals=STATE.stats["gate_proposals"] + int(gate_proposal),
                pose_valid=STATE.stats["pose_valid"] + pose_valid,
            )

            # 按源视频帧率播放；推理比帧率慢时不额外等待。
            target_elapsed = frame_index / source_fps
            delay = target_elapsed - (time.perf_counter() - started)
            if delay > 0:
                time.sleep(delay)
    except Exception as exc:  # 页面会显示具体错误
        STATE.error = str(exc)
    finally:
        if cap is not None:
            cap.release()
        if writer is not None:
            writer.release()
        STATE.finished = True
        with STATE.condition:
            STATE.condition.notify_all()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", type=Path, default=DEFAULT_VIDEO)
    parser.add_argument("--detector", type=Path, default=DEFAULT_DETECTOR)
    parser.add_argument("--pose", type=Path, default=DEFAULT_POSE)
    parser.add_argument(
        "--output", type=Path,
        default=Path(__file__).resolve().parent / "video_test_results/preview_dual_model_web.mp4",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--det-conf", type=float, default=0.35)
    parser.add_argument("--pose-conf", type=float, default=0.30)
    parser.add_argument("--keypoint-conf", type=float, default=0.30)
    parser.add_argument("--min-gate-area", type=float, default=0.01)
    parser.add_argument("--gate-class", default="gate")
    parser.add_argument("--ball-classes", nargs="+", default=["blue_ball", "red_ball"])
    parser.add_argument("--jpeg-quality", type=int, default=85)
    parser.add_argument("--half", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    worker = threading.Thread(target=inference_worker, args=(args,), daemon=True)
    worker.start()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.timeout = 1.0
    print(f"浏览器预览：http://{args.host}:{args.port}", flush=True)
    print(f"结果保存：{args.output}", flush=True)
    try:
        while not STATE.stop_event.is_set():
            server.handle_request()
            if STATE.finished:
                # 视频结束后页面仍保留，直到用户停止或终端 Ctrl+C。
                continue
    except KeyboardInterrupt:
        STATE.stop_event.set()
    finally:
        server.server_close()
        worker.join(timeout=5.0)


if __name__ == "__main__":
    main()
