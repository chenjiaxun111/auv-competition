#!/usr/bin/env python3
"""通过本地网页实时查看 YOLO 视频检测框，并保存检测视频。"""

from __future__ import annotations

import argparse
import json
import threading
import time
from collections import Counter
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
from ultralytics import YOLO


HERE = Path(__file__).resolve().parent
DEFAULT_VIDEO = HERE / "preview.mp4"
DEFAULT_MODEL = HERE / "1" / "training_runs" / "all_yolo11n" / "weights" / "best.pt"
DEFAULT_OUTPUT = HERE / "video_test_results" / "preview_detected.mp4"


class SharedState:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.jpeg: bytes | None = None
        self.sequence = 0
        self.client_connected = threading.Event()
        self.finished = False
        self.error: str | None = None
        self.stats: dict[str, object] = {"status": "等待浏览器连接"}

    def publish(self, jpeg: bytes, stats: dict[str, object]) -> None:
        with self.condition:
            self.jpeg = jpeg
            self.stats = stats
            self.sequence += 1
            self.condition.notify_all()


STATE = SharedState()


INDEX_HTML = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><title>AUV YOLO11n 视频测试</title>
<style>body{margin:0;background:#10151b;color:#eaf2f8;font-family:sans-serif;text-align:center}
h2{margin:12px 0 6px}#stats{color:#73d5ff;margin-bottom:8px}img{max-width:96vw;max-height:84vh;border:1px solid #345;background:#000}</style></head>
<body><h2>AUV YOLO11n 实时检测</h2><div id="stats">正在连接……</div>
<img src="/stream" alt="实时检测画面"><script>
setInterval(async()=>{try{let r=await fetch('/stats');let s=await r.json();
document.getElementById('stats').textContent=`${s.status||''}　帧 ${s.frame||0}/${s.total||0}　${s.progress||0}%　推理 ${s.fps||0} FPS`; }catch(e){}},500);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format: str, *args: object) -> None:
        return

    def do_GET(self) -> None:
        if self.path == "/":
            body = INDEX_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/stats":
            body = json.dumps(STATE.stats, ensure_ascii=False).encode("utf-8")
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
                while True:
                    with STATE.condition:
                        STATE.condition.wait_for(
                            lambda: STATE.sequence != last_sequence or STATE.finished,
                            timeout=2.0,
                        )
                        jpeg = STATE.jpeg
                        last_sequence = STATE.sequence
                        finished = STATE.finished
                    if jpeg is not None:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg + b"\r\n")
                    if finished:
                        break
            except (BrokenPipeError, ConnectionResetError):
                pass
            return
        self.send_error(404)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, default=DEFAULT_VIDEO)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--no-realtime", action="store_true")
    return parser.parse_args()


def inference_worker(args: argparse.Namespace) -> None:
    video = args.video.resolve()
    model_path = args.model.resolve()
    output = args.output.resolve()
    try:
        capture = cv2.VideoCapture(str(video))
        if not capture.isOpened():
            raise RuntimeError(f"无法打开视频: {video}")
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        source_fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
        total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        output.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(output), cv2.VideoWriter_fourcc(*"mp4v"), source_fps, (width, height)
        )
        if not writer.isOpened():
            raise RuntimeError(f"无法创建输出视频: {output}")

        model = YOLO(str(model_path))
        counts: Counter[str] = Counter()
        fps_ema: float | None = None
        frame_index = 0
        STATE.stats = {"status": "等待浏览器打开实时画面", "frame": 0, "total": total}
        STATE.client_connected.wait()
        print("CLIENT_CONNECTED=1", flush=True)

        while True:
            started = time.perf_counter()
            ok, frame = capture.read()
            if not ok:
                break
            result = model.predict(
                frame, imgsz=args.imgsz, conf=args.conf, device=args.device, verbose=False
            )[0]
            annotated = result.plot(line_width=2, font_size=0.7)
            if result.boxes is not None:
                for class_id in result.boxes.cls.int().cpu().tolist():
                    counts[result.names[class_id]] += 1

            elapsed = time.perf_counter() - started
            instant_fps = 1.0 / max(elapsed, 1e-6)
            fps_ema = instant_fps if fps_ema is None else fps_ema * 0.9 + instant_fps * 0.1
            frame_index += 1
            progress = 100.0 * frame_index / max(total, 1)
            cv2.putText(
                annotated,
                f"Inference {fps_ema:.1f} FPS  Frame {frame_index}/{total}  {progress:.1f}%",
                (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2, cv2.LINE_AA,
            )
            writer.write(annotated)
            encoded, buffer = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if encoded:
                STATE.publish(buffer.tobytes(), {
                    "status": "检测中", "frame": frame_index, "total": total,
                    "progress": round(progress, 1), "fps": round(fps_ema, 1),
                    "detections": dict(counts),
                })
            if not args.no_realtime:
                time.sleep(max(0.0, 1.0 / source_fps - (time.perf_counter() - started)))
            if frame_index % 300 == 0:
                print(f"PROGRESS={frame_index}/{total} ({progress:.1f}%)", flush=True)

        capture.release()
        writer.release()
        STATE.stats = {
            "status": "检测完成", "frame": frame_index, "total": total,
            "progress": 100.0, "fps": round(fps_ema or 0.0, 1), "detections": dict(counts),
        }
        print(f"SAVED={output}\nDETECTIONS={dict(counts)}", flush=True)
    except Exception as error:
        STATE.error = str(error)
        STATE.stats = {"status": f"错误：{error}"}
        print(f"ERROR={error}", flush=True)
    finally:
        with STATE.condition:
            STATE.finished = True
            STATE.condition.notify_all()


def main() -> int:
    args = parse_args()
    worker = threading.Thread(target=inference_worker, args=(args,), daemon=True)
    worker.start()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"PREVIEW_URL=http://{args.host}:{args.port}/", flush=True)
    try:
        while worker.is_alive():
            server.handle_request()
        # 完成后继续响应几次，方便浏览器显示最终一帧和完成状态。
        server.timeout = 1.0
        for _ in range(10):
            server.handle_request()
    finally:
        server.server_close()
    return 1 if STATE.error else 0


if __name__ == "__main__":
    raise SystemExit(main())
