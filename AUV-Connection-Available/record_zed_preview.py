#!/usr/bin/python3
"""Record a lossless ZED SVO2 and serve a read-only left-eye web preview."""

from __future__ import annotations

import argparse
import json
import shutil
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, NoReturn

import cv2
import pyzed.sl as sl


HTML = b"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ZED recording preview</title>
<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}
h2{margin:.6rem}.wrap{max-width:1100px;margin:auto;padding:.5rem}
img{width:100%;height:auto;background:#222;border:1px solid #444}
#s{white-space:pre-wrap;text-align:left;font-family:monospace;font-size:14px}</style>
</head><body><div class="wrap"><h2>ZED SVO2 recording preview</h2>
<img src="/stream.mjpg" alt="Waiting for ZED frames"><div id="s">Starting...</div>
</div><script>async function u(){try{let r=await fetch('/api/status',{cache:'no-store'});
let x=await r.json();document.getElementById('s').textContent=JSON.stringify(x,null,2)}
catch(e){document.getElementById('s').textContent='Preview disconnected: '+e}}
setInterval(u,1000);u();</script></body></html>"""


def fail(message: str) -> NoReturn:
    raise RuntimeError(message)


class PreviewServer:
    """Newest-frame-only MJPEG server; it never owns or controls the camera."""

    def __init__(self, host: str, port: int) -> None:
        self.condition = threading.Condition()
        self.jpeg: bytes | None = None
        self.status: dict[str, Any] = {"state": "starting"}
        self.updated: float | None = None
        self.sequence = 0
        self.closed = False
        preview = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_: Any) -> None:
                pass

            def send_bytes(
                self, body: bytes, content_type: str, status: int = 200
            ) -> None:
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
                self.connection.settimeout(5)
                path = self.path.split("?", 1)[0]
                try:
                    if path == "/":
                        self.send_bytes(HTML, "text/html; charset=utf-8")
                    elif path == "/api/status":
                        with preview.condition:
                            age = (
                                time.monotonic() - preview.updated
                                if preview.updated is not None
                                else None
                            )
                            state = dict(preview.status)
                            state["preview_age_s"] = (
                                round(age, 3) if age is not None else None
                            )
                        self.send_bytes(
                            json.dumps(state, ensure_ascii=False).encode("utf-8"),
                            "application/json; charset=utf-8",
                        )
                    elif path == "/frame.jpg":
                        with preview.condition:
                            jpeg = preview.jpeg
                        self.send_bytes(
                            jpeg or b"Waiting for camera",
                            "image/jpeg" if jpeg else "text/plain",
                            200 if jpeg else 503,
                        )
                    elif path == "/stream.mjpg":
                        self.send_response(200)
                        self.send_header(
                            "Content-Type",
                            "multipart/x-mixed-replace; boundary=frame",
                        )
                        self.send_header("Cache-Control", "no-store")
                        self.end_headers()
                        sequence = -1
                        while not preview.closed:
                            with preview.condition:
                                preview.condition.wait_for(
                                    lambda: preview.sequence != sequence
                                    or preview.closed,
                                    timeout=2,
                                )
                                if preview.closed:
                                    break
                                if preview.jpeg is None or preview.sequence == sequence:
                                    continue
                                sequence = preview.sequence
                                jpeg = preview.jpeg
                            self.wfile.write(
                                b"--frame\r\nContent-Type: image/jpeg\r\n"
                                + b"Content-Length: "
                                + str(len(jpeg)).encode("ascii")
                                + b"\r\n\r\n"
                                + jpeg
                                + b"\r\n"
                            )
                            self.wfile.flush()
                    else:
                        self.send_bytes(b"Not found", "text/plain", 404)
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    pass

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    @property
    def port(self) -> int:
        return int(self.server.server_address[1])

    def publish(self, jpeg: bytes, status: dict[str, Any]) -> None:
        with self.condition:
            self.jpeg = jpeg
            self.status = status
            self.updated = time.monotonic()
            self.sequence += 1
            self.condition.notify_all()

    def close(self) -> None:
        with self.condition:
            self.closed = True
            self.condition.notify_all()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", type=int, default=600)
    parser.add_argument("--resolution", default="HD720")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--preview-host", default="192.168.137.1")
    parser.add_argument("--preview-port", type=int, default=8080)
    parser.add_argument("--preview-fps", type=float, default=8)
    parser.add_argument("--preview-width", type=int, default=640)
    parser.add_argument("--jpeg-quality", type=int, default=75)
    parser.add_argument("--min-free-gb", type=int, default=10)
    args = parser.parse_args()
    if args.duration != 0 and args.duration < 5:
        parser.error("duration must be 0 or at least 5 seconds")
    if not 1 <= args.fps <= 120:
        parser.error("fps must be between 1 and 120")
    if not 1 <= args.preview_port <= 65535:
        parser.error("preview-port must be between 1 and 65535")
    if not 0 < args.preview_fps <= min(30, args.fps):
        parser.error("preview-fps must be positive and no higher than camera FPS")
    if not 160 <= args.preview_width <= 1920:
        parser.error("preview-width must be between 160 and 1920")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("jpeg-quality must be between 1 and 100")
    if args.min_free_gb < 1:
        parser.error("min-free-gb must be at least 1")
    return args


def main() -> int:
    args = parse_args()
    output = args.output.expanduser().resolve()
    if output.exists():
        fail(f"output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)

    try:
        resolution = getattr(sl.RESOLUTION, args.resolution)
    except AttributeError:
        fail(f"unsupported ZED resolution: {args.resolution}")

    stop_event = threading.Event()

    def request_stop(*_: Any) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    preview = PreviewServer(args.preview_host, args.preview_port)
    camera = sl.Camera()
    opened = False
    recording = False
    frames = 0
    errors = 0
    started = time.monotonic()
    last_preview = 0.0
    last_report = started
    last_report_frames = 0
    target_frames = args.duration * args.fps if args.duration else 0
    reserve_bytes = args.min_free_gb * 1024**3
    image = sl.Mat()

    try:
        init = sl.InitParameters()
        init.camera_resolution = resolution
        init.camera_fps = args.fps
        init.depth_mode = sl.DEPTH_MODE.NONE
        status = camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            fail(f"camera open failed: {status}")
        opened = True

        record_params = sl.RecordingParameters(
            str(output), sl.SVO_COMPRESSION_MODE.LOSSLESS
        )
        status = camera.enable_recording(record_params)
        if status != sl.ERROR_CODE.SUCCESS:
            fail(f"recording initialization failed: {status}")
        recording = True
        started = last_report = time.monotonic()
        print(
            f"READY preview=http://{args.preview_host}:{preview.port} "
            f"output={output} compression=LOSSLESS",
            flush=True,
        )

        runtime = sl.RuntimeParameters()
        while not stop_event.is_set() and (
            target_frames == 0 or frames < target_frames
        ):
            status = camera.grab(runtime)
            if status != sl.ERROR_CODE.SUCCESS:
                errors += 1
                if errors >= 30:
                    fail(f"repeated camera grab failure: {status}")
                time.sleep(0.01)
                continue
            errors = 0
            frames += 1
            now = time.monotonic()

            if frames % max(args.fps, 1) == 0:
                free_bytes = shutil.disk_usage(output.parent).free
                if free_bytes < reserve_bytes:
                    print(
                        f"STOPPING: free space below {args.min_free_gb} GiB reserve",
                        flush=True,
                    )
                    stop_event.set()

            if now - last_preview >= 1.0 / args.preview_fps:
                camera.retrieve_image(image, sl.VIEW.LEFT)
                view = image.get_data()[:, :, :3]
                height = max(1, round(view.shape[0] * args.preview_width / view.shape[1]))
                small = cv2.resize(view, (args.preview_width, height))
                elapsed = now - started
                cv2.putText(
                    small,
                    f"REC {elapsed:6.1f}s  frame {frames}",
                    (12, 26),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 0, 255),
                    2,
                )
                encoded, jpeg = cv2.imencode(
                    ".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
                )
                if encoded:
                    free_gb = shutil.disk_usage(output.parent).free / 1024**3
                    preview.publish(
                        jpeg.tobytes(),
                        {
                            "state": "recording",
                            "frames": frames,
                            "target_frames": target_frames or None,
                            "elapsed_s": round(elapsed, 1),
                            "camera_fps": args.fps,
                            "preview_fps_limit": args.preview_fps,
                            "free_gb": round(free_gb, 1),
                            "output": str(output),
                            "compression": "LOSSLESS_ZSTD",
                        },
                    )
                last_preview = now

            if now - last_report >= 5:
                capture_fps = (frames - last_report_frames) / (now - last_report)
                print(
                    json.dumps(
                        {
                            "frames": frames,
                            "target_frames": target_frames or None,
                            "capture_fps": round(capture_fps, 2),
                            "elapsed_s": round(now - started, 1),
                        }
                    ),
                    flush=True,
                )
                last_report = now
                last_report_frames = frames

        return 0
    finally:
        if recording:
            camera.disable_recording()
        if opened:
            camera.close()
        preview.close()
        elapsed = time.monotonic() - started
        print(
            json.dumps(
                {
                    "summary": {
                        "frames": frames,
                        "elapsed_s": round(elapsed, 2),
                        "average_fps": round(frames / elapsed, 2) if elapsed else 0,
                        "output": str(output),
                    }
                }
            ),
            flush=True,
        )


if __name__ == "__main__":
    raise SystemExit(main())
