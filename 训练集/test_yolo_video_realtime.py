#!/usr/bin/env python3
"""使用比赛 YOLO 模型实时显示视频检测框，并保存带框视频。"""

from __future__ import annotations

import argparse
import time
from collections import Counter
from pathlib import Path

import cv2
from ultralytics import YOLO


HERE = Path(__file__).resolve().parent
DEFAULT_VIDEO = HERE / "preview.mp4"
DEFAULT_MODEL = HERE / "1" / "training_runs" / "all_yolo11n" / "weights" / "best.pt"
DEFAULT_OUTPUT = HERE / "video_test_results" / "preview_detected.mp4"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, default=DEFAULT_VIDEO)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--no-show", action="store_true", help="不打开窗口，只保存结果")
    parser.add_argument("--no-realtime", action="store_true", help="不按原视频帧率限速")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    video_path = args.video.resolve()
    model_path = args.model.resolve()
    output_path = args.output.resolve()
    if not video_path.is_file():
        raise SystemExit(f"视频不存在: {video_path}")
    if not model_path.is_file():
        raise SystemExit(f"模型不存在: {model_path}")

    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise SystemExit(f"无法打开视频: {video_path}")
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    source_fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        source_fps,
        (width, height),
    )
    if not writer.isOpened():
        capture.release()
        raise SystemExit(f"无法创建输出视频: {output_path}")

    model = YOLO(str(model_path))
    counts: Counter[str] = Counter()
    frame_index = 0
    fps_ema: float | None = None
    window_name = "AUV YOLO11n - Q/ESC to stop"
    if not args.no_show:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, min(width, 1280), min(height, 720))

    print(
        f"VIDEO={video_path}\nMODEL={model_path}\nOUTPUT={output_path}\n"
        f"SIZE={width}x{height} FPS={source_fps:.2f} FRAMES={total_frames}"
    )
    try:
        while True:
            frame_start = time.perf_counter()
            ok, frame = capture.read()
            if not ok:
                break

            result = model.predict(
                frame,
                imgsz=args.imgsz,
                conf=args.conf,
                device=args.device,
                verbose=False,
            )[0]
            annotated = result.plot(line_width=2, font_size=0.7)

            if result.boxes is not None:
                for class_id in result.boxes.cls.int().cpu().tolist():
                    counts[result.names[class_id]] += 1

            processing_seconds = time.perf_counter() - frame_start
            instant_fps = 1.0 / max(processing_seconds, 1e-6)
            fps_ema = instant_fps if fps_ema is None else 0.9 * fps_ema + 0.1 * instant_fps
            progress = 100.0 * (frame_index + 1) / max(total_frames, 1)
            cv2.putText(
                annotated,
                f"Inference {fps_ema:.1f} FPS  Frame {frame_index + 1}/{total_frames}  {progress:.1f}%",
                (18, 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            writer.write(annotated)

            wait_ms = 1
            if not args.no_realtime:
                wait_ms = max(1, round(1000.0 / source_fps - processing_seconds * 1000.0))
            key = cv2.waitKey(wait_ms) & 0xFF if not args.no_show else 255
            if key in (ord("q"), ord("Q"), 27):
                print("STOPPED_BY_USER=1")
                break

            frame_index += 1
            if frame_index % 300 == 0:
                print(
                    f"PROGRESS={frame_index}/{total_frames} "
                    f"({100.0 * frame_index / max(total_frames, 1):.1f}%) "
                    f"INFERENCE_FPS={fps_ema:.1f}",
                    flush=True,
                )
    finally:
        capture.release()
        writer.release()
        if not args.no_show:
            cv2.destroyAllWindows()

    print(f"SAVED={output_path}")
    print(f"PROCESSED_FRAMES={frame_index + 1 if total_frames else frame_index}")
    print("DETECTIONS=" + ",".join(f"{name}:{count}" for name, count in sorted(counts.items())))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
