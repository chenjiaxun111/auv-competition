#!/usr/bin/env python3
"""Test the trained four-corner gate pose model on a complete video."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


HERE = Path(__file__).resolve().parent
DEFAULT_VIDEO = HERE / "preview.mp4"
DEFAULT_MODEL = (
    HERE / "1" / "training_runs" / "gate_yolo11n_pose_v2" / "weights" / "best.pt"
)
DEFAULT_OUTPUT = HERE / "video_test_results" / "preview_gate_pose_v2.mp4"
COLORS = ((0, 255, 255), (0, 255, 0), (255, 0, 255), (255, 128, 0))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, default=DEFAULT_VIDEO)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--conf", type=float, default=0.50)
    parser.add_argument("--keypoint-conf", type=float, default=0.50)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    return parser.parse_args()


def valid_quad(points: np.ndarray, width: int, height: int) -> bool:
    if points.shape != (4, 2) or not np.isfinite(points).all():
        return False
    if np.any(points[:, 0] < 0) or np.any(points[:, 0] >= width):
        return False
    if np.any(points[:, 1] < 0) or np.any(points[:, 1] >= height):
        return False
    lt, rt, rb, lb = points
    if (lt[0] + lb[0]) >= (rt[0] + rb[0]):
        return False
    if (lt[1] + rt[1]) >= (lb[1] + rb[1]):
        return False
    contour = points.astype(np.int32).reshape((-1, 1, 2))
    return bool(cv2.isContourConvex(contour))


def draw_pose(
    frame: np.ndarray,
    box: np.ndarray,
    box_conf: float,
    points: np.ndarray,
    point_conf: np.ndarray,
    valid: bool,
) -> None:
    x1, y1, x2, y2 = map(int, box)
    outline = (255, 255, 255) if valid else (0, 0, 255)
    cv2.rectangle(frame, (x1, y1), (x2, y2), outline, 2)
    minimum = float(np.min(point_conf))
    cv2.putText(
        frame,
        f"gate {box_conf:.2f} kpt_min {minimum:.2f}",
        (x1, max(25, y1 - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        outline,
        2,
        cv2.LINE_AA,
    )
    integer_points = np.rint(points).astype(int)
    for index, (point, confidence) in enumerate(zip(integer_points, point_conf)):
        color = COLORS[index]
        cv2.circle(frame, tuple(point), 7, color, -1)
        cv2.putText(
            frame,
            f"{index + 1}:{confidence:.2f}",
            (int(point[0]) + 8, int(point[1]) - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
            cv2.LINE_AA,
        )
    for index in range(4):
        cv2.line(
            frame,
            tuple(integer_points[index]),
            tuple(integer_points[(index + 1) % 4]),
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )


def main() -> int:
    args = parse_args()
    video = args.video.expanduser().resolve()
    model_path = args.model.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not video.is_file():
        raise SystemExit(f"视频不存在: {video}")
    if not model_path.is_file():
        raise SystemExit(f"模型不存在: {model_path}")

    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise SystemExit(f"无法打开视频: {video}")
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = float(capture.get(cv2.CAP_PROP_FPS) or 30.0)
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
    )
    if not writer.isOpened():
        raise SystemExit(f"无法创建输出视频: {output}")

    model = YOLO(str(model_path))
    frame_count = 0
    detected_frames = 0
    valid_frames = 0
    multi_frames = 0
    instance_count = 0
    valid_instances = 0
    box_scores: list[float] = []
    minimum_keypoint_scores: list[float] = []
    inference_ms: list[float] = []
    started_all = time.perf_counter()

    while True:
        ok, frame = capture.read()
        if not ok:
            break
        started = time.perf_counter()
        result = model.predict(
            frame,
            imgsz=args.imgsz,
            conf=args.conf,
            device=args.device,
            verbose=False,
        )[0]
        inference_ms.append((time.perf_counter() - started) * 1000.0)
        detections_this_frame = 0
        valid_this_frame = 0
        if result.boxes is not None and result.keypoints is not None:
            boxes = result.boxes.xyxy.cpu().numpy()
            scores = result.boxes.conf.cpu().numpy()
            points_all = result.keypoints.xy.cpu().numpy()
            confidences = result.keypoints.conf
            confidences = (
                confidences.cpu().numpy()
                if confidences is not None
                else np.ones((len(points_all), 4), dtype=np.float32)
            )
            for box, score, points, point_scores in zip(
                boxes, scores, points_all, confidences
            ):
                minimum = float(np.min(point_scores))
                geometry_ok = valid_quad(points, width, height)
                valid = geometry_ok and minimum >= args.keypoint_conf
                draw_pose(frame, box, float(score), points, point_scores, valid)
                detections_this_frame += 1
                instance_count += 1
                box_scores.append(float(score))
                minimum_keypoint_scores.append(minimum)
                if valid:
                    valid_this_frame += 1
                    valid_instances += 1

        frame_count += 1
        if detections_this_frame:
            detected_frames += 1
        if valid_this_frame:
            valid_frames += 1
        if detections_this_frame > 1:
            multi_frames += 1
        cv2.putText(
            frame,
            f"Pose v2  frame {frame_count}/{total}  detections {detections_this_frame}",
            (18, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        writer.write(frame)
        if frame_count % 500 == 0:
            print(f"PROGRESS={frame_count}/{total}", flush=True)

    elapsed = time.perf_counter() - started_all
    capture.release()
    writer.release()

    def average(values: list[float]) -> float | None:
        return None if not values else float(np.mean(values))

    def percentile(values: list[float], value: float) -> float | None:
        return None if not values else float(np.percentile(values, value))

    report = {
        "video": str(video),
        "model": str(model_path),
        "output": str(output),
        "thresholds": {
            "box_confidence": args.conf,
            "minimum_keypoint_confidence": args.keypoint_conf,
        },
        "frames": frame_count,
        "detected_frames": detected_frames,
        "detected_frame_ratio": detected_frames / max(frame_count, 1),
        "valid_pose_frames": valid_frames,
        "valid_pose_frame_ratio": valid_frames / max(frame_count, 1),
        "multiple_detection_frames": multi_frames,
        "instances": instance_count,
        "valid_instances": valid_instances,
        "mean_box_confidence": average(box_scores),
        "mean_minimum_keypoint_confidence": average(minimum_keypoint_scores),
        "p05_minimum_keypoint_confidence": percentile(minimum_keypoint_scores, 5),
        "mean_inference_ms": average(inference_ms),
        "p95_inference_ms": percentile(inference_ms, 95),
        "wall_time_s": elapsed,
        "processing_fps": frame_count / max(elapsed, 1e-9),
    }
    report_path = output.with_suffix(".json")
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"VIDEO_OUTPUT={output}")
    print(f"REPORT={report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
