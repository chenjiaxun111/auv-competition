#!/usr/bin/env python3
"""Replay a video through the competition detector and gate Pose model."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
import time

import cv2
import numpy as np
from ultralytics import YOLO


HERE = Path(__file__).resolve().parent
DEFAULT_VIDEO = HERE / 'preview.mp4'
DEFAULT_DETECTOR = (
    HERE / '1' / 'training_runs' / 'all_yolo11n' / 'weights' / 'best.pt'
)
DEFAULT_POSE = (
    HERE / '1' / 'training_runs' / 'gate_yolo11n_pose_v2' / 'weights' / 'best.pt'
)
DEFAULT_OUTPUT = HERE / 'video_test_results' / 'preview_dual_model_realtime.mp4'
POINT_COLOURS = ((0, 255, 255), (0, 255, 0), (255, 0, 255), (255, 128, 0))
BOX_COLOURS = {
    'red_ball': (0, 0, 255),
    'blue_ball': (255, 120, 0),
    'gate': (255, 220, 0),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', type=Path, default=DEFAULT_VIDEO)
    parser.add_argument('--detector', type=Path, default=DEFAULT_DETECTOR)
    parser.add_argument('--pose', type=Path, default=DEFAULT_POSE)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--det-conf', type=float, default=0.45)
    parser.add_argument('--pose-conf', type=float, default=0.50)
    parser.add_argument('--keypoint-conf', type=float, default=0.50)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--device', default='0')
    parser.add_argument('--start-seconds', type=float, default=0.0)
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--no-realtime', action='store_true')
    return parser.parse_args()


def valid_quad(points: np.ndarray, width: int, height: int) -> bool:
    if points.shape != (4, 2) or not np.isfinite(points).all():
        return False
    if np.any(points[:, 0] < 0) or np.any(points[:, 0] >= width):
        return False
    if np.any(points[:, 1] < 0) or np.any(points[:, 1] >= height):
        return False
    tl, tr, br, bl = points
    if (tl[0] + bl[0]) >= (tr[0] + br[0]):
        return False
    if (tl[1] + tr[1]) >= (bl[1] + br[1]):
        return False
    contour = np.rint(points).astype(np.int32).reshape((-1, 1, 2))
    area = abs(float(cv2.contourArea(contour))) / max(width * height, 1)
    return bool(cv2.isContourConvex(contour)) and area > 0.002


def draw_detector(frame: np.ndarray, result) -> tuple[bool, Counter]:
    gate_proposal = False
    counts = Counter()
    if result.boxes is None:
        return gate_proposal, counts
    boxes = result.boxes.xyxy.cpu().numpy()
    confidences = result.boxes.conf.cpu().numpy()
    class_ids = result.boxes.cls.cpu().numpy().astype(int)
    for box, confidence, class_id in zip(boxes, confidences, class_ids):
        name = str(result.names[int(class_id)])
        counts[name] += 1
        gate_proposal |= name == 'gate'
        x1, y1, x2, y2 = (int(value) for value in box)
        colour = BOX_COLOURS.get(name, (230, 230, 230))
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
        cv2.putText(
            frame, f'{name} {float(confidence):.2f}',
            (x1, max(22, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX,
            0.58, colour, 2, cv2.LINE_AA)
    return gate_proposal, counts


def draw_gate_pose(
    frame: np.ndarray,
    result,
    keypoint_threshold: float,
) -> tuple[int, int]:
    detected = 0
    valid_count = 0
    if result.boxes is None or result.keypoints is None:
        return detected, valid_count
    box_confidences = result.boxes.conf.cpu().numpy()
    points_all = result.keypoints.xy.cpu().numpy()
    confidences = result.keypoints.conf
    point_scores_all = (
        confidences.cpu().numpy()
        if confidences is not None
        else np.ones((len(points_all), 4), dtype=np.float32)
    )
    height, width = frame.shape[:2]
    for box_confidence, points, point_scores in zip(
            box_confidences, points_all, point_scores_all):
        if len(points) != 4 or len(point_scores) != 4:
            continue
        detected += 1
        minimum = float(np.min(point_scores))
        valid = valid_quad(points, width, height) and minimum >= keypoint_threshold
        valid_count += int(valid)
        colour = (0, 220, 0) if valid else (0, 0, 255)
        polygon = np.rint(points).astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(frame, [polygon], True, colour, 3, cv2.LINE_AA)
        centre = np.mean(points, axis=0)
        cv2.circle(frame, tuple(np.rint(centre).astype(int)), 7, (255, 255, 255), 2)
        for index, (point, score) in enumerate(zip(points, point_scores)):
            xy = tuple(np.rint(point).astype(int))
            point_colour = POINT_COLOURS[index]
            cv2.circle(frame, xy, 7, point_colour, -1)
            cv2.putText(
                frame, f'{index + 1}:{float(score):.2f}',
                (xy[0] + 8, xy[1] - 8), cv2.FONT_HERSHEY_SIMPLEX,
                0.52, point_colour, 2, cv2.LINE_AA)
        x1, y1 = int(np.min(points[:, 0])), int(np.min(points[:, 1]))
        cv2.putText(
            frame,
            f'POSE box={float(box_confidence):.2f} min_kpt={minimum:.2f} '
            f'{"VALID" if valid else "REJECT"}',
            (max(10, x1), max(48, y1 - 30)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.60, colour, 2, cv2.LINE_AA)
    return detected, valid_count


def main() -> int:
    args = parse_args()
    paths = (args.video.resolve(), args.detector.resolve(), args.pose.resolve())
    for path in paths:
        if not path.is_file():
            raise SystemExit(f'文件不存在: {path}')

    capture = cv2.VideoCapture(str(paths[0]))
    if not capture.isOpened():
        raise SystemExit(f'无法打开视频: {paths[0]}')
    source_fps = float(capture.get(cv2.CAP_PROP_FPS) or 30.0)
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    start_frame = max(0, int(args.start_seconds * source_fps))
    capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(args.output.resolve()), cv2.VideoWriter_fourcc(*'mp4v'),
        source_fps, (width, height))
    if not writer.isOpened():
        raise SystemExit(f'无法创建输出视频: {args.output}')

    detector = YOLO(str(paths[1]))
    pose_model = YOLO(str(paths[2]))
    window = 'AUV dual model: detector + gate Pose | Q/ESC stop'
    if not args.no_show:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, min(width, 1280), min(height, 720))

    counts = Counter()
    processed = 0
    fps_ema = None
    print(
        f'VIDEO={paths[0]}\nDETECTOR={paths[1]}\nPOSE={paths[2]}\n'
        f'OUTPUT={args.output.resolve()}\nWINDOW={window}', flush=True)
    try:
        while True:
            started = time.perf_counter()
            ok, frame = capture.read()
            if not ok:
                break
            frame_index = int(capture.get(cv2.CAP_PROP_POS_FRAMES))
            detector_result = detector.predict(
                frame, imgsz=args.imgsz, conf=args.det_conf,
                device=args.device, verbose=False)[0]
            gate_proposal, frame_counts = draw_detector(frame, detector_result)
            counts.update(frame_counts)

            pose_detected = pose_valid = 0
            active_models = 'DETECTOR'
            if gate_proposal:
                active_models = 'DETECTOR -> GATE POSE'
                pose_result = pose_model.predict(
                    frame, imgsz=args.imgsz, conf=args.pose_conf,
                    device=args.device, verbose=False)[0]
                pose_detected, pose_valid = draw_gate_pose(
                    frame, pose_result, args.keypoint_conf)
                counts['pose_instances'] += pose_detected
                counts['valid_pose_instances'] += pose_valid

            elapsed = time.perf_counter() - started
            instant_fps = 1.0 / max(elapsed, 1e-6)
            fps_ema = instant_fps if fps_ema is None else 0.9 * fps_ema + 0.1 * instant_fps
            cv2.rectangle(frame, (0, 0), (width, 70), (20, 20, 20), -1)
            cv2.putText(
                frame, active_models, (16, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.72, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(
                frame,
                f'frame {frame_index}/{total} | processing {fps_ema:.1f} FPS | '
                f'pose valid {pose_valid}/{pose_detected}',
                (16, 58), cv2.FONT_HERSHEY_SIMPLEX,
                0.62, (255, 255, 255), 2, cv2.LINE_AA)
            writer.write(frame)
            processed += 1

            wait_ms = 1
            if not args.no_realtime:
                wait_ms = max(1, round(1000.0 / source_fps - elapsed * 1000.0))
            key = cv2.waitKey(wait_ms) & 0xFF if not args.no_show else 255
            if key in (ord('q'), ord('Q'), 27):
                print('STOPPED_BY_USER=1', flush=True)
                break
            if args.max_frames > 0 and processed >= args.max_frames:
                break
    finally:
        capture.release()
        writer.release()
        if not args.no_show:
            cv2.destroyAllWindows()

    print(f'PROCESSED_FRAMES={processed}', flush=True)
    print(f'COUNTS={dict(counts)}', flush=True)
    print(f'SAVED={args.output.resolve()}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
