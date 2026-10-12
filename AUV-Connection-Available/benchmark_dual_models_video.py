#!/usr/bin/env python3
"""Benchmark the AUV detector and gate-pose models on a recorded MP4.

This tool imports neither ROS nor the NanoPi bridge. It performs offline,
read-only inference and writes one JSON report.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import statistics
import time

import cv2
import numpy as np
import torch
from ultralytics import YOLO


MODES = ('detector', 'pose', 'coordinator', 'worst_case')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--detector', type=Path, required=True)
    parser.add_argument('--pose', type=Path, required=True)
    parser.add_argument('--mode', choices=MODES, required=True)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--start-seconds', type=float, default=0.0)
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--stride', type=int, default=1)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--det-conf', type=float, default=0.45)
    parser.add_argument('--pose-conf', type=float, default=0.50)
    parser.add_argument('--keypoint-conf', type=float, default=0.50)
    parser.add_argument('--device', default='0')
    parser.add_argument('--half', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--warmup-runs', type=int, default=3)
    return parser.parse_args()


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def latency_summary(values: list[float]) -> dict[str, float | int | None]:
    return {
        'runs': len(values),
        'mean_ms': None if not values else float(statistics.fmean(values)),
        'p50_ms': percentile(values, 50),
        'p95_ms': percentile(values, 95),
        'maximum_ms': None if not values else float(max(values)),
    }


def valid_gate_pose(result, minimum_confidence: float) -> tuple[int, int]:
    if result.boxes is None or result.keypoints is None:
        return 0, 0
    points_all = result.keypoints.xy.cpu().numpy()
    confidences = result.keypoints.conf
    scores_all = (
        confidences.cpu().numpy()
        if confidences is not None
        else np.ones((len(points_all), 4), dtype=np.float32)
    )
    detected = 0
    valid = 0
    for points, scores in zip(points_all, scores_all):
        if len(points) != 4 or len(scores) != 4:
            continue
        detected += 1
        finite = np.isfinite(points).all() and np.isfinite(scores).all()
        valid += int(finite and float(np.min(scores)) >= minimum_confidence)
    return detected, valid


def predict(model, frame, confidence, args):
    if torch.cuda.is_available():
        torch.cuda.synchronize()
    started = time.perf_counter()
    result = model.predict(
        source=frame,
        conf=confidence,
        imgsz=args.imgsz,
        device=args.device,
        half=args.half,
        verbose=False,
    )[0]
    if torch.cuda.is_available():
        torch.cuda.synchronize()
    return result, (time.perf_counter() - started) * 1000.0


def main() -> int:
    args = parse_args()
    for path in (args.video, args.detector, args.pose):
        if not path.is_file():
            raise SystemExit(f'file does not exist: {path}')
    if args.stride <= 0 or args.imgsz <= 0 or args.max_frames < 0:
        raise SystemExit('stride/imgsz must be positive and max-frames non-negative')
    if str(args.device) != 'cpu' and not torch.cuda.is_available():
        raise SystemExit('CUDA device requested but torch.cuda.is_available() is false')

    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise SystemExit(f'cannot open video: {args.video}')
    source_fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
    total_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    start_frame = max(0, int(round(args.start_seconds * source_fps)))
    capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
    ok, warmup_frame = capture.read()
    if not ok:
        raise SystemExit('start position is beyond the end of the video')

    detector = YOLO(str(args.detector))
    pose_model = YOLO(str(args.pose))
    for _ in range(args.warmup_runs):
        if args.mode in ('detector', 'coordinator', 'worst_case'):
            predict(detector, warmup_frame, args.det_conf, args)
        if args.mode in ('pose', 'worst_case'):
            predict(pose_model, warmup_frame, args.pose_conf, args)
    capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    detector_ms: list[float] = []
    pose_ms: list[float] = []
    frame_ms: list[float] = []
    class_instances: Counter[str] = Counter()
    class_frames: Counter[str] = Counter()
    pose_instances = 0
    valid_pose_instances = 0
    pose_frames = 0
    valid_pose_frames = 0
    gate_proposal_frames = 0
    frames_read = 0
    frames_processed = 0
    benchmark_started = time.perf_counter()

    while True:
        ok, frame = capture.read()
        if not ok:
            break
        source_index = start_frame + frames_read
        frames_read += 1
        if (source_index - start_frame) % args.stride:
            continue

        frame_started = time.perf_counter()
        gate_proposal = False
        if args.mode in ('detector', 'coordinator', 'worst_case'):
            result, elapsed = predict(detector, frame, args.det_conf, args)
            detector_ms.append(elapsed)
            names_this_frame: set[str] = set()
            if result.boxes is not None:
                class_ids = result.boxes.cls.cpu().numpy().astype(int)
                for class_id in class_ids:
                    name = str(result.names[int(class_id)])
                    class_instances[name] += 1
                    names_this_frame.add(name)
                    gate_proposal |= name == 'gate'
            class_frames.update(names_this_frame)
            gate_proposal_frames += int(gate_proposal)

        run_pose = (
            args.mode in ('pose', 'worst_case')
            or (args.mode == 'coordinator' and gate_proposal)
        )
        if run_pose:
            result, elapsed = predict(pose_model, frame, args.pose_conf, args)
            pose_ms.append(elapsed)
            detected, valid = valid_gate_pose(result, args.keypoint_conf)
            pose_instances += detected
            valid_pose_instances += valid
            pose_frames += int(detected > 0)
            valid_pose_frames += int(valid > 0)

        frame_ms.append((time.perf_counter() - frame_started) * 1000.0)
        frames_processed += 1
        if frames_processed % 300 == 0:
            print(f'progress={frames_processed}', flush=True)
        if args.max_frames and frames_processed >= args.max_frames:
            break

    wall_s = time.perf_counter() - benchmark_started
    capture.release()
    frame_latency = latency_summary(frame_ms)
    mean_frame_ms = frame_latency['mean_ms']
    p95_frame_ms = frame_latency['p95_ms']
    report = {
        'verification_level': 'L4 hardware powered, offline video, no actuation',
        'video': {
            'path': str(args.video.resolve()),
            'width': width,
            'height': height,
            'source_fps': source_fps,
            'total_frames': total_frames,
            'duration_s': total_frames / source_fps if source_fps else None,
            'start_seconds': args.start_seconds,
            'stride': args.stride,
            'frames_read': frames_read,
            'frames_processed': frames_processed,
        },
        'runtime': {
            'torch': torch.__version__,
            'torchvision_cuda': torch.version.cuda,
            'cuda_available': torch.cuda.is_available(),
            'cuda_device': (
                torch.cuda.get_device_name(0) if torch.cuda.is_available() else None),
            'device_argument': args.device,
            'half': args.half,
            'imgsz': args.imgsz,
        },
        'mode': args.mode,
        'thresholds': {
            'detector_confidence': args.det_conf,
            'pose_confidence': args.pose_conf,
            'keypoint_confidence': args.keypoint_conf,
        },
        'detector_latency': latency_summary(detector_ms),
        'pose_latency': latency_summary(pose_ms),
        'processed_frame_latency': frame_latency,
        'model_capacity_fps_from_mean': (
            None if not mean_frame_ms else 1000.0 / mean_frame_ms),
        'model_capacity_fps_from_p95': (
            None if not p95_frame_ms else 1000.0 / p95_frame_ms),
        'wall_processing_fps': frames_processed / max(wall_s, 1e-9),
        'meets_configured_15hz_by_mean': bool(
            mean_frame_ms is not None and mean_frame_ms <= 1000.0 / 15.0),
        'meets_configured_15hz_by_p95': bool(
            p95_frame_ms is not None and p95_frame_ms <= 1000.0 / 15.0),
        'detections': {
            'class_instances': dict(class_instances),
            'class_frames': dict(class_frames),
            'gate_proposal_frames': gate_proposal_frames,
            'pose_runs': len(pose_ms),
            'pose_instances': pose_instances,
            'valid_pose_instances': valid_pose_instances,
            'pose_frames': pose_frames,
            'valid_pose_frames': valid_pose_frames,
        },
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    print(f'report={args.report.resolve()}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
