#!/usr/bin/env python3
"""Generate conservative gate-pose prelabels for later human review."""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


HERE = Path(__file__).resolve().parent
DETECTION_ROOT = HERE / "all_yolo11_dataset"
MANUAL_ROOT = HERE / "gate_pose_annotations" / "labels" / "train"
DEFAULT_MODEL = HERE / "training_runs" / "gate_yolo11n_pose_v1" / "weights" / "best.pt"
DEFAULT_OUTPUT = HERE / "gate_pose_pseudo"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--box-conf", type=float, default=0.75)
    parser.add_argument("--keypoint-conf", type=float, default=0.70)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def has_gate_detection(path: Path) -> bool:
    return any(
        line.split() and line.split()[0] == "1"
        for line in path.read_text(encoding="utf-8").splitlines()
    )


def valid_geometry(points: np.ndarray, width: int, height: int) -> tuple[bool, str]:
    if points.shape != (4, 2) or not np.isfinite(points).all():
        return False, "invalid_points"
    if np.any(points[:, 0] < 0) or np.any(points[:, 0] >= width):
        return False, "outside_x"
    if np.any(points[:, 1] < 0) or np.any(points[:, 1] >= height):
        return False, "outside_y"
    lt, rt, rb, lb = points
    if (lt[0] + lb[0]) >= (rt[0] + rb[0]):
        return False, "left_right_order"
    if (lt[1] + rt[1]) >= (lb[1] + rb[1]):
        return False, "top_bottom_order"
    contour = points.astype(np.float32).reshape((-1, 1, 2))
    if not cv2.isContourConvex(contour.astype(np.int32)):
        return False, "non_convex"
    area = abs(cv2.contourArea(contour))
    if area / (width * height) < 0.01:
        return False, "too_small"
    edges = np.linalg.norm(points - np.roll(points, -1, axis=0), axis=1)
    if float(edges.min()) < 15:
        return False, "short_edge"
    return True, "accepted"


def pose_line(box: np.ndarray, points: np.ndarray, width: int, height: int) -> str:
    x1, y1, x2, y2 = box
    fields = [
        "0",
        f"{(x1 + x2) / (2 * width):.6f}",
        f"{(y1 + y2) / (2 * height):.6f}",
        f"{(x2 - x1) / width:.6f}",
        f"{(y2 - y1) / height:.6f}",
    ]
    for x, y in points:
        fields.extend((f"{x / width:.6f}", f"{y / height:.6f}", "2"))
    return " ".join(fields)


def main() -> int:
    args = parse_args()
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        raise SystemExit(f"模型不存在: {model_path}")
    output = args.output.expanduser().resolve()
    labels_dir = output / "labels" / "train"
    labels_dir.mkdir(parents=True, exist_ok=True)

    image_dir = DETECTION_ROOT / "images" / "train"
    detection_dir = DETECTION_ROOT / "labels" / "train"
    candidates: list[Path] = []
    for image_path in sorted(image_dir.glob("*.jpg")):
        detection_path = detection_dir / f"{image_path.stem}.txt"
        manual_path = MANUAL_ROOT / f"{image_path.stem}.txt"
        pseudo_path = labels_dir / f"{image_path.stem}.txt"
        if manual_path.is_file() or (pseudo_path.is_file() and not args.force):
            continue
        if detection_path.is_file() and has_gate_detection(detection_path):
            candidates.append(image_path)
    if not candidates:
        raise SystemExit("没有需要自动预标注的训练图片")

    model = YOLO(str(model_path))
    results = model.predict(
        source=[str(path) for path in candidates],
        imgsz=args.imgsz,
        conf=args.box_conf,
        device=args.device,
        stream=True,
        verbose=False,
    )
    stats: Counter[str] = Counter()
    manifest: list[list[str | int | float]] = []
    for image_path, result in zip(candidates, results):
        image = cv2.imread(str(image_path))
        height, width = image.shape[:2]
        accepted: list[str] = []
        reasons: list[str] = []
        if result.boxes is not None and result.keypoints is not None:
            boxes = result.boxes.xyxy.cpu().numpy()
            box_confidences = result.boxes.conf.cpu().numpy()
            points_all = result.keypoints.xy.cpu().numpy()
            keypoint_confidences = result.keypoints.conf
            keypoint_confidences = (
                keypoint_confidences.cpu().numpy()
                if keypoint_confidences is not None
                else np.ones((len(points_all), 4), dtype=np.float32)
            )
            for box, box_score, points, point_scores in zip(
                boxes, box_confidences, points_all, keypoint_confidences
            ):
                minimum_point_score = float(np.min(point_scores))
                if minimum_point_score < args.keypoint_conf:
                    reasons.append("low_keypoint_conf")
                    continue
                valid, reason = valid_geometry(points, width, height)
                if not valid:
                    reasons.append(reason)
                    continue
                accepted.append(pose_line(box, points, width, height))
                manifest.append(
                    [image_path.name, "accepted", float(box_score), minimum_point_score, reason]
                )
        if accepted:
            (labels_dir / f"{image_path.stem}.txt").write_text(
                "\n".join(accepted) + "\n", encoding="utf-8"
            )
            stats["accepted_images"] += 1
            stats["accepted_instances"] += len(accepted)
        else:
            reason = ",".join(sorted(set(reasons))) if reasons else "no_detection"
            manifest.append([image_path.name, "rejected", 0.0, 0.0, reason])
            stats["rejected_images"] += 1

    manifest_path = output / "manifest.csv"
    with manifest_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["image", "status", "box_conf", "min_keypoint_conf", "reason"])
        writer.writerows(manifest)
    print(f"CANDIDATES={len(candidates)}")
    for key in ("accepted_images", "accepted_instances", "rejected_images"):
        print(f"{key.upper()}={stats[key]}")
    print(f"PSEUDO_LABELS={labels_dir}")
    print(f"MANIFEST={manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
