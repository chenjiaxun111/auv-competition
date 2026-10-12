#!/usr/bin/env python3
"""Train a four-corner gate model from official YOLO11n-Pose weights."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


HERE = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=150)
    parser.add_argument("--batch", type=int, default=32)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--name", default="gate_yolo11n_pose")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    data = HERE / "gate_pose_yolo11_dataset" / "data.yaml"
    if not data.is_file():
        raise SystemExit("请先完成四角标注并运行 prepare_gate_pose_dataset.py")
    model = YOLO("yolo11n-pose.pt")
    model.train(
        data=str(data),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=str(HERE / "training_runs"),
        name=args.name,
        pretrained=True,
        amp=True,
        patience=35,
        seed=2026,
        deterministic=True,
        plots=True,
        exist_ok=False,
    )


if __name__ == "__main__":
    main()
