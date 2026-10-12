#!/usr/bin/env python3
"""使用三组已审核数据，从官方 YOLO11n 通用权重训练完整比赛模型。"""

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
    parser.add_argument("--name", default="all_yolo11n")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    data = HERE / "all_yolo11_dataset" / "data.yaml"
    if not data.is_file():
        raise SystemExit(
            f"缺少 {data}\n请先运行 prepare_all_yolo11_dataset.py"
        )

    # 从官方COCO通用权重开始，不继承之前的三类比赛模型。
    model = YOLO(str(HERE / "yolo11n.pt"))
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
        amp=False,
        patience=35,
        seed=2026,
        deterministic=True,
        plots=True,
        exist_ok=False,
    )


if __name__ == "__main__":
    main()
