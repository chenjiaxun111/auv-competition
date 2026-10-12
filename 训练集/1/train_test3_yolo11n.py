#!/usr/bin/env python3
"""用官方 YOLO11n 权重训练 test3，不继承旧比赛模型。"""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


HERE = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument(
        "--batch",
        type=float,
        default=-1,
        help="默认 -1：自动使用约 60%% GPU 显存；也可指定固定批量",
    )
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0", help="GPU 为 0；CPU 为 cpu")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--name", default="test3_yolo11n")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    data = HERE / "test3_yolo11_dataset" / "data.yaml"
    if not data.is_file():
        raise SystemExit(
            f"缺少数据配置 {data}\n"
            f"请先运行: python3 {HERE / 'prepare_test3_yolo11_dataset.py'}"
        )

    # 关键点：这里是官方通用 yolo11n.pt，不是 /模型/best(1).pt。
    # 本机无该文件时，Ultralytics 会自动下载官方权重。
    model = YOLO("yolo11n.pt")
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
        # 避免新版 Ultralytics 为 AMP 自检额外下载无关模型；仍使用 CUDA GPU。
        amp=False,
        patience=25,
        seed=2026,
        deterministic=True,
        plots=True,
        exist_ok=False,
    )


if __name__ == "__main__":
    main()
