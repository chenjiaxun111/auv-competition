#!/usr/bin/env python3
"""使用训练好的 YOLO 模型为 test1/test2 生成预标注。

默认不覆盖任何已经存在的 .txt 标签。自动标签仍需人工复核，尤其是蓝球。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

from ultralytics import YOLO


HERE = Path(__file__).resolve().parent
DEFAULT_MODEL = HERE / "training_runs" / "test3_yolo11n" / "weights" / "best.pt"
BATCHES = {
    "test1": HERE / "yolo_frames_n10_q95",
    "test2": HERE / "yolo_frames_n10_q95-2" / "yolo_frames_n10_q95",
}
NAMES = {0: "red_ball", 1: "gate", 2: "blue_ball"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batches", nargs="+", choices=sorted(BATCHES))
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--red-conf", type=float, default=0.35)
    parser.add_argument("--gate-conf", type=float, default=0.35)
    parser.add_argument(
        "--blue-conf",
        type=float,
        default=0.15,
        help="蓝球模型召回偏低，因此默认阈值低于另外两类",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="覆盖已有标签；默认关闭，通常不要使用",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(text)
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def process_batch(model: YOLO, batch_name: str, args: argparse.Namespace) -> dict:
    images_dir = BATCHES[batch_name].resolve()
    labels_dir = images_dir / "labels"
    images = sorted(images_dir.glob("*.jpg"))
    if not images:
        raise RuntimeError(f"{batch_name} 没有 JPG 图片: {images_dir}")

    existing = {path.stem for path in labels_dir.glob("*.txt")} if labels_dir.exists() else set()
    pending = images if args.overwrite else [image for image in images if image.stem not in existing]
    thresholds = {0: args.red_conf, 1: args.gate_conf, 2: args.blue_conf}
    min_conf = min(thresholds.values())

    print(
        f"{batch_name}: images={len(images)} existing={len(existing)} "
        f"to_process={len(pending)}"
    )
    stats: Counter[str] = Counter()
    confidences: dict[str, list[float]] = {name: [] for name in NAMES.values()}

    if pending:
        results = model.predict(
            source=[str(path) for path in pending],
            imgsz=args.imgsz,
            conf=min_conf,
            iou=0.7,
            device=args.device,
            batch=args.batch_size,
            stream=True,
            verbose=False,
        )
        for image, result in zip(pending, results, strict=True):
            output_lines: list[str] = []
            boxes = result.boxes
            if boxes is not None:
                for xywhn, class_tensor, conf_tensor in zip(
                    boxes.xywhn.cpu().tolist(),
                    boxes.cls.cpu().tolist(),
                    boxes.conf.cpu().tolist(),
                    strict=True,
                ):
                    class_id = int(class_tensor)
                    confidence = float(conf_tensor)
                    if class_id not in thresholds or confidence < thresholds[class_id]:
                        continue
                    x, y, width, height = xywhn
                    output_lines.append(
                        f"{class_id} {x:.6f} {y:.6f} {width:.6f} {height:.6f}"
                    )
                    stats[f"instances_{NAMES[class_id]}"] += 1
                    confidences[NAMES[class_id]].append(confidence)

            stats["processed"] += 1
            stats["positive_images" if output_lines else "negative_images"] += 1
            if not args.dry_run:
                atomic_write(
                    labels_dir / f"{image.stem}.txt",
                    "\n".join(output_lines) + ("\n" if output_lines else ""),
                )

    manifest = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "batch": batch_name,
        "images_dir": str(images_dir),
        "model": str(args.model.resolve()),
        "model_sha256": sha256(args.model.resolve()),
        "class_names": NAMES,
        "confidence_thresholds": {
            "red_ball": args.red_conf,
            "gate": args.gate_conf,
            "blue_ball": args.blue_conf,
        },
        "imgsz": args.imgsz,
        "existing_labels_preserved": len(existing) if not args.overwrite else 0,
        "dry_run": args.dry_run,
        "statistics": dict(stats),
        "mean_confidence": {
            name: (sum(values) / len(values) if values else None)
            for name, values in confidences.items()
        },
        "warning": "模型生成的是预标注，必须人工复核；蓝球优先检查漏标和误分类。",
    }
    if not args.dry_run:
        atomic_write(
            labels_dir / "autolabel_manifest.json",
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        )
    print(json.dumps(manifest["statistics"], ensure_ascii=False, sort_keys=True))
    return manifest


def main() -> int:
    args = parse_args()
    model_path = args.model.resolve()
    if not model_path.is_file():
        raise SystemExit(f"模型不存在: {model_path}")
    for name, value in {
        "red-conf": args.red_conf,
        "gate-conf": args.gate_conf,
        "blue-conf": args.blue_conf,
    }.items():
        if not 0.0 <= value <= 1.0:
            raise SystemExit(f"--{name} 必须在 [0, 1] 内")

    model = YOLO(str(model_path))
    for batch_name in args.batches:
        process_batch(model, batch_name, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
