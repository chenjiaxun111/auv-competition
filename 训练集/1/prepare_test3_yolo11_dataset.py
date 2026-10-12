#!/usr/bin/env python3
"""将已完成标注的 test3 整理为 YOLO11 训练目录。

不复制大图片，而是创建指向原图和原标签的符号链接。验证集按连续 20 张
为一个时间块抽取，避免把每一张相邻视频帧随机分散到训练集和验证集。
"""

from __future__ import annotations

import argparse
import os
import shutil
from collections import Counter
from pathlib import Path


HERE = Path(__file__).resolve().parent
DEFAULT_SOURCE = (
    HERE / "yolo_frames_n10_q95-3" / "yolo_frames_n10_q95"
)
DEFAULT_OUTPUT = HERE / "test3_yolo11_dataset"
CLASS_NAMES = {0: "red_ball", 1: "gate", 2: "blue_ball"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--force",
        action="store_true",
        help="删除并重建已经存在的输出目录（只删除 --output）",
    )
    return parser.parse_args()


def read_label(path: Path) -> tuple[list[str], Counter[int]]:
    if not path.is_file():
        raise ValueError(f"缺少标签文件: {path}")
    lines = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    counts: Counter[int] = Counter()
    for line_no, line in enumerate(lines, 1):
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"{path}:{line_no} 应有 5 列，实际为 {len(fields)} 列")
        class_id = int(fields[0])
        values = [float(value) for value in fields[1:]]
        if class_id not in CLASS_NAMES:
            raise ValueError(f"{path}:{line_no} 非法类别 ID: {class_id}")
        if not all(0.0 <= value <= 1.0 for value in values):
            raise ValueError(f"{path}:{line_no} 坐标超出 [0, 1]")
        if values[2] <= 0.0 or values[3] <= 0.0:
            raise ValueError(f"{path}:{line_no} 宽高必须大于 0")
        counts[class_id] += 1
    return lines, counts


def main() -> int:
    args = parse_args()
    source = args.source.resolve()
    output = args.output.resolve()
    labels_source = source / "labels"

    images = sorted(source.glob("*.jpg"))
    if not images:
        raise SystemExit(f"没有找到 JPG 图片: {source}")

    # 先完整校验，确认不会在建到一半时才发现坏标签。
    labels: dict[str, Path] = {}
    for image in images:
        label = labels_source / f"{image.stem}.txt"
        read_label(label)
        labels[image.name] = label

    if output.exists():
        if not args.force:
            raise SystemExit(f"输出目录已存在: {output}\n如需重建，请加 --force")
        shutil.rmtree(output)

    for split in ("train", "val"):
        (output / "images" / split).mkdir(parents=True)
        (output / "labels" / split).mkdir(parents=True)

    stats = {
        "train": {"images": 0, "positive": 0, "instances": Counter()},
        "val": {"images": 0, "positive": 0, "instances": Counter()},
    }

    for index, image in enumerate(images):
        # 每 20 张为一块，每 5 块抽第一块进验证集：约 20% 验证数据。
        split = "val" if (index // 20) % 5 == 0 else "train"
        label = labels[image.name]
        lines, counts = read_label(label)
        os.symlink(image, output / "images" / split / image.name)
        os.symlink(label, output / "labels" / split / label.name)
        stats[split]["images"] += 1
        stats[split]["positive"] += bool(lines)
        stats[split]["instances"].update(counts)

    yaml_text = f"""# 由 prepare_test3_yolo11_dataset.py 自动生成
path: {output.as_posix()}
train: images/train
val: images/val

names:
  0: red_ball
  1: gate
  2: blue_ball
"""
    (output / "data.yaml").write_text(yaml_text, encoding="utf-8")

    print(f"DATASET={output}")
    for split in ("train", "val"):
        item = stats[split]
        print(
            f"{split.upper()}: images={item['images']} "
            f"positive={item['positive']} negative={item['images'] - item['positive']} "
            f"red_ball={item['instances'][0]} gate={item['instances'][1]} "
            f"blue_ball={item['instances'][2]}"
        )
    print(f"YAML={output / 'data.yaml'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
