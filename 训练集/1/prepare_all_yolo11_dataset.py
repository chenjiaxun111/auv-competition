#!/usr/bin/env python3
"""合并 test1/test2/test3，并按连续时间块生成 YOLO11 训练集。"""

from __future__ import annotations

import argparse
import csv
import os
import shutil
from collections import Counter
from pathlib import Path


HERE = Path(__file__).resolve().parent
SOURCES = {
    "test1": HERE / "yolo_frames_n10_q95",
    "test2": HERE / "yolo_frames_n10_q95-2" / "yolo_frames_n10_q95",
    "test3": HERE / "yolo_frames_n10_q95-3" / "yolo_frames_n10_q95",
}
DEFAULT_OUTPUT = HERE / "all_yolo11_dataset"
CLASS_NAMES = {0: "red_ball", 1: "gate", 2: "blue_ball"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--force", action="store_true", help="重建输出目录（不修改三组原始数据）"
    )
    return parser.parse_args()


def validated_unique_lines(path: Path) -> tuple[list[str], Counter[int], int]:
    if not path.is_file():
        raise ValueError(f"缺少标签: {path}")
    raw = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    lines = list(dict.fromkeys(raw))
    counts: Counter[int] = Counter()
    for line_number, line in enumerate(lines, 1):
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"{path}:{line_number} 应为5列")
        class_id = int(fields[0])
        values = [float(value) for value in fields[1:]]
        if class_id not in CLASS_NAMES:
            raise ValueError(f"{path}:{line_number} 非法类别 {class_id}")
        if not all(0.0 <= value <= 1.0 for value in values):
            raise ValueError(f"{path}:{line_number} 坐标越界")
        if values[2] <= 0.0 or values[3] <= 0.0:
            raise ValueError(f"{path}:{line_number} 宽高必须大于0")
        counts[class_id] += 1
    return lines, counts, len(raw) - len(lines)


def main() -> int:
    args = parse_args()
    output = args.output.resolve()

    # 先校验全部输入，避免输出目录生成一半后才报错。
    records: list[tuple[str, int, Path, Path, list[str], Counter[int], int]] = []
    for source_name, source in SOURCES.items():
        images = sorted(source.glob("*.jpg"))
        if not images:
            raise SystemExit(f"{source_name} 没有图片: {source}")
        for index, image in enumerate(images):
            label = source / "labels" / f"{image.stem}.txt"
            lines, counts, duplicates = validated_unique_lines(label)
            records.append((source_name, index, image, label, lines, counts, duplicates))

    if output.exists():
        if not args.force:
            raise SystemExit(f"输出目录已存在: {output}\n重建请加 --force")
        shutil.rmtree(output)
    for split in ("train", "val"):
        (output / "images" / split).mkdir(parents=True)
        (output / "labels" / split).mkdir(parents=True)

    stats = {
        split: {"images": 0, "positive": 0, "instances": Counter()}
        for split in ("train", "val")
    }
    duplicate_count = 0
    manifest_rows: list[list[str | int]] = []

    for source_name, index, image, label, lines, counts, duplicates in records:
        # 每20张相邻帧作为一个不可拆分时间块，每5块取第一块做验证。
        split = "val" if (index // 20) % 5 == 0 else "train"
        output_stem = f"{source_name}_{image.stem}"
        output_image = output / "images" / split / f"{output_stem}.jpg"
        output_label = output / "labels" / split / f"{output_stem}.txt"
        os.symlink(image.resolve(), output_image)
        output_label.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

        stats[split]["images"] += 1
        stats[split]["positive"] += bool(lines)
        stats[split]["instances"].update(counts)
        duplicate_count += duplicates
        manifest_rows.append(
            [split, source_name, image.name, output_image.name, len(lines), duplicates]
        )

    (output / "data.yaml").write_text(
        f"""# 由 prepare_all_yolo11_dataset.py 自动生成
path: {output.as_posix()}
train: images/train
val: images/val

names:
  0: red_ball
  1: gate
  2: blue_ball
""",
        encoding="utf-8",
    )
    with (output / "split_manifest.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["split", "source", "source_image", "output_image", "instances", "duplicates_removed"]
        )
        writer.writerows(manifest_rows)

    print(f"DATASET={output}")
    print(f"SOURCE_IMAGES={len(records)}")
    print(f"DUPLICATE_BOXES_REMOVED={duplicate_count}")
    for split in ("train", "val"):
        item = stats[split]
        print(
            f"{split.upper()}: images={item['images']} positive={item['positive']} "
            f"negative={item['images'] - item['positive']} red_ball={item['instances'][0]} "
            f"gate={item['instances'][1]} blue_ball={item['instances'][2]}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
