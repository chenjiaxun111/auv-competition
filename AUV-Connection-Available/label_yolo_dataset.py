#!/usr/bin/env python3
"""Standalone OpenCV annotator adapted from the older AUV workspace tool.

An empty label file means that a human reviewed the image and confirmed that
it contains no target.  A missing label file means that the image has not been
reviewed yet.  Labels use standard YOLO ``class cx cy width height`` values.
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import cv2


DEFAULT_ROOT = (
    Path(__file__).resolve().parents[1]
    / "训练集"
    / "1"
    / "merged_yolo_dataset"
)

# Operator-friendly keys are intentionally independent from YOLO class IDs.
# Keep the saved model mapping fixed: red_ball=0, gate=1, blue_ball=2.
CLASS_HOTKEYS = {
    ord("1"): 2,  # blue_ball
    ord("2"): 0,  # red_ball
    ord("3"): 1,  # gate
}


@dataclass(frozen=True)
class Box:
    class_id: int
    x1: int
    y1: int
    x2: int
    y2: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=DEFAULT_ROOT,
        help="Dataset root containing images/ and classes.json.",
    )
    parser.add_argument(
        "--images-dir",
        type=Path,
        help="Optional image directory; defaults to <root>/images.",
    )
    parser.add_argument(
        "--labels-dir",
        type=Path,
        help="Optional label directory; defaults to <root>/labels.",
    )
    parser.add_argument(
        "--classes-file",
        type=Path,
        help="Optional classes JSON; defaults to <root>/classes.json.",
    )
    parser.add_argument(
        "--include-reviewed",
        action="store_true",
        help="Show all images, including images that already have a label file.",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Validate the dataset and print counts without opening a GUI.",
    )
    return parser.parse_args()


def load_classes(path: Path) -> list[str]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not data:
        raise ValueError(f"invalid class mapping: {path}")
    ids = sorted(int(key) for key in data)
    if ids != list(range(len(ids))):
        raise ValueError("class IDs must be continuous and start at zero")
    return [str(data[str(index)]) for index in ids]


def clip_box(
    class_id: int,
    coordinates: tuple[int, int, int, int],
    width: int,
    height: int,
) -> Box | None:
    x1, y1, x2, y2 = coordinates
    x1, x2 = sorted((x1, x2))
    y1, y2 = sorted((y1, y2))
    x1 = max(0, min(width - 1, x1))
    x2 = max(0, min(width - 1, x2))
    y1 = max(0, min(height - 1, y1))
    y2 = max(0, min(height - 1, y2))
    if x2 - x1 <= 5 or y2 - y1 <= 5:
        return None
    return Box(class_id, x1, y1, x2, y2)


def load_boxes(
    label_path: Path,
    width: int,
    height: int,
    class_count: int,
) -> list[Box]:
    boxes: list[Box] = []
    if not label_path.exists():
        return boxes
    for line_number, line in enumerate(
        label_path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"{label_path}:{line_number}: expected 5 fields")
        class_id = int(fields[0])
        if not 0 <= class_id < class_count:
            raise ValueError(f"{label_path}:{line_number}: invalid class {class_id}")
        cx, cy, box_w, box_h = map(float, fields[1:])
        pixel_box = clip_box(
            class_id,
            (
                round((cx - box_w / 2.0) * width),
                round((cy - box_h / 2.0) * height),
                round((cx + box_w / 2.0) * width),
                round((cy + box_h / 2.0) * height),
            ),
            width,
            height,
        )
        if pixel_box is None:
            raise ValueError(f"{label_path}:{line_number}: box is empty or too small")
        boxes.append(pixel_box)
    return boxes


def save_boxes(label_path: Path, boxes: list[Box], width: int, height: int) -> None:
    lines: list[str] = []
    for box in boxes:
        cx = (box.x1 + box.x2) / (2.0 * width)
        cy = (box.y1 + box.y2) / (2.0 * height)
        box_w = (box.x2 - box.x1) / width
        box_h = (box.y2 - box.y1) / height
        lines.append(
            f"{box.class_id} {cx:.6f} {cy:.6f} {box_w:.6f} {box_h:.6f}"
        )
    label_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = label_path.parent / f".{label_path.name}.tmp"
    temporary.write_text(("\n".join(lines) + "\n") if lines else "", encoding="utf-8")
    temporary.replace(label_path)


def color_for(class_id: int) -> tuple[int, int, int]:
    colors = ((0, 0, 255), (0, 255, 255), (255, 128, 0))
    return colors[class_id % len(colors)]


def audit_labels(
    images: list[Path], labels_dir: Path, classes: list[str]
) -> bool:
    """Validate existing labels and print a compact annotation summary."""
    image_by_stem = {image.stem: image for image in images}
    label_paths = sorted(labels_dir.glob("*.txt")) if labels_dir.is_dir() else []
    positive_images = 0
    negative_images = 0
    instances: Counter[int] = Counter()
    errors: list[str] = []

    for label_path in label_paths:
        image_path = image_by_stem.get(label_path.stem)
        if image_path is None:
            errors.append(f"orphan label without image: {label_path.name}")
            continue
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            errors.append(f"cannot decode image for label: {image_path.name}")
            continue
        height, width = image.shape[:2]
        try:
            boxes = load_boxes(label_path, width, height, len(classes))
        except (TypeError, ValueError) as error:
            errors.append(str(error))
            continue
        if boxes:
            positive_images += 1
            instances.update(box.class_id for box in boxes)
        else:
            negative_images += 1

    print(f"POSITIVE_IMAGES={positive_images}")
    print(f"NEGATIVE_IMAGES={negative_images}")
    for class_id, class_name in enumerate(classes):
        print(f"INSTANCES_{class_name.upper()}={instances[class_id]}")
    print(f"LABEL_ERRORS={len(errors)}")
    for error in errors[:20]:
        print(f"ERROR={error}")
    if len(errors) > 20:
        print(f"ERROR=... and {len(errors) - 20} more")
    return not errors


def main() -> int:
    args = parse_args()
    root = args.root.expanduser().resolve()
    images_dir = (
        args.images_dir.expanduser().resolve()
        if args.images_dir is not None
        else root / "images"
    )
    labels_dir = (
        args.labels_dir.expanduser().resolve()
        if args.labels_dir is not None
        else root / "labels"
    )
    classes_path = (
        args.classes_file.expanduser().resolve()
        if args.classes_file is not None
        else root / "classes.json"
    )

    if not images_dir.is_dir():
        raise SystemExit(f"图片目录不存在: {images_dir}")
    if not classes_path.is_file():
        raise SystemExit(f"类别文件不存在: {classes_path}")

    classes = load_classes(classes_path)
    all_images = sorted(images_dir.glob("*.jpg"))
    reviewed = sum((labels_dir / f"{image.stem}.txt").exists() for image in all_images)
    pending_count = len(all_images) - reviewed
    print(f"DATASET={root}")
    print(f"IMAGES_DIR={images_dir}")
    print(f"LABELS_DIR={labels_dir}")
    print(f"IMAGES={len(all_images)}")
    print(f"REVIEWED={reviewed}")
    print(f"PENDING={pending_count}")
    print("CLASSES=" + ", ".join(f"{i}:{name}" for i, name in enumerate(classes)))
    labels_ok = audit_labels(all_images, labels_dir, classes)
    if args.check_only:
        return 0 if labels_ok else 2

    images = all_images
    if not args.include_reviewed:
        images = [
            image
            for image in images
            if not (labels_dir / f"{image.stem}.txt").exists()
        ]
    if not images:
        raise SystemExit("没有待标注图像；可用 --include-reviewed 复查已标注图像。")

    index = 0
    selected_class = CLASS_HOTKEYS[ord("1")]
    drawing = False
    start = (-1, -1)
    pending: tuple[int, int, int, int] | None = None

    def mouse(event, x, y, _flags, _param) -> None:
        nonlocal drawing, start, pending
        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            start = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and drawing:
            pending = (start[0], start[1], x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False
            pending = (start[0], start[1], x, y)

    window = "AUV YOLO annotation"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1280, 720)
    cv2.setMouseCallback(window, mouse)

    while 0 <= index < len(images):
        image_path = images[index]
        image = cv2.imread(str(image_path))
        if image is None:
            print(f"WARNING: 无法读取 {image_path}")
            index += 1
            continue
        height, width = image.shape[:2]
        label_path = labels_dir / f"{image_path.stem}.txt"
        boxes = load_boxes(label_path, width, height, len(classes))
        pending = None

        while True:
            display = image.copy()
            for box in boxes:
                color = color_for(box.class_id)
                cv2.rectangle(display, (box.x1, box.y1), (box.x2, box.y2), color, 2)
                cv2.putText(
                    display,
                    classes[box.class_id],
                    (box.x1, max(18, box.y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                )
            if pending is not None:
                cv2.rectangle(display, pending[:2], pending[2:], color_for(selected_class), 2)
            cv2.putText(
                display,
                f"{index + 1}/{len(images)}  {image_path.name}",
                (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                1,
            )
            cv2.putText(
                display,
                f"class {selected_class}: {classes[selected_class]} | 1:blue 2:red 3:gate",
                (8, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color_for(selected_class),
                2,
            )
            cv2.putText(
                display,
                "drag | S:add | Enter/D:save+next | N:negative | K:skip | Z:undo | C:clear | A:prev | Q:quit",
                (8, height - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (255, 255, 255),
                1,
            )
            cv2.imshow(window, display)
            key = cv2.waitKey(40) & 0xFF

            if key in CLASS_HOTKEYS:
                selected_class = CLASS_HOTKEYS[key]
            elif key == ord("s") and pending is not None:
                box = clip_box(selected_class, pending, width, height)
                if box is not None:
                    boxes.append(box)
                pending = None
            elif key in (10, 13, ord("d")):
                if pending is not None:
                    box = clip_box(selected_class, pending, width, height)
                    if box is not None:
                        boxes.append(box)
                save_boxes(label_path, boxes, width, height)
                index += 1
                break
            elif key == ord("n"):
                save_boxes(label_path, [], width, height)
                index += 1
                break
            elif key == ord("k"):
                print(f"SKIPPED_UNREVIEWED={image_path.name}")
                index += 1
                break
            elif key == ord("z"):
                if pending is not None:
                    pending = None
                elif boxes:
                    boxes.pop()
            elif key == ord("a"):
                index = max(0, index - 1)
                break
            elif key == ord("c"):
                boxes = []
                pending = None
            elif key == ord("q"):
                cv2.destroyAllWindows()
                return 0

    cv2.destroyAllWindows()
    print("全部待标注图像已处理。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
