#!/usr/bin/env python3
"""Annotate one gate box and four ordered corners for YOLO11-Pose.

The existing three-class detection dataset is read-only. Pose labels are saved
to gate_pose_annotations/labels/{train,val}. Corner order is always:
top-left, top-right, bottom-right, bottom-left.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path

import cv2


HERE = Path(__file__).resolve().parent
DETECTION_ROOT = HERE / "all_yolo11_dataset"
ANNOTATION_ROOT = HERE / "gate_pose_annotations"
PSEUDO_ROOT = HERE / "gate_pose_pseudo"
KEYPOINT_NAMES = ("left_top", "right_top", "right_bottom", "left_bottom")
COLORS = ((0, 255, 255), (0, 255, 0), (255, 0, 255), (255, 128, 0))


@dataclass
class PoseInstance:
    box: tuple[int, int, int, int]
    points: list[tuple[int, int]] = field(default_factory=list)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--split", choices=("train", "val"), default="train")
    parser.add_argument("--include-reviewed", action="store_true")
    parser.add_argument(
        "--include-skipped",
        action="store_true",
        help="Show images previously skipped with K so they can be reconsidered.",
    )
    parser.add_argument(
        "--review-pseudo",
        action="store_true",
        help="Review model-generated labels and save accepted labels as manual annotations.",
    )
    parser.add_argument("--check-only", action="store_true")
    return parser.parse_args()


def yolo_box_to_pixels(
    values: list[float], width: int, height: int
) -> tuple[int, int, int, int]:
    cx, cy, box_w, box_h = values
    return (
        round((cx - box_w / 2) * width),
        round((cy - box_h / 2) * height),
        round((cx + box_w / 2) * width),
        round((cy + box_h / 2) * height),
    )


def load_gate_seeds(path: Path, width: int, height: int) -> list[PoseInstance]:
    instances: list[PoseInstance] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        fields = line.split()
        if not fields:
            continue
        if len(fields) != 5:
            raise ValueError(f"{path}:{line_number}: detection label must have 5 fields")
        if int(fields[0]) != 1:  # Existing detector mapping: gate=1.
            continue
        box = yolo_box_to_pixels(list(map(float, fields[1:])), width, height)
        instances.append(PoseInstance(box=box))
    return instances


def load_pose_labels(path: Path, width: int, height: int) -> list[PoseInstance]:
    instances: list[PoseInstance] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        fields = line.split()
        if len(fields) != 17:
            raise ValueError(f"{path}:{line_number}: pose label must have 17 fields")
        if int(fields[0]) != 0:
            raise ValueError(f"{path}:{line_number}: gate pose class must be 0")
        box = yolo_box_to_pixels(list(map(float, fields[1:5])), width, height)
        raw_points = list(map(float, fields[5:]))
        points: list[tuple[int, int]] = []
        for index in range(0, 12, 3):
            x, y, visibility = raw_points[index : index + 3]
            if visibility <= 0:
                raise ValueError(f"{path}:{line_number}: invisible corners are not used")
            points.append((round(x * width), round(y * height)))
        instances.append(PoseInstance(box=box, points=points))
    return instances


def box_from_points(points: list[tuple[int, int]], width: int, height: int) -> tuple[int, int, int, int]:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    padding = max(4, round(0.015 * max(width, height)))
    return (
        max(0, min(xs) - padding),
        max(0, min(ys) - padding),
        min(width - 1, max(xs) + padding),
        min(height - 1, max(ys) + padding),
    )


def save_pose_labels(path: Path, instances: list[PoseInstance], width: int, height: int) -> None:
    lines: list[str] = []
    for instance in instances:
        if len(instance.points) != 4:
            raise ValueError("each saved gate must have four corners")
        x1, y1, x2, y2 = box_from_points(instance.points, width, height)
        cx = (x1 + x2) / (2 * width)
        cy = (y1 + y2) / (2 * height)
        box_w = (x2 - x1) / width
        box_h = (y2 - y1) / height
        fields = ["0", f"{cx:.6f}", f"{cy:.6f}", f"{box_w:.6f}", f"{box_h:.6f}"]
        for x, y in instance.points:
            fields.extend((f"{x / width:.6f}", f"{y / height:.6f}", "2"))
        lines.append(" ".join(fields))
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.parent / f".{path.name}.tmp"
    temporary.write_text("\n".join(lines) + "\n", encoding="utf-8")
    temporary.replace(path)


def draw_instance(image, instance: PoseInstance, selected: bool) -> None:
    x1, y1, x2, y2 = instance.box
    color = (255, 255, 255) if selected else (100, 100, 100)
    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2 if selected else 1)
    for index, point in enumerate(instance.points):
        cv2.circle(image, point, 7, COLORS[index], -1)
        cv2.putText(
            image,
            str(index + 1),
            (point[0] + 8, point[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            COLORS[index],
            2,
        )
    if len(instance.points) > 1:
        for start, end in zip(instance.points, instance.points[1:]):
            cv2.line(image, start, end, (0, 255, 255), 2)
    if len(instance.points) == 4:
        cv2.line(image, instance.points[-1], instance.points[0], (0, 255, 255), 2)


def audit(split: str) -> bool:
    image_dir = DETECTION_ROOT / "images" / split
    labels_dir = ANNOTATION_ROOT / "labels" / split
    reviewed = 0
    instances = 0
    errors: list[str] = []
    for label_path in sorted(labels_dir.glob("*.txt")) if labels_dir.is_dir() else []:
        image_path = image_dir / f"{label_path.stem}.jpg"
        image = cv2.imread(str(image_path))
        if image is None:
            errors.append(f"missing image: {image_path}")
            continue
        try:
            poses = load_pose_labels(label_path, image.shape[1], image.shape[0])
        except ValueError as error:
            errors.append(str(error))
            continue
        reviewed += 1
        instances += len(poses)
    print(f"SPLIT={split}")
    print(f"REVIEWED_IMAGES={reviewed}")
    print(f"POSE_INSTANCES={instances}")
    print(f"LABEL_ERRORS={len(errors)}")
    for error in errors[:20]:
        print(f"ERROR={error}")
    return not errors


def main() -> int:
    args = parse_args()
    audit_ok = audit(args.split)
    if args.check_only:
        return 0 if audit_ok else 2

    image_dir = DETECTION_ROOT / "images" / args.split
    detection_labels = DETECTION_ROOT / "labels" / args.split
    pose_labels = ANNOTATION_ROOT / "labels" / args.split
    skipped_path = ANNOTATION_ROOT / "skipped" / f"{args.split}.txt"
    skipped = set()
    if skipped_path.is_file():
        skipped = {
            line.strip()
            for line in skipped_path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        }
    candidates: list[Path] = []
    for image_path in sorted(image_dir.glob("*.jpg")):
        detection_path = detection_labels / f"{image_path.stem}.txt"
        if not detection_path.is_file():
            continue
        has_gate = any(
            line.split() and line.split()[0] == "1"
            for line in detection_path.read_text(encoding="utf-8").splitlines()
        )
        reviewed = (pose_labels / f"{image_path.stem}.txt").is_file()
        was_skipped = image_path.name in skipped
        pseudo_path = PSEUDO_ROOT / "labels" / args.split / f"{image_path.stem}.txt"
        source_available = pseudo_path.is_file() if args.review_pseudo else has_gate
        if (
            source_available
            and (args.include_reviewed or not reviewed)
            and (args.include_skipped or not was_skipped)
        ):
            candidates.append(image_path)
    if not candidates:
        raise SystemExit("没有待标注门框；需要复查时添加 --include-reviewed")

    index = 0
    selected = 0
    window = "AUV gate pose: LT -> RT -> RB -> LB"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1280, 720)
    click: tuple[int, int] | None = None
    status_message = ""

    def mouse(event, x, y, _flags, _param) -> None:
        nonlocal click
        if event == cv2.EVENT_LBUTTONDOWN:
            click = (x, y)

    cv2.setMouseCallback(window, mouse)

    while 0 <= index < len(candidates):
        image_path = candidates[index]
        image = cv2.imread(str(image_path))
        if image is None:
            index += 1
            continue
        height, width = image.shape[:2]
        detection_path = detection_labels / f"{image_path.stem}.txt"
        pose_path = pose_labels / f"{image_path.stem}.txt"
        if pose_path.is_file():
            instances = load_pose_labels(pose_path, width, height)
        elif args.review_pseudo:
            pseudo_path = PSEUDO_ROOT / "labels" / args.split / f"{image_path.stem}.txt"
            instances = load_pose_labels(pseudo_path, width, height)
        else:
            instances = load_gate_seeds(detection_path, width, height)
        selected = min(selected, max(0, len(instances) - 1))
        click = None
        status_message = ""

        while True:
            if click is not None and instances:
                if len(instances[selected].points) < 4:
                    instances[selected].points.append(click)
                    if len(instances[selected].points) == 4:
                        instances[selected].box = box_from_points(
                            instances[selected].points, width, height
                        )
                        # Move directly to the next unfinished candidate. This
                        # avoids making the operator press Tab after every gate.
                        for offset in range(1, len(instances) + 1):
                            candidate = (selected + offset) % len(instances)
                            if len(instances[candidate].points) < 4:
                                selected = candidate
                                status_message = (
                                    "Next candidate selected; press X if it is a duplicate"
                                )
                                break
                click = None

            display = image.copy()
            for instance_index, instance in enumerate(instances):
                draw_instance(display, instance, instance_index == selected)
            next_name = (
                KEYPOINT_NAMES[len(instances[selected].points)]
                if instances and len(instances[selected].points) < 4
                else "complete"
            )
            cv2.putText(
                display,
                f"{index + 1}/{len(candidates)} {image_path.name}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )
            cv2.putText(
                display,
                f"gate {selected + 1}/{len(instances)} next={next_name}",
                (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2,
            )
            cv2.putText(
                display,
                "click:1LT 2RT 3RB 4LB | TAB:gate Z:undo X:delete K:skip ENTER/D/SPACE:save A:prev Q:quit",
                (10, height - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (255, 255, 255), 1,
            )
            if status_message:
                cv2.putText(
                    display,
                    status_message,
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 255), 2,
                )
            cv2.imshow(window, display)
            key = cv2.waitKey(30) & 0xFF
            if key == 9 and instances:  # Tab
                selected = (selected + 1) % len(instances)
            elif key == ord("z") and instances and instances[selected].points:
                instances[selected].points.pop()
            elif key == ord("x") and instances:
                instances.pop(selected)
                selected = min(selected, max(0, len(instances) - 1))
            elif key in (10, 13, 32, ord("d")):
                if not instances or any(len(instance.points) != 4 for instance in instances):
                    status_message = "NOT SAVED: mark all 4 corners, or press K to skip"
                    print("NOT_SAVED: 每个保留的门框都必须有4个角；不完整图片按K跳过")
                    continue
                save_pose_labels(pose_path, instances, width, height)
                if image_path.name in skipped:
                    skipped.remove(image_path.name)
                    skipped_path.parent.mkdir(parents=True, exist_ok=True)
                    skipped_path.write_text(
                        "\n".join(sorted(skipped)) + ("\n" if skipped else ""),
                        encoding="utf-8",
                    )
                index += 1
                selected = 0
                break
            elif key == ord("k"):
                print(f"SKIPPED={image_path.name}")
                skipped.add(image_path.name)
                skipped_path.parent.mkdir(parents=True, exist_ok=True)
                skipped_path.write_text(
                    "\n".join(sorted(skipped)) + "\n", encoding="utf-8"
                )
                index += 1
                selected = 0
                break
            elif key == ord("a"):
                index = max(0, index - 1)
                selected = 0
                break
            elif key == ord("q"):
                cv2.destroyAllWindows()
                return 0

    cv2.destroyAllWindows()
    print("当前分组的门框关键点标注已完成。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
