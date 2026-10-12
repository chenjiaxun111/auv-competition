#!/usr/bin/python3
"""Atomically extract left-eye JPEG frames from a ZED SVO/SVO2 recording."""

from __future__ import annotations

import argparse
import csv
import shutil
import sys
from pathlib import Path
from typing import NoReturn

import cv2
import pyzed.sl as sl


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract ZED left-eye images for YOLO annotation and training."
    )
    parser.add_argument("input", type=Path, help="input .svo or .svo2 file")
    parser.add_argument("--output", required=True, type=Path, help="output directory")
    parser.add_argument(
        "--every-n",
        type=int,
        default=10,
        help="save one image for every N source frames (default: 10)",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
        help="JPEG quality from 1 to 100 (default: 95)",
    )
    return parser.parse_args()


def fail(message: str) -> NoReturn:
    print(f"FAILED: {message}", file=sys.stderr)
    raise SystemExit(1)


def main() -> int:
    args = parse_args()
    input_path = args.input.expanduser().resolve()
    output_dir = args.output.expanduser().resolve()
    partial_dir = output_dir.with_name(f"{output_dir.name}.partial")

    if not input_path.is_file():
        fail(f"input does not exist: {input_path}")
    if input_path.suffix.lower() not in {".svo", ".svo2"}:
        fail("input must use the .svo or .svo2 extension")
    if not 1 <= args.every_n <= 10000:
        fail("--every-n must be between 1 and 10000")
    if not 1 <= args.jpeg_quality <= 100:
        fail("--jpeg-quality must be between 1 and 100")

    completion_file = output_dir / ".complete"
    if output_dir.exists():
        if completion_file.is_file():
            image_count = len(list(output_dir.glob("frame_*.jpg")))
            print(f"REUSED={output_dir}")
            print(f"IMAGE_COUNT={image_count}")
            return 0
        fail(f"output exists without a completion marker: {output_dir}")

    # This path is owned exclusively by this exporter. Cleaning it makes a
    # power-loss retry safe without touching a completed dataset directory.
    if partial_dir.exists():
        shutil.rmtree(partial_dir)
    partial_dir.mkdir(parents=True)

    init = sl.InitParameters()
    init.set_from_svo_file(str(input_path))
    init.svo_real_time_mode = False
    init.depth_mode = sl.DEPTH_MODE.NONE

    camera = sl.Camera()
    opened = False
    completed = False
    try:
        status = camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            fail(f"ZED SDK could not open the SVO2: {status}")
        opened = True

        info = camera.get_camera_information().camera_configuration
        fps = float(info.fps or 30)
        resolution = info.resolution
        width, height = int(resolution.width), int(resolution.height)
        expected_frames = int(camera.get_svo_number_of_frames())
        end_of_svo = getattr(
            sl.ERROR_CODE,
            "END_OF_SVOFILE_REACHED",
            getattr(sl.ERROR_CODE, "END_OF_SVOFILE", None),
        )
        if end_of_svo is None:
            fail("installed ZED SDK exposes no end-of-SVO status code")

        image = sl.Mat()
        runtime = sl.RuntimeParameters()
        source_frame = 0
        image_count = 0
        manifest_path = partial_dir / "frames.csv"
        with manifest_path.open("w", newline="", encoding="utf-8") as stream:
            manifest = csv.writer(stream)
            manifest.writerow(["filename", "source_frame", "timestamp_ns"])
            while True:
                grab_status = camera.grab(runtime)
                if grab_status == end_of_svo:
                    break
                if grab_status != sl.ERROR_CODE.SUCCESS:
                    fail(f"SVO2 read failed at frame {source_frame}: {grab_status}")

                if source_frame % args.every_n == 0:
                    camera.retrieve_image(image, sl.VIEW.LEFT)
                    bgr = cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2BGR)
                    filename = f"frame_{source_frame:06d}.jpg"
                    output_image = partial_dir / filename
                    ok = cv2.imwrite(
                        str(output_image),
                        bgr,
                        [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality],
                    )
                    if not ok:
                        fail(f"could not write image: {output_image}")
                    timestamp_ns = camera.get_timestamp(
                        sl.TIME_REFERENCE.IMAGE
                    ).get_nanoseconds()
                    manifest.writerow([filename, source_frame, timestamp_ns])
                    image_count += 1

                source_frame += 1
                if source_frame % 300 == 0:
                    print(
                        f"Read {source_frame}/{expected_frames}; "
                        f"saved {image_count} images",
                        flush=True,
                    )

        (partial_dir / ".complete").write_text(
            "\n".join(
                [
                    f"source={input_path}",
                    f"source_frames={source_frame}",
                    f"image_count={image_count}",
                    f"every_n={args.every_n}",
                    f"jpeg_quality={args.jpeg_quality}",
                    f"fps={fps:g}",
                    f"resolution={width}x{height}",
                    "view=left",
                    "",
                ]
            ),
            encoding="utf-8",
        )
        partial_dir.replace(output_dir)
        completed = True

        print(f"OUTPUT={output_dir}")
        print(f"SOURCE_FRAMES={source_frame}")
        print(f"IMAGE_COUNT={image_count}")
        print(f"EVERY_N={args.every_n}")
        print(f"RESOLUTION={width}x{height}")
        return 0
    finally:
        if opened:
            camera.close()
        if not completed and partial_dir.exists():
            shutil.rmtree(partial_dir)


if __name__ == "__main__":
    raise SystemExit(main())
