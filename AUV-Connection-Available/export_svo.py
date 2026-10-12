#!/usr/bin/python3
"""Export the left image stream of a ZED SVO2 to MP4 and/or JPEG frames."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import NoReturn

import cv2
import pyzed.sl as sl


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export a ZED SVO2 left-eye stream without computing depth."
    )
    parser.add_argument("input", type=Path, help="input .svo or .svo2 file")
    parser.add_argument(
        "--video",
        type=Path,
        help="output MP4 path (default: preview.mp4 beside the SVO2)",
    )
    parser.add_argument(
        "--images-dir",
        type=Path,
        help="optionally export JPEG training frames to this directory",
    )
    parser.add_argument(
        "--every-n",
        type=int,
        default=10,
        help="export one JPEG every N source frames (default: 10)",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
        help="JPEG quality from 1 to 100 (default: 95)",
    )
    parser.add_argument(
        "--force", action="store_true", help="overwrite existing outputs"
    )
    return parser.parse_args()


def fail(message: str) -> NoReturn:
    print(f"FAILED: {message}", file=sys.stderr)
    raise SystemExit(1)


def main() -> int:
    args = parse_args()
    input_path = args.input.expanduser().resolve()
    video_path = (
        args.video.expanduser().resolve()
        if args.video
        else input_path.with_name("preview.mp4")
    )
    images_dir = args.images_dir.expanduser().resolve() if args.images_dir else None
    completion_marker = video_path.with_name(f"{video_path.name}.complete")

    if not input_path.is_file():
        fail(f"input does not exist: {input_path}")
    if input_path.suffix.lower() not in {".svo", ".svo2"}:
        fail("input must use the .svo or .svo2 extension")
    if args.every_n < 1:
        fail("--every-n must be at least 1")
    if not 1 <= args.jpeg_quality <= 100:
        fail("--jpeg-quality must be between 1 and 100")
    if video_path.exists() and not args.force:
        fail(f"video already exists (use --force): {video_path}")
    if images_dir and images_dir.exists() and any(images_dir.iterdir()) and not args.force:
        fail(f"images directory is not empty (use --force): {images_dir}")

    init = sl.InitParameters()
    init.set_from_svo_file(str(input_path))
    init.svo_real_time_mode = False
    init.depth_mode = sl.DEPTH_MODE.NONE

    camera = sl.Camera()
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        fail(f"ZED SDK could not open the SVO2: {status}")

    partial_video = video_path.with_name(f"{video_path.stem}.partial{video_path.suffix}")
    partial_video.unlink(missing_ok=True)
    completion_marker.unlink(missing_ok=True)
    writer = None
    video_complete = False
    manifest = None
    manifest_file = None
    try:
        info = camera.get_camera_information().camera_configuration
        fps = float(info.fps or 30)
        resolution = info.resolution
        width, height = int(resolution.width), int(resolution.height)
        total_frames = int(camera.get_svo_number_of_frames())

        video_path.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(partial_video),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (width, height),
        )
        if not writer.isOpened():
            fail("OpenCV could not create an MP4 using the mp4v codec")

        if images_dir:
            images_dir.mkdir(parents=True, exist_ok=True)
            manifest_file = (images_dir / "frames.csv").open(
                "w", newline="", encoding="utf-8"
            )
            manifest = csv.writer(manifest_file)
            manifest.writerow(["filename", "source_frame", "timestamp_ns"])

        image = sl.Mat()
        runtime = sl.RuntimeParameters()
        end_of_svo = getattr(
            sl.ERROR_CODE,
            "END_OF_SVOFILE_REACHED",
            getattr(sl.ERROR_CODE, "END_OF_SVOFILE", None),
        )
        if end_of_svo is None:
            fail("installed ZED SDK exposes no end-of-SVO status code")
        frame_index = 0
        image_count = 0
        while True:
            grab_status = camera.grab(runtime)
            if grab_status == end_of_svo:
                break
            if grab_status != sl.ERROR_CODE.SUCCESS:
                fail(f"SVO2 read failed at frame {frame_index}: {grab_status}")

            camera.retrieve_image(image, sl.VIEW.LEFT)
            bgra = image.get_data()
            bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
            writer.write(bgr)

            if images_dir and frame_index % args.every_n == 0:
                filename = f"frame_{frame_index:06d}.jpg"
                output_image = images_dir / filename
                ok = cv2.imwrite(
                    str(output_image),
                    bgr,
                    [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality],
                )
                if not ok:
                    fail(f"could not write image: {output_image}")
                timestamp_ns = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
                manifest.writerow([filename, frame_index, timestamp_ns])
                image_count += 1

            frame_index += 1
            if frame_index % 300 == 0:
                print(f"Exported {frame_index}/{total_frames} frames", flush=True)

        writer.release()
        writer = None
        partial_video.replace(video_path)
        marker_tmp = completion_marker.with_name(f"{completion_marker.name}.tmp")
        marker_tmp.write_text(
            f"frames={frame_index}\nfps={fps:g}\nresolution={width}x{height}\n",
            encoding="utf-8",
        )
        marker_tmp.replace(completion_marker)
        video_complete = True

        print(f"VIDEO={video_path}")
        print(f"COMPLETE={completion_marker}")
        print(f"FRAMES={frame_index}")
        print(f"FPS={fps:g}")
        print(f"RESOLUTION={width}x{height}")
        if images_dir:
            print(f"IMAGES={images_dir}")
            print(f"IMAGE_COUNT={image_count}")
        return 0
    finally:
        if writer is not None:
            writer.release()
        if not video_complete:
            partial_video.unlink(missing_ok=True)
        if manifest_file is not None:
            manifest_file.close()
        camera.close()


if __name__ == "__main__":
    raise SystemExit(main())
