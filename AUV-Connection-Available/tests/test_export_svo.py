#!/usr/bin/env python3
"""Computer-only tests for export_svo.py using a simulated ZED playback API."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path

import cv2
import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "export_svo.py"


def install_fake_pyzed(frame_count: int = 12, fail_at: int | None = None) -> None:
    sl = types.ModuleType("pyzed.sl")

    class ErrorCode:
        SUCCESS = 0
        END_OF_SVOFILE_REACHED = 1

    class InitParameters:
        def __init__(self) -> None:
            self.svo_real_time_mode = True
            self.depth_mode = None

        def set_from_svo_file(self, path: str) -> None:
            self.path = path

    class Mat:
        def __init__(self) -> None:
            self.data = None

        def get_data(self):
            return self.data

    class Timestamp:
        def __init__(self, value: int) -> None:
            self.value = value

        def get_nanoseconds(self) -> int:
            return self.value

    class Camera:
        def __init__(self) -> None:
            self.index = 0
            self.current = None
            self.frames = []
            for index in range(frame_count):
                frame = np.zeros((48, 64, 4), dtype=np.uint8)
                frame[:, :, 0] = index * 10
                frame[:, :, 1] = 80
                frame[:, :, 2] = 160
                frame[:, :, 3] = 255
                self.frames.append(frame)

        def open(self, _init) -> int:
            return ErrorCode.SUCCESS

        def close(self) -> None:
            pass

        def get_camera_information(self):
            resolution = types.SimpleNamespace(width=64, height=48)
            configuration = types.SimpleNamespace(fps=30, resolution=resolution)
            return types.SimpleNamespace(camera_configuration=configuration)

        def get_svo_number_of_frames(self) -> int:
            return len(self.frames)

        def grab(self, _runtime) -> int:
            if fail_at is not None and self.index == fail_at:
                return 99
            if self.index >= len(self.frames):
                return ErrorCode.END_OF_SVOFILE_REACHED
            self.current = self.frames[self.index]
            self.index += 1
            return ErrorCode.SUCCESS

        def retrieve_image(self, image, _view) -> None:
            image.data = self.current

        def get_timestamp(self, _reference) -> Timestamp:
            return Timestamp(self.index * 33_333_333)

    sl.ERROR_CODE = ErrorCode
    sl.InitParameters = InitParameters
    sl.DEPTH_MODE = types.SimpleNamespace(NONE=0)
    sl.Camera = Camera
    sl.Mat = Mat
    sl.RuntimeParameters = type("RuntimeParameters", (), {})
    sl.VIEW = types.SimpleNamespace(LEFT=0)
    sl.TIME_REFERENCE = types.SimpleNamespace(IMAGE=0)

    pyzed = types.ModuleType("pyzed")
    pyzed.sl = sl
    sys.modules["pyzed"] = pyzed
    sys.modules["pyzed.sl"] = sl


def load_export_module(*, fail_at: int | None = None):
    install_fake_pyzed(fail_at=fail_at)
    spec = importlib.util.spec_from_file_location("export_svo_under_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class ExportSvoTest(unittest.TestCase):
    def test_exports_atomic_mp4_and_sampled_jpegs(self) -> None:
        module = load_export_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            svo = root / "raw.svo2"
            video = root / "preview.mp4"
            partial = root / "preview.partial.mp4"
            complete = root / "preview.mp4.complete"
            images = root / "images"
            svo.write_bytes(b"simulated-svo")
            partial.write_bytes(b"stale-incomplete-video")

            old_argv = sys.argv
            try:
                sys.argv = [
                    str(SCRIPT),
                    str(svo),
                    "--video",
                    str(video),
                    "--images-dir",
                    str(images),
                    "--every-n",
                    "3",
                ]
                self.assertEqual(module.main(), 0)
            finally:
                sys.argv = old_argv

            self.assertTrue(video.is_file())
            self.assertGreater(video.stat().st_size, 0)
            self.assertFalse(partial.exists())
            self.assertTrue(complete.is_file())
            self.assertIn("frames=12", complete.read_text())
            self.assertEqual(len(list(images.glob("frame_*.jpg"))), 4)
            self.assertEqual(len((images / "frames.csv").read_text().splitlines()), 5)

            capture = cv2.VideoCapture(str(video))
            self.assertTrue(capture.isOpened())
            self.assertEqual(int(capture.get(cv2.CAP_PROP_FRAME_COUNT)), 12)
            capture.release()

    def test_failed_export_leaves_no_playable_or_partial_video(self) -> None:
        module = load_export_module(fail_at=2)
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            svo = root / "raw.svo2"
            video = root / "preview.mp4"
            svo.write_bytes(b"simulated-svo")

            old_argv = sys.argv
            try:
                sys.argv = [str(SCRIPT), str(svo), "--video", str(video)]
                with self.assertRaises(SystemExit):
                    module.main()
            finally:
                sys.argv = old_argv

            self.assertFalse(video.exists())
            self.assertFalse((root / "preview.partial.mp4").exists())
            self.assertFalse((root / "preview.mp4.complete").exists())


if __name__ == "__main__":
    unittest.main()
