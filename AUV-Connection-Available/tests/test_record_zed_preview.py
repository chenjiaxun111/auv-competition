#!/usr/bin/env python3
"""Computer-only tests for the combined ZED recorder and web preview."""

from __future__ import annotations

import importlib.util
import json
import socket
import sys
import tempfile
import types
import unittest
import urllib.request
from pathlib import Path

import cv2
import numpy as np

from test_export_svo import install_fake_pyzed


SCRIPT = Path(__file__).resolve().parents[1] / "record_zed_preview.py"


def load_module():
    install_fake_pyzed()
    spec = importlib.util.spec_from_file_location("record_zed_preview_under_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class PreviewRecorderTest(unittest.TestCase):
    def test_read_only_http_preview_endpoints(self) -> None:
        module = load_module()
        server = module.PreviewServer("127.0.0.1", 0)
        try:
            frame = np.zeros((48, 64, 3), dtype=np.uint8)
            ok, jpeg = cv2.imencode(".jpg", frame)
            self.assertTrue(ok)
            server.publish(jpeg.tobytes(), {"state": "recording", "frames": 12})
            base = f"http://127.0.0.1:{server.port}"
            with urllib.request.urlopen(base + "/", timeout=2) as response:
                self.assertIn(b"ZED SVO2 recording preview", response.read())
            with urllib.request.urlopen(base + "/api/status", timeout=2) as response:
                status = json.loads(response.read())
                self.assertEqual(status["frames"], 12)
            with urllib.request.urlopen(base + "/frame.jpg", timeout=2) as response:
                self.assertGreater(len(response.read()), 100)
        finally:
            server.close()

    def test_fake_camera_records_target_frame_count(self) -> None:
        module = load_module()
        sl = sys.modules["pyzed.sl"]
        sl.RESOLUTION = types.SimpleNamespace(HD720=0)
        sl.SVO_COMPRESSION_MODE = types.SimpleNamespace(LOSSLESS=0)

        class RecordingParameters:
            def __init__(self, path: str, compression) -> None:
                self.path = path
                self.compression = compression

        class Camera:
            def __init__(self) -> None:
                self.recording_path = None
                self.frame = np.zeros((48, 64, 4), dtype=np.uint8)
                self.frame[:, :, 1] = 100
                self.frame[:, :, 3] = 255

            def open(self, _init) -> int:
                return sl.ERROR_CODE.SUCCESS

            def enable_recording(self, params) -> int:
                self.recording_path = Path(params.path)
                self.recording_path.write_bytes(b"fake-svo-start\n")
                return sl.ERROR_CODE.SUCCESS

            def grab(self, _runtime) -> int:
                with self.recording_path.open("ab") as stream:
                    stream.write(b"frame\n")
                return sl.ERROR_CODE.SUCCESS

            def retrieve_image(self, image, _view) -> None:
                image.data = self.frame

            def disable_recording(self) -> None:
                pass

            def close(self) -> None:
                pass

        sl.RecordingParameters = RecordingParameters
        sl.Camera = Camera

        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "raw.svo2"
            with socket.socket() as sock:
                sock.bind(("127.0.0.1", 0))
                port = sock.getsockname()[1]
            old_argv = sys.argv
            try:
                sys.argv = [
                    str(SCRIPT),
                    "--output",
                    str(output),
                    "--duration",
                    "5",
                    "--fps",
                    "30",
                    "--preview-host",
                    "127.0.0.1",
                    "--preview-port",
                    str(port),
                    "--min-free-gb",
                    "1",
                ]
                self.assertEqual(module.main(), 0)
            finally:
                sys.argv = old_argv
            self.assertTrue(output.is_file())
            self.assertGreater(output.stat().st_size, 100)


if __name__ == "__main__":
    unittest.main()
