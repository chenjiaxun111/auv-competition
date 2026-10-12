#!/usr/bin/env python3
"""Computer-only tests for extract_svo_frames.py."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

from test_export_svo import install_fake_pyzed


SCRIPT = Path(__file__).resolve().parents[1] / "extract_svo_frames.py"


def load_module(*, fail_at: int | None = None):
    install_fake_pyzed(fail_at=fail_at)
    spec = importlib.util.spec_from_file_location("extract_svo_frames_under_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class ExtractSvoFramesTest(unittest.TestCase):
    def run_main(self, module, svo: Path, output: Path) -> int:
        old_argv = sys.argv
        try:
            sys.argv = [
                str(SCRIPT),
                str(svo),
                "--output",
                str(output),
                "--every-n",
                "3",
            ]
            return module.main()
        finally:
            sys.argv = old_argv

    def test_extracts_atomic_left_image_dataset_and_reuses_it(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            svo = root / "raw.svo2"
            output = root / "images"
            svo.write_bytes(b"simulated-svo")

            self.assertEqual(self.run_main(module, svo, output), 0)
            self.assertTrue((output / ".complete").is_file())
            self.assertEqual(len(list(output.glob("frame_*.jpg"))), 4)
            self.assertEqual(len((output / "frames.csv").read_text().splitlines()), 5)
            self.assertFalse((root / "images.partial").exists())

            self.assertEqual(self.run_main(module, svo, output), 0)

    def test_read_failure_removes_partial_dataset(self) -> None:
        module = load_module(fail_at=2)
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            svo = root / "raw.svo2"
            output = root / "images"
            svo.write_bytes(b"simulated-svo")

            with self.assertRaises(SystemExit):
                self.run_main(module, svo, output)

            self.assertFalse(output.exists())
            self.assertFalse((root / "images.partial").exists())


if __name__ == "__main__":
    unittest.main()
