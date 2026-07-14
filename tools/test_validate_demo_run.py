#!/usr/bin/env python3
"""Regression tests for the five-minute demo acceptance gate."""

import json
import tempfile
import unittest
from pathlib import Path

from tools.validate_demo_run import validate


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload))


def _passing_fixture(root: Path) -> None:
    outcome = {
        "aligned": True,
        "verdict": "aligned",
        "issues": [],
        "expected_collision": ["hero", "v1"],
        "actual_collision_actors": ["v1", "hero"],
        "wrong_lane_count": 0,
    }
    _write_json(root / "result.json", {"carla_status": "ok", "scene_outcome": outcome})
    _write_json(root / "summary.json", {
        "collision_count": 1,
        "lane_invasion_count": 2,
        "off_road_time": 0.15,
    })
    _write_json(root / "behavior_check.json", outcome)
    for name in ("road_seed.json", "scene_seed.json", "sim_feedback.json"):
        _write_json(root / name, {})
    (root / "case.xodr").write_text("<OpenDRIVE/>")
    (root / "case.xosc").write_text("<OpenSCENARIO/>")
    (root / "carla_rgb.mp4").write_bytes(b"0" * 100_000)
    frames = root / "rgb_frames"
    frames.mkdir()
    for i in range(5):
        (frames / f"frame_{i:06d}.jpg").write_bytes(bytes([i]) * 32)


class ValidateDemoRunTest(unittest.TestCase):
    def test_passing_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            _passing_fixture(root)
            report = validate(root)
            self.assertTrue(report["passed"])

    def test_off_road_run_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            _passing_fixture(root)
            summary = json.loads((root / "summary.json").read_text())
            summary["off_road_time"] = 0.30
            _write_json(root / "summary.json", summary)
            report = validate(root)
            self.assertFalse(report["passed"])
            failed = {item["name"] for item in report["checks"] if not item["passed"]}
            self.assertIn("off_road", failed)


if __name__ == "__main__":
    unittest.main()
