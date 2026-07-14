#!/usr/bin/env python3
"""Focused regression tests for the L3 intent and trajectory verdict."""

import json
import tempfile
import unittest
from pathlib import Path

from tools.scene_outcome import check_scene_outcome


SCENE = {"scene": {"collision": {"a": "ego", "b": "v1"}}}


def _check(*, lanes: int, off_road_time: float):
    feedback = {
        "summary": {
            "collision_detected": True,
            "collision_actors": ["hero", "v1"],
            "min_distance_m": 3.0,
        },
        "behavior_checks": {"impact_area_estimated": "front"},
    }
    summary = {
        "collision_count": 1,
        "lane_invasion_count": lanes,
        "off_road_time": off_road_time,
        "criteria": [{"name": "WrongLaneTest", "actual_value": 0}],
    }
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "sim_feedback.json"
        path.write_text(json.dumps(feedback))
        return check_scene_outcome(SCENE, summary, path)


class SceneOutcomeTest(unittest.TestCase):
    def test_clean_aligned_run(self):
        out = _check(lanes=0, off_road_time=0.0)
        self.assertTrue(out["aligned"])
        self.assertEqual(out["trajectory_quality"], "clean")

    def test_brief_deviation_is_visible_warning(self):
        out = _check(lanes=2, off_road_time=0.15)
        self.assertTrue(out["aligned"])
        self.assertEqual(out["trajectory_quality"], "acceptable_with_warnings")
        self.assertEqual(len(out["warnings"]), 2)

    def test_large_deviation_invalidates_run(self):
        out = _check(lanes=3, off_road_time=0.30)
        self.assertFalse(out["aligned"])
        self.assertEqual(out["trajectory_quality"], "invalid")
        self.assertEqual(len(out["issues"]), 2)

    def test_small_preimpact_center_excess_is_warning(self):
        feedback = {
            "summary": {
                "collision_detected": True,
                "collision_time_s": 9.70,
                "collision_actors": ["hero", "v1"],
                "min_distance_m": 3.0,
            },
            "behavior_checks": {"impact_area_estimated": "front"},
        }
        summary = {
            "collision_count": 1,
            "lane_invasion_count": 2,
            "off_road_time": 0.95,
            "criteria": [{"name": "WrongLaneTest", "actual_value": 0}],
        }
        xodr = """<OpenDRIVE><road id="2" length="100" junction="-1"><lanes>
          <laneSection s="0"><right><lane id="-1" type="driving">
          <width sOffset="0" a="3.7" b="0" c="0" d="0"/>
          </lane></right></laneSection></lanes></road></OpenDRIVE>"""
        rows = [
            {"simulation_time": 8.80, "actors": {"hero": {
                "road_id": 2, "lane_id": -1,
                "nearest_driving_waypoint_distance_m": 1.88}}},
            {"simulation_time": 9.70, "actors": {"hero": {
                "road_id": 2, "lane_id": -1,
                "nearest_driving_waypoint_distance_m": 2.02}}},
        ]
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            fb_path = root / "sim_feedback.json"
            fb_path.write_text(json.dumps(feedback))
            (root / "map.xodr").write_text(xodr)
            (root / "sim_trace_raw.jsonl").write_text(
                "\n".join(json.dumps(row) for row in rows) + "\n")
            out = check_scene_outcome(SCENE, summary, fb_path)
        self.assertTrue(out["aligned"])
        self.assertEqual(out["trajectory_quality"], "acceptable_with_warnings")
        self.assertAlmostEqual(out["max_lateral_excess_m"], 0.17, places=3)
        self.assertAlmostEqual(out["off_road_before_collision_s"], 0.90, places=2)


if __name__ == "__main__":
    unittest.main()
