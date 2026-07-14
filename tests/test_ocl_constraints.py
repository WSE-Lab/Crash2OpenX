from __future__ import annotations

import sys
import unittest
from copy import deepcopy
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.ocl_constraints import CONSTRAINT_IDS, evaluate_constraints, paper_example


class OCLConstraintTests(unittest.TestCase):
    def setUp(self) -> None:
        self.road, self.scene = paper_example()

    def assert_only_failure(self, expected: str, road=None, scene=None) -> None:
        outcomes = evaluate_constraints(road or self.road, scene or self.scene)
        failures = [key for key, passed in outcomes.items() if not passed]
        self.assertEqual([expected], failures)

    def test_catalog_and_paper_example(self) -> None:
        self.assertEqual(15, len(CONSTRAINT_IDS))
        self.assertTrue(all(evaluate_constraints(self.road, self.scene).values()))

    def test_i1_lane_boundaries(self) -> None:
        for forward, backward in ((1, 0), (5, 5)):
            with self.subTest(valid=(forward, backward)):
                self.road["lanes"] = {"forward": forward, "backward": backward}
                self.assertTrue(evaluate_constraints(self.road, self.scene)["I1"])
        for forward, backward in ((0, 1), (6, 1), (1, -1), (1, 6)):
            with self.subTest(invalid=(forward, backward)):
                road, scene = paper_example()
                road["lanes"] = {"forward": forward, "backward": backward}
                self.assert_only_failure("I1", road, scene)

    def test_i2_ego_id(self) -> None:
        self.scene["sut"]["id"] = "av"
        self.scene["collision"]["striker_id"] = "av"
        self.assert_only_failure("I2")

    def test_i3_pedestrian_position(self) -> None:
        npc = self.scene["npcs"][0]
        npc.update(kind="pedestrian", position="oncoming")
        self.assert_only_failure("I3")
        for position in ("roadside_left", "roadside_right", "ahead_same_lane"):
            npc["position"] = position
            self.assertTrue(evaluate_constraints(self.road, self.scene)["I3"])

    def test_i4_static_behavior(self) -> None:
        npc = self.scene["npcs"][0]
        npc["kind"] = "static"
        self.assert_only_failure("I4")
        npc["behavior"]["block"] = "static_block"
        self.assertTrue(evaluate_constraints(self.road, self.scene)["I4"])

    def test_i5_actor_id_uniqueness(self) -> None:
        self.scene["npcs"].append(deepcopy(self.scene["npcs"][0]))
        self.assert_only_failure("I5")

    def test_i6_collision_references_and_distinctness(self) -> None:
        self.scene["collision"]["struck_id"] = "ghost"
        self.assert_only_failure("I6")
        self.scene["collision"] = {"striker_id": "ego", "struck_id": "ego"}
        self.assert_only_failure("I6")

    def test_i7_friction_boundaries(self) -> None:
        for value in (0.1, 1.0):
            self.scene["environment"]["friction_scale"] = value
            self.assertTrue(evaluate_constraints(self.road, self.scene)["I7"])
        for value in (0.099, 1.001):
            self.scene["environment"]["friction_scale"] = value
            self.assert_only_failure("I7")

    def test_i8_optional_behavior_parameters(self) -> None:
        params = self.scene["npcs"][0]["behavior"]["params"]
        params.update(target_speed=None, duration=None)
        self.assertTrue(evaluate_constraints(self.road, self.scene)["I8"])
        params.update(target_speed=-0.01, duration=None)
        self.assert_only_failure("I8")
        params.update(target_speed=None, duration=0.0)
        self.assert_only_failure("I8")

    def test_i9_trigger_value(self) -> None:
        self.scene["npcs"][0]["behavior"]["trigger"]["value"] = -0.01
        self.assert_only_failure("I9")
        self.scene["npcs"][0]["behavior"]["trigger"]["value"] = 0.0
        self.assertTrue(evaluate_constraints(self.road, self.scene)["I9"])

    def test_p1_junction_block_requires_junction(self) -> None:
        self.scene["npcs"][0]["behavior"]["block"] = "junction_cross"
        self.assert_only_failure("P1")
        self.road["topology"] = "y_junction"
        self.assertTrue(evaluate_constraints(self.road, self.scene)["P1"])

    def test_p2_oncoming_requires_backward_lane(self) -> None:
        self.scene["npcs"][0]["position"] = "oncoming"
        self.road["lanes"]["backward"] = 0
        self.assert_only_failure("P2")

    def test_p3_overtake_requires_backward_lane_and_broken_line(self) -> None:
        self.scene["sut"]["maneuver"] = "overtake_oncoming"
        self.road["center_line"] = "solid"
        self.assert_only_failure("P3")
        self.road["center_line"] = "broken"
        self.assertTrue(evaluate_constraints(self.road, self.scene)["P3"])

    def test_p4_adjacent_requires_two_forward_lanes(self) -> None:
        for position in ("adjacent_left", "adjacent_right"):
            with self.subTest(position=position):
                road, scene = paper_example()
                scene["npcs"][0]["position"] = position
                self.assert_only_failure("P4", road, scene)
                road["lanes"]["forward"] = 2
                self.assertTrue(evaluate_constraints(road, scene)["P4"])

    def test_p5_turn_requires_junction(self) -> None:
        self.scene["sut"]["maneuver"] = "left"
        self.assert_only_failure("P5")
        self.road["topology"] = "t_junction"
        self.assertTrue(evaluate_constraints(self.road, self.scene)["P5"])

    def test_p6_lane_change_requires_two_forward_lanes(self) -> None:
        self.scene["sut"]["maneuver"] = "lane_change_right"
        self.assert_only_failure("P6")
        self.road["lanes"]["forward"] = 2
        self.assertTrue(evaluate_constraints(self.road, self.scene)["P6"])


if __name__ == "__main__":
    unittest.main()
