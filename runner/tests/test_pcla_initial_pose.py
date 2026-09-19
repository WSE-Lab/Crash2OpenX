"""Full source routes must not move a parked AV into a driving lane."""
import importlib.util
import os
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock, patch

import carla

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
SOURCE = Path(os.environ.get("C2X_DEMO_UNDER_TEST", str(ROOT / "src/demo.py")))
SPEC = importlib.util.spec_from_file_location("demo_pose_under_test", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
MODULE.carla = carla


class InitialPoseTest(unittest.TestCase):
    def test_full_route_keeps_parked_position_heading_and_later_turn(self):
        original = [carla.Transform(carla.Location(x=3.55, y=30, z=.5), carla.Rotation(yaw=-90)),
                    carla.Transform(carla.Location(x=3.55, y=10, z=.5), carla.Rotation(yaw=-90)),
                    carla.Transform(carla.Location(x=10, y=5, z=.5), carla.Rotation(yaw=0))]
        road_map = Mock()
        road_map.get_waypoint.return_value.transform = carla.Transform(
            carla.Location(x=1.75, y=30), carla.Rotation(yaw=-80))
        world = Mock()
        world.get_map.return_value = road_map
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "route.xml"
            with patch.object(MODULE, "ensure_world_matches_scenario_town", return_value=world), \
                 patch.object(MODULE, "extract_actor_follow_trajectory_transforms", return_value=original):
                start = MODULE.build_pcla_sut_route_from_xosc(ET.ElementTree(ET.Element("OpenSCENARIO")),
                    "hero", Mock(), str(path), route_mode="full", route_spacing=2)
            points = ET.parse(path).getroot().findall("waypoint")
            self.assertAlmostEqual(start.location.x, 3.55, places=5)
            self.assertAlmostEqual(start.rotation.yaw, -90)
            self.assertAlmostEqual(float(points[0].get("x")), 3.55, places=5)
            self.assertAlmostEqual(float(points[-1].get("x")), 10)
            self.assertAlmostEqual(float(points[-1].get("y")), 5)
            self.assertAlmostEqual(float(points[-1].get("yaw")), 0)


if __name__ == "__main__":
    unittest.main()
