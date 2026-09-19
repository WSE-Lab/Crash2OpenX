"""Generated OSC routes must survive the PCLA adapter without branch replanning."""
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
SPEC = importlib.util.spec_from_file_location("demo_assigned_route_under_test", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
MODULE.carla = carla


def scenario():
    root = ET.Element("OpenSCENARIO")
    for actor, points in [("npc", [(99, 99), (100, 100)]),
                          ("hero", [(0, 0), (5, 2), (10, 0)])]:
        group = ET.SubElement(root, "ManeuverGroup")
        ET.SubElement(ET.SubElement(group, "Actors"), "EntityRef", entityRef=actor)
        route = ET.SubElement(ET.SubElement(group, "AssignRouteAction"), "Route")
        for x, y in points:
            position = ET.SubElement(ET.SubElement(route, "Waypoint"), "Position")
            ET.SubElement(position, "WorldPosition", x=str(x), y=str(y), z="0.2", h="0")
    return ET.ElementTree(root)


class AssignedRouteTest(unittest.TestCase):
    def test_generated_route_keeps_intermediate_waypoints_in_endpoints_mode(self):
        world = Mock()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "route.xml"
            with patch.object(MODULE, "ensure_world_matches_scenario_town", return_value=world), \
                 patch.object(MODULE, "build_straight_lane_route_from_endpoints", side_effect=AssertionError("must not replan")), \
                 patch.object(MODULE, "align_transforms_to_start_driving_lane", side_effect=AssertionError("must not relocate")):
                MODULE.build_pcla_sut_route_from_xosc(scenario(), "hero", Mock(), str(path),
                                                     route_mode="endpoints", route_spacing=2)
            points = ET.parse(path).getroot().findall("waypoint")
            xy = [(float(p.get("x")), float(p.get("y"))) for p in points]
            self.assertEqual(xy[0], (0, 0))
            self.assertIn((5, -2), xy)
            self.assertEqual(xy[-1], (10, 0))
            self.assertFalse(world.get_map.return_value.method_calls)

    def test_multiple_sequential_routes_are_not_silently_concatenated(self):
        tree = scenario()
        group = tree.getroot().findall("ManeuverGroup")[-1]
        ET.SubElement(ET.SubElement(group, "AssignRouteAction"), "Route")
        with self.assertRaisesRegex(ValueError, "multiple assigned routes"):
            MODULE.extract_actor_assigned_route_transforms(tree, "hero")


if __name__ == "__main__":
    unittest.main()
