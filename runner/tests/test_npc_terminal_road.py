"""Finite-map NPC braking must preserve route topology and physical control."""
import importlib.util
import os
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import carla

SOURCE = Path(os.environ.get("C2X_NPC_UNDER_TEST", str(
    Path(__file__).resolve().parents[1] / "runtime_overrides/npc_vehicle_control.py")))
SPEC = importlib.util.spec_from_file_location("npc_terminal_under_test", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def waypoint(x, y=0, continues=False):
    node = Mock()
    node.transform = carla.Transform(carla.Location(x=x, y=y))
    node.next.return_value = [Mock()] if continues else []
    return node


class TerminalRoadTest(unittest.TestCase):
    def test_brakes_before_bumper_reaches_physical_edge(self):
        plan = [(waypoint(100), None)]
        speed, remaining = MODULE.terminal_road_speed(plan, carla.Location(x=90), 8, 8, 2.4)
        self.assertLess(speed, 8)
        self.assertGreater(speed, 0)
        self.assertEqual(remaining, 10)
        speed, _ = MODULE.terminal_road_speed(plan, carla.Location(x=96), 8, 2, 2.4)
        self.assertEqual(speed, 0)

    def test_following_action_route_endpoint_is_not_physical_edge(self):
        plan = [(waypoint(100, continues=True), None)]
        self.assertEqual(MODULE.terminal_road_speed(plan, carla.Location(x=99), 8, 8, 2.4), (8, None))

    def test_distance_follows_selected_bend_not_endpoint_chord(self):
        plan = [(waypoint(30), None), (waypoint(30, 30), None), (waypoint(0, 30), None)]
        speed, remaining = MODULE.terminal_road_speed(plan, carla.Location(), 12, 12, 2.4)
        self.assertEqual(remaining, 90)
        self.assertEqual(speed, 12)
        self.assertEqual([p.transform.location.x for p, _ in plan], [30, 30, 0])

    def controller(self, plan, done=False):
        control = MODULE.NpcVehicleControl.__new__(MODULE.NpcVehicleControl)
        control._waypoints_updated = False
        control._offset_updated = False
        control._target_speed = 8
        control._init_speed = True
        control._brake_lights_active = False
        control._terminal_braking_logged = False
        control._local_planner = Mock()
        control._local_planner.get_plan.return_value = plan
        control._local_planner.done.return_value = done
        control._actor = Mock(is_alive=True)
        control._actor.get_velocity.return_value = carla.Vector3D()
        control._actor.get_location.return_value = carla.Location(x=96)
        control._actor.get_transform.return_value = carla.Transform()
        control._actor.get_light_state.return_value = 0
        control._actor.bounding_box = SimpleNamespace(extent=SimpleNamespace(x=2.4), location=SimpleNamespace(x=0))
        return control

    def test_initial_speed_cannot_override_terminal_braking(self):
        control = self.controller([(waypoint(100), None)])
        control.run_step()
        control._local_planner.set_speed.assert_called_once_with(0)
        control._actor.apply_control.assert_called_once()
        control._actor.set_target_velocity.assert_not_called()

    def test_initial_speed_cannot_restart_completed_plan(self):
        control = self.controller([], done=True)
        control.run_step()
        self.assertTrue(control._reached_goal)
        control._actor.set_target_velocity.assert_not_called()

    def test_original_initial_speed_behavior_remains_away_from_edge(self):
        control = self.controller([(waypoint(100, continues=True), None)])
        control.run_step()
        control._local_planner.set_speed.assert_called_once_with(28.8)
        control._actor.set_target_velocity.assert_called_once()


if __name__ == "__main__":
    unittest.main()
