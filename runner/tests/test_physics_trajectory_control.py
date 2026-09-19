"""Controller regressions in the installed PCLA/CARLA environment."""
import importlib.util
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import carla

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scenario_runner"))
sys.path.insert(0, str(Path.home() / "carla/PythonAPI/carla"))
SOURCE = ROOT / "runtime_overrides/physics_trajectory_control.py"
if not SOURCE.is_file():
    SOURCE = ROOT / "scenario_runner/srunner/scenariomanager/actorcontrols/physics_trajectory_control.py"
SPEC = importlib.util.spec_from_file_location("physics_controller_under_test", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class PhysicsControlTest(unittest.TestCase):
    def actor(self):
        actor = Mock(id=1, is_alive=True, type_id="vehicle.toyota.prius", attributes={"role_name": "npc"})
        actor.get_transform.return_value = carla.Transform()
        actor.get_velocity.return_value = carla.Vector3D()
        return actor

    def test_delayed_actor_uses_bounded_actuation_without_changing_pose_or_velocity(self):
        actor = self.actor()
        controller = MODULE.PhysicsTrajectoryControl(actor)
        controller.update_waypoints([carla.Transform(), carla.Transform(carla.Location(x=30, y=8))], start_time=0, times=[0, 10])
        with patch.object(MODULE.GameTime, "get_time", return_value=5):
            controller.run_step()
        control = actor.apply_control.call_args[0][0]
        self.assertGreater(control.throttle, 0)
        self.assertGreater(control.steer, 0)
        self.assertLessEqual(control.throttle, .81)
        actor.set_transform.assert_not_called()
        actor.set_target_velocity.assert_not_called()
        actor.set_simulate_physics.assert_not_called()

    def test_stationary_coasting_middle_vehicle_is_not_pulled_back_after_impact(self):
        actor = self.actor()
        actor.get_transform.return_value = carla.Transform(carla.Location(x=3))
        actor.get_velocity.return_value = carla.Vector3D(x=2)
        controller = MODULE.PhysicsTrajectoryControl(actor, {"hold_brake": 0})
        controller.update_waypoints([carla.Transform(), carla.Transform()], start_time=0, times=[0, 10])
        with patch.object(MODULE.GameTime, "get_time", return_value=5):
            controller.run_step()
        control = actor.apply_control.call_args[0][0]
        self.assertEqual(control.throttle, 0)
        self.assertEqual(control.brake, 0)
        actor.set_transform.assert_not_called()

    def test_non_monotonic_timing_is_rejected(self):
        controller = MODULE.PhysicsTrajectoryControl(self.actor())
        with self.assertRaises(ValueError):
            controller.update_waypoints([carla.Transform(), carla.Transform()], times=[1, 1])

    def test_secondary_impact_vehicle_coasts_only_after_real_vehicle_contact(self):
        actor = self.actor()
        controller = MODULE.PhysicsTrajectoryControl(actor, {"release_on_collision": "true"})
        controller._release_after_contact(Mock(other_actor=Mock(type_id="static.unknown")))
        self.assertFalse(controller._released)
        controller._release_after_contact(Mock(timestamp=5, other_actor=Mock(id=2, type_id="vehicle.toyota.prius")))
        controller.run_step()
        control = actor.apply_control.call_args[0][0]
        self.assertTrue(controller._released)
        self.assertEqual((control.throttle, control.brake, control.steer), (0, 0, 0))
        actor.set_transform.assert_not_called()
        actor.set_target_velocity.assert_not_called()

    def test_walker_uses_walk_control_instead_of_vehicle_actuation(self):
        actor = self.actor()
        actor.type_id = "walker.pedestrian.0001"
        controller = MODULE.PhysicsTrajectoryControl(actor)
        controller.update_waypoints([carla.Transform(), carla.Transform(carla.Location(x=10))], start_time=0, times=[0, 10])
        with patch.object(MODULE.GameTime, "get_time", return_value=1):
            controller.run_step()
        control = actor.apply_control.call_args[0][0]
        self.assertIsInstance(control, carla.WalkerControl)
        self.assertGreater(control.speed, 0)
        actor.set_transform.assert_not_called()


if __name__ == "__main__":
    unittest.main()
