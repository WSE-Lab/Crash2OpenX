"""Run in the PCLA environment: python -m unittest discover -s runner/tests.

Uses the installed CARLA API types; a live server is not needed for these
regressions. Real collision integration evidence is collected separately.
"""
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import carla

SPEC = importlib.util.spec_from_file_location(
    "collector_under_test", Path(__file__).resolve().parents[1] / "src/data_collector.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class CollisionEvidenceTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.collector = MODULE.DataCollector(output_dir=self.directory.name)
        self.hero = Mock(id=1, is_alive=True, attributes={"role_name": "hero"})
        self.hero.get_location.return_value = carla.Location()
        self.other = Mock(id=2, is_alive=True, type_id="vehicle.toyota.prius",
                          attributes={"role_name": "rear"})
        self.collector.hero_actor = self.hero
        self.collector._write_event = Mock()

    def test_overlap_alone_neither_counts_nor_ends_episode(self):
        self.collector._get_expected_live_actors = lambda: [self.hero, self.other]
        self.collector._actors_overlap = lambda a, b: True
        self.collector._check_synthetic_collisions(10, 2.0)
        self.assertEqual(self.collector.summary["collision_count"], 0)
        self.assertFalse(self.collector.should_terminate())
        event = self.collector._write_event.call_args[0][0]
        self.assertEqual(event["event_type"], "geometric_overlap")

    def test_closed_sensor_does_not_prevent_other_cleanup_or_summary(self):
        broken=Mock(id=21);broken.stop.side_effect=RuntimeError('close: Bad file descriptor [system:9]')
        other=Mock(id=22)
        self.collector._collision_sensors={1:broken}
        self.collector._lane_invasion_sensor=other
        self.collector._write_sim_feedback=Mock()
        self.collector.finalize(termination_reason='collision_exit')
        broken.destroy.assert_called_once()
        other.stop.assert_called_once()
        other.destroy.assert_called_once()
        self.assertFalse(self.collector._collision_sensors)
        self.assertIsNone(self.collector._lane_invasion_sensor)
        summary=json.loads(Path(self.collector.paths['summary']).read_text())
        self.assertEqual(summary['sensor_cleanup_status'],'completed_with_errors')
        self.assertEqual(summary['sensor_cleanup_errors'][0]['sensor_id'],21)
        self.collector._cleanup_sensors()
        broken.destroy.assert_called_once()

    def event(self, timestamp=5.0):
        return SimpleNamespace(other_actor=self.other, timestamp=timestamp,
                               frame=100, normal_impulse=carla.Vector3D(x=500))

    def test_trace_keeps_actual_bounding_box_for_separation_measurement(self):
        actor=Mock(id=2,type_id='vehicle.tesla.model3')
        actor.get_transform.return_value=carla.Transform(carla.Location(x=10,y=20))
        actor.get_velocity.return_value=carla.Vector3D()
        actor.get_control.return_value=carla.VehicleControl()
        actor.bounding_box=carla.BoundingBox(carla.Location(),carla.Vector3D(x=2,y=1,z=.8))
        state=self.collector._extract_actor_state(actor,SimpleNamespace(frame=10),1.0,.05)
        points=state['bounding_box_world_vertices']
        self.assertEqual(len(points),8)
        self.assertEqual((min(p[0] for p in points),max(p[0] for p in points)),(8,12))
        self.assertEqual((min(p[1] for p in points),max(p[1] for p in points)),(19,21))

    def test_physical_impact_preserves_sensor_source_and_post_impact_tail(self):
        self.collector._on_collision(self.event())
        self.assertEqual(self.collector.summary["collision_count"], 1)
        self.assertEqual(self.collector._collision_deadline, 7.0)
        self.assertFalse(self.collector.should_terminate())
        event = self.collector._write_event.call_args[0][0]
        self.assertEqual(event["payload"]["source"], "carla_collision_sensor")
        self.assertEqual(event["payload"]["normal_impulse"]["x"], 500)
        self.collector._on_collision(self.event(6.0))
        self.assertEqual(self.collector._collision_deadline, 7.0)

    def test_multi_impact_mode_does_not_schedule_exit_after_first_collision(self):
        self.collector.stop_on_collision = False
        self.collector._on_collision(self.event())
        self.assertIsNone(self.collector._collision_deadline)
        self.assertFalse(self.collector.should_terminate())

    def test_compiled_contact_sequence_automatically_keeps_recording(self):
        from xml.etree import ElementTree as ET
        root = ET.Element('OpenSCENARIO')
        declarations = ET.SubElement(root, 'ParameterDeclarations')
        ET.SubElement(declarations, 'ParameterDeclaration', name='C2XExpectedContactSequence',
                      parameterType='string', value=json.dumps([
                          {'a': 'hero', 'b': 'middle'}, {'a': 'middle', 'b': 'rear'}]))
        entities = ET.SubElement(root, 'Entities')
        for name in ('hero', 'middle', 'rear'):
            obj = ET.SubElement(entities, 'ScenarioObject', name=name)
            ET.SubElement(obj, 'Vehicle', vehicleCategory='car', name='vehicle.tesla.model3')
        path = Path(self.directory.name)/'multi.xosc'
        ET.ElementTree(root).write(path)
        collector = MODULE.DataCollector(output_dir=self.directory.name, scenario_path=str(path))
        self.assertFalse(collector.stop_on_collision)
        self.assertEqual(len(collector.summary['expected_contact_sequence']), 2)

    def test_npc_contact_is_recorded_without_stopping_before_secondary_impact(self):
        middle = Mock(id=3, is_alive=True, attributes={"role_name": "middle"})
        middle.get_location.return_value = carla.Location()
        self.collector._on_collision(self.event(), middle)
        payload = self.collector._write_event.call_args[0][0]["payload"]
        self.assertEqual(set(payload["actors"]), {"middle", "rear"})
        self.assertFalse(payload["sut_involved"])
        self.assertIsNone(self.collector._collision_deadline)

    def test_two_attached_sensors_do_not_double_count_one_frame_contact(self):
        self.other.get_location.return_value = carla.Location()
        self.hero.type_id = "vehicle.lincoln.mkz_2020"
        self.collector._on_collision(self.event())
        reverse = self.event()
        reverse.other_actor = self.hero
        self.collector._on_collision(reverse, self.other)
        self.assertEqual(self.collector.summary["collision_count"], 1)


if __name__ == "__main__":
    unittest.main()
