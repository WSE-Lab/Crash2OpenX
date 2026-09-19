from xml.etree import ElementTree as ET

from tools.audit_fresh_framework_batch import execution_integrity
from tools.package_fresh_framework_snapshot import portable_scene


def test_recorded_failure_is_not_execution_success():
    result = execution_integrity({'total_ticks': 32, 'termination_reason': 'scenario_failure',
                                  'scenario_tree_status': 'Status.FAILURE'}, {'hero': 0.1})
    assert not result['execution_integrity_pass']
    assert result['autonomous_task_success'] is None


def test_long_run_with_fallen_pedestrian_is_not_valid():
    assert not execution_integrity({'termination_reason': 'blocked_exit'},
                                   {'hero': 0.1, 'pedestrian': -1747})['execution_integrity_pass']


def test_blocked_run_is_not_promoted_to_autonomous_success():
    result = execution_integrity({'termination_reason': 'blocked_exit'}, {'hero': 0.1})
    assert result['execution_integrity_pass']
    assert result['autonomous_task_success'] is None


def test_delivery_relocation_preserves_actions_and_original(tmp_path):
    source = tmp_path/'compiled.xosc'
    payload = b'<OpenSCENARIO><RoadNetwork><LogicFile filepath="original.xodr"/></RoadNetwork><Storyboard><Action name="brake"/></Storyboard></OpenSCENARIO>'
    source.write_bytes(payload)
    destination = tmp_path/'delivery/scenario.xosc'
    result = portable_scene(source, destination)
    assert source.read_bytes() == payload
    a, b = ET.fromstring(payload), ET.parse(destination).getroot()
    a.find('./RoadNetwork/LogicFile').set('filepath', 'map.xodr')
    assert ET.tostring(a) == ET.tostring(b)
    assert result['compiled_sha256'] != result['portable_sha256']
