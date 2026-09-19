import ast
import math
from types import SimpleNamespace
from xml.etree import ElementTree as ET

import pytest
import xmlschema

from tools.api_infer_road_seed import normalize_supported
from tools.api_infer_scene_seed_v2 import normalize
from tools.build_road_seed_opendrive import build, validate_seed
from tools.osc_blocks import ROOT, BlockUnsupported, build_xosc
from tools.visualization_qa import xodr_lane_center_segments


def road():
    return {'road': {'topology': 'straight', 'type': 'town',
        'lanes': {'forward': 2, 'backward': 0}, 'center_line': 'broken',
        'parking': {'left': True}}}


def scene():
    return {'sut': {'id': 'ego', 'kind': 'vehicle', 'maneuver': 'straight'},
            'npcs': [{'id': 'passing', 'kind': 'vehicle', 'position': 'adjacent',
                      'side': 'right', 'params': {'long': -12},
                      'behavior': {'block': 'sequence', 'steps': [
                          {'action': 'drive', 'params': {'speed': 10}, 'when': {'condition': 'start'}},
                          {'action': 'lane_change', 'params': {'direction': 'left', 'lanes': 2},
                           'when': {'condition': 'after_previous'}},
                          {'action': 'lane_change', 'params': {'direction': 'right'},
                           'when': {'condition': 'after_previous'}}]}}],
            'collision': {'a': 'passing', 'b': 'ego'}}


def test_parking_is_not_a_driving_lane_and_full_lateral_sequence_survives(tmp_path):
    r = normalize_supported(road(), tmp_path/'source.pdf', 'test')
    s = normalize({'status': 'supported', 'scene': scene()}, tmp_path/'source.pdf', 'test')['scene']
    xodr = tmp_path/'new.xodr'; out = tmp_path/'new.xosc'
    build(r, xodr, tmp_path/'map.html', ROOT/'xsd/OpenDRIVE_1.5M.xsd')
    build_xosc(s, str(xodr), out, road_seed=r)
    xmlschema.XMLSchema(ROOT/'xsd/OpenSCENARIO.xsd').validate(out)
    xml = ET.parse(xodr)
    assert [l.get('id') for l in xml.findall('.//lane[@type="parking"]')] == ['-1']
    assert [l.get('id') for l in xml.findall('.//lane[@type="driving"]')] == ['-2', '-3']
    centers = xodr_lane_center_segments(xodr)
    inner = next(l for l in centers if l['lane_id'] == -2)
    geometry = xml.find('.//geometry')
    dx = inner['segment'][0] - float(geometry.get('x'))
    dy = inner['segment'][1] - float(geometry.get('y'))
    heading = float(geometry.get('hdg'))
    assert -dx * math.sin(heading) + dy * math.cos(heading) == pytest.approx(-2.5 - 1.75)
    sx = ET.parse(out)
    assert sx.find('.//Private[@entityRef="hero"]//LanePosition').get('laneId') == '-2'
    assert sx.find('.//Private[@entityRef="passing"]//RelativeLanePosition').get('ds') == '-12.0'
    assert [n.get('value') for n in sx.findall('.//RelativeTargetLane')] == ['2', '-1']
    assert sx.find('.//FollowTrajectoryAction') is None
    assert len(sx.findall('.//TeleportAction')) == 2
    assert sx.find('.//Event[@name="passing_follow_lane"]') is None
    assert sx.find('.//Private[@entityRef="hero"]//Property[@value="external_control"]') is not None


def test_sequence_rejects_parking_destination_missing_from_new_map(tmp_path):
    r = road(); del r['road']['parking']
    xodr = tmp_path/'new.xodr'
    build(r, xodr, tmp_path/'map.html', ROOT/'xsd/OpenDRIVE_1.5M.xsd')
    with pytest.raises(BlockUnsupported, match='existing same-direction'):
        build_xosc(scene(), str(xodr), tmp_path/'bad.xosc', road_seed=r)


def test_conflicting_position_representations_cannot_silently_move_actor(tmp_path):
    s = scene(); s['npcs'][0]['behavior']['params'] = {'long': 12}
    with pytest.raises(ValueError, match='conflicting NPC position'):
        normalize({'status': 'supported', 'scene': s}, tmp_path/'source.pdf', 'test')


@pytest.mark.parametrize('mutate', ['curve', 'two_way', 'number', 'extra_key'])
def test_unsupported_parking_geometries_are_rejected(mutate, tmp_path):
    r = road()
    if mutate == 'curve': r['road']['topology'] = 'curve'
    if mutate == 'two_way': r['road']['lanes']['backward'] = 1
    if mutate == 'number': r['road']['parking']['left'] = 1
    if mutate == 'extra_key': r['road']['parking']['width'] = 3
    assert validate_seed(r)
    with pytest.raises(ValueError):
        normalize_supported(r, tmp_path/'source.pdf', 'test')


def test_runtime_parking_support_keeps_default_policy_and_rejects_sidewalk():
    from tools.enable_parking_lane_changes import patched_helper, patched_atomics
    helper = ROOT/'external/scenario_runner/srunner/tools/scenario_helper.py'
    source = patched_helper(helper.read_text())
    tree = ast.parse(source)
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'generate_target_waypoint_list_multilane')
    types = SimpleNamespace(Driving=1, Parking=2, Sidewalk=3)
    namespace = {'carla': SimpleNamespace(LaneType=types),
                 'RoadOption': SimpleNamespace(LANEFOLLOW=0, CHANGELANELEFT=1, CHANGELANERIGHT=2)}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(helper), 'exec'), namespace)
    fn = namespace[function.name]

    class Location:
        def __init__(self, x): self.x = x
        def distance(self, other): return abs(self.x-other.x)

    class Waypoint:
        target_type = types.Parking
        def __init__(self, s=0, lane_id=-2):
            self.s, self.lane_id = s, lane_id
            self.lane_type = types.Driving if lane_id == -2 else self.target_type
            self.transform = SimpleNamespace(location=Location(s))
        def next(self, d): return [Waypoint(self.s+d, self.lane_id)]
        def get_left_lane(self): return Waypoint(self.s, -1)

    assert fn(Waypoint(), check=False) == (None, None)
    plan, target = fn(Waypoint(), check=False, allowed_lane_types=(types.Driving, types.Parking))
    assert plan and target == -1
    Waypoint.target_type = types.Sidewalk
    assert fn(Waypoint(), check=False, allowed_lane_types=(types.Driving, types.Parking)) == (None, None)
    atomics = ROOT/'external/scenario_runner/srunner/scenariomanager/scenarioatomics/atomic_behaviors.py'
    compile(patched_atomics(atomics.read_text()), str(atomics), 'exec')
