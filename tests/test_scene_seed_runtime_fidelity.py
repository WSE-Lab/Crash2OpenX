"""Source seed fields must survive normalization and reach standard OSC."""
import copy
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
import xmlschema

from tools.api_infer_scene_seed_v2 import normalize
from tools.build_road_seed_opendrive import build
from tools.osc_blocks import _hero_cruise, build_xosc

ROOT = Path(__file__).resolve().parents[1]


def inferred_scene():
    return {"status": "supported", "scene": {
        "sut": {"id": "ego", "kind": "vehicle", "maneuver": "straight",
                "params": {"speed": 5.0}},
        "npcs": [{"id": "rear", "kind": "vehicle", "position": "behind_same_lane",
                  "side": "none", "behavior": {"block": "rear_hit", "params": {"speed": 8}}}],
        "collision": {"a": "rear", "b": "ego"}, "control": "none"}}


@pytest.fixture
def fresh_road(tmp_path):
    road = {"road": {"topology": "straight", "type": "town",
                     "lanes": {"forward": 1, "backward": 1}, "center_line": "broken"}}
    xodr = tmp_path / "new.xodr"
    build(road, xodr, tmp_path / "new.html", ROOT / "xsd/OpenDRIVE_1.5M.xsd")
    return road, xodr


@pytest.mark.parametrize("params,expected", [({"speed": 5.0}, 5.0),
    ({"speed": 11.2, "initial_speed_mps": 0.0}, 0.0)])
def test_inferred_speed_survives_to_xosc(params, expected, fresh_road, tmp_path):
    inferred = inferred_scene()
    inferred['scene']['sut']['params'] = params
    normalized = normalize(inferred, tmp_path / 'source.pdf', 'unit-test-no-api')
    assert normalized['scene']['sut']['params'] == params
    road, xodr = fresh_road
    output = tmp_path / 'scenario.xosc'
    build_xosc(normalized['scene'], str(xodr), output, road_seed=road)
    point = ET.parse(output).find("./Storyboard/Init/Actions/Private[@entityRef='hero']//AbsoluteTargetSpeed")
    assert float(point.get('value')) == expected
    xmlschema.XMLSchema(ROOT / 'xsd/OpenSCENARIO.xsd').validate(output)


@pytest.mark.parametrize('key', ['speed', 'initial_speed_mps'])
@pytest.mark.parametrize('invalid', [-1, math.nan, math.inf, True, '5'])
def test_invalid_source_speed_is_rejected(key, invalid, tmp_path):
    inferred = inferred_scene()
    inferred['scene']['sut']['params'] = {key: invalid}
    with pytest.raises(ValueError, match='finite and non-negative'):
        normalize(inferred, tmp_path / 'source.pdf', 'unit-test-no-api')
    with pytest.raises(ValueError, match='finite and non-negative'):
        _hero_cruise(inferred['scene'])


def test_cyclist_compiles_to_two_wheel_carla_actor(fresh_road, tmp_path):
    scene = copy.deepcopy(inferred_scene()['scene'])
    scene['npcs'] = [{"id": "rider", "kind": "cyclist", "position": "oncoming",
                      "side": "none", "behavior": {"block": "oncoming", "params": {"speed": 3}}}]
    scene['collision'] = {"a": "rider", "b": "ego"}
    road, xodr = fresh_road
    output = tmp_path / 'cyclist.xosc'
    build_xosc(scene, str(xodr), output, road_seed=road)
    vehicle = ET.parse(output).find("./Entities/ScenarioObject[@name='rider']/Vehicle")
    assert vehicle.get('vehicleCategory') == 'bicycle'
    assert vehicle.get('name') == 'vehicle.diamondback.century'
    xmlschema.XMLSchema(ROOT / 'xsd/OpenSCENARIO.xsd').validate(output)


@pytest.mark.parametrize('vehicle_class,blueprint,category', [
    ('truck', 'vehicle.carlamotors.european_hgv', 'truck'),
    ('bus', 'vehicle.mitsubishi.fusorosa', 'bus'),
    ('ambulance', 'vehicle.ford.ambulance', 'van'),
])
def test_source_vehicle_class_survives_to_actual_blueprint(vehicle_class, blueprint, category, fresh_road, tmp_path):
    inferred = inferred_scene()
    inferred['scene']['npcs'][0]['vehicle_class'] = vehicle_class
    inferred['scene']['sut']['vehicle_class'] = 'suv'
    normalized = normalize(inferred, tmp_path / 'source.pdf', 'unit-test-no-api')['scene']
    road, xodr = fresh_road
    output = tmp_path / 'classes.xosc'
    build_xosc(normalized, str(xodr), output, road_seed=road)
    root = ET.parse(output)
    npc = root.find("./Entities/ScenarioObject[@name='rear']/Vehicle")
    assert npc.get('name') == blueprint
    assert npc.get('vehicleCategory') == category
    assert root.find("./Entities/ScenarioObject[@name='hero']/Vehicle").get('name') == 'vehicle.nissan.patrol_2021'
    xmlschema.XMLSchema(ROOT / 'xsd/OpenSCENARIO.xsd').validate(output)


@pytest.mark.parametrize('heading,expected', [('parallel', 0), ('opposite', math.pi), ('perpendicular', math.pi / 2)])
def test_parked_vehicle_keeps_source_roadside_position_and_heading(heading, expected, fresh_road, tmp_path):
    inferred = inferred_scene()
    inferred['scene']['npcs'] = [{'id': 'parked', 'kind': 'vehicle', 'vehicle_class': 'car',
        'position': 'roadside', 'side': 'right', 'parked_heading': heading,
        'behavior': {'block': 'static_hold', 'params': {}}}]
    inferred['scene']['collision'] = {'a': 'ego', 'b': 'parked'}
    scene = normalize(inferred, tmp_path / 'source.pdf', 'unit-test-no-api')['scene']
    assert scene['npcs'][0]['position'] == 'roadside'
    road, xodr = fresh_road
    output = tmp_path / 'parked.xosc'
    build_xosc(scene, str(xodr), output, road_seed=road)
    pos = ET.parse(output).find("./Storyboard/Init/Actions/Private[@entityRef='parked']//RelativeLanePosition")
    assert float(pos.get('offset')) == -3.5
    assert float(pos.find('Orientation').get('h')) == pytest.approx(expected)
    xmlschema.XMLSchema(ROOT / 'xsd/OpenSCENARIO.xsd').validate(output)


def test_roadside_crossing_and_parked_defaults_have_distinct_headings():
    from tools.osc_blocks import resolve_position
    base = {'id': 'actor', 'kind': 'vehicle', 'position': 'roadside', 'side': 'right',
            'behavior': {'block': 'static_hold'}}
    parked = resolve_position(base).get_element().find('RelativeLanePosition/Orientation')
    assert float(parked.get('h')) == 0
    base.update(kind='pedestrian', behavior={'block': 'cross'})
    crossing = resolve_position(base).get_element().find('RelativeLanePosition/Orientation')
    assert float(crossing.get('h')) == pytest.approx(math.pi / 2)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('lane_id,yaw', [(-1, 0), (-2, 180), (1, 90)])
def test_roadside_crossing_heads_toward_road_after_runtime_conversion(side, lane_id, yaw):
    from tools.osc_blocks import resolve_position
    from runner.runtime_overrides.lane_coordinates import lane_offset_delta
    npc = {'id': 'walker', 'kind': 'pedestrian', 'position': 'roadside',
           'side': side, 'behavior': {'block': 'cross'}}
    pos = resolve_position(npc, lane_id).get_element().find('RelativeLanePosition')
    dx, dy = lane_offset_delta(yaw, lane_id, float(pos.get('offset')))
    # Match parser's OSC-to-CARLA heading conversion, then measure whether
    # the velocity points toward the lane center from the actual spawn.
    actor_yaw = math.radians(yaw) - float(pos.find('Orientation').get('h'))
    assert dx * math.cos(actor_yaw) + dy * math.sin(actor_yaw) < 0


def test_straight_cyclist_remains_bicycle_without_lateral_action(fresh_road, tmp_path):
    inferred = inferred_scene()
    inferred['scene']['npcs'] = [{'id': 'rider', 'kind': 'cyclist',
        'position': 'roadside', 'side': 'right', 'behavior': {'block': 'cruise', 'params': {'speed': 4}}}]
    inferred['scene']['collision'] = {'a': 'ego', 'b': 'rider'}
    scene = normalize(inferred, tmp_path/'source.pdf', 'test')['scene']
    road, xodr = fresh_road
    output = tmp_path/'cruise.xosc'
    build_xosc(scene, str(xodr), output, road_seed=road)
    tree = ET.parse(output)
    assert tree.find(".//ScenarioObject[@name='rider']/Vehicle").get('vehicleCategory') == 'bicycle'
    event = tree.find(".//Event[@name='rider_cruise']")
    assert float(event.find('.//AbsoluteTargetSpeed').get('value')) == 4
    assert tree.find('.//LaneChangeAction') is None
    assert tree.find('.//LaneOffsetAction') is None
    xmlschema.XMLSchema(ROOT / 'xsd/OpenSCENARIO.xsd').validate(output)


@pytest.mark.parametrize('ego_lane,delta', [(-1, 2), (-2, 3), (1, -2), (2, -3)])
def test_oncoming_vehicle_is_ahead_along_ego_travel_not_target_lane_travel(ego_lane, delta):
    from tools.osc_blocks import resolve_position
    npc = {'id': 'opposing', 'kind': 'vehicle', 'position': 'oncoming',
           'behavior': {'block': 'oncoming', 'params': {'gap': 25}}}
    position = resolve_position(npc, ego_lane).get_element().find('RelativeLanePosition')
    assert int(position.get('dLane')) == delta
    assert float(position.get('ds')) == -25
    # The target lane already faces oncoming; no second 180-degree turn.
    assert float(position.find('Orientation').get('h')) == 0


@pytest.mark.parametrize('role', ['tesla', 'truck_like_id', 'v2', 'hero'])
def test_default_passenger_car_blueprint_is_independent_of_role_name(role):
    from tools.osc_blocks import _vehicle
    vehicle = _vehicle(role, role == 'hero').get_element()
    assert vehicle.get('name') == 'vehicle.tesla.model3'
    assert vehicle.get('vehicleCategory') == 'car'
