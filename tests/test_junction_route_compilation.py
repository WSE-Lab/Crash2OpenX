import copy
from xml.etree import ElementTree as ET

import pytest

from tools import osc_blocks
from tools.build_road_seed_opendrive import build


@pytest.fixture
def junction(tmp_path, monkeypatch):
    road = {"road": {"topology": "cross_intersection", "type": "town",
                     "lanes": {"forward": 1, "backward": 1}, "center_line": "broken"}}
    xodr = tmp_path / "fresh.xodr"
    build(road, xodr, tmp_path / "fresh.html", osc_blocks.ROOT / "xsd/OpenDRIVE_1.5M.xsd")
    W = {}

    def node(name, road_id, lane_id, s, x, y, yaw):
        W[name] = {"id": name, "road_id": road_id, "lane_id": lane_id, "s": s,
                   "is_junction": False, "lane_type": "Driving",
                   "transform": {"x": x, "y": y, "z": 0, "yaw": yaw}}
        return name

    a = node("ego_start:s150", 2, -1, 150, 230, -80, 90)
    b = node("ego_end:s150", 4, 1, 150, 230, 80, 90)
    c = node("npc_start:s160", 1, -1, 160, 160, -1.75, 0)
    d = node("npc_end:s160", 3, 1, 160, 300, -1.75, 0)
    routes = [
        {"type": "straight", "start_road_id": 2, "start_lane_id": -1,
         "waypoint_ids": [a, b], "approach_waypoint_ids": [a]},
        {"type": "straight", "start_road_id": 1, "start_lane_id": -1,
         "waypoint_ids": [c, d], "approach_waypoint_ids": [c]},
    ]
    scene = {"sut": {"id": "ego", "kind": "vehicle", "maneuver": "straight"},
             "npcs": [{"id": "crossing", "kind": "vehicle", "position": "cross", "side": "none",
                       "behavior": {"block": "junction_cross"}}],
             "collision": {"a": "ego", "b": "crossing"}, "control": "traffic_light"}
    monkeypatch.setattr(osc_blocks, "load_roadgraph", lambda name: (W, routes))
    return road, xodr, scene, routes


def test_crossing_npc_gets_selected_route_at_start(junction, tmp_path):
    road, xodr, scene, routes = junction
    output = tmp_path / "scene.xosc"
    osc_blocks.build_xosc(scene, str(xodr), output, name="test", auto_extract=False, road_seed=road)
    event = ET.parse(output).find(".//Event[@name='crossing_cross']")
    assert event.find('.//SimulationTimeCondition').get('value') == '0.0'
    assert event.find('.//RelativeDistanceCondition') is None
    points = event.findall('.//AssignRouteAction/Route/Waypoint/Position/WorldPosition')
    assert [(float(p.get('x')), float(p.get('y'))) for p in points] == [(160, -1.75), (300, -1.75)]
    assert not event.findall('.//AssignRouteAction//LanePosition')
    tree = ET.parse(output)
    import json
    metadata=json.loads(tree.find(".//Private[@entityRef='crossing']//Property[@name='C2XGeneratedLaneRoute']").get('value'))
    assert [(p['road_id'],p['lane_id'],p['s']) for p in metadata]==[(1,-1,160),(3,1,160)]
    stop = tree.find(".//Event[@name='crossing_route_stop']")
    assert stop.find('.//StoryboardElementStateCondition').get('storyboardElementRef') == 'crossing_cross'
    assert float(stop.find('.//AbsoluteTargetSpeed').get('value')) == 0
    spawn = tree.find(".//Private[@entityRef='hero']//TeleportAction//LanePosition")
    assert float(spawn.get('s')) == 150
    hero_points = tree.findall(".//ManeuverGroup[@name='hero_grp']//AssignRouteAction//WorldPosition")
    assert len(hero_points) == 2
    assert (float(hero_points[-1].get('x')), float(hero_points[-1].get('y'))) == (230, 80)


def test_ads_relative_crossing_installs_route_before_waiting_for_departure(junction, tmp_path):
    import xmlschema
    road, xodr, scene, _ = junction
    scene['npcs'][0]['behavior']['ads_trigger'] = {'distance_m': 12}
    output = tmp_path/'relative.xosc'
    osc_blocks.build_xosc(scene, str(xodr), output, name='test', road_seed=road)
    tree = ET.parse(output)
    assert float(tree.find(".//Private[@entityRef='crossing']//AbsoluteTargetSpeed").get('value')) == 0
    assert tree.find(".//Event[@name='crossing_cross']//SimulationTimeCondition") is not None
    assert tree.find(".//Event[@name='crossing_depart']//RelativeDistanceCondition") is not None
    assert tree.find(".//Event[@name='crossing_depart']//SimulationTimeCondition") is None
    assert float(tree.find(".//Event[@name='crossing_depart']//SpeedActionDynamics").get('value')) >= 8/3
    xmlschema.XMLSchema(osc_blocks.ROOT/'xsd/OpenSCENARIO.xsd').validate(output)


def test_rounded_connector_endpoint_uses_sampled_world_transform(tmp_path):
    w = {"connector_end": {"road_id": 101, "lane_id": -1, "s": 60.0,
                           "transform": {"x": 260.0, "y": -1.75, "z": 0, "yaw": 0}}}
    w["connector_start"] = {"road_id": 101, "lane_id": -1, "s": 0.0,
                            "transform": {"x": 200.0, "y": -1.75, "z": 0, "yaw": 0}}
    action = osc_blocks._junction_route_action(w, {"waypoint_ids": ["connector_start", "connector_end"]}, "npc")
    point = action.get_element().findall('.//WorldPosition')[-1]
    assert float(point.get('x')) == 260
    assert float(point.get('y')) == -1.75
    assert action.get_element().find('.//LanePosition') is None


def test_left_route_name_from_actual_extractor_is_honored(junction, tmp_path):
    road, xodr, scene, routes = junction
    # Put a right route first: the old code silently selected it because it
    # expected "left_turn" while the extractor emits "left".
    right = copy.deepcopy(routes[0])
    right['type'] = 'right'
    right['start_road_id'] = 4
    routes[0]['type'] = 'left'
    routes.insert(0, right)
    scene['sut']['maneuver'] = 'left'
    output = tmp_path / 'left.xosc'
    osc_blocks.build_xosc(scene, str(xodr), output, name='test', auto_extract=False, road_seed=road)
    position = ET.parse(output).find(".//Private[@entityRef='hero']//TeleportAction//LanePosition")
    assert position.get('roadId') == '2'


def test_missing_requested_route_is_rejected(junction, tmp_path):
    road, xodr, scene, routes = junction
    scene['sut']['maneuver'] = 'left'
    with pytest.raises(osc_blocks.BlockUnsupported, match='no left_turn route'):
        osc_blocks.build_xosc(scene, str(xodr), tmp_path/'bad.xosc', name='test', auto_extract=False, road_seed=road)


def test_same_lane_scene_also_routes_sut_through_generated_intersection(junction, tmp_path):
    road, xodr, scene, routes = junction
    scene['npcs'] = [{'id':'rear','kind':'vehicle','position':'behind_same_lane',
                     'behavior':{'block':'rear_hit'}}]
    scene['collision'] = {'a':'rear','b':'ego'}
    output=tmp_path/'straight_across.xosc'
    osc_blocks.build_xosc(scene,str(xodr),output,name='test',road_seed=road)
    points=ET.parse(output).findall(".//ManeuverGroup[@name='hero_grp']//WorldPosition")
    assert [(float(p.get('x')),float(p.get('y'))) for p in points]==[(230,-80),(230,80)]


def test_stopped_cross_leg_actor_is_placed_at_approach_end(junction, tmp_path, monkeypatch):
    road, xodr, scene, routes = junction
    W, _ = osc_blocks.load_roadgraph('test')
    # The SUT approaches northbound and turns west; the stopped vehicle is
    # eastbound on that western leg, not on the SUT's original road.
    routes[0]['type'] = 'left'
    W['stop_line'] = {'road_id': 1, 'lane_id': -1, 's': 190,
                      'transform': {'x': 190, 'y': -1.75, 'z': 0, 'yaw': 0}}
    routes[1]['approach_waypoint_ids'].append('stop_line')
    scene['sut']['maneuver'] = 'left'
    scene['npcs'][0].update(position='cross', side='left', behavior={'block':'static_hold'})
    output = tmp_path/'stopped_cross_leg.xosc'
    osc_blocks.build_xosc(scene, str(xodr), output, name='test', road_seed=road)
    tree = ET.parse(output)
    spawn = tree.find(".//Private[@entityRef='crossing']//WorldPosition")
    assert (float(spawn.get('x')), float(spawn.get('y')), float(spawn.get('h'))) == (190, -1.75, 0)
    assert tree.find(".//Event[@name='crossing_cross']") is None
    assert tree.find(".//Private[@entityRef='crossing']//ControllerAction") is None


def test_junction_route_checks_adjacent_lane_in_same_travel_direction():
    scene = {'npcs': [{'id': 'neighbor', 'position': 'adjacent', 'side': 'left'}]}
    available = {'1': {-2, -1, 1, 2}}
    assert not osc_blocks._route_supports_relative_positions(
        {'start_road_id': 1, 'start_lane_id': -1}, scene, available)
    assert osc_blocks._route_supports_relative_positions(
        {'start_road_id': 1, 'start_lane_id': -2}, scene, available)
    assert not osc_blocks._route_supports_relative_positions(
        {'start_road_id': 1, 'start_lane_id': 1}, scene, available)
    assert osc_blocks._route_supports_relative_positions(
        {'start_road_id': 1, 'start_lane_id': 2}, scene, available)


def test_adjacent_turn_merge_uses_generated_connector_to_common_exit(tmp_path, monkeypatch):
    road = {'road': {'topology': 'cross_intersection', 'type': 'town',
                    'lanes': {'forward': 2, 'backward': 1}, 'center_line': 'broken'}}
    xodr = tmp_path/'junction.xodr'
    build(road, xodr, tmp_path/'map.html', osc_blocks.ROOT/'xsd/OpenDRIVE_1.5M.xsd')
    W = {
        'outer': {'road_id': 1, 'lane_id': -2, 's': 150,
                  'transform': {'x': 150, 'y': 5.25, 'z': 0, 'yaw': 0}},
        'inner': {'road_id': 1, 'lane_id': -1, 's': 150,
                  'transform': {'x': 150, 'y': 1.75, 'z': 0, 'yaw': 0}},
        'exit': {'road_id': 2, 'lane_id': 1, 's': 150,
                 'transform': {'x': 230, 'y': 60, 'z': 0, 'yaw': 90}},
    }
    routes = [{'start_road_id': 1, 'start_lane_id': lane, 'type': 'right',
               'waypoint_ids': [wid, 'exit'], 'approach_waypoint_ids': [wid]}
              for lane, wid in [(-2, 'outer'), (-1, 'inner')]]
    monkeypatch.setattr(osc_blocks, 'load_roadgraph', lambda name: (W, routes))
    npc = {'id': 'truck', 'kind': 'vehicle', 'vehicle_class': 'truck',
           'position': 'adjacent', 'side': 'left', 'behavior': {'block': 'junction_merge'}}
    scene = {'sut': {'id': 'ego', 'kind': 'vehicle', 'maneuver': 'right'},
             'npcs': [npc], 'collision': {'a': 'truck', 'b': 'ego'}}
    output = tmp_path/'merge.xosc'
    osc_blocks.build_xosc(scene, str(xodr), output, name='test', road_seed=road)
    tree = ET.parse(output)
    assert tree.find('.//LaneChangeAction') is None
    points = tree.findall(".//Event[@name='truck_cross']//AssignRouteAction//WorldPosition")
    assert [(float(p.get('x')), float(p.get('y'))) for p in points] == [(150, 1.75), (230, 60)]
    assert tree.find(".//ScenarioObject[@name='truck']/Vehicle").get('name') == 'vehicle.carlamotors.european_hgv'
    import xmlschema
    xmlschema.XMLSchema(osc_blocks.XSD).validate(output)
    # A missing merge lane must not select an unrelated turn or fake a lane change.
    with pytest.raises(osc_blocks.BlockUnsupported, match='no adjacent junction route'):
        osc_blocks._junction_merge_route(W, routes[:1], routes[0], npc)
