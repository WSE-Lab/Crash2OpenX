import copy
from xml.etree import ElementTree as ET

import pytest

from tools import osc_blocks

pytest_plugins = ['tests.test_junction_route_compilation']


def test_nonjunction_curve_uses_all_connected_lane_samples():
    W = {str(i): {'road_id': 2, 'lane_id': -1, 's': float(i),
                 'lane_type': 'Driving', 'is_junction': False,
                 'next_ids': [str(i+1)] if i < 6 else []} for i in range(7)}
    route = osc_blocks._corridor_route_from_spawn(W, 2, -1, 2.5)
    assert route['waypoint_ids'] == ['3', '4', '5', '6']
    W['4']['next_ids'].append('6')
    with pytest.raises(osc_blocks.BlockUnsupported, match='branch'):
        osc_blocks._corridor_route_from_spawn(W, 2, -1, 2.5)


def test_turning_lead_uses_ego_approach_with_positive_spacing(junction, tmp_path):
    road, xodr, scene, routes = junction
    W, _ = osc_blocks.load_roadgraph('test')
    original = routes[0]
    start, end = original['waypoint_ids']
    lead = 'same_lane_lead'
    W[lead] = {**copy.deepcopy(W[start]), 's': 175,
               'transform': {'x':230,'y':-55,'z':0,'yaw':90}}
    turn = {**copy.deepcopy(original), 'type':'left',
            'waypoint_ids':[start,lead,end], 'approach_waypoint_ids':[start,lead]}
    routes.append(turn)
    scene['npcs'][0].update(position='ahead_same_lane', side='none',
        behavior={'block':'junction_turn', 'params':{'gap':20}})
    path=tmp_path/'ahead_turn.xosc'
    osc_blocks.build_xosc(scene,str(xodr),path,name='test',road_seed=road)
    tree=ET.parse(path)
    spawn=tree.find(".//Private[@entityRef='crossing']//TeleportAction//WorldPosition")
    assert (float(spawn.get('x')),float(spawn.get('y'))) == (230,-55)
    points=tree.findall(".//Event[@name='crossing_cross']//WorldPosition")
    assert len(points)==2
    assert float(points[0].get('y')) == -55


@pytest.mark.parametrize('position', ['oncoming', 'opposing_leg'])
def test_opposing_turn_does_not_select_cross_leg(junction, tmp_path, position):
    road, xodr, scene, routes = junction
    W, _ = osc_blocks.load_roadgraph('test')
    # A cross-leg turn appears first, as it does in generated route caches.
    routes[1]['type'] = 'left'
    W['opposing_start'] = {'road_id': 4, 'lane_id': -1, 's': 150,
        'transform': {'x': 228.25, 'y': 80, 'z': 0, 'yaw': 270}}
    routes.append({'type': 'left', 'start_road_id': 4, 'start_lane_id': -1,
        'waypoint_ids': ['opposing_start', routes[1]['waypoint_ids'][-1]],
        'approach_waypoint_ids': ['opposing_start']})
    scene['npcs'][0].update(position=position, side='none',
        behavior={'block': 'junction_turn'})
    path = tmp_path/'opposing_turn.xosc'
    osc_blocks.build_xosc(scene, str(xodr), path, name='test', road_seed=road)
    tree = ET.parse(path)
    spawn = tree.find(".//Private[@entityRef='crossing']//TeleportAction//WorldPosition")
    assert (float(spawn.get('x')), float(spawn.get('y'))) == (228.25, 80)
    routes.pop()
    with pytest.raises(osc_blocks.BlockUnsupported, match='no NPC route matching leg'):
        osc_blocks.build_xosc(scene, str(xodr), path, name='test', road_seed=road)


def test_measured_approach_rejects_wrong_incoming_road():
    from tools.review_ads_expansion import approach_matches
    ego = {'x': 0, 'y': 0, 'yaw': 0}
    npc = {'position': 'oncoming', 'behavior': {'block': 'junction_turn'}}
    assert approach_matches(npc, ego, {'x': 30, 'y': 3, 'yaw': 180})
    assert not approach_matches(npc, ego, {'x': 30, 'y': 30, 'yaw': -90})
    npc.update(position='cross', side='right')
    assert approach_matches(npc, ego, {'x': 30, 'y': 30, 'yaw': -90})
    assert not approach_matches(npc, ego, {'x': 30, 'y': -30, 'yaw': 90})


def test_optional_brake_cap_is_explicit_and_validated():
    from tools.replay_scene_tools import _add_vehicle_controller
    from scenariogeneration import xosc
    default = xosc.Init()
    _add_vehicle_controller(default, 'lead')
    assert default.get_element().find(".//Property[@name='C2XMaxBrake']") is None
    calibrated = xosc.Init()
    _add_vehicle_controller(calibrated, 'lead', max_brake=.02)
    assert float(calibrated.get_element().find(".//Property[@name='C2XMaxBrake']").get('value')) == .02
    for invalid in [0, -1, 1.1, True, float('nan')]:
        with pytest.raises(ValueError, match='max_brake'):
            _add_vehicle_controller(xosc.Init(), 'lead', max_brake=invalid)
