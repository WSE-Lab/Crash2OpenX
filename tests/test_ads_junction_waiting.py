import copy
import json
from xml.etree import ElementTree as ET

import pytest
import xmlschema

from tools import osc_blocks
pytest_plugins = ['tests.test_junction_route_compilation']


def test_waiting_actor_spawn_route_and_controller_share_same_anchor(junction, tmp_path):
    road, xodr, scene, routes = junction
    W, _ = osc_blocks.load_roadgraph('test')
    route = routes[1]
    start, end = route['waypoint_ids']
    for x in [170, 180, 190]:
        W[str(x)] = {**copy.deepcopy(W[start]), 's': x,
                     'transform': {'x': x, 'y': -1.75, 'z': 0, 'yaw': 0}}
    route['approach_waypoint_ids'] = [start, '170', '180', '190']
    route['waypoint_ids'] = [*route['approach_waypoint_ids'], end]
    before = copy.deepcopy(route)
    scene['npcs'][0]['behavior'].update(params={'approach_distance_m': 10}, ads_trigger={'distance_m': 22})
    out = tmp_path/'scene.xosc'
    osc_blocks.build_xosc(scene, str(xodr), out, name='test', road_seed=road)
    xmlschema.XMLSchema(osc_blocks.ROOT/'xsd/OpenSCENARIO.xsd').validate(out)
    tree = ET.parse(out)
    spawn = tree.find(".//Private[@entityRef='crossing']//TeleportAction//WorldPosition")
    assigned = tree.findall(".//Event[@name='crossing_cross']//Waypoint//WorldPosition")
    generated = json.loads(tree.find(".//Private[@entityRef='crossing']//Property[@name='C2XGeneratedLaneRoute']").get('value'))
    assert float(spawn.get('x')) == 180
    assert [float(p.get('x')) for p in assigned] == [180, 190, 300]
    assert [p['x'] for p in generated] == [180, 190, 300]
    assert routes[1] == before
    assert tree.find(".//Event[@name='crossing_depart']//RelativeDistanceCondition") is not None
    assert tree.find('.//FollowTrajectoryAction') is None


@pytest.mark.parametrize('distance', [True, -1, 0, 4, float('nan'), float('inf'), 1000])
def test_invalid_or_unavailable_waiting_distance_is_rejected(junction, distance):
    _, _, _, routes = junction
    W, _ = osc_blocks.load_roadgraph('test')
    with pytest.raises(osc_blocks.BlockUnsupported):
        osc_blocks._waiting_junction_route(W, routes[1], distance)
