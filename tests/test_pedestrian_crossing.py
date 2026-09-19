import json
from xml.etree import ElementTree as ET

import pytest
import xmlschema

from tools.build_road_seed_opendrive import build
from tools.osc_blocks import ROOT, build_xosc
from tools.pedestrian_crossing import crossing_geometry


@pytest.fixture
def generated_road(tmp_path):
    seed = {'road': {'topology':'straight', 'type':'town',
                     'lanes':{'forward':2,'backward':1}, 'center_line':'broken'}}
    path=tmp_path/'new.xodr'
    build(seed,path,tmp_path/'new.html',ROOT/'xsd/OpenDRIVE_1.5M.xsd')
    return seed,path


@pytest.mark.parametrize('lane_id,side,start_t,end_t', [
    (-2,'right',-7.75,4.25), (-1,'left',4.25,-7.75),
    (1,'right',4.25,-7.75), (1,'left',-7.75,4.25)])
def test_full_crossing_spans_driving_lanes_and_both_curb_clearances(generated_road,lane_id,side,start_t,end_t):
    _,path=generated_road
    plan=crossing_geometry(path,1,lane_id,80,side=side)
    assert plan['start_t']==start_t
    assert plan['end_t']==end_t
    assert plan['distance_m']==12
    assert plan['s']==(105 if lane_id<0 else 55)


def test_explicit_report_lateral_is_not_replaced(generated_road):
    _,path=generated_road
    plan=crossing_geometry(path,1,-2,80,side='right',lateral=3.5)
    assert plan['start_offset']==-3.5
    assert plan['distance_m']==13
    assert plan['source_lateral_preserved']


def test_unspecified_source_side_remains_a_disclosed_compiler_default(generated_road):
    _, path = generated_road
    plan = crossing_geometry(path, 1, -2, 80, side='none')
    assert plan['source_side'] == 'none'
    assert plan['side'] == 'right'
    assert plan['side_is_compiler_default']
    assert plan['start_t'] == -7.75


def test_missing_walkable_endpoint_and_road_end_are_rejected(generated_road,tmp_path):
    _,path=generated_road
    with pytest.raises(ValueError,match='beyond'):
        crossing_geometry(path,1,-2,199,side='right')
    tree=ET.parse(path)
    for side in tree.findall('.//laneSection/left')+tree.findall('.//laneSection/right'):
        for lane in list(side):
            if lane.get('type')!='driving':side.remove(lane)
    missing=tmp_path/'no_shoulder.xodr';tree.write(missing)
    with pytest.raises(ValueError,match='walkable'):
        crossing_geometry(missing,1,-2,80,side='right')


def test_compiled_crossing_stops_walker_without_scripted_sut_path(generated_road,tmp_path):
    seed,path=generated_road
    scene={'sut':{'id':'ego','kind':'vehicle','maneuver':'straight'},
           'npcs':[{'id':'walker','kind':'pedestrian','position':'roadside','side':'right',
                    'behavior':{'block':'cross'}}], 'collision':{'a':'ego','b':'walker'}}
    output=tmp_path/'cross.xosc';build_xosc(scene,str(path),output,road_seed=seed)
    tree=ET.parse(output)
    plan=json.loads(tree.find(".//ParameterDeclaration[@name='C2XPedestrianCrossings']").get('value'))['walker']
    stop=tree.find(".//Event[@name='walker_cross_stop']")
    assert float(stop.find('.//TraveledDistanceCondition').get('value'))==12
    assert stop.find('.//TriggeringEntities/EntityRef').get('entityRef')=='walker'
    assert float(stop.find('.//AbsoluteTargetSpeed').get('value'))==0
    spawn=tree.find(".//Private[@entityRef='walker']//LanePosition")
    assert float(spawn.get('offset'))==plan['start_offset']
    assert tree.find(".//Private[@entityRef='hero']//Property[@value='external_control']") is not None
    assert tree.find('.//FollowTrajectoryAction') is None
    xmlschema.XMLSchema(ROOT/'xsd/OpenSCENARIO.xsd').validate(output)


def test_ads_window_does_not_wait_for_stationary_walker_to_move(generated_road,tmp_path):
    seed, path = generated_road
    scene = {'sut': {'id': 'ego', 'kind': 'vehicle', 'maneuver': 'straight'},
             'npcs': [{'id': 'walker', 'kind': 'pedestrian', 'position': 'roadside', 'side': 'right',
                       'behavior': {'block': 'cross', 'ads_trigger': {'distance_m': 8}}}],
             'collision': {'a': 'ego', 'b': 'walker'}}
    output = tmp_path/'relative_cross.xosc'
    build_xosc(scene, str(path), output, road_seed=seed)
    tree = ET.parse(output)
    start = tree.find(".//Event[@name='walker_move']")
    assert start.find('.//SimulationTimeCondition') is None
    assert {c.get('entityRef') for c in start.findall('.//TriggeringEntities/EntityRef')} == {'hero'}
    assert tree.find(".//Event[@name='walker_cross_stop']//TraveledDistanceCondition") is not None
    xmlschema.XMLSchema(ROOT/'xsd/OpenSCENARIO.xsd').validate(output)
