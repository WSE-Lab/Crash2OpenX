import json
from xml.etree import ElementTree as ET

import pytest
import xmlschema

from tools.api_infer_scene_seed_v2 import normalize
from tools.build_road_seed_opendrive import build
from tools.osc_blocks import ROOT, BlockUnsupported, build_xosc
from tools.osc_blocks import _straight_route_from_spawn
from tools.partial_lane_intrusion import intrusion_plan


@pytest.fixture
def road(tmp_path):
    seed={'road':{'topology':'straight','type':'town','lanes':{'forward':2,'backward':2},'center_line':'broken'}}
    path=tmp_path/'new.xodr'
    build(seed,path,tmp_path/'new.html',ROOT/'xsd/OpenDRIVE_1.5M.xsd')
    return seed,path


def actor(side='right'):
    return {'id':'moving_car','kind':'vehicle','vehicle_class':'car','position':'adjacent','side':side,
            'behavior':{'block':'partial_lane_intrusion','params':{}}}


@pytest.mark.parametrize('lane,side,target', [(-2,'right',1.4),(-1,'left',-1.4),(2,'right',-1.4),(1,'left',1.4)])
def test_both_lane_directions_use_partial_target_toward_other_vehicle(road,lane,side,target):
    _,path=road
    plan=intrusion_plan(actor(side),path,1,lane)
    assert plan['target_offset_xodr_m']==pytest.approx(target)
    assert abs(plan['target_offset_xodr_m']) < plan['lane_width_m']/2
    assert 'offset_fraction' in plan['default_parameters']


@pytest.mark.parametrize('value', [.5,1,-.1,float('nan')])
def test_complete_lane_change_or_invalid_fraction_is_rejected(road,value):
    _,path=road;data=actor();data['behavior']['params']['offset_fraction']=value
    with pytest.raises(ValueError,match='initial side'):
        intrusion_plan(data,path,1,-2)


def test_missing_adjacent_driving_lane_is_rejected(road):
    _,path=road
    with pytest.raises(ValueError,match='same-direction'):
        intrusion_plan(actor('left'),path,1,-2)


def test_source_roles_cyclist_and_partial_motion_survive_compilation(road,tmp_path):
    data={'status':'supported','scene':{
        'sut':{'id':'ego','kind':'vehicle','vehicle_class':'truck','maneuver':'straight'},
        'npcs':[actor(),{'id':'rider','kind':'cyclist','relative_to':'moving_car','position':'roadside',
                         'side':'right','behavior':{'block':'cruise','params':{}}}],
        'collision':{'a':'ego','b':'moving_car'},'control':'none'}}
    scene=normalize(data,tmp_path/'original.pdf','unit-test')['scene']
    seed,path=road;output=tmp_path/'new.xosc'
    build_xosc(scene,str(path),output,road_seed=seed)
    tree=ET.parse(output)
    assert tree.find(".//ScenarioObject[@name='hero']/Vehicle").get('name')=='vehicle.carlamotors.european_hgv'
    assert tree.find(".//Private[@entityRef='hero']//Property[@value='external_control']") is not None
    assert tree.find('.//LaneChangeAction') is None
    action=tree.find('.//LaneOffsetAction')
    assert action.get('continuous')=='true'
    assert action.find('.//LaneOffsetActionDynamics').get('dynamicsShape')=='sinusoidal'
    assert float(action.find('.//AbsoluteTargetLaneOffset').get('value'))==pytest.approx(1.4)
    rider=tree.find(".//Private[@entityRef='rider']//RelativeLanePosition")
    assert rider.get('entityRef')=='moving_car' and float(rider.get('ds'))==0
    assert tree.find(".//Private[@entityRef='rider']//Property[@name='C2XInitialLaneOffset']").get('value')=='3.5'
    assert 'moving_car' in json.loads(tree.find(".//ParameterDeclaration[@name='C2XPartialLaneIntrusions']").get('value'))
    xmlschema.XMLSchema(ROOT/'xsd/OpenSCENARIO.xsd').validate(output)


def test_straight_continuation_uses_lane_identity_and_discards_points_behind_spawn():
    nodes={str(i):{'road_id':1 if i<4 else 10,'lane_id':-2,'s':float(i*2)} for i in range(7)}
    straight={'type':'straight','start_road_id':1,'start_lane_id':-2,'waypoint_ids':list(nodes)}
    wrong={**straight,'type':'left'}
    route=_straight_route_from_spawn(nodes,[wrong,straight],1,-2,5)
    assert route['type']=='straight'
    assert route['waypoint_ids']==['3','4','5','6']
    assert straight['waypoint_ids']==list(nodes)


def test_continuation_follows_departure_lane_beyond_short_route_candidate():
    nodes={str(i):{'road_id':1 if i<2 else 4, 'lane_id':-2 if i<2 else 2,
                   's':float(i*2), 'next_ids':[str(i+1)] if i<5 else []} for i in range(6)}
    route={'type':'straight','start_road_id':1,'start_lane_id':-2,
           'waypoint_ids':['0','1','2','3']}
    result=_straight_route_from_spawn(nodes,[route],1,-2,0)
    assert result['waypoint_ids']==list(nodes)
    assert route['waypoint_ids']==['0','1','2','3']
    # A next junction branch is not silently chosen for the actor.
    nodes['5']['next_ids']=['6']
    nodes['6']={'road_id':8,'lane_id':2,'s':0,'next_ids':[]}
    assert _straight_route_from_spawn(nodes,[route],1,-2,0)['waypoint_ids']==list(nodes)[:-1]


@pytest.mark.parametrize('tail', [['2'], ['3','4']])
def test_continuation_rejects_cycles_and_ambiguous_same_lane_edges(tail):
    nodes={str(i):{'road_id':1,'lane_id':-1,'s':float(i),'next_ids':[]} for i in range(5)}
    nodes['2']['next_ids']=tail
    route={'type':'straight','start_road_id':1,'start_lane_id':-1,'waypoint_ids':['0','1','2']}
    with pytest.raises(BlockUnsupported, match='Ambiguous or cyclic'):
        _straight_route_from_spawn(nodes,[route],1,-1,0)


def test_dangling_station_alias_continues_on_dense_native_lane_samples():
    stations=[10.,8.,6.02,6.,4.,2.,0.]
    nodes={str(i):{'road_id':4,'lane_id':2,'section_id':0,'s':s,'next_ids':[],
                   'transform':{'x':10.-s,'y':0.,'yaw':0.}} for i,s in enumerate(stations)}
    nodes['2']['next_ids']=['missing_s4.02']
    route={'type':'straight','start_road_id':4,'start_lane_id':2,'waypoint_ids':['0','1','2']}
    result=_straight_route_from_spawn(nodes,[route],4,2,10.)
    assert result['waypoint_ids']==['0','1','2','4','5','6']
    nodes['4']['transform']['y']=20.
    with pytest.raises(BlockUnsupported,match='Discontinuous'):
        _straight_route_from_spawn(nodes,[route],4,2,10.)
