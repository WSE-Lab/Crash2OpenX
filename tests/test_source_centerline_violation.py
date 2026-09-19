from xml.etree import ElementTree as ET

import pytest
import xmlschema

from tools.api_infer_scene_seed_v2 import normalize
from tools.build_road_seed_opendrive import build
from tools.ocl_constraints import evaluate_constraints
from tools.osc_blocks import ROOT, WFViolation, build_xosc


def scene():
    return {'sut': {'id':'ego', 'kind':'vehicle', 'vehicle_class':'bus',
                    'maneuver':'overtake_solid_centerline'},
            'npcs':[{'id':'stopped', 'kind':'vehicle', 'position':'ahead_same_lane',
                     'behavior':{'block':'stopped_ahead'}}],
            'collision':{'a':'ego','b':'stopped'}}


def test_source_violation_keeps_solid_road_and_external_sut_control(tmp_path):
    s = normalize({'status':'supported','scene':scene()}, tmp_path/'report.pdf','test')['scene']
    road = {'road':{'topology':'straight','type':'town',
                    'lanes':{'forward':1,'backward':1},'center_line':'solid'}}
    assert all(evaluate_constraints(road['road'],s).values())
    xodr=tmp_path/'fresh.xodr'
    build(road,xodr,tmp_path/'map.html',ROOT/'xsd/OpenDRIVE_1.5M.xsd')
    out=tmp_path/'source_violation.xosc'
    build_xosc(s,str(xodr),out,road_seed=road)
    xmlschema.XMLSchema(ROOT/'xsd/OpenSCENARIO.xsd').validate(out)
    tree=ET.parse(out)
    assert tree.find(".//ParameterDeclaration[@name='C2XSourceManeuver']").get('value') == 'overtake_solid_centerline'
    init=tree.find(".//Private[@entityRef='hero']//LanePosition")
    goal=tree.find(".//ManeuverGroup[@name='hero_grp']//LanePosition")
    assert (goal.get('roadId'),goal.get('laneId')) == (init.get('roadId'),init.get('laneId'))
    assert tree.find('.//FollowTrajectoryAction') is None
    assert tree.find('.//LaneChangeAction') is None
    assert tree.find(".//Private[@entityRef='hero']//Property[@value='external_control']") is not None
    assert tree.find(".//ScenarioObject[@name='hero']/Vehicle").get('vehicleCategory') == 'bus'
    assert all(mark.get('type') == 'solid' for mark in ET.parse(xodr).findall('.//center/lane/roadMark'))


@pytest.mark.parametrize('maneuver,center,backward', [
    ('overtake_oncoming','solid',1),
    ('overtake_solid_centerline','broken',1),
    ('overtake_solid_centerline','solid',0),
])
def test_source_annotation_does_not_bypass_road_consistency(tmp_path,maneuver,center,backward):
    s=scene();s['sut']['maneuver']=maneuver
    road={'road':{'topology':'straight','type':'town',
                  'lanes':{'forward':1,'backward':backward},'center_line':center}}
    assert not evaluate_constraints(road['road'],s)['P3']
    with pytest.raises(WFViolation,match='WF7'):
        build_xosc(s,str(tmp_path/'unused.xodr'),tmp_path/'rejected.xosc',road_seed=road)
