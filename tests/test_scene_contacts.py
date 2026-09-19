import copy
import json
from pathlib import Path

import pytest
import xmlschema

from tools.scene_contacts import normalize_contacts, read_xosc_contacts, evaluate_contact_sequence
from tools.api_infer_scene_seed_v2 import normalize
from tools.ocl_constraints import evaluate_constraints
from tools.build_road_seed_opendrive import build
from tools.osc_blocks import build_xosc, ROOT


def chain():
    return {'sut': {'id': 'ego', 'kind': 'vehicle', 'maneuver': 'straight'},
            'npcs': [{'id': 'middle', 'kind': 'vehicle', 'position': 'ahead_same_lane',
                      'behavior': {'block': 'static_hold'}},
                     {'id': 'front', 'kind': 'vehicle', 'position': 'ahead_same_lane',
                      'behavior': {'block': 'static_hold', 'params': {'gap': 40}}}],
            'collision': {'a': 'ego', 'b': 'middle'},
            'collisions': [{'a': 'ego', 'b': 'middle'}, {'a': 'middle', 'b': 'front'}]}


def contact(t, a, b):
    return {'event_type': 'collision', 'simulation_time': t,
            'payload': {'actors': [a, b], 'source': 'carla_collision_sensor'}}


def test_missing_contact_expectations_cannot_pass_sensor_validation():
    assert not evaluate_contact_sequence([], [])['sensor_sequence_pass']
    assert not evaluate_contact_sequence([], [contact(1,'a','b')])['sensor_sequence_pass']


def test_all_contacts_survive_inference_compilation_and_runtime_metadata(tmp_path):
    scene = normalize({'status': 'supported', 'scene': chain()}, tmp_path/'source.pdf', 'test')['scene']
    assert scene['collisions'] == chain()['collisions']
    road = {'road': {'topology': 'straight', 'type': 'town', 'lanes': {'forward': 1, 'backward': 1}, 'center_line': 'broken'}}
    xodr = tmp_path/'fresh.xodr'
    build(road, xodr, tmp_path/'map.html', ROOT/'xsd/OpenDRIVE_1.5M.xsd')
    out = tmp_path/'contacts.xosc'
    build_xosc(scene, str(xodr), out, road_seed=road)
    xmlschema.XMLSchema(ROOT/'xsd/OpenSCENARIO.xsd').validate(out)
    assert read_xosc_contacts(out) == [{'a': 'hero', 'b': 'middle'}, {'a': 'middle', 'b': 'front'}]
    # Expectations must not become a synthetic collision, teleport sequence,
    # trajectory follower, or forced post-impact speed/rotation action.
    from xml.etree import ElementTree as ET
    tree = ET.parse(out)
    assert len(tree.findall('.//TeleportAction')) == 3  # Init only
    assert tree.find('.//FollowTrajectoryAction') is None


def test_queue_references_compile_in_dependency_order_instead_of_overlapping(tmp_path):
    original = chain()
    original['npcs'][1]['relative_to'] = 'middle'
    original['npcs'].reverse()  # Input order is not spawn dependency order.
    scene = normalize({'status': 'supported', 'scene': original}, tmp_path/'source.pdf', 'test')['scene']
    road = {'road': {'topology': 'straight', 'type': 'town', 'lanes': {'forward': 1, 'backward': 1}, 'center_line': 'broken'}}
    xodr = tmp_path/'fresh.xodr'
    build(road, xodr, tmp_path/'map.html', ROOT/'xsd/OpenDRIVE_1.5M.xsd')
    out = tmp_path/'queue.xosc'; build_xosc(scene, str(xodr), out, road_seed=road)
    from xml.etree import ElementTree as ET
    root = ET.parse(out)
    assert [n.get('name') for n in root.findall('./Entities/ScenarioObject')] == ['hero','middle','front']
    pos = root.find(".//Private[@entityRef='front']//RelativeLanePosition")
    assert pos.get('entityRef') == 'middle'
    xmlschema.XMLSchema(ROOT/'xsd/OpenSCENARIO.xsd').validate(out)


@pytest.mark.parametrize('bad_ref', ['missing','front'])
def test_missing_or_cyclic_position_reference_is_rejected(tmp_path, bad_ref):
    scene=chain();scene['npcs'][0]['relative_to']=bad_ref;scene['npcs'][1]['relative_to']='middle'
    with pytest.raises(ValueError):
        normalize({'status':'supported','scene':scene},tmp_path/'source.pdf','test')


def test_secondary_pair_cannot_bypass_I6_or_normalization():
    scene = chain()
    scene['collisions'][1]['b'] = 'missing'
    road = {'lanes': {'forward': 1, 'backward': 1}, 'topology': 'straight'}
    assert evaluate_constraints(road, scene)['I6'] is False
    with pytest.raises(ValueError, match='declared actors'):
        normalize_contacts(scene, {'ego', 'middle', 'front'})


def test_compatibility_pair_cannot_conflict_with_ordered_sequence():
    scene = chain(); scene['collision'] = {'a': 'ego', 'b': 'front'}
    with pytest.raises(ValueError, match='first collisions pair'):
        normalize_contacts(scene, {'ego', 'middle', 'front'})


@pytest.mark.parametrize('events,passed', [
    ([contact(1, 'rear', 'middle'), contact(2, 'middle', 'front')], True),
    ([contact(1, 'middle', 'front'), contact(2, 'rear', 'middle')], False),
    ([contact(1, 'rear', 'middle')], False),
    ([contact(1, 'rear', 'front'), contact(2, 'middle', 'front')], False),
    ([contact(1, 'rear', 'middle'), contact(1, 'middle', 'front')], False),
])
def test_ordered_sensor_evidence_cannot_be_replaced_by_aggregate_count(events, passed):
    required = [{'a': 'rear', 'b': 'middle'}, {'a': 'middle', 'b': 'front'}]
    result = evaluate_contact_sequence(required, events)
    assert result['sensor_sequence_pass'] is passed
    assert result['source_fidelity_accepted'] is False


def test_sustained_contact_is_not_two_impacts_and_time_gap_is_not_separation_proof():
    required = [{'a': 'rear', 'b': 'front'}] * 2
    events = [contact(i/20, 'rear', 'front') for i in range(20, 80)]
    assert not evaluate_contact_sequence(required, events)['sensor_sequence_pass']
    result = evaluate_contact_sequence(required, [contact(1, 'rear', 'front'), contact(3, 'rear', 'front')])
    assert result['sensor_sequence_pass']
    assert result['physical_separation_requires_trace_review']
    assert not result['source_fidelity_accepted']
