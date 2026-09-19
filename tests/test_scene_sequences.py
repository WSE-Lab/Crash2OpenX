import copy
from xml.etree import ElementTree as ET

import pytest
import xmlschema

from tools.api_infer_scene_seed_v2 import normalize
from tools.build_road_seed_opendrive import build
from tools.osc_blocks import build_xosc, ROOT
from tools.scene_modeling import infer_scene_model
from tools.scene_sequences import normalize_steps


def staged_scene():
    return {'sut': {'id': 'ego', 'kind': 'vehicle', 'maneuver': 'straight'},
            'npcs': [{'id': 'rear', 'kind': 'vehicle', 'position': 'behind_same_lane',
                'behavior': {'block': 'sequence', 'steps': [
                    {'action': 'drive', 'params': {'speed': 10}, 'when': {'condition': 'start'}},
                    {'action': 'match_speed', 'target': 'ego', 'when': {'condition': 'contact', 'target': 'ego'}},
                    {'action': 'brake', 'params': {'end_speed': 0}, 'when': {'condition': 'after_previous', 'delay': 1}},
                    {'action': 'drive', 'params': {'speed': 10}, 'when': {'condition': 'separated_and_target_stopped', 'target': 'ego'}},
                ]}}],
            'collision': {'a': 'rear', 'b': 'ego'},
            'collisions': [{'a': 'rear', 'b': 'ego'}, {'a': 'rear', 'b': 'ego'}]}


def test_complete_sequence_survives_normalization_modeling_and_standard_osc(tmp_path):
    scene = normalize({'status': 'supported', 'scene': staged_scene()}, tmp_path/'input.pdf', 'test')['scene']
    steps = scene['npcs'][0]['behavior']['steps']
    assert [s['action'] for s in steps] == ['drive', 'match_speed', 'brake', 'drive']
    model = infer_scene_model({'scene': scene}, name='test')
    assert [p.action.type for p in model.phases if p.actor_id == 'rear'] == ['drive', 'match_speed', 'brake', 'drive']
    road = {'road': {'topology': 'straight', 'type': 'town', 'lanes': {'forward': 1, 'backward': 1}, 'center_line': 'broken'}}
    xodr = tmp_path/'fresh.xodr'
    build(road, xodr, tmp_path/'map.html', ROOT/'xsd/OpenDRIVE_1.5M.xsd')
    out = tmp_path/'sequence.xosc'; build_xosc(scene, str(xodr), out, road_seed=road)
    xmlschema.XMLSchema(ROOT/'xsd/OpenSCENARIO.xsd').validate(out)
    tree = ET.parse(out)
    contact_step = tree.find(".//Event[@name='rear_step_2']")
    assert contact_step.find('.//CollisionCondition/EntityRef').get('entityRef') == 'hero'
    assert contact_step.find('.//RelativeTargetSpeed').get('entityRef') == 'hero'
    assert contact_step.find('.//RelativeTargetSpeed').get('continuous') == 'false'
    resume = tree.find(".//Event[@name='rear_step_4']")
    assert resume.find('.//RelativeDistanceCondition').get('rule') == 'greaterThan'
    assert resume.find('.//RelativeDistanceCondition').get('freespace') == 'true'
    assert resume.find('.//StandStillCondition') is not None
    assert resume.find('.//StoryboardElementStateCondition').get('storyboardElementRef') == 'rear_step_3'
    assert len(tree.findall('.//TeleportAction')) == 2
    assert tree.find('.//FollowTrajectoryAction') is None


@pytest.mark.parametrize('mutation', ['unknown_action','unknown_target','negative_speed','reset_start','silent_rotation'])
def test_sequence_cannot_silently_drop_unsupported_actions_or_targets(tmp_path, mutation):
    scene = staged_scene(); steps = scene['npcs'][0]['behavior']['steps']
    if mutation == 'unknown_action': steps[1]['action'] = 'force_contact'
    if mutation == 'unknown_target': steps[1]['target'] = 'missing'
    if mutation == 'negative_speed': steps[0]['params']['speed'] = -1
    if mutation == 'reset_start': steps[2]['when']['condition'] = 'start'
    if mutation == 'silent_rotation': steps[2]['yaw'] = 90
    with pytest.raises(ValueError):
        normalize({'status': 'supported', 'scene': scene}, tmp_path/'source.pdf', 'test')
