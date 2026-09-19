import sys
from xml.etree import ElementTree as ET

import pytest
import xmlschema

from tools.ads_trigger import normalize_ads_trigger
from tools.api_infer_scene_seed_v2 import normalize
from tools.build_road_seed_opendrive import build
from tools.osc_blocks import build_xosc, ROOT
from tools.scene_modeling import infer_scene_model
from runner.src.ads_geometry import hull, body_clearance, constant_velocity_ttc


def scene(block='front_brake'):
    return {'sut': {'id': 'ego', 'kind': 'vehicle', 'maneuver': 'straight'},
            'npcs': [{'id': 'lead', 'kind': 'vehicle',
                      'position': 'adjacent' if block == 'cut_in' else 'ahead_same_lane', 'side': 'left',
                      'behavior': {'block': block, 'params': {'speed': 6, 'brake_t': 1.2},
                                   'ads_trigger': {'distance_m': 8}}}],
            'collision': {'a': 'ego', 'b': 'lead'}, 'control': 'none'}


@pytest.mark.parametrize('block', ['front_brake', 'cut_in'])
def test_compiled_trigger_is_relative_live_and_keeps_ads_external(tmp_path, block):
    data = scene(block)
    normalized = normalize({'status': 'supported', 'scene': data}, tmp_path/'source.pdf', 'test')['scene']
    assert normalized['npcs'][0]['behavior']['ads_trigger']['distance_m'] == 8
    model = infer_scene_model({'scene': normalized}, name='test')
    phases = [p for p in model.phases if p.actor_id == 'lead']
    assert phases[-1].action.trigger.type == 'ads_relative_window'
    assert phases[-1].action.trigger.conditions['min_clearance_m'] == .5
    road = {'road': {'topology': 'straight', 'type': 'town', 'lanes': {'forward': 2, 'backward': 1}, 'center_line': 'broken'}}
    path = tmp_path/'road.xodr'
    build(road, path, tmp_path/'road.html', ROOT/'xsd/OpenDRIVE_1.5M.xsd')
    output = tmp_path/'scene.xosc'
    build_xosc(normalized, str(path), output, road_seed=road)
    xmlschema.XMLSchema(ROOT/'xsd/OpenSCENARIO.xsd').validate(output)
    tree = ET.parse(output)
    event = tree.find(".//Event[@name='lead_%s']" % ('brake' if block == 'front_brake' else 'cut'))
    assert event.find('.//SimulationTimeCondition') is None
    assert all(c.get('conditionEdge') == 'none' for c in event.findall('.//Condition'))
    assert {r.get('entityRef') for r in event.findall('.//RelativeDistanceCondition')} == {'lead'}
    assert event.find('.//TriggeringEntities/EntityRef').get('entityRef') == 'hero'
    assert tree.find(".//Private[@entityRef='hero']//Property[@value='external_control']") is not None
    assert tree.find('.//FollowTrajectoryAction') is None
    assert tree.find(".//ParameterDeclaration[@name='C2XADSActor']").get('value') == 'hero'


@pytest.mark.parametrize('config', [None, {}, {'distance_m': float('nan')}, {'distance_m': -1},
    {'distance_m': 3, 'min_clearance_m': 3}, {'distance_m': 5, 'unknown': 1}])
def test_bad_trigger_fails_closed(config):
    npc = scene()['npcs'][0]
    npc['behavior']['ads_trigger'] = config
    with pytest.raises(ValueError):
        normalize_ads_trigger(npc)


def test_ambiguous_legacy_params_are_rejected():
    npc = scene()['npcs'][0]
    npc['behavior']['params']['trig_simtime'] = 2
    with pytest.raises(ValueError, match='conflicts'):
        normalize_ads_trigger(npc)


def rectangle(x, y, yaw=0):
    import math
    c, s = math.cos(yaw), math.sin(yaw)
    return hull([(x+a*c-b*s, y+a*s+b*c) for a in (-2, 2) for b in (-1, 1)])


def test_adjacent_cars_have_real_clearance_and_no_fictitious_ttc():
    a, b = rectangle(0, 0), rectangle(0, 3.5)
    assert body_clearance(a, b) == pytest.approx(1.5)
    assert constant_velocity_ttc(a, b, (-2, 0)) is None
    lead = rectangle(10, 0)
    assert body_clearance(a, lead) == pytest.approx(6)
    assert constant_velocity_ttc(a, lead, (-2, 0)) == pytest.approx(3)
    assert constant_velocity_ttc(a, lead, (2, 0)) is None
    assert constant_velocity_ttc(a, rectangle(2, 0), (0, 0)) == 0


def test_crossing_ttc_uses_vectors():
    import math
    a, b = rectangle(0, 0), rectangle(5, 5, math.pi/2)
    assert constant_velocity_ttc(a, b, (-2, -2)) == pytest.approx(1)


def test_native_hook_is_idempotent_and_leaves_unrelated_conditions_alone():
    from tools.enable_ads_conditions import patched_scenario
    path = ROOT/'runner/src/open_scenario.py'
    assert patched_scenario(path.read_text()) == path.read_text()
    native = (ROOT/'external/scenario_runner/srunner/scenarios/open_scenario.py').read_text()
    assert patched_scenario(native).count('from ads_conditions import') == 1


def test_conditions_must_be_true_on_same_tick():
    py_trees = pytest.importorskip('py_trees')
    runtime = str(ROOT/'runner/src')
    sys.path.insert(0, runtime)
    try:
        from ads_conditions import live_ads_condition_group
        xml = ET.fromstring('<ConditionGroup><Condition name="c2x_ads_near"/><Condition name="c2x_ads_speed"/></ConditionGroup>')
        state = {'c2x_ads_near': True, 'c2x_ads_speed': False}
        class Probe(py_trees.behaviour.Behaviour):
            def update(self):
                return py_trees.common.Status.SUCCESS if state[self.name] else py_trees.common.Status.RUNNING
        group = live_ads_condition_group(xml, lambda c: Probe(c.get('name')))
        list(group.tick())
        state.update(c2x_ads_near=False, c2x_ads_speed=True)
        list(group.tick())
        assert group.status == py_trees.common.Status.RUNNING
        state['c2x_ads_near'] = True
        list(group.tick())
        assert group.status == py_trees.common.Status.SUCCESS
        assert live_ads_condition_group(ET.fromstring('<ConditionGroup><Condition name="old"/></ConditionGroup>'), None) is None
    finally:
        sys.path.remove(runtime)


def test_stress_derivation_is_a_copy_and_bounds_wet_road_braking():
    from tools.ads_stress import derive
    source = scene()
    source['environment'] = {'friction_scale': .3}
    source['npcs'][0]['behavior']['params'].update(speed=10, brake_t=.2)
    changed, changes = derive(source, 'critical')
    assert source['npcs'][0]['behavior']['params']['brake_t'] == .2
    assert changed['npcs'][0]['behavior']['params']['brake_t'] >= 10/(.8*.3*9.81)
    assert changed['sut'] == source['sut']
    assert changes[0]['before'] == source['npcs'][0]['behavior']


def test_event_observation_and_clearance_do_not_invent_collision():
    from tools.review_ads_stress import evaluate
    def state(x, speed):
        return {'x': x, 'y': 0, 'yaw': 0, 'vx': speed, 'vy': 0,
                'bounding_box_world_vertices': [[px, py, 0] for px, py in rectangle(x, 0)],
                'applied_control': {'brake': 0}}
    rows = [{'simulation_time': 1+i*.05, 'actors': {'hero': state(i*.25, 5), 'lead': state(10+i*.15, 3)}} for i in range(5)]
    events = [{'event_type': 'storyboard_transition', 'simulation_time': 1.05,
               'payload': {'element_type': 'EVENT', 'element_name': 'lead_brake', 'transition': 'START'}}]
    report = evaluate(rows, events, 'hero', {'lead': {'event': 'lead_brake'}})
    assert report['actors']['lead']['at_start']['constant_velocity_ttc_s'] == pytest.approx(2.95)
    assert not report['actors']['lead']['physical_contact_with_ads']
    missing = evaluate(rows, [], 'hero', {'lead': {'event': 'lead_brake'}})
    assert missing['actors']['lead']['start_status'] == 'missing_instrumentation'


def test_calibration_uses_unbraked_pre_hazard_cruise(tmp_path):
    import json
    from tools.ads_stress import calibrate_approach
    rows = [{'simulation_time': i*.05, 'actors': {'hero': {
        'planar_speed_mps': 12 if i < 40 else 5, 'applied_control': {'brake': 0}}}} for i in range(200)]
    (tmp_path/'sim_trace_raw.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (tmp_path/'events.jsonl').write_text('')
    (tmp_path/'scenario.runtime.xosc').write_text('<OpenSCENARIO><Storyboard><Init><Actions><Private entityRef="hero">'
        '<Controller><Properties><Property value="external_control"/></Properties></Controller>'
        '</Private></Actions></Init></Storyboard></OpenSCENARIO>')
    source = scene('cut_in')
    calibrated, evidence = calibrate_approach(source, tmp_path)
    assert evidence['observed_ego_median_mps'] == 5
    assert calibrated['npcs'][0]['behavior']['params']['speed'] == 3
    assert calibrated['npcs'][0]['behavior']['params']['cut_dist'] == 7.5
    assert 'params' not in source['sut']
