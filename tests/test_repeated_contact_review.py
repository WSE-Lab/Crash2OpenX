import copy

import pytest

from tools.repeated_contact_review import box_clearance, review_repeated_contacts


def state(x, speed=0):
    return {'vx': speed, 'vy': 0, 'bounding_box_world_vertices':
            [[x+dx, dy, z] for dx in (-2, 2) for dy in (-1, 1) for z in (0, 1.5)]}


def contact(t):
    return {'event_type': 'collision', 'simulation_time': t,
            'payload': {'source': 'carla_collision_sensor', 'actors': ['hero', 'car']}}


def fixture():
    scene = {'collisions': [{'a': 'ego', 'b': 'car'}]*2, 'npcs': [
        {'id': 'car', 'behavior': {'block': 'sequence', 'steps': [
            {'action': 'drive', 'when': {'condition': 'start'}},
            {'action': 'drive', 'when': {'condition': 'separated_and_target_stopped', 'target': 'ego'}}]}}]}
    events = [contact(1), contact(1.9), contact(8), {'event_type': 'storyboard_transition',
        'simulation_time': 5, 'payload': {'source': 'scenario_runner_blackboard',
        'element_type': 'EVENT', 'element_name': 'car_step_2', 'transition': 'START'}}]
    rows = [{'simulation_time': i/20, 'actors': {'hero': state(0), 'car': state(10, 4)}} for i in range(180)]
    return scene, events, rows


def test_actual_boxes_supply_clearance_independent_of_actor_origin():
    assert box_clearance(state(0), state(10)) == 6
    assert box_clearance(state(0), state(3)) == 0
    assert box_clearance(state(0), state(4)) == 0


def test_sensor_flicker_is_not_selected_in_place_of_post_restart_impact():
    result = review_repeated_contacts(*fixture())
    assert result['pairs'][0]['selected_contact_times_s'] == [1, 8]
    assert result['separation_and_restart_motion_pass']
    assert not result['source_fidelity_accepted']


@pytest.mark.parametrize('missing', ['start', 'separation', 'standstill', 'trace_gap', 'boxes', 'second_contact'])
def test_declared_story_cannot_replace_missing_measured_evidence(missing):
    scene, events, rows = copy.deepcopy(fixture())
    if missing == 'start': events.pop()
    if missing == 'second_contact': events.pop(2)
    if missing == 'trace_gap': rows = [r for r in rows if not 4.7 < r['simulation_time'] < 4.95]
    for row in rows:
        if 4.4 <= row['simulation_time'] <= 5:
            if missing == 'separation': row['actors']['car'] = state(3, 4)
            if missing == 'standstill': row['actors']['hero']['vx'] = 1
            if missing == 'boxes': row['actors']['car'].pop('bounding_box_world_vertices')
    result = review_repeated_contacts(scene, events, rows)
    assert not result['separation_and_restart_motion_pass']
    assert not result['source_fidelity_accepted']
