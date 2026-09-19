from xml.etree import ElementTree as ET

from tools.review_runtime_action_progress import review_action_progress


def scene():
    return ET.fromstring('<OpenSCENARIO><ManeuverGroup><Actors><EntityRef entityRef="v1"/></Actors>'
                         '<Maneuver><Event name="drive"/><Event name="brake"/></Maneuver>'
                         '</ManeuverGroup></OpenSCENARIO>')


def test_absent_instrumentation_is_not_untriggered_behavior():
    result = review_action_progress(scene(), [], [])
    assert not result['has_stage_observations']
    assert all(e['observation_status'] == 'no_stage_observations_in_run' for e in result['planned_events'])


def test_start_observation_does_not_promote_action_to_success():
    event = {'event_type': 'storyboard_transition', 'simulation_time': 3., 'payload': {
        'source': 'scenario_runner_blackboard', 'element_type': 'EVENT',
        'element_name': 'drive', 'transition': 'START'}}
    result = review_action_progress(scene(), [], [event])
    assert [e['observation_status'] for e in result['planned_events']] == ['start_observed', 'start_not_observed']
    assert not result['source_fidelity_accepted']
    assert result['autonomous_task_success'] is None


def test_motion_intervals_do_not_fill_missing_trace_gaps():
    def row(t, x, speed, brake):
        return {'simulation_time': t, 'actors': {'hero': {
            'x': x, 'y': 0, 'vx': speed, 'vy': 0, 'applied_control': {'brake': brake}}}}
    rows = [row(0, 0, 1, 0), row(.05, .05, 0, 1), row(10, 100, 0, 1)]
    result = review_action_progress(scene(), rows, [])['actor_motion']['hero']
    assert result['observed_interval_seconds'] == .05
    assert result['sampled_path_length_m'] == .05
    assert result['stationary_seconds_below_0_1_mps'] == 0
    assert result['brake_seconds_at_least_0_5'] == 0
    assert result['tilted_over_60_degrees_seconds'] is None


def test_orientation_excursion_is_recorded_without_claiming_source_failure():
    rows = [{'simulation_time': t, 'actors': {'v1': {'x': 0, 'y': 0, 'vx': 0, 'vy': 0,
            'roll': roll, 'pitch': 0}}} for t, roll in [(0, 170), (.05, 100), (.1, 0)]]
    result = review_action_progress(scene(), rows, [])
    assert result['actor_motion']['v1']['max_abs_roll_degrees'] == 170
    assert result['actor_motion']['v1']['tilted_over_60_degrees_seconds'] == .1
    assert result['source_fidelity_accepted'] is False
