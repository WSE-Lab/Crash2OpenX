"""Describe observed scenario actions and motion without inferring task success.

Reads compiled events, blackboard observations, actor trace and collision
sensors. Missing instrumentation is distinct from a missing START observation.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET


def review_action_progress(root, rows, events):
    transitions = [e for e in events if e.get('event_type') == 'storyboard_transition'
                   and e.get('payload', {}).get('source') == 'scenario_runner_blackboard'
                   and e['payload'].get('element_type') == 'EVENT']
    plans = []
    for group in root.findall('.//ManeuverGroup'):
        actors = [n.get('entityRef') for n in group.findall('./Actors/EntityRef')]
        for event in group.findall('./Maneuver/Event'):
            name = event.get('name')
            observed = [e for e in transitions if e['payload'].get('element_name') == name]
            times = {state: sorted(e['simulation_time'] for e in observed
                                  if e['payload'].get('transition') == state)
                     for state in ('START', 'END', 'CANCEL')}
            status = ('start_observed' if times['START'] else
                      'end_or_cancel_only' if observed else
                      'start_not_observed' if transitions else 'no_stage_observations_in_run')
            plans.append({'event': name, 'actors': actors, 'observation_status': status,
                          'transition_times_s': times,
                          'trigger_xml': ET.tostring(event.find('StartTrigger'), encoding='unicode')
                          if event.find('StartTrigger') is not None else None})
    actor_ids = sorted({actor for row in rows for actor in row.get('actors', {})})
    motion = {}
    for actor in actor_ids:
        samples = [(float(row['simulation_time']), row['actors'][actor])
                   for row in rows if actor in row.get('actors', {})]
        if not samples:
            continue
        speeds = [math.hypot(s['vx'], s['vy']) for _, s in samples]
        if any(not math.isfinite(speed) for speed in speeds):
            raise ValueError('Nonfinite actor speed: ' + actor)
        orientation_available = all(all(k in s and math.isfinite(s[k]) for k in ('roll', 'pitch'))
                                    for _, s in samples)
        seconds = stationary = braking = distance = tilted = 0.0
        for (t0, a), (t1, b), speed in zip(samples, samples[1:], speeds):
            dt = t1 - t0
            if not 0 < dt <= .1 + 1e-6:
                continue  # Do not extrapolate across trace gaps.
            seconds += dt
            stationary += dt * (speed < .1)
            braking += dt * (a.get('applied_control', {}).get('brake', 0) >= .5)
            distance += math.hypot(b['x'] - a['x'], b['y'] - a['y'])
            if orientation_available:
                tilted += dt * (max(abs(a['roll']), abs(a['pitch'])) > 60)
        motion[actor] = {'samples': len(samples), 'observed_interval_seconds': seconds,
                         'stationary_seconds_below_0_1_mps': stationary,
                         'brake_seconds_at_least_0_5': braking,
                         'sampled_path_length_m': distance, 'max_planar_speed_mps': max(speeds),
                         'start_xy': [samples[0][1][k] for k in ('x', 'y')],
                         'end_xy': [samples[-1][1][k] for k in ('x', 'y')]}
        motion[actor].update(
            orientation_available=orientation_available,
            max_abs_roll_degrees=max(abs(s['roll']) for _, s in samples) if orientation_available else None,
            max_abs_pitch_degrees=max(abs(s['pitch']) for _, s in samples) if orientation_available else None,
            tilted_over_60_degrees_seconds=tilted if orientation_available else None)
    pairs = {}
    for event in events:
        payload = event.get('payload', {})
        if event.get('event_type') != 'collision' or payload.get('source') != 'carla_collision_sensor':
            continue
        pair = tuple(sorted(payload.get('actors', [])))
        if len(pair) != 2:
            continue
        record = pairs.setdefault(pair, {'actors': list(pair), 'sensor_records': 0,
                                        'first_time_s': event['simulation_time'],
                                        'last_time_s': event['simulation_time']})
        record['sensor_records'] += 1
        record['first_time_s'] = min(record['first_time_s'], event['simulation_time'])
        record['last_time_s'] = max(record['last_time_s'], event['simulation_time'])
    return {'has_stage_observations': bool(transitions), 'planned_events': plans,
            'actor_motion': motion, 'contact_pairs': list(pairs.values()),
            'source_fidelity_accepted': False, 'autonomous_task_success': None,
            'limitations': ['START/END observations establish behavior-tree stages, not successful physical actions.',
                            'Missing START is an observation result; no stage records means instrumentation is unverified.',
                            'Stationary and braking intervals use the preceding sample and omit gaps over 0.1 seconds.',
                            'Braking or absent contact alone does not identify why an agent stopped.',
                            'Measured roll/pitch over 60 degrees is an orientation observation; source relevance requires separate review.',
                            'Collision sensor records are not independent accident counts.']}


def review_run(run):
    run = Path(run)
    result = json.loads((run/'result.json').read_text())
    xosc = Path(result['xosc_path'])
    rows = [json.loads(line) for line in (run/'sim_trace_raw.jsonl').read_text().splitlines()]
    events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
    review = review_action_progress(ET.parse(xosc).getroot(), rows, events)
    review.update(run=str(run.resolve()), sha256={str(p.name): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in (xosc, run/'sim_trace_raw.jsonl', run/'events.jsonl', run/'runtime_manifest.json')})
    return review


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(review_run(args.run), ensure_ascii=False, indent=2)+'\n')
