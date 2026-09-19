"""Review repeated impacts against observed action phases and CARLA box states.

This is motion evidence, not a source-fidelity acceptance decision. Sensor
silence alone never establishes separation, braking or a new approach.
"""
import math


def _cross(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def _hull(vertices):
    points = sorted(set((float(v[0]), float(v[1])) for v in vertices))
    if len(points) < 3 or not all(math.isfinite(v) for p in points for v in p):
        raise ValueError('finite world bounding-box vertices required')
    lower, upper = [], []
    for collection, ordered in ((lower, points), (upper, reversed(points))):
        for p in ordered:
            while len(collection) >= 2 and _cross(collection[-2], collection[-1], p) <= 0:
                collection.pop()
            collection.append(p)
    result = lower[:-1]+upper[:-1]
    if len(result) < 3:
        raise ValueError('degenerate bounding box')
    return result


def box_clearance(a, b):
    """Planar convex-box clearance from measured world vertices (metres)."""
    polygons = [_hull(state['bounding_box_world_vertices']) for state in (a, b)]
    separated = False
    for polygon in polygons:
        for start, end in zip(polygon, polygon[1:]+polygon[:1]):
            nx, ny = start[1]-end[1], end[0]-start[0]
            intervals = [[nx*x+ny*y for x, y in p] for p in polygons]
            if max(intervals[0]) < min(intervals[1]) or max(intervals[1]) < min(intervals[0]):
                separated = True
    if not separated:
        return 0.0

    def point_edge(p, a, b):
        dx, dy = b[0]-a[0], b[1]-a[1]
        t = min(1., max(0., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
        return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)

    return min(point_edge(p, a, b) for one, two in (polygons, polygons[::-1])
               for p in one for a, b in zip(two, two[1:]+two[:1]))


def review_repeated_contacts(scene, events, rows):
    canonical = lambda value: 'hero' if value == 'ego' else value
    expected = scene.get('collisions') or []
    counts = {}
    for contact in expected:
        pair = tuple(sorted(canonical(contact[k]) for k in ('a', 'b')))
        counts[pair] = counts.get(pair, 0)+1
    repeated = {pair: count for pair, count in counts.items() if count > 1}
    if not repeated:
        return {'applicable': False, 'source_fidelity_accepted': False}
    rows = sorted(rows, key=lambda row: row['simulation_time'])
    contacts = sorted((e for e in events if e.get('event_type') == 'collision'
                       and e.get('payload', {}).get('source') == 'carla_collision_sensor'),
                      key=lambda e: e['simulation_time'])
    starts = {e['payload']['element_name']: float(e['simulation_time']) for e in events
              if e.get('event_type') == 'storyboard_transition'
              and e.get('payload', {}).get('source') == 'scenario_runner_blackboard'
              and e['payload'].get('element_type') == 'EVENT' and e['payload'].get('transition') == 'START'}
    phases = {}
    for npc in scene.get('npcs', []):
        for index, step in enumerate((npc.get('behavior') or {}).get('steps', [])):
            when = step.get('when') or {}
            if step.get('action') != 'drive' or when.get('condition') != 'separated_and_target_stopped':
                continue
            pair = tuple(sorted((npc['id'], canonical(when['target']))))
            name = f"{npc['id']}_step_{index+1}"
            phases.setdefault(pair, []).append({'name': name, 'actor': npc['id'],
                'target': canonical(when['target']), 'time': starts.get(name), 'when': when})

    def state_before(t):
        candidates = [r for r in rows if r['simulation_time'] <= t+1e-6]
        if not candidates or t-candidates[-1]['simulation_time'] > .1:
            raise ValueError('missing measured state near event')
        return candidates[-1]['actors']

    def speed(state):
        return math.hypot(state['vx'], state['vy'])

    results = []
    for pair, count in repeated.items():
        pair_events = [e for e in contacts if tuple(sorted(e['payload'].get('actors', []))) == pair]
        first = pair_events[0] if pair_events else None
        times = [first['simulation_time']] if first else []
        boundaries = phases.get(pair, [])[:count-1]
        checks = {'first_sensor_contact_observed': first is not None,
                  'restart_phases_declared': len(boundaries) == count-1}
        measurements = []
        for index, phase in enumerate(boundaries):
            label = str(index+2)
            t = phase['time']
            record = {'event': phase['name'], 'restart_time_s': t}
            valid = t is not None and bool(times) and t > times[-1]
            checks['restart_observed_before_impact_'+label] = valid
            if not valid:
                measurements.append(record)
                continue
            subsequent = next((e for e in pair_events if e['simulation_time'] >= t), None)
            checks['post_restart_sensor_contact_'+label] = subsequent is not None
            if subsequent:
                times.append(subsequent['simulation_time'])
                record['contact_time_s'] = times[-1]
            try:
                states = state_before(t)
                clearance = box_clearance(states[phase['actor']], states[phase['target']])
                duration = float(phase['when'].get('standstill_duration', .5))
                window = [r for r in rows if t-duration-1e-6 <= r['simulation_time'] <= t+1e-6]
                covered = (bool(window) and window[0]['simulation_time'] <= t-duration+.001
                           and window[-1]['simulation_time'] >= t-.051
                           and all(b['simulation_time']-a['simulation_time'] <= .1+1e-6 for a, b in zip(window, window[1:])))
                maximum = max(speed(r['actors'][phase['target']]) for r in window)
                checks['measured_box_separation_'+label] = clearance >= float(phase['when'].get('clearance', 2))
                checks['target_standstill_before_restart_'+label] = covered and maximum < .1
                record.update({'box_clearance_at_restart_m': clearance, 'standstill_window_s': duration,
                               'target_max_speed_before_restart_mps': maximum})
                if subsequent:
                    before = state_before(times[-1]-.05)
                    target_speed = speed(before[phase['target']]); actor_speed = speed(before[phase['actor']])
                    record.update({'target_precontact_speed_mps': target_speed, 'actor_precontact_speed_mps': actor_speed})
                    checks['target_stationary_at_repeat_impact_'+label] = target_speed < .2
                    checks['actor_moving_at_repeat_impact_'+label] = actor_speed > .15
            except (KeyError, ValueError, TypeError) as exc:
                checks['measured_trace_available_'+label] = False
                record['measurement_error'] = str(exc)
            measurements.append(record)
        checks['all_expected_contacts_selected'] = len(times) == count
        results.append({'actors': list(pair), 'selected_contact_times_s': times,
                        'checks': checks, 'phase_measurements': measurements,
                        'separation_and_restart_motion_pass': all(checks.values())})
    return {'applicable': True, 'pairs': results,
            'separation_and_restart_motion_pass': all(r['separation_and_restart_motion_pass'] for r in results),
            'source_fidelity_accepted': False,
            'remaining_source_review': ['sustained initial contact', 'impact locations', 'complete source narrative']}
