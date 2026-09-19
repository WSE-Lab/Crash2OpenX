"""Ordered contact expectations; observations never command actor motion."""
import json
from pathlib import Path
from xml.etree import ElementTree as ET


CONTACT_PARAMETER = 'C2XExpectedContactSequence'


def contact_sequence(scene):
    sequence = scene.get('collisions')
    if sequence is None:
        one = scene.get('collision')
        return [one] if isinstance(one, dict) and one else []
    if not isinstance(sequence, list) or not sequence:
        raise ValueError('scene.collisions must be a nonempty ordered list')
    return sequence


def normalize_contacts(scene, actor_ids):
    result = []
    for item in contact_sequence(scene):
        if not isinstance(item, dict):
            raise ValueError('every collision must be an actor pair')
        a, b = item.get('a', item.get('striker_id')), item.get('b', item.get('struck_id'))
        if a not in actor_ids or b not in actor_ids or a == b:
            raise ValueError('collision endpoints must be distinct declared actors')
        if set(item) - {'a', 'b', 'striker_id', 'struck_id'}:
            raise ValueError('collision pair has unsupported fields')
        result.append({'a': a, 'b': b})
    if not result:
        raise ValueError('at least one expected contact is required')
    primary = scene.get('collision')
    if primary and 'collisions' in scene:
        pair = {primary.get('a'), primary.get('b')}
        if pair != set(result[0].values()):
            raise ValueError('collision compatibility field must match first collisions pair')
    return result


def runtime_contacts(scene):
    return [{key: 'hero' if value == 'ego' else value for key, value in c.items()}
            for c in contact_sequence(scene)]


def read_xosc_contacts(path):
    if not path or not Path(path).is_file():
        return []
    root = ET.parse(path).getroot()
    declaration = root.find('./ParameterDeclarations/ParameterDeclaration[@name="'+CONTACT_PARAMETER+'"]')
    if declaration is None:
        return []
    sequence = json.loads(declaration.get('value', ''))
    actors = {a.get('name') for a in root.findall('./Entities/ScenarioObject')}
    return normalize_contacts({'collisions': sequence}, actors)


def evaluate_contact_sequence(expected, events, episode_gap_seconds=.75):
    """Compare sensor contact episodes in order, without using aggregate counts.

    A time gap separates sensor episodes only. It does not itself prove the
    physical separation or other source actions required by a repeated impact.
    """
    expected_pairs = [tuple(sorted((c['a'], c['b']))) for c in expected]
    last_by_pair, episodes = {}, []
    for event in sorted(events, key=lambda e: e.get('simulation_time', 0)):
        payload = event.get('payload') or {}
        if event.get('event_type') != 'collision' or payload.get('source') != 'carla_collision_sensor':
            continue
        pair = tuple(sorted(payload.get('actors') or []))
        if len(pair) != 2:
            continue
        t = float(event['simulation_time'])
        if pair not in last_by_pair or t - last_by_pair[pair] > episode_gap_seconds:
            episodes.append({'actors': list(pair), 'simulation_time': t})
        last_by_pair[pair] = t
    prefix = episodes[:len(expected_pairs)]
    actual_pairs = [tuple(e['actors']) for e in prefix]
    pairs_match = bool(expected_pairs) and actual_pairs == expected_pairs
    times = [e['simulation_time'] for e in prefix]
    order_resolved = (len(times) == len(expected_pairs)
                      and all(a < b for a, b in zip(times, times[1:])))
    repeated = len(set(expected_pairs)) < len(expected_pairs)
    return {'expected_pairs': [list(p) for p in expected_pairs], 'observed_episodes': episodes,
            'pairs_match_in_order': pairs_match, 'strict_order_resolved': order_resolved,
            'sensor_sequence_pass': pairs_match and order_resolved,
            'physical_separation_requires_trace_review': repeated,
            'source_fidelity_accepted': False}
