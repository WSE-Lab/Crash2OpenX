#!/usr/bin/env python3
"""Screen derived episodes using measured interactions and map containment."""
import argparse
import json
import math
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.review_ads_stress import evaluate, read_rows, scenario_info
from tools.review_fresh_trajectory_batch import native_geometry_review
from tools.review_runtime_action_progress import review_action_progress
from tools.run_fresh_framework_batch import digest, write
from tools.xodr_corridor_review import review_trace


def approach_matches(npc, ego, other):
    """Check the declared incoming junction leg in measured CARLA coordinates."""
    if npc['behavior']['block'] not in {'junction_turn', 'junction_cross'}:
        return True
    difference = abs((other['yaw']-ego['yaw']+180) % 360-180)
    position = npc.get('position')
    if position in {'oncoming', 'opposing_leg'}:
        return difference > 150
    if position == 'ahead_same_lane':
        return difference < 30
    if position == 'cross':
        if not 60 < difference < 120:
            return False
        yaw = math.radians(ego['yaw'])
        right = -(other['x']-ego['x'])*math.sin(yaw)+(other['y']-ego['y'])*math.cos(yaw)
        return (right > 0 if npc.get('side') == 'right' else
                right < 0 if npc.get('side') == 'left' else True)
    return True


def screen(folder):
    run = folder/'run'
    rows = read_rows(run/'sim_trace_raw.jsonl')
    events = read_rows(run/'events.jsonl')
    ads, plans = scenario_info(folder/'scenario.xosc')
    measured = evaluate(rows, events, ads, plans)
    actions = review_action_progress(ET.parse(folder/'scenario.xosc').getroot(), rows, events)
    manifest = json.loads((folder/'manifest.json').read_text())
    npcs = {n['id']: n for n in manifest['derived_scene']['npcs']}
    source = Path(manifest['source_run'])
    map_context = json.loads((source/'result.json').read_text())
    if manifest.get('roadgraph_cache_dir'):
        map_context.update(roadgraph_cache_dir=manifest['roadgraph_cache_dir'], xodr_path=str(folder/'map.xodr'))
    native = native_geometry_review(map_context)
    summary = json.loads((run/'summary.json').read_text())
    interactions = []
    for actor, record in measured['actors'].items():
        start = record['start_time_s']
        window = [r for r in rows if start is not None and start <= r['simulation_time'] <= start+6]
        if len(window) < 2:
            continue
        first = window[0]['actors'][actor]
        samples = [r['actors'][actor] for r in window if actor in r['actors']]
        ego = [r['actors'][ads] for r in window if ads in r['actors']]
        speeds = [math.hypot(s['vx'], s['vy']) for s in samples]
        angle = math.radians(first['yaw'])
        offsets = [-(s['x']-first['x'])*math.sin(angle)+(s['y']-first['y'])*math.cos(angle) for s in samples]
        npc_change = max(abs(o) for o in offsets) >= .5 or max(speeds)-min(speeds) >= 1
        ego_speeds = [math.hypot(s['vx'], s['vy']) for s in ego]
        braking = max(s.get('applied_control', {}).get('brake', 0) for s in ego)
        clearance = record['six_seconds_after_start_min_clearance_m']
        ttc = record['six_seconds_after_start_min_positive_ttc_s']
        relevant = (clearance is not None and clearance < 10) or (ttc is not None and ttc < 5)
        corridor = review_trace(folder/'map.xodr', window, actor=ads)
        npc_corridor = review_trace(folder/'map.xodr', window, actor=actor)
        upright = all(max(abs(s.get('pitch', 0)), abs(s.get('roll', 0))) < 30 for s in samples+ego)
        # Test validity must not require the ADS to succeed: its lane departure
        # is an outcome. Require a valid initial ADS lane and a valid NPC path;
        # retain the complete ADS corridor violations separately for review.
        initial_ego = review_trace(folder/'map.xodr', rows[:20], actor=ads)
        geometry = not any(c['outside_samples'] or c['unresolved_samples'] or c['missing_actor_samples'] or c['unsupported'] for c in (initial_ego, npc_corridor))
        approach = approach_matches(npcs[actor], rows[0]['actors'][ads], rows[0]['actors'][actor])
        deceleration = record['six_seconds_after_start_max_npc_deceleration_mps2']
        # A scripted lead brake must not be a near-instantaneous velocity jump.
        plausible_brake = (npcs[actor]['behavior']['block'] != 'front_brake'
                           or (deceleration is not None and deceleration <= 12))
        complete = window[-1]['simulation_time']-window[0]['simulation_time'] >= 5.8
        contact_terminated = (summary.get('termination_reason') == 'collision_exit'
                              and record['physical_contact_with_ads'])
        sufficient_window = complete or contact_terminated
        interactions.append({'actor': actor, 'event': record['plan']['event'],
            'start_since_trace_s': start-rows[0]['simulation_time'],
            'window_duration_s': window[-1]['simulation_time']-window[0]['simulation_time'],
            'npc_max_lateral_displacement_m': max(abs(o) for o in offsets),
            'npc_speed_range_mps': max(speeds)-min(speeds), 'ego_speed_drop_mps': ego_speeds[0]-min(ego_speeds),
            'ego_max_brake': braking, 'min_clearance_m': clearance, 'min_positive_ttc_s': ttc,
            'physical_contact': record['physical_contact_with_ads'],
            'checks': {'event_observed': True, 'npc_motion_observed': npc_change,
                'relevant_proximity': relevant, 'six_second_window_or_contact_termination': sufficient_window,
                'valid_ego_spawn_and_npc_corridor': geometry, 'upright_actors': upright,
                'declared_junction_approach': approach, 'plausible_lead_braking': plausible_brake},
            'npc_max_deceleration_mps2': deceleration,
            'ads_lane_departure_samples': corridor['outside_samples'],
            'contact_terminated_before_six_seconds': bool(contact_terminated and not complete),
            'candidate_pass': bool(npc_change and relevant and sufficient_window and geometry and upright and approach and plausible_brake),
            'ego_corridor': corridor, 'npc_corridor': npc_corridor})
    provenance = json.loads((run/'remote_run.json').read_text())
    integrity = (provenance['xodr_sha256'] == digest(folder/'map.xodr')
                 and provenance['xosc_sha256'] == digest(folder/'scenario.xosc')
                 and provenance.get('pcla_agent') == 'if_if' and provenance.get('sut_actor') == ads
                 and len(rows) > 100 and native['verified'])
    result = {'case_id': manifest['case_id'], 'profile': folder.name,
              'candidate_for_visual_review': bool(integrity and any(i['candidate_pass'] for i in interactions)),
              'integrity_and_native_map_pass': integrity, 'native_map': native,
              'interactions': interactions, 'onset_review': measured, 'actions': actions,
              'termination': summary.get('termination_reason'), 'total_ticks': len(rows),
              'full_episode_lane_invasions': summary.get('lane_invasion_count'),
              'full_episode_offroad_seconds': summary.get('off_road_time'),
              'full_episode_collision_sensor_records': summary.get('collision_count'),
              'source_reconstruction_accepted': False, 'visual_review_pass': None,
              'limitations': ['Screening checks actor centres, not full footprints or lane legality.',
                  'ADS lane departures and physical collisions are recorded outcomes, not automatic invalidation of an otherwise valid test.',
                  'One episode per parameter setting; no failure-rate or robustness claim.',
                  'Experimental onset, speed and spacing differ from source reports.',
                  'Full-episode termination and failures remain visible alongside the six-second interaction.'],
              'sha256': {p.name: digest(p) for p in [folder/'scenario.xosc', folder/'map.xodr',
                  run/'sim_trace_raw.jsonl', run/'events.jsonl', run/'carla_rgb.mp4']}}
    write(folder/'quality_review.json', result)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('output', type=Path)
    args = ap.parse_args()
    results = []
    for marker in sorted(args.output.glob('*/*/execution.json')):
        if not json.loads(marker.read_text()).get('complete'):
            continue
        folder = marker.parent
        if folder.name not in {'early', 'tight', 'critical'}:
            continue
        result = screen(folder)
        print(result['case_id'][:3], folder.name, result['candidate_for_visual_review'],
              [(i['actor'], round(i['min_clearance_m'], 2), i['checks']) for i in result['interactions']], flush=True)
        results.append({'case_id': result['case_id'], 'profile': folder.name,
                        'candidate_for_visual_review': result['candidate_for_visual_review'],
                        'review': str(folder/'quality_review.json')})
    write(args.output/'quality_index.json', results)


if __name__ == '__main__':
    main()
