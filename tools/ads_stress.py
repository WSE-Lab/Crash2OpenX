#!/usr/bin/env python3
"""Derive reviewable ADS-relative variants from a seed without changing its source.

This calibrates hazard onset, not the probability of collision. A smaller
window may never be reached by a cautious ADS; that is a measured outcome.
"""
import argparse
import copy
import hashlib
import json
import shutil
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.ads_trigger import EVENT_SUFFIX, normalize_ads_trigger
from tools.build_road_seed_opendrive import build
from tools.ocl_constraints import violations
from tools.osc_blocks import build_xosc
from tools.replay_scene_tools import validate_xosc

PROFILES = {'early': 16.0, 'tight': 8.0, 'critical': 3.0}


def calibrate_approach(scene, run, actor_ids=None):
    """Use a prior, pre-hazard ADS cruise observation for an explicit variant.

    The policy and sensors are untouched. This only changes the next episode's
    initial ADS speed and selected same-direction NPC speed/dynamics.
    """
    from tools.review_ads_stress import read_rows, scenario_info
    rows = read_rows(run/'sim_trace_raw.jsonl')
    ads, plans = scenario_info(run/'scenario.runtime.xosc')
    start = rows[0]['simulation_time']
    hazard_names = {p['event'] for p in plans.values()}
    starts = [e['simulation_time'] for e in read_rows(run/'events.jsonl')
              if e.get('event_type') == 'storyboard_transition' and
              e.get('payload', {}).get('element_name') in hazard_names and
              e['payload'].get('transition') == 'START']
    end = min(start+10, min(starts, default=start+10))
    speeds = [r['actors'][ads]['planar_speed_mps'] for r in rows
              if start+2 <= r['simulation_time'] < end and ads in r['actors']
              and r['actors'][ads]['planar_speed_mps'] > 1
              and r['actors'][ads].get('applied_control', {}).get('brake', 0) < .1]
    if len(speeds) < 20:
        raise ValueError('Calibration needs at least 20 unbraked moving samples before hazard onset, after startup')
    ego_speed = statistics.median(speeds)
    npc_speed = .6*ego_speed
    if npc_speed <= 2.1:
        raise ValueError('Observed cruise too slow for this moving lane-change calibration')
    result = copy.deepcopy(scene)
    changes = []
    for npc in result['npcs']:
        if actor_ids and npc['id'] not in actor_ids:
            continue
        if npc['behavior']['block'] not in {'cut_in', 'front_brake', 'partial_lane_intrusion'}:
            continue
        before = copy.deepcopy(npc['behavior'])
        params = npc['behavior'].setdefault('params', {})
        params['speed'] = round(npc_speed, 3)
        if npc['behavior']['block'] == 'cut_in':
            params['cut_dist'] = round(npc_speed*2.5, 3)
        changes.append({'actor': npc['id'], 'before': before, 'after': copy.deepcopy(npc['behavior'])})
    if not changes:
        raise ValueError('Calibration requires a selected same-direction moving hazard actor')
    before_sut = copy.deepcopy(result['sut'])
    result['sut'].setdefault('params', {})['initial_speed_mps'] = round(ego_speed, 3)
    return result, {'calibration_run': str(run.resolve()), 'unbraked_samples': len(speeds),
                    'observed_ego_median_mps': ego_speed, 'npc_speed_ratio': .6,
                    'nominal_cut_duration_s': 2.5, 'npc_changes': changes,
                    'sut_before': before_sut, 'sut_after': result['sut']}


def derive(scene, profile, actor_ids=None):
    result = copy.deepcopy(scene)
    changes = []
    chosen = set(actor_ids or [])
    available = {n['id'] for n in scene.get('npcs', [])}
    if chosen - available:
        raise ValueError('Unknown NPCs: ' + str(chosen-available))
    for npc in result.get('npcs', []):
        if chosen and npc['id'] not in chosen:
            continue
        behavior = npc['behavior']
        if behavior['block'] not in EVENT_SUFFIX:
            if chosen:
                raise ValueError('Selected NPC has no supported hazard onset: '+npc['id'])
            continue
        before = copy.deepcopy(behavior)
        params = behavior.setdefault('params', {})
        for key in ('trig_dist', 'trig_ttc', 'trig_simtime', 'trig_simtime_min'):
            params.pop(key, None)
        if behavior['block'] == 'front_brake':
            # Bound the requested target-speed ramp. Actual acceleration still
            # needs measurement; CARLA's physical controller may respond slower.
            delta = abs(float(params.get('speed', 8))-float(params.get('end_speed', 0)))
            friction = float((scene.get('environment') or {}).get('friction_scale', 1.0))
            if friction <= 0:
                raise ValueError('positive friction required for the braking envelope')
            limit = min(6.0, .8*friction*9.81)
            params['brake_t'] = max(float(params.get('brake_t', 1.2)), delta/limit)
        behavior['ads_trigger'] = {'distance_m': PROFILES[profile]}
        behavior['ads_trigger'] = normalize_ads_trigger(npc)
        changes.append({'actor': npc['id'], 'before': before, 'after': copy.deepcopy(behavior)})
    if not changes:
        raise ValueError('No supported hazard onset; stationary obstacles/continuous cruise need a different experiment')
    return result, changes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--road', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--profiles', nargs='+', choices=list(PROFILES), default=list(PROFILES))
    parser.add_argument('--actors', nargs='+')
    parser.add_argument('--xodr', type=Path, help='Reuse this exact generated map; do not regenerate geometry')
    parser.add_argument('--map-name', help='Existing roadgraph cache name for a junction map')
    parser.add_argument('--calibrate-from', type=Path, help='Prior CARLA run: use observed pre-hazard ADS cruise to set a slower NPC approach')
    args = parser.parse_args()
    data = json.loads(args.scene.read_text())
    scene = data.get('scene', data)
    calibration = None
    if args.calibrate_from:
        scene, calibration = calibrate_approach(scene, args.calibrate_from, args.actors)
    road_data = json.loads(args.road.read_text())
    road = road_data.get('road', road_data)
    args.out.mkdir(parents=True, exist_ok=False)
    (args.out/'source_scene_seed.json').write_text(args.scene.read_text())
    (args.out/'road_seed.json').write_text(args.road.read_text())
    xodr = args.out/'map.xodr'
    if args.xodr:
        shutil.copy2(args.xodr, xodr)
    else:
        build({'road': road}, xodr, args.out/'map.html', ROOT/'xsd/OpenDRIVE_1.5M.xsd')
    manifest = {'source_scene_sha256': hashlib.sha256(args.scene.read_bytes()).hexdigest(),
                'map_sha256': hashlib.sha256(xodr.read_bytes()).hexdigest(),
                'mode': 'derived_ADS_stress_variants', 'profiles': [], 'calibration': calibration,
                'assumptions': ['Distances are experimental body-clearance thresholds, not report facts.',
                    'Smaller threshold is not proof of higher measured danger.',
                    'Actor roles, source geometry and target speeds are preserved.',
                    'Requested braking ramps are bounded to 6 m/s^2; actual dynamics require runtime review.',
                    'Requires the live ADS condition group runtime hook; no timeout fallback.']}
    for profile in args.profiles:
        variant, changes = derive(scene, profile, args.actors)
        issues = violations(road, variant)
        if issues:
            raise ValueError('OCL: '+str(issues))
        folder = args.out/profile
        folder.mkdir()
        (folder/'scene_seed.json').write_text(json.dumps({'scene': variant}, ensure_ascii=False, indent=2)+'\n')
        output = folder/'scenario.xosc'
        build_xosc(variant, str(xodr), output, name=args.map_name, road_seed={'road': road})
        valid, error = validate_xosc(output, ROOT/'xsd/OpenSCENARIO.xsd')
        if not valid:
            raise ValueError(error)
        manifest['profiles'].append({'profile': profile, 'changes': changes, 'ocl': 'pass', 'xsd': 'pass'})
    (args.out/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'output': str(args.out), 'compiled_profiles': args.profiles}))


if __name__ == '__main__':
    main()
