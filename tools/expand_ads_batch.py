#!/usr/bin/env python3
"""Derive and execute auditable ADS tests from the verified 42-report batch."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import sys
import traceback
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.ads_stress import calibrate_approach, derive
from tools.ads_trigger import EVENT_SUFFIX
from tools.carla_remote import CarlaRemoteClient
from tools.ocl_constraints import violations
from tools.osc_blocks import build_xosc
from tools.replay_scene_tools import validate_xosc
from tools.run_fresh_framework_batch import SerializedCarlaClient, digest, write


def space_approach(scene, moving):
    """Move an approach cluster together so calibration preserves its ordering."""
    original = copy.deepcopy(scene['npcs'])
    for index, npc in enumerate(n for n in scene['npcs'] if n['id'] in moving):
        params = npc['behavior']['params']
        key = 'long' if npc['position'] == 'adjacent' else 'gap'
        new = 24 + 14*index
        old = params.get(key)
        params[key] = new
        if key != 'long' or old is None:
            continue
        for other, before in zip(scene['npcs'], original):
            other_params = other['behavior'].setdefault('params', {})
            prior_params = before['behavior'].get('params', {})
            if (other['id'] not in moving and other.get('relative_to', 'ego') == npc.get('relative_to', 'ego')
                    and other.get('position') == 'adjacent' and other.get('side') == npc.get('side')
                    and 'long' in prior_params):
                other_params['long'] = prior_params['long']+new-old


def inventory(batch):
    result = []
    for case in json.loads((batch/'audit.json').read_text())['cases']:
        attempts = [a for a in case['attempts'] if a.get('generation_pass') and a.get('runtime_evidence_pass')]
        def source_quality(attempt):
            run = Path(attempt['run'])
            from tools.review_fresh_trajectory_batch import native_geometry_review
            native = native_geometry_review(json.loads((run/'result.json').read_text()))['verified']
            invocation = json.loads((run.parent.parent/'framework_invocation.json').read_text())
            return native, invocation.get('started_at', '')
        attempts.sort(key=source_quality, reverse=True)
        if not attempts:
            result.append({'case_id': case['case_id'], 'status': 'no_verified_source'})
            continue
        source = Path(attempts[0]['run'])
        data = json.loads((source/'scene_seed.json').read_text())
        scene = data.get('scene', data)
        hazards = [n['id'] for n in scene['npcs'] if n['behavior']['block'] in EVENT_SUFFIX]
        result.append({'case_id': case['case_id'], 'source': str(source),
                       'hazard_actors': hazards, 'blocks': [n['behavior']['block'] for n in scene['npcs']],
                       'status': 'candidate' if hazards else 'source_reference_only'})
    return sorted(result, key=lambda r: r['case_id'])


def prepare(item, output, calibration_run, profile, simulation_seconds=25):
    source = Path(item['source'])
    folder = output/item['case_id']/profile
    folder.mkdir(parents=True, exist_ok=False)
    original = json.loads((source/'scene_seed.json').read_text())
    scene = copy.deepcopy(original.get('scene', original))
    road = json.loads((source/'road_seed.json').read_text())
    result = json.loads((source/'result.json').read_text())
    rebuilt_map = None
    for prior in sorted(folder.parent.glob('*/manifest.json')):
        record = json.loads(prior.read_text())
        if (record.get('roadgraph_cache_dir') and record.get('road_recompiled_from_source_seed')
                and record['source_scene_sha256'] == digest(source/'scene_seed.json')
                and digest(prior.parent/'map.xodr') == record['map_sha256']):
            rebuilt_map = (prior.parent, record)
            break
    for name in ['scene_seed.json', 'road_seed.json', 'source_text.txt', 'source_extraction.json', 'road_generation.json']:
        shutil.copy2(source/name, folder/('source_'+name))
    shutil.copy2(source.parent.parent/'source.pdf', folder/'source.pdf')
    shutil.copy2(result['xodr_path'], folder/'map.xodr')
    if result.get('html_path') and Path(result['html_path']).is_file():
        shutil.copy2(result['html_path'], folder/'map.html')
    if rebuilt_map:
        previous, record = rebuilt_map
        shutil.copy2(previous/'map.xodr', folder/'map.xodr')
        shutil.copy2(previous/'map.html', folder/'map.html')
        result['name'] = Path(record['roadgraph_cache_dir']).name
    moving = [n['id'] for n in scene['npcs'] if n['behavior']['block'] in {'cut_in', 'front_brake', 'partial_lane_intrusion'}]
    calibration = None
    if moving:
        # Reference calibration is explicitly shared across cases, not claimed
        # to be a measurement of the new case's ADS cruise.
        scene, calibration = calibrate_approach(scene, calibration_run, moving)
        calibration['scope'] = 'shared prior InterFuser cruise; validate actual speed in this episode'
        space_approach(scene, moving)
    scene, _ = derive(scene, profile)
    for npc in scene['npcs']:
        block = npc['behavior']['block']
        if block == 'front_brake':
            npc['behavior']['ads_trigger']['distance_m'] = {'early': 16, 'tight': 10, 'critical': 6}[profile]
            if profile == 'critical':
                # This is a decelerating-lead test, not a reconstruction of a
                # full stop. CARLA's low-speed vehicle solver snapped this SUV
                # to rest below 2 m/s even with a small brake-control cap.
                npc['behavior']['params'].update(speed=3.8, end_speed=2.4,
                                                gap=18, max_brake=.08)
        elif block in {'junction_cross', 'junction_turn'}:
            npc['behavior']['params']['approach_distance_m'] = 10
            npc['behavior']['ads_trigger']['distance_m'] = {'early': 30, 'tight': 22, 'critical': 16}[profile]
    issues = violations(road.get('road', road), scene)
    if issues:
        raise ValueError('OCL violations: '+str(issues))
    write(folder/'scene_seed.json', {'scene': scene})
    build_xosc(scene, str(folder/'map.xodr'), folder/'scenario.xosc', name=result['name'], road_seed=road)
    junction_timing = {}
    if profile == 'critical':
        from tools.ads_junction_timing import departure_window
        root = ET.parse(folder/'scenario.xosc').getroot()
        for npc in scene['npcs']:
            if npc['behavior']['block'] in {'junction_cross', 'junction_turn'}:
                timing = departure_window(root, npc['id'], npc_speed=float(npc['behavior']['params'].get('speed', 8)),
                    time_offset=1.5 if npc.get('position') == 'ahead_same_lane' else 0)
                npc['behavior']['ads_trigger']['distance_m'] = timing['distance_m']
                junction_timing[npc['id']] = timing
        if junction_timing:
            write(folder/'scene_seed.json', {'scene': scene})
            build_xosc(scene, str(folder/'map.xodr'), folder/'scenario.xosc', name=result['name'], road_seed=road)
    tree = ET.parse(folder/'scenario.xosc')
    tree.find('.//RoadNetwork/LogicFile').set('filepath', 'map.xodr')
    # Bound the reviewed experiment to 25 simulation seconds, while retaining
    # the complete episode and all existing distance stop conditions.
    # ScenarioRunner executes Act StopTrigger; retain the top-level standard
    # stop condition too, for other OpenSCENARIO consumers.
    for stop in [tree.find('./Storyboard/StopTrigger'), *tree.findall('./Storyboard/Story/Act/StopTrigger')]:
        group = ET.SubElement(stop, 'ConditionGroup')
        condition = ET.SubElement(group, 'Condition', name='experiment_duration', delay='0', conditionEdge='rising')
        ET.SubElement(ET.SubElement(condition, 'ByValueCondition'), 'SimulationTimeCondition', value=str(simulation_seconds), rule='greaterThan')
    tree.write(folder/'scenario.xosc', encoding='utf-8', xml_declaration=True)
    valid, error = validate_xosc(folder/'scenario.xosc', ROOT/'xsd/OpenSCENARIO.xsd')
    if not valid:
        raise ValueError(error)
    write(folder/'manifest.json', {'case_id': item['case_id'], 'mode': 'derived_ADS_test',
        'source_run': str(source), 'source_result_sha256': digest(source/'result.json'),
        'source_scene_sha256': digest(source/'scene_seed.json'), 'source_map_sha256': digest(Path(result['xodr_path'])),
        'map_sha256': digest(folder/'map.xodr'), 'scenario_sha256': digest(folder/'scenario.xosc'),
        'map_preserved': rebuilt_map is None, 'source_scene': original, 'derived_scene': scene,
        **({k: rebuilt_map[1][k] for k in ['roadgraph_cache_dir', 'road_recompiled_from_source_seed', 'native_geometry_check']} if rebuilt_map else {}),
        'calibration': calibration, 'junction_timing': junction_timing,
        'simulation_limit_s': simulation_seconds, 'ocl_pass': True, 'xosc_xsd_pass': True,
        'claim': 'Source-derived ADS test; modified onset, speeds and spacing are experimental, not report facts.'})
    return folder, scene, road


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--batch', type=Path, default=ROOT/'outputs/framework_42_20260917')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--cases', nargs='+')
    ap.add_argument('--profile', choices=['early', 'tight', 'critical'], default='tight')
    ap.add_argument('--calibration-run', type=Path, default=ROOT/'outputs/ads_stress_20260919/cut_in/early/run')
    ap.add_argument('--runtime-config', type=Path, default=ROOT/'outputs/ads_stress_20260919/runtime_config.json')
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--simulation-seconds', type=int, default=25)
    args = ap.parse_args()
    args.out = args.out.resolve()
    args.out.mkdir(parents=True, exist_ok=True)
    items = inventory(args.batch)
    write(args.out/'inventory_42.json', items)
    os.environ.update(json.loads(args.runtime_config.read_text()))
    os.environ['CARLA_REMOTE_KEEP_RUNS'] = '1'
    os.environ['CARLA_REMOTE_COMPACT'] = '1'
    client = SerializedCarlaClient(CarlaRemoteClient(), ROOT/'outputs/.carla_rpc.lock')
    for item in items:
        if item['status'] != 'candidate' or (args.cases and item['case_id'][:3] not in args.cases):
            continue
        folder = args.out/item['case_id']/args.profile
        if (folder/'execution.json').exists():
            continue
        print('START', item['case_id'], args.profile, flush=True)
        try:
            if (folder/'manifest.json').exists():
                scene = json.loads((folder/'scene_seed.json').read_text())['scene']
                road = json.loads((folder/'source_road_seed.json').read_text())
            else:
                folder, scene, road = prepare(item, args.out, args.calibration_run, args.profile, args.simulation_seconds)
            if args.execute:
                run = client.run_scenario(folder/'map.xodr', folder/'scenario.xosc',
                    name='expand_'+item['case_id'][:3]+'_'+args.profile,
                    pcla_agent='if_if', sut_actor='hero', rgb_actor_role='hero',
                    max_seconds=600, timeout=1000, out_dir=folder/'run',
                    scene_seed={'scene': scene}, road_seed=road, paper_render=False, retries=0)
                write(folder/'execution.json', {'elapsed_s': run.elapsed_s, 'summary': run.summary,
                    'run_directory': str(run.local_dir), 'complete': True})
                print('DONE', item['case_id'], run.summary.get('termination_reason'), run.elapsed_s, flush=True)
        except Exception:
            folder.mkdir(parents=True, exist_ok=True)
            (folder/'error.txt').write_text(traceback.format_exc())
            print('ERROR', item['case_id'], traceback.format_exc(), flush=True)


if __name__ == '__main__':
    main()
