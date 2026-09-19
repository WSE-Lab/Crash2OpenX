"""Measure generated NPC route following and finite pedestrian crossing evidence."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET


def distance_to_segment(x, y, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    length2 = dx*dx+dy*dy
    f = max(0, min(1, ((x-a[0])*dx+(y-a[1])*dy)/length2)) if length2 else 0
    return math.hypot(x-a[0]-f*dx, y-a[1]-f*dy)


def review(run):
    result = json.loads((run/'result.json').read_text())
    tree = ET.parse(result['xosc_path'])
    trace = [json.loads(line) for line in (run/'sim_trace_raw.jsonl').read_text().splitlines()]
    events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
    routes = {}
    for private in tree.findall('.//Private'):
        metadata = private.find(".//Property[@name='C2XGeneratedLaneRoute']")
        if metadata is None:
            continue
        actor = private.get('entityRef')
        points = [(p['x'], -p['y']) for p in json.loads(metadata.get('value'))]
        measured = []
        for frame in trace:
            state = frame['actors'].get(actor)
            if state:
                measured.append({'time': frame['simulation_time'], 'distance_m': min(
                    distance_to_segment(state['x'], state['y'], a, b) for a, b in zip(points, points[1:]))})
        if not measured:
            raise ValueError('Missing generated-route actor trace: '+actor)
        errors = sorted(x['distance_m'] for x in measured)
        routes[actor] = {'samples': len(measured), 'max_deviation': max(measured, key=lambda v:v['distance_m']),
                         'p95_distance_m': errors[math.ceil(len(errors)*.95)-1],
                         'samples_over_1m': sum(v > 1 for v in errors),
                         'method': 'Actor origin to compiled route polyline in CARLA coordinates; not footprint/lane-legality acceptance'}
    pedestrians = {}
    plans = tree.find(".//ParameterDeclaration[@name='C2XPedestrianCrossings']")
    for actor, plan in (json.loads(plans.get('value')) if plans is not None else {}).items():
        states = [(frame['simulation_time'], frame['actors'][actor]) for frame in trace if actor in frame['actors']]
        if not states:
            raise ValueError('Missing pedestrian trace: '+actor)
        start, finish = states[0][1], states[-1][1]
        stops = [e for e in events if e.get('event_type')=='storyboard_transition'
                 and e['payload'].get('element_name')==actor+'_cross_stop' and e['payload'].get('transition')=='START']
        after = [(t, s['planar_speed_mps']) for t,s in states if stops and t >= stops[0]['simulation_time']]
        moving = [t for t,speed in after if speed > .01]
        pedestrians[actor] = {'compiled_plan': plan, 'stop_event': stops[0] if stops else None,
            'measured_displacement_m': math.hypot(finish['x']-start['x'], finish['y']-start['y']),
            'max_planar_speed_after_stop_event_mps': max(speed for _,speed in after) if after else None,
            'last_time_above_0_01_mps_after_stop_event': max(moving) if moving else None,
            'final_planar_speed_mps': finish['planar_speed_mps'],
            'minimum_z': min(s['z'] for _,s in states),
            'start_xyz': [start[k] for k in ('x','y','z')], 'end_xyz': [finish[k] for k in ('x','y','z')]}
    return {'run': str(run.resolve()), 'generated_routes': routes, 'pedestrian_crossings': pedestrians,
            'actor_minimum_z': {actor:min(f['actors'][actor]['z'] for f in trace if actor in f['actors'])
                                for actor in trace[0]['actors']},
            'summary': json.loads((run/'summary.json').read_text()),
            'source_fidelity_accepted': False, 'autonomous_task_success': None,
            'limitations': ['Measurement alone does not validate source actor roles, interaction order, or impact location.',
                            'Only a single run; compare input and runtime hashes before attributing changed behavior.'],
            'sha256': {name:hashlib.sha256((run/name).read_bytes()).hexdigest() for name in
                       ('sim_trace_raw.jsonl','events.jsonl','carla_rgb.mp4','demo.log')}}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    args.output.write_text(json.dumps(review(args.run),ensure_ascii=False,indent=2)+'\n')
