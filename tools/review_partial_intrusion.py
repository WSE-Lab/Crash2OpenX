"""Measure body-boundary intrusion on the generated initial straight road.

Keeps actor-centre, actual bounding-box and collision evidence distinct.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

from tools.xodr_corridor_review import CorridorMap


def straight_route_projection(points, state):
    """Signed left offset in XODR coordinates along a verified straight route.

    The axis extends only between the compiled endpoints. This measures a lane
    corridor through a junction, where the boundary need not be painted.
    """
    start, end = points[0], points[-1]
    dx, dy = end['x']-start['x'], end['y']-start['y']
    length = math.hypot(dx, dy)
    if length <= 0:
        raise ValueError('Degenerate straight route')
    c, s = dx/length, dy/length
    def project(x, y):
        x, y = x-start['x'], y-start['y']
        return x*c+y*s, -x*s+y*c
    stations = [project(p['x'], p['y']) for p in points]
    if (any(abs(t) > .025 for _,t in stations)
            or any(b[0] < a[0]-.025 for a,b in zip(stations,stations[1:]))):
        raise ValueError('Compiled route is not collinear and forward ordered')
    station, offset = project(state['x'], -state['y'])
    body = [project(v[0], -v[1])[1] for v in state['bounding_box_world_vertices']]
    return {'station_m':station, 'offset_left_m':offset,
            'body_left_m':max(body), 'body_right_m':min(body),
            'within_compiled_endpoints': -.025 <= station <= length+.025}


def route_intrusion_review(points, frames, actor, plan):
    target_left = plan['target_offset_xodr_m'] * (1 if plan['lane_id'] < 0 else -1)
    half_width = plan['lane_width_m']/2
    samples=[]
    for frame in frames:
        state=frame['actors'].get(actor)
        if state is None:continue
        item=straight_route_projection(points,state)
        penetration=(item['body_left_m']-half_width if target_left>0
                     else -half_width-item['body_right_m'])
        item.update(time=frame['simulation_time'],body_intrusion_m=max(0,penetration),
                    centre_in_route_lane=abs(item['offset_left_m'])<=half_width)
        samples.append(item)
    in_route=[s for s in samples if s['within_compiled_endpoints']]
    partial=[s for s in in_route if s['body_intrusion_m']>.01 and s['centre_in_route_lane']]
    return {'method':'collinear compiled CARLA lane route, actual actor world bounding box',
            'target_offset_left_m':target_left, 'samples_within_compiled_endpoints':len(in_route),
            'samples_beyond_compiled_endpoints':len(samples)-len(in_route),
            'partial_body_intrusion_samples':len(partial),
            'first_partial_intrusion':partial[0] if partial else None,
            'last_partial_intrusion':partial[-1] if partial else None,
            'centre_outside_route_lane_samples':sum(not s['centre_in_route_lane'] for s in in_route),
            'peak_body_intrusion':max(in_route,key=lambda v:v['body_intrusion_m']) if in_route else None,
            'samples':samples,
            'limitation':'Includes virtual lane corridor through the junction; not painted-boundary or collision acceptance.'}


def review(run):
    result=json.loads((run/'result.json').read_text())
    xodr=Path(result['xodr_path'])
    graph=json.loads((Path(result['roadgraph_cache_dir'])/'roadgraph_selfcheck.json').read_text())
    if not graph.get('geometry_consistency_pass'):
        raise ValueError('Actual CARLA geometry consistency must be checked first')
    root=ET.parse(result['xosc_path'])
    declaration=root.find(".//ParameterDeclaration[@name='C2XPartialLaneIntrusions']")
    if declaration is None:
        raise ValueError('No compiled partial-intrusion plan')
    plans=json.loads(declaration.get('value'))
    model=CorridorMap(xodr)
    frames=[json.loads(line) for line in (run/'sim_trace_raw.jsonl').read_text().splitlines()]
    events=[json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
    actors={}
    for actor,plan in plans.items():
        geometries=[g for g in model.geometries if g['road_id']==str(plan['road_id'])]
        if len(geometries)!=1 or geometries[0]['curve'] is not None:
            raise ValueError('This reviewer requires a straight initial road')
        g=geometries[0]
        lane=next(l for l in g['lanes'] if l['lane_id']==plan['lane_id'])
        centre=(lane['lo']+lane['hi'])/2
        c,s=math.cos(g['h']),math.sin(g['h'])
        def project(x,y):
            dx,dy=x-g['x'],-y-g['y']
            return dx*c+dy*s,-dx*s+dy*c
        samples=[];outside_station=0
        for frame in frames:
            state=frame['actors'].get(actor)
            if state is None:continue
            station,t=project(state['x'],state['y'])
            if not 0 <= station <= g['length']:
                outside_station+=1;continue
            body=[project(v[0],v[1])[1] for v in state['bounding_box_world_vertices']]
            penetration=max(body)-lane['hi'] if plan['target_offset_xodr_m']>0 else lane['lo']-min(body)
            samples.append({'time':frame['simulation_time'],'station':station,'centre_t':t,
                            'offset_m':t-centre,'body_intrusion_m':max(0,penetration),
                            'centre_in_original_lane':lane['lo']<=t<=lane['hi']})
        if not samples:raise ValueError('No actor samples on its initial road')
        partial=[v for v in samples if v['body_intrusion_m']>.01 and v['centre_in_original_lane']]
        actors[actor]={'compiled_plan':plan,'initial_road_samples':len(samples),
                       'samples_after_leaving_initial_road':outside_station,
                       'partial_body_intrusion_samples':len(partial),
                       'first_partial_intrusion':partial[0] if partial else None,
                       'peak_body_intrusion':max(samples,key=lambda v:v['body_intrusion_m']),
                       'centre_outside_original_lane_samples':sum(not v['centre_in_original_lane'] for v in samples),
                       'minimum_offset_m':min(v['offset_m'] for v in samples),
                       'maximum_offset_m':max(v['offset_m'] for v in samples),
                       'samples':samples}
        metadata=root.find(".//Private[@entityRef='"+actor+"']//Property[@name='C2XGeneratedLaneRoute']")
        if metadata is not None:
            actors[actor]['compiled_straight_route_review']=route_intrusion_review(
                json.loads(metadata.get('value')),frames,actor,plan)
    return {'run':str(run.resolve()),'actors':actors,
            'actor_type_ids':{a:frames[0]['actors'][a]['type_id'] for a in frames[0]['actors']},
            'actor_minimum_z':{a:min(f['actors'][a]['z'] for f in frames if a in f['actors']) for a in frames[0]['actors']},
            'collision_events':[e for e in events if e['event_type']=='collision'],
            'summary':json.loads((run/'summary.json').read_text()),
            'source_fidelity_accepted':False,
            'limitations':['Only the initial straight-road section is measured.',
                           'A measured body-boundary intrusion is not collision or full source-fidelity acceptance.',
                           'PCLA SUT may differ from the report AV; source roles must be disclosed.'],
            'sha256':{name:hashlib.sha256((run/name).read_bytes()).hexdigest() for name in
                      ('sim_trace_raw.jsonl','events.jsonl','carla_rgb.mp4','demo.log')}}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args()
    args.output.write_text(json.dumps(review(args.run),ensure_ascii=False,indent=2)+'\n')
