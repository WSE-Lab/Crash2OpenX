"""Derive finite roadside pedestrian crossings from the newly compiled road.

This supplies geometry defaults, not source facts or case-specific trajectories.
The cross primitive is a straight walk normal to the initial road tangent.
"""
import math
from xml.etree import ElementTree as ET


def crossing_geometry(xodr_path, road_id, lane_id, reference_s, *, side,
                      gap=25.0, lateral=None, curb_clearance=0.75):
    source_side = side
    if side in (None, 'none'):
        side = 'right'
    if side not in ('left', 'right'):
        raise ValueError('pedestrian cross needs an explicit roadside side')
    road = ET.parse(xodr_path).find(f"./road[@id='{road_id}']")
    if road is None:
        raise ValueError('crossing reference road does not exist')
    direction = 1 if lane_id < 0 else -1
    s = reference_s + direction * float(gap)
    if not 0 <= s <= float(road.get('length')):
        raise ValueError('crossing spawn gap extends beyond its initial road')
    sections = road.findall('./lanes/laneSection')
    offsets = road.findall('./lanes/laneOffset')
    if (len(sections) != 1 or float(sections[0].get('s', 0)) != 0
            or len(offsets) > 1 or any(float(n.get(k, 0)) for n in offsets for k in ('s', 'b', 'c', 'd'))):
        raise ValueError('finite crossing requires constant lane sections and offsets')
    offset = float(offsets[0].get('a', 0)) if offsets else 0.0
    lanes = {}
    for road_side, sign in (('left', 1), ('right', -1)):
        distance = 0.0
        for lane in sorted(sections[0].findall(road_side+'/lane'), key=lambda n: abs(int(n.get('id')))):
            widths = lane.findall('width')
            if len(widths) != 1 or any(float(n.get(k, 0)) for n in widths for k in ('sOffset', 'b', 'c', 'd')):
                raise ValueError('finite crossing requires constant lane widths')
            width = float(widths[0].get('a'))
            if width < 0 or not math.isfinite(width):
                raise ValueError('invalid crossing lane width')
            a, b = offset + sign*distance, offset + sign*(distance+width)
            lanes[int(lane.get('id'))] = {'lo':min(a,b), 'hi':max(a,b), 'type':lane.get('type')}
            distance += width
    if lane_id not in lanes or lanes[lane_id]['type'] != 'driving':
        raise ValueError('crossing reference lane is not a driving lane')
    driving = [v for v in lanes.values() if v['type']=='driving']
    lo, hi = min(v['lo'] for v in driving), max(v['hi'] for v in driving)
    center = (lanes[lane_id]['lo']+lanes[lane_id]['hi'])/2
    outward = direction * (1 if side=='left' else -1)
    start = ((hi if outward > 0 else lo) + outward*curb_clearance
             if lateral is None else center + outward*float(lateral))
    finish = (lo-curb_clearance if outward > 0 else hi+curb_clearance)
    walkable = [v for v in lanes.values() if v['type'] in ('shoulder','sidewalk','parking')]
    def on_walkable(t):
        return any(v['lo']+0.30 <= t <= v['hi']-0.30 for v in walkable)
    if lateral is None and not on_walkable(start):
        raise ValueError('crossing start has no generated walkable road margin')
    if not on_walkable(finish):
        raise ValueError('crossing end has no generated walkable road margin')
    if not all(math.isfinite(v) for v in (s,start,finish)) or (finish-start)*(-outward) <= 0:
        raise ValueError('crossing endpoint is not across the initial road')
    return {'road_id':int(road_id), 'lane_id':int(lane_id), 's':s,
            'start_offset':start-center, 'end_offset':finish-center,
            'start_t':start, 'end_t':finish, 'distance_m':abs(finish-start),
            'driving_bounds_t':[lo,hi], 'curb_clearance_m':curb_clearance,
            'source_lateral_preserved':lateral is not None,
            'side':side, 'source_side':source_side,
            'side_is_compiler_default':source_side in (None, 'none'),
            'method':'constant-width XODR cross section; 0.75m curb clearance is a compiler default',
            'completion':'stop after straight crossing distance; observed endpoint requires runtime validation'}
