"""Compile a source-described partial intrusion without a complete lane change."""
import math
from xml.etree import ElementTree as ET


def intrusion_plan(npc, xodr_path, road_id, lane_id):
    if (npc.get('kind') != 'vehicle' or npc.get('position') != 'adjacent'
            or npc.get('side') not in ('left', 'right') or npc.get('relative_to', 'ego') != 'ego'):
        raise ValueError('partial_lane_intrusion requires a vehicle adjacent to ego')
    road = ET.parse(xodr_path).find(f"./road[@id='{road_id}']")
    sections = road.findall('./lanes/laneSection')
    if len(sections) != 1:
        raise ValueError('partial intrusion requires a constant lane section')
    lane = sections[0].find(f"./*/lane[@id='{lane_id}']")
    target_lane_id = lane_id + (1 if npc['side']=='right' else -1) * (1 if lane_id < 0 else -1)
    target_lane = sections[0].find(f"./*/lane[@id='{target_lane_id}']")
    if (lane is None or lane.get('type') != 'driving' or target_lane is None
            or target_lane.get('type') != 'driving' or target_lane_id*lane_id <= 0):
        raise ValueError('partial intrusion requires an adjacent same-direction driving lane')
    widths = lane.findall('width') if lane is not None else []
    if len(widths) != 1 or any(float(widths[0].get(k, 0)) for k in ('sOffset','b','c','d')):
        raise ValueError('partial intrusion requires a constant driving-lane width')
    width = float(widths[0].get('a'))
    parameters = npc.get('behavior', {}).get('params', {})
    fraction = float(parameters.get('offset_fraction', .4))
    acceleration = float(parameters.get('max_lateral_acceleration', .8))
    if not all(math.isfinite(v) for v in (width,fraction,acceleration)) or width <= 0 or not 0 < fraction < .5 or acceleration <= 0:
        raise ValueError('partial intrusion target must remain on its initial side of the lane boundary')
    # side describes the initial adjacent lane; the motion is toward ego.
    travel_left = 1 if npc['side']=='right' else -1
    target = width * fraction * travel_left * (1 if lane_id < 0 else -1)
    return {'road_id':int(road_id), 'lane_id':int(lane_id), 'lane_width_m':width,
            'offset_fraction':fraction, 'target_offset_xodr_m':target,
            'max_lateral_acceleration_mps2':acceleration,
            'default_parameters':[k for k in ('offset_fraction','max_lateral_acceleration') if k not in parameters],
            'method':'hold partial lane offset toward ego; default 0.4 lane width, 0.8m/s2 target ramp',
            'limitations':'vehicle footprint crossing and actual acceleration require runtime measurement; no forced collision'}
