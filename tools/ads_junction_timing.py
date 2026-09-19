"""Estimate a junction departure window from the two compiled lane routes.

This configures an experiment, not a prediction of the ADS controller.
Runtime measurements remain the acceptance evidence.
"""
import math
import numpy as np
from runner.src.ads_geometry import body_clearance, hull


def sampled_route(root, actor):
    group = next(g for g in root.findall('.//ManeuverGroup')
                 if g.find('./Actors/EntityRef').get('entityRef') == actor)
    points = np.array([(float(p.get('x')), float(p.get('y'))) for p in group.findall('.//AssignRouteAction//WorldPosition')])
    if len(points) < 2:
        raise ValueError('A generated route is required for '+actor)
    ds = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep = np.r_[True, ds > 1e-6]
    points = points[keep]
    distances = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    grid = np.arange(0, distances[-1]+.01, .5)
    xy = np.column_stack([np.interp(grid, distances, points[:, axis]) for axis in (0, 1)])
    return grid, xy


def footprint(root, actor, xy, heading):
    box = root.find(f".//ScenarioObject[@name='{actor}']//BoundingBox")
    dimensions, center = box.find('Dimensions'), box.find('Center')
    length, width = float(dimensions.get('length')), float(dimensions.get('width'))
    cx, cy = float(center.get('x')), float(center.get('y'))
    c, s = math.cos(heading), math.sin(heading)
    return hull([(xy[0]+(a+cx)*c-(b+cy)*s, xy[1]+(a+cx)*s+(b+cy)*c)
                 for a in (-length/2, length/2) for b in (-width/2, width/2)])


def departure_window(root, actor, npc_speed=8, ego_speed=5, time_offset=0):
    if npc_speed <= 0 or ego_speed <= 0:
        raise ValueError('Positive target speeds required')
    es, ep = sampled_route(root, 'hero')
    ns, npcs = sampled_route(root, actor)
    distances = np.linalg.norm(ep[:, None, :]-npcs[None, :, :], axis=2)
    pairs = np.argwhere(distances < 1.0)
    if not len(pairs):
        raise ValueError('Generated routes have no common conflict within 1 m')
    i, j = min(pairs, key=lambda p: es[p[0]]+ns[p[1]])
    # The runtime departure ramp requests <=3 m/s2. Constant-speed travel plus
    # half the ramp duration approximates arrival after accelerating from rest.
    duration = ns[j]/npc_speed + max(1, npc_speed/3)/2
    ego_onset_s = es[i]-ego_speed*(duration+time_offset)
    if not 0 < ego_onset_s < es[-1]:
        raise ValueError('The requested arrival window precedes the ego spawn')
    index = int(np.argmin(abs(es-ego_onset_s)))
    heading = math.atan2(*(ep[min(index+1, len(ep)-1)]-ep[max(index-1, 0)])[::-1])
    npc_heading = math.atan2(*(npcs[1]-npcs[0])[::-1])
    clearance = body_clearance(footprint(root, 'hero', ep[index], heading),
                               footprint(root, actor, npcs[0], npc_heading))
    return {'distance_m': round(clearance, 3), 'npc_route_distance_to_conflict_m': float(ns[j]),
            'ego_route_distance_to_conflict_m': float(es[i]), 'ego_onset_route_s_m': float(es[index]),
            'assumed_ego_speed_mps': ego_speed, 'requested_npc_speed_mps': npc_speed,
            'estimated_npc_arrival_s': float(duration), 'time_offset_s': time_offset,
            'conflict_xy': ep[i].tolist(), 'method': 'earliest common generated lane-route point; finite acceleration approximation'}
