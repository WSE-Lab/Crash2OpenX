"""Independent centre-in-lane review using OpenDRIVE line/arc/clothoid geometry.

Preserves CARLA's raw waypoint metrics. This checks a point, not vehicle-footprint
occupancy, lane legality, source fidelity or autonomous driving success.
"""
import hashlib
import math
from pathlib import Path
from xml.etree import ElementTree as ET


class CorridorMap:
    def __init__(self, path):
        from pyclothoids import Clothoid
        self.geometries = []
        self.unsupported = []
        self.sha256 = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        for road in ET.parse(path).findall('road'):
            rid = road.get('id')
            sections = road.findall('./lanes/laneSection')
            offsets = road.findall('./lanes/laneOffset')
            if (len(sections) != 1 or float(sections[0].get('s', 0)) != 0
                    or len(offsets) > 1 or any(float(n.get(k, 0)) for n in offsets for k in ('s', 'b', 'c', 'd'))):
                self.unsupported.append({'road': rid, 'reason': 'nonconstant lane sections or offsets'})
                continue
            offset = float(offsets[0].get('a', 0)) if offsets else 0
            lanes, supported = [], True
            for side, sign in (('left', 1), ('right', -1)):
                cumulative = 0
                nodes = sorted(sections[0].findall(side+'/lane'), key=lambda n: abs(int(n.get('id'))))
                for node in nodes:
                    widths = node.findall('width')
                    if len(widths) != 1 or any(float(n.get(k, 0)) for n in widths for k in ('sOffset', 'b', 'c', 'd')):
                        supported = False
                        break
                    width = float(widths[0].get('a', 0))
                    if width < 0 or not math.isfinite(width):
                        supported = False
                        break
                    if node.get('type') == 'driving' and width > 0:
                        a, b = offset+sign*cumulative, offset+sign*(cumulative+width)
                        lanes.append({'lane_id': int(node.get('id')), 'lo': min(a, b), 'hi': max(a, b)})
                    cumulative += width
            if not supported:
                self.unsupported.append({'road': rid, 'reason': 'nonconstant lane widths'})
                continue
            for geom in road.findall('./planView/geometry'):
                x, y, h, length = (float(geom.get(k, 0)) for k in ('x', 'y', 'hdg', 'length'))
                if length <= 0:
                    self.unsupported.append({'road': rid, 'reason': 'nonpositive geometry length'})
                    continue
                kind = 'line'
                k0 = k1 = 0
                if geom.find('spiral') is not None:
                    kind = 'spiral'; node = geom.find('spiral')
                    k0, k1 = float(node.get('curvStart')), float(node.get('curvEnd'))
                elif geom.find('arc') is not None:
                    kind = 'arc'; k0 = k1 = float(geom.find('arc').get('curvature'))
                elif geom.find('line') is None:
                    self.unsupported.append({'road': rid, 'reason': 'unsupported plan geometry'})
                    continue
                curve = None if kind == 'line' else Clothoid.StandardParams(x, y, h, k0, (k1-k0)/length, length)
                self.geometries.append({'road_id': rid, 'x': x, 'y': y, 'h': h,
                    'length': length, 'curve': curve, 's0': float(geom.get('s', 0)), 'lanes': lanes})

    def project(self, x, y, tolerance_m=1e-4):
        """Coordinates are OpenDRIVE, with numerical tolerance reported explicitly."""
        if not math.isfinite(x) or not math.isfinite(y):
            return {'classification': 'unresolved', 'reason': 'nonfinite coordinates'}
        candidates = []
        for g in self.geometries:
            curve = g['curve']
            if curve is None:
                h = g['h']; c, s = math.cos(h), math.sin(h)
                along = min(g['length'], max(0., (x-g['x'])*c+(y-g['y'])*s))
                px, py = g['x']+along*c, g['y']+along*s
            else:
                along = curve.ClosestPointArcLength(x, y)
                px, py, h = curve.X(along), curve.Y(along), curve.Theta(along)
            dx, dy = x-px, y-py
            tangent_residual = dx*math.cos(h)+dy*math.sin(h)
            lateral = -dx*math.sin(h)+dy*math.cos(h)
            for lane in g['lanes']:
                excess = max(lane['lo']-lateral, lateral-lane['hi'], 0.)
                outside = max(abs(tangent_residual), excess)
                candidates.append({'road_id': g['road_id'], 'lane_id': lane['lane_id'],
                    's': g['s0']+along, 'lateral_t_m': lateral,
                    'lane_bounds_t_m': [lane['lo'], lane['hi']],
                    'center_distance_m': abs(lateral-(lane['lo']+lane['hi'])/2),
                    'lateral_excess_m': excess, 'tangent_residual_m': tangent_residual,
                    'inside': outside <= tolerance_m, '_distance': math.hypot(tangent_residual, excess)})
        if not candidates:
            return {'classification': 'unresolved', 'reason': 'no supported driving-lane geometry'}
        best = min(candidates, key=lambda c: (not c['inside'], c['_distance'], c['center_distance_m']))
        best.pop('_distance')
        best['classification'] = 'inside' if best['inside'] else ('unresolved' if self.unsupported else 'outside')
        return best


def review_trace(xodr, rows, actor='hero'):
    geometry = CorridorMap(xodr)
    results = []
    missing = 0
    for row in rows:
        state = row.get('actors', {}).get(actor)
        if state is None:
            missing += 1
            continue
        projection = geometry.project(float(state['x']), -float(state['y']))
        results.append({'simulation_time': row['simulation_time'], **projection,
                        'raw_waypoint_distance_m': state.get('nearest_driving_waypoint_distance_m'),
                        'raw_road_id': state.get('road_id'), 'raw_lane_id': state.get('lane_id')})
    with_raw = [r for r in results if isinstance(r['raw_waypoint_distance_m'], (int, float))]
    return {'method': 'OpenDRIVE constant-width lane corridors; exact line/arc or pyclothoids projection',
            'coordinate_conversion': 'raw CARLA x,y -> OpenDRIVE x,-y', 'numerical_tolerance_m': 1e-4,
            'point_scope': 'actor origin/centre; does not establish full vehicle-footprint containment',
            'map_sha256': geometry.sha256, 'unsupported': geometry.unsupported,
            'missing_actor_samples': missing,
            'samples': len(results), 'inside_samples': sum(r['classification'] == 'inside' for r in results),
            'outside_samples': sum(r['classification'] == 'outside' for r in results),
            'unresolved_samples': sum(r['classification'] == 'unresolved' for r in results),
            'peak_raw_projection': max(with_raw, key=lambda r: r['raw_waypoint_distance_m']) if with_raw else None,
            'non_inside_samples': [r for r in results if r['classification'] != 'inside'],
            'raw_metrics_overwritten': False, 'source_fidelity_accepted': False}
