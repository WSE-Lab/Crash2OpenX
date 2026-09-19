"""Resolve compiler-selected CARLA lane IDs without nearest-lane replanning."""
import math
from xml.etree import ElementTree as ET


def resolve_generated_lane_route(road_map, records):
    if not isinstance(records, list) or len(records) < 2:
        raise ValueError('generated lane route needs at least two sampled waypoints')
    lengths = {int(r.get('id')):float(r.get('length'))
               for r in ET.fromstring(road_map.to_opendrive()).findall('road')}
    result = []
    for record in records:
        road, lane, s = int(record['road_id']), int(record['lane_id']), float(record['s'])
        length = lengths.get(road)
        if length is None or not math.isfinite(s) or not -0.001 <= s <= length+0.001:
            raise ValueError('generated lane-route station does not belong to runtime map')
        # Extracted stations are rounded, and CARLA rejects some exact road ends.
        station = min(length-0.001, max(0.001, s))
        point = road_map.get_waypoint_xodr(road, lane, station)
        if point is None:
            raise ValueError('generated lane-route waypoint absent from runtime map')
        expected_x, expected_y = float(record['x']), -float(record['y'])
        error = math.hypot(point.transform.location.x-expected_x,
                           point.transform.location.y-expected_y)
        yaw_error = abs((point.transform.rotation.yaw+float(record['yaw'])+180)%360-180)
        if not math.isfinite(error) or error > 0.25 or yaw_error > 2:
            raise ValueError('generated lane route disagrees with sampled runtime geometry: '
                             'distance={} yaw={}'.format(error, yaw_error))
        result.append(point)
    return result
