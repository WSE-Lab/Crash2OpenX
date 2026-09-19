"""Render OpenDRIVE lane markings as CARLA world debug primitives.

These marks are part of the live simulation view, not MP4 post-processing.
They add no collision geometry and do not alter agent controls. This is an
explicit rendering approximation for the untextured runtime OpenDRIVE mesh.
"""
import math
import carla


def draw_opendrive_markings(world, *, brightness=1.0, life_time=3600):
    if not 0 < brightness <= 1:
        raise ValueError('marking brightness must be in (0, 1]')
    road_map = world.get_map()
    seen = set()
    count = 0
    for wp in road_map.generate_waypoints(1.0):
        if wp.is_junction or wp.lane_type != carla.LaneType.Driving:
            continue
        end = road_map.get_waypoint_xodr(wp.road_id, wp.lane_id, wp.s + 1.0)
        if end is None or end.is_junction:
            continue
        for side, marking in ((-1, wp.left_lane_marking), (1, wp.right_lane_marking)):
            kind = str(marking.type)
            if kind in ("NONE", "None", "Other"):
                continue
            if "Broken" in kind and wp.s % 12 >= 4:
                continue
            points = []
            for point in (wp, end):
                transform = point.transform
                yaw = math.radians(transform.rotation.yaw)
                offset = side * point.lane_width / 2
                points.append(carla.Location(x=transform.location.x - math.sin(yaw) * offset,
                                             y=transform.location.y + math.cos(yaw) * offset,
                                             z=transform.location.z + .035))
            key = tuple(sorted((round(p.x, 2), round(p.y, 2)) for p in points))
            if key in seen:
                continue
            seen.add(key)
            rgb = (240, 195, 25) if "Yellow" in str(marking.color) else (245, 245, 245)
            color = carla.Color(*(max(1, round(value*brightness)) for value in rgb))
            world.debug.draw_line(points[0], points[1], thickness=max(.1, float(marking.width)),
                                  color=color, life_time=life_time, persistent_lines=True)
            count += 1
    print("C2X_ROAD_MARKINGS segments={} source=OpenDRIVE representation=CARLA_world_debug_lines collision_geometry=false brightness={}".format(count, brightness), flush=True)
    return count
