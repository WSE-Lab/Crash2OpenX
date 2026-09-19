"""Planar CARLA body geometry shared by live trigger checks and trace review."""
import math


def hull(vertices):
    points = sorted(set((float(p[0]), float(p[1])) for p in vertices))
    if len(points) < 3:
        raise ValueError('at least three body vertices required')
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for collection, sequence in ((lower, points), (upper, reversed(points))):
        for point in sequence:
            while len(collection) >= 2 and cross(collection[-2], collection[-1], point) <= 0:
                collection.pop()
            collection.append(point)
    return lower[:-1]+upper[:-1]


def axes(a, b):
    for polygon in (a, b):
        for p, q in zip(polygon, polygon[1:]+polygon[:1]):
            dx, dy = q[0]-p[0], q[1]-p[1]
            length = math.hypot(dx, dy)
            if length > 1e-9:
                yield -dy/length, dx/length


def projection(polygon, axis):
    values = [p[0]*axis[0]+p[1]*axis[1] for p in polygon]
    return min(values), max(values)


def body_clearance(a, b):
    """Shortest distance between two convex footprints; overlap returns zero."""
    if all(not (projection(a, axis)[1] < projection(b, axis)[0] or
                projection(b, axis)[1] < projection(a, axis)[0]) for axis in axes(a, b)):
        return 0.0
    def point_segment(p, u, v):
        dx, dy = v[0]-u[0], v[1]-u[1]
        k = max(0.0, min(1.0, ((p[0]-u[0])*dx+(p[1]-u[1])*dy)/(dx*dx+dy*dy)))
        return math.hypot(p[0]-u[0]-k*dx, p[1]-u[1]-k*dy)
    return min(point_segment(p, u, v) for pset, edges in ((a, b), (b, a))
               for p in pset for u, v in zip(edges, edges[1:]+edges[:1]))


def constant_velocity_ttc(a, b, relative_velocity):
    """Swept separating axes, fixed orientations and constant planar velocities.

    Parallel cars in separate lanes have no predicted collision, hence None.
    This is a projection, not a forecast of either controller's next action.
    """
    enter, leave = 0.0, math.inf
    for axis in axes(a, b):
        amin, amax = projection(a, axis)
        bmin, bmax = projection(b, axis)
        speed = relative_velocity[0]*axis[0]+relative_velocity[1]*axis[1]
        if abs(speed) < 1e-9:
            if amax < bmin or bmax < amin:
                return None
            continue
        first, last = sorted(((amin-bmax)/speed, (amax-bmin)/speed))
        enter, leave = max(enter, first), min(leave, last)
        if enter > leave:
            return None
    return enter if leave >= enter else None


def pair_metrics(ego, npc):
    a = hull(ego['bounding_box_world_vertices'])
    b = hull(npc['bounding_box_world_vertices'])
    dx, dy = npc['x']-ego['x'], npc['y']-ego['y']
    dvx, dvy = npc['vx']-ego['vx'], npc['vy']-ego['vy']
    center = math.hypot(dx, dy)
    speed = math.hypot(ego['vx'], ego['vy'])
    gap = body_clearance(a, b)
    yaw = math.radians(ego['yaw'])
    ahead = dx*math.cos(yaw)+dy*math.sin(yaw)
    return {'body_clearance_m': gap, 'center_distance_m': center,
            'radial_closing_speed_mps': -(dx*dvx+dy*dvy)/center if center > 1e-9 else 0.0,
            'ego_speed_mps': speed, 'npc_speed_mps': math.hypot(npc['vx'], npc['vy']),
            'npc_forward_m': ahead,
            'constant_velocity_ttc_s': constant_velocity_ttc(a, b, (dvx, dvy)),
            'clearance_over_ego_speed_s': gap/speed if speed > .1 else None}
