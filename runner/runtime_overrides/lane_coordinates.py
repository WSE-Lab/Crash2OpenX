"""OpenDRIVE lane offsets in CARLA's left-handed world coordinates."""
import math


def lane_heading(lane_yaw_degrees, heading_radians=0, absolute=False, use_carla_coordinates=False):
    """Resolve OSC orientation at the target lane point, including curves."""
    delta = math.degrees(float(heading_radians))
    if not use_carla_coordinates:
        delta = -delta
    return delta if absolute else lane_yaw_degrees + delta


def lane_offset_delta(lane_yaw_degrees, lane_id, offset, use_carla_coordinates=False):
    """Return a world XY displacement using the lane, not actor orientation.

    OpenDRIVE t grows left of the road reference line. CARLA waypoint yaw
    follows lane travel, reversing the reference direction on positive lanes.
    CARLA-native scenario coordinates retain the runtime's rightward offset.
    """
    if not lane_id:
        raise ValueError("lane offset requires a nonzero driving lane id")
    rightward = float(offset)
    if not use_carla_coordinates:
        rightward *= 1 if lane_id > 0 else -1
    yaw = math.radians(lane_yaw_degrees)
    return -math.sin(yaw) * rightward, math.cos(yaw) * rightward
