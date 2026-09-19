#!/usr/bin/env python3
"""Mechanical builders for agent-authored accident replay trace scenes.

This module intentionally does not infer accident semantics.  The Codex agent
authors semantic_scene.json and trace_scene.json from the skill instructions;
this tool validates the trace geometry, emits a minimal OpenDRIVE road file,
emits an OpenSCENARIO replay where actors follow those traces, and runs XSD
validation when requested.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

import xmlschema

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import scenariogeneration.xodr as xodr
import scenariogeneration.xosc as xosc
from tools import opendrive_repair
from tools.stage4_xosc import _build_env_action, _make_pedestrian, _make_vehicle


TRACE_SCHEMA = Path("schemas/trace_scene_schema.json")
STANDARD_LANE_WIDTH_M = 6.0
STANDARD_CROSS_JUNCTION_ROAD_LENGTH_M = 100.0
STANDARD_CROSS_JUNCTION_RADIUS_M = 65.0
STANDARD_CROSS_JUNCTION_ID = 100
STANDARD_SAFETY_SHOULDER_WIDTH_M = 10.0
MIN_POST_COLLISION_TIME_S = 2.0
DEFAULT_LANES_PER_DIRECTION = 1
MAX_LANES_PER_DIRECTION = 5
ADJACENT_LANE_COLLISION_TYPES = {"adjacent_lane", "lane_change", "parallel_conflict", "sideswipe"}
OPPOSING_LANE_MARKERS = ("opposing lane", "opposing_lane", "double yellow", "center line", "centerline")
DEFAULT_LANE_WIDTH_M = STANDARD_LANE_WIDTH_M
INTERSECTION_LAYOUTS = {
    "common_cross_junction",
    "cross_junction_with_bike_lane",
    "t_junction",
    "orthogonal_turn_left",
    "orthogonal_turn_right",
    "y_junction",
}
SUPPORTED_LAYOUTS = {
    "curved_corridor",
    "straight_corridor",
    "single_corridor",
    "curbside_parking_corridor",
    "straight_with_centerline_passing",
    "ramp_merge",
    "ramp_fork",
    *INTERSECTION_LAYOUTS,
}
DEFAULT_MAX_TRACE_SEGMENT_LENGTH_M = 45.0
DEFAULT_MAX_SCENE_SPAN_M = 220.0
DEFAULT_MAX_ABS_COORD_M = 180.0
DEFAULT_FOOTPRINT_DIMENSIONS_M = {
    "vehicle": (4.5, 2.1),
    "bus": (12.0, 2.6),
    "truck": (8.0, 2.6),
    "pedestrian": (0.6, 0.6),
    "cyclist": (1.8, 0.6),
    "obstacle": (0.6, 0.6),
}


def _as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _as_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")


def validate_trace_schema(trace_scene: dict[str, Any]) -> list[str]:
    # The JSON schema is the documented contract; runtime checks stay
    # dependency-free so this helper does not require extra packages.
    errors: list[str] = []
    for key in [
        "metadata",
        "environment",
        "coordinate_frame",
        "duration_s",
        "collision_time_s",
        "collision_point",
        "collision",
        "actors",
        "road_generation",
        "pipeline",
    ]:
        if key not in trace_scene:
            errors.append(f"missing required key: {key}")
    if trace_scene.get("coordinate_frame") != "generated_local_map":
        errors.append("coordinate_frame must be generated_local_map")
    duration = _float_or_none(trace_scene.get("duration_s"))
    collision_time = _float_or_none(trace_scene.get("collision_time_s"))
    if duration is None or duration <= 0.0:
        errors.append("duration_s must be a positive number")
    if collision_time is None or collision_time < 0.0:
        errors.append("collision_time_s must be a non-negative number")
    if duration is not None and collision_time is not None:
        if collision_time >= duration:
            errors.append("collision_time_s must be less than duration_s")
        if duration - collision_time < MIN_POST_COLLISION_TIME_S:
            errors.append(f"duration_s must leave at least {MIN_POST_COLLISION_TIME_S:g}s after collision_time_s")
    actors = _as_dict(trace_scene.get("actors"))
    if not actors:
        errors.append("actors must contain at least one actor")
    for actor_id, actor_value in actors.items():
        actor = _as_dict(actor_value)
        actor_type = actor.get("type")
        if actor_type not in {"vehicle", "pedestrian", "cyclist", "obstacle"}:
            errors.append(f"{actor_id}: invalid actor type {actor_type!r}")
        controller = actor.get("controller")
        if controller not in {"scripted", "static"}:
            errors.append(f"{actor_id}: invalid controller {controller!r}")
        if controller == "static":
            if "pose" not in actor:
                errors.append(f"{actor_id}: static actor requires pose")
        else:
            trace = [_as_dict(point) for point in _as_list(actor.get("trace"))]
            if len(trace) < 2:
                errors.append(f"{actor_id}: scripted actor requires at least two trace points")
            errors.extend(_validate_trace_timing(actor_id, trace, duration))
    collision = _as_dict(trace_scene.get("collision"))
    if collision.get("initiator") not in actors:
        errors.append("collision.initiator must match an actor id")
    if collision.get("target") not in actors:
        errors.append("collision.target must match an actor id")
    errors.extend(_validate_coordinate_scale(trace_scene))
    errors.extend(_validate_road_generation(trace_scene))
    return errors


def _float_or_none(value: Any) -> float | None:
    try:
        return float(value)
    except Exception:
        return None


def _validate_trace_timing(actor_id: str, trace: list[dict[str, Any]], duration: float | None) -> list[str]:
    errors: list[str] = []
    previous_t: float | None = None
    for idx, point in enumerate(trace):
        t = _float_or_none(point.get("t"))
        if t is None:
            errors.append(f"{actor_id}: trace[{idx}].t must be numeric")
            continue
        if t < 0.0:
            errors.append(f"{actor_id}: trace[{idx}].t must be >= 0")
        if duration is not None and t > duration:
            errors.append(f"{actor_id}: trace[{idx}].t must be <= duration_s")
        if previous_t is not None and t <= previous_t:
            errors.append(f"{actor_id}: trace timestamps must be strictly increasing")
        previous_t = t
    if trace and _float_or_none(trace[0].get("t")) != 0.0:
        errors.append(f"{actor_id}: trace must start at t=0.0; use repeated poses to model waiting")
    return errors


def _validate_coordinate_scale(trace_scene: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    qa_config = _as_dict(trace_scene.get("qa_config"))
    max_segment = float(qa_config.get("max_trace_segment_length_m", DEFAULT_MAX_TRACE_SEGMENT_LENGTH_M))
    max_span = float(qa_config.get("max_scene_span_m", DEFAULT_MAX_SCENE_SPAN_M))
    max_abs = float(qa_config.get("max_abs_coordinate_m", DEFAULT_MAX_ABS_COORD_M))
    xs: list[float] = []
    ys: list[float] = []

    for actor_id, actor_value in _as_dict(trace_scene.get("actors")).items():
        actor = _as_dict(actor_value)
        points = [_as_dict(point) for point in (_as_list(actor.get("trace")) or [_as_dict(actor.get("pose"))])]
        previous: dict[str, Any] | None = None
        for idx, point in enumerate(points):
            if not point:
                continue
            x = float(point.get("x", 0.0))
            y = float(point.get("y", 0.0))
            xs.append(x)
            ys.append(y)
            if max(abs(x), abs(y)) > max_abs:
                errors.append(
                    f"{actor_id}: point[{idx}] coordinate ({x:.1f}, {y:.1f}) exceeds local-map limit {max_abs:g}m"
                )
            if previous is not None:
                step = math.hypot(x - float(previous.get("x", 0.0)), y - float(previous.get("y", 0.0)))
                if step > max_segment:
                    errors.append(
                        f"{actor_id}: trace gap {step:.1f}m between adjacent points exceeds {max_segment:g}m"
                    )
            previous = point

    if xs and ys:
        span = max(max(xs) - min(xs), max(ys) - min(ys))
        if span > max_span:
            errors.append(f"trace scene span {span:.1f}m exceeds local-map limit {max_span:g}m")
    return errors


def _validate_close(name: str, actual: Any, expected: float, errors: list[str], tolerance: float = 1e-6) -> None:
    value = _float_or_none(actual)
    if value is None or abs(value - expected) > tolerance:
        errors.append(f"road_generation.{name} must be {expected:g}")


def _validate_road_generation(trace_scene: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    road_generation = _as_dict(trace_scene.get("road_generation"))
    layout = str(road_generation.get("layout") or "")
    if road_generation.get("mode") != "generated_open_drive":
        errors.append("road_generation.mode must be generated_open_drive")
    if not layout:
        errors.append("road_generation.layout must be explicit")
    elif layout not in SUPPORTED_LAYOUTS:
        errors.append(f"road_generation.layout {layout!r} is not supported by the deterministic generator")
    _validate_close("lane_width_m", road_generation.get("lane_width_m"), STANDARD_LANE_WIDTH_M, errors)
    _validate_close(
        "safety_shoulder_width_m",
        road_generation.get("safety_shoulder_width_m"),
        STANDARD_SAFETY_SHOULDER_WIDTH_M,
        errors,
    )
    expected_lanes = _required_lanes_per_direction(trace_scene)
    actual_lanes = _road_lanes_per_direction(trace_scene)
    if "lanes_per_direction" not in road_generation:
        errors.append("road_generation.lanes_per_direction must be explicit")
    if actual_lanes < expected_lanes:
        errors.append(
            "road_generation.lanes_per_direction must be at least "
            f"{expected_lanes} for collision.type={_as_dict(trace_scene.get('collision')).get('type')!r}"
        )

    if layout in INTERSECTION_LAYOUTS:
        _validate_close("road_length", road_generation.get("road_length"), STANDARD_CROSS_JUNCTION_ROAD_LENGTH_M, errors)
        _validate_close("junction_radius", road_generation.get("junction_radius"), STANDARD_CROSS_JUNCTION_RADIUS_M, errors)
        junction_id = road_generation.get("junction_id")
        try:
            junction_id_value = int(junction_id)
        except Exception:
            junction_id_value = None
        if junction_id_value != STANDARD_CROSS_JUNCTION_ID:
            errors.append(f"road_generation.junction_id must be {STANDARD_CROSS_JUNCTION_ID}")
        if _as_list(road_generation.get("roads")):
            errors.append("intersection road_generation must use a generated OpenDRIVE junction, not ad hoc overlapping roads")
    return errors


def _required_lanes_per_direction(trace_scene: dict[str, Any]) -> int:
    road_generation = _as_dict(trace_scene.get("road_generation"))
    collision_type = str(_as_dict(trace_scene.get("collision")).get("type") or "").lower()
    evidence_text = " ".join(
        [
            str(road_generation.get("layout") or ""),
            str(road_generation.get("junction_type") or ""),
            str(road_generation.get("notes") or ""),
            str(_as_dict(road_generation.get("lane_evidence")).get("conclusion") or ""),
            " ".join(str(item) for item in _as_list(_as_dict(road_generation.get("lane_evidence")).get("source_snippets"))),
        ]
    ).lower()
    is_opposing_lane_case = any(marker in evidence_text for marker in OPPOSING_LANE_MARKERS)
    configured_min = _float_or_none(road_generation.get("min_lanes_per_direction"))
    if configured_min is not None:
        configured_min = max(1, min(MAX_LANES_PER_DIRECTION, int(configured_min)))
    else:
        configured_min = 1
    if bool(road_generation.get("adjacent_lane_required")):
        return max(2, int(configured_min))
    if collision_type in ADJACENT_LANE_COLLISION_TYPES and not is_opposing_lane_case:
        return max(2, int(configured_min))
    return int(configured_min)


def pose_at(actor: dict[str, Any], t: float) -> dict[str, Any] | None:
    trace = [_as_dict(p) for p in _as_list(actor.get("trace"))]
    if not trace:
        pose = actor.get("pose")
        return _as_dict(pose) if isinstance(pose, dict) else None
    return min(trace, key=lambda point: abs(float(point.get("t", 0.0)) - t))


def _actor_footprint_dimensions(actor: dict[str, Any]) -> tuple[float, float]:
    dimensions = _as_dict(actor.get("dimensions"))
    length = _float_or_none(dimensions.get("length_m"))
    if length is None:
        length = _float_or_none(dimensions.get("length"))
    width = _float_or_none(dimensions.get("width_m"))
    if width is None:
        width = _float_or_none(dimensions.get("width"))
    if length is not None and width is not None and length > 0.0 and width > 0.0:
        return length, width

    actor_type = str(actor.get("type") or "").lower()
    subtype = str(actor.get("subtype") or "").lower()
    if actor_type == "vehicle" and any(marker in subtype for marker in ("bus", "coach")):
        return DEFAULT_FOOTPRINT_DIMENSIONS_M["bus"]
    if actor_type == "vehicle" and any(marker in subtype for marker in ("truck", "freight", "food_truck")):
        return DEFAULT_FOOTPRINT_DIMENSIONS_M["truck"]
    return DEFAULT_FOOTPRINT_DIMENSIONS_M.get(actor_type, DEFAULT_FOOTPRINT_DIMENSIONS_M["vehicle"])


def _footprint_corners(pose: dict[str, Any], length: float, width: float) -> list[tuple[float, float]]:
    x = float(pose.get("x", 0.0))
    y = float(pose.get("y", 0.0))
    heading = math.radians(float(pose.get("h", 0.0)))
    forward = (math.cos(heading), math.sin(heading))
    left = (-math.sin(heading), math.cos(heading))
    half_length = length / 2.0
    half_width = width / 2.0
    return [
        (
            x + forward[0] * long_sign * half_length + left[0] * lat_sign * half_width,
            y + forward[1] * long_sign * half_length + left[1] * lat_sign * half_width,
        )
        for long_sign, lat_sign in ((1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0))
    ]


def _project_polygon(points: list[tuple[float, float]], axis: tuple[float, float]) -> tuple[float, float]:
    values = [x * axis[0] + y * axis[1] for x, y in points]
    return min(values), max(values)


def _polygons_intersect(a: list[tuple[float, float]], b: list[tuple[float, float]]) -> bool:
    for points in (a, b):
        for idx, p1 in enumerate(points):
            p2 = points[(idx + 1) % len(points)]
            edge = (p2[0] - p1[0], p2[1] - p1[1])
            axis_length = math.hypot(edge[0], edge[1])
            if axis_length == 0.0:
                continue
            axis = (-edge[1] / axis_length, edge[0] / axis_length)
            a_min, a_max = _project_polygon(a, axis)
            b_min, b_max = _project_polygon(b, axis)
            if a_max < b_min or b_max < a_min:
                return False
    return True


def _point_segment_distance(
    point: tuple[float, float],
    seg_start: tuple[float, float],
    seg_end: tuple[float, float],
) -> float:
    vx = seg_end[0] - seg_start[0]
    vy = seg_end[1] - seg_start[1]
    wx = point[0] - seg_start[0]
    wy = point[1] - seg_start[1]
    length_sq = vx * vx + vy * vy
    if length_sq == 0.0:
        return math.hypot(point[0] - seg_start[0], point[1] - seg_start[1])
    projection = max(0.0, min(1.0, (wx * vx + wy * vy) / length_sq))
    nearest = (seg_start[0] + projection * vx, seg_start[1] + projection * vy)
    return math.hypot(point[0] - nearest[0], point[1] - nearest[1])


def _polygon_distance(a: list[tuple[float, float]], b: list[tuple[float, float]]) -> float:
    if _polygons_intersect(a, b):
        return 0.0
    distances: list[float] = []
    for point in a:
        for idx, seg_start in enumerate(b):
            distances.append(_point_segment_distance(point, seg_start, b[(idx + 1) % len(b)]))
    for point in b:
        for idx, seg_start in enumerate(a):
            distances.append(_point_segment_distance(point, seg_start, a[(idx + 1) % len(a)]))
    return min(distances) if distances else 0.0


def _actor_footprint_distance(
    actor_a: dict[str, Any],
    pose_a: dict[str, Any],
    actor_b: dict[str, Any],
    pose_b: dict[str, Any],
) -> float:
    length_a, width_a = _actor_footprint_dimensions(actor_a)
    length_b, width_b = _actor_footprint_dimensions(actor_b)
    return _polygon_distance(
        _footprint_corners(pose_a, length_a, width_a),
        _footprint_corners(pose_b, length_b, width_b),
    )


def validate_trace_geometry(trace_scene: dict[str, Any]) -> dict[str, Any]:
    actors = _as_dict(trace_scene.get("actors"))
    collision = _as_dict(trace_scene.get("collision"))
    collision_time = float(trace_scene.get("collision_time_s", 0.0))
    collision_point = _as_dict(trace_scene.get("collision_point"))

    initiator_id = str(collision.get("initiator") or "")
    target_id = str(collision.get("target") or "")
    initiator_pose = pose_at(_as_dict(actors.get(initiator_id)), collision_time)
    target_pose = pose_at(_as_dict(actors.get(target_id)), collision_time)

    center_distance = None
    contact_distance = None
    collision_point_max_error = None
    max_actor_error = None
    collision_contact_threshold = float(_as_dict(trace_scene.get("qa_config")).get("collision_contact_threshold_m", 1.0))
    collision_point_tolerance = float(_as_dict(trace_scene.get("qa_config")).get("collision_point_tolerance_m", 1.0))
    if initiator_pose and target_pose:
        center_distance = math.hypot(
            float(initiator_pose.get("x", 0.0)) - float(target_pose.get("x", 0.0)),
            float(initiator_pose.get("y", 0.0)) - float(target_pose.get("y", 0.0)),
        )
        contact_distance = _actor_footprint_distance(
            _as_dict(actors.get(initiator_id)),
            initiator_pose,
            _as_dict(actors.get(target_id)),
            target_pose,
        )
        mid_x = (float(initiator_pose.get("x", 0.0)) + float(target_pose.get("x", 0.0))) / 2.0
        mid_y = (float(initiator_pose.get("y", 0.0)) + float(target_pose.get("y", 0.0))) / 2.0
        collision_point_max_error = math.hypot(
            mid_x - float(collision_point.get("x", 0.0)),
            mid_y - float(collision_point.get("y", 0.0)),
        )
        max_actor_error = max(
            math.hypot(
                float(pose.get("x", 0.0)) - float(collision_point.get("x", 0.0)),
                float(pose.get("y", 0.0)) - float(collision_point.get("y", 0.0)),
            )
            for pose in (initiator_pose, target_pose)
        )

    speeds_ok = True
    max_speed = 0.0
    non_monotonic: list[str] = []
    missing_trace: list[str] = []
    for actor_id, actor_value in actors.items():
        actor = _as_dict(actor_value)
        if actor.get("controller") == "static":
            continue
        trace = [_as_dict(p) for p in _as_list(actor.get("trace"))]
        if len(trace) < 2:
            missing_trace.append(str(actor_id))
            speeds_ok = False
            continue
        for a, b in zip(trace, trace[1:]):
            dt = float(b.get("t", 0.0)) - float(a.get("t", 0.0))
            if dt <= 0:
                non_monotonic.append(str(actor_id))
                speeds_ok = False
                continue
            dist = math.hypot(
                float(b.get("x", 0.0)) - float(a.get("x", 0.0)),
                float(b.get("y", 0.0)) - float(a.get("y", 0.0)),
            )
            speed = dist / dt
            max_speed = max(max_speed, speed)
            if speed > 23.0:
                speeds_ok = False

    return {
        "collision_reproduced": contact_distance is not None and contact_distance <= collision_contact_threshold,
        "min_distance_m": round(contact_distance, 3) if contact_distance is not None else None,
        "collision_center_distance_m": round(center_distance, 3) if center_distance is not None else None,
        "collision_contact_threshold_m": collision_contact_threshold,
        "collision_point_matched": collision_point_max_error is not None and collision_point_max_error <= collision_point_tolerance,
        "collision_point_max_error_m": round(collision_point_max_error, 3) if collision_point_max_error is not None else None,
        "collision_actor_max_error_m": round(max_actor_error, 3) if max_actor_error is not None else None,
        "collision_point_tolerance_m": collision_point_tolerance,
        "speed_reasonable": speeds_ok,
        "max_speed_mps": round(max_speed, 3),
        "non_monotonic_trace_actors": sorted(set(non_monotonic)),
        "missing_trace_actors": sorted(set(missing_trace)),
    }


def _trace_extent(trace_scene: dict[str, Any]) -> float:
    max_abs = 60.0
    for actor_value in _as_dict(trace_scene.get("actors")).values():
        actor = _as_dict(actor_value)
        points = _as_list(actor.get("trace")) or [_as_dict(actor.get("pose"))]
        for point_value in points:
            point = _as_dict(point_value)
            max_abs = max(max_abs, abs(float(point.get("x", 0.0))), abs(float(point.get("y", 0.0))))
    return max_abs * 2.0 + 60.0


def _heading_error_rad(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0


def _straight_road_start_aligned_to_trace(
    trace_scene: dict[str, Any],
    heading: float,
    length: float,
    lane_width: float,
    lanes_per_direction: int,
) -> tuple[float, float]:
    """Place a straight support road so legal lane centers support traces.

    Stage 3 traces define local actor geometry. For simple corridors, the
    generated road reference line is arbitrary unless we align it to those
    traces; otherwise valid same-direction passing traces can be written on
    OpenDRIVE lanes whose legal direction is opposite the actor heading.
    """
    forward = (math.cos(heading), math.sin(heading))
    left = (-math.sin(heading), math.cos(heading))
    lane_offsets: list[tuple[float, float]] = []
    for idx in range(lanes_per_direction):
        offset = (idx + 0.5) * lane_width
        lane_offsets.append((-offset, heading))
        lane_offsets.append((offset, heading + math.pi))

    reference_laterals: list[float] = []
    for actor_value in _as_dict(trace_scene.get("actors")).values():
        actor = _as_dict(actor_value)
        if actor.get("type") not in {"vehicle", "cyclist", "motorcycle", "pedestrian"}:
            continue
        points = _as_list(actor.get("trace")) or [_as_dict(actor.get("pose"))]
        for point_value in points:
            point = _as_dict(point_value)
            if not point:
                continue
            px = float(point.get("x", 0.0))
            py = float(point.get("y", 0.0))
            point_heading = math.radians(float(point.get("h", 0.0)))
            point_lateral = px * left[0] + py * left[1]
            matching_offsets = [
                offset
                for offset, legal_heading in lane_offsets
                if _heading_error_rad(point_heading, legal_heading) <= math.pi / 2.0
            ]
            if not matching_offsets:
                matching_offsets = [offset for offset, _ in lane_offsets]
            nearest_offset = min(matching_offsets, key=lambda offset: abs(point_lateral - offset))
            reference_laterals.append(point_lateral - nearest_offset)

    reference_lateral = _median(reference_laterals) if reference_laterals else 0.0
    start_along = -length / 2.0
    return (
        forward[0] * start_along + left[0] * reference_lateral,
        forward[1] * start_along + left[1] * reference_lateral,
    )


def _straight_road_heading_from_trace(trace_scene: dict[str, Any]) -> float:
    """Infer a straight-corridor reference heading from vehicle traces."""
    weighted_vectors: list[tuple[float, float, float]] = []
    heading_vectors: list[tuple[float, float]] = []
    for actor_value in _as_dict(trace_scene.get("actors")).values():
        actor = _as_dict(actor_value)
        if actor.get("type") not in {"vehicle", "cyclist", "motorcycle"}:
            continue
        points = [_as_dict(point) for point in _as_list(actor.get("trace"))]
        points = [point for point in points if point]
        for first, second in zip(points, points[1:]):
            dx = float(second.get("x", 0.0)) - float(first.get("x", 0.0))
            dy = float(second.get("y", 0.0)) - float(first.get("y", 0.0))
            distance = math.hypot(dx, dy)
            if distance > 1.0:
                weighted_vectors.append((dx / distance, dy / distance, distance))
        for point in points:
            action = str(point.get("action") or "").lower()
            if any(marker in action for marker in ("turn", "impact", "stationary", "stopped", "remain")):
                continue
            h = math.radians(float(point.get("h", 0.0)))
            heading_vectors.append((math.cos(h), math.sin(h)))

    # Axes are bidirectional for a two-way straight road. Fold vectors into a
    # common half-plane, then choose the dominant local corridor direction.
    sx = 0.0
    sy = 0.0
    for vx, vy, weight in weighted_vectors:
        if vx < 0.0 or (abs(vx) < 1e-9 and vy < 0.0):
            vx = -vx
            vy = -vy
        sx += vx * weight
        sy += vy * weight
    for vx, vy in heading_vectors:
        if vx < 0.0 or (abs(vx) < 1e-9 and vy < 0.0):
            vx = -vx
            vy = -vy
        sx += vx
        sy += vy
    if math.hypot(sx, sy) <= 1e-6:
        return math.pi / 2.0
    return math.atan2(sy, sx)


def _road_lane_width(trace_scene: dict[str, Any]) -> float:
    return float(_as_dict(trace_scene.get("road_generation")).get("lane_width_m", DEFAULT_LANE_WIDTH_M))


def _road_half_width(lane_width: float) -> float:
    return lane_width * DEFAULT_LANES_PER_DIRECTION + 0.5


def _road_lanes_per_direction(trace_scene: dict[str, Any]) -> int:
    rg = _as_dict(trace_scene.get("road_generation"))
    raw = rg.get("lanes_forward", rg.get("lanes_per_direction", DEFAULT_LANES_PER_DIRECTION))
    try:
        lanes = int(raw)
    except Exception:
        lanes = DEFAULT_LANES_PER_DIRECTION
    return max(1, min(MAX_LANES_PER_DIRECTION, lanes))


def _road_lanes_forward(trace_scene: dict[str, Any]) -> int:
    """Lanes in ego/travel direction (compiles to OpenDRIVE right lanes)."""
    return _road_lanes_per_direction(trace_scene)


def _road_lanes_backward(trace_scene: dict[str, Any]) -> int:
    """Oncoming-direction lanes (compiles to OpenDRIVE left lanes). 0 = one-way.

    Falls back to lanes_forward when the field is absent (legacy symmetric seed).
    """
    rg = _as_dict(trace_scene.get("road_generation"))
    if "lanes_backward" not in rg:
        return _road_lanes_forward(trace_scene)
    try:
        lanes = int(rg.get("lanes_backward"))
    except Exception:
        return _road_lanes_forward(trace_scene)
    return max(0, min(MAX_LANES_PER_DIRECTION, lanes))


def _road_center_mark(trace_scene: dict[str, Any]) -> Any:
    """Centerline road mark: solid (no crossing) vs broken (may borrow oncoming)."""
    center = _as_dict(trace_scene.get("road_generation")).get("center_line", "broken")
    return xodr.std_roadmark_solid() if center == "solid" else xodr.std_roadmark_broken()


def _road_half_width_for(trace_scene: dict[str, Any], lane_width: float) -> float:
    return lane_width * _road_lanes_per_direction(trace_scene) + 0.5


def _safety_shoulder_width(trace_scene: dict[str, Any]) -> float:
    return float(_as_dict(trace_scene.get("road_generation")).get("safety_shoulder_width_m", STANDARD_SAFETY_SHOULDER_WIDTH_M))


def _patch_safety_shoulders(xodr_path: Path, shoulder_width: float) -> None:
    if shoulder_width <= 0.0:
        return

    tree = ET.parse(xodr_path)
    root = tree.getroot()
    for section in root.findall(".//laneSection"):
        left = section.find("left")
        if left is not None:
            left_lane_ids = [int(lane.get("id", "0")) for lane in left.findall("./lane") if lane.get("id", "0").lstrip("-").isdigit()]
            shoulder_id = max(left_lane_ids, default=1) + 1
            if left.find(f"./lane[@id='{shoulder_id}']") is None:
                lane = ET.Element("lane", {"id": str(shoulder_id), "type": "shoulder", "level": "false"})
                ET.SubElement(lane, "link")
                ET.SubElement(lane, "width", {"a": f"{shoulder_width:g}", "b": "0.0", "c": "0.0", "d": "0.0", "sOffset": "0"})
                ET.SubElement(lane, "roadMark", {"sOffset": "0", "type": "none", "weight": "standard", "color": "standard", "width": "0.0"})
                left.append(lane)

        right = section.find("right")
        if right is not None:
            right_lane_ids = [int(lane.get("id", "0")) for lane in right.findall("./lane") if lane.get("id", "0").lstrip("-").isdigit()]
            shoulder_id = min(right_lane_ids, default=-1) - 1
            if right.find(f"./lane[@id='{shoulder_id}']") is None:
                lane = ET.Element("lane", {"id": str(shoulder_id), "type": "shoulder", "level": "false"})
                ET.SubElement(lane, "link")
                ET.SubElement(lane, "width", {"a": f"{shoulder_width:g}", "b": "0.0", "c": "0.0", "d": "0.0", "sOffset": "0"})
                ET.SubElement(lane, "roadMark", {"sOffset": "0", "type": "none", "weight": "standard", "color": "standard", "width": "0.0"})
                right.append(lane)

    tree.write(xodr_path, encoding="utf-8", xml_declaration=True)


def _patch_roadmark_visibility(xodr_path: Path, center_line: str = "broken") -> None:
    tree = ET.parse(xodr_path)
    root = tree.getroot()
    mark_name = "solid" if center_line == "solid" else "broken"
    for section in root.findall(".//laneSection"):
        center_mark = section.find("./center/lane[@id='0']/roadMark")
        if center_mark is not None:
            center_mark.set("type", mark_name)
            center_mark.set("color", "yellow")
            center_mark.set("width", "0.25")
            mark_type = center_mark.find("type")
            if mark_type is not None:
                mark_type.set("name", mark_name)
                mark_type.set("width", "0.25")
                line = mark_type.find("line")
                if line is not None:
                    line.set("width", "0.2")
                    if mark_name == "solid":
                        line.set("length", "0")
                        line.set("space", "0")
                    else:
                        line.set("length", "4")
                        line.set("space", "8")

        for xpath in ("./left/lane[@id='1']/roadMark", "./right/lane[@id='-1']/roadMark"):
            mark = section.find(xpath)
            if mark is not None:
                mark.set("type", "solid")
                mark.set("color", "white")
                mark.set("width", "0.2")

    tree.write(xodr_path, encoding="utf-8", xml_declaration=True)


def _add_straight_road(
    odr: xodr.OpenDrive,
    road_id: int,
    x_start: float,
    y_start: float,
    heading: float,
    length: float,
    lane_width: float,
    lanes_per_direction: int,
    *,
    lanes_backward: int | None = None,
    center_mark_type: Any = None,
    parking: dict | None = None,
) -> None:
    # lanes_per_direction = forward (travel-direction / right) lanes.
    # lanes_backward = oncoming (left) lanes; None -> symmetric (legacy callers).
    # center_mark_type controls the centerline (solid = no crossing).
    lanes_forward = lanes_per_direction
    if lanes_backward is None:
        lanes_backward = lanes_per_direction
    if center_mark_type is None:
        center_mark_type = xodr.RoadMarkType.solid

    plan = xodr.PlanView(x_start=x_start, y_start=y_start, h_start=heading)
    plan.add_geometry(xodr.Line(length))
    plan.adjust_geometries()

    center = xodr.Lane(a=0.0)
    center.add_roadmark(xodr.RoadMark(center_mark_type, 0.2))
    section = xodr.LaneSection(0, center)
    from tools.road_parking import PARKING_WIDTH_M
    parking = parking or {}

    def add_parking():
        lane = xodr.Lane(lane_type=xodr.LaneType.parking, a=PARKING_WIDTH_M)
        lane.add_roadmark(xodr.RoadMark(xodr.RoadMarkType.broken, 0.15))
        section.add_right_lane(lane)

    # Same-direction left parking is the innermost negative lane on a
    # one-way road. Traffic lanes remain type=driving and keep their count.
    if parking.get('left'):
        if lanes_backward:
            raise ValueError('left parking cannot replace an opposing traffic lane')
        add_parking()

    for _ in range(lanes_backward):
        left = xodr.Lane(a=lane_width)
        left.add_roadmark(xodr.RoadMark(xodr.RoadMarkType.broken, 0.2))
        section.add_left_lane(left)
    for _ in range(lanes_forward):
        right = xodr.Lane(a=lane_width)
        right.add_roadmark(xodr.RoadMark(xodr.RoadMarkType.broken, 0.2))
        section.add_right_lane(right)
    if parking.get('right'):
        add_parking()

    lanes = xodr.Lanes()
    lanes.add_lanesection(section)
    road = xodr.Road(road_id, plan, lanes)
    odr.add_road(road)


def _write_curved_corridor(trace_scene: dict[str, Any], xodr_path: Path) -> None:
    """Write a curved corridor as straight-stub -> arc -> straight-stub.

    A lone arc road with no predecessor/successor imports unreliably in
    CARLA. Bracketing the arc with short straight stubs gives the arc real
    road links (and lane links via adjust_roads_and_lanes), while the open
    ends fall on straight geometry, which CARLA handles robustly.
    """
    road_generation = _as_dict(trace_scene.get("road_generation"))
    lane_width = _road_lane_width(trace_scene)
    lanes_per_direction = _road_lanes_per_direction(trace_scene)
    lanes_backward = _road_lanes_backward(trace_scene)
    center_mark = _road_center_mark(trace_scene)
    shoulder_width = _safety_shoulder_width(trace_scene)
    angle = float(road_generation.get("curve_angle_rad", -math.pi / 2.0))
    # curve_radius_m: if the road_seed sets it explicitly, honour it (replay
    # mode for actual reconstructed traces). Otherwise derive from road_length
    # so the arc-length matches the per-type runway budget set in
    # ROAD_TYPE_DEFAULTS (lowSpeed 140 m → ~89 m radius, motorway 500 m →
    # ~318 m radius). Without this, every curve scene was pinned at radius
    # 45 m (arc length 70.7 m), too short for PCLA on motorway types — 165
    # (curve + stopped_ahead, motorway) was running out of runway in <2 s
    # after the 2026-06-27 length bump.
    explicit_radius = road_generation.get("curve_radius_m")
    if explicit_radius is not None:
        radius = float(explicit_radius)
    else:
        desired_arc = float(road_generation.get("road_length", 0.0) or 0.0)
        radius = (desired_arc / abs(angle)) if (desired_arc > 0 and abs(angle) > 1e-3) else 45.0
    radius = max(radius, lane_width * lanes_per_direction + shoulder_width + 10.0)
    curvature = math.copysign(1.0 / radius, angle)
    length = abs(angle) * radius
    stub_length = max(15.0, lane_width * lanes_per_direction)

    def _corridor_road(road_id: int, geometry: Any) -> Any:
        return xodr.create_road(
            geometry,
            road_id,
            left_lanes=lanes_backward,
            right_lanes=lanes_per_direction,
            center_road_mark=center_mark,
            lane_width=lane_width,
        )

    entry = _corridor_road(1, xodr.Line(stub_length))
    arc = _corridor_road(2, xodr.Arc(curvature, length=length))
    exit_road = _corridor_road(3, xodr.Line(stub_length))

    entry.add_successor(xodr.ElementType.road, 2, xodr.ContactPoint.start)
    arc.add_predecessor(xodr.ElementType.road, 1, xodr.ContactPoint.end)
    arc.add_successor(xodr.ElementType.road, 3, xodr.ContactPoint.start)
    exit_road.add_predecessor(xodr.ElementType.road, 2, xodr.ContactPoint.end)

    odr = xodr.OpenDrive(xodr_path.stem)
    for road in (entry, arc, exit_road):
        odr.add_road(road)
    odr.adjust_roads_and_lanes()
    odr.write_xml(str(xodr_path))
    _patch_safety_shoulders(xodr_path, shoulder_width)
    _patch_roadmark_visibility(xodr_path, _as_dict(trace_scene.get("road_generation")).get("center_line", "broken"))
    extent = 2.0 * radius + 2.0 * stub_length + _road_half_width_for(trace_scene, lane_width) + shoulder_width
    _patch_xodr_header(xodr_path, extent, _road_half_width_for(trace_scene, lane_width), shoulder_width)


def write_generated_opendrive(trace_scene: dict[str, Any], xodr_path: Path) -> None:
    _dispatch_generated_opendrive(trace_scene, xodr_path)
    _finalize_carla_topology(xodr_path)


def _finalize_carla_topology(xodr_path: Path) -> None:
    """Bring the seed map into CARLA's import-ready form at generation time.

    Reuses the exact repair CARLA runs on load (opendrive_repair), so the
    on-disk seed already carries the lane-level links CARLA's topology import
    needs and the load-time repair becomes a no-op.
    """
    data = xodr_path.read_text(encoding="utf-8")
    repaired, _ = opendrive_repair.repair_opendrive_xml(data)
    xodr_path.write_text("<?xml version='1.0' encoding='utf-8'?>\n" + repaired, encoding="utf-8")


def _dispatch_generated_opendrive(trace_scene: dict[str, Any], xodr_path: Path) -> None:
    xodr_path.parent.mkdir(parents=True, exist_ok=True)
    layout = _as_dict(trace_scene.get("road_generation")).get("layout")
    if layout == "curved_corridor":
        _write_curved_corridor(trace_scene, xodr_path)
        return
    if layout in {"orthogonal_turn_left", "orthogonal_turn_right"}:
        _write_orthogonal_turn_junction(trace_scene, xodr_path, str(layout))
        return
    if layout in {"ramp_merge", "ramp_fork"}:
        _write_ramp_junction(trace_scene, xodr_path, str(layout))
        return
    if layout in {"t_junction", "y_junction"}:
        _write_three_leg_junction(trace_scene, xodr_path, str(layout))
        return
    if layout in INTERSECTION_LAYOUTS:
        _write_common_cross_junction(trace_scene, xodr_path)
        return

    odr = xodr.OpenDrive(xodr_path.stem)
    lane_width = _road_lane_width(trace_scene)
    lanes_per_direction = _road_lanes_per_direction(trace_scene)
    lanes_backward = _road_lanes_backward(trace_scene)
    center_line = _as_dict(trace_scene.get("road_generation")).get("center_line", "broken")
    center_mark_type = xodr.RoadMarkType.solid if center_line == "solid" else xodr.RoadMarkType.broken
    shoulder_width = _safety_shoulder_width(trace_scene)
    # The fallback (straight) layout used to size itself purely from actor
    # traces via `_trace_extent`, which floors at 60·2+60 = 180 m for the
    # actor-less road_seed flow. After the 2026-06-27 road_length bump in
    # ROAD_TYPE_DEFAULTS we want straight roads to honor the per-type length
    # (200 m for town, 280 m for townArterial, 500 m for motorway, ...). Use
    # whichever of road_generation.road_length / _trace_extent is larger so
    # actor-bearing replay traces still get a margin around the actor extent.
    rg_dict = _as_dict(trace_scene.get("road_generation"))
    desired_length = float(rg_dict.get("road_length", 0.0) or 0.0)
    length = max(_trace_extent(trace_scene), desired_length)
    half = length / 2.0
    road_specs = _as_list(_as_dict(trace_scene.get("road_generation")).get("roads"))
    if road_specs:
        for idx, spec_value in enumerate(road_specs, start=1):
            spec = _as_dict(spec_value)
            _add_straight_road(
                odr,
                int(spec.get("id", idx)),
                float(spec.get("x_start", -half)),
                float(spec.get("y_start", 0.0)),
                float(spec.get("heading_rad", 0.0)),
                float(spec.get("length", length)),
                lane_width,
                lanes_per_direction,
                lanes_backward=lanes_backward,
                center_mark_type=center_mark_type,
                parking=rg_dict.get('parking'),
            )
    else:
        # Do not create fake crossing roads without OpenDRIVE junction/lane links.
        # A single support road is more stable in CARLA than overlapping roads
        # that visually cross but are topologically disconnected.
        heading = _straight_road_heading_from_trace(trace_scene)
        x_start, y_start = _straight_road_start_aligned_to_trace(
            trace_scene,
            heading,
            length,
            lane_width,
            lanes_per_direction,
        )
        _add_straight_road(
            odr, 1, x_start, y_start, heading, length, lane_width, lanes_per_direction,
            lanes_backward=lanes_backward, center_mark_type=center_mark_type,
            parking=rg_dict.get('parking'),
        )
    odr.write_xml(str(xodr_path))
    _patch_safety_shoulders(xodr_path, shoulder_width)
    _patch_roadmark_visibility(xodr_path, _as_dict(trace_scene.get("road_generation")).get("center_line", "broken"))
    _patch_xodr_header(xodr_path, length, _road_half_width_for(trace_scene, lane_width), shoulder_width)


def _patch_xodr_header(xodr_path: Path, length: float, road_half_width: float, shoulder_width: float = 0.0) -> None:
    """Fill OpenDRIVE header metadata that CARLA warns about when empty."""
    tree = ET.parse(xodr_path)
    root = tree.getroot()
    header = root.find("header")
    if header is None:
        header = ET.Element("header")
        root.insert(0, header)
    # Pin the date to a constant: scenariogeneration writes a wall-clock
    # timestamp here, which makes otherwise-identical geometry non-reproducible.
    header.set("date", "2025-01-01T00:00:00")
    half = length / 2.0 + road_half_width + shoulder_width
    header.set("north", f"{half:.3f}")
    header.set("south", f"{-half:.3f}")
    header.set("east", f"{half:.3f}")
    header.set("west", f"{-half:.3f}")
    if header.find("geoReference") is None:
        geo = ET.SubElement(header, "geoReference")
        geo.text = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs"
    tree.write(xodr_path, encoding="utf-8", xml_declaration=True)


def _road_junction_contact_point(road: ET.Element) -> tuple[float, float] | None:
    """Return the road endpoint that connects to a generated junction."""
    geom = road.find("./planView/geometry")
    if geom is None:
        return None

    x = float(geom.get("x", "0"))
    y = float(geom.get("y", "0"))
    heading = float(geom.get("hdg", "0"))
    length = float(geom.get("length", road.get("length", "0")))

    link = road.find("./link")
    successor = link.find("./successor") if link is not None else None
    predecessor = link.find("./predecessor") if link is not None else None
    if successor is not None and successor.get("elementType") == "junction":
        return (x + math.cos(heading) * length, y + math.sin(heading) * length)
    if predecessor is not None and predecessor.get("elementType") == "junction":
        return (x, y)
    return None


def _incoming_junction_center(root: ET.Element) -> tuple[float, float] | None:
    contacts = []
    for road in root.findall("road"):
        if road.get("junction") not in {None, "-1"}:
            continue
        contact = _road_junction_contact_point(road)
        if contact is not None:
            contacts.append(contact)
    if not contacts:
        return None
    return (
        sum(x for x, _ in contacts) / len(contacts),
        sum(y for _, y in contacts) / len(contacts),
    )


def _align_common_junction_to_trace(trace_scene: dict[str, Any], xodr_path: Path) -> None:
    actors = _as_dict(trace_scene.get("actors"))
    actor = _as_dict(actors.get("hero")) or _as_dict(next(iter(actors.values()), {}))
    trace = [_as_dict(point) for point in _as_list(actor.get("trace"))]
    if not trace:
        return

    trace_heading = math.radians(float(trace[0].get("h", 0.0)))
    approach_points = [
        point
        for point in trace
        if abs((math.radians(float(point.get("h", 0.0))) - trace_heading + math.pi) % (2.0 * math.pi) - math.pi)
        <= math.radians(15.0)
    ]
    alignment_trace = approach_points if len(approach_points) >= 2 else trace
    trace_points = [(float(point.get("x", 0.0)), float(point.get("y", 0.0))) for point in alignment_trace]
    trace_dir = (math.cos(trace_heading), math.sin(trace_heading))
    trace_left = (-trace_dir[1], trace_dir[0])
    trace_along = [x * trace_dir[0] + y * trace_dir[1] for x, y in trace_points]
    trace_across = [x * trace_left[0] + y * trace_left[1] for x, y in trace_points]
    trace_along_mid = (min(trace_along) + max(trace_along)) / 2.0
    trace_across_mid = sum(trace_across) / len(trace_across)
    trace_along_anchor = trace_along_mid
    use_stopline_anchor = (
        max(trace_along) - min(trace_along) <= 5.0
        and any(str(point.get("action", "")).startswith(("stop", "yield", "impact")) for point in trace)
    )
    if use_stopline_anchor:
        trace_along_anchor = sum(trace_along) / len(trace_along)

    tree = ET.parse(xodr_path)
    root = tree.getroot()
    road_generation = _as_dict(trace_scene.get("road_generation"))
    layout = str(road_generation.get("layout") or "")
    lanes_per_direction = _road_lanes_per_direction(trace_scene)
    alignment_text = " ".join(
        [
            str(actor.get("description") or ""),
            " ".join(str(point.get("action") or "") for point in trace),
        ]
    ).lower()
    preferred_abs_lane: int | None = None
    if lanes_per_direction > 1:
        if "rightmost" in alignment_text or "outer" in alignment_text:
            preferred_abs_lane = lanes_per_direction
        elif "leftmost" in alignment_text or "inner" in alignment_text:
            preferred_abs_lane = 1
    junction_center_hint = _as_dict(road_generation.get("junction_center"))
    collision_point = _as_dict(trace_scene.get("collision_point"))
    junction_anchor_point = junction_center_hint or collision_point
    common_layouts = {"common_cross_junction", "cross_junction_with_bike_lane", "t_junction", "y_junction"}
    junction_anchor_along: float | None = None
    if layout in common_layouts and junction_anchor_point:
        junction_center = _incoming_junction_center(root)
        if junction_center is not None:
            trace_along_anchor = (
                float(junction_anchor_point.get("x", 0.0)) * trace_dir[0]
                + float(junction_anchor_point.get("y", 0.0)) * trace_dir[1]
            )
            junction_anchor_along = junction_center[0] * trace_dir[0] + junction_center[1] * trace_dir[1]

    best: dict[str, float] | None = None
    for road in root.findall("road"):
        if road.get("junction") not in {None, "-1"}:
            continue
        geom = road.find("./planView/geometry")
        lane_section = road.find("./lanes/laneSection")
        if geom is None or lane_section is None or geom.find("line") is None:
            continue
        x = float(geom.get("x", "0"))
        y = float(geom.get("y", "0"))
        heading = float(geom.get("hdg", "0"))
        length = float(geom.get("length", road.get("length", "0")))
        left_vec = (-math.sin(heading), math.cos(heading))
        forward = (math.cos(heading), math.sin(heading))
        for side_name, sign in (("right", -1.0), ("left", 1.0)):
            side = lane_section.find(side_name)
            if side is None:
                continue
            cumulative = 0.0
            lanes = sorted(side.findall("./lane"), key=lambda lane: abs(int(lane.get("id", "0"))))
            for lane in lanes:
                if lane.get("type") != "driving":
                    continue
                lane_id = int(lane.get("id", "0"))
                width_node = lane.find("./width")
                width = float(width_node.get("a", "0")) if width_node is not None else 0.0
                if width <= 0.0:
                    continue
                offset = sign * (cumulative + width / 2.0)
                sx = x + left_vec[0] * offset
                sy = y + left_vec[1] * offset
                ex = sx + forward[0] * length
                ey = sy + forward[1] * length
                legal_heading = heading if lane_id < 0 else heading + math.pi
                heading_error = abs((legal_heading - trace_heading + math.pi) % (2.0 * math.pi) - math.pi)
                lane_along_mid = ((sx * trace_dir[0] + sy * trace_dir[1]) + (ex * trace_dir[0] + ey * trace_dir[1])) / 2.0
                lane_across_mid = ((sx * trace_left[0] + sy * trace_left[1]) + (ex * trace_left[0] + ey * trace_left[1])) / 2.0
                if use_stopline_anchor:
                    anchor_x, anchor_y = (ex, ey) if lane_id < 0 else (sx, sy)
                    lane_along_anchor = anchor_x * trace_dir[0] + anchor_y * trace_dir[1]
                elif junction_anchor_along is not None:
                    lane_along_anchor = junction_anchor_along
                else:
                    lane_along_anchor = lane_along_mid
                lane_preference_penalty = (
                    10.0
                    if preferred_abs_lane is not None
                    and heading_error <= math.radians(45.0)
                    and abs(lane_id) != preferred_abs_lane
                    else 0.0
                )
                score = heading_error + lane_preference_penalty + abs(trace_across_mid - lane_across_mid) / 1000.0
                if best is None or score < best["score"]:
                    best = {
                        "score": score,
                        "dx": (trace_along_anchor - lane_along_anchor) * trace_dir[0]
                        + (trace_across_mid - lane_across_mid) * trace_left[0],
                        "dy": (trace_along_anchor - lane_along_anchor) * trace_dir[1]
                        + (trace_across_mid - lane_across_mid) * trace_left[1],
                    }
                cumulative += width

    if best is None:
        return
    for geom in root.findall(".//planView/geometry"):
        geom.set("x", f"{float(geom.get('x', '0')) + best['dx']:.12g}")
        geom.set("y", f"{float(geom.get('y', '0')) + best['dy']:.12g}")
    tree.write(xodr_path, encoding="utf-8", xml_declaration=True)


def _trace_bounds(trace_scene: dict[str, Any]) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for actor_value in _as_dict(trace_scene.get("actors")).values():
        actor = _as_dict(actor_value)
        points = _as_list(actor.get("trace")) or [_as_dict(actor.get("pose"))]
        for point_value in points:
            point = _as_dict(point_value)
            xs.append(float(point.get("x", 0.0)))
            ys.append(float(point.get("y", 0.0)))
    if not xs:
        return -60.0, -60.0, 60.0, 60.0
    return min(xs), min(ys), max(xs), max(ys)


def _dominant_actor_x(trace_scene: dict[str, Any], actor_id: str) -> float:
    actor = _as_dict(_as_dict(trace_scene.get("actors")).get(actor_id))
    points = [_as_dict(point) for point in _as_list(actor.get("trace"))]
    if not points:
        return float(_as_dict(trace_scene.get("collision_point")).get("x", 0.0))
    return round(sum(float(point.get("x", 0.0)) for point in points) / len(points), 3)


def _write_trace_aligned_bike_lane_crossing(trace_scene: dict[str, Any], xodr_path: Path) -> None:
    """Write an intersection support map centered on the authored trace, not on the 001 map frame."""
    lane_width = _road_lane_width(trace_scene)
    lanes_per_direction = _road_lanes_per_direction(trace_scene)
    shoulder_width = _safety_shoulder_width(trace_scene)
    min_x, min_y, max_x, max_y = _trace_bounds(trace_scene)
    pad = 35.0
    main_x = float(
        _as_dict(trace_scene.get("road_generation")).get(
            "main_road_x",
            _dominant_actor_x(trace_scene, "hero"),
        )
    )
    cross_y = float(_as_dict(trace_scene.get("road_generation")).get("cross_road_y", 0.0))
    odr = xodr.OpenDrive(xodr_path.stem)

    # Main vehicle corridor, usually the Zoox straight-through direction.
    y_start = min_y - pad
    main_length = max((max_y - min_y) + 2.0 * pad, 120.0)
    _add_straight_road(odr, 1, main_x, y_start, math.pi / 2.0, main_length, lane_width, lanes_per_direction)

    # Cross-street support so the scene reads as an intersection without
    # forcing the 001 CommonJunctionCreator coordinate frame.
    x_start = min_x - pad
    cross_length = max((max_x - min_x) + 2.0 * pad, 120.0)
    _add_straight_road(odr, 2, x_start, cross_y, 0.0, cross_length, lane_width, lanes_per_direction)

    odr.write_xml(str(xodr_path))
    _patch_safety_shoulders(xodr_path, shoulder_width)
    _patch_roadmark_visibility(xodr_path, _as_dict(trace_scene.get("road_generation")).get("center_line", "broken"))
    extent = max(main_length, cross_length)
    _patch_xodr_header(xodr_path, extent, _road_half_width_for(trace_scene, lane_width), shoulder_width)


def _connect_directional_junction_lanes(creator, first_id, second_id, trace_scene):
    """Keep asymmetric approach lanes connected through the junction.

    CommonJunctionCreator's automatic unequal-lane mode connects only the
    common lane count. Explicit extra connections let excess incoming lanes
    merge into the outer outgoing lane (and reach extra outgoing lanes in
    the reverse asymmetry) without changing the source's lane counts.
    """
    creator.add_connection(first_id, second_id)
    incoming = _road_lanes_forward(trace_scene)
    outgoing = _road_lanes_backward(trace_scene)
    if not incoming or not outgoing or incoming == outgoing:
        return
    common = min(incoming, outgoing)
    pairs = {(index, min(index, outgoing)) for index in range(common + 1, incoming + 1)}
    pairs.update((min(index, incoming), index) for index in range(common + 1, outgoing + 1))
    for source, destination in sorted(pairs):
        creator.add_connection(first_id, second_id, -source, destination)
        creator.add_connection(second_id, first_id, -source, destination)


def _write_common_cross_junction(trace_scene: dict[str, Any], xodr_path: Path) -> None:
    """Write a real OpenDRIVE cross junction with connecting roads/laneLinks."""
    road_length = float(_as_dict(trace_scene.get("road_generation")).get("road_length", STANDARD_CROSS_JUNCTION_ROAD_LENGTH_M))
    radius = float(_as_dict(trace_scene.get("road_generation")).get("junction_radius", STANDARD_CROSS_JUNCTION_RADIUS_M))
    junction_id = int(_as_dict(trace_scene.get("road_generation")).get("junction_id", STANDARD_CROSS_JUNCTION_ID))
    lane_width = _road_lane_width(trace_scene)
    lanes_per_direction = _road_lanes_per_direction(trace_scene)
    shoulder_width = _safety_shoulder_width(trace_scene)

    junction_creator = xodr.CommonJunctionCreator(junction_id, "skill_cross_junction")
    roads = []
    for road_id, angle in enumerate([0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0], start=1):
        road = xodr.create_road(
            xodr.Line(road_length),
            road_id,
            left_lanes=_road_lanes_backward(trace_scene),
            right_lanes=lanes_per_direction,
            center_road_mark=_road_center_mark(trace_scene),
            lane_width=lane_width,
        )
        roads.append(road)
        junction_creator.add_incoming_road_circular_geometry(road, radius, angle, "successor")
        for previous_id in range(1, road_id):
            _connect_directional_junction_lanes(junction_creator, previous_id, road_id, trace_scene)

    odr = xodr.OpenDrive(xodr_path.stem)
    for road in roads:
        odr.add_road(road)
    odr.add_junction_creator(junction_creator)
    odr.adjust_roads_and_lanes()
    odr.write_xml(str(xodr_path))
    _align_common_junction_to_trace(trace_scene, xodr_path)
    _patch_safety_shoulders(xodr_path, shoulder_width)
    _patch_roadmark_visibility(xodr_path, _as_dict(trace_scene.get("road_generation")).get("center_line", "broken"))

    # The generated junction occupies roughly 0..2*(length+radius) in x/y.
    extent = 2.0 * (road_length + radius) + _road_half_width_for(trace_scene, lane_width) + shoulder_width
    _patch_xodr_header(xodr_path, extent, _road_half_width_for(trace_scene, lane_width), shoulder_width)


def _write_three_leg_junction(trace_scene: dict[str, Any], xodr_path: Path, layout: str) -> None:
    """Write a real three-leg OpenDRIVE junction for T/Y road seeds."""
    road_length = float(_as_dict(trace_scene.get("road_generation")).get("road_length", STANDARD_CROSS_JUNCTION_ROAD_LENGTH_M))
    radius = float(_as_dict(trace_scene.get("road_generation")).get("junction_radius", STANDARD_CROSS_JUNCTION_RADIUS_M))
    junction_id = int(_as_dict(trace_scene.get("road_generation")).get("junction_id", STANDARD_CROSS_JUNCTION_ID))
    lane_width = _road_lane_width(trace_scene)
    lanes_per_direction = _road_lanes_per_direction(trace_scene)
    shoulder_width = _safety_shoulder_width(trace_scene)

    if layout == "t_junction":
        angles = [0.0, math.pi / 2.0, math.pi]
        junction_name = "skill_t_junction"
    elif layout == "y_junction":
        angles = [math.pi / 6.0, 5.0 * math.pi / 6.0, 3.0 * math.pi / 2.0]
        junction_name = "skill_y_junction"
    else:
        raise ValueError(f"unsupported three-leg junction layout: {layout}")

    junction_creator = xodr.CommonJunctionCreator(junction_id, junction_name)
    roads = []
    for road_id, angle in enumerate(angles, start=1):
        road = xodr.create_road(
            xodr.Line(road_length),
            road_id,
            left_lanes=_road_lanes_backward(trace_scene),
            right_lanes=lanes_per_direction,
            center_road_mark=_road_center_mark(trace_scene),
            lane_width=lane_width,
        )
        roads.append(road)
        junction_creator.add_incoming_road_circular_geometry(road, radius, angle, "successor")
        for previous_id in range(1, road_id):
            _connect_directional_junction_lanes(junction_creator, previous_id, road_id, trace_scene)

    odr = xodr.OpenDrive(xodr_path.stem)
    for road in roads:
        odr.add_road(road)
    odr.add_junction_creator(junction_creator)
    odr.adjust_roads_and_lanes()
    odr.write_xml(str(xodr_path))
    _align_common_junction_to_trace(trace_scene, xodr_path)
    _patch_safety_shoulders(xodr_path, shoulder_width)
    _patch_roadmark_visibility(xodr_path, _as_dict(trace_scene.get("road_generation")).get("center_line", "broken"))

    extent = 2.0 * (road_length + radius) + _road_half_width_for(trace_scene, lane_width) + shoulder_width
    _patch_xodr_header(xodr_path, extent, _road_half_width_for(trace_scene, lane_width), shoulder_width)


def _normalize_angle_rad(angle: float) -> float:
    return angle % (2.0 * math.pi)


def _nearest_cardinal_angle(angle: float) -> float:
    normalized = _normalize_angle_rad(angle)
    quarter_turn = math.pi / 2.0
    return _normalize_angle_rad(round(normalized / quarter_turn) * quarter_turn)


def _primary_turn_trace(trace_scene: dict[str, Any]) -> list[dict[str, Any]]:
    actors = _as_dict(trace_scene.get("actors"))
    for actor_value in actors.values():
        trace = [_as_dict(point) for point in _as_list(_as_dict(actor_value).get("trace"))]
        if any("turn" in str(point.get("action") or "").lower() for point in trace):
            return trace
        if trace:
            start_heading = float(trace[0].get("h", 0.0))
            if any(_heading_error_deg(float(point.get("h", start_heading)), start_heading) >= 10.0 for point in trace):
                return trace
    hero_trace = _as_list(_as_dict(actors.get("hero")).get("trace"))
    if hero_trace:
        return [_as_dict(point) for point in hero_trace]
    for actor_value in actors.values():
        trace = _as_list(_as_dict(actor_value).get("trace"))
        if trace:
            return [_as_dict(point) for point in trace]
    return []


def _orthogonal_turn_angles(trace_scene: dict[str, Any], layout: str) -> tuple[float, float]:
    trace = _primary_turn_trace(trace_scene)
    start_heading = math.radians(float(trace[0].get("h", 0.0))) if trace else 0.0
    approach_angle = _nearest_cardinal_angle(start_heading)
    turn_delta = -math.pi / 2.0 if layout == "orthogonal_turn_right" else math.pi / 2.0
    departure_heading = _normalize_angle_rad(approach_angle + turn_delta)

    # CommonJunctionCreator places each incoming road by the heading that
    # points into the junction. The outgoing leg therefore uses the opposite
    # heading of the vehicle's departure direction.
    departure_leg_angle = _normalize_angle_rad(departure_heading + math.pi)
    return approach_angle, departure_leg_angle


def _heading_error_deg(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def _orthogonal_turn_support_points(trace_scene: dict[str, Any]) -> list[tuple[float, float, float]]:
    trace = _primary_turn_trace(trace_scene)
    if len(trace) < 2:
        return []

    start_heading = float(trace[0].get("h", 0.0))
    turn_start_idx: int | None = None
    for idx, point in enumerate(trace):
        action = str(point.get("action") or "").lower()
        heading = float(point.get("h", start_heading))
        if "turn" in action or _heading_error_deg(heading, start_heading) >= 10.0:
            turn_start_idx = idx
            break
    if turn_start_idx is None:
        return []

    points: list[tuple[float, float, float]] = []
    for point in trace[turn_start_idx:]:
        x = float(point.get("x", 0.0))
        y = float(point.get("y", 0.0))
        h = float(point.get("h", start_heading))
        if points and math.hypot(x - points[-1][0], y - points[-1][1]) < 0.25:
            points[-1] = (points[-1][0], points[-1][1], h)
            continue
        points.append((x, y, h))

    if len(points) < 2:
        return []

    # Keep stopped/impact points lane-legal by extending the connector in the
    # final vehicle heading instead of leaving them snapped to the inbound leg.
    last_x, last_y, last_h = points[-1]
    final_heading = math.radians(last_h)
    points.append((last_x + math.cos(final_heading) * 30.0, last_y + math.sin(final_heading) * 30.0, last_h))
    return points


def _patch_orthogonal_turn_connector_to_trace(trace_scene: dict[str, Any], xodr_path: Path, junction_id: int) -> None:
    """Shape the two-leg connector around the authored local turn trace.

    CommonJunctionCreator gives us valid junction/laneLink topology, but its
    connector is positioned in the library's default frame.  For generated
    local-map replay scenes the connector must support the Stage 3 turn lane.
    """
    trace = _primary_turn_trace(trace_scene)
    points = _orthogonal_turn_support_points(trace_scene)
    if len(points) < 3:
        return

    lane_width = _road_lane_width(trace_scene)
    # Lane -1 travels with the road reference direction in the QA/OpenDRIVE
    # convention used by this repository.  Offset the reference so lane -1 is
    # centered on the authored turn path.
    lane_offset = -lane_width / 2.0

    tree = ET.parse(xodr_path)
    root = tree.getroot()
    connector = None
    for road in root.findall("road"):
        if road.get("junction") == str(junction_id):
            connector = road
            break
    if connector is None:
        return

    plan_view = connector.find("planView")
    if plan_view is None:
        plan_view = ET.SubElement(connector, "planView")
    for child in list(plan_view):
        plan_view.remove(child)

    cumulative_s = 0.0
    connector_start: tuple[float, float] | None = None
    connector_end: tuple[float, float, float] | None = None
    for start, end in zip(points, points[1:]):
        x1, y1, _ = start
        x2, y2, _ = end
        length = math.hypot(x2 - x1, y2 - y1)
        if length < 0.25:
            continue
        heading = math.atan2(y2 - y1, x2 - x1)
        left_x = -math.sin(heading)
        left_y = math.cos(heading)
        ref_x = x1 - left_x * lane_offset
        ref_y = y1 - left_y * lane_offset
        geom = ET.SubElement(
            plan_view,
            "geometry",
            {
                "s": f"{cumulative_s:.12g}",
                "x": f"{ref_x:.12g}",
                "y": f"{ref_y:.12g}",
                "hdg": f"{heading:.12g}",
                "length": f"{length:.12g}",
            },
        )
        ET.SubElement(geom, "line")
        if connector_start is None:
            connector_start = (ref_x, ref_y)
        cumulative_s += length
        connector_end = (
            ref_x + math.cos(heading) * length,
            ref_y + math.sin(heading) * length,
            heading,
        )

    if cumulative_s <= 0.0:
        return
    connector.set("length", f"{cumulative_s:.12g}")

    if connector_start is not None and connector_end is not None:
        junction = root.find(f"./junction[@id='{junction_id}']")
        connecting_road_id = connector.get("id")
        approach_road_id = None
        departure_road_id = None
        if junction is not None:
            for connection in junction.findall("connection"):
                if connection.get("connectingRoad") != connecting_road_id:
                    continue
                if connection.get("contactPoint") == "start":
                    approach_road_id = connection.get("incomingRoad")
                elif connection.get("contactPoint") == "end":
                    departure_road_id = connection.get("incomingRoad")

        def patch_connected_leg(
            road_id: str | None,
            contact_xy: tuple[float, float],
            heading: float,
        ) -> None:
            road = root.find(f"./road[@id='{road_id}']") if road_id else None
            if road is None:
                return
            plan_view = road.find("planView")
            if plan_view is None:
                plan_view = ET.SubElement(road, "planView")
            for child in list(plan_view):
                plan_view.remove(child)
            length = float(road.get("length", "100.0"))
            end_x, end_y = contact_xy
            start_x = end_x - math.cos(heading) * length
            start_y = end_y - math.sin(heading) * length
            geom = ET.SubElement(
                plan_view,
                "geometry",
                {
                    "s": "0",
                    "x": f"{start_x:.12g}",
                    "y": f"{start_y:.12g}",
                    "hdg": f"{heading:.12g}",
                    "length": f"{length:.12g}",
                },
            )
            ET.SubElement(geom, "line")

        approach_heading = math.radians(float(trace[0].get("h", 0.0)))
        departure_heading = math.radians(points[-1][2])
        patch_connected_leg(approach_road_id, connector_start, approach_heading)
        patch_connected_leg(departure_road_id, (connector_end[0], connector_end[1]), _normalize_angle_rad(departure_heading + math.pi))

    tree.write(xodr_path, encoding="utf-8", xml_declaration=True)


def _write_orthogonal_turn_junction(trace_scene: dict[str, Any], xodr_path: Path, layout: str) -> None:
    """Write a two-leg real junction for L-shaped left/right-turn scenes."""
    road_length = float(_as_dict(trace_scene.get("road_generation")).get("road_length", STANDARD_CROSS_JUNCTION_ROAD_LENGTH_M))
    radius = float(_as_dict(trace_scene.get("road_generation")).get("junction_radius", STANDARD_CROSS_JUNCTION_RADIUS_M))
    junction_id = int(_as_dict(trace_scene.get("road_generation")).get("junction_id", STANDARD_CROSS_JUNCTION_ID))
    lane_width = _road_lane_width(trace_scene)
    lanes_per_direction = _road_lanes_per_direction(trace_scene)
    shoulder_width = _safety_shoulder_width(trace_scene)
    approach_angle, departure_leg_angle = _orthogonal_turn_angles(trace_scene, layout)

    junction_creator = xodr.CommonJunctionCreator(junction_id, "skill_orthogonal_turn_junction")
    roads = []
    for road_id, angle in enumerate([approach_angle, departure_leg_angle], start=1):
        road = xodr.create_road(
            xodr.Line(road_length),
            road_id,
            left_lanes=_road_lanes_backward(trace_scene),
            right_lanes=lanes_per_direction,
            center_road_mark=_road_center_mark(trace_scene),
            lane_width=lane_width,
        )
        roads.append(road)
        junction_creator.add_incoming_road_circular_geometry(road, radius, angle, "successor")

    junction_creator.add_connection(1, 2)

    odr = xodr.OpenDrive(xodr_path.stem)
    for road in roads:
        odr.add_road(road)
    odr.add_junction_creator(junction_creator)
    odr.adjust_roads_and_lanes()
    odr.write_xml(str(xodr_path))
    _align_common_junction_to_trace(trace_scene, xodr_path)
    _patch_orthogonal_turn_connector_to_trace(trace_scene, xodr_path, junction_id)
    _patch_safety_shoulders(xodr_path, shoulder_width)
    _patch_roadmark_visibility(xodr_path, _as_dict(trace_scene.get("road_generation")).get("center_line", "broken"))

    extent = 2.0 * (road_length + radius) + _road_half_width_for(trace_scene, lane_width) + shoulder_width
    _patch_xodr_header(xodr_path, extent, _road_half_width_for(trace_scene, lane_width), shoulder_width)


def _write_ramp_junction(trace_scene: dict[str, Any], xodr_path: Path, layout: str) -> None:
    """Write a ramp-style on-ramp merge / off-ramp fork via a direct junction.

    ramp_fork (off-ramp): mainline_in -> mainline_out (through) plus the
    rightmost mainline lane diverging onto a one-way ramp.
    ramp_merge (on-ramp): a one-way ramp converging into the rightmost lane of
    the downstream mainline, alongside the through mainline_in -> mainline_out.

    A DirectJunctionCreator wires the lane links; adjust_roads_and_lanes places
    geometry from the road links, so no actor-trace alignment is needed for a
    road-only seed.
    """
    rg = _as_dict(trace_scene.get("road_generation"))
    road_length = float(rg.get("road_length", STANDARD_CROSS_JUNCTION_ROAD_LENGTH_M))
    junction_id = int(rg.get("junction_id", STANDARD_CROSS_JUNCTION_ID))
    lane_width = _road_lane_width(trace_scene)
    lanes_per_direction = _road_lanes_per_direction(trace_scene)
    shoulder_width = _safety_shoulder_width(trace_scene)
    ramp_length = max(40.0, road_length * 0.6)

    junction_creator = xodr.DirectJunctionCreator(junction_id, f"skill_{layout}")

    def _mainline(road_id: int) -> Any:
        return xodr.create_road(
            xodr.Line(road_length),
            road_id,
            left_lanes=_road_lanes_backward(trace_scene),
            right_lanes=lanes_per_direction,
            center_road_mark=_road_center_mark(trace_scene),
            lane_width=lane_width,
        )

    mainline_in = _mainline(1)
    mainline_out = _mainline(2)
    mainline_in.add_successor(xodr.ElementType.junction, junction_id)
    mainline_out.add_predecessor(xodr.ElementType.junction, junction_id)

    if layout == "ramp_fork":
        # Off-ramp: gentle right-diverging spiral carrying one lane.
        ramp = xodr.create_road(
            xodr.Spiral(-0.00001, -0.01, ramp_length),
            10,
            left_lanes=0,
            right_lanes=1,
            lane_width=lane_width,
        )
        ramp.add_predecessor(xodr.ElementType.junction, junction_id)
        junction_creator.add_connection(incoming_road=mainline_in, linked_road=mainline_out)
        junction_creator.add_connection(
            incoming_road=mainline_in,
            linked_road=ramp,
            incoming_lane_ids=-lanes_per_direction,
            linked_lane_ids=-1,
        )
    else:  # ramp_merge
        # On-ramp: gentle right-converging spiral carrying one lane.
        ramp = xodr.create_road(
            xodr.Spiral(0.01, 0.00001, ramp_length),
            20,
            left_lanes=0,
            right_lanes=1,
            lane_width=lane_width,
        )
        ramp.add_successor(xodr.ElementType.junction, junction_id)
        junction_creator.add_connection(incoming_road=mainline_in, linked_road=mainline_out)
        junction_creator.add_connection(
            incoming_road=ramp,
            linked_road=mainline_out,
            incoming_lane_ids=-1,
            linked_lane_ids=-lanes_per_direction,
        )

    odr = xodr.OpenDrive(xodr_path.stem)
    for road in (mainline_in, mainline_out, ramp):
        odr.add_road(road)
    odr.add_junction_creator(junction_creator)
    odr.adjust_roads_and_lanes()
    odr.write_xml(str(xodr_path))
    _patch_safety_shoulders(xodr_path, shoulder_width)
    _patch_roadmark_visibility(xodr_path, _as_dict(trace_scene.get("road_generation")).get("center_line", "broken"))

    extent = 2.0 * road_length + ramp_length + _road_half_width_for(trace_scene, lane_width) + shoulder_width
    _patch_xodr_header(xodr_path, extent, _road_half_width_for(trace_scene, lane_width), shoulder_width)


def _sim_time_trigger(name: str, seconds: float, triggeringpoint: str = "start") -> xosc.ValueTrigger:
    return xosc.ValueTrigger(
        name,
        0.0,
        xosc.ConditionEdge.none,
        xosc.SimulationTimeCondition(seconds, xosc.Rule.greaterThan),
        triggeringpoint=triggeringpoint,
    )


def _make_follow_trajectory_action(
    trace: list[dict[str, Any]],
    actor_id: str,
    start_time: float = 0.0,
) -> xosc.FollowTrajectoryAction:
    times = [max(0.0, float(point["t"]) - start_time) for point in trace]
    positions = [
        xosc.WorldPosition(
            x=float(point["x"]),
            y=float(point["y"]),
            z=float(point.get("z", 0.2)),
            h=math.radians(float(point.get("h", 0.0))),
        )
        for point in trace
    ]
    trajectory = xosc.Trajectory(f"{actor_id}_trajectory", closed=False)
    trajectory.add_shape(xosc.Polyline(times, positions))
    return xosc.FollowTrajectoryAction(
        trajectory,
        xosc.FollowingMode.position,
        xosc.ReferenceContext.relative,
        1.0,
        0.0,
    )


def _add_vehicle_controller(init: xosc.Init, actor_id: str, *, generated_lane_route=None, initial_lane_offset=None, max_brake=None) -> None:
    props = xosc.Properties()
    props.add_property("module", "npc_vehicle_control")
    if max_brake is not None:
        if isinstance(max_brake, bool) or not isinstance(max_brake, (int, float)) or not 0 < max_brake <= 1:
            raise ValueError('max_brake must be a control fraction in (0, 1]')
        props.add_property('C2XMaxBrake', str(max_brake))
    if generated_lane_route is not None:
        props.add_property('C2XGeneratedLaneRoute', json.dumps(generated_lane_route))
    if initial_lane_offset is not None:
        props.add_property('C2XInitialLaneOffset', str(initial_lane_offset))
    controller = xosc.Controller(f"{actor_id}_scripted_control", props)
    assign = xosc.AssignControllerAction(controller=controller)
    override = xosc.OverrideControllerValueAction()
    override.set_throttle(False, 0)
    override.set_brake(False, 0)
    override.set_clutch(False, 0)
    override.set_steeringwheel(False, 0)
    override.set_gear(False, 0)
    override.set_parkingbrake(False, 0)
    init.add_init_action(actor_id, xosc.ControllerAction(assignControllerAction=assign, overrideControllerValueAction=override))


def _patch_monitoring_criteria(xosc_path: Path) -> None:
    tree = ET.parse(xosc_path)
    root = tree.getroot()
    stop_trigger = root.find("./Storyboard/StopTrigger")
    if stop_trigger is None:
        storyboard = root.find("./Storyboard")
        if storyboard is None:
            return
        stop_trigger = ET.SubElement(storyboard, "StopTrigger")
    condition_group = stop_trigger.find("./ConditionGroup")
    if condition_group is None:
        condition_group = ET.SubElement(stop_trigger, "ConditionGroup")

    existing = {condition.get("name") for condition in stop_trigger.findall("./ConditionGroup/Condition")}
    criteria = [
        ("criteria_RunningStopTest", "", ""),
        ("criteria_RunningRedLightTest", "", ""),
        ("criteria_WrongLaneTest", "", ""),
        ("criteria_OnSidewalkTest", "", ""),
        ("criteria_KeepLaneTest", "", ""),
        ("criteria_CollisionTest", "", ""),
        ("criteria_DrivenDistanceTest", "distance_success", "100"),
    ]
    for name, parameter_ref, value in criteria:
        if name in existing:
            continue
        criteria_group = ET.SubElement(stop_trigger, "ConditionGroup")
        condition = ET.SubElement(criteria_group, "Condition", {"name": name, "delay": "0", "conditionEdge": "rising"})
        by_value = ET.SubElement(condition, "ByValueCondition")
        ET.SubElement(
            by_value,
            "ParameterCondition",
            {"parameterRef": parameter_ref, "value": value, "rule": "lessThan"},
        )
    tree.write(xosc_path, encoding="utf-8", xml_declaration=True)


def write_replay_xosc(trace_scene: dict[str, Any], xosc_path: Path) -> None:
    xosc_path.parent.mkdir(parents=True, exist_ok=True)
    actors = _as_dict(trace_scene.get("actors"))
    metadata = _as_dict(trace_scene.get("metadata"))
    environment = _as_dict(trace_scene.get("environment"))
    xodr_path = str(_as_dict(trace_scene.get("road_generation")).get("xodr_path", ""))

    entities = xosc.Entities()
    init = xosc.Init()
    init.add_global_action(_build_env_action({"environment": environment}))

    for actor_id, actor_value in actors.items():
        actor = _as_dict(actor_value)
        actor_type = str(actor.get("type") or "vehicle")
        subtype = str(actor.get("subtype") or "car")
        role = "ego_vehicle" if actor_id == "hero" else "simulation"
        trace = [_as_dict(point) for point in _as_list(actor.get("trace"))]
        pose = _as_dict(actor.get("pose"))
        start = trace[0] if trace else pose

        if actor_type == "pedestrian":
            obj = _make_pedestrian(str(actor_id))
            if hasattr(obj, "add_property"):
                obj.add_property("type", "simulation")
        elif actor_type == "cyclist":
            obj = _make_vehicle("vehicle.bh.crossbike", "car", role)
            obj.add_property("semantic_type", "cyclist")
        elif actor_type == "obstacle":
            obj = _make_vehicle("vehicle.tesla.model3", "car", "simulation")
            obj.add_property("object_proxy", "true")
        else:
            blueprint = "vehicle.volkswagen.t2" if subtype == "bus" else "vehicle.tesla.model3"
            obj = _make_vehicle(blueprint, "bus" if subtype == "bus" else "car", role)
        if hasattr(obj, "add_property"):
            obj.add_property("replay_role", str(actor.get("role") or ""))
        entities.add_scenario_object(str(actor_id), obj)

        init.add_init_action(str(actor_id), xosc.TeleportAction(xosc.WorldPosition(
            x=float(start.get("x", 0.0)),
            y=float(start.get("y", 0.0)),
            z=float(start.get("z", 0.2)),
            h=math.radians(float(start.get("h", 0.0))),
        )))
        if actor_type in {"vehicle", "cyclist"}:
            _add_vehicle_controller(init, str(actor_id))

    duration = float(trace_scene.get("duration_s", 12.0))
    story = xosc.Story("accident_replay_story")
    act = xosc.Act(
        "accident_replay_act",
        starttrigger=_sim_time_trigger("act_start", 0.0, "start"),
        stoptrigger=_sim_time_trigger("act_stop", duration, "stop"),
    )

    for actor_id, actor_value in actors.items():
        actor = _as_dict(actor_value)
        trace = [_as_dict(point) for point in _as_list(actor.get("trace"))]
        if len(trace) < 2:
            continue
        action_start_time = float(actor.get("trajectory_start_time_s", 0.0))
        action_trace = [point for point in trace if float(point.get("t", 0.0)) >= action_start_time]
        if len(action_trace) < 2:
            action_trace = trace
            action_start_time = 0.0
        event = xosc.Event(f"{actor_id}_follow_trace", xosc.Priority.overwrite)
        event.add_trigger(_sim_time_trigger(f"{actor_id}_trace_start", action_start_time))
        event.add_action(
            f"{actor_id}_trajectory_action",
            _make_follow_trajectory_action(action_trace, str(actor_id), action_start_time),
        )
        maneuver = xosc.Maneuver(f"{actor_id}_maneuver")
        maneuver.add_event(event)
        group = xosc.ManeuverGroup(f"{actor_id}_maneuver_group")
        group.add_actor(str(actor_id))
        group.add_maneuver(maneuver)
        act.add_maneuver_group(group)

    story.add_act(act)
    storyboard = xosc.StoryBoard(init, stoptrigger=_sim_time_trigger("story_stop", duration + 0.5, "stop"))
    storyboard.add_story(story)

    catalog = xosc.Catalog()
    catalog_dir = Path("openscenarios/catalogs")
    if catalog_dir.exists():
        for cname in ["VehicleCatalog", "ControllerCatalog", "PedestrianCatalog", "MiscObjectCatalog", "EnvironmentCatalog"]:
            catalog.add_catalog(cname, str(catalog_dir))

    scenario = xosc.Scenario(
        name=str(metadata.get("name") or xosc_path.stem),
        author="ads_testing",
        parameters=xosc.ParameterDeclarations(),
        entities=entities,
        storyboard=storyboard,
        roadnetwork=xosc.RoadNetwork(roadfile=xodr_path, scenegraph=""),
        catalog=catalog,
        osc_minor_version=0,
    )
    scenario.write_xml(str(xosc_path))
    _patch_monitoring_criteria(xosc_path)


def validate_xosc(xosc_path: Path, xsd_path: Path) -> tuple[bool, str | None]:
    try:
        xmlschema.XMLSchema(str(xsd_path)).validate(str(xosc_path))
        return True, None
    except Exception as exc:
        return False, str(exc)


def build(trace_path: Path, xodr_path: Path | None, xosc_path: Path | None, xsd_path: Path | None) -> dict[str, Any]:
    trace_scene = read_json(trace_path)
    schema_errors = validate_trace_schema(trace_scene)
    if schema_errors:
        return {"status": "failed", "trace": str(trace_path), "errors": schema_errors}

    qa = validate_trace_geometry(trace_scene)
    trace_scene["qa"] = {**_as_dict(trace_scene.get("qa")), **qa}
    if not qa["collision_reproduced"] or not qa["collision_point_matched"] or not qa["speed_reasonable"]:
        write_json(trace_path, trace_scene)
        return {"status": "failed", "trace": str(trace_path), "qa": qa}

    if xodr_path is None:
        xodr_path = Path(_as_dict(trace_scene.get("road_generation")).get("xodr_path"))
    if xosc_path is None:
        xosc_path = Path("outputs/xosc_skill") / f"{trace_path.stem}.xosc"
    trace_scene["road_generation"]["xodr_path"] = str(xodr_path)

    write_generated_opendrive(trace_scene, xodr_path)
    write_replay_xosc(trace_scene, xosc_path)

    result = {
        "status": "generated",
        "trace": str(trace_path),
        "xodr": str(xodr_path),
        "xosc": str(xosc_path),
        "qa": qa,
    }
    if xsd_path:
        valid, error = validate_xosc(xosc_path, xsd_path)
        result["xsd_validation_passed"] = valid
        if error:
            result["xsd_validation_error"] = error
    trace_scene.setdefault("pipeline", {})["xodr_path"] = str(xodr_path)
    trace_scene["pipeline"]["xosc_path"] = str(xosc_path)
    trace_scene["pipeline"]["xsd_validation_passed"] = result.get("xsd_validation_passed")
    write_json(trace_path, trace_scene)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Build OpenDRIVE and OpenSCENARIO from an agent-authored trace_scene JSON")
    parser.add_argument("--trace", required=True, type=Path)
    parser.add_argument("--xodr", type=Path, default=None)
    parser.add_argument("--xosc", type=Path, default=None)
    parser.add_argument("--xsd", type=Path, default=Path("xsd/OpenSCENARIO.xsd"))
    args = parser.parse_args()
    result = build(args.trace, args.xodr, args.xosc, args.xsd)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result.get("status") == "generated" and result.get("xsd_validation_passed", True) else 1


if __name__ == "__main__":
    sys.exit(main())
