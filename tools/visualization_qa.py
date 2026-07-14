#!/usr/bin/env python3
"""Visual QA gate for generated replay scenes.

This tool complements XSD validation. It inspects the generated trace,
OpenDRIVE map, source facts/semantic scene, and visualization HTML, then emits
a compact JSON report covering road type, actor count, trajectories, road
envelope alignment, and collision type.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

try:
    from tools.chinese_scene_text import valid_chinese_scene_description
except ModuleNotFoundError:  # Allow direct execution as python tools/visualization_qa.py.
    from chinese_scene_text import valid_chinese_scene_description

INTERSECTION_LAYOUTS = {
    "common_cross_junction",
    "cross_junction_with_bike_lane",
    "t_junction",
    "orthogonal_turn_left",
    "orthogonal_turn_right",
    "y_junction",
}
SUPPORTED_QA_LAYOUTS = INTERSECTION_LAYOUTS | {
    "straight_corridor",
    "single_corridor",
    "curbside_parking_corridor",
    "straight_with_centerline_passing",
}
ADJACENT_LANE_MARKERS = (
    "adjacent",
    "next lane",
    "another lane",
    "left lane",
    "right lane",
    "lane change",
    "change lanes",
    "sideswipe",
    "side-by-side",
    "side by side",
    "parallel",
    "overtak",
    "passing",
    "dedicated turn lane",
    "turn lane",
    "相邻",
    "隔壁车道",
    "另一车道",
    "左侧车道",
    "右侧车道",
    "变道",
    "并行",
    "超车",
    "转弯车道",
)
OPPOSING_LANE_MARKERS = (
    "opposing lane",
    "opposing_lane",
    "opposing-direction",
    "opposing direction",
    "opposite_direction",
    "opposite direction",
    "oncoming",
    "double yellow",
    "center line",
    "centerline",
    "对向车道",
    "对向",
    "迎面",
    "双黄线",
)


def _as_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _as_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def read_json(path: Path | None) -> dict[str, Any]:
    if path is None or not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def source_narrative(source: dict[str, Any]) -> str:
    event_description = _as_dict(source.get("event_description"))
    if event_description.get("text"):
        return str(event_description.get("text"))
    incident = _as_dict(source.get("incident"))
    if incident.get("narrative"):
        return str(incident.get("narrative"))
    structured = _as_dict(source.get("structured"))
    incident_description = _as_dict(structured.get("incident_description"))
    return str(incident_description.get("narrative") or "")


def chinese_scene_description_quality(trace: dict[str, Any], source: dict[str, Any], semantic: dict[str, Any]) -> dict[str, Any]:
    source_event = _as_dict(source.get("event_description"))
    candidates = [
        _as_dict(semantic.get("incident")).get("narrative_zh"),
        _as_dict(trace.get("incident")).get("narrative_zh"),
        _as_dict(source.get("incident")).get("narrative_zh"),
        source_event.get("zh_summary"),
    ]
    best = ""
    for value in candidates:
        text = str(value or "").strip()
        if len(text) > len(best):
            best = text
        if valid_chinese_scene_description(text):
            return {"passed": True, "source": "json_sidecar", "sample": text[:160]}
    return {
        "passed": False,
        "reason": "missing_placeholder_or_mostly_english_chinese_summary",
        "sample": best[:160],
    }


def expected_road_family(source: dict[str, Any], semantic: dict[str, Any]) -> str:
    road_context = _as_dict(semantic.get("road_context"))
    road_model = _as_dict(semantic.get("road_model"))
    topology = str(road_context.get("topology") or "").lower()
    semantic_layout = str(road_model.get("layout") or topology or "").strip()
    if semantic_layout in SUPPORTED_QA_LAYOUTS:
        return semantic_layout
    narrative = source_narrative(source).lower()
    if "bike lane" in narrative or "ebike" in narrative or "cyclist" in narrative:
        if "intersection" in narrative or topology == "intersection":
            return "cross_junction_with_bike_lane"
    if "intersection" in narrative or "red light" in narrative or "stop sign" in narrative or topology == "intersection":
        return "common_cross_junction"
    passing_context = (
        "double yellow" in narrative
        or "maneuver around" in narrative
        or "began to pass" in narrative
        or "attempted to pass" in narrative
        or re.search(r"\bpass(?:ed|ing)?\b", narrative) is not None
    )
    if passing_context:
        return "straight_with_centerline_passing"
    if "parked in-lane" in narrative or "parked in lane" in narrative:
        return "straight_with_centerline_passing"
    if "parked" in narrative or "side mirror" in narrative or "curb" in narrative:
        return "curbside_parking_corridor"
    if topology == "straight":
        return "single_corridor"
    return "unknown"


def expected_collision_family(source: dict[str, Any], semantic: dict[str, Any]) -> str:
    collision = _as_dict(semantic.get("collision"))
    if collision.get("type"):
        return str(collision.get("type"))
    narrative = source_narrative(source).lower()
    if "rear" in narrative and ("from behind" in narrative or "behind" in narrative):
        return "rear_end"
    if "side mirror" in narrative or "side" in narrative or "sideswipe" in narrative:
        return "sideswipe"
    if "head-on" in narrative or "head on" in narrative:
        return "head_on"
    if "cyclist" in narrative or "ebike" in narrative:
        return "pedestrian_or_cyclist"
    return "unknown"


def road_type_compatible(expected: str, actual: str) -> bool:
    if expected == "unknown" or expected == actual:
        return True
    if expected == "single_corridor" and actual == "straight_corridor":
        return True
    if expected == "common_cross_junction" and actual in INTERSECTION_LAYOUTS:
        return True
    return False


def _lane_evidence_text(source: dict[str, Any], semantic: dict[str, Any]) -> str:
    parts = [source_narrative(source)]
    road_context = _as_dict(semantic.get("road_context"))
    parts.extend(str(road_context.get(key) or "") for key in ("source_text", "lanes", "topology"))
    road_model = _as_dict(semantic.get("road_model"))
    parts.extend(str(road_model.get(key) or "") for key in ("assumptions", "notes"))
    for actor in _as_list(semantic.get("actors")):
        actor = _as_dict(actor)
        parts.append(str(actor.get("description") or ""))
        movement = _as_dict(actor.get("movement_intent"))
        parts.extend(str(movement.get(key) or "") for key in ("path_relation_to_hero", "lane_index"))
    return " ".join(parts).lower()


def expected_min_lanes_per_direction(source: dict[str, Any], semantic: dict[str, Any]) -> int:
    text = _lane_evidence_text(source, semantic)
    road_model = _as_dict(semantic.get("road_model"))
    road_model_lanes = None
    try:
        road_model_lanes = max(1, min(3, int(road_model.get("lanes_per_direction", 1))))
    except Exception:
        road_model_lanes = None
    opposing_lane_case = any(marker in text for marker in OPPOSING_LANE_MARKERS)
    layout = str(road_model.get("layout") or "")
    curbside_parking_case = (
        layout == "curbside_parking_corridor"
        and any(marker in text for marker in ("parallel park", "parallel-park", "curb", "parking", "parked"))
        and not any(
            marker in text
            for marker in (
                "lane change",
                "change lanes",
                "sideswipe",
                "side-by-side",
                "side by side",
                "next lane",
                "another lane",
                "left lane",
                "right lane",
                "turn lane",
            )
        )
    )
    negated_adjacent_lane_evidence = any(
        marker in text
        for marker in (
            "no adjacent",
            "no mention of adjacent",
            "no adjacent-lane",
            "no adjacent lane",
            "mentions no adjacent",
            "no lane change",
            "same-lane",
            "same lane",
        )
    )
    if re.search(r"\b(three|3)\s+(?:same[- ]direction\s+)?(?:travel\s+)?lanes?\b", text) or "三条" in text:
        return 3
    if curbside_parking_case:
        return road_model_lanes or 1
    if (
        any(marker in text for marker in ADJACENT_LANE_MARKERS)
        and not negated_adjacent_lane_evidence
        and not (opposing_lane_case and road_model_lanes == 1)
    ):
        return 2
    if re.search(r"\blane\s+\d+\b.*\blane\s+\d+\b", text):
        return 2
    return road_model_lanes or 1


def lane_count_sufficient(trace: dict[str, Any], source: dict[str, Any], semantic: dict[str, Any]) -> dict[str, Any]:
    expected_min = expected_min_lanes_per_direction(source, semantic)
    actual = int(_as_dict(trace.get("road_generation")).get("lanes_per_direction", 1))
    return {
        "passed": actual >= expected_min,
        "expected_min_lanes_per_direction": expected_min,
        "actual_lanes_per_direction": actual,
    }


def opendrive_junction_topology(trace: dict[str, Any], xodr_path: Path, expected_road: str) -> dict[str, Any]:
    layout = str(_as_dict(trace.get("road_generation")).get("layout") or "")
    requires_junction = layout in INTERSECTION_LAYOUTS or expected_road == "common_cross_junction"
    if not requires_junction:
        return {"passed": True, "required": False, "layout": layout}
    try:
        root = ET.parse(xodr_path).getroot()
    except Exception as exc:
        return {"passed": False, "required": True, "layout": layout, "error": str(exc)}

    junctions = root.findall("./junction")
    default_junctions = [junction for junction in junctions if junction.get("type") == "default"]
    connections = []
    lane_links = []
    for junction in default_junctions:
        for connection in junction.findall("./connection"):
            connections.append(connection)
            lane_links.extend(connection.findall("./laneLink"))
    connector_roads = [
        road
        for road in root.findall("./road")
        if road.get("junction") not in {None, "-1"}
    ]
    passed = bool(default_junctions) and bool(connections) and bool(lane_links) and bool(connector_roads)
    return {
        "passed": passed,
        "required": True,
        "layout": layout,
        "default_junction_count": len(default_junctions),
        "connection_count": len(connections),
        "lane_link_count": len(lane_links),
        "connector_road_count": len(connector_roads),
    }


def expected_actor_count(source: dict[str, Any], semantic: dict[str, Any]) -> int | None:
    actors = _as_list(semantic.get("actors"))
    if actors:
        return len(actors)
    entities = _as_dict(source.get("entities"))
    if entities:
        return 1 + len(_as_list(entities.get("others")))
    structured = _as_dict(source.get("structured"))
    if structured.get("ego_vehicle"):
        count = 1
        if structured.get("other_vehicle"):
            count += 1
        narrative = source_narrative(source).lower()
        if "pedestrian" in narrative:
            count += 1
        if "cyclist" in narrative or "ebike" in narrative:
            count += 1
        return count
    return None


def xodr_line_segments(path: Path) -> list[tuple[float, float, float, float]]:
    root = ET.parse(path).getroot()
    segments: list[tuple[float, float, float, float]] = []
    for road in root.findall("road"):
        geometries = road.findall("./planView/geometry")
        road_points: list[tuple[float, float]] = []
        for geom in geometries:
            road_points.append((float(geom.get("x", 0.0)), float(geom.get("y", 0.0))))
        if road_points:
            last = geometries[-1]
            x = float(last.get("x", 0.0))
            y = float(last.get("y", 0.0))
            hdg = float(last.get("hdg", 0.0))
            length = float(last.get("length", 0.0))
            road_points.append((x + math.cos(hdg) * length, y + math.sin(hdg) * length))
        for start, end in zip(road_points, road_points[1:]):
            segments.append((start[0], start[1], end[0], end[1]))
        for geom in geometries:
            if geom.find("line") is None:
                continue
            x = float(geom.get("x", 0.0))
            y = float(geom.get("y", 0.0))
            hdg = float(geom.get("hdg", 0.0))
            length = float(geom.get("length", 0.0))
            segments.append((x, y, x + math.cos(hdg) * length, y + math.sin(hdg) * length))
    return segments


def _norm_angle_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def _heading_error_deg(actual: float, expected: float) -> float:
    return abs(_norm_angle_deg(actual - expected))


def _geometry_samples(geom: ET.Element) -> list[tuple[float, float, float]]:
    """Sample an OpenDRIVE geometry as (x, y, tangent heading rad)."""
    x = float(geom.get("x", 0.0))
    y = float(geom.get("y", 0.0))
    hdg = float(geom.get("hdg", 0.0))
    length = float(geom.get("length", 0.0))
    if length <= 0.0:
        return [(x, y, hdg)]

    if geom.find("line") is not None:
        return [(x, y, hdg), (x + math.cos(hdg) * length, y + math.sin(hdg) * length, hdg)]

    curvature_start = 0.0
    curvature_end = 0.0
    spiral = geom.find("spiral")
    arc = geom.find("arc")
    if spiral is not None:
        curvature_start = float(spiral.get("curvStart", 0.0))
        curvature_end = float(spiral.get("curvEnd", 0.0))
    elif arc is not None:
        curvature_start = curvature_end = float(arc.get("curvature", 0.0))
    else:
        return [(x, y, hdg), (x + math.cos(hdg) * length, y + math.sin(hdg) * length, hdg)]

    steps = max(8, int(length / 3.0))
    ds = length / steps
    samples = [(x, y, hdg)]
    cx, cy, ch = x, y, hdg
    for idx in range(steps):
        s_mid = (idx + 0.5) * ds
        frac_mid = s_mid / length
        k_mid = curvature_start + (curvature_end - curvature_start) * frac_mid
        mid_h = ch + k_mid * ds * 0.5
        cx += math.cos(mid_h) * ds
        cy += math.sin(mid_h) * ds
        s_next = (idx + 1.0) * ds
        frac_next = s_next / length
        k_next = curvature_start + (curvature_end - curvature_start) * frac_next
        ch += (k_mid + k_next) * 0.5 * ds
        samples.append((cx, cy, ch))
    return samples


def xodr_lane_center_segments(path: Path) -> list[dict[str, Any]]:
    """Return sampled centerline segments for every driving lane.

    OpenDRIVE right lanes travel with the road reference direction; left lanes
    travel opposite that direction. The generated maps in this repo use
    constant-width lanes, so sampling each road geometry and offsetting by lane
    center is sufficient for visual QA.
    """
    root = ET.parse(path).getroot()
    candidates: list[dict[str, Any]] = []
    for road in root.findall("road"):
        lane_section = road.find("./lanes/laneSection")
        if lane_section is None:
            continue

        lane_offsets: list[tuple[int, float, float]] = []
        for side_name, sign in (("right", -1.0), ("left", 1.0)):
            side = lane_section.find(side_name)
            if side is None:
                continue
            lanes = []
            for lane in side.findall("./lane"):
                lane_id = int(lane.get("id", "0"))
                if lane.get("type") != "driving":
                    continue
                width_node = lane.find("./width")
                width = float(width_node.get("a", 0.0)) if width_node is not None else 0.0
                if width <= 0.0:
                    continue
                lanes.append((abs(lane_id), lane_id, width))
            lanes.sort()
            cumulative = 0.0
            for _, lane_id, width in lanes:
                lane_offsets.append((lane_id, sign * (cumulative + width / 2.0), width))
                cumulative += width

        for geom in road.findall("./planView/geometry"):
            samples = _geometry_samples(geom)
            if len(samples) < 2:
                continue
            is_junction_connector = road.get("junction") not in {None, "-1"} and geom.find("line") is None
            if is_junction_connector:
                for a, b in zip(samples, samples[1:]):
                    x1, y1, _ = a
                    x2, y2, _ = b
                    seg_h = math.atan2(y2 - y1, x2 - x1)
                    candidates.append(
                        {
                            "road_id": road.get("id"),
                            "lane_id": "connector_ref",
                            "width": 0.0,
                            "segment": (x1, y1, x2, y2),
                            "legal_heading_deg": math.degrees(seg_h) % 360.0,
                        }
                    )
            for lane_id, offset, width in lane_offsets:
                lane_points = []
                for sx, sy, sh in samples:
                    left_x = -math.sin(sh)
                    left_y = math.cos(sh)
                    lane_points.append((sx + left_x * offset, sy + left_y * offset, sh))
                for a, b in zip(lane_points, lane_points[1:]):
                    x1, y1, h1 = a
                    x2, y2, h2 = b
                    seg_h = math.atan2(y2 - y1, x2 - x1) if math.hypot(x2 - x1, y2 - y1) > 1e-6 else (h1 + h2) / 2.0
                    legal_h = seg_h if lane_id < 0 else seg_h + math.pi
                    candidates.append(
                        {
                            "road_id": road.get("id"),
                            "lane_id": lane_id,
                            "width": width,
                            "segment": (x1, y1, x2, y2),
                            "legal_heading_deg": math.degrees(legal_h) % 360.0,
                        }
                    )
    return candidates


def distance_to_segment(px: float, py: float, segment: tuple[float, float, float, float]) -> float:
    x1, y1, x2, y2 = segment
    dx = x2 - x1
    dy = y2 - y1
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / length_sq))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def nearest_lane_candidate(
    px: float,
    py: float,
    heading_deg: float,
    candidates: list[dict[str, Any]],
) -> tuple[float, float, dict[str, Any] | None]:
    best_score = float("inf")
    best_distance = float("inf")
    best_heading_error = float("inf")
    best_candidate: dict[str, Any] | None = None
    for candidate in candidates:
        distance = distance_to_segment(px, py, candidate["segment"])
        heading_error = _heading_error_deg(heading_deg, float(candidate["legal_heading_deg"]))
        score = distance + min(heading_error, 90.0) / 90.0
        if score < best_score:
            best_score = score
            best_distance = distance
            best_heading_error = heading_error
            best_candidate = candidate
    return best_distance, best_heading_error, best_candidate


def road_envelope_m(trace: dict[str, Any], include_shoulder: bool = False) -> float:
    road_generation = _as_dict(trace.get("road_generation"))
    lane_width = float(road_generation.get("lane_width_m", 6.0))
    lanes_per_direction = int(road_generation.get("lanes_per_direction", 1))
    shoulder = float(road_generation.get("safety_shoulder_width_m", 30.0))
    envelope = lane_width * max(1, lanes_per_direction) + 0.5
    if include_shoulder:
        envelope += max(0.0, shoulder)
    return envelope


def actor_may_use_shoulder(trace: dict[str, Any], actor: dict[str, Any]) -> bool:
    road_generation = _as_dict(trace.get("road_generation"))
    layout = str(road_generation.get("layout") or "")
    actor_type = str(actor.get("type") or "")
    role = str(actor.get("role") or "")
    fault_role = str(actor.get("fault_role") or "")
    if actor_type in {"pedestrian", "cyclist"}:
        return True
    if (
        layout == "curbside_parking_corridor"
        and actor.get("controller") == "static"
        and (role == "collision_target" or fault_role == "collision_target" or role == "report_subject")
    ):
        return True
    return False


def point_may_use_curbside_parking_envelope(trace: dict[str, Any], actor: dict[str, Any], point: dict[str, Any]) -> bool:
    road_generation = _as_dict(trace.get("road_generation"))
    if str(road_generation.get("layout") or "") != "curbside_parking_corridor":
        return False
    if actor.get("type") != "vehicle":
        return False
    text = " ".join(
        str(value or "").lower()
        for value in (
            actor.get("description"),
            point.get("action"),
            _as_dict(actor.get("semantic_movement_intent")).get("maneuver_type"),
        )
    )
    return any(marker in text for marker in ("parallel park", "parallel-park", "parking", "parked", "reverse"))


def trace_road_alignment(trace: dict[str, Any], xodr_path: Path) -> dict[str, Any]:
    segments = xodr_line_segments(xodr_path)
    default_envelope = road_envelope_m(trace, include_shoulder=False)
    tolerance = float(_as_dict(trace.get("qa_config")).get("road_alignment_tolerance_m", 1.0))
    max_distance = 0.0
    offenders: list[dict[str, Any]] = []
    for actor_id, actor_value in _as_dict(trace.get("actors")).items():
        actor = _as_dict(actor_value)
        envelope = road_envelope_m(trace, include_shoulder=actor_may_use_shoulder(trace, actor))
        points = _as_list(actor.get("trace")) or [_as_dict(actor.get("pose"))]
        for point in points:
            point = _as_dict(point)
            if not point:
                continue
            distance = min(
                distance_to_segment(float(point.get("x", 0.0)), float(point.get("y", 0.0)), segment)
                for segment in segments
            ) if segments else float("inf")
            max_distance = max(max_distance, distance)
            if distance > envelope + tolerance:
                offenders.append({
                    "actor": actor_id,
                    "t": point.get("t"),
                    "distance_m": round(distance, 3),
                    "action": point.get("action"),
                })
    return {
        "passed": not offenders,
        "max_distance_to_road_centerline_m": round(max_distance, 3),
        "driving_envelope_m": round(default_envelope, 3),
        "tolerance_m": round(tolerance, 3),
        "off_road_points": offenders,
    }


def vehicle_lane_legality(trace: dict[str, Any], xodr_path: Path) -> dict[str, Any]:
    candidates = xodr_lane_center_segments(xodr_path)
    lane_center_tolerance = float(_as_dict(trace.get("qa_config")).get("lane_center_tolerance_m", 1.75))
    heading_tolerance = float(_as_dict(trace.get("qa_config")).get("lane_heading_tolerance_deg", 35.0))
    epsilon = 1e-6
    max_distance = 0.0
    max_heading_error = 0.0
    offenders: list[dict[str, Any]] = []
    intentional_opposing_lane_points: list[dict[str, Any]] = []
    for actor_id, actor_value in _as_dict(trace.get("actors")).items():
        actor = _as_dict(actor_value)
        if actor.get("type") != "vehicle":
            continue
        if actor.get("controller") == "static" and actor_may_use_shoulder(trace, actor):
            continue
        actor_text = " ".join(
            str(actor.get(key) or "").lower()
            for key in ("description", "role", "fault_role")
        )
        actor_allows_opposing_lane = any(
            marker in actor_text
            for marker in ("opposing lane", "opposing_lane", "center line", "centerline", "double yellow")
        )
        points = _as_list(actor.get("trace")) or [_as_dict(actor.get("pose"))]
        for point_value in points:
            point = _as_dict(point_value)
            if not point:
                continue
            px = float(point.get("x", 0.0))
            py = float(point.get("y", 0.0))
            heading_deg = float(point.get("h", 0.0))
            if point_may_use_curbside_parking_envelope(trace, actor, point):
                distance_to_road = min(
                    distance_to_segment(px, py, candidate["segment"])
                    for candidate in candidates
                ) if candidates else float("inf")
                if distance_to_road <= road_envelope_m(trace, include_shoulder=True) + epsilon:
                    continue
            distance, heading_error, candidate = nearest_lane_candidate(px, py, heading_deg, candidates)
            if candidate is None:
                offenders.append({"actor": actor_id, "t": point.get("t"), "reason": "no_driving_lanes_found"})
                continue
            max_distance = max(max_distance, distance)
            max_heading_error = max(max_heading_error, heading_error)
            action = str(point.get("action") or "").lower()
            point_allows_opposing_lane = actor_allows_opposing_lane or any(
                marker in action
                for marker in ("opposing", "centerline", "center_line", "pass", "impact", "post_impact")
            )
            wrong_way_but_intentional = (
                point_allows_opposing_lane
                and distance <= lane_center_tolerance + epsilon
                and heading_error > heading_tolerance + epsilon
                and heading_error >= 135.0
            )
            if wrong_way_but_intentional:
                intentional_opposing_lane_points.append(
                    {
                        "actor": actor_id,
                        "t": point.get("t"),
                        "x": round(px, 3),
                        "y": round(py, 3),
                        "h": round(heading_deg, 3),
                        "nearest_road_id": candidate.get("road_id"),
                        "nearest_lane_id": candidate.get("lane_id"),
                        "legal_heading_deg": round(float(candidate["legal_heading_deg"]), 1),
                        "action": point.get("action"),
                    }
                )
                continue
            if distance > lane_center_tolerance + epsilon or heading_error > heading_tolerance + epsilon:
                offenders.append(
                    {
                        "actor": actor_id,
                        "t": point.get("t"),
                        "x": round(px, 3),
                        "y": round(py, 3),
                        "h": round(heading_deg, 3),
                        "distance_to_lane_center_m": round(distance, 3),
                        "heading_error_deg": round(heading_error, 1),
                        "nearest_road_id": candidate.get("road_id"),
                        "nearest_lane_id": candidate.get("lane_id"),
                        "legal_heading_deg": round(float(candidate["legal_heading_deg"]), 1),
                        "action": point.get("action"),
                    }
                )
    return {
        "passed": not offenders,
        "lane_center_tolerance_m": lane_center_tolerance,
        "lane_heading_tolerance_deg": heading_tolerance,
        "max_distance_to_lane_center_m": round(max_distance, 3),
        "max_heading_error_deg": round(max_heading_error, 1),
        "illegal_vehicle_points": offenders,
        "intentional_opposing_lane_points": intentional_opposing_lane_points,
    }


def vehicle_lane_position_quality(trace: dict[str, Any], xodr_path: Path) -> dict[str, Any]:
    """Catch centerline straddling that broad lane-legality tolerances can miss."""
    candidates = xodr_lane_center_segments(xodr_path)
    road_generation = _as_dict(trace.get("road_generation"))
    lane_width = float(road_generation.get("lane_width_m", 6.0))
    max_offset = float(_as_dict(trace.get("qa_config")).get("max_lane_center_offset_m", min(2.25, lane_width * 0.4)))
    max_observed = 0.0
    offenders: list[dict[str, Any]] = []
    ignored_intentional: list[dict[str, Any]] = []

    for actor_id, actor_value in _as_dict(trace.get("actors")).items():
        actor = _as_dict(actor_value)
        if actor.get("type") != "vehicle":
            continue
        if actor.get("controller") == "static" and actor_may_use_shoulder(trace, actor):
            continue
        actor_text = " ".join(
            str(actor.get(key) or "").lower()
            for key in ("description", "role", "fault_role")
        )
        actor_allows_centerline = any(
            marker in actor_text
            for marker in (
                "opposing lane",
                "opposing_lane",
                "center line",
                "centerline",
                "double yellow",
                "wrong way",
                "wrong-way",
                "encroach",
                "partially entered",
                "partially enter",
                "partially in",
                "partly in",
                "straddl",
            )
        )
        points = _as_list(actor.get("trace")) or [_as_dict(actor.get("pose"))]
        for point_value in points:
            point = _as_dict(point_value)
            if not point:
                continue
            action = str(point.get("action") or "").lower()
            # Generated turn connectors and curbside proxies can legitimately
            # depart from straight-lane centers; straight/approach/stop/yield
            # points should not straddle the centerline.
            if "turn" in action or "impact" in action:
                continue
            point_allows_centerline = actor_allows_centerline or any(
                marker in action
                for marker in (
                    "opposing",
                    "centerline",
                    "center_line",
                    "double_yellow",
                    "pass",
                    "encroach",
                    "partially_entered",
                    "partially_enter",
                    "partially_in",
                    "partly_in",
                    "straddl",
                )
            )
            px = float(point.get("x", 0.0))
            py = float(point.get("y", 0.0))
            heading_deg = float(point.get("h", 0.0))
            if point_may_use_curbside_parking_envelope(trace, actor, point):
                distance_to_road = min(
                    distance_to_segment(px, py, candidate["segment"])
                    for candidate in candidates
                ) if candidates else float("inf")
                if distance_to_road <= road_envelope_m(trace, include_shoulder=True):
                    continue
            distance, heading_error, candidate = nearest_lane_candidate(px, py, heading_deg, candidates)
            if candidate is None:
                offenders.append({"actor": actor_id, "t": point.get("t"), "reason": "no_driving_lanes_found"})
                continue
            if float(candidate.get("width") or 0.0) <= 0.0:
                continue
            max_observed = max(max_observed, distance)
            if distance <= max_offset:
                continue
            item = {
                "actor": actor_id,
                "t": point.get("t"),
                "x": round(px, 3),
                "y": round(py, 3),
                "h": round(heading_deg, 3),
                "distance_to_lane_center_m": round(distance, 3),
                "max_allowed_offset_m": round(max_offset, 3),
                "nearest_road_id": candidate.get("road_id"),
                "nearest_lane_id": candidate.get("lane_id"),
                "legal_heading_deg": round(float(candidate["legal_heading_deg"]), 1),
                "heading_error_deg": round(heading_error, 1),
                "action": point.get("action"),
            }
            if point_allows_centerline:
                ignored_intentional.append(item)
            else:
                offenders.append(item)

    return {
        "passed": not offenders,
        "max_lane_center_offset_m": round(max_offset, 3),
        "max_observed_offset_m": round(max_observed, 3),
        "centerline_or_lane_boundary_points": offenders,
        "intentional_centerline_or_opposing_lane_points": ignored_intentional,
    }


def trajectory_summary_ok(trace: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    max_heading_step = float(_as_dict(trace.get("qa_config")).get("max_heading_step_deg", 120.0))
    for actor_id, actor_value in _as_dict(trace.get("actors")).items():
        actor = _as_dict(actor_value)
        if actor.get("controller") == "static":
            if not actor.get("pose"):
                failures.append(f"{actor_id}: static actor missing pose")
            continue
        points = [_as_dict(point) for point in _as_list(actor.get("trace"))]
        if len(points) < 2:
            failures.append(f"{actor_id}: moving actor has fewer than 2 points")
            continue
        if float(points[0].get("t", -1.0)) != 0.0:
            failures.append(f"{actor_id}: trace does not start at t=0")
        for prev, curr in zip(points, points[1:]):
            if float(curr.get("t", 0.0)) <= float(prev.get("t", 0.0)):
                failures.append(f"{actor_id}: non-monotonic timestamps")
                break
            if actor.get("type") in {"vehicle", "cyclist"}:
                prev_h = float(prev.get("h", 0.0))
                curr_h = float(curr.get("h", 0.0))
                heading_step = _heading_error_deg(curr_h, prev_h)
                if heading_step > max_heading_step:
                    failures.append(
                        f"{actor_id}: heading jumps {heading_step:.1f} deg between "
                        f"t={prev.get('t')} and t={curr.get('t')}"
                    )
                    break
    return {"passed": not failures, "failures": failures}


def screenshot_html(
    html_path: Path,
    output_path: Path | None,
    viewport_width: int = 1920,
    viewport_height: int = 1080,
    wait_ms: int = 2500,
) -> dict[str, Any]:
    if output_path is None:
        return {"attempted": False, "path": None, "error": None}
    try:
        from playwright.sync_api import sync_playwright
    except Exception as exc:
        return {"attempted": True, "path": str(output_path), "error": f"playwright unavailable: {exc}"}
    try:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with sync_playwright() as p:
            # Use Playwright's bundled Chromium instead of a system Chrome
            # executable. In batch QA the system browser path has been more
            # prone to abrupt exits, while bundled Chromium follows the same
            # stable render flow as the standalone render helper.
            browser = p.chromium.launch(headless=True)
            page = browser.new_page(viewport={"width": viewport_width, "height": viewport_height})
            page.goto(html_path.resolve().as_uri(), wait_until="networkidle", timeout=30000)
            page.wait_for_timeout(wait_ms)
            page.screenshot(path=str(output_path), full_page=True)
            browser.close()
        return {
            "attempted": True,
            "path": str(output_path),
            "error": None,
            "viewport": {"width": viewport_width, "height": viewport_height},
            "wait_ms": wait_ms,
        }
    except Exception as exc:
        return {
            "attempted": True,
            "path": str(output_path),
            "error": str(exc),
            "viewport": {"width": viewport_width, "height": viewport_height},
            "wait_ms": wait_ms,
        }


def qa(
    trace_path: Path,
    xodr_path: Path,
    html_path: Path,
    source_path: Path | None,
    semantic_path: Path | None,
    screenshot_path: Path | None,
    screenshot_width: int = 1920,
    screenshot_height: int = 1080,
    screenshot_wait_ms: int = 2500,
) -> dict[str, Any]:
    trace = read_json(trace_path)
    source = read_json(source_path)
    semantic = read_json(semantic_path)
    expected_road = expected_road_family(source, semantic)
    actual_road = str(_as_dict(trace.get("road_generation")).get("layout") or "unknown")
    expected_collision = expected_collision_family(source, semantic)
    actual_collision = str(_as_dict(trace.get("collision")).get("type") or "unknown")
    expected_count = expected_actor_count(source, semantic)
    actual_count = len(_as_dict(trace.get("actors")))
    html_text = html_path.read_text(encoding="utf-8") if html_path.exists() else ""
    actor_mentions = sorted(set(re.findall(r"<div><span[^>]*></span><b>([^<]+)</b>", html_text)))

    checks = {
        "road_type_matches_pdf": {
            "passed": road_type_compatible(expected_road, actual_road),
            "expected": expected_road,
            "actual": actual_road,
        },
        "actor_count_matches_pdf": {
            "passed": expected_count is None or expected_count == actual_count,
            "expected": expected_count,
            "actual": actual_count,
            "visualized_actor_labels": actor_mentions,
        },
        "entity_trajectories_present_and_ordered": trajectory_summary_ok(trace),
        "entity_trajectories_on_road": trace_road_alignment(trace, xodr_path),
        "vehicle_lane_legality": vehicle_lane_legality(trace, xodr_path),
        "vehicle_lane_position_quality": vehicle_lane_position_quality(trace, xodr_path),
        "lane_count_sufficient_for_source": lane_count_sufficient(trace, source, semantic),
        "opendrive_junction_topology": opendrive_junction_topology(trace, xodr_path, expected_road),
        "chinese_scene_description": chinese_scene_description_quality(trace, source, semantic),
        "collision_type_matches_pdf": {
            "passed": expected_collision in {"unknown", actual_collision},
            "expected": expected_collision,
            "actual": actual_collision,
        },
    }
    passed = all(bool(item.get("passed")) for item in checks.values())
    return {
        "scene": trace_path.stem,
        "passed": passed,
        "trace": str(trace_path),
        "xodr": str(xodr_path),
        "html": str(html_path),
        "screenshot": screenshot_html(
            html_path,
            screenshot_path,
            viewport_width=screenshot_width,
            viewport_height=screenshot_height,
            wait_ms=screenshot_wait_ms,
        ),
        "checks": checks,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Run visual/fidelity QA for a replay visualization")
    parser.add_argument("--trace", required=True, type=Path)
    parser.add_argument("--xodr", required=True, type=Path)
    parser.add_argument("--html", required=True, type=Path)
    parser.add_argument("--source", type=Path)
    parser.add_argument("--semantic", type=Path)
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument("--screenshot-width", type=int, default=1920)
    parser.add_argument("--screenshot-height", type=int, default=1080)
    parser.add_argument("--screenshot-wait-ms", type=int, default=2500)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    result = qa(
        args.trace,
        args.xodr,
        args.html,
        args.source,
        args.semantic,
        args.screenshot,
        args.screenshot_width,
        args.screenshot_height,
        args.screenshot_wait_ms,
    )
    text = json.dumps(result, indent=2, ensure_ascii=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
