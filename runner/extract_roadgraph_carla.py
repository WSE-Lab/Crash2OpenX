#!/usr/bin/env python3
"""Extract CARLA/OpenDRIVE waypoint and junction-lane roadgraph data.

This extractor is intentionally based on the OpenDRIVE junction laneLink table
plus CARLA waypoint geometry. It avoids CARLA's previous_until_lane_start() on
junction connector waypoints, which can segfault in CARLA 0.9.15 for generated
OpenDRIVE maps.

Single map:

    python extract_roadgraph_carla.py \
        --xodr opendrive_seed/001_Zoox_April_11_2025.xodr \
        --out outputs/roadgraph_map_cache/001_Zoox_April_11_2025 \
        --spacing 2.0

Batch:

    python extract_roadgraph_carla.py --batch \
        --input-dir opendrive_seed \
        --output-root outputs/roadgraph_map_cache

Outputs:
    waypoints.json
    junctions.json
    legs.json
    route_candidates.json
    roadgraph_selfcheck.json
"""

from __future__ import annotations

import argparse
import glob
import hashlib
import json
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


REPO_ROOT = Path(__file__).resolve().parent
CARLA_PYTHONAPI = Path(os.environ.get(
    "CARLA_PYTHONAPI", "/opt/carla/PythonAPI/carla"))
CARLA_EGGS = glob.glob(str(CARLA_PYTHONAPI / "dist" / "carla-*py3*.egg"))

sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(CARLA_PYTHONAPI))
sys.path.extend(CARLA_EGGS)

try:
    import carla  # type: ignore  # provided by the CARLA Python API
except ImportError as exc:
    raise RuntimeError(
        "Cannot import carla. Install the client (scripts/setup_exec_env.sh "
        "pins carla==0.9.16) or point CARLA_PYTHONAPI at PythonAPI/carla."
    ) from exc

try:
    from opendrive_repair import repair_opendrive_xml
except ImportError:
    repair_opendrive_xml = None


EPS = 0.02
END_MATCH_TOLERANCE = 3.0


@dataclass
class EndRef:
    element_type: str
    element_id: int
    contact_point: Optional[str]


@dataclass
class LaneInfo:
    lane_id: int
    lane_type: str
    predecessor: Optional[int]
    successor: Optional[int]


@dataclass
class WidthInfo:
    s_offset: float
    a: float
    b: float
    c: float
    d: float


@dataclass
class LaneSectionInfo:
    s: float
    lanes: Dict[int, LaneInfo]
    widths: Dict[int, List[WidthInfo]]


@dataclass
class GeometryInfo:
    s: float
    x: float
    y: float
    hdg: float
    length: float
    kind: str
    params: Dict[str, float]


@dataclass
class RoadInfo:
    road_id: int
    length: float
    junction_id: Optional[int]
    predecessor: Optional[EndRef]
    successor: Optional[EndRef]
    lanes: Dict[int, LaneInfo]
    lane_sections: List[LaneSectionInfo]
    geometries: List[GeometryInfo]


@dataclass
class LaneLinkInfo:
    from_lane: int
    to_lane: int


@dataclass
class ConnectionInfo:
    junction_id: int
    connection_id: str
    incoming_road: int
    connecting_road: int
    contact_point: str
    lane_links: List[LaneLinkInfo]


def enum_to_string(value) -> str:
    text = str(value)
    return text.split(".")[-1] if "." in text else text


def safe_int(value: Optional[str]) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        return None


def load_xodr(path: Path) -> str:
    data = path.read_text(encoding="utf-8")
    index = data.find("<OpenDRIVE")
    if index == -1:
        raise ValueError(f"{path} does not contain an <OpenDRIVE> root")
    return data[index:]


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def prepare_xodr(xodr_text: str, use_repair: bool) -> Tuple[str, int]:
    if not use_repair:
        return xodr_text, 0
    if repair_opendrive_xml is None:
        raise RuntimeError("opendrive_repair.repair_opendrive_xml is not available")
    repaired, changes = repair_opendrive_xml(xodr_text)
    return repaired, int(changes)


def parse_end_ref(element: Optional[ET.Element]) -> Optional[EndRef]:
    if element is None:
        return None
    element_id = safe_int(element.get("elementId"))
    if element_id is None:
        return None
    return EndRef(
        element_type=element.get("elementType") or "",
        element_id=element_id,
        contact_point=element.get("contactPoint"),
    )


def parse_opendrive(xodr_text: str) -> Tuple[Dict[int, RoadInfo], Dict[int, List[ConnectionInfo]]]:
    root = ET.fromstring(xodr_text)
    roads: Dict[int, RoadInfo] = {}
    junction_connections: Dict[int, List[ConnectionInfo]] = {}

    for road in root.findall("road"):
        road_id = safe_int(road.get("id"))
        if road_id is None:
            continue
        junction_raw = safe_int(road.get("junction"))
        junction_id = junction_raw if junction_raw is not None and junction_raw >= 0 else None
        link = road.find("link")
        lanes: Dict[int, LaneInfo] = {}
        lane_sections: List[LaneSectionInfo] = []
        geometries: List[GeometryInfo] = []
        for geometry in road.findall("./planView/geometry"):
            children = list(geometry)
            if not children:
                continue
            child = children[0]
            geometries.append(
                GeometryInfo(
                    s=float(geometry.get("s") or 0.0),
                    x=float(geometry.get("x") or 0.0),
                    y=float(geometry.get("y") or 0.0),
                    hdg=float(geometry.get("hdg") or 0.0),
                    length=float(geometry.get("length") or 0.0),
                    kind=child.tag,
                    params={key: float(value) for key, value in child.attrib.items()},
                )
            )
        for lane_section in road.findall("./lanes/laneSection"):
            section_lanes: Dict[int, LaneInfo] = {}
            section_widths: Dict[int, List[WidthInfo]] = {}
            for side_name in ("left", "center", "right"):
                side = lane_section.find(side_name)
                if side is None:
                    continue
                for lane in side.findall("lane"):
                    lane_id = safe_int(lane.get("id"))
                    if lane_id is None:
                        continue
                    lane_link = lane.find("link")
                    predecessor = safe_int(lane_link.find("predecessor").get("id")) if lane_link is not None and lane_link.find("predecessor") is not None else None
                    successor = safe_int(lane_link.find("successor").get("id")) if lane_link is not None and lane_link.find("successor") is not None else None
                    lanes[lane_id] = LaneInfo(
                        lane_id=lane_id,
                        lane_type=lane.get("type") or "",
                        predecessor=predecessor,
                        successor=successor,
                    )
                    section_lanes[lane_id] = lanes[lane_id]
                    width_records = []
                    for width in lane.findall("width"):
                        width_records.append(
                            WidthInfo(
                                s_offset=float(width.get("sOffset") or 0.0),
                                a=float(width.get("a") or 0.0),
                                b=float(width.get("b") or 0.0),
                                c=float(width.get("c") or 0.0),
                                d=float(width.get("d") or 0.0),
                            )
                        )
                    section_widths[lane_id] = sorted(width_records, key=lambda item: item.s_offset)
            lane_sections.append(
                LaneSectionInfo(
                    s=float(lane_section.get("s") or 0.0),
                    lanes=section_lanes,
                    widths=section_widths,
                )
            )
        roads[road_id] = RoadInfo(
            road_id=road_id,
            length=float(road.get("length") or 0.0),
            junction_id=junction_id,
            predecessor=parse_end_ref(link.find("predecessor") if link is not None else None),
            successor=parse_end_ref(link.find("successor") if link is not None else None),
            lanes=lanes,
            lane_sections=sorted(lane_sections, key=lambda item: item.s),
            geometries=sorted(geometries, key=lambda item: item.s),
        )

    for junction in root.findall("junction"):
        junction_id = safe_int(junction.get("id"))
        if junction_id is None:
            continue
        entries: List[ConnectionInfo] = []
        for connection in junction.findall("connection"):
            incoming_road = safe_int(connection.get("incomingRoad"))
            connecting_road = safe_int(connection.get("connectingRoad"))
            if incoming_road is None or connecting_road is None:
                continue
            lane_links: List[LaneLinkInfo] = []
            for lane_link in connection.findall("laneLink"):
                from_lane = safe_int(lane_link.get("from"))
                to_lane = safe_int(lane_link.get("to"))
                if from_lane is None or to_lane is None:
                    continue
                lane_links.append(LaneLinkInfo(from_lane=from_lane, to_lane=to_lane))
            entries.append(
                ConnectionInfo(
                    junction_id=junction_id,
                    connection_id=connection.get("id") or "",
                    incoming_road=incoming_road,
                    connecting_road=connecting_road,
                    contact_point=connection.get("contactPoint") or "start",
                    lane_links=lane_links,
                )
            )
        junction_connections[junction_id] = entries

    return roads, junction_connections


def wid(wp) -> str:
    return f"r{wp.road_id}:sec{wp.section_id}:l{wp.lane_id}:s{float(wp.s):.2f}"


def xyyaw(wp) -> Tuple[float, float, float]:
    transform = wp.transform
    return (
        float(transform.location.x),
        float(transform.location.y),
        float(transform.rotation.yaw) % 360.0,
    )


def xy(wp) -> Tuple[float, float]:
    return float(wp.transform.location.x), float(wp.transform.location.y)


def dist_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def turn_type(entry_yaw: float, exit_yaw: float) -> str:
    delta = (exit_yaw - entry_yaw + 180.0) % 360.0 - 180.0
    if delta > 25.0:
        return "left"
    if delta < -25.0:
        return "right"
    return "straight"


def normalize_degrees(angle: float) -> float:
    return angle % 360.0


def find_geometry(road: RoadInfo, s_value: float) -> Optional[GeometryInfo]:
    selected = None
    for geometry in road.geometries:
        if geometry.s <= s_value + 1e-6:
            selected = geometry
        else:
            break
    return selected or (road.geometries[0] if road.geometries else None)


def road_reference_pose(road: RoadInfo, s_value: float) -> Optional[Tuple[float, float, float]]:
    geometry = find_geometry(road, s_value)
    if geometry is None:
        return None
    ds = min(max(0.0, s_value - geometry.s), geometry.length)
    hdg0 = geometry.hdg

    if geometry.kind == "line":
        return (
            geometry.x + ds * math.cos(hdg0),
            geometry.y + ds * math.sin(hdg0),
            hdg0,
        )

    if geometry.kind == "arc":
        curvature = geometry.params.get("curvature", 0.0)
        if abs(curvature) < 1e-9:
            return (
                geometry.x + ds * math.cos(hdg0),
                geometry.y + ds * math.sin(hdg0),
                hdg0,
            )
        hdg = hdg0 + curvature * ds
        return (
            geometry.x + (math.sin(hdg) - math.sin(hdg0)) / curvature,
            geometry.y - (math.cos(hdg) - math.cos(hdg0)) / curvature,
            hdg,
        )

    if geometry.kind == "spiral":
        curv_start = geometry.params.get("curvStart", 0.0)
        curv_end = geometry.params.get("curvEnd", 0.0)
        length = max(geometry.length, 1e-9)
        dk = (curv_end - curv_start) / length
        steps = max(1, int(abs(ds) / 0.1))
        step = ds / steps
        x_value = geometry.x
        y_value = geometry.y
        for index in range(steps):
            mid_s = (index + 0.5) * step
            mid_hdg = hdg0 + curv_start * mid_s + 0.5 * dk * mid_s * mid_s
            x_value += step * math.cos(mid_hdg)
            y_value += step * math.sin(mid_hdg)
        hdg = hdg0 + curv_start * ds + 0.5 * dk * ds * ds
        return x_value, y_value, hdg

    return None


def lane_section_at(road: RoadInfo, s_value: float) -> Optional[LaneSectionInfo]:
    selected = None
    for section in road.lane_sections:
        if section.s <= s_value + 1e-6:
            selected = section
        else:
            break
    return selected or (road.lane_sections[0] if road.lane_sections else None)


def lane_width_at(road: RoadInfo, section: LaneSectionInfo, lane_id: int, s_value: float) -> float:
    widths = section.widths.get(int(lane_id), [])
    if not widths:
        return 0.0
    local_s = max(0.0, s_value - section.s)
    selected = widths[0]
    for width in widths:
        if width.s_offset <= local_s + 1e-6:
            selected = width
        else:
            break
    ds = local_s - selected.s_offset
    return selected.a + selected.b * ds + selected.c * ds * ds + selected.d * ds * ds * ds


def lane_center_offset(road: RoadInfo, lane_id: int, s_value: float) -> Optional[float]:
    section = lane_section_at(road, s_value)
    if section is None or lane_id == 0 or lane_id not in section.lanes:
        return None

    if lane_id > 0:
        offset = 0.0
        for current_lane_id in sorted(lid for lid in section.lanes if lid > 0):
            width = lane_width_at(road, section, current_lane_id, s_value)
            if current_lane_id == lane_id:
                return offset + width / 2.0
            offset += width
        return None

    offset = 0.0
    for current_lane_id in sorted((lid for lid in section.lanes if lid < 0), reverse=True):
        width = lane_width_at(road, section, current_lane_id, s_value)
        if current_lane_id == lane_id:
            return -(offset + width / 2.0)
        offset += width
    return None


def xodr_lane_pose(
    roads: Dict[int, RoadInfo],
    road_id: int,
    lane_id: int,
    s_value: float,
) -> Optional[Tuple[float, float, float]]:
    road = roads.get(int(road_id))
    if road is None:
        return None
    s_clamped = clamp_s(road, float(s_value))
    reference = road_reference_pose(road, s_clamped)
    offset = lane_center_offset(road, int(lane_id), s_clamped)
    if reference is None or offset is None:
        return None
    ref_x, ref_y, ref_hdg = reference
    x_value = ref_x - math.sin(ref_hdg) * offset
    y_value = ref_y + math.cos(ref_hdg) * offset
    lane_hdg = ref_hdg if int(lane_id) < 0 else ref_hdg + math.pi
    return x_value, y_value, normalize_degrees(math.degrees(lane_hdg))


def pose_for_wp(wp, roads: Optional[Dict[int, RoadInfo]] = None) -> Tuple[float, float, float]:
    if roads is not None:
        pose = xodr_lane_pose(roads, int(wp.road_id), int(wp.lane_id), float(wp.s))
        if pose is not None:
            return pose
    return xyyaw(wp)


def xy_for_wp(wp, roads: Optional[Dict[int, RoadInfo]] = None) -> Tuple[float, float]:
    x_value, y_value, _yaw = pose_for_wp(wp, roads)
    return x_value, y_value


def safe_waypoint_list(callable_obj, default: Optional[List] = None) -> List:
    try:
        result = callable_obj()
    except RuntimeError:
        return default or []
    return list(result or [])


def road_end_for_s(s_value: float, length: float) -> str:
    if abs(s_value) <= abs(length - s_value):
        return "start"
    return "end"


def xodr_s_for_end(road: RoadInfo, end_name: str) -> float:
    if end_name == "start":
        return 0.0
    return max(0.0, road.length - EPS)


def clamp_s(road: RoadInfo, s_value: float) -> float:
    if road.length <= 0.0:
        return 0.0
    return min(max(0.0, s_value), max(0.0, road.length - EPS))


def get_xodr_wp(cmap, roads: Dict[int, RoadInfo], road_id: int, lane_id: int, s_value: float):
    road = roads.get(int(road_id))
    if road is None:
        return None
    candidates = [
        clamp_s(road, float(s_value)),
        clamp_s(road, float(s_value) - EPS),
        clamp_s(road, float(s_value) + EPS),
        0.0 if float(s_value) <= END_MATCH_TOLERANCE else clamp_s(road, road.length - EPS),
    ]
    seen = set()
    for candidate in candidates:
        key = round(candidate, 4)
        if key in seen:
            continue
        seen.add(key)
        try:
            wp = cmap.get_waypoint_xodr(int(road_id), int(lane_id), float(candidate))
        except RuntimeError:
            wp = None
        if wp is not None:
            return wp
    return None


def sample_lane_between(
    cmap,
    roads: Dict[int, RoadInfo],
    road_id: int,
    lane_id: int,
    start_s: float,
    end_s: float,
    spacing: float,
    start_wp=None,
    end_wp=None,
) -> List:
    road = roads.get(int(road_id))
    if road is None:
        return []

    points = []
    if start_wp is not None:
        points.append(start_wp)
    else:
        wp = get_xodr_wp(cmap, roads, road_id, lane_id, start_s)
        if wp is not None:
            points.append(wp)

    if abs(end_s - start_s) > 1e-6:
        step = spacing if end_s > start_s else -spacing
        current = start_s + step
        while (step > 0 and current < end_s - 0.05) or (step < 0 and current > end_s + 0.05):
            wp = get_xodr_wp(cmap, roads, road_id, lane_id, current)
            if wp is not None:
                points.append(wp)
            current += step

    if end_wp is not None:
        points.append(end_wp)
    else:
        wp = get_xodr_wp(cmap, roads, road_id, lane_id, end_s)
        if wp is not None:
            points.append(wp)

    return dedup_wps(points)


def dedup_wps(waypoints: Iterable) -> List:
    result = []
    seen_last = None
    for wp in waypoints:
        if wp is None:
            continue
        waypoint_id = wid(wp)
        if waypoint_id == seen_last:
            continue
        result.append(wp)
        seen_last = waypoint_id
    return result


def nearest_road_endpoint(cmap, roads: Dict[int, RoadInfo], road_id: int, lane_id: int, target_wp):
    road = roads.get(int(road_id))
    if road is None:
        return None, None
    candidates = []
    for end_name in ("start", "end"):
        wp = get_xodr_wp(cmap, roads, road_id, lane_id, xodr_s_for_end(road, end_name))
        if wp is not None:
            candidates.append((end_name, wp, dist_xy(xy_for_wp(wp, roads), xy_for_wp(target_wp, roads))))
    if not candidates:
        return None, None
    end_name, wp, _distance = min(candidates, key=lambda item: item[2])
    return end_name, wp


def sample_approach(
    cmap,
    roads: Dict[int, RoadInfo],
    road_id: int,
    lane_id: int,
    endpoint_wp,
    distance: float,
    spacing: float,
) -> List:
    road = roads.get(int(road_id))
    if road is None or endpoint_wp is None:
        return []
    endpoint_s = float(endpoint_wp.s)
    if endpoint_s >= road.length / 2.0:
        start_s = max(0.0, endpoint_s - distance)
    else:
        start_s = min(max(0.0, road.length - EPS), endpoint_s + distance)
    return sample_lane_between(cmap, roads, road_id, lane_id, start_s, endpoint_s, spacing, end_wp=endpoint_wp)


def sample_departure(
    cmap,
    roads: Dict[int, RoadInfo],
    road_id: int,
    lane_id: int,
    endpoint_wp,
    distance: float,
    spacing: float,
) -> List:
    road = roads.get(int(road_id))
    if road is None or endpoint_wp is None:
        return []
    start_s = float(endpoint_wp.s)
    if start_s >= road.length / 2.0:
        end_s = max(0.0, start_s - distance)
    else:
        end_s = min(max(0.0, road.length - EPS), start_s + distance)
    return sample_lane_between(cmap, roads, road_id, lane_id, start_s, end_s, spacing, start_wp=endpoint_wp)


def find_entry_connection(
    connections: Sequence[ConnectionInfo],
    connector_road_id: int,
    connector_lane_id: int,
    entry_end: str,
) -> Tuple[Optional[ConnectionInfo], Optional[LaneLinkInfo]]:
    relaxed: List[Tuple[ConnectionInfo, LaneLinkInfo]] = []
    for connection in connections:
        if connection.connecting_road != connector_road_id:
            continue
        for lane_link in connection.lane_links:
            if lane_link.to_lane != connector_lane_id:
                continue
            if connection.contact_point == entry_end:
                return connection, lane_link
            relaxed.append((connection, lane_link))
    return relaxed[0] if relaxed else (None, None)


def outgoing_from_connector(
    road: RoadInfo,
    connector_lane_id: int,
    exit_end: str,
) -> Tuple[Optional[int], Optional[int]]:
    lane = road.lanes.get(int(connector_lane_id))
    if lane is None:
        return None, None
    if exit_end == "end":
        end_ref = road.successor
        outgoing_lane = lane.successor
    else:
        end_ref = road.predecessor
        outgoing_lane = lane.predecessor
    if end_ref is None or end_ref.element_type != "road" or outgoing_lane is None:
        return None, None
    return end_ref.element_id, outgoing_lane


def connect_world(args, xodr_text: str):
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    params = carla.OpendriveGenerationParameters(
        vertex_distance=args.vertex_distance,
        max_road_length=args.max_road_length,
        wall_height=args.wall_height,
        additional_width=args.additional_width,
        smooth_junctions=True,
        enable_mesh_visibility=False,
    )
    return client.generate_opendrive_world(xodr_text, params).get_map()


def collect_junction_objects(cmap) -> Dict[int, object]:
    junctions = {}
    for start_wp, _end_wp in cmap.get_topology():
        if not start_wp.is_junction:
            continue
        try:
            junction = start_wp.get_junction()
        except RuntimeError:
            junction = None
        if junction is not None:
            junctions[junction.id] = junction
    return junctions


def build_junctions_and_routes(
    cmap,
    roads: Dict[int, RoadInfo],
    junction_connections: Dict[int, List[ConnectionInfo]],
    spacing: float,
    approach_distance: float,
    departure_distance: float,
):
    junctions = {}
    routes = []
    extra_waypoints = []
    diagnostics = []
    route_id = 0

    for carla_junction_id, junction in sorted(collect_junction_objects(cmap).items()):
        try:
            pairs = junction.get_waypoints(carla.LaneType.Driving)
        except RuntimeError as exc:
            diagnostics.append({"level": "error", "junction_id": carla_junction_id, "error": str(exc)})
            continue

        bb = junction.bounding_box
        center = {"x": round(float(bb.location.x), 3), "y": round(float(bb.location.y), 3)}
        connectors = []

        for entry_wp, exit_wp in pairs:
            connector_road_id = int(entry_wp.road_id)
            connector_lane_id = int(entry_wp.lane_id)
            connector_road = roads.get(connector_road_id)
            if connector_road is None:
                diagnostics.append({"level": "warn", "reason": "missing_connector_road", "road_id": connector_road_id})
                continue
            junction_id = connector_road.junction_id if connector_road.junction_id is not None else int(carla_junction_id)
            entry_end = road_end_for_s(float(entry_wp.s), connector_road.length)
            exit_end = road_end_for_s(float(exit_wp.s), connector_road.length)
            connection, lane_link = find_entry_connection(
                junction_connections.get(junction_id, []),
                connector_road_id,
                connector_lane_id,
                entry_end,
            )
            if connection is None or lane_link is None:
                diagnostics.append(
                    {
                        "level": "warn",
                        "reason": "missing_entry_lane_link",
                        "junction_id": junction_id,
                        "connector_road_id": connector_road_id,
                        "connector_lane_id": connector_lane_id,
                        "entry_end": entry_end,
                    }
                )
                continue

            incoming_road_id = connection.incoming_road
            incoming_lane_id = lane_link.from_lane
            outgoing_road_id, outgoing_lane_id = outgoing_from_connector(connector_road, connector_lane_id, exit_end)

            incoming_end, incoming_endpoint_wp = nearest_road_endpoint(
                cmap, roads, incoming_road_id, incoming_lane_id, entry_wp
            )
            outgoing_end = None
            outgoing_endpoint_wp = None
            if outgoing_road_id is not None and outgoing_lane_id is not None:
                outgoing_end, outgoing_endpoint_wp = nearest_road_endpoint(
                    cmap, roads, outgoing_road_id, outgoing_lane_id, exit_wp
                )

            approach_wps = sample_approach(
                cmap, roads, incoming_road_id, incoming_lane_id, incoming_endpoint_wp, approach_distance, spacing
            )
            connector_wps = sample_lane_between(
                cmap,
                roads,
                connector_road_id,
                connector_lane_id,
                float(entry_wp.s),
                float(exit_wp.s),
                spacing,
                start_wp=entry_wp,
                end_wp=exit_wp,
            )
            departure_wps = []
            if outgoing_road_id is not None and outgoing_lane_id is not None:
                departure_wps = sample_departure(
                    cmap,
                    roads,
                    outgoing_road_id,
                    outgoing_lane_id,
                    outgoing_endpoint_wp,
                    departure_distance,
                    spacing,
                )

            all_wps = dedup_wps(list(approach_wps) + list(connector_wps) + list(departure_wps))
            extra_waypoints.extend(all_wps)
            entry_yaw = pose_for_wp(incoming_endpoint_wp or entry_wp, roads)[2]
            exit_yaw = pose_for_wp(outgoing_endpoint_wp or exit_wp, roads)[2]
            turn = turn_type(entry_yaw, exit_yaw)

            connector_record = {
                "junction_id": junction_id,
                "connection_id": connection.connection_id,
                "incoming_road_id": incoming_road_id,
                "incoming_lane_id": incoming_lane_id,
                "connector_road_id": connector_road_id,
                "connector_lane_id": connector_lane_id,
                "outgoing_road_id": outgoing_road_id,
                "outgoing_lane_id": outgoing_lane_id,
                "entry_end": entry_end,
                "exit_end": exit_end,
                "incoming_end": incoming_end,
                "outgoing_end": outgoing_end,
                "turn_type": turn,
                "entry": wid(entry_wp),
                "exit": wid(exit_wp),
                "polyline": [wid(wp) for wp in connector_wps],
            }
            connectors.append(connector_record)

            if all_wps:
                routes.append(
                    {
                        "id": f"route_{route_id:04d}",
                        "type": turn,
                        "turn_type": turn,
                        "junction_id": junction_id,
                        "connection_id": connection.connection_id,
                        "incoming_road_id": incoming_road_id,
                        "incoming_lane_id": incoming_lane_id,
                        "connector_road_id": connector_road_id,
                        "connector_lane_id": connector_lane_id,
                        "outgoing_road_id": outgoing_road_id,
                        "outgoing_lane_id": outgoing_lane_id,
                        "approach_waypoint_ids": [wid(wp) for wp in approach_wps],
                        "connector_waypoint_ids": [wid(wp) for wp in connector_wps],
                        "departure_waypoint_ids": [wid(wp) for wp in departure_wps],
                        "waypoint_ids": [wid(wp) for wp in all_wps],
                        "start_road_id": int(all_wps[0].road_id),
                        "start_lane_id": int(all_wps[0].lane_id),
                        "end_road_id": int(all_wps[-1].road_id),
                        "end_lane_id": int(all_wps[-1].lane_id),
                        "through_junction": True,
                    }
                )
                route_id += 1

        junctions[str(carla_junction_id)] = {
            "junction_id": int(carla_junction_id),
            "opendrive_junction_id": int(carla_junction_id),
            "center": center,
            "connector_count": len(connectors),
            "connectors": connectors,
        }

    return junctions, routes, extra_waypoints, diagnostics


def add_waypoint(index: Dict[str, object], wp) -> bool:
    if wp is None:
        return False
    waypoint_id = wid(wp)
    if waypoint_id in index:
        return False
    index[waypoint_id] = wp
    return True


def collect_waypoint_index(cmap, spacing: float, extra_waypoints: Sequence) -> Dict[str, object]:
    index: Dict[str, object] = {}
    for wp in cmap.generate_waypoints(spacing):
        add_waypoint(index, wp)
    for wp in extra_waypoints:
        add_waypoint(index, wp)

    for _round in range(4):
        added = False
        for wp in list(index.values()):
            for candidate in safe_waypoint_list(lambda wp=wp: wp.next(spacing)):
                added = add_waypoint(index, candidate) or added
            for candidate in safe_waypoint_list(lambda wp=wp: wp.previous(spacing)):
                added = add_waypoint(index, candidate) or added
            for lane_getter in (wp.get_left_lane, wp.get_right_lane):
                try:
                    lane_wp = lane_getter()
                except RuntimeError:
                    lane_wp = None
                if lane_wp is not None and enum_to_string(lane_wp.lane_type) == "Driving":
                    added = add_waypoint(index, lane_wp) or added
        if not added:
            break
    return index


def build_node_record(wp, spacing: float, roads: Dict[int, RoadInfo]):
    x_value, y_value, yaw_value = pose_for_wp(wp, roads)
    next_wps = safe_waypoint_list(lambda: wp.next(spacing))
    previous_wps = safe_waypoint_list(lambda: wp.previous(spacing))
    try:
        left_wp = wp.get_left_lane()
    except RuntimeError:
        left_wp = None
    try:
        right_wp = wp.get_right_lane()
    except RuntimeError:
        right_wp = None

    return {
        "id": wid(wp),
        "transform": {
            "x": round(x_value, 3),
            "y": round(y_value, 3),
            "z": round(float(wp.transform.location.z), 3),
            "yaw": round(yaw_value, 3),
        },
        "road_id": int(wp.road_id),
        "section_id": int(wp.section_id),
        "lane_id": int(wp.lane_id),
        "s": round(float(wp.s), 3),
        "lane_width": round(float(wp.lane_width), 3),
        "lane_type": enum_to_string(wp.lane_type),
        "lane_change": enum_to_string(wp.lane_change),
        "is_junction": bool(wp.is_junction),
        "next_ids": [wid(candidate) for candidate in next_wps],
        "previous_ids": [wid(candidate) for candidate in previous_wps],
        "left_lane_id": wid(left_wp) if left_wp is not None and enum_to_string(left_wp.lane_type) == "Driving" else None,
        "right_lane_id": wid(right_wp) if right_wp is not None and enum_to_string(right_wp.lane_type) == "Driving" else None,
    }


def build_legs(junctions: Dict[str, dict]) -> Dict[str, List[dict]]:
    legs = {}
    for jid, junction in junctions.items():
        seen = set()
        leg_records = []
        for connector in junction["connectors"]:
            key = (connector["incoming_road_id"], connector["incoming_lane_id"])
            if key in seen:
                continue
            seen.add(key)
            leg_records.append(
                {
                    "junction_id": connector["junction_id"],
                    "road_id": connector["incoming_road_id"],
                    "lane_id": connector["incoming_lane_id"],
                    "entry_waypoint": connector["entry"],
                }
            )
        legs[jid] = leg_records
    return legs


def selfcheck(
    routes: Sequence[dict],
    index: Dict[str, object],
    spacing: float,
    diagnostics: Sequence[dict],
    roads: Dict[int, RoadInfo],
) -> dict:
    max_gap = 0.0
    bad_segments = []
    missing_route_refs = []
    allowed_gap = max(3.0, spacing * 1.75)

    for route in routes:
        waypoint_ids = route.get("waypoint_ids", [])
        for waypoint_id in waypoint_ids:
            if waypoint_id not in index:
                missing_route_refs.append({"route": route["id"], "waypoint_id": waypoint_id})
        if any(waypoint_id not in index for waypoint_id in waypoint_ids):
            continue
        for current_id, next_id in zip(waypoint_ids, waypoint_ids[1:]):
            current = index[current_id]
            next_wp = index[next_id]
            gap = dist_xy(xy_for_wp(current, roads), xy_for_wp(next_wp, roads))
            max_gap = max(max_gap, gap)
            if gap > allowed_gap:
                bad_segments.append(
                    {
                        "route": route["id"],
                        "gap_m": round(gap, 2),
                        "between": [current_id, next_id],
                    }
                )

    turn_counts = {}
    for route in routes:
        turn_counts[route["type"]] = turn_counts.get(route["type"], 0) + 1

    return {
        "spacing_m": spacing,
        "allowed_gap_m": round(allowed_gap, 3),
        "max_consecutive_gap_m": round(max_gap, 3),
        "continuity_pass": not bad_segments and not missing_route_refs,
        "route_count": len(routes),
        "turn_counts": turn_counts,
        "missing_route_ref_count": len(missing_route_refs),
        "missing_route_refs": missing_route_refs[:20],
        "bad_segment_count": len(bad_segments),
        "bad_segments": bad_segments[:20],
        "diagnostic_count": len(diagnostics),
        "diagnostics": list(diagnostics)[:50],
    }


def write_json(path: Path, payload, indent: Optional[int] = 2) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=indent), encoding="utf-8")


def extract_one(args) -> dict:
    xodr_path = Path(args.xodr).resolve()
    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    original_xodr_text = load_xodr(xodr_path)
    xodr_text, repair_change_count = prepare_xodr(original_xodr_text, not args.no_repair)
    roads, junction_connections = parse_opendrive(xodr_text)
    started = time.perf_counter()
    cmap = connect_world(args, xodr_text)

    junctions, routes, extra_waypoints, diagnostics = build_junctions_and_routes(
        cmap,
        roads,
        junction_connections,
        args.spacing,
        args.approach_distance,
        args.departure_distance,
    )
    index = collect_waypoint_index(cmap, args.spacing, extra_waypoints)
    nodes = [build_node_record(index[key], args.spacing, roads) for key in sorted(index)]
    legs = build_legs(junctions)
    check = selfcheck(routes, index, args.spacing, diagnostics, roads)

    manifest = {
        "map_name": xodr_path.stem,
        "xodr_path": str(xodr_path),
        "xodr_sha256": sha256_text(original_xodr_text),
        "prepared_xodr_sha256": sha256_text(xodr_text),
        "repair_enabled": not args.no_repair,
        "repair_change_count": repair_change_count,
        "carla_map_name": cmap.name,
        "spacing": args.spacing,
        "waypoint_count": len(nodes),
        "junction_count": len(junctions),
        "route_count": len(routes),
        "elapsed_seconds": round(time.perf_counter() - started, 3),
    }

    write_json(out_dir / "waypoints.json", {"waypoints": nodes})
    write_json(out_dir / "junctions.json", {"junctions": list(junctions.values())})
    write_json(out_dir / "legs.json", {"legs": legs})
    write_json(out_dir / "route_candidates.json", {"routes": routes})
    write_json(out_dir / "manifest.json", manifest)
    write_json(out_dir / "roadgraph_selfcheck.json", check)

    result = {"out": str(out_dir), **manifest, **check}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return check


def carla_pythonpath_env() -> dict:
    env = os.environ.copy()
    paths = [str(CARLA_PYTHONAPI)] + CARLA_EGGS
    existing = env.get("PYTHONPATH")
    if existing:
        paths.append(existing)
    env["PYTHONPATH"] = os.pathsep.join(paths)
    return env


def write_failure_marker(out_dir: Path, result: dict) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    marker = {
        "status": "fail",
        "continuity_pass": False,
        "map": result.get("map"),
        "xodr_path": result.get("xodr_path"),
        "returncode": result.get("returncode"),
        "stdout_tail": result.get("stdout_tail"),
        "stderr_tail": result.get("stderr_tail"),
        "failed_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
    write_json(out_dir / "roadgraph_failed.json", marker)


def carla_server_ready(host: str, port: int, timeout: float) -> bool:
    code = (
        "import carla; "
        f"c=carla.Client({host!r},{int(port)}); "
        f"c.set_timeout({float(timeout)!r}); "
        "print(c.get_server_version())"
    )
    completed = subprocess.run(
        [sys.executable, "-c", code],
        cwd=str(REPO_ROOT),
        env=carla_pythonpath_env(),
        capture_output=True,
        text=True,
        timeout=max(timeout + 5.0, 10.0),
    )
    return completed.returncode == 0


def restart_carla_server(args) -> bool:
    print("    restarting CARLA server ...", flush=True)
    subprocess.run(["pkill", "-f", "CarlaUE4-Linux-Shipping"], capture_output=True, text=True)
    subprocess.run(["pkill", "-f", "CarlaUE4.sh"], capture_output=True, text=True)
    time.sleep(args.carla_restart_delay)

    log_path = Path(args.carla_log).resolve()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("ab")
    command = [
        args.carla_script,
        "-RenderOffScreen",
        "-nosound",
        f"-carla-rpc-port={args.port}",
    ]
    subprocess.Popen(
        command,
        cwd=str(Path(args.carla_script).resolve().parent),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    deadline = time.time() + args.carla_startup_timeout
    while time.time() < deadline:
        try:
            if carla_server_ready(args.host, args.port, args.carla_ready_timeout):
                print("    CARLA server ready", flush=True)
                return True
        except (subprocess.SubprocessError, OSError):
            pass
        time.sleep(2.0)
    print(f"    CARLA server did not become ready within {args.carla_startup_timeout}s", flush=True)
    return False


def batch_extract(args) -> int:
    input_dir = Path(args.input_dir).resolve()
    output_root = Path(args.output_root).resolve()
    paths = sorted(input_dir.glob("*.xodr"))
    if args.limit:
        paths = paths[: args.limit]
    output_root.mkdir(parents=True, exist_ok=True)

    script = Path(__file__).resolve()
    results = []
    started = time.perf_counter()

    if args.ensure_carla_at_start:
        try:
            ready = carla_server_ready(args.host, args.port, args.carla_ready_timeout)
        except (subprocess.SubprocessError, OSError):
            ready = False
        if not ready:
            if not restart_carla_server(args):
                raise RuntimeError("CARLA server is not ready and restart failed")

    for index, xodr_path in enumerate(paths, start=1):
        out_dir = output_root / xodr_path.stem
        selfcheck_path = out_dir / "roadgraph_selfcheck.json"
        failure_path = out_dir / "roadgraph_failed.json"
        if args.resume and selfcheck_path.is_file():
            try:
                existing = json.loads(selfcheck_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                existing = {}
            if existing.get("continuity_pass"):
                print(f"[{index}/{len(paths)}] skip pass {xodr_path.name}", flush=True)
                results.append(
                    {
                        "map": xodr_path.stem,
                        "status": "skipped",
                        "returncode": 0,
                        "continuity_pass": True,
                        "route_count": existing.get("route_count"),
                    }
                )
                continue
        if args.resume and failure_path.is_file() and not args.retry_failed:
            try:
                existing_failure = json.loads(failure_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                existing_failure = {}
            print(f"[{index}/{len(paths)}] skip failed {xodr_path.name}", flush=True)
            results.append(
                {
                    "map": xodr_path.stem,
                    "status": "skipped_failed",
                    "returncode": existing_failure.get("returncode", 1),
                    "continuity_pass": False,
                    "route_count": None,
                    "stderr_tail": existing_failure.get("stderr_tail"),
                }
            )
            continue

        command = [
            sys.executable,
            str(script),
            "--xodr",
            str(xodr_path),
            "--out",
            str(out_dir),
            "--spacing",
            str(args.spacing),
            "--host",
            args.host,
            "--port",
            str(args.port),
            "--timeout",
            str(args.timeout),
            "--vertex-distance",
            str(args.vertex_distance),
            "--max-road-length",
            str(args.max_road_length),
            "--wall-height",
            str(args.wall_height),
            "--additional-width",
            str(args.additional_width),
            "--approach-distance",
            str(args.approach_distance),
            "--departure-distance",
            str(args.departure_distance),
        ]
        if args.no_repair:
            command.append("--no-repair")

        print(f"[{index}/{len(paths)}] extract {xodr_path.name}", flush=True)
        try:
            completed = subprocess.run(
                command,
                cwd=str(REPO_ROOT),
                env=carla_pythonpath_env(),
                capture_output=True,
                text=True,
                timeout=args.per_map_timeout,
            )
            returncode = completed.returncode
            stdout_tail = "\n".join(completed.stdout.splitlines()[-20:])
            stderr_tail = "\n".join(completed.stderr.splitlines()[-20:])
        except subprocess.TimeoutExpired as exc:
            returncode = -9
            stdout_tail = exc.stdout or ""
            stderr_tail = f"timeout after {args.per_map_timeout}s"

        check = {}
        if selfcheck_path.is_file():
            try:
                check = json.loads(selfcheck_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                check = {}

        result = {
            "map": xodr_path.stem,
            "xodr_path": str(xodr_path),
            "out": str(out_dir),
            "status": "pass" if returncode == 0 else "fail",
            "returncode": returncode,
            "continuity_pass": check.get("continuity_pass"),
            "route_count": check.get("route_count"),
            "turn_counts": check.get("turn_counts"),
            "max_consecutive_gap_m": check.get("max_consecutive_gap_m"),
            "stdout_tail": stdout_tail,
            "stderr_tail": stderr_tail,
        }
        if returncode != 0:
            write_failure_marker(out_dir, result)
        results.append(result)
        print(
            f"    -> rc={returncode} pass={result['continuity_pass']} "
            f"routes={result['route_count']} max_gap={result['max_consecutive_gap_m']}",
            flush=True,
        )
        if args.fail_fast and returncode != 0:
            break
        if returncode != 0 and args.restart_carla_on_failure:
            restart_carla_server(args)

    summary = {
        "input_dir": str(input_dir),
        "output_root": str(output_root),
        "map_count": len(paths),
        "processed_count": len(results),
        "pass_count": sum(1 for item in results if item.get("returncode") == 0),
        "fail_count": sum(1 for item in results if item.get("returncode") != 0),
        "continuity_pass_count": sum(1 for item in results if item.get("continuity_pass") is True),
        "elapsed_seconds": round(time.perf_counter() - started, 3),
        "results": results,
    }
    write_json(output_root / "batch_summary.json", summary)
    print(json.dumps({k: v for k, v in summary.items() if k != "results"}, ensure_ascii=False, indent=2), flush=True)
    return 0 if summary["fail_count"] == 0 else 1


def parse_args():
    parser = argparse.ArgumentParser(description="Extract CARLA roadgraph cache from OpenDRIVE maps.")
    parser.add_argument("--xodr", help="Single .xodr map to extract.")
    parser.add_argument("--out", help="Output directory for a single map.")
    parser.add_argument("--batch", action="store_true", help="Batch extract all .xodr files under --input-dir.")
    parser.add_argument("--input-dir", default=str(REPO_ROOT / "opendrive_seed"))
    parser.add_argument("--output-root", default=str(REPO_ROOT / "outputs" / "roadgraph_map_cache"))
    parser.add_argument("--limit", type=int, default=0, help="Limit batch to the first N maps.")
    parser.add_argument("--resume", action="store_true", help="Skip maps with a passing roadgraph_selfcheck.json.")
    parser.add_argument("--retry-failed", action="store_true", help="With --resume, retry maps with roadgraph_failed.json.")
    parser.add_argument("--fail-fast", action="store_true")
    parser.add_argument("--per-map-timeout", type=float, default=120.0)
    parser.add_argument("--spacing", type=float, default=2.0)
    parser.add_argument("--approach-distance", type=float, default=30.0)
    parser.add_argument("--departure-distance", type=float, default=30.0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--vertex-distance", type=float, default=2.0)
    parser.add_argument("--max-road-length", type=float, default=500.0)
    parser.add_argument("--wall-height", type=float, default=0.0)
    parser.add_argument("--additional-width", type=float, default=0.6)
    parser.add_argument("--no-repair", action="store_true", help="Disable OpenDRIVE topology repair before CARLA import.")
    parser.add_argument("--restart-carla-on-failure", dest="restart_carla_on_failure", action="store_true", default=True)
    parser.add_argument("--no-restart-carla-on-failure", dest="restart_carla_on_failure", action="store_false")
    parser.add_argument("--ensure-carla-at-start", dest="ensure_carla_at_start", action="store_true", default=True)
    parser.add_argument("--no-ensure-carla-at-start", dest="ensure_carla_at_start", action="store_false")
    parser.add_argument("--carla-script", default="/home/server/carla/CarlaUE4.sh")
    parser.add_argument("--carla-log", default="/tmp/carla_roadgraph_batch.log")
    parser.add_argument("--carla-startup-timeout", type=float, default=90.0)
    parser.add_argument("--carla-ready-timeout", type=float, default=10.0)
    parser.add_argument("--carla-restart-delay", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.batch:
        return batch_extract(args)
    if not args.xodr:
        raise SystemExit("Provide --xodr for one map, or --batch for all maps.")
    if not args.out:
        args.out = str(Path(args.output_root) / Path(args.xodr).stem)
    check = extract_one(args)
    return 0 if check["continuity_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
