#!/usr/bin/env python3
"""Collect CARLA-derived waypoint/topology cache for generated OpenDRIVE maps."""

import argparse
import hashlib
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT_DIR = REPO_ROOT / "opendrive_seed"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "map_cache"

sys.path.insert(0, str(REPO_ROOT / "src"))

from carla_compat import ensure_carla_importable  # noqa: E402
from opendrive_repair import repair_opendrive_xml  # noqa: E402


def import_carla():
    try:
        return ensure_carla_importable()
    except ImportError as exc:
        raise RuntimeError(
            "Cannot import carla. Install carla==0.9.16 in the active environment "
            "or set CARLA_ROOT to a CARLA 0.9.16 distribution."
        ) from exc


def sha256_text(value):
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def read_and_prepare_opendrive(path):
    data = path.read_text(encoding="utf-8")
    index = data.find("<OpenDRIVE")
    if index == -1:
        raise ValueError(f"{path} does not contain an <OpenDRIVE> root")
    data = data[index:]

    root = ET.fromstring(data)
    header = root.find("header")
    if header is None:
        header = ET.Element("header")
        root.insert(0, header)
    if header.find("geoReference") is None:
        geo_ref = ET.Element("geoReference")
        geo_ref.text = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs"
        header.append(geo_ref)

    prepared, _ = repair_opendrive_xml(ET.tostring(root, encoding="unicode"))
    return prepared


def resolve_xodr(case_or_path, input_dir):
    raw_path = Path(case_or_path)
    if raw_path.is_file():
        return raw_path.resolve()

    candidates = sorted(input_dir.glob("*.xodr"))
    query = "".join(ch.lower() for ch in case_or_path if ch.isalnum())
    matches = []
    for path in candidates:
        stem = path.stem
        normalized = "".join(ch.lower() for ch in stem if ch.isalnum())
        prefix = stem.split("_", 1)[0]
        if query.isdigit() and prefix.isdigit() and int(query) == int(prefix):
            matches.append(path)
        elif query and query in normalized:
            matches.append(path)

    if not matches:
        raise FileNotFoundError(f"No .xodr matched {case_or_path!r} under {input_dir}")
    if len(matches) > 1:
        names = "\n  ".join(str(path) for path in matches)
        raise ValueError(f"Multiple .xodr files matched {case_or_path!r}:\n  {names}")
    return matches[0].resolve()


def transform_to_dict(transform):
    return {
        "x": transform.location.x,
        "y": transform.location.y,
        "z": transform.location.z,
        "pitch": transform.rotation.pitch,
        "yaw": transform.rotation.yaw,
        "roll": transform.rotation.roll,
    }


def waypoint_key(waypoint):
    return (
        int(waypoint.road_id),
        int(waypoint.section_id),
        int(waypoint.lane_id),
        round(float(waypoint.s), 2),
    )


def waypoint_id(waypoint):
    road_id, section_id, lane_id, s_value = waypoint_key(waypoint)
    return f"r{road_id}:sec{section_id}:l{lane_id}:s{s_value:.2f}"


def safe_waypoint_id(waypoint):
    if waypoint is None:
        return None
    return waypoint_id(waypoint)


def enum_to_string(value):
    text = str(value)
    return text.split(".")[-1] if "." in text else text


def collect_waypoints(carla_map, spacing):
    waypoints = carla_map.generate_waypoints(spacing)
    waypoints = sorted(waypoints, key=lambda wp: waypoint_key(wp))
    waypoint_by_id = {waypoint_id(wp): wp for wp in waypoints}

    records = []
    for wp in waypoints:
        left_wp = wp.get_left_lane()
        right_wp = wp.get_right_lane()
        next_wps = wp.next(spacing)
        prev_wps = wp.previous(spacing)
        record = {
            "id": waypoint_id(wp),
            "transform": transform_to_dict(wp.transform),
            "road_id": wp.road_id,
            "section_id": wp.section_id,
            "lane_id": wp.lane_id,
            "s": wp.s,
            "lane_width": wp.lane_width,
            "lane_type": enum_to_string(wp.lane_type),
            "lane_change": enum_to_string(wp.lane_change),
            "is_junction": bool(wp.is_junction),
            "next_ids": [safe_waypoint_id(candidate) for candidate in next_wps],
            "previous_ids": [safe_waypoint_id(candidate) for candidate in prev_wps],
            "left_lane_id": safe_waypoint_id(left_wp),
            "right_lane_id": safe_waypoint_id(right_wp),
        }
        records.append(record)
    return waypoints, waypoint_by_id, records


def collect_topology(carla_map):
    edges = []
    for start_wp, end_wp in carla_map.get_topology():
        edges.append(
            {
                "start_id": waypoint_id(start_wp),
                "end_id": waypoint_id(end_wp),
                "start": transform_to_dict(start_wp.transform),
                "end": transform_to_dict(end_wp.transform),
                "start_road_id": start_wp.road_id,
                "end_road_id": end_wp.road_id,
                "start_lane_id": start_wp.lane_id,
                "end_lane_id": end_wp.lane_id,
            }
        )
    return edges


def collect_spawn_points(carla_map, waypoints, max_derived=200):
    spawn_points = [
        {"id": f"carla_spawn_{index:04d}", "transform": transform_to_dict(transform)}
        for index, transform in enumerate(carla_map.get_spawn_points())
    ]

    derived = []
    seen = set()
    for wp in waypoints:
        if enum_to_string(wp.lane_type) != "Driving" or wp.is_junction:
            continue
        lane_key = (wp.road_id, wp.section_id, wp.lane_id, int(wp.s // 8.0))
        if lane_key in seen:
            continue
        seen.add(lane_key)
        transform = wp.transform
        transform.location.z += 0.2
        derived.append(
            {
                "id": f"derived_spawn_{len(derived):04d}",
                "waypoint_id": waypoint_id(wp),
                "transform": transform_to_dict(transform),
                "road_id": wp.road_id,
                "lane_id": wp.lane_id,
                "s": wp.s,
            }
        )
        if len(derived) >= max_derived:
            break

    return {"carla_spawn_points": spawn_points, "derived_spawn_points": derived}


def collect_junctions(waypoints):
    junctions = {}
    for wp in waypoints:
        if not wp.is_junction:
            continue
        junction_id = "unknown"
        try:
            junction = wp.get_junction()
            if junction is not None:
                junction_id = str(junction.id)
        except RuntimeError:
            pass

        entry = junctions.setdefault(
            junction_id,
            {
                "id": junction_id,
                "waypoint_ids": [],
                "road_ids": set(),
                "lane_ids": set(),
            },
        )
        entry["waypoint_ids"].append(waypoint_id(wp))
        entry["road_ids"].add(wp.road_id)
        entry["lane_ids"].add(wp.lane_id)

    output = []
    for entry in junctions.values():
        output.append(
            {
                "id": entry["id"],
                "waypoint_count": len(entry["waypoint_ids"]),
                "road_ids": sorted(entry["road_ids"]),
                "lane_ids": sorted(entry["lane_ids"]),
                "sample_waypoint_ids": entry["waypoint_ids"][:50],
            }
        )
    return sorted(output, key=lambda item: item["id"])


def yaw_delta_degrees(start_yaw, end_yaw):
    return (end_yaw - start_yaw + 180.0) % 360.0 - 180.0


def classify_route(route):
    if len(route) < 2:
        return "short"
    delta = yaw_delta_degrees(route[0].transform.rotation.yaw, route[-1].transform.rotation.yaw)
    if abs(delta) < 25.0:
        return "straight"
    return "left_turn" if delta > 0.0 else "right_turn"


def build_route_candidates(waypoints, spacing, max_routes, min_points):
    candidates = []
    starts = [
        wp for wp in waypoints
        if enum_to_string(wp.lane_type) == "Driving" and not wp.is_junction
    ]
    starts = sorted(starts, key=lambda wp: (wp.road_id, wp.section_id, wp.lane_id, wp.s))

    stride = max(1, len(starts) // max_routes) if starts else 1
    for start_wp in starts[::stride]:
        route = [start_wp]
        current = start_wp
        visited = {waypoint_id(start_wp)}
        for _ in range(80):
            next_wps = current.next(spacing)
            if not next_wps:
                break
            next_wp = sorted(next_wps, key=lambda wp: abs(yaw_delta_degrees(current.transform.rotation.yaw, wp.transform.rotation.yaw)))[0]
            next_id = waypoint_id(next_wp)
            if next_id in visited:
                break
            route.append(next_wp)
            visited.add(next_id)
            current = next_wp
        if len(route) < min_points:
            continue
        candidates.append(
            {
                "id": f"route_{len(candidates):04d}",
                "type": classify_route(route),
                "waypoint_ids": [waypoint_id(wp) for wp in route],
                "start": transform_to_dict(route[0].transform),
                "end": transform_to_dict(route[-1].transform),
                "point_count": len(route),
                "approx_length": round((len(route) - 1) * spacing, 2),
                "start_road_id": route[0].road_id,
                "start_lane_id": route[0].lane_id,
                "end_road_id": route[-1].road_id,
                "end_lane_id": route[-1].lane_id,
            }
        )
        if len(candidates) >= max_routes:
            break
    return candidates


def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def collect_map_cache(args):
    carla = import_carla()
    input_dir = Path(args.input_dir).resolve()
    xodr_path = resolve_xodr(args.case, input_dir)
    output_dir = Path(args.output_root).resolve() / xodr_path.stem
    output_dir.mkdir(parents=True, exist_ok=True)

    prepared_xodr = read_and_prepare_opendrive(xodr_path)
    started = time.perf_counter()
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    base_manifest = {
        "map_name": xodr_path.stem,
        "xodr_path": str(xodr_path),
        "xodr_sha256": sha256_text(xodr_path.read_text(encoding="utf-8")),
        "prepared_xodr_sha256": sha256_text(prepared_xodr),
        "spacing": args.spacing,
        "generation_parameters": {
            "vertex_distance": args.vertex_distance,
            "wall_height": args.wall_height,
            "additional_width": args.additional_width,
            "smooth_junctions": True,
            "enable_mesh_visibility": True,
        },
    }

    try:
        world = client.generate_opendrive_world(
            prepared_xodr,
            carla.OpendriveGenerationParameters(
                vertex_distance=args.vertex_distance,
                wall_height=args.wall_height,
                additional_width=args.additional_width,
                smooth_junctions=True,
                enable_mesh_visibility=True,
            ),
        )
    except RuntimeError as exc:
        manifest = {
            **base_manifest,
            "status": "fail",
            "stage": "generate_opendrive_world",
            "error": str(exc),
            "elapsed_seconds": round(time.perf_counter() - started, 3),
        }
        write_json(output_dir / "manifest.json", manifest)
        print(f"Map cache failed: {output_dir}")
        print(json.dumps(manifest, indent=2, ensure_ascii=False))
        raise

    carla_map = world.get_map()

    waypoints, _, waypoint_records = collect_waypoints(carla_map, args.spacing)
    topology = collect_topology(carla_map)
    spawn_points = collect_spawn_points(carla_map, waypoints, args.max_derived_spawn_points)
    junctions = collect_junctions(waypoints)
    route_candidates = build_route_candidates(waypoints, args.spacing, args.max_routes, args.min_route_points)

    manifest = {
        **base_manifest,
        "status": "pass",
        "carla_map_name": carla_map.name,
        "waypoint_count": len(waypoint_records),
        "topology_edge_count": len(topology),
        "carla_spawn_count": len(spawn_points["carla_spawn_points"]),
        "derived_spawn_count": len(spawn_points["derived_spawn_points"]),
        "junction_count": len(junctions),
        "route_candidate_count": len(route_candidates),
        "elapsed_seconds": round(time.perf_counter() - started, 3),
    }

    write_json(output_dir / "waypoints.json", {"waypoints": waypoint_records})
    write_json(output_dir / "topology.json", {"edges": topology})
    write_json(output_dir / "spawn_points.json", spawn_points)
    write_json(output_dir / "junctions.json", {"junctions": junctions})
    write_json(output_dir / "route_candidates.json", {"routes": route_candidates})
    write_json(output_dir / "manifest.json", manifest)

    print(f"Map cache written: {output_dir}")
    print(json.dumps(manifest, indent=2, ensure_ascii=False))
    return manifest


def parse_args():
    parser = argparse.ArgumentParser(description="Collect CARLA-derived waypoint cache for one OpenDRIVE map.")
    parser.add_argument("case", nargs="?", default="001", help="Case id/name or direct .xodr path. Default: 001")
    parser.add_argument("--input-dir", default=str(DEFAULT_INPUT_DIR), help="Directory containing .xodr maps.")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT), help="Output root for map cache.")
    parser.add_argument("--host", default="localhost", help="CARLA host.")
    parser.add_argument("--port", type=int, default=2000, help="CARLA port.")
    parser.add_argument("--timeout", type=float, default=30.0, help="CARLA client timeout in seconds.")
    parser.add_argument("--spacing", type=float, default=2.0, help="Waypoint sampling spacing in meters.")
    parser.add_argument("--vertex-distance", type=float, default=2.0, help="OpenDRIVE mesh vertex distance.")
    parser.add_argument("--wall-height", type=float, default=1.0, help="OpenDRIVE wall height.")
    parser.add_argument("--additional-width", type=float, default=0.6, help="OpenDRIVE additional road width.")
    parser.add_argument("--max-derived-spawn-points", type=int, default=200, help="Max derived spawn points.")
    parser.add_argument("--max-routes", type=int, default=80, help="Max route candidates.")
    parser.add_argument("--min-route-points", type=int, default=8, help="Minimum points per route candidate.")
    return parser.parse_args()


def main():
    args = parse_args()
    collect_map_cache(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
