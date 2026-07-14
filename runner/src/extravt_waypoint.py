#!/usr/bin/env python3
"""
Extract comprehensive map data from CARLA Town01-05.

Outputs two files per town:
  1. waypoint/{Town}_waypoints.json   — Driving-lane waypoints for spawn_selector
  2. carla_map_data/{Town}_map.json   — Full map data (junctions, traffic lights, etc.)

Usage:
  python tools/extract_carla_map.py
  python tools/extract_carla_map.py --towns Town05
  python tools/extract_carla_map.py --towns Town01,Town03 --interval 1.0
  python tools/extract_carla_map.py --host 192.168.1.100 --port 2000
"""

import argparse
import json
import math
import sys
import time
from collections import defaultdict
from pathlib import Path

try:
    import carla
except ImportError:
    print("ERROR: carla package not found.")
    print("Install the CARLA Python API egg matching your CARLA version, e.g.:")
    print("  pip install carla==0.9.16")
    sys.exit(1)


# ── Helpers ──────────────────────────────────────────────────────────────────

def _transform_to_dict(t: carla.Transform) -> dict:
    loc = t.location
    rot = t.rotation
    return {
        "x": round(loc.x, 3),
        "y": round(loc.y, 3),
        "z": round(loc.z, 3),
        "pitch": round(rot.pitch, 2),
        "yaw": round(rot.yaw, 2),
        "roll": round(rot.roll, 2),
    }


def _loc_to_dict(loc: carla.Location) -> dict:
    return {
        "x": round(loc.x, 3),
        "y": round(loc.y, 3),
        "z": round(loc.z, 3),
    }


def _lane_type_name(lt) -> str:
    """Convert carla.LaneType enum to string."""
    mapping = {
        carla.LaneType.Driving: "Driving",
        carla.LaneType.Sidewalk: "Sidewalk",
        carla.LaneType.Shoulder: "Shoulder",
        carla.LaneType.Parking: "Parking",
        carla.LaneType.Bidirectional: "Bidirectional",
        carla.LaneType.Median: "Median",
        carla.LaneType.Border: "Border",
        carla.LaneType.Restricted: "Restricted",
        carla.LaneType.Stop: "Stop",
        carla.LaneType.NONE: "None",
        carla.LaneType.Any: "Any",
    }
    return mapping.get(lt, str(lt))


def _wp_to_compact(wp) -> dict:
    """Waypoint to compact dict with road/lane info."""
    t = wp.transform
    jid = wp.get_junction().id if wp.is_junction else -1
    right = wp.get_right_lane()
    left = wp.get_left_lane()
    return {
        "road_id": wp.road_id,
        "section_id": wp.section_id,
        "lane_id": wp.lane_id,
        "s": round(wp.s, 2),
        "x": round(t.location.x, 3),
        "y": round(t.location.y, 3),
        "z": round(t.location.z, 3),
        "yaw": round(t.rotation.yaw, 2),
        "lane_type": _lane_type_name(wp.lane_type),
        "lane_width": round(wp.lane_width, 2),
        "is_junction": wp.is_junction,
        "junction_id": jid,
        "has_right_lane": right is not None and right.lane_type == carla.LaneType.Driving,
        "has_left_lane": left is not None and left.lane_type == carla.LaneType.Driving,
    }


# ── Extraction Functions ─────────────────────────────────────────────────────

def extract_spawn_points(carla_map) -> list:
    return [_transform_to_dict(sp) for sp in carla_map.get_spawn_points()]


def extract_waypoints(carla_map, interval: float) -> tuple:
    """
    Generate waypoints at given interval for ALL lane types.
    Returns (all_waypoints, driving_waypoints, sidewalk_waypoints).
    """
    all_wps = []
    driving_wps = []
    sidewalk_wps = []

    # Driving lanes
    for wp in carla_map.generate_waypoints(interval):
        entry = _wp_to_compact(wp)
        all_wps.append(entry)
        if wp.lane_type == carla.LaneType.Driving:
            driving_wps.append(entry)

    # Sidewalk — generate separately since generate_waypoints only does Driving
    # We get sidewalks by checking adjacent lanes of driving waypoints
    seen_sidewalk = set()
    for wp in carla_map.generate_waypoints(interval):
        for side_wp in [wp.get_right_lane(), wp.get_left_lane()]:
            if side_wp and side_wp.lane_type == carla.LaneType.Sidewalk:
                key = (side_wp.road_id, side_wp.lane_id, round(side_wp.s, 1))
                if key not in seen_sidewalk:
                    seen_sidewalk.add(key)
                    entry = _wp_to_compact(side_wp)
                    sidewalk_wps.append(entry)
                    all_wps.append(entry)

    return all_wps, driving_wps, sidewalk_wps


def extract_junctions(carla_map, interval: float) -> list:
    """Extract junction info with bounding boxes from waypoints."""
    junction_wps = defaultdict(list)

    for wp in carla_map.generate_waypoints(interval):
        if wp.is_junction:
            junc = wp.get_junction()
            jid = junc.id
            t = wp.transform
            junction_wps[jid].append({
                "road_id": wp.road_id,
                "lane_id": wp.lane_id,
                "x": round(t.location.x, 3),
                "y": round(t.location.y, 3),
                "z": round(t.location.z, 3),
                "yaw": round(t.rotation.yaw, 2),
            })

    junctions = []
    for jid, wps in sorted(junction_wps.items()):
        xs = [w["x"] for w in wps]
        ys = [w["y"] for w in wps]
        junctions.append({
            "id": jid,
            "bounding_box": {
                "x_min": round(min(xs), 3),
                "x_max": round(max(xs), 3),
                "y_min": round(min(ys), 3),
                "y_max": round(max(ys), 3),
            },
            "num_waypoints": len(wps),
            "waypoints": wps,
        })

    return junctions


def extract_topology(carla_map) -> list:
    """Extract road network topology as (start_wp, end_wp) pairs."""
    topo = []
    for wp_start, wp_end in carla_map.get_topology():
        ts = wp_start.transform
        te = wp_end.transform
        topo.append({
            "from": {
                "road_id": wp_start.road_id,
                "lane_id": wp_start.lane_id,
                "x": round(ts.location.x, 3),
                "y": round(ts.location.y, 3),
                "z": round(ts.location.z, 3),
            },
            "to": {
                "road_id": wp_end.road_id,
                "lane_id": wp_end.lane_id,
                "x": round(te.location.x, 3),
                "y": round(te.location.y, 3),
                "z": round(te.location.z, 3),
            },
        })
    return topo


def extract_traffic_lights(world) -> list:
    """Extract traffic light positions and their affected lane waypoints."""
    tl_actors = world.get_actors().filter("traffic.traffic_light*")
    result = []
    for tl in tl_actors:
        t = tl.get_transform()
        entry = {
            "id": tl.id,
            "x": round(t.location.x, 3),
            "y": round(t.location.y, 3),
            "z": round(t.location.z, 3),
            "yaw": round(t.rotation.yaw, 2),
            "state": str(tl.state).split(".")[-1],  # e.g. "Green"
        }

        # Get affected lane waypoints
        affected = []
        try:
            # get_affected_lane_waypoints returns list of (waypoint, road_option)
            for group in tl.get_affected_lane_waypoints():
                if isinstance(group, tuple):
                    wp = group[0]
                else:
                    wp = group
                wt = wp.transform
                affected.append({
                    "road_id": wp.road_id,
                    "lane_id": wp.lane_id,
                    "x": round(wt.location.x, 3),
                    "y": round(wt.location.y, 3),
                    "z": round(wt.location.z, 3),
                    "yaw": round(wt.rotation.yaw, 2),
                })
        except Exception:
            # Some CARLA versions have different API for this
            pass

        # Also try get_stop_waypoints for the stop line positions
        try:
            for wp in tl.get_stop_waypoints():
                wt = wp.transform
                affected.append({
                    "road_id": wp.road_id,
                    "lane_id": wp.lane_id,
                    "x": round(wt.location.x, 3),
                    "y": round(wt.location.y, 3),
                    "z": round(wt.location.z, 3),
                    "yaw": round(wt.rotation.yaw, 2),
                    "is_stop_line": True,
                })
        except Exception:
            pass

        entry["affected_waypoints"] = affected
        result.append(entry)

    return result


def extract_traffic_signs(world) -> list:
    """Extract stop signs and other traffic signs."""
    result = []

    # Stop signs
    for sign in world.get_actors().filter("traffic.stop"):
        t = sign.get_transform()
        result.append({
            "id": sign.id,
            "type": "stop",
            "x": round(t.location.x, 3),
            "y": round(t.location.y, 3),
            "z": round(t.location.z, 3),
            "yaw": round(t.rotation.yaw, 2),
        })

    # Speed limit signs
    for sign in world.get_actors().filter("traffic.speed_limit*"):
        t = sign.get_transform()
        type_id = sign.type_id  # e.g. "traffic.speed_limit.30"
        limit = type_id.split(".")[-1] if "." in type_id else "unknown"
        result.append({
            "id": sign.id,
            "type": f"speed_limit_{limit}",
            "x": round(t.location.x, 3),
            "y": round(t.location.y, 3),
            "z": round(t.location.z, 3),
            "yaw": round(t.rotation.yaw, 2),
        })

    # Yield signs
    for sign in world.get_actors().filter("traffic.yield"):
        t = sign.get_transform()
        result.append({
            "id": sign.id,
            "type": "yield",
            "x": round(t.location.x, 3),
            "y": round(t.location.y, 3),
            "z": round(t.location.z, 3),
            "yaw": round(t.rotation.yaw, 2),
        })

    return result


def extract_crosswalks(carla_map) -> list:
    """
    Extract crosswalk polygons from map.
    get_crosswalks() returns a flat list of carla.Location vertices.
    Every consecutive group of vertices (separated by z jumps or restarting
    patterns) forms one crosswalk polygon.
    """
    try:
        raw_vertices = carla_map.get_crosswalks()
    except Exception:
        return []

    if not raw_vertices:
        return []

    # Group vertices into crosswalk polygons.
    # CARLA returns vertices in groups — each crosswalk is a closed polygon.
    # We detect polygon boundaries by checking for large jumps between consecutive vertices.
    crosswalks = []
    current_polygon = []
    prev = None

    for v in raw_vertices:
        pt = {"x": round(v.x, 3), "y": round(v.y, 3), "z": round(v.z, 3)}

        if prev is not None:
            dist = math.sqrt((v.x - prev.x) ** 2 + (v.y - prev.y) ** 2)
            # New polygon if jump > 10m (crosswalks are typically < 10m wide)
            if dist > 10.0:
                if len(current_polygon) >= 3:
                    crosswalks.append({"vertices": current_polygon})
                current_polygon = []

        current_polygon.append(pt)
        prev = v

    # Don't forget the last polygon
    if len(current_polygon) >= 3:
        crosswalks.append({"vertices": current_polygon})

    return crosswalks


# ── Main Extraction Pipeline ────────────────────────────────────────────────

def extract_town(client, town: str, interval: float, output_dir: Path, waypoint_dir: Path):
    """Extract all map data for a single town."""
    print(f"\n{'='*60}")
    print(f"  Loading {town}...")
    print(f"{'='*60}")

    client.load_world(town)
    # Wait for world to be ready
    world = client.get_world()
    for _ in range(20):
        world.tick()
        time.sleep(0.1)
    world = client.get_world()
    carla_map = world.get_map()

    print(f"  Map loaded: {carla_map.name}")

    # 1. Spawn points
    print("  [1/7] Extracting spawn points...")
    spawn_points = extract_spawn_points(carla_map)
    print(f"         {len(spawn_points)} spawn points")

    # 2. Waypoints (all types)
    print(f"  [2/7] Generating waypoints (interval={interval}m)...")
    all_wps, driving_wps, sidewalk_wps = extract_waypoints(carla_map, interval)
    print(f"         {len(driving_wps)} driving, {len(sidewalk_wps)} sidewalk, {len(all_wps)} total")

    # 3. Junctions
    print("  [3/7] Extracting junctions...")
    junctions = extract_junctions(carla_map, interval)
    print(f"         {len(junctions)} junctions")

    # 4. Topology
    print("  [4/7] Extracting topology...")
    topology = extract_topology(carla_map)
    print(f"         {len(topology)} road segments")

    # 5. Traffic lights
    print("  [5/7] Extracting traffic lights...")
    traffic_lights = extract_traffic_lights(world)
    print(f"         {len(traffic_lights)} traffic lights")

    # 6. Traffic signs
    print("  [6/7] Extracting traffic signs...")
    traffic_signs = extract_traffic_signs(world)
    print(f"         {len(traffic_signs)} traffic signs")

    # 7. Crosswalks
    print("  [7/7] Extracting crosswalks...")
    crosswalks = extract_crosswalks(carla_map)
    print(f"         {len(crosswalks)} crosswalk polygons")

    # ── Save full map data ───────────────────────────────────────────────
    full_data = {
        "town": town,
        "map_name": carla_map.name,
        "waypoint_interval": interval,
        "spawn_points": spawn_points,
        "junctions": junctions,
        "waypoints": all_wps,
        "traffic_lights": traffic_lights,
        "traffic_signs": traffic_signs,
        "topology": topology,
        "sidewalk_waypoints": sidewalk_wps,
        "crosswalks": crosswalks,
        "stats": {
            "num_spawn_points": len(spawn_points),
            "num_driving_waypoints": len(driving_wps),
            "num_sidewalk_waypoints": len(sidewalk_wps),
            "num_junctions": len(junctions),
            "num_topology_segments": len(topology),
            "num_traffic_lights": len(traffic_lights),
            "num_traffic_signs": len(traffic_signs),
            "num_crosswalks": len(crosswalks),
        },
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    full_path = output_dir / f"{town}_map.json"
    full_path.write_text(json.dumps(full_data, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n  Saved full map:  {full_path}")
    print(f"    File size: {full_path.stat().st_size / 1024 / 1024:.1f} MB")

    # ── Save driving waypoints (spawn_selector compatible) ───────────────
    waypoint_dir.mkdir(parents=True, exist_ok=True)
    wp_data = {
        "waypoints": [
            {
                "x": w["x"],
                "y": w["y"],
                "z": w["z"],
                "h": w["yaw"],  # spawn_selector expects "h" not "yaw"
            }
            for w in driving_wps
        ]
    }
    wp_path = waypoint_dir / f"{town}_waypoints.json"
    wp_path.write_text(json.dumps(wp_data, indent=2), encoding="utf-8")
    print(f"  Saved waypoints: {wp_path}")
    print(f"    {len(wp_data['waypoints'])} driving waypoints")

    return full_data["stats"]


# ── CLI ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Extract map data from CARLA Town01-05 for the ADS testing pipeline.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python tools/extract_carla_map.py                          # All towns, default settings
  python tools/extract_carla_map.py --towns Town05           # Single town
  python tools/extract_carla_map.py --towns Town01,Town03    # Multiple towns
  python tools/extract_carla_map.py --interval 1.0           # Higher resolution
  python tools/extract_carla_map.py --host 192.168.1.100     # Remote CARLA server

Requires:
  - CARLA server running (default: localhost:2000)
  - carla Python package installed (pip install carla==0.9.16)
""",
    )
    parser.add_argument("--host", default="localhost", help="CARLA server host (default: localhost)")
    parser.add_argument("--port", type=int, default=2000, help="CARLA server port (default: 2000)")
    parser.add_argument(
        "--towns",
        default="Town01,Town02,Town03,Town04,Town05",
        help="Comma-separated town names (default: Town01-05)",
    )
    parser.add_argument("--interval", type=float, default=2.0, help="Waypoint sampling interval in meters (default: 2.0)")
    parser.add_argument("--output-dir", default="carla_map_data", help="Output directory for full map JSONs")
    parser.add_argument("--waypoint-dir", default="waypoint", help="Output directory for spawn_selector waypoints")
    args = parser.parse_args()

    towns = [t.strip() for t in args.towns.split(",") if t.strip()]
    output_dir = Path(args.output_dir)
    waypoint_dir = Path(args.waypoint_dir)

    print("=" * 60)
    print("  CARLA Map Data Extractor")
    print("=" * 60)
    print(f"  Server:   {args.host}:{args.port}")
    print(f"  Towns:    {', '.join(towns)}")
    print(f"  Interval: {args.interval}m")
    print(f"  Output:   {output_dir}/")
    print(f"  Waypoints:{waypoint_dir}/")
    print("=" * 60)

    # Connect to CARLA
    print(f"\nConnecting to CARLA at {args.host}:{args.port}...")
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(30.0)
        server_version = client.get_server_version()
        client_version = client.get_client_version()
        print(f"  Connected! Server: {server_version}, Client: {client_version}")
    except Exception as e:
        print(f"ERROR: Failed to connect to CARLA: {e}")
        print("Make sure the CARLA server is running.")
        sys.exit(1)

    # Extract each town
    all_stats = {}
    success = 0
    failed = 0

    for town in towns:
        try:
            stats = extract_town(client, town, args.interval, output_dir, waypoint_dir)
            all_stats[town] = stats
            success += 1
        except Exception as e:
            print(f"\n  ERROR extracting {town}: {e}")
            failed += 1
            continue

    # Summary
    print(f"\n{'='*60}")
    print("  Extraction Complete")
    print(f"{'='*60}")
    print(f"  Success: {success}/{len(towns)}, Failed: {failed}/{len(towns)}")

    if all_stats:
        print(f"\n  {'Town':<10} {'Driving':>8} {'Sidewalk':>9} {'Junctions':>10} {'TL':>5} {'Signs':>6} {'Crosswalks':>11}")
        print(f"  {'-'*10} {'-'*8} {'-'*9} {'-'*10} {'-'*5} {'-'*6} {'-'*11}")
        for town, s in all_stats.items():
            print(
                f"  {town:<10} {s['num_driving_waypoints']:>8} {s['num_sidewalk_waypoints']:>9} "
                f"{s['num_junctions']:>10} {s['num_traffic_lights']:>5} {s['num_traffic_signs']:>6} "
                f"{s['num_crosswalks']:>11}"
            )

    print(f"\nFiles saved to:")
    print(f"  Full map data:  {output_dir}/")
    print(f"  Waypoints:      {waypoint_dir}/")
    print("=" * 60)


if __name__ == "__main__":
    main()
