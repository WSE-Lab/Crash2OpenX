#!/usr/bin/env python3
"""Build road-only OpenDRIVE and HTML previews from a minimal road seed."""

from __future__ import annotations

import argparse
import html
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

from tools.replay_scene_tools import write_generated_opendrive
from tools.visualization_qa import _geometry_samples, xodr_lane_center_segments


ALLOWED_TOPOLOGIES = {
    "straight",
    "curve",
    "cross_intersection",
    "t_junction",
    "y_junction",
    "merge",
    "fork",
}
ALLOWED_ROAD_TYPES = {
    "town",
    "lowSpeed",
    "rural",
    "motorway",
    "townArterial",
    "townCollector",
    "townLocal",
}
# schema v2: lanes is direction-aware. forward >= 1 (ego travel direction),
# backward 0..5 (0 = one-way). center_line controls whether the centerline can
# be crossed to borrow the oncoming lane (e.g. to pass an obstacle).
ALLOWED_FORWARD_LANES = {1, 2, 3, 4, 5}
ALLOWED_BACKWARD_LANES = {0, 1, 2, 3, 4, 5}
ALLOWED_CENTER_LINES = {"solid", "broken"}

ROAD_TYPE_DEFAULTS = {
    # 2026-06-27: road_length_m roughly doubled across the board after the
    # 18-medoid CARLA batch showed PCLA running out of runway. Heuristic:
    # length ≥ cruise_speed × 10 s decision window + ~50 m of scene-geometry
    # budget (ego_s margin + gap=25 + NPC clearance). Old values (70 / 80 /
    # 100 / 110 / 140 / 250 m) left 12 of 18 cases unable to even reach the
    # ACT_STOP_DIST=200 m stop trigger — scenarios were all timing out at
    # max_seconds=60 with route_completion < 25%. See paper/sprint3_runtime_fixes.md
    # § "B1: road_length bump (2026-06-27)" for the per-case capacity table.
    "town":          {"lane_width_m": 3.5, "road_length_m": 200.0, "junction_radius_m": 30.0, "speed_mps": 13.4},
    "lowSpeed":      {"lane_width_m": 3.2, "road_length_m": 140.0, "junction_radius_m": 20.0, "speed_mps":  8.0},
    "rural":         {"lane_width_m": 3.5, "road_length_m": 280.0, "junction_radius_m": 35.0, "speed_mps": 22.0},
    "motorway":      {"lane_width_m": 3.7, "road_length_m": 500.0, "junction_radius_m": 80.0, "speed_mps": 30.0},
    "townArterial":  {"lane_width_m": 3.5, "road_length_m": 280.0, "junction_radius_m": 40.0, "speed_mps": 16.7},
    "townCollector": {"lane_width_m": 3.5, "road_length_m": 220.0, "junction_radius_m": 32.0, "speed_mps": 13.4},
    "townLocal":     {"lane_width_m": 3.3, "road_length_m": 160.0, "junction_radius_m": 24.0, "speed_mps": 11.2},
}


def read_seed(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    road = data.get("road", data)
    if not isinstance(road, dict):
        raise ValueError("road seed must be an object or contain a road object")
    return {"road": road}


def validate_seed(seed: dict[str, Any]) -> list[str]:
    road = seed.get("road")
    errors: list[str] = []
    if not isinstance(road, dict):
        return ["road must be an object"]

    unknown = sorted(set(road) - {"topology", "type", "lanes", "center_line", "parking"})
    if unknown:
        errors.append(f"road contains unsupported fields: {', '.join(unknown)}")

    topology = road.get("topology")
    if topology not in ALLOWED_TOPOLOGIES:
        errors.append(f"road.topology must be one of {sorted(ALLOWED_TOPOLOGIES)}")

    road_type = road.get("type")
    if road_type not in ALLOWED_ROAD_TYPES:
        errors.append(f"road.type must be one of {sorted(ALLOWED_ROAD_TYPES)}")

    lanes = road.get("lanes")
    if not isinstance(lanes, dict):
        errors.append("road.lanes must be an object {forward, backward} (schema v2)")
    else:
        extra = sorted(set(lanes) - {"forward", "backward"})
        if extra:
            errors.append(f"road.lanes has unsupported keys: {', '.join(extra)}")
        if lanes.get("forward") not in ALLOWED_FORWARD_LANES:
            errors.append("road.lanes.forward must be 1, 2, 3, 4, or 5")
        if lanes.get("backward") not in ALLOWED_BACKWARD_LANES:
            errors.append("road.lanes.backward must be 0, 1, 2, 3, 4, or 5")

    center_line = road.get("center_line")
    if center_line not in ALLOWED_CENTER_LINES:
        errors.append(f"road.center_line must be one of {sorted(ALLOWED_CENTER_LINES)}")

    from tools.road_parking import normalize_parking
    try:
        normalize_parking(road)
    except ValueError as exc:
        errors.append(str(exc))

    return errors


def road_seed_to_trace(seed: dict[str, Any], xodr_path: Path) -> dict[str, Any]:
    road = seed["road"]
    topology = road["topology"]
    road_type = road["type"]
    # schema v2: lanes is {forward, backward}. Tolerate a legacy int (treated as
    # a symmetric two-way road) so previously generated seeds still compile.
    raw_lanes = road["lanes"]
    if isinstance(raw_lanes, dict):
        forward = int(raw_lanes.get("forward", 1))
        backward = int(raw_lanes.get("backward", forward))
    else:
        forward = backward = int(raw_lanes)
    center_line = road.get("center_line", "broken")
    defaults = ROAD_TYPE_DEFAULTS[road_type]

    if topology == "cross_intersection":
        layout = "common_cross_junction"
    elif topology == "straight":
        layout = "straight_corridor"
    elif topology == "curve":
        layout = "curved_corridor"
    elif topology == "t_junction":
        layout = "t_junction"
    elif topology == "y_junction":
        layout = "y_junction"
    elif topology == "merge":
        layout = "ramp_merge"
    elif topology == "fork":
        layout = "ramp_fork"
    else:
        raise NotImplementedError(f"road-only compiler does not yet implement topology: {topology}")

    return {
        "metadata": {"name": xodr_path.stem},
        "road_generation": {
            "mode": "generated_open_drive",
            "road_type": road_type,
            "junction_type": topology if topology in {"cross_intersection", "t_junction", "y_junction", "merge", "fork"} else "none",
            "layout": layout,
            "lane_width_m": defaults["lane_width_m"],
            "lanes_forward": forward,
            "lanes_backward": backward,
            "center_line": center_line,
            "parking": road.get('parking', {}),
            "lanes_per_direction": forward,  # back-compat shim for callers reading the old field
            "road_length": defaults["road_length_m"],
            "junction_radius": defaults["junction_radius_m"],
            "junction_id": 100,
            "safety_shoulder_width_m": 10.0,  # both-side shoulder so CARLA actors don't drop off the road edge
            "xodr_path": str(xodr_path),
            "notes": "Generated from minimal road seed; actor trajectories are intentionally excluded.",
        },
        "entities": [],
        "events": [],
    }


def patch_road_type(xodr_path: Path, road_type: str, speed_mps: float) -> None:
    tree = ET.parse(xodr_path)
    root = tree.getroot()
    for road in root.findall("./road"):
        for existing in list(road.findall("./type")):
            road.remove(existing)
        type_node = ET.Element("type", {"s": "0", "type": road_type})
        ET.SubElement(type_node, "speed", {"max": f"{speed_mps:g}", "unit": "m/s"})
        insert_at = 0
        link = road.find("./link")
        if link is not None:
            insert_at = list(road).index(link) + 1
        road.insert(insert_at, type_node)
    tree.write(xodr_path, encoding="utf-8", xml_declaration=True)


def patch_xsd_compatibility(xodr_path: Path) -> None:
    """Remove empty optional profile blocks that OpenDRIVE 1.5M XSD rejects."""
    tree = ET.parse(xodr_path)
    root = tree.getroot()
    for road in root.findall("./road"):
        for tag in ("elevationProfile", "lateralProfile"):
            node = road.find(f"./{tag}")
            if node is not None and len(list(node)) == 0 and not (node.text or "").strip():
                road.remove(node)
    # OpenDRIVE 1.5M XSD requires <link> children in the order
    # [predecessor?, successor?]. scenariogeneration occasionally emits them
    # in the reverse order on connector lanes — fine for CARLA but rejected by
    # xmlschema. Walk every <link> under <lane> and reorder if needed. Observed
    # 2026-06-27 after road_length doubled: junction cases (665/021/153/081/
    # 193/161/322/435/091) all failed validation with "Unexpected child with
    # tag 'predecessor' at position 2" until this patch landed.
    for link in root.iter("link"):
        children = list(link)
        if len(children) <= 1:
            continue
        # Only reorder direct children of lane <link> (not road-level <link>,
        # which has a different child contract — elementType etc.).
        kinds = [c.tag for c in children]
        if set(kinds) <= {"predecessor", "successor"} and kinds != sorted(
                kinds, key=lambda k: 0 if k == "predecessor" else 1):
            for c in children:
                link.remove(c)
            for c in sorted(children, key=lambda x: 0 if x.tag == "predecessor" else 1):
                link.append(c)
    tree.write(xodr_path, encoding="utf-8", xml_declaration=True)


def patch_strip_zero_length_lanesections(xodr_path: Path) -> None:
    """Strip laneSection elements whose s >= road.length (zero-length sections).

    The upstream OpenDRIVE generator (scenariogeneration / replay_scene_tools)
    occasionally emits a SECOND <laneSection> at s = road.length on connector
    roads of t_junction layouts (observed 2026-06-26 on case 153 with
    forward=2/backward=2 t_junction: road 100 length=66.41 had laneSection
    at s=0 AND s=66.41). CARLA's UE4 mesh baker chokes on the duplicate
    section — manifests as `time-out of 90000ms while waiting for the
    simulator` even after server restart.

    Stripping any section with s within 1e-3 of road.length is safe: such a
    section has zero physical span and contributes nothing to lane geometry.
    """
    tree = ET.parse(xodr_path)
    root = tree.getroot()
    stripped = 0
    for road in root.findall("./road"):
        try:
            length = float(road.get("length", "0"))
        except (TypeError, ValueError):
            continue
        lanes_node = road.find("./lanes")
        if lanes_node is None:
            continue
        for section in list(lanes_node.findall("./laneSection")):
            try:
                s = float(section.get("s", "0"))
            except (TypeError, ValueError):
                continue
            # zero-length tail section
            if s >= length - 1e-3:
                lanes_node.remove(section)
                stripped += 1
    if stripped:
        tree.write(xodr_path, encoding="utf-8", xml_declaration=True)


def patch_strip_dangling_junction_connections(xodr_path: Path) -> None:
    """Strip junction.connection elements that point to a non-existent connectingRoad.

    The fork-layout OpenDRIVE generator sometimes emits `<connection
    connectingRoad="None" ...>` because the connector road was never created
    (observed 2026-06-27 on case 246: ramp_fork with `forward=1, backward=0`
    produced 3 main roads + 1 junction whose 2 connections both pointed to
    `connectingRoad="None"`). CARLA's UE4 junction builder either hangs or
    rejects the file silently — manifests as 90s warmup timeout.

    Cleanup rule:
      - Any `<connection>` whose `connectingRoad` is missing, empty, "None",
        or doesn't resolve to a real `<road id=...>` in the file → strip it.
      - If a `<junction>` ends up with zero valid connections → strip the
        junction entirely. The remaining 3 main roads still form a usable
        scenario (ego drives on its starting road; fork branching becomes
        a soft topology hint rather than a hard routing requirement).
    """
    tree = ET.parse(xodr_path)
    root = tree.getroot()
    valid_road_ids = {road.get("id") for road in root.findall("./road") if road.get("id")}
    stripped_conns = 0
    stripped_junctions = 0
    for junction in list(root.findall("./junction")):
        for conn in list(junction.findall("./connection")):
            cr = conn.get("connectingRoad", "").strip()
            if cr in ("", "None") or cr not in valid_road_ids:
                junction.remove(conn)
                stripped_conns += 1
        if not junction.findall("./connection"):
            root.remove(junction)
            stripped_junctions += 1
    if stripped_conns or stripped_junctions:
        tree.write(xodr_path, encoding="utf-8", xml_declaration=True)


def road_reference_segments(xodr_path: Path) -> list[dict[str, Any]]:
    root = ET.parse(xodr_path).getroot()
    segments: list[dict[str, Any]] = []
    for road in root.findall("./road"):
        road_id = road.get("id", "")
        junction = road.get("junction", "-1")
        for geom in road.findall("./planView/geometry"):
            samples = _geometry_samples(geom)
            for start, end in zip(samples, samples[1:]):
                segments.append(
                    {
                        "road_id": road_id,
                        "junction": junction,
                        "segment": (start[0], start[1], end[0], end[1]),
                    }
                )
    return segments


def summarize_xodr(xodr_path: Path) -> dict[str, Any]:
    root = ET.parse(xodr_path).getroot()
    roads = root.findall("./road")
    junctions = root.findall("./junction")
    connections = root.findall("./junction/connection")
    lane_links = root.findall("./junction/connection/laneLink")
    first_section = root.find("./road/lanes/laneSection")
    driving_lanes_per_direction = None
    total_driving_lanes = None
    if first_section is not None:
        left_count = len(first_section.findall("./left/lane[@type='driving']"))
        right_count = len(first_section.findall("./right/lane[@type='driving']"))
        driving_lanes_per_direction = max(left_count, right_count)
        total_driving_lanes = left_count + right_count
    return {
        "road_count": len(roads),
        "junction_count": len(junctions),
        "connection_count": len(connections),
        "lane_link_count": len(lane_links),
        "driving_lanes_per_direction": driving_lanes_per_direction,
        "total_driving_lanes_on_approach": total_driving_lanes,
        "road_ids": [road.get("id") for road in roads],
        "junction_ids": [junction.get("id") for junction in junctions],
    }


def xodr_lane_boundary_segments(xodr_path: Path) -> list[dict[str, Any]]:
    root = ET.parse(xodr_path).getroot()
    segments: list[dict[str, Any]] = []
    for road in root.findall("./road"):
        lane_section = road.find("./lanes/laneSection")
        if lane_section is None:
            continue

        offsets = {0.0}
        for side_name, sign in (("left", 1.0), ("right", -1.0)):
            side = lane_section.find(side_name)
            if side is None:
                continue
            lanes = []
            for lane in side.findall("./lane"):
                if lane.get("type") != "driving":
                    continue
                width_node = lane.find("./width")
                width = float(width_node.get("a", 0.0)) if width_node is not None else 0.0
                if width > 0.0:
                    lanes.append((abs(int(lane.get("id", "0"))), width))
            lanes.sort()
            cumulative = 0.0
            for _, width in lanes:
                cumulative += width
                offsets.add(sign * cumulative)

        for geom in road.findall("./planView/geometry"):
            samples = _geometry_samples(geom)
            if len(samples) < 2:
                continue
            for offset in sorted(offsets):
                points = []
                for sx, sy, sh in samples:
                    left_x = -math.sin(sh)
                    left_y = math.cos(sh)
                    points.append((sx + left_x * offset, sy + left_y * offset))
                for start, end in zip(points, points[1:]):
                    segments.append(
                        {
                            "road_id": road.get("id"),
                            "junction": road.get("junction", "-1"),
                            "offset": offset,
                            "segment": (start[0], start[1], end[0], end[1]),
                        }
                    )
    return segments


def bounds_for_segments(segments: list[tuple[float, float, float, float]]) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for x1, y1, x2, y2 in segments:
        xs.extend([x1, x2])
        ys.extend([y1, y2])
    if not xs or not ys:
        return (-10.0, -10.0, 10.0, 10.0)
    pad = max(10.0, 0.08 * max(max(xs) - min(xs), max(ys) - min(ys)))
    return (min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)


def svg_line(segment: tuple[float, float, float, float], bounds: tuple[float, float, float, float], cls: str) -> str:
    min_x, min_y, max_x, max_y = bounds
    width = max_x - min_x
    height = max_y - min_y

    def tx(x: float) -> float:
        return (x - min_x) / width * 1000.0

    def ty(y: float) -> float:
        return (max_y - y) / height * 1000.0

    x1, y1, x2, y2 = segment
    return f'<line class="{cls}" x1="{tx(x1):.2f}" y1="{ty(y1):.2f}" x2="{tx(x2):.2f}" y2="{ty(y2):.2f}" />'


def write_html(seed: dict[str, Any], xodr_path: Path, html_path: Path, validation: dict[str, Any]) -> None:
    refs = road_reference_segments(xodr_path)
    boundaries = xodr_lane_boundary_segments(xodr_path)
    lanes = xodr_lane_center_segments(xodr_path)
    all_segments = [item["segment"] for item in refs] + [item["segment"] for item in boundaries] + [item["segment"] for item in lanes]
    bounds = bounds_for_segments(all_segments)
    summary = summarize_xodr(xodr_path)

    ref_lines = "\n".join(svg_line(item["segment"], bounds, "road-ref junction" if item["junction"] != "-1" else "road-ref",) for item in refs)
    boundary_lines = "\n".join(
        svg_line(
            item["segment"],
            bounds,
            "lane-boundary center" if abs(float(item["offset"])) < 1e-6 else "lane-boundary",
        )
        for item in boundaries
    )
    lane_lines = "\n".join(
        svg_line(item["segment"], bounds, "lane connector" if item["lane_id"] == "connector_ref" else "lane")
        for item in lanes
    )
    seed_json = html.escape(json.dumps(seed, ensure_ascii=False, indent=2))
    summary_json = html.escape(json.dumps({**summary, **validation}, ensure_ascii=False, indent=2))
    xodr_label = html.escape(str(xodr_path))

    html_path.parent.mkdir(parents=True, exist_ok=True)
    html_path.write_text(
        f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <title>{html.escape(xodr_path.stem)} road-only preview</title>
  <style>
    body {{ margin: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; color: #17202a; background: #f7f8fa; }}
    header {{ padding: 18px 24px; background: #ffffff; border-bottom: 1px solid #d8dee8; }}
    h1 {{ margin: 0 0 8px; font-size: 20px; }}
    .meta {{ display: flex; gap: 16px; flex-wrap: wrap; color: #4e5968; font-size: 13px; }}
    main {{ display: grid; grid-template-columns: minmax(520px, 1fr) 420px; gap: 18px; padding: 18px; }}
    .panel {{ background: #ffffff; border: 1px solid #d8dee8; border-radius: 8px; overflow: hidden; }}
    .panel h2 {{ margin: 0; padding: 12px 14px; font-size: 15px; border-bottom: 1px solid #e5e9f0; }}
    .canvas {{ aspect-ratio: 1 / 1; background: #eef2f6; }}
    svg {{ width: 100%; height: 100%; display: block; }}
    .road-ref {{ stroke: #202b3a; stroke-width: 4; stroke-linecap: round; opacity: 0.9; }}
    .road-ref.junction {{ stroke: #6f4bd8; stroke-width: 3; }}
    .lane-boundary {{ stroke: #7c8794; stroke-width: 1.2; stroke-linecap: round; opacity: 0.72; }}
    .lane-boundary.center {{ stroke: #c8a018; stroke-width: 1.6; stroke-dasharray: 8 7; opacity: 0.9; }}
    .lane {{ stroke: #15926f; stroke-width: 1.7; stroke-dasharray: 6 5; stroke-linecap: round; opacity: 0.85; }}
    .lane.connector {{ stroke: #d07819; stroke-width: 1.5; stroke-dasharray: 4 4; }}
    pre {{ margin: 0; padding: 14px; overflow: auto; font-size: 12px; line-height: 1.45; }}
    .legend {{ display: flex; gap: 14px; padding: 10px 14px; border-top: 1px solid #e5e9f0; color: #4e5968; font-size: 12px; }}
    .key {{ display: inline-flex; align-items: center; gap: 6px; }}
    .swatch {{ width: 24px; height: 3px; display: inline-block; border-radius: 2px; }}
    .ref {{ background: #202b3a; }}
    .boundary {{ background: #7c8794; }}
    .junction {{ background: #6f4bd8; }}
    .lane-s {{ background: #15926f; }}
    .connector {{ background: #d07819; }}
  </style>
</head>
<body>
  <header>
    <h1>Road-only OpenDRIVE Preview</h1>
    <div class="meta">
      <span>XODR: {xodr_label}</span>
      <span>roads: {summary["road_count"]}</span>
      <span>junctions: {summary["junction_count"]}</span>
      <span>connections: {summary["connection_count"]}</span>
      <span>laneLinks: {summary["lane_link_count"]}</span>
      <span>lanes: {summary["driving_lanes_per_direction"]}/direction, {summary["total_driving_lanes_on_approach"]} total</span>
    </div>
  </header>
  <main>
    <section class="panel">
      <h2>Road Geometry</h2>
      <div class="canvas">
        <svg viewBox="0 0 1000 1000" role="img" aria-label="OpenDRIVE road geometry preview">
          {boundary_lines}
          {ref_lines}
          {lane_lines}
        </svg>
      </div>
      <div class="legend">
        <span class="key"><span class="swatch ref"></span>road reference</span>
        <span class="key"><span class="swatch boundary"></span>lane boundary</span>
        <span class="key"><span class="swatch junction"></span>junction connector</span>
        <span class="key"><span class="swatch lane-s"></span>driving lane center</span>
        <span class="key"><span class="swatch connector"></span>connector sample</span>
      </div>
    </section>
    <section class="panel">
      <h2>Road Seed</h2>
      <pre>{seed_json}</pre>
      <h2>OpenDRIVE Summary</h2>
      <pre>{summary_json}</pre>
    </section>
  </main>
</body>
</html>
""",
        encoding="utf-8",
    )


def validate_xodr(xodr_path: Path, xsd_path: Path) -> dict[str, Any]:
    try:
        xmlschema.XMLSchema(str(xsd_path)).validate(str(xodr_path))
    except Exception as exc:  # pragma: no cover - CLI report path
        return {"xodr_schema_valid": False, "xodr_schema_error": str(exc)}
    return {"xodr_schema_valid": True}


def build(seed: dict[str, Any], xodr_path: Path, html_path: Path, xsd_path: Path) -> dict[str, Any]:
    errors = validate_seed(seed)
    if errors:
        raise ValueError("; ".join(errors))

    xodr_path.parent.mkdir(parents=True, exist_ok=True)
    trace = road_seed_to_trace(seed, xodr_path)
    write_generated_opendrive(trace, xodr_path)
    road_type = seed["road"]["type"]
    patch_road_type(xodr_path, road_type, ROAD_TYPE_DEFAULTS[road_type]["speed_mps"])
    patch_strip_zero_length_lanesections(xodr_path)
    patch_strip_dangling_junction_connections(xodr_path)
    patch_xsd_compatibility(xodr_path)

    validation = validate_xodr(xodr_path, xsd_path)
    write_html(seed, xodr_path, html_path, validation)
    return {
        "seed": seed,
        "xodr": str(xodr_path),
        "html": str(html_path),
        **summarize_xodr(xodr_path),
        **validation,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Build road-only OpenDRIVE and HTML from a minimal road seed")
    parser.add_argument("--seed", type=Path, required=True)
    parser.add_argument("--xodr", type=Path, required=True)
    parser.add_argument("--html", type=Path, required=True)
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--xsd", type=Path, default=Path("xsd/OpenDRIVE_1.5M.xsd"))
    args = parser.parse_args()

    result = build(read_seed(args.seed), args.xodr, args.html, args.xsd)
    if args.summary:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        args.summary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("xodr_schema_valid") else 1


if __name__ == "__main__":
    raise SystemExit(main())
