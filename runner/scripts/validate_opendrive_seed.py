#!/usr/bin/env python3
"""Validate OpenDRIVE seed maps before using them for scenario generation."""

import argparse
import csv
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT_DIR = REPO_ROOT / "opendrive_seed"
DEFAULT_REPORT_DIR = REPO_ROOT / "reports"


def _as_float(value):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed):
        return None
    return parsed


def _add_issue(issues, severity, code, message, context=None):
    issue = {
        "severity": severity,
        "code": code,
        "message": message,
    }
    if context:
        issue["context"] = context
    issues.append(issue)


def _driving_lanes_by_road(road):
    lanes = set()
    for lane in road.findall("lanes/laneSection/*/lane"):
        if lane.attrib.get("type") == "driving":
            lanes.add(lane.attrib.get("id"))
    return lanes


def _check_numeric_attribute(issues, element, attr_name, code, message, context):
    value = _as_float(element.attrib.get(attr_name))
    if value is None:
        _add_issue(issues, "error", code, message, context)
    return value


def _validate_header(root, issues):
    header = root.find("header")
    if header is None:
        _add_issue(issues, "warning", "missing_header", "OpenDRIVE has no header element")
        return

    for attr_name in ("north", "south", "east", "west"):
        if attr_name in header.attrib:
            _check_numeric_attribute(
                issues,
                header,
                attr_name,
                "invalid_header_bounds",
                "Header bound is not a finite number",
                {"attribute": attr_name, "value": header.attrib.get(attr_name)},
            )

    if header.find("geoReference") is None:
        _add_issue(issues, "warning", "missing_geo_reference", "Header has no geoReference")


def _validate_roads(root, issues):
    roads = root.findall("road")
    road_by_id = {}
    lane_ids_by_road = {}
    road_stats = {
        "roads": len(roads),
        "roads_with_geometry": 0,
        "roads_with_driving_lanes": 0,
        "driving_lanes": 0,
        "junctions": len(root.findall("junction")),
    }

    if not roads:
        _add_issue(issues, "error", "no_roads", "OpenDRIVE contains no road elements")
        return road_by_id, lane_ids_by_road, road_stats

    for road in roads:
        road_id = road.attrib.get("id")
        context = {"road_id": road_id}
        if road_id is None:
            _add_issue(issues, "error", "missing_road_id", "Road has no id attribute")
        elif road_id in road_by_id:
            _add_issue(issues, "error", "duplicate_road_id", "Duplicate road id", context)
        else:
            road_by_id[road_id] = road

        length = _check_numeric_attribute(
            issues,
            road,
            "length",
            "invalid_road_length",
            "Road length is missing or not finite",
            context,
        )
        if length is not None and length <= 0.0:
            _add_issue(issues, "error", "non_positive_road_length", "Road length must be positive", context)

        plan_view = road.find("planView")
        if plan_view is None:
            _add_issue(issues, "error", "missing_plan_view", "Road has no planView", context)
        else:
            geometries = plan_view.findall("geometry")
            if not geometries:
                _add_issue(issues, "error", "missing_geometry", "Road planView has no geometry", context)
            else:
                road_stats["roads_with_geometry"] += 1
                geometry_length_sum = 0.0
                for index, geometry in enumerate(geometries):
                    geometry_context = {"road_id": road_id, "geometry_index": index}
                    for attr_name in ("s", "x", "y", "hdg", "length"):
                        attr_value = _check_numeric_attribute(
                            issues,
                            geometry,
                            attr_name,
                            "invalid_geometry_attribute",
                            "Geometry attribute is missing or not finite",
                            {**geometry_context, "attribute": attr_name},
                        )
                        if attr_name == "length" and attr_value is not None:
                            if attr_value <= 0.0:
                                _add_issue(
                                    issues,
                                    "error",
                                    "non_positive_geometry_length",
                                    "Geometry length must be positive",
                                    geometry_context,
                                )
                            geometry_length_sum += attr_value
                    if not list(geometry):
                        _add_issue(
                            issues,
                            "error",
                            "missing_geometry_shape",
                            "Geometry has no shape child such as line/arc/spiral/poly3",
                            geometry_context,
                        )
                if length is not None and geometry_length_sum > 0.0:
                    tolerance = max(0.1, length * 0.02)
                    if abs(geometry_length_sum - length) > tolerance:
                        _add_issue(
                            issues,
                            "warning",
                            "geometry_length_mismatch",
                            "Sum of geometry lengths differs from road length",
                            {
                                "road_id": road_id,
                                "road_length": length,
                                "geometry_length_sum": geometry_length_sum,
                            },
                        )

        lane_sections = road.findall("lanes/laneSection")
        if not lane_sections:
            _add_issue(issues, "error", "missing_lane_section", "Road has no laneSection", context)

        driving_lanes = _driving_lanes_by_road(road)
        lane_ids_by_road[road_id] = driving_lanes
        road_stats["driving_lanes"] += len(driving_lanes)
        if driving_lanes:
            road_stats["roads_with_driving_lanes"] += 1
        else:
            _add_issue(issues, "warning", "no_driving_lanes", "Road has no driving lanes", context)

        for lane in road.findall("lanes/laneSection/*/lane"):
            lane_id = lane.attrib.get("id")
            lane_context = {"road_id": road_id, "lane_id": lane_id}
            try:
                numeric_lane_id = int(lane_id)
            except (TypeError, ValueError):
                _add_issue(issues, "error", "invalid_lane_id", "Lane id must be an integer", lane_context)
                numeric_lane_id = None
            if lane.attrib.get("type") == "driving" and numeric_lane_id == 0:
                _add_issue(issues, "error", "driving_center_lane", "Lane 0 cannot be a driving lane", lane_context)

            if lane.attrib.get("type") == "driving":
                widths = lane.findall("width")
                if not widths:
                    _add_issue(issues, "warning", "driving_lane_missing_width", "Driving lane has no width", lane_context)
                for index, width in enumerate(widths):
                    width_context = {**lane_context, "width_index": index}
                    coefficients = []
                    for attr_name in ("a", "b", "c", "d"):
                        value = _check_numeric_attribute(
                            issues,
                            width,
                            attr_name,
                            "invalid_lane_width",
                            "Lane width coefficient is missing or not finite",
                            {**width_context, "attribute": attr_name},
                        )
                        coefficients.append(value)
                    if coefficients[0] is not None and coefficients[0] <= 0.0:
                        _add_issue(issues, "warning", "non_positive_lane_width", "Lane width starts non-positive", width_context)

    return road_by_id, lane_ids_by_road, road_stats


def _validate_road_links(root, road_by_id, issues):
    junction_ids = {junction.attrib.get("id") for junction in root.findall("junction")}
    for road_id, road in road_by_id.items():
        for tag in ("predecessor", "successor"):
            link = road.find(f"link/{tag}")
            if link is None:
                continue
            element_type = link.attrib.get("elementType")
            element_id = link.attrib.get("elementId")
            context = {"road_id": road_id, "link": tag, "element_type": element_type, "element_id": element_id}
            if element_type == "road" and element_id not in road_by_id:
                _add_issue(issues, "error", "road_link_missing_road", "Road link references a missing road", context)
            elif element_type == "junction" and element_id not in junction_ids:
                _add_issue(issues, "error", "road_link_missing_junction", "Road link references a missing junction", context)
            elif element_type not in ("road", "junction"):
                _add_issue(issues, "warning", "unknown_road_link_type", "Road link has an unknown elementType", context)


def _validate_junctions(root, road_by_id, lane_ids_by_road, issues):
    junction_ids = set()
    for junction in root.findall("junction"):
        junction_id = junction.attrib.get("id")
        context = {"junction_id": junction_id}
        if junction_id is None:
            _add_issue(issues, "error", "missing_junction_id", "Junction has no id attribute")
        elif junction_id in junction_ids:
            _add_issue(issues, "error", "duplicate_junction_id", "Duplicate junction id", context)
        junction_ids.add(junction_id)

        connections = junction.findall("connection")
        if not connections:
            _add_issue(issues, "warning", "junction_without_connections", "Junction has no connection elements", context)

        for index, connection in enumerate(connections):
            incoming_road = connection.attrib.get("incomingRoad")
            connecting_road = connection.attrib.get("connectingRoad")
            connection_context = {
                "junction_id": junction_id,
                "connection_index": index,
                "incoming_road": incoming_road,
                "connecting_road": connecting_road,
            }
            if incoming_road not in road_by_id:
                _add_issue(issues, "error", "junction_missing_incoming_road", "Connection references missing incomingRoad", connection_context)
            if connecting_road not in road_by_id:
                _add_issue(issues, "error", "junction_missing_connecting_road", "Connection references missing connectingRoad", connection_context)

            lane_links = connection.findall("laneLink")
            if not lane_links:
                _add_issue(issues, "warning", "connection_without_lane_links", "Connection has no laneLink elements", connection_context)

            incoming_lanes = lane_ids_by_road.get(incoming_road, set())
            connecting_lanes = lane_ids_by_road.get(connecting_road, set())
            for lane_link in lane_links:
                from_lane = lane_link.attrib.get("from")
                to_lane = lane_link.attrib.get("to")
                lane_context = {**connection_context, "from_lane": from_lane, "to_lane": to_lane}
                if from_lane not in incoming_lanes:
                    _add_issue(issues, "error", "lane_link_missing_from_lane", "laneLink references a missing incoming driving lane", lane_context)
                if to_lane not in connecting_lanes:
                    _add_issue(issues, "error", "lane_link_missing_to_lane", "laneLink references a missing connecting driving lane", lane_context)


def validate_file(path):
    started = time.perf_counter()
    issues = []
    result = {
        "file": str(path),
        "name": path.name,
        "status": "fail",
        "error_count": 0,
        "warning_count": 0,
        "stats": {},
        "issues": issues,
        "elapsed_seconds": 0.0,
    }

    try:
        tree = ET.parse(path)
    except ET.ParseError as exc:
        _add_issue(issues, "error", "xml_parse_error", "File is not valid XML", {"detail": str(exc)})
        result["elapsed_seconds"] = round(time.perf_counter() - started, 4)
        result["error_count"] = 1
        return result

    root = tree.getroot()
    if root.tag != "OpenDRIVE":
        _add_issue(issues, "error", "invalid_root", "Root element is not OpenDRIVE", {"root": root.tag})

    _validate_header(root, issues)
    road_by_id, lane_ids_by_road, road_stats = _validate_roads(root, issues)
    _validate_road_links(root, road_by_id, issues)
    _validate_junctions(root, road_by_id, lane_ids_by_road, issues)

    result["stats"] = road_stats
    result["error_count"] = sum(1 for issue in issues if issue["severity"] == "error")
    result["warning_count"] = sum(1 for issue in issues if issue["severity"] == "warning")
    result["status"] = "pass" if result["error_count"] == 0 else "fail"
    result["elapsed_seconds"] = round(time.perf_counter() - started, 4)
    return result


def write_json_report(report_path, payload):
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def write_csv_report(report_path, results):
    report_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "name",
        "status",
        "error_count",
        "warning_count",
        "roads",
        "roads_with_geometry",
        "roads_with_driving_lanes",
        "driving_lanes",
        "junctions",
        "elapsed_seconds",
    ]
    with report_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for result in results:
            stats = result.get("stats", {})
            row = {key: result.get(key) for key in fieldnames}
            row.update({key: stats.get(key) for key in fieldnames if key in stats})
            writer.writerow(row)


def parse_args():
    parser = argparse.ArgumentParser(description="Validate OpenDRIVE seed maps and write reports.")
    parser.add_argument("--input-dir", default=str(DEFAULT_INPUT_DIR), help="Directory containing .xodr seed maps.")
    parser.add_argument("--report-dir", default=str(DEFAULT_REPORT_DIR), help="Directory for JSON/CSV reports.")
    parser.add_argument("--json-name", default="opendrive_seed_validation.json", help="JSON report file name.")
    parser.add_argument("--csv-name", default="opendrive_seed_validation.csv", help="CSV summary file name.")
    return parser.parse_args()


def main():
    args = parse_args()
    input_dir = Path(args.input_dir).resolve()
    report_dir = Path(args.report_dir).resolve()
    paths = sorted(input_dir.glob("*.xodr"))

    results = [validate_file(path) for path in paths]
    summary = {
        "input_dir": str(input_dir),
        "file_count": len(results),
        "passed": sum(1 for result in results if result["status"] == "pass"),
        "failed": sum(1 for result in results if result["status"] == "fail"),
        "total_errors": sum(result["error_count"] for result in results),
        "total_warnings": sum(result["warning_count"] for result in results),
    }
    payload = {
        "summary": summary,
        "results": results,
    }

    json_path = report_dir / args.json_name
    csv_path = report_dir / args.csv_name
    write_json_report(json_path, payload)
    write_csv_report(csv_path, results)

    print(f"Validated {summary['file_count']} OpenDRIVE maps")
    print(f"Passed: {summary['passed']}, failed: {summary['failed']}")
    print(f"Errors: {summary['total_errors']}, warnings: {summary['total_warnings']}")
    print(f"JSON report: {json_path}")
    print(f"CSV report: {csv_path}")
    return 1 if summary["failed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
