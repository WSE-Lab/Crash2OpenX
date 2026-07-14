#!/usr/bin/env python3
"""Repair small generated OpenDRIVE maps for CARLA topology import."""

import argparse
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


def _as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _angle_normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _geometry_heading(road):
    geometry = road.find("planView/geometry")
    if geometry is None:
        return None
    return _as_float(geometry.attrib.get("hdg"))


def _heading_from_road_contact(road, contact_point):
    hdg = _geometry_heading(road)
    if hdg is None:
        return None
    if contact_point == "start":
        return _angle_normalize(hdg + math.pi)
    return _angle_normalize(hdg)


def _heading_into_road_contact(road, contact_point):
    hdg = _geometry_heading(road)
    if hdg is None:
        return None
    if contact_point == "end":
        return _angle_normalize(hdg + math.pi)
    return _angle_normalize(hdg)


def _endpoint_xy(road, contact_point):
    geometry = road.find("planView/geometry")
    if geometry is None:
        return None
    x = _as_float(geometry.attrib.get("x"))
    y = _as_float(geometry.attrib.get("y"))
    if contact_point == "start":
        return x, y
    length = _as_float(road.attrib.get("length"))
    hdg = _as_float(geometry.attrib.get("hdg"))
    if geometry.find("line") is None:
        return None
    return x + math.cos(hdg) * length, y + math.sin(hdg) * length


def _distance(point_a, point_b):
    if point_a is None or point_b is None:
        return float("inf")
    return math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1])


def _driving_lane_ids(road):
    lane_ids = []
    for lane in road.findall("lanes/laneSection/*/lane"):
        if lane.attrib.get("type") == "driving" and lane.attrib.get("id") != "0":
            lane_ids.append(lane.attrib["id"])
    return sorted(lane_ids, key=lambda lane_id: int(lane_id))


def _ensure_lane_link(lane, tag, target_id):
    link = lane.find("link")
    if link is None:
        link = ET.Element("link")
        lane.insert(0, link)
    child = link.find(tag)
    if child is None:
        child = ET.SubElement(link, tag)
    if child.attrib.get("id") != str(target_id):
        child.set("id", str(target_id))
        return 1
    return 0


def _clear_and_set_connection_lane_links(connection, lane_pairs):
    existing = [(link.attrib.get("from"), link.attrib.get("to")) for link in connection.findall("laneLink")]
    desired = [(str(src), str(dst)) for src, dst in lane_pairs]
    if existing == desired:
        return 0
    for link in list(connection.findall("laneLink")):
        connection.remove(link)
    for src, dst in desired:
        ET.SubElement(connection, "laneLink", {"from": src, "to": dst})
    return 1


def _lane_pairs(incoming_road, connecting_road, reverse_sign):
    incoming_lanes = _driving_lane_ids(incoming_road)
    connecting_lanes = set(_driving_lane_ids(connecting_road))
    pairs = []
    for lane_id in incoming_lanes:
        target_id = str(-int(lane_id)) if reverse_sign else lane_id
        if target_id in connecting_lanes:
            pairs.append((lane_id, target_id))
    return pairs


def _road_junction_contact(road, junction_id):
    successor = road.find("link/successor")
    if successor is not None and successor.attrib.get("elementType") == "junction":
        if successor.attrib.get("elementId") == junction_id:
            return "end"
    predecessor = road.find("link/predecessor")
    if predecessor is not None and predecessor.attrib.get("elementType") == "junction":
        if predecessor.attrib.get("elementId") == junction_id:
            return "start"
    return "end"


def _reverse_between_contacts(source_contact, target_contact):
    return source_contact == target_contact


def _repair_junction_connections(root, roads):
    changes = 0
    for junction in root.findall("junction"):
        for connection in junction.findall("connection"):
            incoming = roads.get(connection.attrib.get("incomingRoad"))
            connecting = roads.get(connection.attrib.get("connectingRoad"))
            if incoming is None or connecting is None:
                continue

            contact = connection.attrib.get("contactPoint", "start")
            predecessor = connecting.find("link/predecessor")
            successor = connecting.find("link/successor")
            if predecessor is not None and predecessor.attrib.get("elementId") == incoming.attrib.get("id"):
                expected_contact = "start"
            elif successor is not None and successor.attrib.get("elementId") == incoming.attrib.get("id"):
                expected_contact = "end"
            else:
                expected_contact = contact
            if contact != expected_contact:
                connection.set("contactPoint", expected_contact)
                contact = expected_contact
                changes += 1

            incoming_contact = _road_junction_contact(incoming, junction.attrib.get("id"))
            reverse = _reverse_between_contacts(incoming_contact, contact)
            lane_pairs = _lane_pairs(incoming, connecting, reverse)
            if lane_pairs:
                changes += _clear_and_set_connection_lane_links(connection, lane_pairs)
    return changes


def _repair_connector_lane_links(root, roads):
    changes = 0
    for road in root.findall("road"):
        if road.attrib.get("junction", "-1") == "-1":
            continue
        predecessor = road.find("link/predecessor")
        successor = road.find("link/successor")
        predecessor_road = roads.get(predecessor.attrib.get("elementId")) if predecessor is not None else None
        successor_road = roads.get(successor.attrib.get("elementId")) if successor is not None else None
        if predecessor_road is None and successor_road is None:
            continue

        pred_reverse = (
            predecessor is not None
            and _reverse_between_contacts("start", predecessor.attrib.get("contactPoint", "end"))
        )
        succ_reverse = (
            successor is not None
            and _reverse_between_contacts("end", successor.attrib.get("contactPoint", "start"))
        )

        for lane in road.findall("lanes/laneSection/*/lane"):
            if lane.attrib.get("type") != "driving":
                continue
            lane_id = lane.attrib["id"]
            if predecessor_road is not None:
                target_id = str(-int(lane_id)) if pred_reverse else lane_id
                if target_id in _driving_lane_ids(predecessor_road):
                    changes += _ensure_lane_link(lane, "predecessor", target_id)
            if successor_road is not None:
                target_id = str(-int(lane_id)) if succ_reverse else lane_id
                if target_id in _driving_lane_ids(successor_road):
                    changes += _ensure_lane_link(lane, "successor", target_id)
    return changes


def _repair_boundary_road_links(root, roads):
    changes = 0
    for road in root.findall("road"):
        if road.attrib.get("junction", "-1") != "-1":
            continue
        road_id = road.attrib.get("id")
        lane_successors = {}
        lane_predecessors = {}
        for connection in root.findall(".//junction/connection"):
            connecting = roads.get(connection.attrib.get("connectingRoad"))
            if connecting is None:
                continue
            contact = connection.attrib.get("contactPoint", "start")
            if connection.attrib.get("incomingRoad") == road_id:
                for lane_link in connection.findall("laneLink"):
                    lane_successors[lane_link.attrib.get("from")] = lane_link.attrib.get("to")
            elif connecting.find("link/successor") is not None and connecting.find("link/successor").attrib.get("elementId") == road_id:
                for lane_link in connection.findall("laneLink"):
                    target = lane_link.attrib.get("from")
                    lane_predecessors[lane_link.attrib.get("to")] = target

        for lane in road.findall("lanes/laneSection/*/lane"):
            if lane.attrib.get("type") != "driving":
                continue
            lane_id = lane.attrib["id"]
            if lane_id in lane_successors and lane_successors[lane_id] is not None:
                changes += _ensure_lane_link(lane, "successor", lane_successors[lane_id])
            if lane_id in lane_predecessors and lane_predecessors[lane_id] is not None:
                changes += _ensure_lane_link(lane, "predecessor", lane_predecessors[lane_id])
    return changes


def _repair_straight_connectors(root, roads, straight_tolerance_degrees):
    changes = 0
    tolerance = math.radians(straight_tolerance_degrees)
    for road in root.findall("road"):
        if road.attrib.get("junction", "-1") == "-1":
            continue
        predecessor = road.find("link/predecessor")
        successor = road.find("link/successor")
        if predecessor is None or successor is None:
            continue
        pred_road = roads.get(predecessor.attrib.get("elementId"))
        succ_road = roads.get(successor.attrib.get("elementId"))
        if pred_road is None or succ_road is None:
            continue
        pred_heading = _heading_from_road_contact(pred_road, predecessor.attrib.get("contactPoint", "end"))
        connector_start_heading = _heading_into_road_contact(road, "start")
        succ_heading = _heading_into_road_contact(succ_road, successor.attrib.get("contactPoint", "start"))
        if pred_heading is None or connector_start_heading is None or succ_heading is None:
            continue
        if abs(_angle_normalize(pred_heading - connector_start_heading)) > tolerance:
            continue
        if abs(_angle_normalize(pred_heading - succ_heading)) > tolerance:
            continue

        first_geometry = road.find("planView/geometry")
        if first_geometry is None or first_geometry.find("line") is None:
            continue
        start = _endpoint_xy(pred_road, predecessor.attrib.get("contactPoint", "end"))
        end = _endpoint_xy(succ_road, successor.attrib.get("contactPoint", "start"))
        if start is None or end is None:
            continue
        length = _distance(start, end)
        if length <= 0.0:
            continue
        expected_hdg = math.atan2(end[1] - start[1], end[0] - start[0])
        updates = {
            "x": str(start[0]),
            "y": str(start[1]),
            "hdg": str(expected_hdg),
            "length": str(length),
        }
        for key, value in updates.items():
            if first_geometry.attrib.get(key) != value:
                first_geometry.set(key, value)
                changes += 1
        if road.attrib.get("length") != str(length):
            road.set("length", str(length))
            changes += 1
    return changes


def repair_opendrive_xml(opendrive_data, straighten=True, straight_tolerance_degrees=8.0):
    root = ET.fromstring(opendrive_data)
    roads = {road.attrib.get("id"): road for road in root.findall("road")}
    changes = 0
    changes += _repair_junction_connections(root, roads)
    changes += _repair_connector_lane_links(root, roads)
    changes += _repair_boundary_road_links(root, roads)
    if straighten:
        changes += _repair_straight_connectors(root, roads, straight_tolerance_degrees)
    try:
        ET.indent(root, space="    ")
    except AttributeError:
        pass
    return ET.tostring(root, encoding="unicode"), changes


def repair_file(path, backup=True):
    path = Path(path)
    original = path.read_text(encoding="utf-8")
    repaired, changes = repair_opendrive_xml(original)
    if changes and repaired != original:
        if backup:
            backup_path = path.with_suffix(path.suffix + ".bak")
            if not backup_path.exists():
                shutil.copy2(path, backup_path)
        path.write_text("<?xml version='1.0' encoding='utf-8'?>\n" + repaired, encoding="utf-8")
    return changes


def main():
    parser = argparse.ArgumentParser(description="Repair generated OpenDRIVE junction/lane topology.")
    parser.add_argument("paths", nargs="+", help="OpenDRIVE files or directories.")
    parser.add_argument("--no-backup", action="store_true", help="Do not create .bak files before writing.")
    args = parser.parse_args()

    xodr_paths = []
    for raw_path in args.paths:
        path = Path(raw_path)
        if path.is_dir():
            xodr_paths.extend(sorted(path.rglob("*.xodr")))
        elif path.suffix == ".xodr":
            xodr_paths.append(path)

    total_changes = 0
    for path in xodr_paths:
        changes = repair_file(path, backup=not args.no_backup)
        total_changes += changes
        print(f"{path}: {changes} changes")
    print(f"Repaired {len(xodr_paths)} files, {total_changes} total changes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
