"""Pure-geometry helpers shared by replay visualizers.

Extracted from the archived ``visualize_replay_scene.py`` so both the new web
viewer and any future seed visualizer can render XODR road centerlines without
reaching back into ``archive_20260617/``.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

try:
    import pyclothoids as pcloth
except Exception:
    pcloth = None


def parse_xodr_lines(path: Path) -> list[dict[str, Any]]:
    """Extract per-road centerline polylines from an OpenDRIVE file.

    Returns a list of ``{"road_id": str, "points": [(x, y), ...]}``. Line,
    arc, and spiral geometries are sampled; poly3 remains unsupported. Returns
    ``[]`` when the file is missing.
    """
    if not path.exists():
        return []
    root = ET.parse(path).getroot()
    roads: list[dict[str, Any]] = []
    for road in root.findall("road"):
        for geom in road.findall("./planView/geometry"):
            x = float(geom.attrib.get("x", 0.0))
            y = float(geom.attrib.get("y", 0.0))
            hdg = float(geom.attrib.get("hdg", 0.0))
            length = float(geom.attrib.get("length", 0.0))
            points: list[tuple[float, float]] = [(x, y)]
            if geom.find("line") is not None:
                points.append((x + math.cos(hdg) * length, y + math.sin(hdg) * length))
            elif geom.find("arc") is not None and length > 0:
                curvature = float(geom.find("arc").attrib.get("curvature", 0.0))
                samples = max(2, min(240, math.ceil(length / 3.0)))
                if abs(curvature) <= 1e-12:
                    points.append((x + math.cos(hdg) * length,
                                   y + math.sin(hdg) * length))
                else:
                    points = []
                    for idx in range(samples + 1):
                        s = length * idx / samples
                        theta = hdg + curvature * s
                        points.append((
                            x + (math.sin(theta) - math.sin(hdg)) / curvature,
                            y - (math.cos(theta) - math.cos(hdg)) / curvature,
                        ))
            elif geom.find("spiral") is not None and pcloth is not None and length > 0:
                spiral = geom.find("spiral")
                curv_start = float(spiral.attrib.get("curvStart", 0.0))
                curv_end = float(spiral.attrib.get("curvEnd", 0.0))
                cloth = pcloth.Clothoid.StandardParams(
                    x, y, hdg, curv_start, (curv_end - curv_start) / length, length,
                )
                samples = 18
                points = [
                    (cloth.X(length * idx / samples), cloth.Y(length * idx / samples))
                    for idx in range(samples + 1)
                ]
            else:
                continue
            roads.append({"road_id": road.attrib.get("id", ""), "points": points})
    return roads


def offset_polyline(points: list[tuple[float, float]], offset: float) -> list[tuple[float, float]]:
    """Shift a polyline perpendicular to its local tangent by ``offset`` metres."""
    if len(points) < 2 or abs(offset) <= 1e-9:
        return points
    shifted: list[tuple[float, float]] = []
    for idx, (x, y) in enumerate(points):
        if idx == 0:
            nx = points[1][0] - x
            ny = points[1][1] - y
        elif idx == len(points) - 1:
            nx = x - points[idx - 1][0]
            ny = y - points[idx - 1][1]
        else:
            nx = points[idx + 1][0] - points[idx - 1][0]
            ny = points[idx + 1][1] - points[idx - 1][1]
        length = math.hypot(nx, ny)
        if length <= 1e-9:
            shifted.append((x, y))
            continue
        left_x = -ny / length
        left_y = nx / length
        shifted.append((x + left_x * offset, y + left_y * offset))
    return shifted


def svg_points(points: list[tuple[float, float]]) -> str:
    """Render a polyline as ``x,-y`` SVG points (Y-flipped for screen space)."""
    return " ".join(f"{x:.3f},{-y:.3f}" for x, y in points)
