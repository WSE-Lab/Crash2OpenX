"""Behavior-vs-intent verification for scenario runs.

Layer 3 of the validation gate (Layer 1 = XSD conformance, Layer 2 = pipeline
phase success). Diffs the scene_seed.collision intent against the actual CARLA
outcome (sim_feedback.json + summary.json), so we catch silent failures like:

  - kinematics deadlock: ego and NPC at identical cruise speed → no collision,
    but neither did anything testable. summary.collision_count=0, but no brake
    event fired. (Case A_rainy_night_LVD, observed 2026-06-23.)
  - pathological short-path: ego goal placed in opposing lane → external_control
    PCLA executed a U-turn instead of overtaking. (Case B, same date.)

Both pass XSD validation and complete without runtime error, yet neither tests
the intended ADS behavior. This module is the gate that flags them.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET


# This metric is lane-center excess, not departure from the paved road. Allow a
# short, shallow impact-approach overrun while still rejecting sustained drift.
MAX_MINOR_OFF_ROAD_S = 1.25
MAX_MINOR_LATERAL_EXCESS_M = 0.25
MAX_MINOR_PRE_IMPACT_S = 1.25


def _trace_lane_excess(sim_feedback_path: Path, collision_time_s: float | None) -> dict[str, float | None]:
    """Measure how far/long hero's center exceeded its driving-lane half-width."""
    run_dir = sim_feedback_path.parent
    xodr = run_dir / "map.xodr"
    trace = run_dir / "sim_trace_raw.jsonl"
    if not xodr.is_file() or not trace.is_file():
        return {"max_lateral_excess_m": None, "off_road_before_collision_s": None}

    widths: dict[tuple[str, int], float] = {}
    try:
        root = ET.parse(xodr).getroot()
        for road in root.findall("road"):
            road_id = str(road.get("id"))
            for lane in road.findall("./lanes/laneSection/*/lane"):
                if lane.get("type") != "driving":
                    continue
                width = lane.find("width")
                if width is not None:
                    widths[(road_id, int(lane.get("id", "0")))] = float(width.get("a", "0"))
    except (OSError, ValueError, ET.ParseError):
        return {"max_lateral_excess_m": None, "off_road_before_collision_s": None}

    max_excess = 0.0
    first_excess_t: float | None = None
    try:
        for line in trace.read_text(encoding="utf-8").splitlines():
            row = json.loads(line)
            hero = (row.get("actors") or {}).get("hero") or {}
            width = widths.get((str(hero.get("road_id")), int(hero.get("lane_id") or 0)))
            distance = hero.get("nearest_driving_waypoint_distance_m")
            if not width or not isinstance(distance, (int, float)):
                continue
            excess = max(0.0, float(distance) - width / 2.0)
            max_excess = max(max_excess, excess)
            if excess > 1e-6 and first_excess_t is None:
                first_excess_t = float(row.get("simulation_time") or 0.0)
    except (OSError, ValueError, json.JSONDecodeError):
        return {"max_lateral_excess_m": None, "off_road_before_collision_s": None}

    before_collision = None
    if first_excess_t is not None and collision_time_s is not None:
        before_collision = max(0.0, float(collision_time_s) - first_excess_t)
    return {
        "max_lateral_excess_m": round(max_excess, 4),
        "off_road_before_collision_s": round(before_collision, 4) if before_collision is not None else None,
    }


def check_scene_outcome(scene_seed: dict | None,
                        carla_summary: dict | None,
                        sim_feedback_path: Path) -> dict[str, Any]:
    """Compare scene_seed intent vs CARLA-reported outcome.

    Returns a dict with:
      verdict: 'aligned' | 'drifted' | 'no_feedback' | 'skipped'
      aligned: True | False | None
      issues: list[str]                   # human-readable mismatch reasons
      expected_collision: list[str] | None
      actual_collision_actors: list[str] | None
      lane_invasion_count, wrong_lane_count, impact_area
    """
    out: dict[str, Any] = {
        "aligned": None, "verdict": "skipped", "issues": [], "warnings": [],
        "expected_collision": None, "actual_collision_actors": None,
        "lane_invasion_count": None, "wrong_lane_count": None, "impact_area": None,
        "off_road_time": None, "max_lateral_excess_m": None,
        "off_road_before_collision_s": None, "trajectory_quality": None,
    }
    expected = ((scene_seed or {}).get("scene") or {}).get("collision") or {}
    if expected.get("a") and expected.get("b"):
        # scene_seed uses 'ego'; the xosc/sim_feedback layer renames to 'hero'.
        def _canon(a: str) -> str:
            return "hero" if a == "ego" else a
        out["expected_collision"] = sorted([_canon(str(expected["a"])), _canon(str(expected["b"]))])

    feedback: dict[str, Any] | None = None
    if sim_feedback_path.exists():
        try:
            feedback = json.loads(sim_feedback_path.read_text())
        except (OSError, json.JSONDecodeError):
            feedback = None
    if not feedback:
        out["verdict"] = "no_feedback"
        return out

    fsum = feedback.get("summary") or {}
    actors = sorted(str(a) for a in (fsum.get("collision_actors") or []))
    out["actual_collision_actors"] = actors or None
    out["impact_area"] = (feedback.get("behavior_checks") or {}).get("impact_area_estimated")
    out["lane_invasion_count"] = (carla_summary or {}).get("lane_invasion_count")
    out["off_road_time"] = (carla_summary or {}).get("off_road_time")
    out.update(_trace_lane_excess(sim_feedback_path, fsum.get("collision_time_s")))
    out["wrong_lane_count"] = next(
        (int(c.get("actual_value") or 0)
         for c in ((carla_summary or {}).get("criteria") or [])
         if c.get("name") == "WrongLaneTest"),
        None,
    )

    issues: list[str] = []
    if out["expected_collision"]:
        if not fsum.get("collision_detected"):
            issues.append("expected collision but none occurred")
        elif actors and actors != out["expected_collision"]:
            issues.append(f"collision actors {actors} != expected {out['expected_collision']}")
        if out["impact_area"] and out["impact_area"] not in ("front", "rear", "side"):
            issues.append(f"impact area '{out['impact_area']}' did not match a clean impact geometry")
    if (out["wrong_lane_count"] or 0) > 0:
        issues.append(f"hero entered wrong lane (WrongLaneTest={out['wrong_lane_count']})")
    if (out["lane_invasion_count"] or 0) > 2:
        issues.append(f"excessive lane invasions ({out['lane_invasion_count']})")
    warnings: list[str] = []
    if 0 < (out["lane_invasion_count"] or 0) <= 2:
        warnings.append(f"minor lane-boundary contacts ({out['lane_invasion_count']})")
    off_road_time = float(out["off_road_time"] or 0.0)
    if off_road_time > 0:
        excess = out["max_lateral_excess_m"]
        pre_impact = out["off_road_before_collision_s"]
        if excess is None:
            # Older runs without trace/map cannot prove a longer excursion was minor.
            if off_road_time > 0.25:
                issues.append(f"off-road occupancy ({off_road_time:.2f} s; trace unavailable)")
            else:
                warnings.append(f"brief off-road occupancy ({off_road_time:.2f} s; trace unavailable)")
        else:
            severe = (
                off_road_time > MAX_MINOR_OFF_ROAD_S
                or float(excess) > MAX_MINOR_LATERAL_EXCESS_M
                or (pre_impact is not None and float(pre_impact) > MAX_MINOR_PRE_IMPACT_S)
            )
            detail = (f"{off_road_time:.2f} s, max center excess={float(excess):.3f} m"
                      + (f", began {float(pre_impact):.2f} s before impact" if pre_impact is not None else ""))
            if severe:
                issues.append(f"off-road trajectory excursion ({detail})")
            else:
                warnings.append(f"brief impact-approach boundary overrun ({detail})")

    # NEW (post-2026-06-23 lessons): also catch deadlock + U-turn anti-patterns
    # that DON'T trigger collision or wrong-lane but still fail the scenario intent.
    # sim_feedback.json's summary uses min_distance_m (numeric); carla_summary
    # carries duration_seconds at the top level (not in fsum).
    duration_s = float((carla_summary or {}).get("duration_seconds") or 0.0)
    min_dist = fsum.get("min_distance_m")
    if (out["expected_collision"] and not fsum.get("collision_detected")
            and isinstance(min_dist, (int, float))
            and min_dist >= 12.0
            and duration_s >= 5.0):
        # ego never approached: kinematics deadlock signature.
        issues.append(f"kinematics deadlock suspected (min_distance={min_dist:.1f} m over {duration_s:.1f} s)")

    out["issues"] = issues
    out["warnings"] = warnings
    out["aligned"] = not issues
    out["verdict"] = "aligned" if out["aligned"] else "drifted"
    out["trajectory_quality"] = (
        "invalid" if issues else ("acceptable_with_warnings" if warnings else "clean")
    )
    return out
