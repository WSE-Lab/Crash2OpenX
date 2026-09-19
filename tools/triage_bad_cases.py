#!/usr/bin/env python3
"""Triage the 42-medoid CARLA batch into *actionable* buckets.

`classify_deadlocks.py` answers "whose fault is it" (taxonomy A-E: pipeline vs
ADS). This script answers the next question: "which of OUR bugs does this case
hit, and what is the evidence". Each bad case is mapped either to a numbered
pipeline bug (P1..P5, fix it) or to an ADS/design bucket (T1..T3, that IS the
test result). Every bucket rule is expressed over measured fields only — no
hand-maintained case lists — so the report regenerates from a fresh batch.

Evidence per case comes from the run dir:
  summary.json          termination_reason, route_completion, lane_invasion_count,
                        off_road_time, collision_count, min_distance, min_ttc
  behavior_check.json   expected_collision, verdict, trajectory_quality
  frame_states.jsonl    ego-only pose trace -> travelled distance and, crucially,
                        the signed progress along the initial heading. Projecting
                        onto heading[0] recovers a lane-frame s offset without
                        needing the XODR: s(t) ~= s0 + (p(t)-p(0)).h(0) on the
                        straight templates these medoids compile to. `s_min < 0`
                        means ego left the road through the s=0 START, which is
                        what makes WrongLaneTest crash (see EGO_REAR_RUNWAY_M in
                        tools/osc_blocks.py).
  scenario.runtime.xosc the s0 actually executed, so s_min is absolute

Usage (from repo root):
    .venv/bin/python tools/triage_bad_cases.py
    .venv/bin/python tools/triage_bad_cases.py --batch outputs/medoid_runs_agent_if
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BATCH = ROOT / "outputs" / "medoid_runs"
SCENE_SEED_DIR = ROOT / "data" / "seeds" / "scene_seed"

# Positions/blocks that need the CARLA-derived RoadGraph ("option B"). A case is
# a junction case iff its seed uses one of these — same predicate osc_blocks.py
# uses to decide whether to demand outputs/map_cache/<cid>/.
JUNCTION_POSITIONS = {"cross", "opposing_leg"}
JUNCTION_BLOCKS = {"junction_cross", "junction_turn"}
# Blocks whose NPC never moves: a braking ego can always avoid them, so
# "no collision" is a scenario-design property, not a runtime failure.
STATIC_BLOCKS = {"stopped_ahead", "static_block", "static_hold"}

# Bucket table: (id, whose fault, one-line meaning). Order matters — `classify`
# returns the first match, most-specific first. `B*` = our bug, `C*` = an ADS
# deficiency the tool correctly exposed, `D*` = a property of the scenario design.
BUCKETS = {
    "B1": ("pipeline", "cut_in RelativeTargetLane value=0 -> ZeroDivisionError (Bug1, FIXED)"),
    "B3": ("pipeline", "junction case: needs the CARLA-derived roadgraph map_cache (Bug3, BLOCKED)"),
    "B2": ("pipeline", "ego slid backwards out of the s=0 road end and fell off the mesh (Bug2, FIXED)"),
    "B4": ("pipeline", "runtime exception, cause not yet localized"),
    "C1": ("ads", "ADS lane departure: drifted past the shoulder edge and fell off the mesh"),
    "C2": ("ads", "ADS never engaged: ego barely moved"),
    "C3": ("ads", "ADS never closed on a moving NPC (routing/speed miss)"),
    "D1": ("design", "static-NPC family: ego braked in time, collision unrealizable by design"),
    "OK": ("none", "aligned"),
}

STALL_PATH_M = 10.0          # ego travelled less than this => never got moving
LANE_INVASION_BAD = 3        # CheckKeepLane tolerance used by the runner criteria
# z below this means ego is no longer on the generated mesh. Not 0: shoulder
# lanes sit ~1 m below the crown, and 040 bottoms out at exactly -1.00 m while
# still driving, so a -0.5 threshold would misread a shoulder excursion as a fall.
OFF_MESH_Z = -2.0


def load_verdicts(batch: Path) -> dict[str, dict]:
    """Last `_summary.tsv` row per cid (a cid may have been retried)."""
    tsv = batch / "_summary.tsv"
    if not tsv.is_file():
        raise SystemExit(f"no _summary.tsv in {batch}")
    latest: dict[str, dict] = {}
    with tsv.open() as f:
        for row in csv.DictReader(f, delimiter="\t"):
            if row.get("cid"):
                latest[row["cid"]] = row
    return latest


def _json(path: Path) -> dict:
    try:
        return json.loads(path.read_text())
    except Exception:
        return {}


def _ego_s0(run_dir: Path) -> float | None:
    """First <LanePosition/@s> in the executed xosc — that is ego's Init s."""
    xo = run_dir / "scenario.runtime.xosc"
    if not xo.is_file():
        return None
    for lp in ET.parse(xo).getroot().iter("LanePosition"):
        return float(lp.attrib["s"])
    return None


def road_geometry(cid: str, run_dir: Path) -> dict:
    """Road length, lateral half-widths, and whether the reference line is straight.

    The straightness matters: `trace_metrics` projects the ego trace onto its
    initial heading to recover a lane-frame (s, t). That is exact on a single
    <line> planView and meaningless on a curve, so the flag gates whether the
    projected numbers may be used as evidence.
    """
    xodr = run_dir / "map.xodr"
    if not xodr.is_file():
        xodr = ROOT / "data" / "compiled" / "opendrive_seed" / f"{cid}.xodr"
    if not xodr.is_file():
        return {}
    road = ET.parse(xodr).getroot().find("road")
    if road is None:
        return {}
    geos = road.findall("planView/geometry")
    sides = {}
    sec = road.find("lanes/laneSection")
    for side in ("left", "right"):
        el = None if sec is None else sec.find(side)
        w = 0.0
        for lane in ([] if el is None else el.findall("lane")):
            wel = lane.find("width")
            if wel is not None:
                w += float(wel.attrib["a"])
        sides[side] = round(w, 2)
    return {
        "road_length_m": float(road.attrib["length"]),
        "half_width_left_m": sides.get("left"),
        "half_width_right_m": sides.get("right"),
        # Narrower side: crossing this much lateral offset leaves the mesh.
        "lat_edge_m": min(v for v in sides.values() if v) if any(sides.values()) else None,
        "straight": len(geos) == 1 and geos[0][0].tag == "line",
    }


def trace_metrics(run_dir: Path) -> dict:
    """Ego kinematics reduced to the few numbers the bucket rules need."""
    fs = run_dir / "frame_states.jsonl"
    if not fs.is_file():
        return {}
    rows = [json.loads(l) for l in fs.read_text().splitlines() if l.strip()]
    if not rows:
        return {}
    r0 = rows[0]
    yaw = math.radians(r0["yaw"])
    hx, hy = math.cos(yaw), math.sin(yaw)

    def project(r: dict) -> tuple[float, float]:
        dx, dy = r["x"] - r0["x"], r["y"] - r0["y"]
        return dx * hx + dy * hy, -dx * hy + dy * hx

    path, px, py = 0.0, r0["x"], r0["y"]
    for r in rows:
        path += math.hypot(r["x"] - px, r["y"] - py)
        px, py = r["x"], r["y"]

    s0 = _ego_s0(run_dir)
    # Split the trace at the moment ego leaves the mesh. Everything after that is
    # a tumble through the void: the horizontal projection keeps accumulating, so
    # a whole-trace min/max would wildly overstate the on-road excursion (009
    # reads -32 m over the full trace but only -10.5 m at the actual exit).
    exit_row = next((r for r in rows if r["z"] < OFF_MESH_Z), None)
    on_road = [r for r in rows if r["z"] >= OFF_MESH_Z]
    on_proj = [project(r) for r in on_road] or [(0.0, 0.0)]

    out = {
        "sim_seconds": round(rows[-1]["simulation_time"] - r0["simulation_time"], 1),
        "ticks": len(rows),
        "path_m": round(path, 1),
        "ego_s0": s0,
        # Measured on-road only, so these describe driving, not falling.
        "progress_min_m": round(min(p for p, _ in on_proj), 1),
        "progress_max_m": round(max(p for p, _ in on_proj), 1),
        "lateral_max_m": round(max(abs(t) for _, t in on_proj), 1),
        "ego_s_min": None if s0 is None else round(s0 + min(p for p, _ in on_proj), 1),
        "z_min_m": round(min(r["z"] for r in rows), 2),
        # `speed` in frame_states includes vz, so a falling ego reads 80 m/s.
        # Horizontal speed is the only meaningful driving-speed figure.
        "max_speed_h_mps": round(max(math.hypot(r["vx"], r["vy"]) for r in rows), 1),
        "final_lane": f"{rows[-1]['road_id']}/{rows[-1]['lane_id']}",
        "final_lane_type": rows[-1].get("lane_type"),
    }
    if exit_row is not None:
        p, t = project(exit_row)
        out.update({
            "off_mesh": True,
            "exit_time_s": round(exit_row["simulation_time"] - r0["simulation_time"], 2),
            "exit_progress_m": round(p, 1),
            "exit_lateral_m": round(t, 1),
            "exit_s": None if s0 is None else round(s0 + p, 1),
        })
    else:
        out["off_mesh"] = False
    return out


def seed_shape(cid: str) -> dict:
    """What the SceneSeed asks for: junction-ness and NPC block families."""
    seed = SCENE_SEED_DIR / f"{cid}.json"
    scene = (_json(seed).get("scene") or {}) if seed.is_file() else {}
    npcs = scene.get("npcs") or []
    blocks, positions = [], []
    for n in npcs:
        blocks.append(((n.get("behavior") or {}).get("block")) or "")
        positions.append(n.get("position") or "")
    return {
        "npc_blocks": blocks,
        "npc_positions": positions,
        "is_junction": bool(
            set(positions) & JUNCTION_POSITIONS or set(blocks) & JUNCTION_BLOCKS
        ),
        "all_static_npcs": bool(blocks) and all(b in STATIC_BLOCKS for b in blocks),
    }


def collect(cid: str, row: dict, batch: Path) -> dict:
    run_dir = batch / cid
    summ = _json(run_dir / "summary.json")
    beh = _json(run_dir / "behavior_check.json")
    ev = {
        "cid": cid,
        "runner_verdict": row.get("verdict", "unknown"),
        "runner_detail": row.get("detail", ""),
        "termination_reason": summ.get("termination_reason"),
        "route_completion_pct": summ.get("final_route_completion"),
        "collision_count": summ.get("collision_count"),
        "lane_invasion_count": summ.get("lane_invasion_count"),
        "off_road_time_s": summ.get("off_road_time"),
        "min_distance_m": (summ.get("min_distance") or {}).get("value"),
        "min_ttc_s": (summ.get("min_ttc") or {}).get("value"),
        "behavior_verdict": beh.get("verdict"),
        "expected_collision": bool(beh.get("expected_collision")),
        "trajectory_quality": beh.get("trajectory_quality"),
        "issues": beh.get("issues") or [],
    }
    ev.update(trace_metrics(run_dir))
    ev.update(road_geometry(cid, run_dir))
    ev.update(seed_shape(cid))
    return ev


def classify(ev: dict) -> str:
    """First matching bucket. Rules read only measured fields."""
    term = ev.get("termination_reason") or ""

    # B1: the cut_in emission bug has a unique signature. Bucket by the exception
    # itself, not by its consequences — the crash fires ~2 s in, so these runs also
    # look like "ego never moved", and a path-length rule would steal them.
    if term == "exception:ZeroDivisionError":
        return "B1"

    # B3: junction cases are blocked upstream — they cannot even compile locally
    # without outputs/map_cache/<cid>/, so no runtime signature is diagnostic yet.
    if ev.get("is_junction"):
        return "B3"

    # B2 vs C1: both end with ego off the mesh, but through different boundaries,
    # and only one is ours. Longitudinal exit through s=0 is our placement bug
    # (ego was parked too close to the road start). Lateral exit past the shoulder
    # edge is an ADS lane departure that the fall merely obscures.
    if ev.get("off_mesh"):
        s_exit = ev.get("exit_s")
        lat_exit = abs(ev.get("exit_lateral_m") or 0.0)
        edge = ev.get("lat_edge_m")
        if s_exit is not None and s_exit < 0 and (edge is None or lat_exit < edge):
            return "B2"
        return "C1"

    if term.startswith("exception:"):
        return "B4"

    if ev.get("behavior_verdict") == "aligned" or ev.get("runner_verdict") == "ok":
        return "OK"

    if (ev.get("path_m") or 0.0) < STALL_PATH_M:
        return "C2"

    # Remaining bad cases drove on the road for the whole episode. Split by *why*
    # the expected collision did not happen.
    if ev.get("all_static_npcs"):
        return "D1"
    return "C3"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--batch", type=Path, default=DEFAULT_BATCH)
    ap.add_argument("--out-json", type=Path,
                    default=ROOT / "data" / "eval" / "bad_case_triage.json")
    ap.add_argument("--out-md", type=Path,
                    default=ROOT / "data" / "eval" / "bad_case_triage.md")
    args = ap.parse_args()

    rows = load_verdicts(args.batch)
    cases = []
    for cid, row in sorted(rows.items()):
        ev = collect(cid, row, args.batch)
        ev["bucket"] = classify(ev)
        ev["fault"], ev["bucket_meaning"] = BUCKETS[ev["bucket"]]
        cases.append(ev)

    counts = Counter(c["bucket"] for c in cases)
    pipeline_n = sum(v for k, v in counts.items() if BUCKETS[k][0] == "pipeline")

    payload = {
        "batch": str(args.batch.relative_to(ROOT)),
        "cases_total": len(cases),
        "bucket_counts": dict(sorted(counts.items())),
        "pipeline_bug_cases": pipeline_n,
        "buckets": {k: {"fault": v[0], "meaning": v[1]} for k, v in BUCKETS.items()},
        "thresholds": {
            "stall_path_m": STALL_PATH_M,
            "lane_invasion_bad": LANE_INVASION_BAD,
            "off_mesh_z": OFF_MESH_Z,
        },
        "cases": cases,
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")

    by_fault = Counter(BUCKETS[c["bucket"]][0] for c in cases)
    lines = [
        f"# Bad-case triage — `{payload['batch']}`",
        "",
        f"{len(cases)} cases: **{by_fault['pipeline']} hit a pipeline bug of ours**, "
        f"{by_fault['ads']} expose an ADS deficiency (that is the tool working), "
        f"{by_fault['design']} are unrealizable by scenario design, {by_fault['none']} aligned.",
        "",
        "Regenerate with `.venv/bin/python tools/triage_bad_cases.py`. Every bucket rule",
        "reads measured fields only — there are no hand-maintained case lists.",
        "",
        "| bucket | fault | n | meaning |",
        "| --- | --- | --- | --- |",
    ]
    for b, (fault, meaning) in BUCKETS.items():
        if counts.get(b):
            lines.append(f"| {b} | {fault} | {counts[b]} | {meaning} |")

    lines += [
        "",
        "## Off-mesh excursions",
        "",
        "`speed` in `frame_states.jsonl` includes the vertical component, so a falling",
        "ego reads up to 83 m/s. These figures use horizontal speed, and the excursion",
        "is measured **at the exit frame** — after that the ego is tumbling and the",
        "projection accumulates meaningless distance.",
        "",
        "| case | bucket | exit t | exit s | exit lateral | lat edge | z min |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for c in sorted((c for c in cases if c.get("off_mesh")),
                    key=lambda c: (c["bucket"], c["cid"])):
        lines.append(
            f"| {c['cid']} | {c['bucket']} | {c.get('exit_time_s')} s | "
            f"{c.get('exit_s')} m | {c.get('exit_lateral_m')} m | "
            f"±{c.get('lat_edge_m')} m | {c.get('z_min_m')} m |"
        )

    lines += ["", "## All cases", "",
              "| case | bucket | termination | route% | path m | v_h max | lane inv | min dist m |",
              "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    cols = ("termination_reason", "route_completion_pct", "path_m",
            "max_speed_h_mps", "lane_invasion_count", "min_distance_m")
    for c in sorted(cases, key=lambda c: (c["bucket"], c["cid"])):
        vals = [c.get(k) if c.get(k) is not None else "-" for k in cols]
        lines.append("| " + " | ".join([c["cid"], c["bucket"], *map(str, vals)]) + " |")
    args.out_md.write_text("\n".join(lines) + "\n")

    print(f"{len(cases)} cases -> {dict(sorted(counts.items()))}")
    print(f"by fault: {dict(by_fault)}")
    print(f"wrote {args.out_json.relative_to(ROOT)} and {args.out_md.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
