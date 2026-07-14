#!/usr/bin/env python3
"""OpenSCENARIO block assembler (CARLA-runnable).

scene_seed (position x behavior(block+params)) -> parameterized OpenSCENARIO fragments
placed RELATIVE to ego, assembled into one XOSC modeled on CARLA's official examples
(FollowLeadingVehicle / PedestrianCrossingFront / LaneChangeSimple):
  - ego = external_control (handed to the ADS under test)
  - NPC vehicles = npc_vehicle_control + SpeedAction/LaneChangeAction
  - triggers fire on the HERO entity (condition references the NPC)
  - Environment with Sun; scenario_runner criteria_* monitoring patched in

Triggers are NOT a scene_seed axis: each block hardcodes its trigger TYPE; the threshold is a
block param (default + mutable). scene_seed carries only position + behavior(block+params).

Junction blocks (cross / opposing_leg) need the RoadGraph (option B) and raise BlockUnsupported.

Demo:
    uv run python tools/osc_blocks.py --demo rear_end --xodr outputs/opendrive_seed/<name>.xodr
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import scenariogeneration.xosc as xosc

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.replay_scene_tools import (
    _add_vehicle_controller, _build_env_action, _patch_monitoring_criteria, validate_xosc,
)

XSD = ROOT / "xsd" / "OpenSCENARIO.xsd"
EGO = "hero"
EGO_ROAD, EGO_LANE, EGO_S0 = 1, -1, 30.0
CATALOG_DIR = "openscenarios/catalogs"
ACT_STOP_DIST = 200.0
# Distance ego stays away from BOTH road ends. Remote scenario_runner's
# atomic_criteria.update() calls lane_waypoint.next(2.0)[0] every tick — at the
# road boundary that returns [] and raises IndexError on the first tick. 5 m is
# enough on every map we've seen to keep a 2 m forward lookup inside the lane.
EGO_EDGE_MARGIN = 5.0
# Default ego↔NPC longitudinal spacing for positions that consume `gap`.
# 2026-06-27 (D3): split position-aware after the A-fix one-size-fits-all
# 25 m hurt `rear_hit`: NPC at 25 m behind ego only closes at ~2-8 m/s, often
# never catches up in 60 s (263 went deadlock under uniform 25). Defaults
# now follow the geometric role of each position:
#   - behind_same_lane: 15 m so the trailing vehicle is close enough to
#     close in within the episode (was 25 → too far).
#   - oncoming: 20 m. Closer than 25 so head-on closing speed (18 m/s
#     post-D2) eats the gap fast, but not so close PCLA emergency-brakes
#     at t=0.
#   - everything else (ahead_same_lane / adjacent / roadside / cross /
#     opposing_leg): 25 m, matching the A-fix rationale (PCLA acceleration
#     window before reaching trigger ranges).
DEFAULT_GAP_BY_POSITION = {
    "behind_same_lane": 15.0,
    "oncoming":         20.0,
}
# A static lead vehicle does not need the 25 m acceleration window used by
# dynamic ahead/adjacent actors. CARLA tuning on case-165 found 19 m lets
# InterFuser avoid/stop, while 22 m reliably produces the report's rear-end
# interaction without the 0.30 s shoulder excursion observed at 25 m.
DEFAULT_GAP_BY_BLOCK = {
    "stopped_ahead": 23.0,
    "front_brake": 20.0,
}
DEFAULT_GAP_M = 25.0  # fallback for positions not in the dict above

# Schema v2.2: qualitative time_of_day -> ISO datetime for _build_env_action.
# Date is fixed (any non-leap day works); only the hour matters because
# _build_env_action derives the sun position from time_value.hour.
_TIME_QUAL_TO_ISO = {
    "dawn":      "2024-06-15T06:00:00",
    "morning":   "2024-06-15T09:00:00",
    "afternoon": "2024-06-15T14:00:00",
    "evening":   "2024-06-15T18:00:00",
    "dusk":      "2024-06-15T19:30:00",
    "night":     "2024-06-15T22:00:00",
}


def _env_for_compile(env: dict) -> dict:
    """scene.environment (qualitative) -> dict shape _build_env_action expects.

    weather strings already match _build_env_action's substring contract
    (it does ``"rain" in weather_text`` etc), so only time_of_day needs
    qualitative -> ISO conversion. ``unknown`` / missing time falls through
    to _build_env_action's default (noon, sun overhead)."""
    out = dict(env or {})
    tod = (out.get("time_of_day") or "").lower()
    if tod in _TIME_QUAL_TO_ISO:
        out["time_of_day"] = _TIME_QUAL_TO_ISO[tod]
    elif not tod or tod == "unknown":
        out.pop("time_of_day", None)
    return out


class BlockUnsupported(Exception):
    pass


def _params(npc: dict) -> dict:
    """Merge behavior params and position params (tolerant to either location)."""
    return {**((npc.get("behavior") or {}).get("params") or {}), **(npc.get("params") or {})}


def _default_gap_for(npc: dict) -> float:
    """Position-aware default gap (D3 split). Falls through to DEFAULT_GAP_M
    for any position not explicitly listed in DEFAULT_GAP_BY_POSITION."""
    block = (npc.get("behavior") or {}).get("block")
    if block in DEFAULT_GAP_BY_BLOCK:
        return DEFAULT_GAP_BY_BLOCK[block]
    return DEFAULT_GAP_BY_POSITION.get(npc.get("position"), DEFAULT_GAP_M)


def _xodr_road_length(xodr_path: str, road_id) -> float | None:
    try:
        root = ET.parse(xodr_path).getroot()
    except (ET.ParseError, OSError):
        return None
    for r in root.findall("road"):
        if str(r.attrib.get("id")) == str(road_id):
            try:
                return float(r.attrib.get("length"))
            except (TypeError, ValueError):
                return None
    return None


def resolve_ego_placement(xodr_path: str, scene: dict) -> tuple[int, int, float]:
    """Deterministically pick ego (road_id, lane_id, s) on the seed XODR.
    Straight-road scenes are translation-invariant, so any long-enough driving lane works;
    we take the longest non-junction road's driving lane and leave margins for the
    relative NPC offsets + lead-in. (Junction placement needs the RoadGraph / option B.)

    Lane choice depends on adjacent-NPC sides:
      - default: innermost forward lane (max(neg) = -1, closest to centerline);
      - if any NPC is `adjacent/left`: the relative dLane=+1 used by resolve_position
        would land that NPC on lane 0 (centerline, no driving width) and CARLA crashes
        with `lane_width_info != nullptr`. Instead pick the OUTERMOST forward lane
        (min(neg)) so dLane=+1 lands on -1 (a valid inner forward lane), letting the
        adjacent-left NPC live in same-direction multi-lane traffic as physically
        intended (observed root cause for 081 / 273 silent-spawn 2026-06-26).
    """
    path = Path(xodr_path)
    if not path.is_absolute():
        path = ROOT / xodr_path
    try:
        root = ET.parse(path).getroot()
    except Exception:
        return EGO_ROAD, EGO_LANE, EGO_S0
    needs_outer = any(
        npc.get("position") == "adjacent" and npc.get("side") == "left"
        for npc in scene.get("npcs", [])
    )
    best = None  # (length, road_id, lane_id)
    for road in root.findall("road"):
        if road.get("junction", "-1") != "-1":
            continue
        length = float(road.get("length", 0) or 0)
        neg = [int(l.get("id")) for l in road.findall(".//lane")
               if l.get("type") == "driving" and l.get("id") and int(l.get("id")) < 0]
        if neg and (best is None or length > best[0]):
            # `adjacent/left` requires ≥2 forward lanes (so ego can sit on the outer
            # one with an inner forward lane on its left). With only 1 forward lane,
            # adjacent-left has no valid mapping; we still pick max(neg) but the
            # caller (WF gate) should reject the scene.
            chosen = min(neg) if (needs_outer and len(neg) >= 2) else max(neg)
            best = (length, int(road.get("id")), chosen)
    if best is None:
        return EGO_ROAD, EGO_LANE, EGO_S0
    length, road_id, lane = best
    behind, ahead = 10.0, 30.0
    needs_oncoming_clear = 0.0
    for npc in scene.get("npcs", []):
        gap = float(_params(npc).get("gap", _default_gap_for(npc)))
        if npc.get("position") == "behind_same_lane":
            behind = max(behind, gap + 6.0)
        else:
            ahead = max(ahead, gap + 20.0)
        # Oncoming-clearance margin (2026-06-26 fix for 444 / 665):
        # CARLA scenario_runner's `convert_position_to_transform` resolves
        # RelativeLanePosition(dlane=+2, h=π) by calling
        # `waypoint.next(ds)[-1]` from ego's lane in the OPPOSING lane's
        # driving direction (which is s-decreasing). With ego_s small
        # (default 10), walking ds=15m in -s direction lands at s=-5,
        # off-road, and waypoint.next returns []. Force ego_s ≥ gap + margin
        # so that walk stays inside the road.
        if npc.get("position") == "oncoming" or (npc.get("behavior") or {}).get("block") == "oncoming":
            needs_oncoming_clear = max(needs_oncoming_clear, gap + EGO_EDGE_MARGIN + 5.0)
    s = max(behind, needs_oncoming_clear, 10.0)
    if length - s < ahead:
        s = max(5.0, length * 0.3)
    return road_id, lane, round(s, 2)


CRUISE_BLOCKS = {"front_brake", "cut_in"}
# Closing-speed delta added to ego cruise so it actually approaches the leading
# NPC. Without this, _hero_cruise returns NPC's own speed → both vehicles cruise
# at the same speed → ego never reaches the distance trigger → brake event never
# fires → "video shows two cars locked at fixed gap" (real failure mode observed
# 2026-06-23 on Case A_rainy_night_LVD; PCLA agent did 0 brake events in 5.5s).
# +4 m/s closes 15 m in ~3.75 s, well within a 60s episode and still inside
# PCLA's training distribution for "following slower lead".
HERO_CLOSING_DELTA_MPS = 4.0


def _hero_cruise(scene: dict) -> float:
    # PCLA-style learned planners regress badly when handed off at 0 m/s with a static
    # obstacle in the near field. Init hero at the leading NPC's cruise speed *plus a
    # small closing delta* so (a) takeover happens inside the "tracking traffic"
    # distribution the model was trained on, AND (b) ego actually approaches the lead
    # so distance triggers can fire. Falls back to 6.0 m/s when nothing in the scene
    # implies a cruise speed.
    speeds = [float(_params(n).get("speed", 8.0))
              for n in (scene.get("npcs") or [])
              if (n.get("behavior") or {}).get("block") in CRUISE_BLOCKS]
    if not speeds:
        return 6.0
    return max(speeds) + HERO_CLOSING_DELTA_MPS


# ----------------------------- roadgraph (junction mode, option B data) -----------------------------

import re as _re

JUNCTION_POSITIONS = {"cross", "opposing_leg"}
JUNCTION_BLOCKS = {"junction_cross", "junction_turn"}
# scene maneuver -> route.type as emitted by outputs/map_cache/<name>/route_candidates.json
MANEUVER_TO_ROUTE_TYPE = {"straight": "straight", "left": "left_turn", "right": "right_turn"}


def _s_of(wid: str) -> float:
    m = _re.search(r":s([\d.]+)$", wid)
    return float(m.group(1)) if m else 0.0


def _wpos(W: dict, wid: str):
    t = W[wid]["transform"]
    return float(t["x"]), float(t["y"]), float(t.get("z", 0.0)), float(t.get("yaw", 0.0))


def _exit_waypoint(W: dict, wid: str, max_hops: int = 12) -> str:
    """Walk forward from ``wid`` until we land on a non-junction driving waypoint.

    Used to turn the last vertex of a junction-crossing route (which often sits
    inside the junction body, where CARLA's path planner refuses to resolve a
    LanePosition target) into a target that's safely on the exit road. We hop
    along ``next_ids`` instead of taking the geometric "nearest" waypoint
    because junction interior IDs reference connection lanes whose road_id
    isn't a top-level XODR road and crash scenario_runner with IndexError.
    """
    cur = wid
    for _ in range(max_hops):
        node = W.get(cur)
        if node is None:
            return wid
        if not node.get("is_junction", False) and node.get("lane_type", "").lower() == "driving":
            return cur
        nxt = (node.get("next_ids") or [None])[0]
        if not nxt or nxt == cur:
            return cur
        cur = nxt
    return cur


def _lane_position_or_world(W: dict, wid: str, *, z_offset: float = 0.2):
    """Prefer a LanePosition target snapped via the cached waypoint metadata.

    Returns the snapped (LanePosition, exit_wid). Falls back to WorldPosition
    only when the waypoint has no road/lane fields (defensive — current cache
    always carries them).
    """
    exit_wid = _exit_waypoint(W, wid)
    node = W.get(exit_wid)
    if node is None or "road_id" not in node or "lane_id" not in node:
        ex, ey, ez, _ = _wpos(W, wid)
        return xosc.WorldPosition(x=ex, y=ey, z=ez + z_offset, h=0.0), wid
    return xosc.LanePosition(float(node["s"]), 0.0, int(node["lane_id"]), int(node["road_id"])), exit_wid


def load_roadgraph(name: str):
    base = ROOT / "outputs/map_cache" / name
    rc = base / "route_candidates.json"
    if not rc.exists():
        return None
    W = {w["id"]: w for w in json.loads((base / "waypoints.json").read_text())["waypoints"]}
    routes = json.loads(rc.read_text())["routes"]
    return W, routes


def _route_ids(route: dict) -> list[str]:
    ids = route.get("waypoint_ids")
    if isinstance(ids, list) and ids:
        return ids
    return (route.get("approach_waypoint_ids", []) + route.get("connector_waypoint_ids", [])
            + route.get("departure_waypoint_ids", []))


def scene_needs_junction(scene: dict) -> bool:
    for n in scene.get("npcs", []):
        if n.get("position") in JUNCTION_POSITIONS or (n.get("behavior") or {}).get("block") in JUNCTION_BLOCKS:
            return True
    return False


def _leg_of_route(W: dict, route: dict, ego_route: dict) -> str:
    """Classify an NPC candidate route's incoming leg relative to ego's incoming leg.

    Returns 'opposing' | 'cross_left' | 'cross_right' | 'same' | 'unknown'.
    Uses yaw difference at the approach start + NPC start's lateral offset in ego's
    local frame (sign of dy_local distinguishes the two cross legs)."""
    e_ids = _route_ids(ego_route)
    r_ids = _route_ids(route)
    if not (e_ids and r_ids and e_ids[0] in W and r_ids[0] in W):
        return "unknown"
    ex, ey, _, eyaw = _wpos(W, e_ids[0])
    nx, ny, _, nyaw = _wpos(W, r_ids[0])
    yaw_rel = (nyaw - eyaw + 540.0) % 360.0 - 180.0  # [-180, 180]
    if abs(yaw_rel) < 30:
        return "same"
    if abs(abs(yaw_rel) - 180.0) < 30:
        return "opposing"
    cy = math.cos(math.radians(eyaw))
    sy = math.sin(math.radians(eyaw))
    dy_local = -(nx - ex) * sy + (ny - ey) * cy
    return "cross_left" if dy_local > 0 else "cross_right"


def _npc_leg_constraint(position: str, side: str) -> set[str]:
    """schema position+side -> allowed legs for the NPC's incoming route."""
    if position == "opposing_leg":
        return {"opposing"}
    if position == "cross":
        if side == "left":
            return {"cross_left"}
        if side == "right":
            return {"cross_right"}
        return {"cross_left", "cross_right"}
    return set()


def _npc_type_constraint(block: str) -> set[str]:
    """schema block -> allowed route.type for the NPC's path through the junction."""
    if block == "junction_cross":
        return {"straight"}
    if block == "junction_turn":
        return {"left_turn", "right_turn"}
    return {"straight", "left_turn", "right_turn"}


def _lin(t: float) -> xosc.TransitionDynamics:
    return xosc.TransitionDynamics(xosc.DynamicsShapes.linear, xosc.DynamicsDimension.time, t)


def _step() -> xosc.TransitionDynamics:
    return xosc.TransitionDynamics(xosc.DynamicsShapes.step, xosc.DynamicsDimension.time, 0.0)


def _speed(v: float, t: float = 1.0):
    return xosc.AbsoluteSpeedAction(v, _lin(t))


# ----------------------------- entities -----------------------------

def _vehicle(name: str, ego: bool) -> xosc.Vehicle:
    v = xosc.Vehicle(name, xosc.VehicleCategory.car, xosc.BoundingBox(2.1, 4.5, 1.8, 1.5, 0.0, 0.9),
                     xosc.Axle(0.5, 0.6, 1.8, 3.1, 0.3), xosc.Axle(0.0, 0.6, 1.8, 0.0, 0.3), 69.4, 200.0, 10.0)
    v.add_property("type", "ego_vehicle" if ego else "simulation")
    return v


def _pedestrian(name: str) -> xosc.Pedestrian:
    p = xosc.Pedestrian(name, 90.0, xosc.PedestrianCategory.pedestrian,
                        xosc.BoundingBox(0.6, 0.6, 1.8, 0.0, 0.0, 0.9), model="walker.pedestrian.0001")
    p.add_property("type", "simulation")
    return p


def _entity(kind: str, name: str):
    if kind == "pedestrian":
        return _pedestrian(name)
    v = _vehicle(name, False)
    if kind == "cyclist":
        v.add_property("semantic_type", "cyclist")
    return v


def _ego_controller(init: xosc.Init) -> None:
    props = xosc.Properties()
    props.add_property("module", "external_control")
    ctrl = xosc.Controller("HeroAgent", props)
    assign = xosc.AssignControllerAction(controller=ctrl)
    override = xosc.OverrideControllerValueAction()
    for setter in ("set_throttle", "set_brake", "set_clutch", "set_steeringwheel", "set_gear", "set_parkingbrake"):
        getattr(override, setter)(False, 0)
    init.add_init_action(EGO, xosc.ControllerAction(assignControllerAction=assign, overrideControllerValueAction=override))


# ----------------------------- axis 1: position -----------------------------

def resolve_position(npc: dict):
    pos, side = npc.get("position"), npc.get("side", "none")
    p = _params(npc)
    gap = float(p.get("gap", _default_gap_for(npc)))
    if pos == "ahead_same_lane":
        return xosc.RelativeLanePosition(0, EGO, ds=gap)
    if pos == "behind_same_lane":
        return xosc.RelativeLanePosition(0, EGO, ds=-gap)
    if pos == "adjacent":
        # Block-aware default for `long` (2026-06-26): cut_in starts the NPC
        # in the adjacent lane and then fires a RelativeLaneChangeAction once
        # ego is within `trig_dist` cartesian. If the initial cartesian
        # distance √(long² + lane_width²) is already < trig_dist, the lane
        # change action fires at t=0 before the NPC has any longitudinal
        # velocity and CARLA divides by ~0 → ZeroDivisionError. Bump default
        # to 16 m so √(16² + 3.5²) ≈ 16.4 > trig_dist=15. Other adjacent
        # blocks (cyclist cut_in, static_hold, oncoming) keep the legacy
        # long=5 m default — they don't share the lane-change action.
        block = (npc.get("behavior") or {}).get("block")
        default_long = 16.0 if block == "cut_in" else 5.0
        return xosc.RelativeLanePosition(1 if side == "left" else -1, EGO,
                                         ds=float(p.get("long", default_long)))
    if pos == "oncoming":
        return xosc.RelativeLanePosition(2, EGO, ds=gap, orientation=xosc.Orientation(h=math.pi))
    if pos == "roadside":
        off = float(p.get("lateral", 3.5)) * (1 if side == "left" else -1)
        h = math.pi / 2 * (1 if side == "left" else -1)  # face across the road
        return xosc.RelativeLanePosition(0, EGO, ds=gap, offset=off, orientation=xosc.Orientation(h=h))
    if pos in ("cross", "opposing_leg"):
        raise BlockUnsupported(f"position {pos} needs RoadGraph (option B)")
    raise BlockUnsupported(f"unknown position {pos}")


# ----------------------------- triggers (hardcoded per block; threshold is a param) -----------------------------

def _trig_hero_distance(npc_id: str, value: float, dist_type=None) -> xosc.Trigger:
    dt = dist_type or xosc.RelativeDistanceType.longitudinal
    cond = xosc.RelativeDistanceCondition(value, xosc.Rule.lessThan, dt, npc_id, freespace=True)
    et = xosc.EntityTrigger(f"{npc_id}_trig", 0.0, xosc.ConditionEdge.rising, cond, EGO)
    cg = xosc.ConditionGroup(); cg.add_condition(et)
    t = xosc.Trigger(); t.add_conditiongroup(cg); return t


def _trig_hero_distance_after(npc_id: str, value: float, prev_event: str,
                              dist_type=None) -> xosc.Trigger:
    # AND'ing StoryboardElementStateCondition into the same ConditionGroup makes the
    # brake event wait for keep to endTransition, so at spawn-gap ~= trig_dist (where
    # both distance conds are true at t=0) keep no longer races brake. Pure XOSC,
    # no new params, no timer hacks.
    dt = dist_type or xosc.RelativeDistanceType.longitudinal
    dist_cond = xosc.RelativeDistanceCondition(value, xosc.Rule.lessThan, dt, npc_id, freespace=True)
    sb_cond = xosc.StoryboardElementStateCondition(
        xosc.StoryboardElementType.event, prev_event, xosc.StoryboardElementState.endTransition)
    et = xosc.EntityTrigger(f"{npc_id}_trig", 0.0, xosc.ConditionEdge.rising, dist_cond, EGO)
    vt = xosc.ValueTrigger(f"{npc_id}_after_{prev_event}", 0.0, xosc.ConditionEdge.rising, sb_cond)
    cg = xosc.ConditionGroup(); cg.add_condition(et); cg.add_condition(vt)
    t = xosc.Trigger(); t.add_conditiongroup(cg); return t


def _trig_hero_ttc(npc_id: str, value: float) -> xosc.Trigger:
    cond = xosc.TimeToCollisionCondition(value, xosc.Rule.lessThan, entity=npc_id)
    et = xosc.EntityTrigger(f"{npc_id}_trig", 0.0, xosc.ConditionEdge.rising, cond, EGO)
    cg = xosc.ConditionGroup(); cg.add_condition(et)
    t = xosc.Trigger(); t.add_conditiongroup(cg); return t


def _trig_simtime(npc_id: str, value: float = 0.0) -> xosc.Trigger:
    vt = xosc.ValueTrigger(f"{npc_id}_trig", 0.0, xosc.ConditionEdge.rising,
                           xosc.SimulationTimeCondition(value, xosc.Rule.greaterThan))
    cg = xosc.ConditionGroup(); cg.add_condition(vt)
    t = xosc.Trigger(); t.add_conditiongroup(cg); return t


def _ego_travel_trigger(value: float, point: str) -> xosc.Trigger:
    et = xosc.EntityTrigger(f"act_{point}", 0.0, xosc.ConditionEdge.rising,
                            xosc.TraveledDistanceCondition(value), EGO, triggeringpoint=point)
    cg = xosc.ConditionGroup(); cg.add_condition(et)
    t = xosc.Trigger(point); t.add_conditiongroup(cg); return t


# ----------------------------- axis 2: behavior blocks -> events -----------------------------

def _event(name: str, trigger: xosc.Trigger, action) -> xosc.Event:
    e = xosc.Event(name, xosc.Priority.overwrite)
    e.add_trigger(trigger)
    e.add_action(name, action)
    return e


def block_events(npc: dict) -> list[xosc.Event]:
    nid = npc["id"]
    block = npc.get("behavior", {}).get("block")
    p = _params(npc)
    speed = float(p.get("speed", 8.0))
    if block == "front_brake":
        # Cruise is now set in Init (see CRUISE_BLOCKS in build_xosc), so this event only
        # needs to fire the brake when hero closes in — no keep event, no storyboard-state
        # gating, no t=0 double-trigger race.
        if "trig_simtime" in p:
            brake_trigger = _trig_simtime(nid, float(p["trig_simtime"]))
        else:
            brake_trigger = _trig_hero_distance(nid, float(p.get("trig_dist", 18.0)))
        stop = _event(f"{nid}_brake",
                      brake_trigger,
                      _speed(float(p.get("end_speed", 0.0)), float(p.get("brake_t", 1.2))))
        return [stop]
    if block in ("rear_hit", "oncoming"):
        # 2026-06-27 (D2): default speed bumped per-block from the legacy
        # shared 8.0 m/s. With hero_cruise ≈ 6 m/s, the old 8 m/s gave a
        # closing speed of only 2 m/s for `rear_hit` (NPC trails ego on the
        # same lane) — over a 60 s episode the NPC could only close ~120 m,
        # so it never caught up in time on the post-B1 long roads (263
        # ended up at min_distance 25 m for the whole run). For `oncoming`
        # in head-on geometry the relative speed becomes ~6+12=18 m/s
        # versus ~6+8=14 m/s, materially raising the collision probability
        # without changing event semantics. scene_seed authors / LLM stay
        # free to override `params.speed` explicitly when a specific
        # accident report calls for it.
        block_default_speed = {"rear_hit": 14.0, "oncoming": 12.0}[block]
        actor_speed = float(p.get("speed", block_default_speed))
        return [_event(f"{nid}_go", _trig_simtime(nid), _speed(actor_speed))]
    if block == "cut_in":
        # Trigger swap (2026-06-26): old code used `TimeToCollisionCondition`
        # which CARLA 0.9.16's atomic_trigger_conditions resolves via
        # `global_route_planner.trace_route(...)` — and that route planner
        # crashes with `TypeError: 'NoneType' object is not subscriptable`
        # whenever the two actors sit on adjacent lanes (the planner's path
        # search returns None on cross-lane queries). Switch to a simple
        # cartesian distance trigger: identical "fire when ego closes in"
        # semantics, no route-planner involvement, no TypeError.
        #
        # ALSO AND in a `simtime ≥ 1.5s` guard — without it, when adjacent
        # NPC's longitudinal offset (default long=5) is smaller than
        # trig_dist=15, the cartesian distance is already < 15 at t=0 and
        # the lane-change action fires before NPC has any motion. CARLA's
        # RelativeLaneChangeAction then divides the cross-lane delta by
        # near-zero longitudinal velocity → `ZeroDivisionError`. The 1.5s
        # gate gives NPC time to accelerate and ego time to advance so the
        # cross-lane interpolation is well-conditioned. trig_dist param is
        # kept; trig_ttc is no longer consumed but accepted for back-compat.
        go = _event(f"{nid}_go", _trig_simtime(nid), _speed(speed))
        # Build the AND'd trigger inline (mirrors _trig_hero_distance_after's
        # ConditionGroup pattern but with SimulationTimeCondition instead of
        # StoryboardElementStateCondition).
        trig_dist = float(p.get("trig_dist", 15.0))
        min_delay = float(p.get("trig_simtime_min", 1.5))
        dist_cond = xosc.RelativeDistanceCondition(
            trig_dist, xosc.Rule.lessThan, xosc.RelativeDistanceType.cartesianDistance,
            nid, freespace=True)
        sim_cond = xosc.SimulationTimeCondition(min_delay, xosc.Rule.greaterThan)
        et = xosc.EntityTrigger(f"{nid}_trig", 0.0, xosc.ConditionEdge.rising, dist_cond, EGO)
        vt = xosc.ValueTrigger(f"{nid}_simgate", 0.0, xosc.ConditionEdge.rising, sim_cond)
        cg = xosc.ConditionGroup()
        cg.add_condition(et)
        cg.add_condition(vt)
        cut_trig = xosc.Trigger()
        cut_trig.add_conditiongroup(cg)
        cut = _event(f"{nid}_cut", cut_trig,
                     xosc.RelativeLaneChangeAction(0, EGO, _lin(2.0)))
        return [go, cut]
    if block in ("cross", "walk_along"):
        return [_event(f"{nid}_move", _trig_hero_distance(nid, float(p.get("trig_dist", 15.0)),
                       xosc.RelativeDistanceType.cartesianDistance), _speed(float(p.get("speed", 1.5)), 0.5))]
    if block in ("stopped_ahead", "static_block", "static_hold"):
        # stationary: a hold event so the Act is non-empty (keeps it at speed 0).
        # static_hold (added 2026-06-25, schema v2.3) is the position-agnostic
        # variant — same hold, but allowed at adjacent / opposing_leg / etc., to
        # express "vehicle parked / stopped at the stop line / blocking adjacent
        # lane" without inheriting stopped_ahead's "ahead_same_lane" implication.
        return [_event(f"{nid}_hold", _trig_simtime(nid), _speed(0.0, 0.5))]
    if block == "light_change_start":
        # NPC starts at rest, then accelerates to cruise speed at sim t=trig_simtime
        # (light-change moment approximated by simtime delay; seed XODR has no
        # traffic-signal geometry, so the qualitative info lives in scene.control).
        delay = float(p.get("trig_simtime", 2.0))
        return [
            _event(f"{nid}_hold", _trig_simtime(nid), _speed(0.0, 0.5)),
            _event(f"{nid}_start", _trig_simtime(nid, value=delay), _speed(speed, 0.5)),
        ]
    if block in ("junction_cross", "junction_turn"):
        raise BlockUnsupported(f"block {block} needs RoadGraph (option B)")
    raise BlockUnsupported(f"unknown block {block}")


# ----------------------------- cross-metamodel WF gate -----------------------------

class WFViolation(BlockUnsupported):
    """Cross-metamodel well-formedness violation (e.g. WF6, WF7).

    Subclassing BlockUnsupported keeps existing 'pending_roadgraph' / fix-hint
    flows working; coordinator.dispatch_full's xosc phase already routes
    BlockUnsupported into _format_xosc_failure_hint and a retry. The subclass
    just lets callers catch WF-specific failures separately when they want."""


def _check_wf(scene: dict, road_seed: dict) -> list[str]:
    """Return list of cross-metamodel WF violations between scene and road.

    Gates encoded here (all derived from real CARLA-runtime failure modes —
    catching them at xosc emission cuts the round-trip vs discovering after
    a 60-second remote scenario run):

      WF6  : `oncoming` position/block ⇒ road.lanes.backward ≥ 1
             (NPC at lane +1 needs that lane to exist)
      WF7  : sut.maneuver=overtake_oncoming ⇒ backward ≥ 1 AND center_line=broken
      WF8  : `adjacent + side=left` ⇒ road.lanes.forward ≥ 2
             (with only 1 forward lane, ego sits on -1 and dlane=+1 lands the
              NPC on lane 0 = centerline → CARLA: `lane_width_info != nullptr`.
              Observed on 081 / 273 silent-spawn, 2026-06-26.)
      WF9  : `adjacent + cut_in` ⇒ NPC.params.long ≥ ceil(trig_dist) − ~lane_width
             (initial cartesian distance √(long² + 3.5²) must exceed the
              `trig_dist` threshold, otherwise the lane-change action fires
              at t=0 before the NPC has any longitudinal velocity and CARLA
              divides by ~0 → ZeroDivisionError. Observed on 038 / 081 / 273
              retry, 2026-06-26.)
      WF4/WF5 : implicitly enforced elsewhere (RoadGraph for junction blocks;
                RelativeLanePosition(±1) failing at compile for missing lanes).
    """
    issues: list[str] = []
    road = (road_seed or {}).get("road") or road_seed or {}
    lanes = road.get("lanes") or {}
    if isinstance(lanes, dict):
        forward = int(lanes.get("forward", 1))
        backward = int(lanes.get("backward", forward))
    else:
        forward = backward = int(lanes)
    center = road.get("center_line", "broken")
    topology = road.get("topology", "straight")

    sut_maneuver = (scene.get("sut") or {}).get("maneuver", "straight")
    has_oncoming = any(
        n.get("position") == "oncoming"
        or (n.get("behavior") or {}).get("block") == "oncoming"
        for n in scene.get("npcs") or []
    )

    # WF6
    if has_oncoming and backward < 1:
        issues.append(
            f"WF6: scene has 'oncoming' position/block but road.lanes.backward={backward} "
            "(need ≥1; oncoming requires an opposing lane to exist)"
        )
    # WF7
    if sut_maneuver == "overtake_oncoming":
        if backward < 1:
            issues.append(
                f"WF7: sut.maneuver=overtake_oncoming requires road.lanes.backward≥1, got {backward}"
            )
        if center != "broken":
            issues.append(
                f"WF7: sut.maneuver=overtake_oncoming requires road.center_line=broken, got '{center}'"
            )

    # WF8: adjacent/left needs ≥2 forward lanes so the NPC can live on an
    # inner forward lane (lane -1) while ego sits on an outer one (lane -2).
    for n in scene.get("npcs") or []:
        if n.get("position") == "adjacent" and n.get("side") == "left" and forward < 2:
            issues.append(
                f"WF8: NPC {n.get('id','?')} uses 'adjacent/left' but road.lanes.forward={forward} "
                "(need ≥2; otherwise dlane=+1 lands the NPC on the centerline and CARLA crashes "
                "with lane_width_info != nullptr at spawn)"
            )

    # WF9: adjacent + cut_in needs longitudinal clearance ≥ trig_dist − lane_width.
    # We auto-correct the default `long` in resolve_position (block-aware), so
    # this gate only fires if the LLM (or a human author) EXPLICITLY set long
    # to a too-small value.
    LANE_WIDTH_M = 3.5
    for n in scene.get("npcs") or []:
        if n.get("position") != "adjacent":
            continue
        if (n.get("behavior") or {}).get("block") != "cut_in":
            continue
        p = (n.get("params") or {})
        if "long" not in p:
            continue  # default will be auto-bumped to safe value by resolve_position
        long_v = float(p["long"])
        trig_dist = float(((n.get("behavior") or {}).get("params") or {}).get("trig_dist", 15.0))
        min_long = max(0.0, (trig_dist ** 2 - LANE_WIDTH_M ** 2) ** 0.5) + 1.0
        if long_v < min_long:
            issues.append(
                f"WF9: NPC {n.get('id','?')} cut_in needs params.long ≥ {min_long:.1f} "
                f"(given trig_dist={trig_dist}, lane_width={LANE_WIDTH_M}), got long={long_v}"
            )

    # WF10: ego.maneuver in (left,right) on a junction topology + any NPC at
    # `position=oncoming` (or `block=oncoming`) is geometrically broken:
    # `oncoming` places the NPC on the OPPOSING lane of ego's CURRENT road.
    # Once ego turns into the cross/T road, the NPC is left behind on ego's
    # original road and the scripted collision can never happen — ego's
    # camera also loses the NPC entirely (observed 665 in 18-medoid batch
    # 2026-06-26: T_junction + ego right + oncoming cyclist → ego turned
    # away, cyclist invisible, collision never fired). The fix at the
    # schema level is to express "NPC on the road ego turns INTO" with
    # `position=cross + block=junction_cross/junction_turn` (or the
    # opposing-leg variant for straight cross), which the RoadGraph
    # routing path can place on the correct exit road. WF10 raises this
    # back to the LLM as a fix_hint instead of silently emitting a scene
    # that can't collide.
    JUNCTION_TOPOLOGIES = {"t_junction", "cross_intersection", "y_junction"}
    if (sut_maneuver in ("left", "right")
            and topology in JUNCTION_TOPOLOGIES
            and has_oncoming):
        issues.append(
            f"WF10: sut.maneuver={sut_maneuver} on topology={topology} cannot collide with an "
            f"NPC at position=oncoming (NPC stays on ego's original road after the turn); "
            f"use position=cross + block=junction_cross/junction_turn (or opposing_leg) instead"
        )

    return issues


# ----------------------------- assembly -----------------------------

def build_xosc(scene: dict, xodr_path: str, out_path: Path,
               name: str | None = None,
               *,
               auto_extract: bool = False,
               remote_client=None,
               road_seed: dict | None = None) -> Path:
    entities = xosc.Entities()
    init = xosc.Init()
    init.add_global_action(_build_env_action({"environment": _env_for_compile(scene.get("environment") or {})}))

    # Cross-metamodel WF gate (WF6, WF7). Skipped silently when no road_seed
    # is passed; legacy callers that only pass scene+xodr keep working.
    if road_seed is not None:
        wf_issues = _check_wf(scene, road_seed)
        if wf_issues:
            raise WFViolation(" ; ".join(wf_issues))

    junction = scene_needs_junction(scene)
    maneuver = (scene.get("sut") or {}).get("maneuver", "straight")
    # RoadGraph cache is REQUIRED for junction NPCs (so legs can be classified
    # against ego's chosen route). It is also USEFUL — but not required —
    # whenever ego's maneuver is non-straight: with a cache we can point
    # ego's AcquirePosition goal at the exit lane of the matching route,
    # giving PCLA a target consistent with the intended turn. Without one we
    # fall back to the legacy straight-200m goal, which causes WrongLane /
    # lane-invasion drift on turning scenes (observed across the 18-medoid
    # CARLA batch 2026-06-26). auto_extract is still gated on junction=True;
    # for non-junction turns we use whatever cache happens to exist.
    wants_rg = junction or maneuver in ("left", "right")
    rg = load_roadgraph(name) if (wants_rg and name) else None
    if junction and not rg and auto_extract and name and xodr_path:
        try:
            from tools.carla_client import get_carla_client
            client = remote_client or get_carla_client()
            client.extract_roadgraph(Path(xodr_path), name=name)
            rg = load_roadgraph(name)
        except Exception as exc:
            raise BlockUnsupported(
                f"junction scene needs roadgraph_map_cache/{name}; auto-extract failed: {exc}"
            ) from exc
    if junction and not rg:
        raise BlockUnsupported(f"junction scene needs roadgraph_map_cache/{name}")

    entities.add_scenario_object(EGO, _vehicle(EGO, True))
    ego_route = None
    W = None
    if rg:
        W, routes = rg
        route_type = MANEUVER_TO_ROUTE_TYPE.get(maneuver, "straight")
        # For non-junction scenes a missing match is acceptable — ego_route stays
        # None and placement / goal fall back to the legacy non-rg branch.
        # Junction scenes require some route to anchor NPC leg classification,
        # so we keep the legacy strict path (any route is better than none).
        ego_route = next((r for r in routes if r.get("type") == route_type),
                         routes[0] if (junction and routes) else None)
        if junction and ego_route is None:
            raise BlockUnsupported("no routes in roadgraph for ego")

    if junction and ego_route is not None:
        ap = ego_route.get("approach_waypoint_ids") or _route_ids(ego_route)
        ego_road_id = ego_route["start_road_id"]
        ego_lane_id = ego_route["start_lane_id"]
        # Keep ego ≥ EGO_EDGE_MARGIN from either road end. The remote
        # scenario_runner's atomic_criteria.update() calls
        # lane_waypoint.next(2.0)[0] every tick; when ego sits at the road
        # boundary that returns [] and crashes with IndexError on first tick.
        # Margin must respect lane direction: in OpenDRIVE, lane_id<0 drives
        # s-increasing, lane_id>0 drives s-decreasing.
        road_len = _xodr_road_length(xodr_path, ego_road_id)
        ap_s = (_s_of(ap[0]) if ap else 20.0)
        if ego_lane_id > 0:
            # Spawn near the high-s end (lane head from driving perspective).
            base = (road_len - EGO_EDGE_MARGIN) if road_len is not None else max(20.0, ap_s)
            ego_s = min(base, max(EGO_EDGE_MARGIN, road_len - ap_s if road_len else ap_s))
        else:
            ego_s = max(EGO_EDGE_MARGIN, ap_s - 40.0)
    else:
        er, el, es = resolve_ego_placement(xodr_path, scene)
        ego_s, ego_road_id, ego_lane_id = es, er, el
        road_len = _xodr_road_length(xodr_path, ego_road_id)
    init.add_init_action(EGO, xosc.TeleportAction(
        xosc.LanePosition(ego_s, 0.0, ego_lane_id, ego_road_id)))
    _ego_controller(init)
    # See _hero_cruise: avoid OOD-handoff swerves by booting hero at cruise.
    init.add_init_action(EGO, _speed(_hero_cruise(scene), 0.0))

    story = xosc.Story("scene_story")
    act = xosc.Act("scene_act",
                   starttrigger=_ego_travel_trigger(1.0, "start"),
                   stoptrigger=_ego_travel_trigger(ACT_STOP_DIST, "stop"))

    # EGO destination so PCLA's build_pcla_sut_route_from_xosc can synthesize a
    # route via the (Init Teleport -> AcquirePosition) fallback in demo.py. The
    # trigger fires at sim t=0 — AcquirePositionAction is a goal hint for the
    # external_control ADS, not a scripted motion, so there's no race with PCLA.
    # Clamp destination s to [EGO_EDGE_MARGIN, road_len - EGO_EDGE_MARGIN] so
    # the same atomic_criteria edge case (next(2.0)[0] -> IndexError) doesn't
    # trip when ego reaches the goal late in the scenario.
    if ego_lane_id > 0:
        goal_s = ego_s - ACT_STOP_DIST  # driving s-decreasing
    else:
        goal_s = ego_s + ACT_STOP_DIST  # driving s-increasing
    if road_len is not None:
        goal_s = min(road_len - EGO_EDGE_MARGIN, max(EGO_EDGE_MARGIN, goal_s))
    else:
        goal_s = max(EGO_EDGE_MARGIN, goal_s)
    # Schema v2.1: sut.maneuver=overtake_oncoming is a SCHEMA-LEVEL annotation
    # (it says "expect this report to need lateral evasion across the centerline,
    # and require road.lanes.backward>=1 + center_line=broken via WF7"). It must
    # NOT flip ego's goal lane: an earlier design did so, but external_control
    # PCLA agents read "goal in opposing lane far ahead" as "U-turn here" and
    # executed a shortest-path 180°, defeating the test (observed 2026-06-23
    # on Case B_overtake_broken_centerline). Correct behavior: keep ego goal
    # in the driving lane past the obstacle; the stopped/slow NPC + remote
    # in-lane goal forces ADS to decide (overtake / stop / fail). That decision
    # IS the test outcome.
    ego_goal_pos = xosc.LanePosition(goal_s, 0.0, ego_lane_id, ego_road_id)
    # For LEFT / RIGHT maneuvers with a RoadGraph cache available, retarget the
    # goal to the EXIT lane (last vertex of the matched ego_route, snapped to a
    # LanePosition on the exit road by _lane_position_or_world). This gives
    # PCLA a target that matches the scene's intended turn — without it the
    # 200m-ahead in-same-lane fallback makes PCLA drive past the junction in
    # the original lane and trip WrongLane / lane-invasion criteria (observed
    # 665 / 435, 2026-06-26). overtake_oncoming is explicitly excluded (see
    # the comment above). Scenes whose turn cache is missing still fall back
    # to the straight goal and accept the regression — the alternative would
    # be to require map_cache for every left/right scene, which we don't yet
    # have for the non-junction-NPC 4 cases (032, 665, 091, 273).
    if rg is not None and ego_route is not None and W is not None \
            and maneuver in ("left", "right"):
        ids = _route_ids(ego_route)
        if ids:
            ego_goal_pos, _ = _lane_position_or_world(W, ids[-1])
    ego_goal_man = xosc.Maneuver(f"{EGO}_goal_man")
    ego_goal_man.add_event(_event(f"{EGO}_acquire_goal",
                                  _trig_simtime(EGO, 0.0),
                                  xosc.AcquirePositionAction(ego_goal_pos)))
    ego_goal_grp = xosc.ManeuverGroup(f"{EGO}_grp")
    ego_goal_grp.add_actor(EGO)
    ego_goal_grp.add_maneuver(ego_goal_man)
    act.add_maneuver_group(ego_goal_grp)

    for npc in scene.get("npcs", []):
        nid, kind = npc["id"], npc["kind"]
        block = (npc.get("behavior") or {}).get("block")
        is_junc = npc.get("position") in JUNCTION_POSITIONS or block in JUNCTION_BLOCKS
        is_static = block in ("stopped_ahead", "static_block", "static_hold")
        entities.add_scenario_object(nid, _entity(kind, nid))
        if is_junc:
            W, routes = rg
            # cache may emit routes referencing waypoint ids that are not in waypoints.json
            # (start/end on lane discretization edges); we only need ids[0] and ids[-1] for
            # WorldPosition teleport/acquire, so drop routes whose endpoints aren't resolvable.
            def _endpoints_resolvable(r):
                ids = _route_ids(r)
                return bool(ids) and ids[0] in W and ids[-1] in W

            base = [r for r in routes
                    if r.get("start_road_id") != ego_route.get("start_road_id")
                    and _endpoints_resolvable(r)]
            want_legs = _npc_leg_constraint(npc.get("position"), npc.get("side", "none"))
            want_types = _npc_type_constraint(block)
            leg_pool = [r for r in base if _leg_of_route(W, r, ego_route) in want_legs] if want_legs else base
            strict = [r for r in leg_pool if r.get("type") in want_types]
            # progressive fallback: relax type first, then leg
            chosen_list = strict or leg_pool or [r for r in base if r.get("type") in want_types] or base
            if not chosen_list:
                raise BlockUnsupported("no crossing route from a different leg")
            cr = chosen_list[0]
            ids = _route_ids(cr)
            if is_static:
                # static_hold at a junction position (e.g. stopped vehicle at the
                # opposing-leg stop line). Teleport at the END of the approach so
                # ego's turning path actually reaches it; skip controller, init
                # speed, and the cross AcquirePosition event so the NPC just sits.
                approach_ids = cr.get("approach_waypoint_ids") or []
                anchor_id = approach_ids[-1] if approach_ids else ids[0]
                sx, sy, sz, syaw = _wpos(W, anchor_id)
                init.add_init_action(nid, xosc.TeleportAction(
                    xosc.WorldPosition(x=sx, y=sy, z=sz + 0.2, h=math.radians(syaw))))
                events = block_events(npc)
            else:
                sx, sy, sz, syaw = _wpos(W, ids[0])
                init.add_init_action(nid, xosc.TeleportAction(
                    xosc.WorldPosition(x=sx, y=sy, z=sz + 0.2, h=math.radians(syaw))))
                _add_vehicle_controller(init, nid)
                init.add_init_action(nid, _speed(float(_params(npc).get("speed", 8.0))))
                # NPC goal: snap to a LanePosition on the exit road. Raw WorldPosition
                # from the route's last vertex often lands inside the junction body,
                # which makes the remote ScenarioManager crash on first tick with
                # IndexError: list index out of range when it tries to plan v2's path.
                acquire_pos, _ = _lane_position_or_world(W, ids[-1])
                ev = _event(f"{nid}_cross",
                            _trig_hero_distance(nid, float(_params(npc).get("trig_dist", 40.0)),
                                                xosc.RelativeDistanceType.cartesianDistance),
                            xosc.AcquirePositionAction(acquire_pos))
                events = [ev]
        else:
            init.add_init_action(nid, xosc.TeleportAction(resolve_position(npc)))
            if kind in ("vehicle", "cyclist") and not is_static:
                _add_vehicle_controller(init, nid)
                # Cruise-style NPCs start already moving so the "lead vehicle cruising → sudden
                # brake" narrative is physical; pairs with hero's Init cruise speed above.
                if block in CRUISE_BLOCKS:
                    init.add_init_action(nid, _speed(float(_params(npc).get("speed", 8.0)), 0.0))
            events = block_events(npc)
            if kind == "vehicle" and block == "front_brake":
                events.insert(0, _event(
                    f"{nid}_follow_lane",
                    _trig_simtime(nid, 0.0),
                    xosc.AcquirePositionAction(ego_goal_pos),
                ))
        if not events:
            continue
        man = xosc.Maneuver(f"{nid}_man")
        for e in events:
            man.add_event(e)
        grp = xosc.ManeuverGroup(f"{nid}_grp")
        grp.add_actor(nid)
        grp.add_maneuver(man)
        act.add_maneuver_group(grp)

    story.add_act(act)
    sb = xosc.StoryBoard(init, stoptrigger=_ego_travel_trigger(ACT_STOP_DIST + 20, "stop"))
    sb.add_story(story)

    catalog = xosc.Catalog()
    for c in ["VehicleCatalog", "ControllerCatalog", "PedestrianCatalog", "MiscObjectCatalog", "EnvironmentCatalog"]:
        catalog.add_catalog(c, CATALOG_DIR)

    # LogicFile carries just the basename: the runner rewrites it to the actual
    # map path at load time (runner/src/demo.py), and a bare filename keeps the
    # emitted XOSC free of machine-local absolute paths.
    sc = xosc.Scenario("scene_seed_block_scenario", "ads_testing", xosc.ParameterDeclarations(),
                       entities, sb, xosc.RoadNetwork(roadfile=Path(xodr_path).name), catalog, osc_minor_version=0)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    sc.write_xml(str(out_path))
    _patch_monitoring_criteria(out_path)
    _assert_hero_has_route_anchor(out_path)
    return out_path


def _assert_hero_has_route_anchor(xosc_path: Path) -> None:
    """Fail loudly if the emitted XOSC lacks a hero-owned ManeuverGroup carrying
    an AcquirePosition / FollowTrajectory / waypoint property.

    The remote PCLA route builder needs ≥2 route vertices for the SUT (Init
    Teleport + at least one anchor). Earlier emit paths (before this commit)
    silently dropped hero_grp for junction scenes, producing XSD-valid files
    that crash at remote `build_pcla_sut_route_from_xosc` with the message
    "hero 没有足够的 FollowTrajectory、AcquirePosition 或 waypoint property 顶点".
    Catching it here means stale/regressed files never reach CARLA.
    """
    import xml.etree.ElementTree as _ET
    root = _ET.parse(xosc_path).getroot()
    for mg in root.iter("ManeuverGroup"):
        actors_node = mg.find("Actors")
        if actors_node is None:
            continue
        if not any(er.attrib.get("entityRef") == EGO for er in actors_node.findall("EntityRef")):
            continue
        for tag in ("AcquirePositionAction", "FollowTrajectoryAction"):
            if next(iter(mg.iter(tag)), None) is not None:
                return
        if next((p for p in mg.iter("Property") if p.attrib.get("name", "").startswith("waypoint")), None):
            return
    raise BlockUnsupported(
        f"emitted xosc lacks a hero-owned route anchor "
        f"(AcquirePosition/FollowTrajectory/waypoint property); "
        f"remote PCLA route builder will fail. xosc={xosc_path}"
    )


DEMOS = {
    "rear_end": {"npcs": [
        {"id": "v2", "kind": "vehicle", "position": "ahead_same_lane",
         "behavior": {"block": "front_brake", "params": {"speed": 6.0, "trig_dist": 18.0}},
         "params": {"gap": 20.0}}]},
    "rear_hit": {"npcs": [
        {"id": "v2", "kind": "vehicle", "position": "behind_same_lane",
         "behavior": {"block": "rear_hit", "params": {"speed": 12.0}}, "params": {"gap": 14.0}}]},
    "cut_in": {"npcs": [
        {"id": "v2", "kind": "vehicle", "position": "adjacent", "side": "left",
         "behavior": {"block": "cut_in", "params": {"speed": 9.0, "trig_ttc": 3.0}}, "params": {"long": 6.0}}]},
    "vru_cross": {"npcs": [
        {"id": "ped", "kind": "pedestrian", "position": "roadside", "side": "right",
         "behavior": {"block": "cross", "params": {"speed": 1.5, "trig_dist": 15.0}},
         "params": {"gap": 18.0, "lateral": 4.0}}]},
    "static_hold_adjacent": {"npcs": [
        {"id": "v2", "kind": "vehicle", "position": "adjacent", "side": "left",
         "behavior": {"block": "static_hold"}, "params": {"long": 3.0}}]},
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", choices=list(DEMOS))
    ap.add_argument("--scene", type=Path, help="a scene_seed_v2 JSON (status=supported)")
    ap.add_argument("--xodr", required=True)
    ap.add_argument("--out", default="outputs/osc_blocks_demo/demo.xosc")
    ap.add_argument("--auto-extract", action="store_true",
                    help="if map_cache/<name>/ is missing, fetch it from the remote CARLA host "
                         "via tools.carla_remote.CarlaRemoteClient")
    args = ap.parse_args()
    name = None
    road_seed = None
    if args.scene:
        scene_path = args.scene if args.scene.is_absolute() else ROOT / args.scene
        seed = json.loads(scene_path.read_text())
        if seed.get("status") != "supported":
            print({"error": f"scene_seed status={seed.get('status')}"})
            return 2
        scene = seed["scene"]
        name = args.scene.stem
        # Auto-load road_seed.json from the same dir so WF6/WF7 (cross-metamodel
        # gate) can run. Pipeline-driven runs land both files in the same run dir;
        # ad-hoc CLI runs may not — we just skip the gate then.
        sibling_road = scene_path.parent / "road_seed.json"
        if sibling_road.is_file():
            try:
                road_seed = json.loads(sibling_road.read_text())
            except Exception:
                road_seed = None
    elif args.demo:
        scene = DEMOS[args.demo]
    else:
        print({"error": "need --demo or --scene"})
        return 2
    try:
        out = build_xosc(scene, args.xodr, ROOT / args.out, name=name,
                         auto_extract=args.auto_extract,
                         road_seed=road_seed)
    except WFViolation as exc:
        print({"out": None, "scene": str(args.scene) if args.scene else None,
               "status": "wf_violation", "reason": str(exc)})
        return 4
    except BlockUnsupported as exc:
        print({"out": None, "scene": str(args.scene) if args.scene else None,
               "status": "pending_roadgraph", "reason": str(exc)})
        return 3
    ok, err = validate_xosc(out, XSD)
    print({"out": str(out), "demo": args.demo, "scene": str(args.scene) if args.scene else None,
           "xsd_valid": ok, "error": err})
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
