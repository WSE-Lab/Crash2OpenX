#!/usr/bin/env python3
"""OpenSCENARIO block assembler (CARLA-runnable).

scene_seed (position x behavior(block+params)) -> parameterized OpenSCENARIO fragments
placed RELATIVE to ego, assembled into one XOSC modeled on CARLA's official examples
(FollowLeadingVehicle / PedestrianCrossingFront / LaneChangeSimple):
  - ego = external_control (handed to the ADS under test)
  - NPC vehicles = npc_vehicle_control + SpeedAction/LaneChangeAction
  - triggers fire on the HERO entity (condition references the NPC)
  - Environment with Sun; scenario_runner criteria_* monitoring patched in

Blocks have default trigger types and thresholds. An explicit behavior.ads_trigger
replaces the hazard onset with a live ADS-relative clearance/speed window.

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
from tools.scene_contacts import CONTACT_PARAMETER, runtime_contacts
from tools.scene_positions import ordered_npcs
from tools.ads_trigger import build_ads_trigger, normalize_ads_trigger, trigger_manifest, PARAMETER as ADS_TRIGGER_PARAMETER
EGO_ROAD, EGO_LANE, EGO_S0 = 1, -1, 30.0
CATALOG_DIR = "openscenarios/catalogs"
ACT_STOP_DIST = 200.0
# Distance ego stays away from BOTH road ends. Remote scenario_runner's
# atomic_criteria.update() calls lane_waypoint.next(2.0)[0] every tick — at the
# road boundary that returns [] and raises IndexError on the first tick. 5 m is
# enough on every map we've seen to keep a 2 m forward lookup inside the lane.
EGO_EDGE_MARGIN = 5.0
# Runway ego keeps BEHIND its spawn point, i.e. its minimum distance from the
# road's s=0 end (2026-08-05, from the 42-pattern batch evidence for 065 / 162 /
# 203). Those three don't fail the way the old `next(2.0)` comment above
# assumes — they never get near the far end. What happens is:
#   1. ego boots at _hero_cruise (12 m/s here) and the PCLA agent loses lane
#      keeping within ~4 s, swerving across the centerline (19 / 13 / 15 lane
#      invasions recorded). That drift IS the test result — deadlock class C,
#      an ADS lane-keep deficiency the tool correctly exposed.
#   2. the spin-out carries ego ~14-16 m BACKWARDS along the lane. With the old
#      10 m floor that puts it past s=0, off the road start.
#   3. straddling the centerline just off the road start, WrongLaneTest's
#      `self._map.get_waypoint(...)` snaps to the OPPOSING lane (whose driving
#      direction is s-decreasing) clamped to s=0. Its lane end is right there,
#      so `lane_waypoint.next(2.0)` is empty and `[0]` raises IndexError —
#      terminating the run at ~8 s with `exception:IndexError`.
# So the crash doesn't cause the failure, it MASKS it: a legitimate class-C
# finding is reported as a pipeline exception instead. 30 m absorbs the observed
# 16 m worst-case rearward excursion and still leaves 170 m of forward road on
# the 200 m templates (the `length - s < ahead` fallback below covers shorter ones).
EGO_REAR_RUNWAY_M = 30.0
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
    s = max(behind, needs_oncoming_clear, EGO_REAR_RUNWAY_M)
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

# Takeover speed for scenes that declare a collision into a static same-lane
# lead. The 6 m/s fallback lets every PCLA agent stop comfortably inside the
# 23 m stopped_ahead spawn gap, so the declared rear-end could never be
# realized (observed 2026-07-15 across the 42-medoid batch: uniform
# "min_distance≈14 m over 9 s" deadlocks). 12 m/s puts the gap at the edge of
# the braking envelope — the declared impact becomes kinematically possible
# while an attentive ADS can still avoid it, which is exactly what the
# execution gate is meant to discriminate.
HERO_STATIC_LEAD_TAKEOVER_MPS = 12.0


def _hero_cruise(scene: dict) -> float:
    params = ((scene.get("sut") or {}).get("params") or {})
    # Source-derived speeds take precedence over the legacy takeover heuristic.
    # An initial stop may coexist with a later reported cruising speed.
    for key in ("initial_speed_mps", "speed"):
        if key in params:
            value = params[key]
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not math.isfinite(value) or value < 0):
                raise ValueError(f"sut.params.{key} must be finite and non-negative")
            return float(value)
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
        # Declared rear-end into a static same-lane lead: see
        # HERO_STATIC_LEAD_TAKEOVER_MPS above.
        static_lead = any(
            (n.get("behavior") or {}).get("block") in ("stopped_ahead", "static_block", "static_hold")
            and n.get("position") == "ahead_same_lane"
            for n in (scene.get("npcs") or [])
        )
        if static_lead and scene.get("collision"):
            return HERO_STATIC_LEAD_TAKEOVER_MPS
        return 6.0
    return max(speeds) + HERO_CLOSING_DELTA_MPS


# ----------------------------- roadgraph (junction mode, option B data) -----------------------------

import re as _re

JUNCTION_POSITIONS = {"cross", "opposing_leg"}
JUNCTION_BLOCKS = {"junction_cross", "junction_turn", "junction_merge"}
# scene maneuver -> route.type as emitted by outputs/map_cache/<name>/route_candidates.json
MANEUVER_TO_ROUTE_TYPE = {"straight": "straight", "left": "left_turn", "right": "right_turn"}


def _route_type(route: dict) -> str:
    """Accept the extractor's left/right names and older *_turn caches."""
    value = route.get("type", route.get("turn_type", ""))
    return {"left": "left_turn", "right": "right_turn"}.get(value, value)


def _junction_route_action(W: dict, route: dict, actor_id: str):
    """Assign the selected lane sequence before an NPC enters the junction."""
    ids = _route_ids(route)
    if not ids or any(wid not in W for wid in ids):
        raise BlockUnsupported("junction route contains unresolved waypoints")
    route_action = xosc.Route(actor_id + "_junction_route")
    for wid in ids:
        # The extractor rounds s values. At a connector endpoint this can turn
        # a valid sampled waypoint into s == road.length, which CARLA's XODR
        # lookup rejects. Preserve the actual sampled centerline transform;
        # these points still come exclusively from this map's CARLA roadgraph.
        wx, wy, wz, yaw = _wpos(W, wid)
        route_action.add_waypoint(
            xosc.WorldPosition(x=wx, y=wy, z=wz + 0.2, h=math.radians(yaw)),
            xosc.RouteStrategy.shortest,
        )
    return xosc.AssignRouteAction(route_action)


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


def _waiting_junction_route(W, route, distance):
    """Start on a sampled approach waypoint near the junction entrance.

    Keep the selected connector and exit; trim only the unused approach.
    Distance is measured backwards along the extracted approach centerline.
    """
    if isinstance(distance, bool) or not isinstance(distance, (int, float)) or not math.isfinite(distance) or distance < 5:
        raise BlockUnsupported('junction approach_distance_m must be finite and at least 5 m')
    approach = route.get('approach_waypoint_ids', [])
    ids = _route_ids(route)
    if len(approach) < 2 or any(wid not in W or wid not in ids for wid in approach):
        raise BlockUnsupported('waiting junction actor requires a sampled approach')
    remaining = 0.0
    anchor = None
    for left, right in zip(reversed(approach[:-1]), reversed(approach[1:])):
        a, b = _wpos(W, left), _wpos(W, right)
        remaining += math.hypot(a[0]-b[0], a[1]-b[1])
        if remaining >= distance:
            anchor = left
            break
    if anchor is None:
        raise BlockUnsupported('junction approach is shorter than approach_distance_m')
    return {**route, 'waypoint_ids': ids[ids.index(anchor):],
            'approach_waypoint_ids': approach[approach.index(anchor):]}


def _corridor_route_from_spawn(W, road_id, lane_id, station):
    """Follow a uniquely connected sampled corridor, including curved roads."""
    direction = 1 if lane_id < 0 else -1
    candidates = [wid for wid, w in W.items() if w['road_id'] == road_id
                  and w['lane_id'] == lane_id and direction*(float(w['s'])-station) >= -1e-4]
    if not candidates:
        raise BlockUnsupported('No sampled corridor after NPC spawn')
    first = min(candidates, key=lambda wid: abs(float(W[wid]['s'])-station))
    ids, seen = [first], {first}
    while True:
        following = [wid for wid in W[ids[-1]].get('next_ids', []) if wid in W and wid not in seen
                     and W[wid].get('lane_type', '').lower() == 'driving']
        if not following:
            break
        if len(following) != 1 or W[following[0]].get('is_junction'):
            raise BlockUnsupported('Corridor has a branch; use a generated junction route')
        ids.append(following[0]); seen.add(following[0])
    if len(ids) < 2:
        raise BlockUnsupported('Insufficient sampled corridor after NPC spawn')
    return {'waypoint_ids': ids, 'start_road_id': road_id, 'start_lane_id': lane_id}


def _straight_route_from_spawn(W, routes, road_id, lane_id, station):
    route = next((r for r in routes if r.get('start_road_id') == road_id
                  and r.get('start_lane_id') == lane_id and _route_type(r) == 'straight'), None)
    if route is None:
        raise BlockUnsupported('No generated straight route for continuing partial-intrusion participant')
    ids = _route_ids(route)
    direction = 1 if lane_id < 0 else -1
    first = next((i for i, wid in enumerate(ids) if W[wid]['road_id'] != road_id
                  or direction*(float(W[wid]['s'])-station) >= -1e-4), len(ids))
    if len(ids)-first < 2:
        raise BlockUnsupported('Continuing participant has insufficient generated route after its spawn')
    ids = ids[first:]
    # Route candidates stop a short distance after the junction. A continuing
    # participant must follow the rest of that same extracted departure lane;
    # otherwise the lateral controller steers back toward the truncated endpoint.
    # Never select a new junction branch or synthesize trajectory coordinates.
    end = W[ids[-1]]
    seen = set(ids)
    while True:
        candidates = [wid for wid in W[ids[-1]].get('next_ids', [])
                      if wid in W and W[wid]['road_id'] == end['road_id']
                      and W[wid]['lane_id'] == end['lane_id']]
        if not candidates:
            # Junction-route samples and the full-road samples can use slightly
            # different stations (e.g. 239.98 vs 240.00). A dangling next ID is
            # not the road end. Continue on the same extracted lane by station,
            # with the same spatial gap bound as the roadgraph self-check.
            current = W[ids[-1]]
            travel = 1 if int(end['lane_id']) < 0 else -1
            ahead = [(travel*(float(node['s'])-float(current['s'])), wid)
                     for wid,node in W.items() if wid not in seen
                     and node['road_id'] == end['road_id'] and node['lane_id'] == end['lane_id']
                     and node.get('section_id') == current.get('section_id')
                     and travel*(float(node['s'])-float(current['s'])) > .05]
            if not ahead:
                break
            gap, wid = min(ahead)
            x,y,_,yaw = _wpos(W,ids[-1]); nx,ny,_,nyaw = _wpos(W,wid)
            if gap > 3.5 or math.hypot(nx-x,ny-y) > 3.5 or abs((nyaw-yaw+180)%360-180) > 2:
                raise BlockUnsupported('Discontinuous extracted straight departure lane')
            candidates = [wid]
        if len(candidates) != 1 or candidates[0] in seen:
            raise BlockUnsupported('Ambiguous or cyclic generated departure lane')
        ids.append(candidates[0])
        seen.add(candidates[0])
    return {**route, 'waypoint_ids':ids}


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
    if position in {"opposing_leg", "oncoming"}:
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

def _vehicle(name: str, ego: bool, vehicle_class: str | None = None) -> xosc.Vehicle:
    from tools.scene_actor_catalog import VEHICLE_CLASSES
    vehicle_class = vehicle_class or "car"
    if vehicle_class not in VEHICLE_CLASSES:
        raise ValueError(f"unknown vehicle_class {vehicle_class!r}")
    # Vehicle/@name is a CARLA blueprint selector, not the ScenarioObject id.
    # A role such as "tesla" matched Cybertruck; unknown roles invoked the
    # runner's random fallback. Keep default passenger cars deterministic.
    name, category = VEHICLE_CLASSES[vehicle_class]
    v = xosc.Vehicle(name, getattr(xosc.VehicleCategory, category), xosc.BoundingBox(2.1, 4.5, 1.8, 1.5, 0.0, 0.9),
                     xosc.Axle(0.5, 0.6, 1.8, 3.1, 0.3), xosc.Axle(0.0, 0.6, 1.8, 0.0, 0.3), 69.4, 200.0, 10.0)
    v.add_property("type", "ego_vehicle" if ego else "simulation")
    return v


def _pedestrian(name: str) -> xosc.Pedestrian:
    p = xosc.Pedestrian(name, 90.0, xosc.PedestrianCategory.pedestrian,
                        xosc.BoundingBox(0.6, 0.6, 1.8, 0.0, 0.0, 0.9), model="walker.pedestrian.0001")
    p.add_property("type", "simulation")
    return p


def _entity(kind: str, name: str, vehicle_class: str | None = None):
    if kind == "pedestrian":
        return _pedestrian(name)
    if kind == "cyclist":
        # The runner selects the CARLA blueprint from Vehicle/@name. Merely
        # adding semantic_type to a car still spawned a four-wheel sedan.
        v = xosc.Vehicle("vehicle.diamondback.century", xosc.VehicleCategory.bicycle,
                         xosc.BoundingBox(0.6, 1.8, 1.8, 0.0, 0.0, 0.9),
                         xosc.Axle(0.7, 0.7, 0.1, 0.6, 0.35),
                         xosc.Axle(0.0, 0.7, 0.1, -0.6, 0.35), 15.0, 4.0, 8.0)
        v.add_property("type", "simulation")
        v.add_property("semantic_type", "cyclist")
        return v
    return _vehicle(name, False, vehicle_class)


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

def _relative_lane_delta(npc: dict, ego_lane_id: int) -> int:
    """Map semantic left/right/opposing lanes in either travel direction."""
    if ego_lane_id == 0:
        raise BlockUnsupported("ego cannot occupy the OpenDRIVE center lane")
    toward_center = 1 if ego_lane_id < 0 else -1
    if npc.get("position") == "adjacent":
        return toward_center if npc.get("side") == "left" else -toward_center
    if npc.get("position") == "oncoming":
        opposing_lane = 1 if ego_lane_id < 0 else -1
        return opposing_lane - ego_lane_id
    return 0


def _route_supports_relative_positions(route: dict, scene: dict, lane_ids: dict) -> bool:
    ego_lane = int(route["start_lane_id"])
    available = lane_ids.get(str(route["start_road_id"]), set())
    for npc in scene.get("npcs", []):
        if npc.get("position") in {"adjacent", "oncoming"}:
            target = ego_lane + _relative_lane_delta(npc, ego_lane)
            if target == 0 or target not in available:
                return False
            if npc["position"] == "adjacent" and target * ego_lane < 0:
                return False
    return True


def _junction_merge_route(W: dict, routes: list, ego_route: dict, npc: dict) -> dict:
    """Find an adjacent incoming lane joining the SUT's actual exit lane.

    Uses only route candidates extracted from this generated map. This is a
    junction turn/merge, not a lateral lane-change over disconnected roads.
    """
    if npc.get("position") != "adjacent":
        raise BlockUnsupported("junction_merge requires an adjacent incoming lane")
    if _route_type(ego_route) not in {"left_turn", "right_turn"}:
        raise BlockUnsupported("junction_merge requires a turning SUT route")
    lane = int(ego_route["start_lane_id"]) + _relative_lane_delta(npc, int(ego_route["start_lane_id"]))
    ego_end = W[_route_ids(ego_route)[-1]]
    for route in routes:
        ids = _route_ids(route)
        if not ids or any(wid not in W for wid in ids):
            continue
        end = W[ids[-1]]
        if (str(route["start_road_id"]) == str(ego_route["start_road_id"])
                and int(route["start_lane_id"]) == lane
                and _route_type(route) == _route_type(ego_route)
                and str(end["road_id"]) == str(ego_end["road_id"])
                and int(end["lane_id"]) == int(ego_end["lane_id"])):
            return route
    raise BlockUnsupported("no adjacent junction route merges into the SUT exit lane")


def resolve_position(npc: dict, ego_lane_id: int = -1, reference_actor: str = EGO):
    pos, side = npc.get("position"), npc.get("side", "none")
    p = _params(npc)
    gap = float(p.get("gap", _default_gap_for(npc)))
    if pos == "ahead_same_lane":
        return xosc.RelativeLanePosition(0, reference_actor, ds=gap)
    if pos == "behind_same_lane":
        return xosc.RelativeLanePosition(0, reference_actor, ds=-gap)
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
        return xosc.RelativeLanePosition(_relative_lane_delta(npc, ego_lane_id), reference_actor,
                                         ds=float(p.get("long", default_long)))
    if pos == "oncoming":
        # scenario_runner walks ds along the TARGET lane's driving direction.
        # An oncoming vehicle ahead of ego must be placed opposite that walk;
        # positive ds put it behind ego, heading farther away from the scene.
        return xosc.RelativeLanePosition(_relative_lane_delta(npc, ego_lane_id), reference_actor,
                                         ds=-gap, orientation=xosc.Orientation(h=0))
    if pos == "roadside":
        if npc.get('kind') == 'cyclist' and (npc.get('behavior') or {}).get('block') == 'cruise':
            gap = float(p.get('gap', 0.0))
        off = float(p.get("lateral", 3.5)) * (1 if side == "left" else -1)
        if ego_lane_id > 0:
            off = -off  # OpenDRIVE t is relative to the road reference direction.
        heading = npc.get("parked_heading")
        if heading is not None and heading not in {"parallel", "opposite", "perpendicular"}:
            raise ValueError(f"invalid parked_heading {heading!r}")
        crosses = (npc.get("behavior") or {}).get("block") == "cross"
        if heading == "opposite":
            h = math.pi
        elif heading == "perpendicular" or (heading is None and crosses):
            # OSC heading is right-handed; the parser negates it for CARLA.
            # A person on the right faces left, toward the carriageway.
            h = math.pi / 2 * (-1 if side == "left" else 1)
        else:
            h = 0.0  # Parked vehicles and along-road actors face along the lane.
        return xosc.RelativeLanePosition(0, reference_actor, ds=gap, offset=off, orientation=xosc.Orientation(h=h))
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


def _trig_event_end(event_name: str) -> xosc.Trigger:
    condition = xosc.StoryboardElementStateCondition(
        xosc.StoryboardElementType.event, event_name, xosc.StoryboardElementState.endTransition)
    trigger = xosc.ValueTrigger(event_name + "_finished", 0.0, xosc.ConditionEdge.rising, condition)
    group = xosc.ConditionGroup(); group.add_condition(trigger)
    result = xosc.Trigger(); result.add_conditiongroup(group); return result


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


def _cut_in_lane_change(npc: dict) -> xosc.RelativeLaneChangeAction:
    """Lane-change action that moves an `adjacent` NPC into ego's lane.

    Two scenario_runner facts shape this emission — both were observed as
    hard runtime failures on the 42-pattern batch (2026-07-15):

    1. `openscenario_parser` reads only `RelativeTargetLane/@value` and drops
       `@entityRef`, then sets `lane_changes = abs(value)`. Emitting the
       ego-relative form (`value=0 entityRef="hero"`, i.e. "end up in ego's
       lane") is standard-legal but leaves `lane_changes=0`, and
       `generate_target_waypoint_list_multilane` divides the lane-change
       length by it → `ZeroDivisionError` at the first tick. So the value has
       to be expressed relative to the NPC itself: ∓1, sign chosen so the
       parser's `direction = "left" if value > 0 else "right"` points at ego.
       RHT lane ids decrease rightwards, so an NPC on ego's left (dLane=+1)
       merges right (value=-1) and one on ego's right merges left (value=+1).

    2. `ChangeActorLateralMotion` consumes only the *distance* form of
       `LaneChangeActionDynamics`; with `dynamicsDimension="time"` the parser
       leaves `distance=inf`, `waypoint.next(inf)` returns no waypoints and
       the atomic installs an empty plan — the NPC silently never cuts in.
       Emit metres instead.
    """
    side = npc.get("side", "none")
    value = -1 if side == "left" else 1
    length = float(_params(npc).get("cut_dist", 20.0))
    dyn = xosc.TransitionDynamics(xosc.DynamicsShapes.linear,
                                  xosc.DynamicsDimension.distance, length)
    return xosc.RelativeLaneChangeAction(value, npc["id"], dyn)


def _validate_lateral_sequence(npc, xodr_path, road_id, lane_id):
    """Reject missing/non-traversable lateral destinations before execution."""
    road = ET.parse(xodr_path).find(f"road[@id='{road_id}']")
    if road is None or road.get('junction', '-1') != '-1' or road.find('.//arc') is not None or road.find('.//spiral') is not None:
        raise BlockUnsupported('lateral sequences currently require a straight non-junction road')
    lanes = {int(l.get('id')): l.get('type') for l in road.findall('./lanes/laneSection/*/lane')}
    current = lane_id
    for step in npc['behavior']['steps']:
        if step['action'] != 'lane_change':
            continue
        params = step['params']
        delta = (1 if params['direction'] == 'left' else -1) * (1 if current < 0 else -1)
        for _ in range(params.get('lanes', 1)):
            target = current + delta
            if target * current <= 0 or lanes.get(target) not in {'driving', 'parking'}:
                raise BlockUnsupported('lateral sequence requires existing same-direction driving/parking lanes')
            current = target


def _sequence_events(npc: dict) -> list[xosc.Event]:
    from tools.scene_sequences import normalize_steps
    steps = normalize_steps(npc['behavior'].get('steps'))
    nid = npc['id']
    events = []
    canonical = lambda name: EGO if name == 'ego' else name
    for index, step in enumerate(steps):
        name = f'{nid}_step_{index+1}'
        when, params = step['when'], step['params']
        group = xosc.ConditionGroup()
        if index == 0:
            group.add_condition(xosc.ValueTrigger(name+'_start', 0, xosc.ConditionEdge.rising,
                xosc.SimulationTimeCondition(0, xosc.Rule.greaterThan)))
        else:
            group.add_condition(xosc.ValueTrigger(name+'_after_previous', float(when.get('delay', 0)),
                xosc.ConditionEdge.rising, xosc.StoryboardElementStateCondition(
                    xosc.StoryboardElementType.event, f'{nid}_step_{index}',
                    xosc.StoryboardElementState.completeState)))
        condition = when['condition']
        if condition == 'contact':
            group.add_condition(xosc.EntityTrigger(name+'_contact', 0, xosc.ConditionEdge.rising,
                xosc.CollisionCondition(canonical(when['target'])), nid))
        elif condition == 'separated_and_target_stopped':
            target = canonical(when['target'])
            group.add_condition(xosc.EntityTrigger(name+'_separated', 0, xosc.ConditionEdge.rising,
                xosc.RelativeDistanceCondition(float(when.get('clearance', 2)), xosc.Rule.greaterThan,
                    xosc.RelativeDistanceType.cartesianDistance, target, freespace=True), nid))
            group.add_condition(xosc.EntityTrigger(name+'_target_stopped', 0, xosc.ConditionEdge.rising,
                xosc.StandStillCondition(float(when.get('standstill_duration', .5))), target))
        trigger = xosc.Trigger(); trigger.add_conditiongroup(group)
        duration = float(params.get('duration', 1.0))
        if step['action'] == 'match_speed':
            action = xosc.RelativeSpeedAction(1.0, canonical(step['target']), _lin(duration),
                valuetype=xosc.SpeedTargetValueType.factor, continuous=False)
        elif step['action'] == 'brake':
            action = _speed(float(params.get('end_speed', 0)), duration)
        elif step['action'] == 'lane_change':
            delta = int(params.get('lanes', 1)) * (1 if params['direction'] == 'left' else -1)
            dynamics = xosc.TransitionDynamics(xosc.DynamicsShapes.sinusoidal,
                xosc.DynamicsDimension.distance, float(params.get('distance', 20)))
            action = xosc.RelativeLaneChangeAction(delta, nid, dynamics)
        else:
            action = _speed(float(params.get('speed', 8)), duration)
        events.append(_event(name, trigger, action))
    return events


def block_events(npc: dict, *, intrusion=None) -> list[xosc.Event]:
    nid = npc["id"]
    block = npc.get("behavior", {}).get("block")
    p = _params(npc)
    speed = float(p.get("speed", 8.0))
    ads_trigger = build_ads_trigger(npc) if normalize_ads_trigger(npc) is not None else None
    if block == 'sequence':
        return _sequence_events(npc)
    if block == 'partial_lane_intrusion':
        if intrusion is None:
            raise BlockUnsupported('partial intrusion requires a generated-road geometry plan')
        return [_event(nid+'_go', _trig_simtime(nid), _speed(speed)),
                _event(nid+'_partial_intrusion', ads_trigger or _trig_simtime(nid, 1.5),
                       xosc.AbsoluteLaneOffsetAction(intrusion['target_offset_xodr_m'],
                           xosc.DynamicsShapes.sinusoidal,
                           intrusion['max_lateral_acceleration_mps2'], continuous=True))]
    if block == "cruise":
        cruise_speed = float(p.get("speed", 3.0 if npc.get("kind") == "cyclist" else 8.0))
        return [_event(f"{nid}_cruise", _trig_simtime(nid), _speed(cruise_speed))]
    if block == "front_brake":
        # Cruise is now set in Init (see CRUISE_BLOCKS in build_xosc), so this event only
        # needs to fire the brake when hero closes in — no keep event, no storyboard-state
        # gating, no t=0 double-trigger race.
        if ads_trigger is not None:
            brake_trigger = ads_trigger
        elif "trig_simtime" in p:
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
        cut = _event(f"{nid}_cut", ads_trigger or cut_trig, _cut_in_lane_change(npc))
        return [go, cut]
    if block in ("cross", "walk_along"):
        return [_event(f"{nid}_move", ads_trigger or _trig_hero_distance(nid, float(p.get("trig_dist", 15.0)),
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
    if sut_maneuver in {"overtake_oncoming", "overtake_solid_centerline"}:
        if backward < 1:
            issues.append(
                f"WF7: sut.maneuver={sut_maneuver} requires road.lanes.backward≥1, got {backward}"
            )
        expected_center = "solid" if sut_maneuver == "overtake_solid_centerline" else "broken"
        if center != expected_center:
            issues.append(
                f"WF7: sut.maneuver={sut_maneuver} requires road.center_line={expected_center}, got '{center}'"
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
            f"use position=cross on the correct incoming leg with block=static_hold for a stopped "
            f"participant or junction_cross/junction_turn for a moving participant instead"
        )

    return issues


# ----------------------------- assembly -----------------------------

def build_xosc(scene: dict, xodr_path: str, out_path: Path,
               name: str | None = None,
               *,
               auto_extract: bool = False,
               remote_client=None,
               road_seed: dict | None = None) -> Path:
    ads_plans = trigger_manifest(scene)
    entities = xosc.Entities()
    init = xosc.Init()
    init.add_global_action(_build_env_action({"environment": _env_for_compile(scene.get("environment") or {})}))

    # Cross-metamodel WF gate (WF6, WF7). Skipped silently when no road_seed
    # is passed; legacy callers that only pass scene+xodr keep working.
    if road_seed is not None:
        wf_issues = _check_wf(scene, road_seed)
        if wf_issues:
            raise WFViolation(" ; ".join(wf_issues))

    # A straight SUT also has to traverse a generated intersection. Without
    # this, same-lane scenarios stopped at the end of the incoming road.
    junction = scene_needs_junction(scene) or bool(ET.parse(xodr_path).findall('junction'))
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
    curved = bool(ET.parse(xodr_path).findall('.//planView/geometry/arc')
                  or ET.parse(xodr_path).findall('.//planView/geometry/spiral'))
    wants_rg = junction or maneuver in ("left", "right") or curved
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

    entities.add_scenario_object(EGO, _vehicle(EGO, True, (scene.get("sut") or {}).get("vehicle_class")))
    ego_route = None
    W = None
    if rg:
        W, routes = rg
        route_type = MANEUVER_TO_ROUTE_TYPE.get(maneuver, "straight")
        lane_ids = {road.get("id"): {int(l.get("id")) for l in road.findall(".//lane")
                    if l.get("type") == "driving"}
                    for road in ET.parse(xodr_path).getroot().findall("road")}
        ego_route = next((r for r in routes if _route_type(r) == route_type
                          and _route_supports_relative_positions(r, scene, lane_ids)), None)
        if ego_route is None and (junction or maneuver in ("left", "right")):
            raise BlockUnsupported(f"no {route_type} route in roadgraph for ego supporting the NPC lane positions")

    if ego_route is not None and W is not None:
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
        # The extracted approach already contains an interior spawn anchor.
        # Read its actual s in either lane direction; subtracting an arbitrary
        # 40 m only for ego desynchronizes otherwise symmetric crossing routes.
        anchor = W.get(ap[0], {}) if ap else {}
        ap_s = float(anchor.get("s", _s_of(ap[0]) if ap else 20.0))
        ego_s = max(EGO_EDGE_MARGIN, ap_s)
        if road_len is not None:
            ego_s = min(road_len - EGO_EDGE_MARGIN, ego_s)
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
                   starttrigger=_trig_simtime("scene_act", 0.0),
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
    # Every selected junction route, including straight, ends on its exit leg.
    # A goal clamped to the incoming road stops the SUT before the junction.
    if rg is not None and ego_route is not None and W is not None \
            and (junction or maneuver in ("left", "right")):
        ids = _route_ids(ego_route)
        if ids:
            ego_goal_pos, _ = _lane_position_or_world(W, ids[-1])
    ego_goal_man = xosc.Maneuver(f"{EGO}_goal_man")
    ego_action = ( _junction_route_action(W, ego_route, EGO)
                  if ego_route is not None and W is not None else xosc.AcquirePositionAction(ego_goal_pos))
    ego_goal_man.add_event(_event(f"{EGO}_acquire_goal",
                                  _trig_simtime(EGO, 0.0),
                                  ego_action))
    ego_goal_grp = xosc.ManeuverGroup(f"{EGO}_grp")
    ego_goal_grp.add_actor(EGO)
    ego_goal_grp.add_maneuver(ego_goal_man)
    act.add_maneuver_group(ego_goal_grp)

    spawn_lane_ids = {'ego': ego_lane_id}
    spawn_stations = {'ego': ego_s}
    crossing_plans = {}
    intrusion_plans = {}
    for npc in ordered_npcs(scene.get("npcs", [])):
        nid, kind = npc["id"], npc["kind"]
        block = (npc.get("behavior") or {}).get("block")
        is_junc = npc.get("position") in JUNCTION_POSITIONS or block in JUNCTION_BLOCKS
        is_static = block in ("stopped_ahead", "static_block", "static_hold")
        entities.add_scenario_object(nid, _entity(kind, nid, npc.get("vehicle_class")))
        if is_junc:
            W, routes = rg
            # cache may emit routes referencing waypoint ids that are not in waypoints.json
            # (start/end on lane discretization edges); we only need ids[0] and ids[-1] for
            # WorldPosition teleport/acquire, so drop routes whose endpoints aren't resolvable.
            def _endpoints_resolvable(r):
                ids = _route_ids(r)
                return bool(ids) and ids[0] in W and ids[-1] in W

            if block == "junction_merge":
                cr = _junction_merge_route(W, routes, ego_route, npc)
            else:
                same_approach = npc.get('position') == 'ahead_same_lane'
                base = [r for r in routes if _endpoints_resolvable(r)
                        and ((r.get('start_road_id') == ego_route.get('start_road_id')
                              and r.get('start_lane_id') == ego_route.get('start_lane_id'))
                             if same_approach else r.get('start_road_id') != ego_route.get('start_road_id'))]
                want_legs = _npc_leg_constraint(npc.get("position"), npc.get("side", "none"))
                want_types = _npc_type_constraint(block)
                leg_pool = [r for r in base if _leg_of_route(W, r, ego_route) in want_legs] if want_legs else base
                chosen_list = [r for r in leg_pool if _route_type(r) in want_types]
                if not chosen_list:
                    raise BlockUnsupported(f"no NPC route matching leg={sorted(want_legs)} and maneuver={sorted(want_types)}")
                cr = chosen_list[0]
                if same_approach and 'approach_distance_m' not in _params(npc):
                    gap = float(_params(npc).get('gap', DEFAULT_GAP_M))
                    direction = 1 if ego_lane_id < 0 else -1
                    approach = cr.get('approach_waypoint_ids', [])
                    first = next((wid for wid in approach
                                  if direction*(float(W[wid]['s'])-ego_s) >= gap), None)
                    if not math.isfinite(gap) or gap <= 0 or first is None:
                        raise BlockUnsupported('No generated same-lane turning spawn ahead of ego')
                    route_ids = _route_ids(cr)
                    cr = {**cr, 'waypoint_ids': route_ids[route_ids.index(first):],
                          'approach_waypoint_ids': approach[approach.index(first):]}
            if 'approach_distance_m' in _params(npc):
                if nid not in ads_plans or is_static:
                    raise BlockUnsupported('approach_distance_m requires a moving ADS-triggered junction actor')
                cr = _waiting_junction_route(W, cr, _params(npc)['approach_distance_m'])
            ids = _route_ids(cr)
            spawn_lane_ids[nid] = int(cr['start_lane_id'])
            spawn_stations[nid] = float(W[ids[0]]['s'])
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
                _add_vehicle_controller(init, nid, generated_lane_route=[{
                    'road_id':int(W[wid]['road_id']), 'lane_id':int(W[wid]['lane_id']),
                    's':float(W[wid]['s']), 'x':_wpos(W,wid)[0], 'y':_wpos(W,wid)[1],
                    'yaw':_wpos(W,wid)[3]} for wid in ids])
                init.add_init_action(nid, _speed(0.0 if nid in ads_plans else float(_params(npc).get("speed", 8.0))))
                # The NPC already moves from Init. A distance-delayed routing
                # action lets LocalPlanner pick a random turn before the intended
                # route is installed. Use the extracted lane sequence from t=0.
                ev = _event(f"{nid}_cross",
                            _trig_simtime(nid, 0.0),
                            _junction_route_action(W, cr, nid))
                # Finite generated maps have no continuation beyond the route.
                # Once the selected route completes, stop instead of allowing
                # the controller to invent a path and eventually drive off mesh.
                stop = _event(f"{nid}_route_stop", _trig_event_end(f"{nid}_cross"), _speed(0.0))
                events = [ev, stop]
                if nid in ads_plans:
                    # Install the generated route at t=0, then depart only when
                    # ADS enters the window. Never delay route installation.
                    departure_speed = float(_params(npc).get('speed', 8.0))
                    events.insert(1, _event(nid+'_depart', build_ads_trigger(npc),
                                          _speed(departure_speed, max(1.0, departure_speed/3.0))))
        else:
            reference = npc.get('relative_to', 'ego')
            reference_lane = spawn_lane_ids[reference]
            spawn_lane_ids[nid] = reference_lane + _relative_lane_delta(npc, reference_lane)
            reference_actor = EGO if reference == 'ego' else reference
            pedestrian_cross = kind == 'pedestrian' and block == 'cross'
            if pedestrian_cross:
                if reference != 'ego' or npc.get('position') != 'roadside':
                    raise BlockUnsupported('finite pedestrian cross currently requires roadside relative to ego')
                from tools.pedestrian_crossing import crossing_geometry
                parameters = _params(npc)
                try:
                    plan = crossing_geometry(xodr_path, ego_road_id, reference_lane, ego_s,
                        side=npc.get('side'), gap=parameters.get('gap', DEFAULT_GAP_M),
                        lateral=parameters.get('lateral'))
                except ValueError as exc:
                    raise BlockUnsupported(str(exc)) from exc
                crossing_plans[nid] = plan
                heading = math.pi/2 * (-1 if plan['side']=='left' else 1)
                position = xosc.LanePosition(plan['s'], plan['start_offset'], reference_lane,
                                             ego_road_id, orientation=xosc.Orientation(h=heading))
            else:
                position = resolve_position(npc, reference_lane, reference_actor)
            init.add_init_action(nid, xosc.TeleportAction(position))
            relative_position = position.get_element().find('RelativeLanePosition')
            if relative_position is not None:
                spawn_stations[nid] = spawn_stations[reference] + (
                    1 if spawn_lane_ids[nid] < 0 else -1) * float(relative_position.get('ds'))
            continuing_route = None
            alongside_intruder = (kind == 'cyclist' and block == 'cruise'
                and any(n.get('id') == reference and (n.get('behavior') or {}).get('block') == 'partial_lane_intrusion'
                        for n in scene.get('npcs', [])))
            # A longitudinal NPC on an incoming lane must not pick a random
            # junction turn. Its source maneuver remains lane following.
            longitudinal = kind == 'vehicle' and (block in {'cruise','front_brake','cut_in'} or
                (block == 'sequence' and all(step['action'] != 'lane_change'
                                            for step in npc['behavior']['steps'])))
            has_straight = rg and any(r.get('start_road_id') == ego_road_id
                and r.get('start_lane_id') == spawn_lane_ids[nid] and _route_type(r) == 'straight'
                for r in routes)
            if rg and (block == 'partial_lane_intrusion' or alongside_intruder or
                       (longitudinal and has_straight)):
                continuing_route = _straight_route_from_spawn(W, routes, ego_road_id,
                    spawn_lane_ids[nid], spawn_stations[nid])
            elif rg and not junction and longitudinal:
                continuing_route = _corridor_route_from_spawn(W, ego_road_id,
                    spawn_lane_ids[nid], spawn_stations[nid])
            if kind in ("vehicle", "cyclist") and not is_static:
                initial_offset = None
                if kind == 'cyclist' and npc.get('position') == 'roadside' and block == 'cruise':
                    initial_offset = float(_params(npc).get('lateral', 3.5)) * (1 if npc['side']=='right' else -1)
                generated_route = ([{'road_id':int(W[wid]['road_id']), 'lane_id':int(W[wid]['lane_id']),
                    's':float(W[wid]['s']), 'x':_wpos(W,wid)[0], 'y':_wpos(W,wid)[1],
                    'yaw':_wpos(W,wid)[3]} for wid in _route_ids(continuing_route)] if continuing_route else None)
                _add_vehicle_controller(init, nid, initial_lane_offset=initial_offset, generated_lane_route=generated_route,
                                        max_brake=_params(npc).get('max_brake'))
                # Cruise-style NPCs start already moving so the "lead vehicle cruising → sudden
                # brake" narrative is physical; pairs with hero's Init cruise speed above.
                if block in CRUISE_BLOCKS:
                    init.add_init_action(nid, _speed(float(_params(npc).get("speed", 8.0)), 0.0))
            if block == 'partial_lane_intrusion':
                from tools.partial_lane_intrusion import intrusion_plan
                try:
                    intrusion_plans[nid] = intrusion_plan(npc, xodr_path, ego_road_id, spawn_lane_ids[nid])
                except ValueError as exc:
                    raise BlockUnsupported(str(exc)) from exc
            events = block_events(npc, intrusion=intrusion_plans.get(nid))
            if continuing_route:
                events.insert(0, _event(nid+'_straight_route', _trig_simtime(nid,0.0),
                                       _junction_route_action(W, continuing_route, nid)))
            if pedestrian_cross:
                condition = xosc.TraveledDistanceCondition(plan['distance_m'])
                trigger = xosc.EntityTrigger(nid+'_crossed_road', 0.0,
                    xosc.ConditionEdge.rising, condition, nid)
                events.append(_event(nid+'_cross_stop', trigger, _speed(0.0, 0.0)))
            lateral_sequence = block == 'sequence' and any(
                step['action'] == 'lane_change' for step in npc['behavior']['steps'])
            if lateral_sequence:
                _validate_lateral_sequence(npc, xodr_path, ego_road_id, spawn_lane_ids[nid])
            if (kind == "vehicle" and block in {"front_brake", "sequence"}
                    and not lateral_sequence and not continuing_route):
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
    declarations = xosc.ParameterDeclarations()
    declarations.add_parameter(xosc.Parameter('C2XADSActor', xosc.ParameterType.string, EGO))
    if ads_plans:
        declarations.add_parameter(xosc.Parameter(ADS_TRIGGER_PARAMETER, xosc.ParameterType.string, json.dumps(ads_plans)))
    declarations.add_parameter(xosc.Parameter('C2XSourceManeuver', xosc.ParameterType.string, maneuver))
    if crossing_plans:
        declarations.add_parameter(xosc.Parameter('C2XPedestrianCrossings', xosc.ParameterType.string,
                                                  json.dumps(crossing_plans)))
    if intrusion_plans:
        declarations.add_parameter(xosc.Parameter('C2XPartialLaneIntrusions', xosc.ParameterType.string,
                                                  json.dumps(intrusion_plans)))
    if scene.get('collisions'):
        declarations.add_parameter(xosc.Parameter(CONTACT_PARAMETER, xosc.ParameterType.string,
                                                  json.dumps(runtime_contacts(scene))))
    sc = xosc.Scenario("scene_seed_block_scenario", "ads_testing", declarations,
                       entities, sb, xosc.RoadNetwork(roadfile=Path(xodr_path).name), catalog, osc_minor_version=0)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    sc.write_xml(str(out_path))
    _patch_monitoring_criteria(out_path)
    _assert_hero_has_route_anchor(out_path)
    return out_path


def _assert_hero_has_route_anchor(xosc_path: Path) -> None:
    """Fail loudly if the emitted XOSC lacks a hero-owned ManeuverGroup carrying
    an AssignRoute / AcquirePosition / FollowTrajectory / waypoint property.

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
        for tag in ("AssignRouteAction", "AcquirePositionAction", "FollowTrajectoryAction"):
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
