#!/usr/bin/env python3
"""Static (CARLA-free) regression tests for the four 2026-06-26 pipeline fixes.

Each test reproduces a known failure mode + asserts the pipeline now catches
or auto-corrects it. Run with:
    uv run --with scenariogeneration --with xmlschema python tools/test_runtime_fixes.py
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import xml.etree.ElementTree as ET

import scenariogeneration.xosc as xosc

from tools.osc_blocks import (
    DEFAULT_GAP_M, WFViolation, _check_wf, build_xosc, resolve_ego_placement,
    resolve_position,
)


def _passing_road(forward=2, backward=1) -> dict:
    return {"road": {"topology": "straight", "type": "town",
                     "lanes": {"forward": forward, "backward": backward},
                     "center_line": "broken"}}


def test_wf8_adjacent_left_requires_forward_ge_2():
    """081 / 273 silent-spawn root cause: adjacent/left with forward=1 → lane 0 spawn."""
    scene = {"sut": {"id": "ego", "kind": "vehicle", "maneuver": "straight"},
             "npcs": [{"id": "v2", "kind": "vehicle", "position": "adjacent",
                       "side": "left", "behavior": {"block": "cut_in"}}]}
    issues = _check_wf(scene, _passing_road(forward=1, backward=1))
    assert any("WF8" in i for i in issues), f"WF8 should fire, got {issues}"
    # And with forward=2 the same scene should pass
    issues2 = _check_wf(scene, _passing_road(forward=2, backward=1))
    assert not any("WF8" in i for i in issues2), f"WF8 should NOT fire with forward=2, got {issues2}"
    print("  ✓ WF8 (adjacent/left ⇒ forward ≥ 2)")


def test_wf9_cut_in_long_clearance():
    """038 / 081 / 273 ZeroDivisionError root cause: cut_in long too small."""
    scene_bad = {"sut": {"id": "ego", "kind": "vehicle", "maneuver": "straight"},
                 "npcs": [{"id": "v2", "kind": "vehicle", "position": "adjacent",
                           "side": "right", "behavior": {"block": "cut_in"},
                           "params": {"long": 5.0}}]}  # explicit too-small long
    issues = _check_wf(scene_bad, _passing_road(forward=2))
    assert any("WF9" in i for i in issues), f"WF9 should fire on explicit long=5, got {issues}"

    # Default (no `long` key) ⇒ no WF9 violation; resolve_position auto-bumps.
    scene_default = {"sut": {"id": "ego", "kind": "vehicle", "maneuver": "straight"},
                     "npcs": [{"id": "v2", "kind": "vehicle", "position": "adjacent",
                               "side": "right", "behavior": {"block": "cut_in"}}]}
    issues_default = _check_wf(scene_default, _passing_road(forward=2))
    assert not any("WF9" in i for i in issues_default), f"WF9 must NOT fire without explicit long, got {issues_default}"
    print("  ✓ WF9 (cut_in long clearance, with auto-correct safety net)")


def test_cut_in_auto_corrected_long():
    """resolve_position bumps adjacent+cut_in long default 5 → 16 to clear trig_dist=15."""
    npc = {"id": "v2", "kind": "vehicle", "position": "adjacent", "side": "right",
           "behavior": {"block": "cut_in"}}
    pos = resolve_position(npc)
    assert pos.ds == 16.0, f"cut_in default long should be 16.0, got {pos.ds}"

    # Non-cut_in adjacent (e.g. static_hold) keeps legacy long=5
    npc2 = {"id": "v2", "kind": "vehicle", "position": "adjacent", "side": "right",
            "behavior": {"block": "static_hold"}}
    pos2 = resolve_position(npc2)
    assert pos2.ds == 5.0, f"non-cut_in adjacent long should stay 5.0, got {pos2.ds}"

    # Explicit override wins
    npc3 = {"id": "v2", "kind": "vehicle", "position": "adjacent", "side": "right",
            "behavior": {"block": "cut_in"}, "params": {"long": 22.0}}
    pos3 = resolve_position(npc3)
    assert pos3.ds == 22.0, f"explicit long override should win, got {pos3.ds}"
    print("  ✓ resolve_position auto-corrects cut_in adjacent long")


def test_ego_placement_picks_outer_lane_for_adjacent_left():
    """081 / 273 fix: ego must sit on outer forward lane when scene has adjacent/left."""
    xodr = ROOT / "outputs/opendrive_seed/081_Zoox_July_20_2024_(A).xodr"
    if not xodr.is_file():
        print(f"  · skipped (xodr missing: {xodr.name})")
        return
    scene_left = {"npcs": [{"id": "v2", "kind": "vehicle", "position": "adjacent",
                            "side": "left", "behavior": {"block": "cut_in"}}]}
    scene_right = {"npcs": [{"id": "v2", "kind": "vehicle", "position": "adjacent",
                             "side": "right", "behavior": {"block": "cut_in"}}]}
    _, lane_left, _ = resolve_ego_placement(str(xodr), scene_left)
    _, lane_right, _ = resolve_ego_placement(str(xodr), scene_right)
    # 081 has forward=2 → neg lanes [-1, -2]; left should pick -2 (outer), right -1 (inner)
    assert lane_left == -2, f"adjacent/left scene should place ego on outer (lane -2), got {lane_left}"
    assert lane_right == -1, f"adjacent/right scene should keep ego on inner (lane -1), got {lane_right}"
    print(f"  ✓ resolve_ego_placement: left→{lane_left}, right→{lane_right}")


def test_oncoming_ego_s_clearance():
    """444 / 665 IndexError fix: ego_s must be ≥ gap + EGO_EDGE_MARGIN + 5 for oncoming."""
    xodr = ROOT / "outputs/opendrive_seed/444_Waymo_November_6_2021_(1).xodr"
    if not xodr.is_file():
        print(f"  · skipped (xodr missing: {xodr.name})")
        return
    scene_oncoming = {"npcs": [{"id": "v1", "kind": "vehicle", "position": "oncoming",
                                "side": "left", "behavior": {"block": "static_hold"},
                                "params": {"gap": 15.0}}]}
    _, _, ego_s = resolve_ego_placement(str(xodr), scene_oncoming)
    # Need ego_s ≥ gap (15) + EGO_EDGE_MARGIN (5) + 5 = 25
    assert ego_s >= 25.0, f"oncoming scene should give ego_s ≥ 25, got {ego_s}"
    print(f"  ✓ resolve_ego_placement oncoming clearance: ego_s={ego_s} ≥ 25")


def test_cut_in_event_has_simtime_gate():
    """Cut event's ConditionGroup must contain BOTH distance + simtime conditions.

    Without the simtime gate, CARLA fires RelativeLaneChangeAction at t=0
    when adjacent NPC is already within trig_dist of ego → ZeroDivisionError.
    """
    scene = {"sut": {"id": "ego", "kind": "vehicle", "maneuver": "straight"},
             "npcs": [{"id": "v2", "kind": "vehicle", "position": "adjacent",
                       "side": "right", "behavior": {"block": "cut_in"}}],
             "environment": {"weather": "clear", "time_of_day": "afternoon"}}
    out_path = ROOT / "outputs/medoid_xosc/_test_cut_in_gate.xosc"
    xodr = ROOT / "outputs/opendrive_seed/038_Waymo_December_17_2024.xodr"
    if not xodr.is_file():
        print(f"  · skipped (xodr missing: {xodr.name})")
        return
    build_xosc(scene, str(xodr), out_path, road_seed=_passing_road(forward=2, backward=0))

    root = ET.parse(out_path).getroot()
    # Find the cut event by name suffix
    cut_event = next((e for e in root.iter("Event") if e.attrib.get("name", "").endswith("_cut")), None)
    assert cut_event is not None, "no _cut event found"
    cgs = list(cut_event.iter("ConditionGroup"))
    assert cgs, "no ConditionGroup in cut event"
    conds = list(cgs[0])
    cond_types = [c.tag for c in conds]
    assert "Condition" in cond_types or len(conds) >= 2, (
        f"cut ConditionGroup should have both entity + value conditions, got: {cond_types}"
    )
    # Sub-condition inspection: at least one RelativeDistanceCondition + one SimulationTimeCondition
    sub = [child.tag for cond in conds for child in cond.iter()]
    assert any("RelativeDistance" in s for s in sub), f"distance cond missing: {sub}"
    assert any("SimulationTime" in s for s in sub), f"simtime cond missing: {sub}"
    out_path.unlink()
    print("  ✓ cut event has both distance + simtime conditions ANDed")


def test_front_brake_has_lane_follow_goal():
    """A cruising lead vehicle must follow the generated road before braking."""
    xodr = ROOT / "outputs/opendrive_seed/038_Waymo_December_17_2024.xodr"
    if not xodr.is_file():
        print(f"  · skipped (xodr missing: {xodr.name})")
        return
    scene = {
        "sut": {"id": "ego", "kind": "vehicle", "maneuver": "straight"},
        "npcs": [{
            "id": "v2", "kind": "vehicle", "position": "ahead_same_lane",
            "side": "none", "behavior": {"block": "front_brake"},
        }],
        "environment": {"weather": "clear", "time_of_day": "afternoon"},
    }
    out_path = ROOT / "outputs/medoid_xosc/_test_front_brake_goal.xosc"
    build_xosc(scene, str(xodr), out_path, road_seed=_passing_road(forward=1, backward=0))
    root = ET.parse(out_path).getroot()
    follow = next((event for event in root.iter("Event")
                   if event.attrib.get("name") == "v2_follow_lane"), None)
    assert follow is not None, "front_brake NPC lane-follow event missing"
    assert follow.find(".//AcquirePositionAction") is not None, (
        "front_brake NPC must receive an AcquirePositionAction before braking"
    )
    assert next((event for event in root.iter("Event")
                if event.attrib.get("name") == "v2_brake"), None) is not None, (
        "front_brake event must remain present"
    )
    out_path.unlink()
    print("  ✓ front_brake NPC follows the lane before braking")


def test_front_brake_supports_simtime_trigger():
    """Reports that say traffic stopped abruptly can brake after a short cruise."""
    from tools.osc_blocks import block_events

    npc = {
        "id": "v2", "kind": "vehicle", "position": "ahead_same_lane",
        "behavior": {"block": "front_brake", "params": {"trig_simtime": 2.0}},
    }
    events = block_events(npc)
    assert len(events) == 1, f"front_brake should emit one brake event, got {len(events)}"
    xml = ET.tostring(events[0].get_element(), encoding="unicode")
    assert "SimulationTimeCondition" in xml, f"simtime trigger missing: {xml[:300]}"
    assert 'value="2.0"' in xml, f"simtime value missing: {xml[:300]}"
    print("  ✓ front_brake supports an explicit abrupt-stop time")


def test_default_gap_bumped_to_25():
    """2026-06-27 fix: default gap moved from 15 m to 25 m so PCLA's external_control
    handoff has ~3s acceleration window before reaching the trigger distance.
    With gap=15 the 18-medoid CARLA batch routinely showed ego frozen at
    min_distance ≈ 15.4 m for the whole episode (444 / 665 / 081 / 193, etc.).

    D3 (2026-06-27 round 2): split position-aware — behind_same_lane (rear_hit
    NPC) reverts to 15 m so a slower trailing vehicle can close the gap
    within 60 s; oncoming uses 20 m. A later CARLA boundary test gives static
    stopped_ahead uses 23 m and front_brake uses a 20 m crash-test setup;
    other dynamic ahead actors retain 25 m."""
    assert DEFAULT_GAP_M == 25.0, f"DEFAULT_GAP_M fallback should be 25.0, got {DEFAULT_GAP_M}"
    # stopped_ahead: behavior-specific 23 m
    npc_ahead = {"id": "v2", "kind": "vehicle", "position": "ahead_same_lane",
                 "behavior": {"block": "stopped_ahead"}}
    assert abs(resolve_position(npc_ahead).ds) == 23.0, "stopped_ahead default should be 23"
    # front_brake uses the validated 20 m abrupt-stop setup
    npc_brake = {"id": "v2", "kind": "vehicle", "position": "ahead_same_lane",
                 "behavior": {"block": "front_brake"}}
    assert abs(resolve_position(npc_brake).ds) == 20.0, "front_brake default should be 20"
    # behind_same_lane (rear_hit): D3 override 15 m
    npc_behind = {"id": "v2", "kind": "vehicle", "position": "behind_same_lane",
                  "behavior": {"block": "rear_hit"}}
    assert abs(resolve_position(npc_behind).ds) == 15.0, "behind_same_lane default should be 15 (D3)"
    # oncoming: D3 override 20 m
    npc_oncoming = {"id": "v2", "kind": "vehicle", "position": "oncoming",
                    "behavior": {"block": "oncoming"}}
    assert abs(resolve_position(npc_oncoming).ds) == 20.0, "oncoming default should be 20 (D3)"
    print("  ✓ DEFAULT_GAP: stopped_ahead=23, front_brake=20, behind=15, oncoming=20")


def test_ego_goal_uses_roadgraph_exit_for_left_right():
    """2026-06-27 fix: for left/right maneuvers with a RoadGraph cache, ego's
    AcquirePosition goal must point at the EXIT lane of the matched route,
    not the legacy 200m-ahead point in the same lane. Without this, PCLA's
    target sits past the junction in the original lane and the planner trips
    WrongLane / lane-invasion (observed on 665 and 435, 2026-06-26)."""
    cid = "435_Waymo_November_26_2021"  # cross_intersection + left, HAS map_cache
    xodr = ROOT / f"outputs/opendrive_seed/{cid}.xodr"
    scene_path = ROOT / f"outputs/scene_seed/{cid}.json"
    if not (xodr.is_file() and scene_path.is_file()
            and (ROOT / f"outputs/map_cache/{cid}/route_candidates.json").is_file()):
        print(f"  · skipped (artifacts missing for {cid})")
        return
    import json
    seed = json.loads(scene_path.read_text())
    scene = seed["scene"]
    road_seed = {"road": {"topology": "cross_intersection",
                          "lanes": {"forward": 1, "backward": 1},
                          "center_line": "broken"}}
    out_path = ROOT / "outputs/medoid_xosc/_test_ego_goal_exit.xosc"
    build_xosc(scene, str(xodr), out_path, name=cid, road_seed=road_seed)

    root = ET.parse(out_path).getroot()
    acquire = next((e for e in root.iter("Event")
                    if e.attrib.get("name") == "hero_acquire_goal"), None)
    assert acquire is not None, "hero_acquire_goal event missing"
    # The retargeted goal lives on an exit road different from ego's spawn road.
    # Spawn road is in the LanePosition of hero's TeleportAction.
    teleports = [t for t in root.iter("TeleportAction")]
    hero_teleport = None
    for pa in root.iter("Private"):
        if pa.attrib.get("entityRef") == "hero":
            hero_teleport = pa.find(".//TeleportAction//LanePosition")
            break
    assert hero_teleport is not None, "could not find hero spawn LanePosition"
    spawn_road = hero_teleport.attrib.get("roadId")

    goal_pos = acquire.find(".//AcquirePositionAction/Position/LanePosition")
    assert goal_pos is not None, (
        f"hero goal should be a LanePosition (rg exit), got: "
        f"{ET.tostring(acquire.find('.//AcquirePositionAction'), encoding='unicode')[:200]}"
    )
    goal_road = goal_pos.attrib.get("roadId")
    assert goal_road != spawn_road, (
        f"hero goal road ({goal_road}) should DIFFER from spawn road ({spawn_road}) "
        f"for a left/right maneuver with RoadGraph cache"
    )
    out_path.unlink()
    print(f"  ✓ ego goal retargeted: spawn road={spawn_road} → exit road={goal_road}")


def test_wf10_oncoming_on_turning_junction():
    """665 root cause: ego turns at T-junction, NPC at position=oncoming stays
    on ego's ORIGINAL road, scripted collision can never happen + cyclist is
    invisible to ego's camera after the turn. WF10 must reject this combo."""
    scene_bad = {
        "sut": {"id": "ego", "kind": "vehicle", "maneuver": "right"},
        "npcs": [{"id": "b1", "kind": "cyclist", "position": "oncoming",
                  "behavior": {"block": "oncoming"}}],
    }
    road_t = {"road": {"topology": "t_junction", "type": "town",
                       "lanes": {"forward": 1, "backward": 1}, "center_line": "broken"}}
    issues = _check_wf(scene_bad, road_t)
    assert any("WF10" in i for i in issues), f"WF10 should fire on T-junction + right + oncoming, got {issues}"

    # cross_intersection + left + oncoming: same problem
    scene_bad2 = {
        "sut": {"id": "ego", "kind": "vehicle", "maneuver": "left"},
        "npcs": [{"id": "v2", "kind": "vehicle", "position": "oncoming",
                  "behavior": {"block": "oncoming"}}],
    }
    road_cross = {"road": {"topology": "cross_intersection", "type": "town",
                           "lanes": {"forward": 1, "backward": 1}, "center_line": "broken"}}
    issues2 = _check_wf(scene_bad2, road_cross)
    assert any("WF10" in i for i in issues2), f"WF10 should fire on cross + left + oncoming, got {issues2}"

    # straight + oncoming on the same road: legitimate (e.g. 444), MUST NOT fire
    scene_ok = {
        "sut": {"id": "ego", "kind": "vehicle", "maneuver": "straight"},
        "npcs": [{"id": "v1", "kind": "vehicle", "position": "oncoming",
                  "behavior": {"block": "static_hold"}}],
    }
    road_straight = {"road": {"topology": "straight", "type": "town",
                              "lanes": {"forward": 1, "backward": 1}, "center_line": "broken"}}
    issues3 = _check_wf(scene_ok, road_straight)
    assert not any("WF10" in i for i in issues3), f"WF10 must NOT fire on straight + oncoming, got {issues3}"

    # T-junction + left + opposing_leg: the geometrically-correct alternative
    # WF10 suggests — must pass too
    scene_ok2 = {
        "sut": {"id": "ego", "kind": "vehicle", "maneuver": "left"},
        "npcs": [{"id": "v2", "kind": "vehicle", "position": "opposing_leg",
                  "behavior": {"block": "static_hold"}}],
    }
    issues4 = _check_wf(scene_ok2, road_t)
    assert not any("WF10" in i for i in issues4), f"WF10 must NOT fire on T + left + opposing_leg, got {issues4}"
    print("  ✓ WF10 (turn + oncoming on junction ⇒ rejected; straight + oncoming + opposing_leg pass)")


def test_d2_closing_speed_defaults():
    """2026-06-27 (D2): rear_hit default speed bumped 8→14 m/s so the
    trailing NPC closes ego (cruise 6 m/s) over the 60 s episode; oncoming
    bumped 8→12 m/s so head-on relative speed is ~18 m/s instead of ~14.
    Explicit params.speed always wins."""
    from tools.osc_blocks import block_events
    # rear_hit
    rear = {"id": "v2", "kind": "vehicle", "position": "behind_same_lane",
            "behavior": {"block": "rear_hit"}}
    events = block_events(rear)
    assert events, "rear_hit must emit an event"
    # Walk the event xml to the AbsoluteTargetSpeed value
    xml_str = ET.tostring(events[0].get_element(), encoding="unicode")
    assert 'value="14.0"' in xml_str, f"rear_hit default speed should be 14.0 in xml: {xml_str[:200]}"

    # oncoming
    onc = {"id": "v1", "kind": "vehicle", "position": "oncoming",
           "behavior": {"block": "oncoming"}}
    events = block_events(onc)
    assert events, "oncoming must emit an event"
    xml_str = ET.tostring(events[0].get_element(), encoding="unicode")
    assert 'value="12.0"' in xml_str, f"oncoming default speed should be 12.0 in xml: {xml_str[:200]}"

    # Explicit override wins
    rear_override = {"id": "v2", "kind": "vehicle", "position": "behind_same_lane",
                     "behavior": {"block": "rear_hit", "params": {"speed": 9.0}}}
    events = block_events(rear_override)
    xml_str = ET.tostring(events[0].get_element(), encoding="unicode")
    assert 'value="9.0"' in xml_str, f"explicit speed=9 override should win, got: {xml_str[:200]}"
    print("  ✓ D2 closing speeds: rear_hit=14, oncoming=12, explicit overrides honoured")


def test_no_zero_length_lanesections():
    """Generator's t_junction occasionally emits a duplicate laneSection at
    s = road.length. Build's `patch_strip_zero_length_lanesections` must
    delete them — CARLA's UE4 mesh baker stalls indefinitely otherwise
    (observed 2026-06-26 on case 153: 90s warmup timeout repro'd, even
    after server restart, until the patch landed).
    """
    # Just verify on the already-generated 18 medoid xodrs (all should have
    # been rebuilt with the patch active). Iterate every <road> and assert
    # no <laneSection s=...> sits at or past <road length=...>.
    import xml.etree.ElementTree as _ET
    seed_dir = ROOT / "outputs/opendrive_seed"
    if not seed_dir.is_dir():
        print("  · skipped (no opendrive_seed dir)")
        return
    n_checked = 0
    n_clean = 0
    for xodr_path in sorted(seed_dir.glob("*.xodr")):
        root = _ET.parse(xodr_path).getroot()
        for r in root.findall("./road"):
            try:
                length = float(r.get("length", "0"))
            except (TypeError, ValueError):
                continue
            for sec in r.findall("./lanes/laneSection"):
                try:
                    s = float(sec.get("s", "0"))
                except (TypeError, ValueError):
                    continue
                assert s < length - 1e-3, (
                    f"{xodr_path.name} road id={r.get('id')} has zero-length "
                    f"laneSection at s={s} (length={length})"
                )
                n_checked += 1
        n_clean += 1
    print(f"  ✓ {n_clean} xodrs, {n_checked} laneSections — no zero-length tail")


def main() -> int:
    print("=== 2026-06-26 runtime-fix regression tests ===\n")
    test_wf8_adjacent_left_requires_forward_ge_2()
    test_wf9_cut_in_long_clearance()
    test_cut_in_auto_corrected_long()
    test_ego_placement_picks_outer_lane_for_adjacent_left()
    test_oncoming_ego_s_clearance()
    test_cut_in_event_has_simtime_gate()
    test_front_brake_has_lane_follow_goal()
    test_front_brake_supports_simtime_trigger()
    test_default_gap_bumped_to_25()
    test_ego_goal_uses_roadgraph_exit_for_left_right()
    test_wf10_oncoming_on_turning_junction()
    test_d2_closing_speed_defaults()
    test_no_zero_length_lanesections()
    print("\n=== all 13 static checks passed ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
