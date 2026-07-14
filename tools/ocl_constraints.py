#!/usr/bin/env python3
"""Executable semantics for the 15 OCL invariants in Table 1.

The paper's metamodel is intentionally represented with plain dictionaries so
the checks stay dependency-free and can be exercised in CI.  This module checks
OCL invariant semantics; XSD/type/multiplicity conformance remains the L1 gate
described in the paper.
"""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping


CONSTRAINT_IDS = tuple(
    [f"I{i}" for i in range(1, 10)] + [f"P{i}" for i in range(1, 7)]
)

JUNCTION_TOPOLOGIES = {"cross_intersection", "t_junction", "y_junction"}
PEDESTRIAN_POSITIONS = {
    "roadside_left",
    "roadside_right",
    "ahead_same_lane",
}
ADJACENT_POSITIONS = {"adjacent_left", "adjacent_right"}
JUNCTION_BLOCKS = {"junction_cross", "junction_turn"}
TURN_MANEUVERS = {"left", "right"}
LANE_CHANGE_MANEUVERS = {"lane_change_left", "lane_change_right"}


def _lanes(road: Mapping[str, Any]) -> tuple[Any, Any]:
    """Read the paper's flattened attributes or its JSON listing shape."""
    if isinstance(road.get("lanes"), Mapping):
        return road["lanes"].get("forward"), road["lanes"].get("backward")
    return road.get("lanes_forward"), road.get("lanes_backward")


def _number_in_range(value: Any, lower: float, upper: float) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and lower <= value <= upper


def _optional_nonnegative(value: Any) -> bool:
    return value is None or (
        isinstance(value, (int, float)) and not isinstance(value, bool) and value >= 0.0
    )


def _optional_positive(value: Any) -> bool:
    return value is None or (
        isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0.0
    )


def evaluate_constraints(
    road: Mapping[str, Any], scene: Mapping[str, Any]
) -> dict[str, bool]:
    """Evaluate I1--I9 and P1--P6 exactly as stated in Table 1."""
    forward, backward = _lanes(road)
    sut = scene.get("sut") if isinstance(scene.get("sut"), Mapping) else {}
    npcs = scene.get("npcs") if isinstance(scene.get("npcs"), list) else []
    collision = (
        scene.get("collision") if isinstance(scene.get("collision"), Mapping) else {}
    )
    environment = (
        scene.get("environment")
        if isinstance(scene.get("environment"), Mapping)
        else {}
    )

    ids = [npc.get("id") for npc in npcs if isinstance(npc, Mapping)]
    actor_ids = set(ids) | {sut.get("id")}
    striker = collision.get("striker_id")
    struck = collision.get("struck_id")

    params = []
    triggers = []
    for npc in npcs:
        if not isinstance(npc, Mapping):
            continue
        behavior = npc.get("behavior")
        if not isinstance(behavior, Mapping):
            continue
        if isinstance(behavior.get("params"), Mapping):
            params.append(behavior["params"])
        if isinstance(behavior.get("trigger"), Mapping):
            triggers.append(behavior["trigger"])

    has_junction_block = any(
        isinstance(npc, Mapping)
        and isinstance(npc.get("behavior"), Mapping)
        and npc["behavior"].get("block") in JUNCTION_BLOCKS
        for npc in npcs
    )
    has_oncoming = any(
        isinstance(npc, Mapping) and npc.get("position") == "oncoming"
        for npc in npcs
    )
    has_adjacent = any(
        isinstance(npc, Mapping) and npc.get("position") in ADJACENT_POSITIONS
        for npc in npcs
    )
    maneuver = sut.get("maneuver")
    topology = road.get("topology")

    return {
        "I1": _number_in_range(forward, 1, 5)
        and _number_in_range(backward, 0, 5),
        "I2": sut.get("id") == "ego",
        "I3": all(
            npc.get("kind") != "pedestrian"
            or npc.get("position") in PEDESTRIAN_POSITIONS
            for npc in npcs
            if isinstance(npc, Mapping)
        ),
        "I4": all(
            npc.get("kind") != "static"
            or (
                isinstance(npc.get("behavior"), Mapping)
                and npc["behavior"].get("block") == "static_block"
            )
            for npc in npcs
            if isinstance(npc, Mapping)
        ),
        "I5": len(ids) == len(set(ids)) and all(npc_id != sut.get("id") for npc_id in ids),
        "I6": {striker, struck}.issubset(actor_ids)
        and striker is not None
        and struck is not None
        and striker != struck,
        "I7": _number_in_range(environment.get("friction_scale"), 0.1, 1.0),
        "I8": all(
            _optional_nonnegative(param.get("target_speed"))
            and _optional_positive(param.get("duration"))
            for param in params
        ),
        "I9": all(
            isinstance(trigger.get("value"), (int, float))
            and not isinstance(trigger.get("value"), bool)
            and trigger["value"] >= 0.0
            for trigger in triggers
        ),
        "P1": not has_junction_block or topology in JUNCTION_TOPOLOGIES,
        "P2": not has_oncoming or _number_in_range(backward, 1, float("inf")),
        "P3": maneuver != "overtake_oncoming"
        or (
            _number_in_range(backward, 1, float("inf"))
            and road.get("center_line") == "broken"
        ),
        "P4": not has_adjacent or _number_in_range(forward, 2, float("inf")),
        "P5": maneuver not in TURN_MANEUVERS or topology in JUNCTION_TOPOLOGIES,
        "P6": maneuver not in LANE_CHANGE_MANEUVERS
        or _number_in_range(forward, 2, float("inf")),
    }


def violations(road: Mapping[str, Any], scene: Mapping[str, Any]) -> list[str]:
    return [key for key, passed in evaluate_constraints(road, scene).items() if not passed]


def paper_example() -> tuple[dict[str, Any], dict[str, Any]]:
    """Return the rainy-night example from Section 3 in executable form."""
    road = {
        "topology": "straight",
        "type": "town",
        "lanes": {"forward": 1, "backward": 1},
        "center_line": "broken",
    }
    scene = {
        "sut": {"id": "ego", "maneuver": "straight"},
        "npcs": [
            {
                "id": "v2",
                "kind": "vehicle",
                "position": "ahead_same_lane",
                "behavior": {
                    "block": "front_brake",
                    "params": {"target_speed": 0.0, "duration": 0.6},
                    "trigger": {"kind": "distance", "value": 9.0},
                },
            }
        ],
        "collision": {"striker_id": "ego", "struck_id": "v2"},
        "environment": {
            "weather": "rain",
            "time_of_day": "night",
            "friction_scale": 0.5,
        },
    }
    return deepcopy(road), deepcopy(scene)
