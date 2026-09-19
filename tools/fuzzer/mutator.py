#!/usr/bin/env python3
"""Mutation operators over SceneSeed.

Each operator takes a deep-copied SceneSeed dict and returns a *mutated copy*.
Operators only touch numeric params + environment enums; they never rewrite
structural fields (topology, position, block), so the OCL/WF gates remain the
authoritative legality check downstream.
"""
from __future__ import annotations

import copy
import random
from typing import Any, Callable

WEATHERS = ["clear", "rain", "snow", "fog", "cloudy"]
TIMES_OF_DAY = ["dawn", "morning", "afternoon", "evening", "dusk", "night"]

# (lo, hi) numeric bounds for known block params. Sourced from
# schemas/scene_seed_schema_v2.md and osc_blocks defaults.
NUMERIC_BOUNDS: dict[str, tuple[float, float]] = {
    "trig_dist": (3.0, 40.0),
    "trig_ttc": (0.5, 5.0),
    "trig_simtime": (0.5, 10.0),
    "speed": (0.0, 25.0),
    "target_speed": (0.0, 25.0),
    "end_speed": (0.0, 25.0),
    "decel": (2.0, 9.0),
    "brake_t": (0.2, 2.0),
    "encroach": (0.5, 3.0),
    "closing_speed": (5.0, 25.0),
    "gap": (5.0, 50.0),
    "lateral": (0.5, 4.0),
    "long": (5.0, 60.0),
}


def _truncated_gauss(mu: float, sigma: float, lo: float, hi: float, rng: random.Random) -> float:
    """Sample a Gaussian truncated to [lo, hi]. sigma is absolute, not fractional."""
    for _ in range(20):
        x = rng.gauss(mu, sigma)
        if lo <= x <= hi:
            return round(x, 2)
    return round(max(lo, min(hi, mu)), 2)


def _rand_param(key: str, current: float | None, rng: random.Random) -> float:
    lo, hi = NUMERIC_BOUNDS.get(key, (0.0, 10.0))
    if current is None:
        return round(rng.uniform(lo, hi), 2)
    return _truncated_gauss(current, 0.2 * (hi - lo), lo, hi, rng)


def mutate_trigger_dist(scene: dict, rng: random.Random) -> dict:
    """Nudge every NPC's trig_dist param (front_brake/cross)."""
    s = copy.deepcopy(scene)
    for npc in s.get("scene", {}).get("npcs", []) or []:
        p = (npc.get("behavior") or {}).setdefault("params", {})
        if (npc.get("behavior") or {}).get("block") in ("front_brake", "cross"):
            p["trig_dist"] = _rand_param("trig_dist", p.get("trig_dist"), rng)
    return s


def mutate_trigger_ttc(scene: dict, rng: random.Random) -> dict:
    """Nudge trig_ttc on cut_in / junction blocks."""
    s = copy.deepcopy(scene)
    for npc in s.get("scene", {}).get("npcs", []) or []:
        p = (npc.get("behavior") or {}).setdefault("params", {})
        if (npc.get("behavior") or {}).get("block") in ("cut_in", "junction_cross", "junction_turn"):
            p["trig_ttc"] = _rand_param("trig_ttc", p.get("trig_ttc"), rng)
    return s


def mutate_target_speed(scene: dict, rng: random.Random) -> dict:
    """Nudge NPC speed (all blocks that carry a speed param)."""
    s = copy.deepcopy(scene)
    for npc in s.get("scene", {}).get("npcs", []) or []:
        p = (npc.get("behavior") or {}).setdefault("params", {})
        # Prefer 'speed' (schema v2 name); fall back to 'target_speed' (paper name).
        key = "target_speed" if "target_speed" in p else "speed"
        current = p.get(key)
        if current is not None or (npc.get("behavior") or {}).get("block") not in ("stopped_ahead", "static_block"):
            p[key] = _rand_param(key, current, rng)
    return s


def mutate_environment(scene: dict, rng: random.Random) -> dict:
    """Nudge friction_scale continuously in [0.1, 1.0]."""
    s = copy.deepcopy(scene)
    env = s.get("scene", {}).setdefault("environment", {})
    cur = env.get("friction_scale")
    env["friction_scale"] = _truncated_gauss(cur if isinstance(cur, (int, float)) else 0.8, 0.2, 0.1, 1.0, rng)
    return s


def mutate_environment_weather_tod(scene: dict, rng: random.Random) -> dict:
    """Draw a fresh (weather, time_of_day) from the schema enums."""
    s = copy.deepcopy(scene)
    env = s.get("scene", {}).setdefault("environment", {})
    env["weather"] = rng.choice(WEATHERS)
    env["time_of_day"] = rng.choice(TIMES_OF_DAY)
    return s


MUTATORS: list[Callable[[dict, random.Random], dict]] = [
    mutate_trigger_dist,
    mutate_trigger_ttc,
    mutate_target_speed,
    mutate_environment,
    mutate_environment_weather_tod,
]


def mutate_scene(scene: dict, rng: random.Random, k: int = 1) -> tuple[dict, list[str]]:
    """Apply k random operators sequentially; return (mutated_scene, ops_applied)."""
    s = scene
    ops: list[str] = []
    for _ in range(k):
        op = rng.choice(MUTATORS)
        s = op(s, rng)
        ops.append(op.__name__)
    return s, ops
