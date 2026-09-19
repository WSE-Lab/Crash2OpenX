#!/usr/bin/env python3
"""Offline evaluator: proxy danger score from SceneSeed content.

Without CARLA available, we cannot measure real min-TTC / crashes. Instead we
score each variant by a physics-motivated proxy:

    danger = closing_intensity * environment_risk_multiplier
    novelty = 1 - max_similarity_to_archive(variant, archive)

The proxy is calibrated so:
    - a small trig_dist (imminent trigger) raises danger
    - high NPC speed raises danger
    - low friction / rain / night raises danger
    - repeated bins get 0 novelty (MAP-Elites will pick the max-danger elite)

Score is used ONLY to rank fuzz candidates offline. On a GPU host with CARLA,
plug the real four-dimension metrics (safety/effectiveness/compliance/comfort)
into ``score_from_carla()``.
"""
from __future__ import annotations

from typing import Any


WEATHER_RISK = {"clear": 1.0, "cloudy": 1.1, "rain": 1.5, "fog": 1.7, "snow": 1.8, "unknown": 1.0}
TOD_RISK = {"morning": 1.0, "afternoon": 1.0, "dawn": 1.3, "dusk": 1.3,
            "evening": 1.4, "night": 1.6, "unknown": 1.0}


def _get_speed(npc: dict) -> float:
    p = (npc.get("behavior") or {}).get("params") or {}
    for k in ("speed", "target_speed", "closing_speed"):
        v = p.get(k)
        if isinstance(v, (int, float)):
            return float(v)
    return 10.0  # default cruise speed


def _get_trigger(npc: dict) -> float:
    """Return a normalized 'triggerness' value in [0, 1]. Larger = more imminent."""
    p = (npc.get("behavior") or {}).get("params") or {}
    if "trig_dist" in p:
        # trig_dist ∈ [3, 40] → imminence 1..0
        return max(0.0, min(1.0, (40.0 - float(p["trig_dist"])) / 37.0))
    if "trig_ttc" in p:
        # trig_ttc ∈ [0.5, 5.0] → imminence 1..0
        return max(0.0, min(1.0, (5.0 - float(p["trig_ttc"])) / 4.5))
    return 0.4  # neutral if no trigger


def score_offline(scene_seed: dict) -> dict[str, float]:
    """Compute proxy score components + composite.

    Returns::
        {"danger": ..., "closing": ..., "env_mult": ..., "score": ...}
    """
    sc = scene_seed.get("scene") or {}
    env = sc.get("environment") or {}
    friction = env.get("friction_scale")
    if not isinstance(friction, (int, float)):
        friction = 1.0

    # Environment multiplier: low friction + risky weather + night raises danger
    env_mult = (
        WEATHER_RISK.get(env.get("weather"), 1.0)
        * TOD_RISK.get(env.get("time_of_day"), 1.0)
        * (2.0 - friction)  # friction=1.0 → 1.0x, friction=0.1 → 1.9x
    )

    # Closing intensity: max over NPCs of speed × triggerness
    closing = 0.0
    for npc in sc.get("npcs") or []:
        closing = max(closing, _get_speed(npc) * (0.5 + _get_trigger(npc)))
    if closing == 0.0:
        closing = 1.0

    danger = closing * env_mult
    return {
        "danger": round(danger, 3),
        "closing": round(closing, 3),
        "env_mult": round(env_mult, 3),
        "score": round(danger, 3),  # composite = danger for now; add novelty in archive
    }


def score_from_carla(carla_summary: dict, scene_outcome: dict) -> dict[str, float]:
    """Plug-in point for online (CARLA) scoring — used by GPU host.

    Wraps the 4-dimension rubric from the thesis proposal §4.3.2::
      safety       : 1/min_ttc + collision_flag
      effectiveness: 1 - route_completion
      compliance   : lane_invasion + wrong_lane + red_light
      comfort      : max_jerk / max_lateral_accel
    """
    s = carla_summary or {}
    o = scene_outcome or {}
    return {
        "safety": float((o.get("issues") and 10.0) or 1.0 / max(s.get("min_ttc", 5.0), 0.1)),
        "effectiveness": 1.0 - float(s.get("route_completion", 0.0)),
        "compliance": float(s.get("lane_invasion_count", 0) + (s.get("wrong_lane_count") or 0)),
        "comfort": float(s.get("max_jerk", 0.0)),
    }
