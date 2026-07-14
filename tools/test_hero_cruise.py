#!/usr/bin/env python3
"""Static regression tests for _hero_cruise takeover-speed selection.

Reproduces the 2026-07-15 medoid-batch failure mode: scenes that declare a
collision into a static same-lane lead booted the hero at the 6 m/s fallback,
which lets every PCLA agent stop comfortably inside the 23 m spawn gap — the
declared rear-end could never be realized (uniform "min_distance≈14 m over
9 s" deadlocks across the batch). Run with:
    uv run pytest tools/test_hero_cruise.py
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.osc_blocks import (
    HERO_CLOSING_DELTA_MPS, HERO_STATIC_LEAD_TAKEOVER_MPS, _hero_cruise,
)


def _scene(npcs, collision=None):
    s = {"npcs": npcs}
    if collision is not None:
        s["collision"] = collision
    return s


def test_static_lead_with_declared_collision_boots_fast():
    scene = _scene(
        [{"id": "v1", "position": "ahead_same_lane",
          "behavior": {"block": "stopped_ahead", "params": {}}}],
        collision={"a": "ego", "b": "v1"},
    )
    assert _hero_cruise(scene) == HERO_STATIC_LEAD_TAKEOVER_MPS


def test_static_lead_without_collision_keeps_fallback():
    scene = _scene(
        [{"id": "v1", "position": "ahead_same_lane",
          "behavior": {"block": "stopped_ahead", "params": {}}}],
    )
    assert _hero_cruise(scene) == 6.0


def test_static_block_not_in_ego_lane_keeps_fallback():
    scene = _scene(
        [{"id": "v1", "position": "oncoming",
          "behavior": {"block": "static_block", "params": {}}}],
        collision={"a": "ego", "b": "v1"},
    )
    assert _hero_cruise(scene) == 6.0


def test_cruise_lead_still_wins_over_static_boost():
    scene = _scene(
        [{"id": "v1", "position": "ahead_same_lane",
          "behavior": {"block": "front_brake", "params": {"speed": 8.0}}},
         {"id": "v2", "position": "ahead_same_lane",
          "behavior": {"block": "stopped_ahead", "params": {}}}],
        collision={"a": "ego", "b": "v1"},
    )
    assert _hero_cruise(scene) == 8.0 + HERO_CLOSING_DELTA_MPS


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"  ✓ {name}")
    print("all hero-cruise tests passed")
