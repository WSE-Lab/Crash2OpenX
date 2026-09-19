#!/usr/bin/env python3
"""MAP-Elites archive for the fuzzer.

Behaviors are binned along 4 discrete axes; each cell keeps the highest-scoring
variant. This trades raw danger maximization for **behavioral coverage** — a
key requirement of the thesis (Sec. 3.2.2).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


TRIG_DIST_EDGES = (0.0, 5.0, 15.0, 30.0, 1e6)   # 4 buckets: <5, 5-15, 15-30, >30
SPEED_EDGES = (0.0, 5.0, 15.0, 25.0, 1e6)        # 4 buckets: <5, 5-15, 15-25, >25


def _bin(val: float, edges: tuple[float, ...]) -> int:
    for i in range(len(edges) - 1):
        if edges[i] <= val < edges[i + 1]:
            return i
    return len(edges) - 2


def feature_of(scene: dict) -> tuple[str, int, int, str]:
    """Extract the 4-axis behavior descriptor from a mutated scene."""
    sc = scene.get("scene") or {}
    npcs = sc.get("npcs") or []
    block = "empty"
    trig_dist = 30.0
    speed = 10.0
    if npcs:
        n0 = npcs[0]
        bh = n0.get("behavior") or {}
        p = bh.get("params") or {}
        block = bh.get("block") or "empty"
        if "trig_dist" in p:
            trig_dist = float(p["trig_dist"])
        elif "trig_ttc" in p:
            # Convert TTC(s) × speed(m/s) → approximate closing distance
            trig_dist = float(p["trig_ttc"]) * max(1.0, float(p.get("speed", p.get("target_speed", 10.0))))
        for k in ("speed", "target_speed", "closing_speed"):
            if k in p:
                speed = float(p[k])
                break
    weather = (sc.get("environment") or {}).get("weather", "unknown") or "unknown"
    return (block, _bin(trig_dist, TRIG_DIST_EDGES), _bin(speed, SPEED_EDGES), weather)


@dataclass
class ArchiveEntry:
    scene_seed: dict
    score: float
    ops_applied: list[str]
    features: tuple[str, int, int, str]


@dataclass
class MapElitesArchive:
    cells: dict[tuple, ArchiveEntry] = field(default_factory=dict)
    history: list[dict] = field(default_factory=list)  # per-step log for plots

    def submit(self, scene: dict, score: float, ops: list[str]) -> bool:
        """Try to place a variant into its cell. Returns True if it displaced (or filled)."""
        key = feature_of(scene)
        prev = self.cells.get(key)
        won = prev is None or score > prev.score
        if won:
            self.cells[key] = ArchiveEntry(scene, score, ops, key)
        self.history.append({
            "step": len(self.history),
            "features": key,
            "score": score,
            "displaced": bool(prev is not None and won),
            "won": bool(won),
            "cells_filled": len(self.cells),
            "best_score_so_far": max((e.score for e in self.cells.values()), default=0.0),
        })
        return won

    def as_records(self) -> list[dict]:
        return [
            {
                "features": list(e.features),
                "score": e.score,
                "ops_applied": e.ops_applied,
                "scene_seed": e.scene_seed,
            }
            for e in self.cells.values()
        ]
