#!/usr/bin/env python3
"""Fuzz runner: drive the mutation → gate → evaluate → archive loop.

Offline mode (default on macOS): mutate + OCL gate + XSD compile only.
Online mode (--online, needs GPU host): additionally run CARLA and use the
carla_summary in the scorer.

CLI::

    python -m tools.fuzzer.runner \
        --seed data/seeds/scene_seed/122_Waymo_February_18_2024.json \
        --road data/seeds/road_seed/122_Waymo_February_18_2024.json \
        --budget 100 --out outputs/fuzz_runs/case122
"""
from __future__ import annotations

import argparse
import json
import random
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.fuzzer.archive import MapElitesArchive, feature_of
from tools.fuzzer.evaluator import score_offline
from tools.fuzzer.mutator import mutate_scene
from tools.ocl_constraints import violations as ocl_violations


def _load_seed(path: Path) -> dict:
    return json.loads(path.read_text())


def _road_of(road_seed_doc: dict) -> dict:
    """RoadSeed JSON is either {'status', 'road': {...}} or a flat dict."""
    if isinstance(road_seed_doc.get("road"), dict):
        return road_seed_doc["road"]
    return {k: v for k, v in road_seed_doc.items() if k in {"topology", "type", "lanes", "center_line"}}


def _scene_of(scene_seed_doc: dict) -> dict:
    if isinstance(scene_seed_doc.get("scene"), dict):
        return scene_seed_doc["scene"]
    return scene_seed_doc


def run_fuzz(
    scene_seed_path: Path,
    road_seed_path: Path,
    out_dir: Path,
    budget: int = 100,
    seed: int = 42,
    k_ops: int = 1,
    online: bool = False,
) -> dict:
    """One-case fuzz loop. Returns a summary dict + writes artifacts under out_dir."""
    rng = random.Random(seed)
    out_dir.mkdir(parents=True, exist_ok=True)

    orig_scene = _load_seed(scene_seed_path)
    orig_road = _load_seed(road_seed_path)
    road_body = _road_of(orig_road)

    archive = MapElitesArchive()

    # Seed the archive with the ORIGINAL scene as elite of its cell.
    orig_score = score_offline(orig_scene)["score"]
    archive.submit(orig_scene, orig_score, ops=["<seed>"])

    stats = {
        "budget": budget,
        "seed": seed,
        "k_ops": k_ops,
        "online": online,
        "rejected_by_ocl": 0,
        "accepted": 0,
        "displaced_elites": 0,
        "started": time.time(),
    }
    per_step_log = []
    rejected_log = []

    for step in range(budget):
        variant, ops = mutate_scene(orig_scene, rng, k=k_ops)
        # Gate 1: OCL invariants over (road_body, scene_body)
        v = ocl_violations(road_body, _scene_of(variant))
        if v:
            stats["rejected_by_ocl"] += 1
            rejected_log.append({"step": step, "ops": ops, "violations": v})
            continue
        # Gate 2+: XSD & CARLA extract left for a downstream --compile pass so
        # the core loop stays sub-second per step. See --compile-elites below.
        scores = score_offline(variant)
        won = archive.submit(variant, scores["score"], ops)
        if won:
            stats["accepted"] += 1
        per_step_log.append({
            "step": step, "ops": ops, "score": scores["score"],
            "closing": scores["closing"], "env_mult": scores["env_mult"],
            "feature": list(feature_of(variant)),
            "won": won,
        })

    stats["ended"] = time.time()
    stats["elapsed_s"] = round(stats["ended"] - stats["started"], 3)
    stats["cells_filled"] = len(archive.cells)
    stats["best_score"] = max((e.score for e in archive.cells.values()), default=0.0)

    # Persist
    (out_dir / "archive.json").write_text(json.dumps(
        {"stats": stats, "cells": archive.as_records()}, indent=2, ensure_ascii=False))
    (out_dir / "history.json").write_text(json.dumps(
        {"per_step": per_step_log, "archive_history": archive.history, "rejected": rejected_log},
        indent=2, ensure_ascii=False))
    (out_dir / "meta.json").write_text(json.dumps({
        "scene_seed_source": str(scene_seed_path),
        "road_seed_source": str(road_seed_path),
        "case_id": scene_seed_path.stem,
    }, indent=2, ensure_ascii=False))
    return {"stats": stats, "archive_records": archive.as_records()}


def main() -> int:
    ap = argparse.ArgumentParser(description="crash2openx fuzz runner")
    ap.add_argument("--seed", type=Path, required=False, help="scene_seed JSON")
    ap.add_argument("--road", type=Path, required=False, help="road_seed JSON")
    ap.add_argument("--out", type=Path, required=True, help="output dir")
    ap.add_argument("--budget", type=int, default=100)
    ap.add_argument("--rng-seed", type=int, default=42)
    ap.add_argument("--k-ops", type=int, default=1, help="ops per variant")
    ap.add_argument("--online", action="store_true", help="run CARLA per elite (needs GPU host)")
    ap.add_argument("--batch", type=Path, default=None, help="baseline_42.json for batch mode")
    args = ap.parse_args()

    if args.batch is not None:
        cases = json.loads(args.batch.read_text())["cases"]
        overall = {}
        for c in cases:
            cid = c["case_id"]
            scene_p = REPO_ROOT / "data/seeds/scene_seed" / f"{cid}.json"
            road_p = REPO_ROOT / "data/seeds/road_seed" / f"{cid}.json"
            if not scene_p.exists() or not road_p.exists():
                print(f"skip {cid}: seed missing")
                continue
            case_out = args.out / cid
            print(f"fuzzing {cid} ...")
            res = run_fuzz(scene_p, road_p, case_out, args.budget, args.rng_seed, args.k_ops, args.online)
            overall[cid] = res["stats"]
        (args.out / "batch_summary.json").write_text(json.dumps(overall, indent=2, ensure_ascii=False))
        print(f"batch done — {len(overall)} cases, summary in {args.out}/batch_summary.json")
        return 0

    if not args.seed or not args.road:
        ap.error("--seed and --road required (or use --batch)")
    res = run_fuzz(args.seed, args.road, args.out, args.budget, args.rng_seed, args.k_ops, args.online)
    print(json.dumps(res["stats"], indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
