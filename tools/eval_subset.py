#!/usr/bin/env python3
"""Stratified eval-set sampler for the Crash2OpenX corpus.

Implements the "rare-stratum boost" protocol agreed during Sprint 1:
  - minority topologies (fork / y_junction / t_junction / curve / merge)
    are taken in full or near-full to guarantee >= 3 cases per topology;
  - majority topologies (cross_intersection / straight) are randomly
    subsampled to fill the remaining quota so the eval set hits ~50 cases;
  - unsupported reports are sampled separately as a §6-limitations pool.

Outputs `paper/eval_set.json` with the deterministic case-id list (frozen
random seed = 20260623). The list is the single source of truth for
Sprint-2 measurements (split B coverage, executability ratio, baseline).

Usage (when the road_seed corpus is available locally):
    uv run python tools/eval_subset.py \
        --road-seed-dir outputs/road_seed \
        --out paper/eval_set.json

The script tolerates an empty / missing dir: it then prints the QUOTA
TABLE only, so the protocol is documented even before split-B data lands.
"""
from __future__ import annotations

import argparse
import json
import random
from collections import defaultdict
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]

# Per-topology eval quota. Designed so every supported topology has >=3
# observable cases and every NHTSA ✓ class is hit at least once. Total = 50.
QUOTA = {
    "cross_intersection": 15,
    "straight": 12,
    "merge": 3,
    "curve": 5,
    "t_junction": 5,
    "y_junction": 2,   # all 2 in split A
    "fork": 1,         # all 1 in split A
    "unsupported": 7,  # boundary cases for §6 limitations
}
RANDOM_SEED = 20260623

# Extension quota (split B': another 50 disjoint from the original 50, used to
# probe cluster-level saturation at n=100). y_junction (2) and fork (1) are
# already exhausted in split A, so the slack goes to majority strata. Seed is
# bumped so no re-sampling collisions when the same script is re-run.
EXTEND_QUOTA = {
    "cross_intersection": 17,
    "straight": 14,
    "merge": 3,
    "curve": 5,
    "t_junction": 5,
    "y_junction": 0,
    "fork": 0,
    "unsupported": 6,
}
EXTEND_SEED = 20260624


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--road-seed-dir", type=Path,
                    help="directory of <case>.json road_seed outputs (e.g. outputs/road_seed/)")
    ap.add_argument("--out", type=Path, default=Path("paper/eval_set.json"))
    ap.add_argument("--extend-from", type=Path,
                    help="existing eval_set.json — sample 50 MORE cases excluding its case_ids")
    args = ap.parse_args()

    extend = args.extend_from is not None
    quota = EXTEND_QUOTA if extend else QUOTA
    seed = EXTEND_SEED if extend else RANDOM_SEED
    label = "Eval set extension (split B', +50)" if extend else "Eval set quota (rare-stratum boost, n=50)"
    print(f"=== {label} ===")
    for k, v in quota.items():
        print(f"  {k:<20s}  {v:>3d}")
    print(f"  {'TOTAL':<20s}  {sum(quota.values()):>3d}")
    print()

    if not args.road_seed_dir or not args.road_seed_dir.is_dir():
        print(f"[note] --road-seed-dir not provided / missing; only the quota table was printed.")
        print("       Re-run with the directory once the road_seed corpus is regenerated.")
        return 0

    excluded: set[str] = set()
    if extend:
        es = json.loads(args.extend_from.read_text())
        excluded = {c["case_id"] for c in es["cases"]}
        print(f"[extend] excluding {len(excluded)} case_ids already in {args.extend_from}")

    rng = random.Random(seed)
    by_topo: dict[str, list[str]] = defaultdict(list)

    for fp in sorted(args.road_seed_dir.glob("*.json")):
        if fp.stem in excluded:
            continue
        try:
            d = json.loads(fp.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        road = d.get("road") or {}
        if d.get("status") == "unsupported":
            by_topo["unsupported"].append(fp.stem)
            continue
        topo = road.get("topology")
        if topo:
            by_topo[topo].append(fp.stem)

    chosen: list[dict[str, str]] = []
    for topo, q in quota.items():
        if q == 0:
            continue
        pool = by_topo.get(topo, [])
        take = pool if len(pool) <= q else rng.sample(pool, q)
        for case_id in sorted(take):
            chosen.append({"case_id": case_id, "topology": topo})

    out = {
        "version": "split_eval_v1_extension" if extend else "split_eval_v1",
        "random_seed": seed,
        "quota": quota,
        "total_cases": len(chosen),
        "cases": chosen,
    }
    if extend:
        out["extends"] = str(args.extend_from)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    try:
        rel = args.out.resolve().relative_to(ROOT)
    except ValueError:
        rel = args.out
    print(f"=== wrote {len(chosen)} cases to {rel} ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
