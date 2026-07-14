#!/usr/bin/env python3
"""Validation gate before opening the floodgates on the full 713 corpus.

Compares feature vectors from the heavyweight scene_seed_v2 path
(_feature_vector) against the lightweight sig_v1 path
(_feature_vector_from_signature) on cases where BOTH artifacts exist.

Pass criterion: >= 95% of cases produce identical 6-tuples. Mismatches are
listed by axis so we can decide whether they are LLM noise (re-run with a
different seed → expected) or genuine schema misalignment (signature prompt
needs fixing).

Run AFTER:
    uv run ... python tools/batch_infer_signatures.py \\
        --only $(jq -r '.cases[].case_id' paper/eval_set_n100.json | tr '\\n' ' ')

Then:
    uv run ... python tools/verify_signature_parity.py
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.cluster_eval_set import _feature_vector, _feature_vector_from_signature  # noqa: E402

DEFAULT_EVAL_SET = REPO_ROOT / "paper/working/eval_set_n100.json"
DEFAULT_ROAD = REPO_ROOT / "outputs/road_seed"
DEFAULT_SCENE = REPO_ROOT / "outputs/scene_seed"
DEFAULT_SIG = REPO_ROOT / "outputs/signature"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--eval-set", type=Path, default=DEFAULT_EVAL_SET)
    ap.add_argument("--road-dir", type=Path, default=DEFAULT_ROAD)
    ap.add_argument("--scene-dir", type=Path, default=DEFAULT_SCENE)
    ap.add_argument("--signature-dir", type=Path, default=DEFAULT_SIG)
    ap.add_argument("--threshold", type=float, default=0.95,
                    help="minimum fraction of identical 6-tuples to pass the gate")
    args = ap.parse_args()

    es = json.loads(args.eval_set.read_text(encoding="utf-8"))
    case_ids = [c["case_id"] for c in es["cases"]]

    rows: list[dict] = []
    skipped: list[tuple[str, str]] = []
    for cid in case_ids:
        rs = args.road_dir / f"{cid}.json"
        sc = args.scene_dir / f"{cid}.json"
        sg = args.signature_dir / f"{cid}.json"
        if not rs.is_file():
            skipped.append((cid, "no road_seed"))
            continue
        if not sc.is_file():
            skipped.append((cid, "no scene_seed"))
            continue
        if not sg.is_file():
            skipped.append((cid, "no signature (run batch_infer_signatures first)"))
            continue
        road = json.loads(rs.read_text(encoding="utf-8"))
        scene = json.loads(sc.read_text(encoding="utf-8"))
        sig = json.loads(sg.read_text(encoding="utf-8"))
        scene_status = scene.get("status")
        sig_status = sig.get("status")
        if scene_status != "supported" or sig_status != "supported":
            skipped.append((cid, f"status mismatch: scene={scene_status} sig={sig_status}"))
            continue
        fv_legacy = _feature_vector(road, scene)
        fv_sig = _feature_vector_from_signature(road, sig)
        rows.append({"case_id": cid, "legacy": fv_legacy, "sig": fv_sig,
                     "match": fv_legacy == fv_sig})

    n = len(rows)
    if n == 0:
        print("FATAL: no cases with all three artifacts present.")
        for cid, reason in skipped[:10]:
            print(f"  - {cid}: {reason}")
        if len(skipped) > 10:
            print(f"  ... +{len(skipped)-10} more")
        return 2

    matches = sum(1 for r in rows if r["match"])
    rate = matches / n
    print(f"=== signature parity gate ===")
    print(f"  eval set       : {args.eval_set.name}  (n={len(case_ids)})")
    print(f"  comparable     : {n}")
    print(f"  skipped        : {len(skipped)}")
    print(f"  identical 6-tuples : {matches}/{n}  ({rate:.1%})")
    print(f"  threshold      : {args.threshold:.0%}")

    # Per-axis disagreement breakdown — which axis is the LLM most often disagreeing on?
    AXES = ["topology", "lanes", "sut_maneuver", "npc_count_bin", "npc_sig", "collision_pair"]
    axis_diffs = Counter()
    for r in rows:
        if r["match"]:
            continue
        for i, name in enumerate(AXES):
            if r["legacy"][i] != r["sig"][i]:
                axis_diffs[name] += 1
    if axis_diffs:
        print("\n  axis disagreement counts:")
        for name in AXES:
            if name in axis_diffs:
                print(f"    {name:<18s}  {axis_diffs[name]}")

    # Mismatch table
    if matches < n:
        print("\n  mismatching cases:")
        for r in rows:
            if r["match"]:
                continue
            for i, name in enumerate(AXES):
                if r["legacy"][i] != r["sig"][i]:
                    lv, sv = r["legacy"][i], r["sig"][i]
                    print(f"    {r['case_id']:<40s} {name:<18s} legacy={lv!r}  sig={sv!r}")

    if skipped:
        print(f"\n  {len(skipped)} skipped:")
        for cid, reason in skipped[:20]:
            print(f"    - {cid} :: {reason}")
        if len(skipped) > 20:
            print(f"    ... +{len(skipped)-20} more")

    if rate < args.threshold:
        print(f"\nGATE FAIL — {rate:.1%} < {args.threshold:.0%}. Inspect axis disagreement before opening full corpus.")
        return 1
    print(f"\nGATE PASS — signature path within tolerance, OK to run full corpus.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
