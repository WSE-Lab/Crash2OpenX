#!/usr/bin/env python3
"""Cluster-level saturation curve for the eval set.

Question this answers:
  At n=k cases, how many DISTINCT cluster signatures (joint feature tuples)
  have we already observed? Plateau ⇒ adding more cases doesn't reveal new
  scenario classes; still climbing ⇒ the eval set undersamples the long tail.

Differs from saturation_plot.py:
  - That figure is TOPOLOGY-level over the 649-case corpus (paper §3).
  - This one is CLUSTER-level over the 35 clusterable eval cases (paper §6).

Bootstrap protocol:
  Sample 200 random permutations of the clusterable case list (seed=20260623)
  and at each prefix length count how many distinct feature signatures the
  prefix covers. Plot mean ± std envelope. The expected number of distinct
  classes at prefix-n is the unbiased "coupon-collector" estimator over the
  underlying distribution — a plateau there is the clean saturation signal.
"""
from __future__ import annotations

import argparse
import json
import random
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.cluster_eval_set import _feature_vector, _feature_vector_from_signature  # noqa: E402

RANDOM_SEED = 20260623
N_PERMUTATIONS = 200


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--eval-set", type=Path, default=ROOT / "paper/eval_set.json")
    ap.add_argument("--road-dir", type=Path, default=ROOT / "outputs/road_seed")
    ap.add_argument("--scene-dir", type=Path, default=ROOT / "outputs/scene_seed")
    ap.add_argument("--signature-dir", type=Path, default=None,
                    help="if set, read sig_v1 signatures from here (4 scene axes) instead of full scene_seed")
    ap.add_argument("--out", type=Path, default=ROOT / "paper/figures/cluster_saturation.pdf")
    args = ap.parse_args()

    es = json.loads(args.eval_set.read_text())
    use_sig = args.signature_dir is not None
    feats: list[tuple] = []
    for c in es["cases"]:
        cid = c["case_id"]
        rs_path = args.road_dir / f"{cid}.json"
        if not rs_path.is_file():
            continue
        road = json.loads(rs_path.read_text())
        if road.get("status") != "supported":
            continue
        if use_sig:
            sig_path = args.signature_dir / f"{cid}.json"
            if not sig_path.is_file():
                continue
            sig_doc = json.loads(sig_path.read_text())
            if sig_doc.get("status") != "supported":
                continue
            feats.append(_feature_vector_from_signature(road, sig_doc))
        else:
            sc_path = args.scene_dir / f"{cid}.json"
            if not sc_path.is_file():
                continue
            scene = json.loads(sc_path.read_text())
            if scene.get("status") != "supported":
                continue
            feats.append(_feature_vector(road, scene))

    n = len(feats)
    distinct_total = len(set(feats))
    print(f"clusterable cases: {n}")
    print(f"distinct cluster signatures: {distinct_total}")

    # Curve: mean ± std distinct-count at each prefix length
    rng = random.Random(RANDOM_SEED)
    curves = np.zeros((N_PERMUTATIONS, n), dtype=int)
    for p in range(N_PERMUTATIONS):
        order = list(range(n))
        rng.shuffle(order)
        seen: set = set()
        for i, idx in enumerate(order):
            seen.add(feats[idx])
            curves[p, i] = len(seen)
    xs = np.arange(1, n + 1)
    mean = curves.mean(axis=0)
    std = curves.std(axis=0)

    # Plateau detection: did the curve stop growing for the last 5 cases?
    last5_growth = mean[-1] - mean[-6] if n >= 6 else float("nan")
    print(f"mean distinct@n={n}: {mean[-1]:.2f}")
    print(f"growth over last 5 cases: +{last5_growth:.2f}")
    print(f"asymptote estimate (max distinct in any permutation): {int(curves[:, -1].max())}")

    # Annotated breakdown of signatures (which feature axis is most novel?)
    print("\n--- top-10 most common signatures ---")
    cnt = Counter(feats)
    for sig, c in cnt.most_common(10):
        print(f"  {c:>2}x  topology={sig[0]}  lanes={sig[1]}  maneuver={sig[2]}  npc={sig[4]}  coll={sig[5]}")
    singles = [s for s, c in cnt.items() if c == 1]
    print(f"\n{len(singles)} of {len(cnt)} signatures are singletons "
          f"({100 * len(singles) / len(cnt):.0f}% of distinct types).")

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[warn] matplotlib not installed; skipping plot")
        return 0

    fig, ax = plt.subplots(figsize=(5.0, 3.0))
    ax.fill_between(xs, mean - std, mean + std, alpha=0.2, color="#2563eb",
                    label="±1σ over 200 permutations")
    ax.plot(xs, mean, color="#1e3a8a", linewidth=1.6,
            label=f"mean distinct signatures (asymptote ≈ {distinct_total})")
    ax.axhline(distinct_total, color="#9ca3af", linestyle="--", linewidth=0.8,
               label=f"observed total = {distinct_total}")
    ax.set_xlabel("cases sampled (random order)", fontsize=9)
    ax.set_ylabel("distinct cluster signatures", fontsize=9)
    ax.set_xlim(0, n + 1)
    ax.set_ylim(0, distinct_total + 1)
    ax.legend(loc="lower right", fontsize=8, frameon=False)
    ax.grid(True, alpha=0.25, linewidth=0.5)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.tight_layout()
    out = args.out if args.out.is_absolute() else (ROOT / args.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200)
    fig.savefig(out.with_suffix(".png"), dpi=200)
    print(f"\nwrote {out.relative_to(ROOT)} (+ .png)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
