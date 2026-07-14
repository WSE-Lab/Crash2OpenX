#!/usr/bin/env python3
"""Saturation-curve plotter for the Crash2OpenX vocabulary derivation.

Plots the cumulative number of distinct topologies observed as a function
of corpus position, on the split-A snapshot frozen in
schemas/road_topology_derivation.md §7.1. The first-occurrence indices
are the only data needed (we don't need the full 713-row corpus).

Output: paper/figures/saturation_curve.pdf (also .png for quick preview).
The figure goes into paper §3 to demonstrate "vocabulary saturation
without ad-hoc expansion": main topologies plateau at index ~155, with
fork as a long-tail member surfacing only at index 644.
"""
from __future__ import annotations

import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# Frozen split-A first-occurrence manifest (schemas/road_topology_derivation.md §7.1).
# Format: (corpus_index, topology_name).
FIRST_OCCURRENCES = [
    (1,   "cross_intersection"),
    (4,   "straight"),
    (18,  "curve"),
    (71,  "y_junction"),
    (76,  "t_junction"),
    (155, "merge"),
    (644, "fork"),
]
TOTAL_N = 649  # supported cases in split A (excluding 64 unsupported and 2 reclassified roundabout)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("paper/figures/saturation_curve.pdf"))
    args = ap.parse_args()

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[error] matplotlib not installed. Run: uv add matplotlib")
        return 1

    fig, ax = plt.subplots(figsize=(5.0, 2.6))

    # Build the step-function: at each index, cumulative distinct count goes up by 1.
    xs = [1]
    ys = [0]
    for n, _ in FIRST_OCCURRENCES:
        xs.extend([n, n])
        ys.extend([ys[-1], ys[-1] + 1])
    xs.append(TOTAL_N)
    ys.append(ys[-1])

    ax.step(xs, ys, where="post", color="#222", linewidth=1.6)

    # Annotate each first occurrence
    for n, name in FIRST_OCCURRENCES:
        idx = [i for i, (m, _) in enumerate(FIRST_OCCURRENCES) if m == n][0] + 1
        ax.plot(n, idx, "o", color="#222", markersize=4)
        # right-leaning label except for the leftmost ones
        ha = "left" if n < 0.7 * TOTAL_N else "right"
        ax.annotate(f"{name}@{n}", (n, idx),
                    xytext=(6 if ha == "left" else -6, -3),
                    textcoords="offset points",
                    fontsize=8, color="#444", ha=ha, va="top")

    ax.set_xlabel("corpus position (split A)", fontsize=9)
    ax.set_ylabel("distinct topologies seen", fontsize=9)
    ax.set_xlim(0, TOTAL_N + 10)
    ax.set_ylim(0, len(FIRST_OCCURRENCES) + 0.5)
    ax.set_yticks(range(0, len(FIRST_OCCURRENCES) + 1))
    ax.tick_params(labelsize=8)
    ax.grid(True, alpha=0.25, linewidth=0.5)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.tight_layout()
    out = args.out if args.out.is_absolute() else (ROOT / args.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200)
    # also drop a .png next to it for quick preview
    fig.savefig(out.with_suffix(".png"), dpi=200)
    print(f"=== wrote {out.relative_to(ROOT)} (and .png) ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
