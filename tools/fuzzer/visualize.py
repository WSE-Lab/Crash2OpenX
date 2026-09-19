#!/usr/bin/env python3
"""Visualize a fuzz run: coverage curve, danger curve, archive heatmap.

Usage::
    python -m tools.fuzzer.visualize --run outputs/fuzz_runs/case122

Reads ``archive.json`` + ``history.json`` and produces three PNGs alongside.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def _load(path: Path) -> dict:
    return json.loads(path.read_text())


def plot_all(run_dir: Path, out_dir: Path | None = None) -> list[Path]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir = out_dir or run_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    hist = _load(run_dir / "history.json")
    arch = _load(run_dir / "archive.json")

    per_step = hist["per_step"]
    archive_history = hist["archive_history"]

    written: list[Path] = []

    # 1. Coverage curve — archive cells filled over time
    if archive_history:
        xs = [h["step"] for h in archive_history]
        ys_cells = [h["cells_filled"] for h in archive_history]
        ys_best = [h["best_score_so_far"] for h in archive_history]

        fig, ax = plt.subplots(figsize=(6.5, 3.6))
        ax.plot(xs, ys_cells, color="#1e40af", linewidth=1.8, label="archive cells filled")
        ax.set_xlabel("fuzz step")
        ax.set_ylabel("distinct behavior cells", color="#1e40af")
        ax.tick_params(axis="y", labelcolor="#1e40af")
        ax2 = ax.twinx()
        ax2.plot(xs, ys_best, color="#dc2626", linewidth=1.8, linestyle="--", label="best score so far")
        ax2.set_ylabel("best danger score", color="#dc2626")
        ax2.tick_params(axis="y", labelcolor="#dc2626")
        fig.suptitle(f"Coverage & danger convergence — {run_dir.name}", fontsize=11)
        fig.tight_layout()
        p = out_dir / "coverage_curve.png"
        fig.savefig(p, dpi=180)
        plt.close(fig)
        written.append(p)

    # 2. Danger histogram over all accepted variants
    if per_step:
        scores = [s["score"] for s in per_step]
        fig, ax = plt.subplots(figsize=(6.5, 3.2))
        ax.hist(scores, bins=25, color="#7c3aed", edgecolor="white")
        ax.set_xlabel("danger score")
        ax.set_ylabel("variants")
        ax.set_title(f"Danger score distribution — {run_dir.name}")
        fig.tight_layout()
        p = out_dir / "danger_hist.png"
        fig.savefig(p, dpi=180)
        plt.close(fig)
        written.append(p)

    # 3. Archive heatmap — trig_dist_bin × speed_bin, averaged over other axes
    cells = arch["cells"]
    if cells:
        grid = [[[] for _ in range(4)] for _ in range(4)]  # 4×4 (trig, speed) bins
        for c in cells:
            feat = c["features"]  # [block, trig_bin, speed_bin, weather]
            t_bin, s_bin = int(feat[1]), int(feat[2])
            grid[t_bin][s_bin].append(c["score"])
        heat = [[(sum(cell) / len(cell) if cell else 0.0) for cell in row] for row in grid]

        fig, ax = plt.subplots(figsize=(5.5, 4.2))
        im = ax.imshow(heat, cmap="viridis", origin="lower", aspect="auto")
        ax.set_xticks(range(4))
        ax.set_yticks(range(4))
        ax.set_xticklabels(["<5", "5-15", "15-25", ">25"])
        ax.set_yticklabels(["<5", "5-15", "15-30", ">30"])
        ax.set_xlabel("NPC target speed bucket (m/s)")
        ax.set_ylabel("trig_dist bucket (m)")
        ax.set_title(f"MAP-Elites archive mean score — {run_dir.name}")
        for i in range(4):
            for j in range(4):
                v = heat[i][j]
                if v > 0:
                    ax.text(j, i, f"{v:.1f}", ha="center", va="center",
                            color="white" if v < max(max(r) for r in heat) * 0.6 else "black",
                            fontsize=9)
        fig.colorbar(im, ax=ax, fraction=0.045, pad=0.04)
        fig.tight_layout()
        p = out_dir / "archive_heatmap.png"
        fig.savefig(p, dpi=180)
        plt.close(fig)
        written.append(p)

    return written


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, required=True, help="fuzz run dir with archive.json/history.json")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    paths = plot_all(args.run, args.out)
    for p in paths:
        print(f"wrote {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
