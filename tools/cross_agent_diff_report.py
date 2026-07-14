#!/usr/bin/env python3
"""Compare two agents under the same post-D pipeline.

Reads:
  outputs/medoid_runs/<cid>/{summary,behavior_check}.json    (agent A, default tfv6_regnet @ D)
  outputs/medoid_runs_agent_if/<cid>/{summary,behavior_check}.json (agent B, default if_if @ D)

Writes:
  paper/medoid_cross_agent_report.md
  paper/figures/medoid_cross_agent_verdict.png
  paper/figures/medoid_cross_agent_driven.png

Usage:
    uv run --with matplotlib python tools/cross_agent_diff_report.py
    uv run --with matplotlib python tools/cross_agent_diff_report.py \
        --agent-a tfv6_regnet --dir-a outputs/medoid_runs \
        --agent-b if_if --dir-b outputs/medoid_runs_agent_if
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt  # noqa: E402

VERDICT_ORDER = ["ok", "drifted", "deadlock", "silent_spawn", "crash", "missing"]
VERDICT_COLORS = {
    "ok": "#2ca02c",
    "drifted": "#ff7f0e",
    "deadlock": "#d62728",
    "silent_spawn": "#7f7f7f",
    "crash": "#1f1f1f",
    "missing": "#cccccc",
}


def _load(path: Path) -> dict | None:
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        return None


def _verdict(case_dir: Path) -> tuple[str, dict]:
    s = _load(case_dir / "summary.json") or {}
    b = _load(case_dir / "behavior_check.json") or {}
    if not s:
        return "missing", {}
    ticks = s.get("total_ticks", 0)
    if ticks == 0:
        return "silent_spawn", s
    bv = b.get("verdict", "skipped")
    if bv == "aligned":
        return "ok", s
    if bv == "drifted":
        if any("expected collision but none occurred" in i for i in (b.get("issues") or [])):
            return "deadlock", s
        return "drifted", s
    return bv, s


def _metrics(case_dir: Path) -> dict:
    s = _load(case_dir / "summary.json") or {}
    b = _load(case_dir / "behavior_check.json") or {}
    wrong = next((c["actual_value"] for c in s.get("criteria", [])
                  if c["name"] == "WrongLaneTest"), 0)
    driven = next((c["actual_value"] for c in s.get("criteria", [])
                   if c["name"] == "CheckDrivenDistance"), 0.0)
    return {
        "ticks": s.get("total_ticks", 0),
        "duration_s": s.get("duration_seconds", 0.0),
        "driven_distance_m": float(driven),
        "min_distance": (s.get("min_distance") or {}).get("value", float("nan")),
        "lane_invasion": s.get("lane_invasion_count", 0),
        "wrong_lane": wrong,
    }


def _fmt(v, prec: int = 1) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        if v != v:
            return "—"
        return f"{v:.{prec}f}"
    return str(v)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--medoids", type=Path, default=ROOT / "paper/eval_medoids_n100.json")
    ap.add_argument("--agent-a", default="tfv6_regnet")
    ap.add_argument("--dir-a", type=Path, default=ROOT / "outputs/medoid_runs")
    ap.add_argument("--agent-b", default="if_if")
    ap.add_argument("--dir-b", type=Path, default=ROOT / "outputs/medoid_runs_agent_if")
    args = ap.parse_args()

    cases = [m["medoid"] for m in json.loads(args.medoids.read_text())["medoids"]]
    print(f"=== cross_agent_diff: {args.agent_a} vs {args.agent_b} ({len(cases)} cases) ===")

    rows = []
    for cid in cases:
        va, _ = _verdict(args.dir_a / cid)
        vb, _ = _verdict(args.dir_b / cid)
        ma = _metrics(args.dir_a / cid)
        mb = _metrics(args.dir_b / cid)
        rows.append({"cid": cid, "a": {"verdict": va, **ma}, "b": {"verdict": vb, **mb}})
        print(f"  {cid}: {va} → {vb}")

    # --- verdict bar chart ---
    fig_dir = ROOT / "paper/figures"; fig_dir.mkdir(parents=True, exist_ok=True)
    counts_a = {v: sum(1 for r in rows if r["a"]["verdict"] == v) for v in VERDICT_ORDER}
    counts_b = {v: sum(1 for r in rows if r["b"]["verdict"] == v) for v in VERDICT_ORDER}
    fig, ax = plt.subplots(figsize=(10, 4.5))
    x = list(range(len(VERDICT_ORDER)))
    w = 0.4
    ax.bar([i - w/2 for i in x], [counts_a[v] for v in VERDICT_ORDER], w,
           label=args.agent_a, color="#1f77b4", edgecolor="black")
    ax.bar([i + w/2 for i in x], [counts_b[v] for v in VERDICT_ORDER], w,
           label=args.agent_b, color="#d62728", edgecolor="black")
    ax.set_xticks(x); ax.set_xticklabels(VERDICT_ORDER)
    ax.set_ylabel(f"# medoids (n={len(rows)})")
    ax.set_title(f"Verdict: {args.agent_a} vs {args.agent_b} (post-D pipeline)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(fig_dir / "medoid_cross_agent_verdict.png", dpi=140)
    plt.close(fig)

    # --- driven_distance bar chart ---
    cids = [r["cid"][:14] for r in rows]
    fig, ax = plt.subplots(figsize=(11, max(4, len(cids) * 0.45)))
    y = list(range(len(cids)))
    h = 0.4
    ax.barh([i - h/2 for i in y], [r["a"]["driven_distance_m"] for r in rows],
            h, color="#1f77b4", edgecolor="black", label=args.agent_a)
    ax.barh([i + h/2 for i in y], [r["b"]["driven_distance_m"] for r in rows],
            h, color="#d62728", edgecolor="black", label=args.agent_b)
    ax.axvline(60, ls="--", color="orange", alpha=0.4, label="60 m (avg 1 m/s)")
    ax.axvline(360, ls="--", color="green", alpha=0.3, label="360 m (ideal)")
    ax.set_yticks(y); ax.set_yticklabels(cids, fontsize=8)
    ax.set_xlabel("driven_distance_m (CheckDrivenDistance)")
    ax.set_title(f"Ego driven distance in 60 s — {args.agent_a} vs {args.agent_b}")
    ax.legend(loc="lower right", fontsize=8)
    ax.invert_yaxis()
    fig.tight_layout()
    fig.savefig(fig_dir / "medoid_cross_agent_driven.png", dpi=140)
    plt.close(fig)

    # --- markdown report ---
    report = ROOT / "paper/medoid_cross_agent_report.md"
    lines = [
        f"# Cross-agent comparison ({args.agent_a} vs {args.agent_b}) under post-D pipeline",
        "",
        f"Same 18-medoid CARLA batch, same xosc/xodr (post-D pipeline), only the PCLA-loaded ADS differs.",
        "",
        "## Verdict distribution",
        "",
        f"| verdict | {args.agent_a} | {args.agent_b} | Δ |",
        "|---|---|---|---|",
    ]
    for v in VERDICT_ORDER:
        d_val = counts_b[v] - counts_a[v]
        sign = "+" if d_val > 0 else ""
        lines.append(f"| `{v}` | {counts_a[v]} | {counts_b[v]} | {sign}{d_val} |")
    lines.append("")
    lines.append("![verdict](figures/medoid_cross_agent_verdict.png)")
    lines.append("")
    lines.append("## Per-case metrics")
    lines.append("")
    lines.append(f"| case | verdict ({args.agent_a} → {args.agent_b}) | min_dist (m) | wrong_lane | lane_inv | driven (m) |")
    lines.append("|---|---|---|---|---|---|")
    for r in sorted(rows, key=lambda x: x["cid"]):
        ver = f"`{r['a']['verdict']}` → `{r['b']['verdict']}`"
        md = f"{_fmt(r['a']['min_distance'])} → {_fmt(r['b']['min_distance'])}"
        wl = f"{r['a']['wrong_lane']} → {r['b']['wrong_lane']}"
        li = f"{r['a']['lane_invasion']} → {r['b']['lane_invasion']}"
        dd = f"{_fmt(r['a']['driven_distance_m'])} → {_fmt(r['b']['driven_distance_m'])}"
        lines.append(f"| `{r['cid']}` | {ver} | {md} | {wl} | {li} | {dd} |")
    lines.append("")
    lines.append("![driven](figures/medoid_cross_agent_driven.png)")
    lines.append("")
    lines.append("## Notes")
    lines.append(f"- {args.agent_a} batch: `{args.dir_a}` (post-D pipeline)")
    lines.append(f"- {args.agent_b} batch: `{args.dir_b}` (post-D pipeline)")
    lines.append("- Same xosc inputs from `outputs/medoid_xosc/` for both agents.")
    report.write_text("\n".join(lines))
    print(f"  ✓ wrote {report.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
