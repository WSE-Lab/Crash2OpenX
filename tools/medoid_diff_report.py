#!/usr/bin/env python3
"""Compare pre-fix vs post-fix 18-medoid CARLA runs.

Reads:
  outputs/medoid_runs_pre_fix_snapshot/<cid>/{summary,behavior_check}.json
  outputs/medoid_runs/<cid>/{summary,behavior_check}.json
  outputs/medoid_runs/_summary_pre_fix.tsv
  outputs/medoid_runs/_summary.tsv     (latest 18 rows = post-fix)

Writes:
  paper/medoid_runtime_fix_report.md
  paper/figures/medoid_fix_verdict.png
  paper/figures/medoid_fix_min_distance.png
  paper/figures/medoid_fix_route_completion.png

The two fixes under test:
  A) DEFAULT_GAP_M 15 → 25  (resolve_position / resolve_ego_placement)
  B) ego AcquirePosition goal retargeted to RoadGraph exit lane for left/right
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

PRE_DIR = ROOT / "outputs/medoid_runs_pre_fix_snapshot"
MID_DIR = ROOT / "outputs/medoid_runs_post_AC_snapshot"
B1_DIR = ROOT / "outputs/medoid_runs_post_B1_snapshot"
POST_DIR = ROOT / "outputs/medoid_runs"
FIG_DIR = ROOT / "paper/figures"
REPORT = ROOT / "paper/medoid_runtime_fix_report.md"

VERDICT_ORDER = ["ok", "drifted", "deadlock", "silent_spawn", "crash"]
VERDICT_COLORS = {
    "ok": "#2ca02c",
    "drifted": "#ff7f0e",
    "deadlock": "#d62728",
    "silent_spawn": "#7f7f7f",
    "crash": "#1f1f1f",
}


def _load(path: Path) -> dict | None:
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        return None


def _verdict_for_run(case_dir: Path) -> tuple[str, dict]:
    """Replicate batch_run_medoids.classify on disk snapshots."""
    s = _load(case_dir / "summary.json") or {}
    b = _load(case_dir / "behavior_check.json") or {}
    ticks = s.get("total_ticks", 0)
    if ticks == 0 and not s:
        return "missing", {}
    if ticks == 0:
        return "silent_spawn", s
    bv = b.get("verdict", "skipped")
    if bv == "aligned":
        return "ok", s
    if bv == "drifted":
        issues = b.get("issues") or []
        if any("expected collision but none occurred" in i for i in issues):
            return "deadlock", s
        return "drifted", s
    return bv, s


def _extract_metrics(case_dir: Path) -> dict:
    s = _load(case_dir / "summary.json") or {}
    b = _load(case_dir / "behavior_check.json") or {}
    wrong = next((c["actual_value"] for c in s.get("criteria", [])
                  if c["name"] == "WrongLaneTest"), 0)
    driven = next((c["actual_value"] for c in s.get("criteria", [])
                   if c["name"] == "CheckDrivenDistance"), 0.0)
    return {
        "ticks": s.get("total_ticks", 0),
        "duration_s": s.get("duration_seconds", 0.0),
        "route_completion": s.get("final_route_completion", 0.0),
        "driven_distance_m": float(driven),
        "min_distance": (s.get("min_distance") or {}).get("value", float("nan")),
        "lane_invasion": s.get("lane_invasion_count", 0),
        "wrong_lane": wrong,
        "off_road_s": s.get("off_road_time", 0.0),
        "issues": b.get("issues", []),
    }


def _diff_case(cid: str) -> dict | None:
    pre_dir = PRE_DIR / cid
    mid_dir = MID_DIR / cid
    b1_dir = B1_DIR / cid
    post_dir = POST_DIR / cid
    if not (pre_dir.is_dir() or mid_dir.is_dir() or b1_dir.is_dir() or post_dir.is_dir()):
        return None
    out = {"cid": cid}
    for stage, d in (("pre", pre_dir), ("mid", mid_dir), ("b1", b1_dir), ("post", post_dir)):
        v, _ = _verdict_for_run(d)
        out[stage] = {"verdict": v, **_extract_metrics(d)}
    return out


STAGES = ("pre", "mid", "b1", "post")
STAGE_LABELS = ("pre-fix", "post A+C", "post B1", "post D")
STAGE_COLORS = ("#bbbbbb", "#1f77b4", "#2ca02c", "#d62728")


def _emit_verdict_chart(diffs: list[dict], out_path: Path) -> None:
    counts = {stage: {v: 0 for v in VERDICT_ORDER} for stage in STAGES}
    for d in diffs:
        for stage in STAGES:
            v = d[stage]["verdict"]
            if v in counts[stage]:
                counts[stage][v] += 1
    fig, ax = plt.subplots(figsize=(11, 4.5))
    x = list(range(len(VERDICT_ORDER)))
    w = 0.2
    offsets = (-1.5 * w, -0.5 * w, 0.5 * w, 1.5 * w)
    for stage, label, off, col in zip(STAGES, STAGE_LABELS, offsets, STAGE_COLORS):
        ax.bar([i + off for i in x], [counts[stage][v] for v in VERDICT_ORDER],
               w, label=label, color=col, edgecolor="black")
    ax.set_xticks(x); ax.set_xticklabels(VERDICT_ORDER)
    ax.set_ylabel(f"# medoids (n={len(diffs)})")
    ax.set_title("Verdict distribution: pre-fix → post A+C → post B1 → post D")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def _emit_min_distance_chart(diffs: list[dict], out_path: Path) -> None:
    cids = [d["cid"][:14] for d in diffs
            if all(d[s]["min_distance"] == d[s]["min_distance"] for s in STAGES)]
    if not cids:
        return
    valid = [d for d in diffs if d["cid"][:14] in cids]
    fig, ax = plt.subplots(figsize=(11, max(4, len(cids) * 0.5)))
    y = list(range(len(cids)))
    h = 0.2
    offsets = (-1.5 * h, -0.5 * h, 0.5 * h, 1.5 * h)
    for stage, label, off, col in zip(STAGES, STAGE_LABELS, offsets, STAGE_COLORS):
        ax.barh([i + off for i in y], [d[stage]["min_distance"] for d in valid],
                h, label=label, color=col, edgecolor="black")
    ax.axvline(15.4, ls="--", color="red", alpha=0.4,
               label="pre-fix anchor (15.4 m, kinematic deadlock)")
    ax.set_yticks(y); ax.set_yticklabels(cids, fontsize=8)
    ax.set_xlabel("min_distance to expected NPC (m)")
    ax.set_title("Min ego↔NPC distance (lower = ego actually approached the NPC)")
    ax.legend(loc="lower right", fontsize=7)
    ax.invert_yaxis()
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def _emit_route_completion_chart(diffs: list[dict], out_path: Path) -> None:
    """Driven distance (absolute meters) — NOT route_completion %, which is
    misleading after road_length doubled (denominator changed)."""
    cids = [d["cid"][:14] for d in diffs]
    fig, ax = plt.subplots(figsize=(11, max(4, len(cids) * 0.5)))
    y = list(range(len(cids)))
    h = 0.2
    offsets = (-1.5 * h, -0.5 * h, 0.5 * h, 1.5 * h)
    for stage, label, off, col in zip(STAGES, STAGE_LABELS, offsets, STAGE_COLORS):
        ax.barh([i + off for i in y], [d[stage]["driven_distance_m"] for d in diffs],
                h, label=label, color=col, edgecolor="black")
    ax.axvline(60, ls="--", color="orange", alpha=0.4, label="60 m (avg speed = 1 m/s)")
    ax.axvline(360, ls="--", color="green", alpha=0.3, label="360 m (ideal 60s×6 m/s)")
    ax.set_yticks(y); ax.set_yticklabels(cids, fontsize=8)
    ax.set_xlabel("driven_distance_m (CheckDrivenDistance, absolute)")
    ax.set_title("Ego actually-driven distance in 60 s — measures PCLA throughput")
    ax.legend(loc="lower right", fontsize=7)
    ax.invert_yaxis()
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def _fmt(v, w: int = 8, prec: int = 1) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        if v != v:  # NaN
            return "—"
        return f"{v:.{prec}f}"
    return str(v)


def _delta_arrow(pre, post, lower_is_better: bool = True) -> str:
    """ASCII arrow indicating direction of change (vs pre)."""
    if pre is None or post is None:
        return ""
    if isinstance(pre, float) and pre != pre:
        return ""
    if isinstance(post, float) and post != post:
        return ""
    if abs(pre - post) < 1e-6:
        return "="
    better = (post < pre) if lower_is_better else (post > pre)
    return "↓✓" if (post < pre and better) else ("↑✓" if (post > pre and better) else
           "↓" if post < pre else "↑")


def emit_report(diffs: list[dict]) -> None:
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    lines.append("# Sprint-4 runtime-fix CARLA verification")
    lines.append("")
    lines.append("Four-way comparison of the 18-medoid CARLA batch:")
    lines.append("")
    lines.append("- **pre-fix** — baseline (sprint-3 defaults: gap=15, road_length 70-250 m, NPC speed 8 m/s)")
    lines.append("- **post A+C** — A: `DEFAULT_GAP_M` 15→25, C: `left/right` ego goal retargeted to RoadGraph exit")
    lines.append("- **post B1** — A+C plus `ROAD_TYPE_DEFAULTS.road_length_m` ≈ doubled (lowSpeed 70→140, town 100→200, motorway 250→500); fixed two bugs in `replay_scene_tools` that ignored road_length on straight/curve corridors")
    lines.append("- **post D** — B1 plus three pipeline tightenings: WF10 rejects `turn + oncoming` on junction topology (665 dropped), rear_hit closing speed 8→14 m/s, oncoming 8→12 m/s, gap split (behind_same_lane 15 m, oncoming 20 m, others 25 m)")
    lines.append("")
    counts = {s: {v: sum(1 for d in diffs if d[s]["verdict"] == v) for v in VERDICT_ORDER}
              for s in STAGES}
    lines.append("## Verdict distribution")
    lines.append("")
    lines.append("| verdict | pre-fix | post A+C | post B1 | post D | Δ (D-pre) |")
    lines.append("|---|---|---|---|---|---|")
    for v in VERDICT_ORDER:
        d_val = counts["post"][v] - counts["pre"][v]
        sign = "+" if d_val > 0 else ""
        lines.append(f"| `{v}` | {counts['pre'][v]} | {counts['mid'][v]} | {counts['b1'][v]} | {counts['post'][v]} | {sign}{d_val} |")
    lines.append("")
    lines.append("![verdict](figures/medoid_fix_verdict.png)")
    lines.append("")
    lines.append("## Per-case metrics (pre → A+C → B1 → D)")
    lines.append("")
    lines.append("`driven_dist` = absolute meters ego actually covered "
                 "(`CheckDrivenDistance`); 60 s × cruise 6 m/s = 360 m ideal.")
    lines.append("")
    lines.append("| case | verdict | min_distance (m) | wrong_lane | lane_invasion | driven_dist (m) |")
    lines.append("|---|---|---|---|---|---|")
    for d in sorted(diffs, key=lambda r: r["cid"]):
        ver = " → ".join(f"`{d[s]['verdict']}`" for s in STAGES)
        md = " → ".join(_fmt(d[s]["min_distance"]) for s in STAGES)
        wl = " → ".join(str(d[s]["wrong_lane"]) for s in STAGES)
        li = " → ".join(str(d[s]["lane_invasion"]) for s in STAGES)
        dd = " → ".join(_fmt(d[s]["driven_distance_m"]) for s in STAGES)
        lines.append(f"| `{d['cid']}` | {ver} | {md} | {wl} | {li} | {dd} |")
    lines.append("")
    lines.append("## Figures")
    lines.append("")
    lines.append("![min_distance](figures/medoid_fix_min_distance.png)")
    lines.append("")
    lines.append("![driven_distance](figures/medoid_fix_route_completion.png)")
    lines.append("")
    lines.append("## Notes")
    lines.append("")
    lines.append(f"- pre-fix snapshot: `outputs/medoid_runs_pre_fix_snapshot/`")
    lines.append(f"- post A+C snapshot: `outputs/medoid_runs_post_AC_snapshot/`")
    lines.append(f"- post B1 snapshot: `outputs/medoid_runs_post_B1_snapshot/`")
    lines.append(f"- post D (current): `outputs/medoid_runs/`")
    lines.append(f"- 665 was rejected by WF10 at xosc compile; its `post` row is missing because no xosc was produced for the D batch.")
    REPORT.write_text("\n".join(lines))
    print(f"  ✓ wrote {REPORT.relative_to(ROOT)}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--medoids", type=Path,
                    default=ROOT / "paper/eval_medoids_n100.json")
    args = ap.parse_args()
    meta = json.loads(args.medoids.read_text())
    cases = [m["medoid"] for m in meta["medoids"]]

    print(f"=== medoid_diff_report ({len(cases)} cases) ===")
    diffs = []
    for cid in cases:
        d = _diff_case(cid)
        if d is None:
            print(f"  · {cid}: missing all snapshots — skip")
            continue
        diffs.append(d)
        print(f"  · {cid}: {d['pre']['verdict']} → {d['mid']['verdict']} → {d['b1']['verdict']} → {d['post']['verdict']}")

    FIG_DIR.mkdir(parents=True, exist_ok=True)
    _emit_verdict_chart(diffs, FIG_DIR / "medoid_fix_verdict.png")
    _emit_min_distance_chart(diffs, FIG_DIR / "medoid_fix_min_distance.png")
    _emit_route_completion_chart(diffs, FIG_DIR / "medoid_fix_route_completion.png")
    print(f"  ✓ wrote 3 figures to {FIG_DIR.relative_to(ROOT)}/")

    emit_report(diffs)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
