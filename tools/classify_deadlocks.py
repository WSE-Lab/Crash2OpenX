#!/usr/bin/env python3
"""Classify 'deadlock' verdict cases into B/C/D/E classes for paper §6.

Taxonomy:
  A — pipeline runtime failure (silent_spawn / crash). Mostly eliminated by Sprint-3 F1-F4.
  B — pipeline scenario-design failure (needs manual inspection — "everything else" bucket).
  C — ADS lane-keep failure (drift / wrong lane). Testing-tool success: exposes ADS deficiency.
  D — ADS stalled (planner does not engage throttle/brake). Testing-tool success.
  E — ADS routing miss (ego drove but never closed on NPC; never entered trigger range).

Reads:
  outputs/medoid_runs/<case>/summary.json + behavior_check.json + run_scene_wrapper.log
  outputs/medoid_runs_agent_if/<case>/ (same)

Writes:
  paper/deadlock_classification.json (machine-readable)
  paper/deadlock_classification.md   (human-readable)

Run from repo root:
  python3 tools/classify_deadlocks.py
"""
from __future__ import annotations

import csv
import json
from collections import Counter
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
BATCHES = [
    ("tfv6_regnet", REPO_ROOT / "outputs" / "medoid_runs"),
    ("if_if", REPO_ROOT / "outputs" / "medoid_runs_agent_if"),
]
OUT_JSON = REPO_ROOT / "paper" / "deadlock_classification.json"
OUT_MD = REPO_ROOT / "paper" / "deadlock_classification.md"


def load_runner_verdicts(batch_dir: Path) -> dict[str, tuple[str, str]]:
    """Return {cid: (verdict, detail)} from the LAST row per cid in `_summary.tsv`.

    The runner-level verdict (ok / drifted / deadlock / silent_spawn / crash) is
    a stronger classification than behavior_check.json's L3 alignment, because
    it folds in kinematics-deadlock detection. We need this verdict, not the
    behavior_check one, to match `medoid_cross_agent_report.md`.
    """
    tsv = batch_dir / "_summary.tsv"
    if not tsv.exists():
        return {}
    latest: dict[str, tuple[str, str]] = {}
    with tsv.open() as f:
        reader = csv.DictReader(f, delimiter="\t")
        for row in reader:
            cid = row.get("cid")
            if not cid:
                continue
            latest[cid] = (row.get("verdict", "unknown"), row.get("detail", ""))
    return latest

# Pipeline runtime error signatures left over from before Sprint-3 fixes.
PIPELINE_ERR_PATTERNS = (
    "RuntimeError: lane_width_info",   # F1: silent spawn on innermost lane
    "IndexError: list index out of range",  # F2: oncoming ego_s clearance
    "ZeroDivisionError",               # F4: cut_in zero-velocity
    "global_route_planner._path_search",  # F3: TimeToCollision route planner
    "time-out of 30000ms",             # CARLA infra timeout (not pipeline strictly)
)


def get_criterion_value(criteria, name):
    for c in criteria or []:
        if c.get("name") == name:
            return c.get("actual_value")
    return None


def classify(case_dir: Path, runner_verdict: str = "unknown") -> dict:
    summary_path = case_dir / "summary.json"
    behavior_path = case_dir / "behavior_check.json"
    log_path = case_dir / "run_scene_wrapper.log"

    if not summary_path.exists():
        return {"class": "MISSING", "reason": "summary.json absent", "metrics": {}}

    summary = json.loads(summary_path.read_text())
    behavior = json.loads(behavior_path.read_text()) if behavior_path.exists() else {}

    l3_verdict = behavior.get("verdict", "unknown")
    driven = get_criterion_value(summary.get("criteria", []), "CheckDrivenDistance") or 0.0
    min_dist = (summary.get("min_distance") or {}).get("value", float("inf"))
    max_long_jerk = abs((summary.get("max_long_jerk") or {}).get("value", 0.0))
    collision_count = summary.get("collision_count", 0)
    lane_inv = summary.get("lane_invasion_count", 0)
    wrong_lane = behavior.get("wrong_lane_count", 0)

    metrics = {
        "driven_m": round(driven, 3),
        "min_dist_m": round(min_dist, 2) if min_dist != float("inf") else None,
        "max_long_jerk_abs": round(max_long_jerk, 4),
        "collision_count": collision_count,
        "lane_invasion_count": lane_inv,
        "wrong_lane_count": wrong_lane,
        "l3_verdict": l3_verdict,
        "runner_verdict": runner_verdict,
    }

    # Only classify cases the BATCH RUNNER labelled "deadlock".
    # ok / drifted (light) / crash / silent_spawn / missing are out of §6 scope.
    if runner_verdict != "deadlock":
        return {"class": "SKIP", "reason": f"runner_verdict={runner_verdict}", "metrics": metrics}

    # A — pipeline runtime error visible in log
    if log_path.exists():
        log_tail = log_path.read_text(errors="ignore")[-50000:]
        for pat in PIPELINE_ERR_PATTERNS:
            if pat in log_tail:
                return {"class": "A", "reason": f"pipeline log shows: {pat!r}", "metrics": metrics}

    # D — ADS stalled (didn't engage powertrain)
    if driven < 5.0 and max_long_jerk < 1.0:
        return {
            "class": "D",
            "reason": f"ADS stalled (driven={driven:.2f} m, |jerk|={max_long_jerk:.3f} m/s³)",
            "metrics": metrics,
        }

    # C — ADS lane-keep failure
    if lane_inv > 5 or wrong_lane > 0:
        return {
            "class": "C",
            "reason": f"ADS lane-keep fail (lane_inv={lane_inv}, wrong_lane={wrong_lane})",
            "metrics": metrics,
        }

    # E — ADS routing miss
    if collision_count == 0 and driven >= 5.0 and (min_dist is None or min_dist > 10.0):
        return {
            "class": "E",
            "reason": f"ADS routing miss (driven={driven:.2f} m, min_dist={min_dist:.1f} m > 10)",
            "metrics": metrics,
        }

    # B — residual scenario-design suspect
    return {"class": "B", "reason": "scenario-design suspect (needs manual inspection)", "metrics": metrics}


def main() -> int:
    results: dict[str, dict[str, dict]] = {}
    for ads, base in BATCHES:
        if not base.exists():
            print(f"[warn] {base} not found, skipping")
            continue
        runner_verdicts = load_runner_verdicts(base)
        results[ads] = {}
        for case_dir in sorted(base.iterdir()):
            if not case_dir.is_dir() or case_dir.name.startswith("_"):
                continue
            rv = runner_verdicts.get(case_dir.name, ("unknown", ""))[0]
            results[ads][case_dir.name] = classify(case_dir, runner_verdict=rv)

    OUT_JSON.write_text(json.dumps(results, indent=2, ensure_ascii=False))

    md = ["# Deadlock B/C/D/E classification (auto-generated)",
          "",
          "Source: `tools/classify_deadlocks.py` over `outputs/medoid_runs*/`.",
          "Underlies paper §6 limitations narrative.",
          "",
          "## Taxonomy",
          "",
          "| Class | Meaning | Attribution |",
          "|---|---|---|",
          "| **A** | pipeline runtime failure (silent_spawn/crash, or CARLA-infra timeout) | pipeline / infra |",
          "| **B** | scenario-design suspect (deadlock residual after A/C/D/E filters) | pipeline (manual review) |",
          "| **C** | ADS lane-keep failure (lane_inv > 5 or wrong_lane > 0) | **ADS** (tool success) |",
          "| **D** | ADS stalled (driven < 5 m, \\|jerk\\| < 1 m/s³) | **ADS** (tool success) |",
          "| **E** | ADS routing miss (drove ≥ 5 m but min_dist > 10 m, no collision) | **ADS** (tool success) |",
          "| SKIP | verdict ≠ deadlock (ok / drifted / missing) | — |",
          "",
          "## Thresholds (defensible)",
          "",
          "- **D** driven < 5 m: 60 s × 6 m/s nominal cruise ≈ 360 m. < 5 m = < 1.4 % route engagement → ADS not commanding powertrain.",
          "- **D** \\|max_long_jerk\\| < 1 m/s³: normal accel/brake events spike to 50-200 m/s³; < 1 = no throttle activity.",
          "- **C** lane_invasion_count > 5: ≥ 1 lane-cross every 12 s on average over 60 s.",
          "- **C** wrong_lane_count > 0: any moment in wrong lane is a hard fail.",
          "- **E** min_distance > 10 m: default NPC trigger distance is 15 m; > 10 m means ego never entered trigger range.",
          ""]

    for ads, cases in results.items():
        tally = Counter(info["class"] for info in cases.values())
        md.append(f"## ADS = `{ads}` (n={len(cases)})")
        md.append("")
        md.append("**Distribution**: " + " · ".join(f"{k}={v}" for k, v in sorted(tally.items())))
        md.append("")
        md.append("| case_id | runner_verdict | L3 verdict | class | reason | driven (m) | min_dist (m) | lane_inv |")
        md.append("|---|---|---|---|---|---|---|---|")
        for case, info in sorted(cases.items()):
            m = info["metrics"]
            md.append(
                f"| `{case}` | {m.get('runner_verdict')} | {m.get('l3_verdict')} | **{info['class']}** | "
                f"{info['reason']} | {m.get('driven_m')} | {m.get('min_dist_m')} | "
                f"{m.get('lane_invasion_count')} |"
            )
        md.append("")

    md.append("## §6 prose hook")
    md.append("")
    md.append("Of the 14 `deadlock` verdicts reported in the post-D batch on tfv6_regnet,")
    md.append("the C/D/E classes ({{C}}, {{D}}, {{E}} respectively) are testing-tool")
    md.append("**successes**: the pipeline produced a runnable scenario, and the L3 gate")
    md.append("correctly flagged that the ADS failed to engage the scenario as intended.")
    md.append("The A/B classes ({{A}}, {{B}}) are remaining engineering work, of which")
    md.append("none are silent-spawn / crash type after Sprint-3 F1-F4 (i.e., `A`-class is")
    md.append("expected to be near-zero post-fixes).")
    md.append("")
    md.append("→ replace `{{X}}` placeholders with actual counts from the tally above")
    md.append("when writing §6 prose.")

    OUT_MD.write_text("\n".join(md))

    print(f"Wrote {OUT_JSON.relative_to(REPO_ROOT)}")
    print(f"Wrote {OUT_MD.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
