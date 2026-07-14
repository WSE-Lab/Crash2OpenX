#!/usr/bin/env python3
"""Run all 18 medoids on the remote CARLA server and collect verdicts.

For each medoid:
  1. Push xodr + xosc to remote, run scenario_runner with PCLA agent
  2. Pull back carla_rgb.mp4 + summary.json + sim_feedback.json + behavior_check.json
  3. Classify outcome:
       ok          — total_ticks > 0, behavior verdict 'aligned'
       drifted     — ran but behavior intent mismatch (no collision, wrong-lane, ...)
       deadlock    — no collision detected though intent had one (kinematics stuck)
       silent_spawn — total_ticks==0 (PCLA failed to attach)
       crash       — remote run exited non-zero
  4. Write per-case row to outputs/medoid_runs/_summary.tsv

Use --pilot to only run the 4-case representative subset (one per topology + W3
sanity check) before committing to the full 30-min batch.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.carla_client import get_carla_client, CarlaRemoteError  # noqa: E402

PILOT_CASES = [
    "038_Waymo_December_17_2024",   # #6  straight + adjacent|cut_in (size 7)
    "193_Waymo_June_29_2023_(2)",   # #8  cross_intersection + stopped_ahead (size 19)
    "435_Waymo_November_26_2021",   # #15 opposing_leg + static_hold (W3 win)
    "273_Cruise_December_18_2022",  # #18 2-NPC sideswipe
]


def classify(rr) -> tuple[str, str]:
    """Return (verdict, detail) given a RunResult."""
    s = rr.summary or {}
    ticks = s.get("total_ticks", 0)
    term = s.get("termination_reason", "")
    if ticks == 0:
        return "silent_spawn", f"term={term}"
    b = rr.behavior or {}
    bv = b.get("verdict", "skipped")
    issues = b.get("issues") or []
    if bv == "aligned":
        return "ok", f"ticks={ticks} aligned"
    if bv == "drifted":
        # subdivide: deadlock = expected collision + none occurred
        if any("expected collision but none occurred" in i for i in issues):
            return "deadlock", "; ".join(issues)
        return "drifted", "; ".join(issues)
    return bv, "; ".join(issues) if issues else f"ticks={ticks}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--medoids", type=Path, default=ROOT / "paper/eval_medoids_n100.json")
    ap.add_argument("--out-root", type=Path, default=ROOT / "outputs/medoid_runs")
    ap.add_argument("--max-seconds", type=int, default=60)
    ap.add_argument("--pcla-agent", default="tfv6_regnet",
                    help="PCLA agent id passed to runner.sh (e.g. tfv6_regnet, if_if, carl_roach)")
    ap.add_argument("--pilot", action="store_true",
                    help="only run PILOT_CASES subset for smoke test")
    ap.add_argument("--cases", nargs="+", default=[],
                    help="explicit case_id list (overrides pilot/full)")
    ap.add_argument("--skip-existing", action="store_true",
                    help="skip cases whose run dir already has summary.json")
    args = ap.parse_args()

    meta = json.loads(args.medoids.read_text())
    all_meds = [m["medoid"] for m in meta["medoids"]]
    if args.cases:
        cases = args.cases
    elif args.pilot:
        cases = PILOT_CASES
    else:
        cases = all_meds

    print(f"=== batch_run_medoids ===")
    print(f"  total : {len(cases)} cases (pilot={args.pilot})")
    print(f"  out   : {args.out_root}")
    print(f"  agent : {args.pcla_agent}")
    print(f"  max-s : {args.max_seconds}")
    args.out_root.mkdir(parents=True, exist_ok=True)
    summary_tsv = args.out_root / "_summary.tsv"
    if not summary_tsv.is_file():
        summary_tsv.write_text("ts\tcid\tverdict\tdetail\telapsed_s\trun_dir\n", encoding="utf-8")

    client = get_carla_client()
    client.deploy_runner()  # remote: refresh runner.sh; local: chmod runner_local.sh

    results = []
    t_total = time.time()
    for i, cid in enumerate(cases, 1):
        run_dir = args.out_root / cid
        if args.skip_existing and (run_dir / "summary.json").is_file():
            print(f"  [{i:>2}/{len(cases)}] · {cid:<55} (skip — exists)")
            continue
        xodr = ROOT / f"outputs/opendrive_seed/{cid}.xodr"
        xosc = ROOT / f"outputs/medoid_xosc/{cid}.xosc"
        if not (xodr.is_file() and xosc.is_file()):
            print(f"  [{i:>2}/{len(cases)}] ✗ {cid:<55} missing xodr/xosc")
            continue
        scene_seed = json.loads((ROOT / f"outputs/scene_seed/{cid}.json").read_text())
        t0 = time.time()
        try:
            rr = client.run_scenario(
                xodr, xosc,
                name=cid,
                scene_seed=scene_seed,
                pcla_agent=args.pcla_agent,
                max_seconds=args.max_seconds,
                out_dir=run_dir,  # merge mode → flat per-case layout
            )
            verdict, detail = classify(rr)
        except CarlaRemoteError as exc:
            verdict, detail = "crash", str(exc).splitlines()[0][:160]
            rr = None
        except Exception as exc:  # noqa: BLE001
            verdict, detail = "crash", f"{type(exc).__name__}: {exc}"[:160]
            rr = None
        elapsed = time.time() - t0
        flag = {"ok":"✓","drifted":"≈","deadlock":"⛔","silent_spawn":"○","crash":"✗"}.get(verdict, "?")
        print(f"  [{i:>2}/{len(cases)}] {flag} {cid:<55}  [{verdict}] {detail[:80]}  ({elapsed:.0f}s)")
        with summary_tsv.open("a", encoding="utf-8") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}\t{cid}\t{verdict}\t{detail}\t{elapsed:.1f}\t{run_dir}\n")
        results.append((cid, verdict, detail, elapsed))

    total_elapsed = time.time() - t_total
    print(f"\n=== done in {total_elapsed/60:.1f} min ===")
    counts: dict[str, int] = {}
    for _, v, _, _ in results:
        counts[v] = counts.get(v, 0) + 1
    for v, c in sorted(counts.items(), key=lambda x: -x[1]):
        print(f"  {v:<15}  {c}")
    print(f"\nrows appended to {summary_tsv.relative_to(ROOT)}")
    print(f"per-case artifacts under {args.out_root.relative_to(ROOT)}/<case>/")
    bad = [c for c, v, _, _ in results if v != "ok"]
    if bad:
        print(f"\nnon-ok cases ({len(bad)}):")
        for c, v, d, _ in results:
            if v != "ok":
                print(f"  {v:>12s}  {c}  :: {d}")
    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
