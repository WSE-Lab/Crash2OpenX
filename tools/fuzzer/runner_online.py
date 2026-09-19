#!/usr/bin/env python3
"""Online CARLA post-processor for a fuzz run's archive.

After ``runner.py`` produced offline archive.json, this script picks top-N
elites by danger score, recompiles their XOSC against the pre-existing XODR
(topology is unchanged by our mutation set), and runs each on the remote
CARLA host. The four-dimension metrics from the paper's §4.3 rubric are
attached back onto each elite record.

Usage::
    python -m tools.fuzzer.runner_online \
        --run outputs/fuzz_runs/case165 \
        --xodr outputs/web_runs/live165_final/live165_final.xodr \
        --topn 3
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.coordinator import load_env
from tools.carla_client import get_carla_client, CarlaClientError
from tools import osc_blocks
from tools.scene_outcome import check_scene_outcome


def _load_json(p: Path) -> dict:
    return json.loads(p.read_text())


def _write_json(p: Path, obj) -> None:
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(obj, indent=2, ensure_ascii=False))


def run_online_batch(
    run_dir: Path,
    xodr_path: Path,
    topn: int = 3,
    max_seconds: int = 60,
    pcla_agent: str = "tfv6_regnet",
) -> dict:
    """Take top-N elites from a fuzz run and execute them on the remote CARLA."""
    archive = _load_json(run_dir / "archive.json")
    cells = sorted(archive["cells"], key=lambda c: -c["score"])[:topn]

    load_env(Path(REPO_ROOT / ".env.local"))
    client = get_carla_client()
    case_name = (_load_json(run_dir / "meta.json")).get("case_id", run_dir.name)

    # Pre-compute the roadgraph once — the XODR does not change across variants
    print(f"[roadgraph] extracting for {case_name} ...")
    t0 = time.time()
    rg = client.extract_roadgraph(xodr_path, name=case_name)
    print(f"  ok in {time.time() - t0:.1f}s (cached={rg.cached}) — {rg.local_dir}")

    online_out = run_dir / "online"
    online_out.mkdir(parents=True, exist_ok=True)

    results = []
    for i, cell in enumerate(cells, 1):
        elite_id = f"elite_{i:02d}"
        elite_dir = online_out / elite_id
        elite_dir.mkdir(parents=True, exist_ok=True)

        scene_seed = cell["scene_seed"]
        _write_json(elite_dir / "scene_seed.json", scene_seed)

        xosc_path = elite_dir / f"{elite_id}.xosc"
        print(f"[{elite_id}] danger={cell['score']:.2f}  ops={cell.get('ops_applied')}")
        try:
            osc_blocks.build_xosc(
                scene_seed.get("scene", scene_seed),
                str(xodr_path),
                xosc_path,
                name=case_name,
                auto_extract=False,
                road_seed=None,
            )
        except Exception as exc:
            print(f"  ✗ xosc compile failed: {exc}")
            results.append({"elite_id": elite_id, "cell": cell, "error": f"xosc: {exc}"})
            continue

        try:
            run = client.run_scenario(
                xodr_path, xosc_path,
                name=f"{case_name}__{elite_id}", out_dir=elite_dir,
                pcla_agent=pcla_agent,
                max_seconds=max_seconds, timeout=1800.0,
                scene_seed=scene_seed,
            )
        except Exception as exc:
            print(f"  ✗ carla run failed: {exc}")
            results.append({"elite_id": elite_id, "cell": cell, "error": f"carla: {exc}"})
            continue

        outcome = check_scene_outcome(
            scene_seed, run.summary,
            Path(run.local_dir) / "sim_feedback.json",
        )
        # 4-dimensional metrics from the CARLA summary
        s = run.summary or {}
        metrics = {
            "safety": {
                "collision_detected": s.get("collision_count", 0) > 0,
                "min_ttc": s.get("min_ttc"),
                "collision_actors": s.get("collision_actors"),
            },
            "effectiveness": {
                "route_completion": s.get("route_completion"),
                "termination": s.get("termination_reason"),
                "elapsed_s": round(run.elapsed_s, 2),
            },
            "compliance": {
                "lane_invasion_count": s.get("lane_invasion_count"),
                "wrong_lane_count": next(
                    (int(c.get("actual_value") or 0)
                     for c in (s.get("criteria") or [])
                     if c.get("name") == "WrongLaneTest"),
                    None),
            },
            "comfort": {
                # Placeholders — jerk/accel to be post-processed from frame_states.
                "note": "post-processing pending (jerk / max lateral accel)",
            },
        }
        result = {
            "elite_id": elite_id,
            "cell": cell,
            "run_dir": str(run.local_dir),
            "outcome_verdict": outcome.get("verdict"),
            "outcome_issues": outcome.get("issues"),
            "metrics": metrics,
        }
        _write_json(elite_dir / "result.json", result)
        results.append(result)
        print(f"  ✓ verdict={outcome.get('verdict')}  collision={metrics['safety']['collision_detected']}  route={metrics['effectiveness']['route_completion']}")

    summary = {
        "case": case_name,
        "xodr": str(xodr_path),
        "topn": topn,
        "results": results,
    }
    _write_json(online_out / "online_summary.json", summary)
    print(f"\nwrote {online_out}/online_summary.json  ({len(results)} elites processed)")
    return summary


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, required=True, help="fuzz run dir (contains archive.json)")
    ap.add_argument("--xodr", type=Path, required=True, help="XODR to reuse (topology unchanged)")
    ap.add_argument("--topn", type=int, default=3)
    ap.add_argument("--max-seconds", type=int, default=60)
    ap.add_argument("--agent", default="tfv6_regnet")
    args = ap.parse_args()
    run_online_batch(args.run, args.xodr, args.topn, args.max_seconds, args.agent)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
