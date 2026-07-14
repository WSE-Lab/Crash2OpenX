#!/usr/bin/env python3
"""Recompute L3 outcome and demo readiness from immutable run artifacts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from tools.scene_outcome import check_scene_outcome
from tools.validate_demo_run import validate


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def rejudge(run_dir: Path) -> dict:
    run_dir = run_dir.resolve()
    result_path = run_dir / "result.json"
    result = _load(result_path)
    scene_seed = _load(run_dir / "scene_seed.json")
    summary = _load(run_dir / "summary.json")
    outcome = check_scene_outcome(scene_seed, summary, run_dir / "sim_feedback.json")

    _write(run_dir / "behavior_check.json", outcome)
    result["scene_outcome"] = outcome
    # validate() reads result.json, so persist the new outcome before invoking it.
    _write(result_path, result)
    result["demo_readiness"] = validate(run_dir)
    _write(result_path, result)
    return {"scene_outcome": outcome, "demo_readiness": result["demo_readiness"]}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir", type=Path)
    args = ap.parse_args()
    report = rejudge(args.run_dir)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["demo_readiness"]["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
