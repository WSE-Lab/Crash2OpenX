#!/usr/bin/env python3
"""Hard acceptance gate for a five-minute end-to-end demo run."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

from tools.scene_outcome import (
    MAX_MINOR_LATERAL_EXCESS_M,
    MAX_MINOR_OFF_ROAD_S,
    MAX_MINOR_PRE_IMPACT_S,
)


REQUIRED_PIPELINE_FILES = (
    "result.json",
    "road_seed.json",
    "scene_seed.json",
    "carla_rgb.mp4",
    "summary.json",
    "sim_feedback.json",
)


def _json(path: Path) -> dict:
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return {}


def validate(run_dir: Path) -> dict:
    run_dir = run_dir.resolve()
    checks: list[dict] = []

    def check(name: str, passed: bool, detail: str) -> None:
        checks.append({"name": name, "passed": bool(passed), "detail": detail})

    missing = [name for name in REQUIRED_PIPELINE_FILES if not (run_dir / name).is_file()]
    check("pipeline_artifacts", not missing,
          "complete" if not missing else f"missing: {', '.join(missing)}")
    check("openx_pair", bool(list(run_dir.glob("*.xodr"))) and bool(list(run_dir.glob("*.xosc"))),
          "XODR + XOSC present")

    result = _json(run_dir / "result.json")
    summary = _json(run_dir / "summary.json")
    outcome = result.get("scene_outcome") or _json(run_dir / "behavior_check.json")

    check("carla_completed", result.get("carla_status") == "ok",
          f"status={result.get('carla_status')}")
    check("collision_triggered", (summary.get("collision_count") or 0) > 0,
          f"count={summary.get('collision_count')}")
    expected = sorted(outcome.get("expected_collision") or [])
    actual = sorted(outcome.get("actual_collision_actors") or [])
    check("collision_pair", bool(expected) and expected == actual,
          f"expected={expected or None}, actual={actual or None}")
    check("intent_aligned", outcome.get("aligned") is True,
          f"verdict={outcome.get('verdict')}, issues={outcome.get('issues') or []}")

    wrong_lane = int(outcome.get("wrong_lane_count") or 0)
    lane_invasions = int(summary.get("lane_invasion_count") or 0)
    off_road_time = float(summary.get("off_road_time") or 0.0)
    check("wrong_lane", wrong_lane == 0, f"count={wrong_lane}")
    check("lane_discipline", lane_invasions <= 2, f"invasions={lane_invasions} (max 2)")
    max_excess = outcome.get("max_lateral_excess_m")
    pre_impact = outcome.get("off_road_before_collision_s")
    if max_excess is None:
        off_road_ok = off_road_time <= 0.25
        off_road_detail = f"time={off_road_time:.2f}s; trace geometry unavailable"
    else:
        off_road_ok = (
            off_road_time <= MAX_MINOR_OFF_ROAD_S
            and float(max_excess) <= MAX_MINOR_LATERAL_EXCESS_M
            and (pre_impact is None or float(pre_impact) <= MAX_MINOR_PRE_IMPACT_S)
        )
        off_road_detail = (
            f"time={off_road_time:.2f}s, max center excess={float(max_excess):.3f}m"
            + (f", began {float(pre_impact):.2f}s before impact" if pre_impact is not None else "")
        )
    check("off_road", off_road_ok, off_road_detail)

    video = run_dir / "carla_rgb.mp4"
    check("video", video.is_file() and video.stat().st_size >= 100_000,
          f"size={video.stat().st_size if video.is_file() else 0} bytes")
    frames = sorted((run_dir / "rgb_frames").glob("*.jpg"))
    if len(frames) >= 3:
        sample_indexes = sorted({0, len(frames) // 4, len(frames) // 2,
                                 (3 * len(frames)) // 4, len(frames) - 1})
        hashes = {hashlib.sha256(frames[i].read_bytes()).hexdigest() for i in sample_indexes}
        dynamic = len(hashes) >= 3
        detail = f"frames={len(frames)}, unique_sampled={len(hashes)}/{len(sample_indexes)}"
    else:
        # Compact transfers preserve the MP4 and timestamps without thousands
        # of JPEGs. Judge the actual recording instead of treating absent JPEG
        # intermediates as an empty/static video.
        try:
            decoded = subprocess.run(
                ["ffmpeg", "-v", "error", "-i", str(video), "-an", "-vf",
                 "fps=2,scale=160:-2", "-f", "framemd5", "-"],
                capture_output=True, text=True, timeout=45, check=True,
            )
            hashes = {line.rsplit(",", 1)[-1].strip()
                      for line in decoded.stdout.splitlines()
                      if line.strip() and not line.startswith("#")}
            dynamic = len(hashes) >= 3
            detail = f"decoded MP4 at 2fps; unique_sampled={len(hashes)}; JPEG intermediates={len(frames)}"
        except (OSError, subprocess.SubprocessError) as exc:
            dynamic = False
            detail = f"MP4 decode failed: {type(exc).__name__}; JPEG intermediates={len(frames)}"
    check("dynamic_video", dynamic, detail)

    passed = all(item["passed"] for item in checks)
    return {"run_dir": str(run_dir), "passed": passed, "checks": checks}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir", type=Path)
    args = ap.parse_args()
    report = validate(args.run_dir)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
