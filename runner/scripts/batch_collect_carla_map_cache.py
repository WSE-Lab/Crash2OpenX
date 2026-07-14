#!/usr/bin/env python3
"""Batch collect CARLA-derived map cache for OpenDRIVE seed files."""

import argparse
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from collect_carla_map_cache import (  # noqa: E402
    DEFAULT_INPUT_DIR,
    DEFAULT_OUTPUT_ROOT,
    collect_map_cache,
    resolve_xodr,
)


def is_pass_manifest(path):
    try:
        return json.loads(path.read_text(encoding="utf-8")).get("status") == "pass"
    except (OSError, json.JSONDecodeError):
        return False


def resolve_cases(args):
    input_dir = Path(args.input_dir).resolve()
    if args.cases:
        return [str(case) for case in args.cases]
    return [str(path.resolve()) for path in sorted(input_dir.glob(args.glob))]


def build_single_args(args, case):
    return SimpleNamespace(
        case=case,
        input_dir=args.input_dir,
        output_root=args.output_root,
        host=args.host,
        port=args.port,
        timeout=args.timeout,
        spacing=args.spacing,
        vertex_distance=args.vertex_distance,
        wall_height=args.wall_height,
        additional_width=args.additional_width,
        max_derived_spawn_points=args.max_derived_spawn_points,
        max_routes=args.max_routes,
        min_route_points=args.min_route_points,
    )


def case_output_dir(output_root, input_dir, case):
    path = Path(case)
    if path.is_file():
        stem = path.stem
    else:
        stem = resolve_xodr(str(case), Path(input_dir).resolve()).stem
    return Path(output_root).resolve() / stem


def write_summary(output_root, summary):
    path = Path(output_root).resolve() / "batch_manifest.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    return path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect CARLA-derived waypoint/topology cache for many OpenDRIVE maps."
    )
    parser.add_argument("cases", nargs="*", help="Optional case ids/names or direct .xodr paths.")
    parser.add_argument("--input-dir", default=str(DEFAULT_INPUT_DIR), help="Directory containing .xodr maps.")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT), help="Output root for map cache.")
    parser.add_argument("--glob", default="*.xodr", help="Glob used under --input-dir when no cases are given.")
    parser.add_argument("--host", default="localhost", help="CARLA host.")
    parser.add_argument("--port", type=int, default=2000, help="CARLA port.")
    parser.add_argument("--timeout", type=float, default=30.0, help="CARLA client timeout in seconds.")
    parser.add_argument("--spacing", type=float, default=2.0, help="Waypoint sampling spacing in meters.")
    parser.add_argument("--vertex-distance", type=float, default=2.0, help="OpenDRIVE mesh vertex distance.")
    parser.add_argument("--wall-height", type=float, default=1.0, help="OpenDRIVE wall height.")
    parser.add_argument("--additional-width", type=float, default=0.6, help="OpenDRIVE additional road width.")
    parser.add_argument("--max-derived-spawn-points", type=int, default=200, help="Max derived spawn points.")
    parser.add_argument("--max-routes", type=int, default=80, help="Max route candidates.")
    parser.add_argument("--min-route-points", type=int, default=8, help="Minimum points per route candidate.")
    parser.add_argument("--force", action="store_true", help="Rebuild caches even when manifest status is pass.")
    parser.add_argument("--stop-on-error", action="store_true", help="Stop at the first failed map.")
    return parser.parse_args()


def main():
    args = parse_args()
    cases = resolve_cases(args)
    if not cases:
        print(f"No .xodr files found under {Path(args.input_dir).resolve()}", file=sys.stderr)
        return 2

    started = time.perf_counter()
    results = []
    for index, case in enumerate(cases, start=1):
        try:
            output_dir = case_output_dir(args.output_root, args.input_dir, case)
        except Exception:
            output_dir = Path(args.output_root).resolve() / Path(case).stem
        manifest_path = output_dir / "manifest.json"
        print(f"[{index}/{len(cases)}] {case}")

        if not args.force and is_pass_manifest(manifest_path):
            print(f"  skip existing pass cache: {output_dir}")
            results.append({"case": case, "status": "skip", "output_dir": str(output_dir)})
            continue

        case_started = time.perf_counter()
        try:
            manifest = collect_map_cache(build_single_args(args, case))
            results.append(
                {
                    "case": case,
                    "status": manifest.get("status", "unknown"),
                    "output_dir": str(output_dir),
                    "elapsed_seconds": round(time.perf_counter() - case_started, 3),
                }
            )
        except Exception as exc:  # pylint: disable=broad-except
            print(f"  failed: {exc}", file=sys.stderr)
            results.append(
                {
                    "case": case,
                    "status": "fail",
                    "output_dir": str(output_dir),
                    "error": str(exc),
                    "elapsed_seconds": round(time.perf_counter() - case_started, 3),
                }
            )
            if args.stop_on_error:
                break

    counts = {}
    for result in results:
        counts[result["status"]] = counts.get(result["status"], 0) + 1

    summary = {
        "input_dir": str(Path(args.input_dir).resolve()),
        "output_root": str(Path(args.output_root).resolve()),
        "total": len(cases),
        "processed": len(results),
        "counts": counts,
        "elapsed_seconds": round(time.perf_counter() - started, 3),
        "results": results,
    }
    summary_path = write_summary(args.output_root, summary)
    print(f"Batch summary written: {summary_path}")
    print(json.dumps({key: summary[key] for key in ("total", "processed", "counts")}, indent=2))
    return 1 if counts.get("fail") else 0


if __name__ == "__main__":
    raise SystemExit(main())
