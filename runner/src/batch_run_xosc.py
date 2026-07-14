import argparse
import os
import subprocess
import sys
from pathlib import Path


def parse_arguments():
    parser = argparse.ArgumentParser(description="Batch run OpenSCENARIO replay files with demo.py")
    parser.add_argument(
        "--scenario-dir",
        default="xosc_replay_dmv",
        help="Directory containing .xosc files, relative to repo root by default",
    )
    parser.add_argument("--pattern", default="*.xosc", help="Glob pattern under scenario-dir")
    parser.add_argument(
        "--only",
        nargs="*",
        default=None,
        help="Optional filename substrings to include, e.g. --only 001 013",
    )
    parser.add_argument(
        "--start-from",
        default=None,
        help="Skip sorted scenarios until this filename substring is reached",
    )
    parser.add_argument(
        "--batch-output",
        default=None,
        help="Optional batch output directory. Default: each scenario writes to runs/<scenario_name>",
    )
    parser.add_argument("--host", default="localhost", help="CARLA host passed to demo.py")
    parser.add_argument("--port", type=int, default=2000, help="CARLA port passed to demo.py")
    parser.add_argument("--timeout", default="10.0", help="CARLA client timeout passed to demo.py")
    parser.add_argument("--real-time-factor", type=float, default=1.0, help="Replay speed passed to demo.py")
    parser.add_argument("--post-run-hold", type=float, default=0.0, help="Final hold seconds passed to demo.py")
    parser.add_argument("--no-rendering", action="store_true", help="Run CARLA with no_rendering_mode enabled")
    parser.add_argument("--record-video", action="store_true", help="Generate a top-down trajectory video for each run")
    parser.add_argument("--video-fps", type=int, default=10, help="Trajectory video FPS passed to demo.py")
    parser.add_argument("--video-frame-stride", type=int, default=2, help="Trajectory video frame stride passed to demo.py")
    parser.add_argument(
        "--stop-on-fail",
        action="store_true",
        help="Stop the batch at the first non-zero demo.py exit code",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands without running them",
    )
    parser.add_argument(
        "--python",
        default=sys.executable,
        help="Python executable used to run demo.py. Defaults to the current interpreter",
    )
    parser.add_argument(
        "--demo-path",
        default="src/demo.py",
        help="Path to demo.py, relative to repo root by default",
    )
    parser.add_argument(
        "demo_args",
        nargs=argparse.REMAINDER,
        help="Extra arguments passed to demo.py after --, e.g. -- --disable-traffic-lights True",
    )
    return parser.parse_args()


def repo_root():
    return Path(__file__).resolve().parents[1]


def resolve_path(root, value):
    path = Path(value)
    if path.is_absolute():
        return path
    return root / path


def select_scenarios(scenario_dir, pattern, only, start_from):
    scenarios = sorted(scenario_dir.glob(pattern))
    scenarios = [path for path in scenarios if path.is_file()]

    if only:
        scenarios = [
            path for path in scenarios
            if any(token in path.name for token in only)
        ]

    if start_from:
        for index, path in enumerate(scenarios):
            if start_from in path.name:
                scenarios = scenarios[index:]
                break
        else:
            scenarios = []

    return scenarios


def clean_demo_args(args):
    if args and args[0] == "--":
        return args[1:]
    return args


def run_one(command, log_path, dry_run):
    if dry_run:
        print("DRY RUN:", " ".join(command))
        print(f"  log: {log_path}")
        return 0

    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log_file:
        process = subprocess.Popen(
            command,
            cwd=str(repo_root()),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        for line in process.stdout:
            print(line, end="")
            log_file.write(line)
        return process.wait()


def main():
    args = parse_arguments()
    root = repo_root()
    scenario_dir = resolve_path(root, args.scenario_dir)
    demo_path = resolve_path(root, args.demo_path)
    batch_output = resolve_path(root, args.batch_output) if args.batch_output else None
    extra_demo_args = clean_demo_args(args.demo_args)

    scenarios = select_scenarios(scenario_dir, args.pattern, args.only, args.start_from)
    if not scenarios:
        print(f"No scenarios matched: dir={scenario_dir}, pattern={args.pattern}")
        return 2

    print(f"Batch output: {batch_output or root / 'runs' / '<scenario_name>'}")
    print(f"Matched scenarios: {len(scenarios)}")
    for index, scenario in enumerate(scenarios, start=1):
        print(f"  [{index:02d}/{len(scenarios):02d}] {scenario.name}")

    failures = []
    for index, scenario in enumerate(scenarios, start=1):
        scenario_stem = scenario.stem
        episode_output = (batch_output / f"{index:02d}_{scenario_stem}") if batch_output else (root / "runs" / scenario_stem)
        log_path = episode_output / "demo.log"
        command = [
            args.python,
            str(demo_path),
            "--scenario",
            str(scenario),
            "--host",
            args.host,
            "--port",
            str(args.port),
            "--timeout",
            str(args.timeout),
            "--real-time-factor",
            str(args.real_time_factor),
            "--post-run-hold",
            str(args.post_run_hold),
            "--data-output",
            str(episode_output),
        ] + extra_demo_args
        if args.no_rendering:
            command.append("--no-rendering")
        if args.record_video:
            command.append("--record-video")
        command.extend(["--video-fps", str(args.video_fps)])
        command.extend(["--video-frame-stride", str(args.video_frame_stride)])

        print("-" * 80)
        print(f"[{index}/{len(scenarios)}] Running {scenario.name}")
        print(f"Output: {episode_output}")
        return_code = run_one(command, log_path, args.dry_run)
        print(f"[{index}/{len(scenarios)}] Exit code: {return_code}")

        if return_code != 0:
            failures.append((scenario.name, return_code, log_path))
            if args.stop_on_fail:
                break

    print("=" * 80)
    print(f"Batch finished. Total={len(scenarios)}, failures={len(failures)}")
    if failures:
        for name, return_code, log_path in failures:
            print(f"  FAIL {name}: exit={return_code}, log={log_path}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
