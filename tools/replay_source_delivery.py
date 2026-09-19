#!/usr/bin/env python3
"""Replay exact files from a delivery through the configured CARLA/PCLA runtime."""
import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, required=True, help="Local crash2openx checkout containing tools/carla_remote.py and connection configuration")
    parser.add_argument("--case", required=True, help="Three-digit source ID or full case ID")
    parser.add_argument("--mode", choices=("reconstruction", "pcla"), required=True)
    parser.add_argument("--output", type=Path, required=True, help="New local directory for this replay")
    args = parser.parse_args()
    delivery = Path(__file__).resolve().parent
    manifest = json.loads((delivery / "manifest.json").read_text())
    matches = [c for c in manifest["cases"] if c["case_id"] == args.case or c["case_id"][:3] == args.case]
    if len(matches) != 1:
        parser.error("Case ID must identify exactly one delivery case")
    if args.output.exists():
        parser.error("Replay output must be a new directory")
    source = delivery / "cases" / matches[0]["case_id"] / args.mode
    if not (source / "scenario.xosc").is_file():
        parser.error("This case/mode has no selected runnable input")
    sys.path.insert(0, str(args.project.resolve()))
    from tools.carla_remote import RemoteCfg
    from tools.validate_source_reconstruction import SourceCaseClient
    client = SourceCaseClient(replace(RemoteCfg.from_env(), compact_artifacts=True, keep_remote_runs=True))
    client.run_scenario(source / "map.xodr", source / "scenario.xosc",
                        name="delivery_" + matches[0]["case_id"][:3] + "_" + args.mode,
                        sut_actor="hero" if args.mode == "pcla" else "", pcla_agent="if_if",
                        rgb_actor_role="hero", max_seconds=600 if args.mode == "pcla" else 110,
                        out_dir=args.output)


if __name__ == "__main__":
    main()
