#!/usr/bin/env python3
"""Rebuild outputs/opendrive_seed/*.xodr for the eval medoids.

Replays build_road_seed_opendrive.build() for each medoid using the existing
road_seed.json on disk. Use this after editing ROAD_TYPE_DEFAULTS (or any
road-emission code) to refresh the cached xodr files without running the
full pipeline.

    uv run python tools/batch_rebuild_medoid_xodrs.py
    uv run python tools/batch_rebuild_medoid_xodrs.py --cases 665_GM_Cruise_April_14_2019_\\(2\\)
"""
from __future__ import annotations

import argparse
import json
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.build_road_seed_opendrive import build, read_seed  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--medoids", type=Path, default=ROOT / "paper/eval_medoids_n100.json")
    ap.add_argument("--xodr-dir", type=Path, default=ROOT / "outputs/opendrive_seed")
    ap.add_argument("--html-dir", type=Path, default=ROOT / "outputs/road_html")
    ap.add_argument("--xsd", type=Path, default=ROOT / "xsd/OpenDRIVE_1.5M.xsd")
    ap.add_argument("--cases", nargs="+", default=[])
    args = ap.parse_args()

    if args.cases:
        cases = args.cases
    else:
        meta = json.loads(args.medoids.read_text())
        cases = [m["medoid"] for m in meta["medoids"]]

    args.xodr_dir.mkdir(parents=True, exist_ok=True)
    args.html_dir.mkdir(parents=True, exist_ok=True)
    print(f"=== batch_rebuild_medoid_xodrs ({len(cases)} cases) ===")

    ok = 0
    failed: list[tuple[str, str]] = []
    skipped: list[tuple[str, str]] = []
    for i, cid in enumerate(cases, 1):
        seed_path = ROOT / f"outputs/road_seed/{cid}.json"
        if not seed_path.is_file():
            skipped.append((cid, "missing road_seed"))
            print(f"  [{i:>2}/{len(cases)}] · {cid:<55} (skip — missing road_seed)")
            continue
        try:
            seed = read_seed(seed_path)
            xodr_path = args.xodr_dir / f"{cid}.xodr"
            html_path = args.html_dir / f"{cid}.html"
            result = build(seed, xodr_path, html_path, args.xsd)
            valid = result.get("xodr_schema_valid")
            road_len = result.get("max_road_length") or "?"
            tag = "✓" if valid else "⚠"
            print(f"  [{i:>2}/{len(cases)}] {tag} {cid:<55} max_road_len={road_len}")
            if valid:
                ok += 1
            else:
                failed.append((cid, str(result.get("xodr_schema_error", "?"))))
        except Exception as exc:  # noqa: BLE001
            failed.append((cid, f"{type(exc).__name__}: {exc}"))
            print(f"  [{i:>2}/{len(cases)}] ✗ {cid:<55} {type(exc).__name__}: {str(exc)[:80]}")
            traceback.print_exc()

    print(f"\n=== done: {ok} ok, {len(failed)} failed, {len(skipped)} skipped ===")
    if failed:
        print("\nfailures:")
        for cid, reason in failed:
            print(f"  ✗ {cid}\n      {reason[:200]}")
    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
