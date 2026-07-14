#!/usr/bin/env python3
"""Rebuild outputs/medoid_xosc/*.xosc for the eval medoids.

Replays osc_blocks.build_xosc for each medoid using the existing scene_seed /
road_seed / xodr / map_cache on disk. Use this after editing osc_blocks.py to
refresh the cached xosc files without running the full pipeline.

    uv run python tools/batch_build_medoids_xosc.py
    uv run python tools/batch_build_medoids_xosc.py --medoids paper/eval_medoids.json
    uv run python tools/batch_build_medoids_xosc.py --cases 665_GM_Cruise_April_14_2019_\\(2\\)
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

from tools import osc_blocks  # noqa: E402


def _load_inputs(cid: str) -> tuple[dict, dict, Path] | None:
    scene_path = ROOT / f"data/seeds/scene_seed/{cid}.json"
    road_path = ROOT / f"data/seeds/road_seed/{cid}.json"
    xodr_path = ROOT / f"data/compiled/opendrive_seed/{cid}.xodr"
    if not (scene_path.is_file() and xodr_path.is_file()):
        return None
    seed = json.loads(scene_path.read_text())
    if seed.get("status") != "supported":
        return None
    scene = seed.get("scene", seed)
    road_seed = json.loads(road_path.read_text()) if road_path.is_file() else None
    return scene, road_seed, xodr_path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--medoids", type=Path, default=ROOT / "data/eval/eval_medoids_top42.json")
    ap.add_argument("--out-dir", type=Path, default=ROOT / "data/compiled/medoid_xosc")
    ap.add_argument("--cases", nargs="+", default=[],
                    help="explicit case_ids (overrides the medoids file)")
    args = ap.parse_args()

    if args.cases:
        cases = args.cases
    else:
        meta = json.loads(args.medoids.read_text())
        cases = [m.get("case_id") or m["medoid"] for m in meta["medoids"]]

    args.out_dir.mkdir(parents=True, exist_ok=True)
    print(f"=== batch_build_medoids_xosc ({len(cases)} cases) ===")

    ok = 0
    failed: list[tuple[str, str]] = []
    skipped: list[tuple[str, str]] = []
    for i, cid in enumerate(cases, 1):
        loaded = _load_inputs(cid)
        if loaded is None:
            skipped.append((cid, "missing scene/xodr or scene status != supported"))
            print(f"  [{i:>2}/{len(cases)}] · {cid:<55} (skip — missing inputs)")
            continue
        scene, road_seed, xodr_path = loaded
        out_path = args.out_dir / f"{cid}.xosc"
        try:
            osc_blocks.build_xosc(scene, str(xodr_path), out_path,
                                  name=cid, auto_extract=False,
                                  road_seed=road_seed)
            ok += 1
            print(f"  [{i:>2}/{len(cases)}] ✓ {cid:<55} → {out_path.name}")
        except osc_blocks.WFViolation as exc:
            failed.append((cid, f"WFViolation: {exc}"))
            print(f"  [{i:>2}/{len(cases)}] ⚠ {cid:<55} WFViolation: {str(exc)[:80]}")
        except osc_blocks.BlockUnsupported as exc:
            failed.append((cid, f"BlockUnsupported: {exc}"))
            print(f"  [{i:>2}/{len(cases)}] ⚠ {cid:<55} BlockUnsupported: {str(exc)[:80]}")
        except Exception as exc:  # noqa: BLE001
            failed.append((cid, f"{type(exc).__name__}: {exc}"))
            print(f"  [{i:>2}/{len(cases)}] ✗ {cid:<55} {type(exc).__name__}: {str(exc)[:80]}")
            traceback.print_exc()

    print(f"\n=== done: {ok} ok, {len(failed)} failed, {len(skipped)} skipped ===")
    if failed:
        print("\nfailures:")
        for cid, reason in failed:
            print(f"  ✗ {cid}\n      {reason[:200]}")
    if skipped:
        print("\nskipped:")
        for cid, reason in skipped:
            print(f"  · {cid} ({reason})")
    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
