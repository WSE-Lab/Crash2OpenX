#!/usr/bin/env python3
"""Recompile OpenDRIVE/OpenSCENARIO artifacts from a frozen seed pair.

Paper Sec. 4: everything after the inference layer is deterministic, so any
reader can rerun the compiled products from a seed pair without re-calling
the LLM. This script is that reproduction path:

    uv run python scripts/compile_from_seeds.py \
        --road examples/rainy_night_front_brake/road_seed.json \
        --scene examples/rainy_night_front_brake/scene_seed.json \
        --out outputs/repro/rainy_night

Steps: OCL gate (Table 1, I1-I9 + P1-P6) -> XODR compile + OpenDRIVE 1.5 XSD
-> XOSC compile (WF gates inside osc_blocks) + OpenSCENARIO 1.0 XSD.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import xmlschema  # noqa: E402

from tools.build_road_seed_opendrive import build as build_road  # noqa: E402
from tools.ocl_constraints import violations as ocl_check  # noqa: E402
from tools.osc_blocks import build_xosc  # noqa: E402

XODR_XSD = ROOT / "xsd" / "OpenDRIVE_1.5M.xsd"
XOSC_XSD = ROOT / "xsd" / "OpenSCENARIO.xsd"


def _load(path: Path, wrapper: str) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    return data.get(wrapper, data)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--road", type=Path, required=True, help="road_seed.json")
    ap.add_argument("--scene", type=Path, required=True, help="scene_seed.json")
    ap.add_argument("--out", type=Path, required=True, help="output directory")
    ap.add_argument("--name", default=None, help="artifact base name (default: out dir name)")
    args = ap.parse_args()

    road = _load(args.road, "road")
    scene = _load(args.scene, "scene")
    name = args.name or args.out.name
    args.out.mkdir(parents=True, exist_ok=True)

    # 1) OCL gate over the seed pair.
    viol = ocl_check(road, scene)
    if viol:
        print(f"OCL violations: {', '.join(viol)} — aborting compilation.")
        return 1
    print("[1/3] OCL gate: I1-I9 + P1-P6 all pass")

    # 2) RoadSeed -> XODR (+ preview HTML), validated against OpenDRIVE 1.5.
    xodr_path = args.out / f"{name}.xodr"
    html_path = args.out / f"{name}_preview.html"
    build_road({"road": road}, xodr_path, html_path, XODR_XSD)
    xmlschema.XMLSchema(str(XODR_XSD)).validate(str(xodr_path))
    print(f"[2/3] XODR compiled + XSD-valid: {xodr_path}")

    # 3) SceneSeed (+ RoadSeed WF gates) -> XOSC, validated in build_xosc
    # against OpenSCENARIO 1.0; re-validated here for a visible verdict.
    xosc_path = args.out / f"{name}.xosc"
    build_xosc(scene, str(xodr_path), xosc_path, name=name,
               road_seed={"road": road})
    xmlschema.XMLSchema(str(XOSC_XSD)).validate(str(xosc_path))
    print(f"[3/3] XOSC compiled + XSD-valid: {xosc_path}")

    print(json.dumps({"xodr": str(xodr_path), "xosc": str(xosc_path),
                      "ocl": "pass", "xsd": "pass"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
