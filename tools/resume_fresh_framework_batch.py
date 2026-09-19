#!/usr/bin/env python3
"""Resume CARLA phases from verified generation artifacts in this fresh batch.

Keeps original attempts intact. Uses the existing framework roadgraph client,
osc_blocks compiler and PCLA runner; never reads frozen maps or authors routes.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools import osc_blocks
from tools.audit_fresh_framework_batch import audit_attempt
from tools.carla_client import get_carla_client
from tools.coordinator import load_env, _shared_check_scene_outcome
from tools.ocl_constraints import violations as ocl_check
from tools.run_fresh_framework_batch import SerializedCarlaClient, digest, write, reject_frozen_artifact_inputs
from tools.validate_demo_run import validate
from tools.build_road_seed_opendrive import build as build_road
from tools.coordinator import DEFAULT_XSD
from tools.road_recompile_equivalence import equivalent_roads


def verify_generation(attempt: Path):
    audited = audit_attempt(attempt)
    if not audited.get("generation_pass"):
        raise ValueError("Cannot resume unverified generation: " + str(attempt))
    run = Path(audited["run"])
    result = json.loads((run / "result.json").read_text())
    for filename, key in (("scene_seed.json", "scene_seed"), ("road_seed.json", "road_seed")):
        if json.loads((run / filename).read_text()) != result[key]:
            raise ValueError(f"Generated artifact differs from QA result: {filename}")
    road = result["road_seed"].get("road", result["road_seed"])
    scene = result["scene_seed"].get("scene", result["scene_seed"])
    violations = ocl_check(road, scene)
    if violations:
        raise ValueError("Generated pair violates current OCL: " + str(violations))
    return run, result


def resume(attempt: Path, destination: Path, client, *, execute: bool, max_seconds: int,
           recompile_road: bool = False):
    old_run, old = verify_generation(attempt)
    destination.mkdir(parents=True, exist_ok=False)
    name = old["name"] + "_" + destination.name
    run = destination / "pipeline" / name
    run.mkdir(parents=True)
    for filename in ("source_text.txt", "source_extraction.json", "road_seed.json",
                     "scene_seed.json", "road_generation.json"):
        shutil.copy2(old_run / filename, run / filename)
    if (old_run / 'road_recompile_equivalence.json').is_file() and not recompile_road:
        shutil.copy2(old_run / 'road_recompile_equivalence.json', run / 'road_recompile_equivalence.json')
    shutil.copy2(attempt / "source.pdf", destination / "source.pdf")
    for folder in ("framework_code",):
        shutil.copytree(attempt / folder, destination / folder)
    for folder in ("qa", "modeling"):
        if (old_run / folder).is_dir():
            shutil.copytree(old_run / folder, run / folder)
    xodr = run / (name + ".xodr")
    if not recompile_road:
        shutil.copy2(old["xodr_path"], xodr)
    html = run / (name + ".html")
    if not recompile_road and old.get("html_path") and Path(old["html_path"]).is_file():
        shutil.copy2(old["html_path"], html)
    invocation = json.loads((attempt / "framework_invocation.json").read_text())
    invocation.update({"execution_kind": "resume_current_batch_generation",
                       "existing_seed_or_xodr_input": True,
                       "preexisting_frozen_input": False,
                       "resumed_from": str(attempt), "execute": execute,
                       "started_at": datetime.now(timezone.utc).isoformat()})
    write(destination / "framework_invocation.json", invocation)
    code_hashes = {}
    for folder in ("tools", "schemas", "xsd"):
        for p in (ROOT / folder).iterdir():
            if p.is_file() and p.suffix in (".py", ".md", ".xsd"):
                target = destination / "resume_code" / folder / p.name
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(p, target)
                code_hashes[str(p.relative_to(ROOT))] = digest(p)
    write(destination / "resume_provenance.json", {
        "parent_attempt": str(attempt), "parent_result_sha256": digest(old_run / "result.json"),
        "generation_preserved": {f: digest(run / f) for f in (
            "source_text.txt", "source_extraction.json", "road_seed.json", "scene_seed.json", "road_generation.json")},
        "xodr_sha256": digest(xodr) if xodr.is_file() else None, "code_sha256": code_hashes,
        "phase_entry": "carla_roadgraph", "original_generation_invocation": str(attempt / "framework_invocation.json"),
    })
    result = {k: v for k, v in old.items() if k not in ("scene_outcome", "demo_readiness")}
    result.update({"name": name, "run_dir": str(run), "xodr_path": str(xodr),
                   "html_path": str(html), "errors": {}, "roadgraph_status": "pending",
                   "xosc_status": "pending", "xosc_path": None, "carla_status": "pending",
                   "carla_summary": None, "carla_run_dir": None, "carla_elapsed_s": None,
                   "resumed_from": str(attempt), "phase_timings": [],
                   "upstream_phase_timings": old.get("phase_timings", [])})
    write(run / "result.json", result)
    phase = "road_compile" if recompile_road else "roadgraph"
    try:
        if recompile_road:
            started = datetime.now(timezone.utc).isoformat()
            compiled = build_road(result['road_seed'], xodr, html, DEFAULT_XSD)
            equivalence = equivalent_roads(Path(old['xodr_path']), xodr)
            evidence = {
                **equivalence, 'parent_attempt': str(attempt),
                'parent_result_path': str(old_run / 'result.json'),
                'parent_result_sha256': digest(old_run / 'result.json'),
                'parent_xodr_path': old['xodr_path'],
                'parent_xodr_sha256': digest(Path(old['xodr_path'])),
                'rebuilt_xodr_sha256': digest(xodr),
                'road_seed_sha256': digest(run / 'road_seed.json'),
                'scene_seed_sha256': digest(run / 'scene_seed.json'),
                'vlm_qa_rerun': False,
                'qa_reuse_basis': 'Verified same-batch seeds and strictly equivalent compiled road',
            }
            write(run / 'road_recompile_equivalence.json', evidence)
            result.update(road_recompiled=True, xodr_schema_valid=compiled.get('xodr_schema_valid'),
                          road_recompile_equivalence=equivalence)
            write(run / 'road_generation.json', {
                'operation': 'build_road_seed_opendrive.build', 'started_at': started,
                'completed_at': datetime.now(timezone.utc).isoformat(),
                'seed_path': str(run / 'road_seed.json'),
                'seed_sha256': digest(run / 'road_seed.json'), 'xodr_path': str(xodr),
                'xodr_sha256': digest(xodr),
                'compiler_sha256': digest(ROOT / 'tools/build_road_seed_opendrive.py'),
                'existing_xodr_used': False, 'seed_reused_from': str(attempt),
            })
            invocation['execution_kind'] = 'recompile_verified_current_batch_seed'
            write(destination / 'framework_invocation.json', invocation)
            provenance = json.loads((destination / 'resume_provenance.json').read_text())
            provenance['generation_preserved'].pop('road_generation.json')
            provenance.update(xodr_sha256=digest(xodr), phase_entry='road_compile',
                              road_recompile_equivalence_sha256=digest(run / 'road_recompile_equivalence.json'))
            write(destination / 'resume_provenance.json', provenance)
            if not equivalence['equivalent'] or compiled.get('xodr_schema_valid') is not True:
                raise ValueError('Rebuilt road is not schema-valid and strictly equivalent; new VLM QA required')
        phase = 'roadgraph'
        graph = client.extract_roadgraph(xodr, name=name)
        result.update({"roadgraph_status": "ok", "roadgraph_selfcheck": graph.selfcheck,
                       "roadgraph_cache_dir": str(graph.local_dir)})
        phase = "xosc"
        xosc = run / (name + ".xosc")
        osc_blocks.build_xosc(result["scene_seed"].get("scene", result["scene_seed"]),
                              xodr, xosc, name=name, auto_extract=False, road_seed=result["road_seed"])
        result.update({"xosc_status": "ok", "xosc_path": str(xosc)})
        write(run / "result.json", result)
        phase = "carla"
        if execute:
            runtime = client.run_scenario(xodr, xosc, name=name, out_dir=run,
                pcla_agent="if_if", max_seconds=max_seconds, timeout=max(900, max_seconds + 180),
                scene_seed=result["scene_seed"])
            result.update({"carla_status": "ok", "carla_summary": runtime.summary,
                "carla_run_dir": str(runtime.local_dir), "carla_elapsed_s": round(runtime.elapsed_s, 2),
                "pcla_agent": "if_if"})
            result["scene_outcome"] = _shared_check_scene_outcome(
                result["scene_seed"], runtime.summary, Path(runtime.local_dir) / "sim_feedback.json")
        else:
            result["carla_status"] = "disabled"
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        result[phase + "_status"] = "error: " + error
        result["errors"][phase] = error
    write(run / "result.json", result)
    result["demo_readiness"] = validate(run)
    write(run / "result.json", result)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--parent", required=True, type=Path)
    ap.add_argument("--attempt", required=True)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--recompile-road", action="store_true",
                    help="Rebuild verified fresh RoadSeed; reject any semantic road change before CARLA")
    ap.add_argument("--max-seconds", type=int, default=600)
    args = ap.parse_args()
    if not args.attempt.startswith("fresh_") or Path(args.attempt).name != args.attempt:
        ap.error("Attempt must be a single fresh_* directory name")
    parent = args.parent.resolve()
    expected_base = ROOT / "outputs/framework_42_20260917/cases"
    if not parent.is_relative_to(expected_base) or parent.parent.parent != expected_base:
        ap.error("Parent must be a current fresh-batch case attempt")
    load_env(ROOT / ".env.local")
    reject_frozen_artifact_inputs()
    endpoint = os.environ.get("CARLA_REMOTE_HOST", "localhost") + ":" + os.environ.get("CARLA_REMOTE_RPC_PORT", "2000")
    lock = Path("/tmp") / ("crash2openx_rpc_" + hashlib.sha256(endpoint.encode()).hexdigest()[:16] + ".lock")
    result = resume(parent, parent.parent / args.attempt,
                    SerializedCarlaClient(get_carla_client(), lock), execute=args.execute,
                    max_seconds=args.max_seconds, recompile_road=args.recompile_road)
    print(json.dumps({k: result.get(k) for k in (
        "name", "roadgraph_status", "xosc_status", "carla_status", "errors")}, ensure_ascii=False))
    return int(result["xosc_status"] != "ok" or (args.execute and result["carla_status"] != "ok"))


if __name__ == "__main__":
    raise SystemExit(main())
