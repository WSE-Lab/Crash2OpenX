#!/usr/bin/env python3
"""Audit fresh framework generation separately from autonomous outcomes."""
import argparse
import hashlib
import json
import math
import subprocess
import sys
from datetime import datetime, timezone
from html import escape
from pathlib import Path
from urllib.parse import quote

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.extract_source import pdf_narrative_coverage
from tools.ocl_constraints import violations as ocl_violations
from tools.qa_agent import _normalize_verdict
from tools.road_recompile_equivalence import equivalent_roads


def read(path):
    try:
        return json.loads(path.read_text()) if path.is_file() else {}
    except json.JSONDecodeError:
        return {}  # The live pipeline may be updating this file; retry next audit.


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def execution_integrity(summary, actor_minima):
    """Basic termination/height checks; not complete motion or source validity."""
    termination = str(summary.get('termination_reason') or '').lower()
    failures = []
    if not termination or any(word in termination for word in ('failure', 'exception', 'error')):
        failures.append('missing or failed scenario termination: ' + termination)
    if termination in ('sut_left_map_surface', 'map_surface_exit_failure'):
        failures.append('actor exited map surface')
    if not actor_minima or any(not math.isfinite(z) or z < -1 for z in actor_minima.values()):
        failures.append('actor height missing, nonfinite, or below map')
    if summary.get('scenario_tree_status') in ('Status.FAILURE', 'FAILURE'):
        failures.append('scenario behavior tree failed')
    return {'execution_integrity_pass': not failures, 'execution_integrity_issues': failures,
            'autonomous_task_success': None, 'source_reconstruction_accepted': None}


def audit_attempt(attempt):
    invocation = read(attempt / "framework_invocation.json")
    candidates = list((attempt / "pipeline").glob("*/result.json"))
    if not candidates:
        interruption = read(attempt / "interruption.json")
        status = ("failed" if (attempt / "exception.txt").exists() else
                  "interrupted" if interruption else "running")
        return {"attempt": attempt.name, "status": status,
                **({"interruption": interruption} if interruption else {})}
    run = candidates[0].parent
    result = read(run / "result.json")
    generation = read(run / "road_generation.json")
    extraction = read(run / "source_extraction.json")
    xodr = Path(result["xodr_path"]) if result.get("xodr_path") else run / "missing.xodr"
    xosc = Path(result["xosc_path"]) if result.get("xosc_path") else run / "missing.xosc"
    checks = {
        "framework_entrypoint": invocation.get("entrypoint") == "tools.coordinator.dispatch_full",
        "source_pdf_matches": sha(attempt / "source.pdf") == invocation.get("source_pdf_sha256") == extraction.get("source_sha256") and bool(extraction),
        "extracted_text_matches": sha(run / "source_text.txt") == extraction.get("text_sha256") and bool(extraction),
        "fresh_road_compilation": generation.get("operation") == "build_road_seed_opendrive.build" and generation.get("existing_xodr_used") is False,
        "generated_seed_matches": bool(generation) and sha(run / "road_seed.json") == generation.get("seed_sha256"),
        "generated_map_matches": bool(generation) and sha(xodr) == generation.get("xodr_sha256"),
        "generated_scene_matches_qa_result": bool(result.get("scene_seed")) and read(run / "scene_seed.json") == result.get("scene_seed"),
        "xodr_schema_pass": result.get("xodr_schema_valid") is True,
        "ocl_pass": result.get("ocl_status") == "pass",
        "qa_pass": result.get("qa", {}).get("final_verdict") == "pass",
    }
    qa_rounds = result.get("qa", {}).get("rounds") or []
    checks["current_qa_issues_resolved"] = bool(qa_rounds) and _normalize_verdict(qa_rounds[-1])["verdict"] == "pass"
    road = result.get("road_seed") or {}
    scene = result.get("scene_seed") or {}
    road_body = road.get("road", road) or {}
    scene_body = scene.get("scene", scene) or {}
    checks["current_ocl_pass"] = bool(road_body and scene_body) and not ocl_violations(
        road_body, scene_body)
    if result.get('road_recompiled'):
        evidence = read(run / 'road_recompile_equivalence.json')
        parent_map = Path(evidence.get('parent_xodr_path', 'missing.xodr'))
        parent_result = Path(evidence.get('parent_result_path', 'missing.json'))
        checks['recompiled_road_equivalent_to_reviewed_parent'] = bool(
            evidence.get('equivalent') is True and parent_map.is_file() and xodr.is_file()
            and sha(parent_map) == evidence.get('parent_xodr_sha256')
            and sha(xodr) == evidence.get('rebuilt_xodr_sha256')
            and sha(parent_result) == evidence.get('parent_result_sha256')
            and sha(run / 'road_seed.json') == evidence.get('road_seed_sha256')
            and sha(run / 'scene_seed.json') == evidence.get('scene_seed_sha256')
            and equivalent_roads(parent_map, xodr)['equivalent'])
    if (attempt / "source.pdf").is_file() and (run / "source_text.txt").is_file():
        coverage = pdf_narrative_coverage(attempt / "source.pdf", (run / "source_text.txt").read_text())
        checks["source_narrative_complete"] = coverage["passed"]
    else:
        checks["source_narrative_complete"] = False
    conversion_checks = {**checks, "carla_roadgraph": result.get("roadgraph_status") == "ok",
                         "scene_compiled": result.get("xosc_status") == "ok" and xosc.is_file()}
    summary = read(run / "summary.json")
    remote = read(run / "remote_run.json")
    runtime_checks = {
        "pipeline_conversion_pass": all(conversion_checks.values()),
        "uploaded_new_map": bool(remote) and remote.get("xodr_sha256") == sha(xodr),
        "uploaded_compiled_scene": bool(remote) and remote.get("xosc_sha256") == sha(xosc),
        "ticks_recorded": summary.get("total_ticks", 0) > 0,
        "pcla_executed": result.get("carla_status") == "ok" and remote.get("execution_mode") == "pcla_autonomous_sut",
        "required_evidence_present": all((run / name).is_file() for name in (
            "demo.log", "runner.log", "carla_server.log.gz", "scenario.runtime.xosc", "events.jsonl", "sim_trace_raw.jsonl", "carla_rgb.mp4")),
    }
    video = run / "carla_rgb.mp4"
    video_sha = sha(video)
    cache = attempt / "video_decode_audit.json"
    v = read(cache)
    if video_sha and v.get("sha256") != video_sha:
        decoded = subprocess.run(["ffmpeg", "-v", "error", "-i", str(video), "-f", "null", "-"],
                                 capture_output=True, text=True, timeout=60)
        probed = subprocess.run(["ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0",
                                "-show_entries", "stream=width,height,nb_read_frames,duration", "-of", "json", str(video)],
                               capture_output=True, text=True, timeout=60)
        streams = json.loads(probed.stdout or "{}").get("streams", [])
        v = {"sha256": video_sha, "decode_pass": decoded.returncode == 0 and not decoded.stderr.strip(),
             "decode_messages": decoded.stderr, "probe": streams[0] if streams else {}}
        cache.write_text(json.dumps(v, indent=2) + "\n")
    runtime_checks["video_decode_pass"] = bool(v.get("decode_pass"))
    states = [json.loads(line) for line in (run / "sim_trace_raw.jsonl").read_text().splitlines()] if (run / "sim_trace_raw.jsonl").is_file() else []
    minima = {}
    finite = True
    for row in states:
        for name, actor in row.get("actors", {}).items():
            minima[name] = min(minima.get(name, float("inf")), actor["z"])
            finite &= all(math.isfinite(actor.get("applied_control", {}).get(k, float("nan"))) for k in ("throttle", "brake", "steer")) if actor.get("type_id", "").startswith("vehicle.") else True
    runtime_checks["finite_vehicle_controls"] = bool(states) and finite
    runtime_checks["no_actor_fell_below_map"] = bool(minima) and min(minima.values()) > -1
    termination = summary.get("termination_reason")
    runtime_checks["no_map_exit_or_exception"] = bool(termination) and "exception" not in termination.lower() and termination not in ("sut_left_map_surface", "map_surface_exit_failure")
    log = (run / "demo.log").read_text(errors="replace") if (run / "demo.log").is_file() else ""
    runtime_checks["actual_interfuser_model_device_logged"] = "PCLA_MODEL_PARAMETER_DEVICE=" in log
    runtime_checks["gnss_coordinate_calibration_logged"] = "PCLA_GNSS_CALIBRATION=" in log
    outcome = result.get("scene_outcome", {})
    return {"attempt": attempt.name, "status": "finished", "run": str(run),
            **execution_integrity(summary, minima),
            "generation_checks": checks, "generation_pass": all(checks.values()),
            "conversion_checks": conversion_checks, "conversion_pass": all(conversion_checks.values()),
            "runtime_checks": runtime_checks, "runtime_evidence_pass": all(runtime_checks.values()),
            "termination": termination, "collision_count": summary.get("collision_count"),
            "intent_verdict": outcome.get("verdict"), "intent_issues": outcome.get("issues", []),
            "expected_collision": outcome.get("expected_collision"), "actual_collision": outcome.get("actual_collision_actors"),
            "offroad_seconds": summary.get("off_road_time"), "route_completion": summary.get("final_route_completion"),
            "recorded_simulation_seconds": states[-1]["simulation_time"] - states[0]["simulation_time"] if states else None,
            "actor_minimum_z": minima, "video": v,
            "terminal_braking_interventions": [line for line in log.splitlines()
                                               if line.startswith("NPC_TERMINAL_ROAD_BRAKING ")]}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("base", type=Path)
    args = ap.parse_args()
    base = args.base.resolve()
    cases = []
    cards = []
    for item in read(ROOT / "data/eval/baseline_42.json")["cases"]:
        cid = item["case_id"]
        attempts = [audit_attempt(p) for p in sorted((base / "cases" / cid).glob("fresh_*")) if p.is_dir()]
        cases.append({"case_id": cid, "attempts": attempts})
        links = []
        for a in attempts:
            if "run" not in a:
                links.append(f'<p>{escape(a["attempt"])}: {escape(a["status"])}</p>')
                continue
            run = Path(a["run"])
            prefix = str(run.relative_to(base))
            links.append(f'<details><summary>{escape(a["attempt"])} · 生成 {a["generation_pass"]} · 运行证据 {a["runtime_evidence_pass"]} · {escape(str(a["termination"]))} · 碰撞传感器记录 {a["collision_count"]}</summary>')
            links.append(f'<p>行为检查：{escape(str(a["intent_verdict"]))}。'
                         + escape('; '.join(a.get('intent_issues', []))) + '</p>')
            if a.get('terminal_braking_interventions'):
                links.append('<p>本次触发了生成道路末端制动保护；不能据此认定原文事故复现。</p>')
            if (run / "carla_rgb.mp4").is_file():
                links.append(f'<video controls preload="none" src="{quote(prefix + "/carla_rgb.mp4")}"></video>')
            links.append('<p>')
            for filename in ["road_seed.json", "scene_seed.json", "road_generation.json", "source_text.txt", run.name + ".xodr", run.name + ".xosc", "demo.log", "result.json"]:
                if (run / filename).is_file():
                    links.append(f'<a href="{quote(prefix + "/" + filename)}">{escape(filename)}</a> ')
            links.append('</p></details>')
        cards.append(f'<article><h2>{escape(cid)}</h2>{"".join(links) or "尚未开始"}</article>')
    report = {"updated_at": datetime.now(timezone.utc).isoformat(), "case_count": 42,
              "generated_cases": sum(any(a.get("generation_pass") for a in c["attempts"]) for c in cases),
              "compiled_scenario_cases": sum(any(a.get("conversion_pass") for a in c["attempts"]) for c in cases),
              "runtime_evidence_cases": sum(any(a.get("runtime_evidence_pass") for a in c["attempts"]) for c in cases),
              "note": "Runtime evidence is not autonomous task success or source accident reproduction. SUT follows framework role selection and is not necessarily the reported AV.", "cases": cases}
    (base / "audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    (base / "index.html").write_text('<!doctype html><meta charset="utf-8"><title>框架重新生成实验</title><style>body{max-width:1050px;margin:32px auto;font:16px/1.6 system-ui}article{border-top:1px solid #ccc;padding:16px}video{width:100%}a{margin-right:12px}</style><h1>框架重新生成实验 · 进行中</h1><p>原PDF→新seed→新地图→CARLA路网→XOSC→PCLA。生成完成与自主碰撞结果分开记录，SUT不一定是报告中的AV。</p>' + ''.join(cards))
    print(json.dumps({k: v for k, v in report.items() if k != "cases"}, ensure_ascii=False))


if __name__ == "__main__":
    main()
