#!/usr/bin/env python3
"""Prepare and optionally run the five reviewed stationary rear-end reports.

The source AV remains the PCLA SUT. A rear vehicle approaches it; a queued
vehicle ahead supplies the reported traffic stop. Speeds/distances not given
by the report are explicitly recorded as reconstruction assumptions.
This utility deliberately has a smaller scope than the 42-case benchmark.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import xmlschema

from tools.carla_remote import CarlaRemoteClient, RemoteCfg
from tools.osc_blocks import build_xosc
from tools.source_runtime_evidence import evaluate

CASE_PREFIXES = {"122", "007", "065", "440", "448"}


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")


def prepare(cid, output):
    prefix = cid.split("_")[0]
    if prefix not in CASE_PREFIXES:
        raise ValueError(f"{cid} is not one of the five reviewed stationary rear-end cases")
    output.mkdir(parents=True, exist_ok=False)
    original = json.loads((ROOT / "data/seeds/scene_seed" / (cid + ".json")).read_text())
    road = json.loads((ROOT / "data/eval/fidelity_42/road_seed" / (cid + ".json")).read_text())
    speed = 2.2352 if prefix == "448" else 4.0
    gap = 8.0 if prefix == "448" else 12.0
    scene = {
        "sut": {"id": "ego", "kind": "vehicle", "maneuver": "straight",
                "params": {"initial_speed_mps": 0.0}},
        "npcs": [
            {"id": "rear_car", "kind": "vehicle", "position": "behind_same_lane", "side": "none",
             "behavior": {"block": "rear_hit", "params": {"gap": gap, "speed": speed}}},
            {"id": "queue_lead", "kind": "vehicle", "position": "ahead_same_lane", "side": "none",
             "behavior": {"block": "static_hold", "params": {"gap": 7.0}}},
        ],
        "collision": {"a": "rear_car", "b": "ego"},
        "control": original["scene"].get("control", "unknown"),
        "environment": original["scene"].get("environment", {}),
    }
    xodr = output / "map.xodr"
    xosc = output / "scenario.xosc"
    shutil.copy2(ROOT / "data/compiled/opendrive_seed" / (cid + ".xodr"), xodr)
    build_xosc(scene, str(xodr.resolve()), xosc, name=cid, road_seed=road)
    tree = ET.parse(xosc)
    root = tree.getroot()
    pose = root.find("./Storyboard/Init/Actions/Private[@entityRef='hero']/PrivateAction/TeleportAction/Position/LanePosition")
    road_xml = ET.parse(xodr).find(f"./road[@id='{pose.get('roadId')}']")
    road_length = float(road_xml.get("length"))
    junction = road_xml.find("./link/successor[@elementType='junction']") is not None
    spawn_s = road_length - 17 if junction else min(80, road_length * 0.5)
    pose.set("s", str(spawn_s))
    for obj in root.findall("./Entities/ScenarioObject"):
        obj.find("Vehicle").set("name", "vehicle.lincoln.mkz_2020" if obj.get("name") == "hero" else "vehicle.toyota.prius")
    catalogs = root.find("CatalogLocations")
    if catalogs is not None:
        catalogs.clear()
    root.find("./RoadNetwork/LogicFile").set("filepath", "map.xodr")
    tree.write(xosc, encoding="utf-8", xml_declaration=True)
    xmlschema.XMLSchema(str(ROOT / "xsd/OpenSCENARIO.xsd")).validate(str(xosc))
    xmlschema.XMLSchema(str(ROOT / "xsd/OpenDRIVE_1.5M.xsd")).validate(str(xodr))
    document = {
        "case_id": cid, "scene": scene,
        "source_review": "data/eval/baseline_42_source_review.json",
        "role_mapping": {"hero": "reported AV", "rear_car": "reported striking passenger car",
                         "queue_lead": "representative traffic queue vehicle; contextual inference"},
        "reconstruction_assumptions": {
            "rear_speed_mps": speed, "rear_speed_reported": prefix == "448",
            "rear_gap_m": gap, "queue_lead_gap_m": 7.0, "spawn_s_m": spawn_s,
            "vehicle_assets": "Stock CARLA Lincoln/ Prius approximate the reported vehicles; manufacturer geometry is not exact.",
            "traffic_control": "Stationary queued traffic supplies the stop constraint. Signal geometry and compliance are not validated.",
            "geometry": "Representative OpenDRIVE geometry from the frozen seed, not a surveyed road map.",
            "environment": "Retains the source-derived environment seed; any friction coefficient is a reconstruction assumption.",
        },
    }
    write_json(output / "scene_seed.json", document)
    write_json(output / "road_seed.json", road)
    write_json(output / "compile_validation.json", {"xodr_xsd": "pass", "xosc_xsd": "pass"})
    return document


def summarize(run_dir, case_prefix=None):
    summary = json.loads((run_dir / "summary.json").read_text())
    events = [json.loads(line) for line in (run_dir / "events.jsonl").read_text().splitlines()]
    contacts = [e for e in events if e.get("event_type") == "collision"
                and e.get("payload", {}).get("source") == "carla_collision_sensor"
                and set(e["payload"].get("actors", [])) == {"hero", "rear_car"}]
    feedback = json.loads((run_dir / "behavior_check.json").read_text())
    video = run_dir / "carla_rgb.mp4"
    probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                            "stream=width,height,avg_frame_rate,nb_frames,duration", "-of", "json", str(video)],
                           capture_output=True, text=True, check=True)
    manifest = {
        "physical_contact_present": bool(contacts),
        "physical_sensor_callbacks": len(contacts),
        "note": "Repeated callbacks during sustained contact are not distinct accidents.",
        "first_contact": contacts[0] if contacts else None,
        "clean_trajectory": feedback.get("trajectory_quality") == "clean",
        "impact_area": feedback.get("impact_area"),
        "off_road_seconds": summary.get("off_road_time"),
        "video_probe": json.loads(probe.stdout),
        "video_visual_review": "pending",
        "source_semantic_review": "stationary AV / approaching rear vehicle / rear impact",
        "runtime_evidence_pass": bool(contacts) and feedback.get("trajectory_quality") == "clean"
                                 and feedback.get("impact_area") == "rear",
        "sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in run_dir.iterdir() if p.is_file() and p.suffix in {".mp4", ".xosc", ".xodr", ".jsonl", ".log"}},
    }
    if case_prefix is not None:
        manifest["measured_source_sequence"] = evaluate(run_dir, case_prefix, [["hero", "rear_car"]])
        manifest["runtime_evidence_pass"] = manifest["runtime_evidence_pass"] and manifest["measured_source_sequence"]["pass"]
    write_json(run_dir / "validation.json", manifest)
    return manifest


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cases", nargs="+", default=sorted(CASE_PREFIXES))
    ap.add_argument("--output", type=Path, default=ROOT / "outputs/validated_42_20260917")
    ap.add_argument("--attempt", required=True)
    ap.add_argument("--run", action="store_true")
    ap.add_argument("--agent", default="if_if")
    args = ap.parse_args()
    frozen = json.loads((ROOT / "data/eval/baseline_42.json").read_text())["cases"]
    by_prefix = {c["case_id"].split("_")[0]: c["case_id"] for c in frozen}
    if set(args.cases) - CASE_PREFIXES:
        ap.error("--cases must be selected from " + ", ".join(sorted(CASE_PREFIXES)))
    if Path(args.attempt).name != args.attempt or args.attempt in {".", ".."}:
        ap.error("--attempt must be one directory name")
    client = CarlaRemoteClient(replace(RemoteCfg.from_env(), keep_remote_runs=True, compact_artifacts=True)) if args.run else None
    results = []
    for prefix in args.cases:
        cid = by_prefix[prefix]
        out = args.output / "cases" / cid / args.attempt
        document = prepare(cid, out)
        result = {"case_id": cid, "attempt": args.attempt, "prepared": True, "run": False}
        if client:
            try:
                client.run_scenario(out / "map.xodr", out / "scenario.xosc", name=f"source{prefix}_{args.attempt}",
                                    pcla_agent=args.agent, max_seconds=75, out_dir=out / "run", scene_seed=document)
                result.update(summarize(out / "run", prefix))
                result["run"] = True
            except Exception as exc:
                result["error"] = f"{type(exc).__name__}: {exc}"
                write_json(out / "run_failure.json", result)
        results.append(result)
        print(json.dumps({k: v for k, v in result.items() if k in {
            "case_id", "run", "error", "runtime_evidence_pass", "impact_area", "video_visual_review"}}, ensure_ascii=False), flush=True)
        write_json(args.output / f"stationary_{args.attempt}.json", results)
    return int(any(r.get("error") or (args.run and not r.get("runtime_evidence_pass")) for r in results))


if __name__ == "__main__":
    raise SystemExit(main())
