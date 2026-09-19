#!/usr/bin/env python3
"""Run frozen PDFs through coordinator.dispatch_full in new attempt directories.

No existing seeds, compiled maps or authored per-case trajectories are inputs.
CARLA map extraction and execution remain serial on the configured RPC port.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import shutil
import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.coordinator import dispatch_full, load_env
from tools.carla_client import get_carla_client
from tools.model_transport import model_resource_blocker, model_settings


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")


def reject_frozen_artifact_inputs():
    """Fail closed if the pipeline tries to read a precomputed seed or XODR."""
    forbidden = tuple(str((ROOT / p).resolve()) + os.sep
                      for p in ("data/compiled", "data/seeds"))

    def audit(event, arguments):
        if event != "open" or not isinstance(arguments[0], (str, bytes, os.PathLike)):
            return
        path = os.path.abspath(os.fsdecode(arguments[0]))
        if path.startswith(forbidden):
            raise PermissionError("Fresh PDF pipeline cannot open frozen artifacts: " + path)

    sys.addaudithook(audit)


class SerializedCarlaClient:
    """Keep each extract/run atomic across independent local batch processes."""
    def __init__(self, client, lock_path):
        self.client = client
        self.lock_path = lock_path

    def _call(self, method, *args, **kwargs):
        with self.lock_path.open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            try:
                return getattr(self.client, method)(*args, **kwargs)
            finally:
                fcntl.flock(lock, fcntl.LOCK_UN)

    def extract_roadgraph(self, *args, **kwargs):
        kwargs["force"] = True
        return self._call("extract_roadgraph", *args, **kwargs)

    def run_scenario(self, *args, **kwargs):
        return self._call("run_scenario", *args, **kwargs)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--source-root", type=Path, required=True,
                    help="Directory containing <case_id>/source/report.pdf")
    ap.add_argument("--cases", nargs="*", help="Frozen case IDs or their numeric prefixes; default all42")
    ap.add_argument("--attempt", required=True)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--generation-only", action="store_true",
                    help="Complete fresh inference/compilation/QA and explicitly defer all CARLA stages")
    ap.add_argument("--reuse-source-extraction", action="store_true",
                    help="Resume a hash-verified PDF read from this fresh batch; seeds/maps are still newly generated")
    ap.add_argument("--qa-max-retries", type=int, default=2)
    ap.add_argument("--max-seconds", type=int, default=600,
                    help="Remote run wall-clock budget, not authored simulation duration")
    args = ap.parse_args()
    if args.execute and args.generation_only:
        ap.error("--execute and --generation-only cannot be combined")
    load_env(ROOT / ".env.local")
    reject_frozen_artifact_inputs()
    frozen = json.loads((ROOT / "data/eval/baseline_42.json").read_text())["cases"]
    requested = set(args.cases or [])
    cases = [r["case_id"] for r in frozen if not requested or
             r["case_id"] in requested or r["case_id"].split("_")[0] in requested]
    resolved = {v for v in requested if any(c == v or c.split("_")[0] == v for c in cases)}
    if requested != resolved:
        ap.error("Unknown cases: " + str(requested - resolved))
    models = model_settings()
    model, vlm, base_url = models["model"], models["vlm_model"], models["base_url"]
    endpoint = os.environ.get("CARLA_REMOTE_HOST", "localhost") + ":" + os.environ.get("CARLA_REMOTE_RPC_PORT", "2000")
    lock_path = Path("/tmp") / ("crash2openx_rpc_" + hashlib.sha256(endpoint.encode()).hexdigest()[:16] + ".lock")
    code_snapshot = {str(p.relative_to(ROOT)): p.read_bytes()
                     for folder in ("tools", "schemas", "xsd")
                     for p in sorted((ROOT / folder).iterdir())
                     if p.is_file() and p.suffix in (".py", ".md", ".xsd")}
    snapshot_time = datetime.now(timezone.utc).isoformat()
    summary = []
    for cid in cases:
        source = args.source_root.resolve() / cid / "source/report.pdf"
        case_root = args.output.resolve() / "cases" / cid
        extraction_checkpoint = None
        if args.reuse_source_extraction:
            from tools.extract_source import load_extraction_checkpoint
            for candidate in sorted(case_root.glob("fresh_*/pipeline/*/source_extraction.json"), reverse=True):
                previous = json.loads((candidate.parents[2] / "framework_invocation.json").read_text())
                if (previous.get("entrypoint") != "tools.coordinator.dispatch_full"
                        or previous.get("existing_seed_or_xodr_input") is not False
                        or previous.get("source_pdf_sha256") != digest(source)):
                    continue
                try:
                    load_extraction_checkpoint(source, candidate.parent, model=vlm, base_url=base_url)
                except ValueError:
                    continue  # Incomplete VLM read must go through the extractor again.
                extraction_checkpoint = candidate.parent
                break
        run = case_root / args.attempt
        run.mkdir(parents=True, exist_ok=False)
        shutil.copy2(source, run / "source.pdf")
        code = run / "framework_code"
        code.mkdir()
        code_hashes = {}
        for relative, payload in code_snapshot.items():
            target = code / relative
            target.parent.mkdir(exist_ok=True)
            target.write_bytes(payload)
            code_hashes[relative] = hashlib.sha256(payload).hexdigest()
        write(run / "framework_invocation.json", {
            "case_id": cid, "entrypoint": "tools.coordinator.dispatch_full",
            "source_pdf_sha256": digest(source),
            "model": model, "vlm_model": vlm, "base_url": base_url,
            "api_key_env": models["api_key_env"],
            "execute": args.execute, "qa_max_retries": args.qa_max_retries,
            "generation_only": args.generation_only,
            "source_extraction_checkpoint": str(extraction_checkpoint) if extraction_checkpoint else None,
            "serialized_model_calls": os.environ.get("C2X_SERIALIZE_MODEL_CALLS") == "1",
            "case_specific_plan_used": False, "existing_seed_or_xodr_input": False,
            "frozen_artifact_read_guard": ["data/compiled", "data/seeds"],
            "framework_code_sha256": code_hashes,
            "framework_snapshot_at": snapshot_time,
            "environment": {k: os.environ.get(k) for k in (
                "CARLA_MODE", "CARLA_REMOTE_HOST", "CARLA_REMOTE_PROJECT_DIR",
                "CARLA_REMOTE_RUNS_ROOT", "CARLA_REMOTE_RUNNER", "CARLA_REMOTE_CONTAINER")},
            "started_at": datetime.now(timezone.utc).isoformat(),
        })
        # A unique name also isolates the framework's map-cache directory.
        cache_name = f"{args.output.name}_{cid.split('_')[0]}_{args.attempt}"
        pipeline_root = run / "pipeline"
        pipeline_dir = pipeline_root / cache_name

        def progress(event, payload):
            record = {"at": datetime.now(timezone.utc).isoformat(),
                      "case_id": cid, "event": event, "payload": payload}
            with (run / "progress.jsonl").open("a") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
            if event != "done":
                print(json.dumps(record, ensure_ascii=False), flush=True)
            if event in ("phase_done", "phase_error") and "round" in payload:
                archive = run / "rounds" / f"r{payload['round']}"
                archive.mkdir(parents=True, exist_ok=True)
                for filename in ("road_seed.json", "scene_seed.json", "road_generation.json", "scene_inference_attempts.json",
                                 f"{cache_name}.xodr", f"{cache_name}.xosc", f"{cache_name}.html"):
                    p = pipeline_dir / filename
                    if p.is_file():
                        shutil.copy2(p, archive / filename)

        print(json.dumps({"case_id": cid, "starting": str(run)}, ensure_ascii=False), flush=True)
        try:
            result = dispatch_full(
                source=run / "source.pdf", name=cache_name, out_root=pipeline_root,
                model=model, vlm_model=vlm, base_url=base_url, api_key_env=models["api_key_env"],
                qa_max_retries=args.qa_max_retries,
                carla_enabled=args.execute, pcla_agent="if_if",
                carla_max_seconds=args.max_seconds,
                carla_timeout=max(900, args.max_seconds + 180), progress=progress,
                carla_client=SerializedCarlaClient(get_carla_client(), lock_path),
                generation_only=args.generation_only,
                extraction_checkpoint=extraction_checkpoint,
            )
            row = {"case_id": cid, "path": str(run), "xodr_path": result.get("xodr_path"),
                   "road_status": result.get("road_status"), "scene_status": result.get("scene_status"),
                   "qa": result.get("qa", {}).get("final_verdict"),
                   "roadgraph_status": result.get("roadgraph_status"),
                   "xosc_status": result.get("xosc_status"), "carla_status": result.get("carla_status")}
            blocker = model_resource_blocker(result.get("errors", {})) or model_resource_blocker(result.get("qa", {}))
        except Exception as exc:
            row = {"case_id": cid, "path": str(run), "error": f"{type(exc).__name__}: {exc}"}
            (run / "exception.txt").write_text(traceback.format_exc())
            blocker = model_resource_blocker(exc)
        if blocker:
            row["blocked_by"] = blocker
        summary.append(row)
        write(args.output / f"batch_{args.attempt}_{'_'.join(args.cases or ['all'])}.json", summary)
        print(json.dumps(row, ensure_ascii=False), flush=True)
        if blocker:
            return 2  # A shared resource failure must not fan out across every remaining PDF.
    return int(any(r.get("error") or (r.get("qa") != "pass" if args.generation_only else r.get("xosc_status") != "ok") or
                   (args.execute and r.get("carla_status") != "ok") for r in summary))


if __name__ == "__main__":
    raise SystemExit(main())
