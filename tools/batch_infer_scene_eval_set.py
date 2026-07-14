#!/usr/bin/env python3
"""Batch-infer scene_seed for every case in paper/eval_set.json.

Why this exists:
  The eval set has 50 stratified cases. road_seed/<case>.json already exists
  for all 50, but scene_seed/<case>.json is missing for all 50 (the legacy
  facts_skill/ inputs that produced the original 10 were deleted in cleanup).
  cluster_eval_set.py only clusters cases where scene_seed.status == "supported",
  so we need to regenerate the scene_seed layer end-to-end.

Pipeline per case (mirrors coordinator.py's scene leg, no road / no XODR / no CARLA):
  1. inputs/dmv_reports/<case>.pdf  --extract_any-->  raw event description (str)
  2. {"event_description": text, ...}  --scene_call-->  raw LLM JSON
  3. scene_normalize(...)  -->  scene_seed dict
  4. write outputs/scene_seed/<case>.json

Idempotent: skips cases that already have a non-failed scene_seed JSON on disk.
Failures are written to outputs/scene_seed/<case>.failed.json so reruns can
target only the broken set (--retry-failed) without re-paying for the rest.

Usage:
    uv run --python 3.12 \
        --with openai --with pymupdf --with "markitdown[pdf]" \
        python tools/batch_infer_scene_eval_set.py --dry-run
    uv run ... python tools/batch_infer_scene_eval_set.py --smoke 1
    uv run ... python tools/batch_infer_scene_eval_set.py --workers 4
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.api_infer_scene_seed_v2 import call_model as scene_call, normalize as scene_normalize  # noqa: E402
from tools.coordinator import load_env, make_args  # noqa: E402
from tools.extract_source import extract_any  # noqa: E402

DEFAULT_EVAL_SET = REPO_ROOT / "paper/eval_set.json"
DEFAULT_PDF_DIR = REPO_ROOT / "inputs/dmv_reports"
DEFAULT_OUT_DIR = REPO_ROOT / "outputs/scene_seed"
DEFAULT_TEXT_CACHE = REPO_ROOT / "outputs/extracted_text"
DEFAULT_ENV = REPO_ROOT / ".env.local"


def _ok_path(case_id: str, out_dir: Path) -> Path:
    return out_dir / f"{case_id}.json"


def _failed_path(case_id: str, out_dir: Path) -> Path:
    return out_dir / f"{case_id}.failed.json"


def _extract_with_cache(case_id: str, pdf: Path, cache_dir: Path,
                        *, vlm_model: str, base_url: str, api_key_env: str) -> str:
    cache = cache_dir / f"{case_id}.txt"
    if cache.is_file():
        return cache.read_text(encoding="utf-8")
    text = extract_any(pdf, vlm_model=vlm_model, vlm_base_url=base_url,
                       vlm_api_key_env=api_key_env)
    cache_dir.mkdir(parents=True, exist_ok=True)
    cache.write_text(text, encoding="utf-8")
    return text


def _process_one(case_id: str, args_cli: argparse.Namespace,
                 scene_args: SimpleNamespace) -> tuple[str, str, str]:
    """Returns (case_id, outcome, detail). outcome in {ok, skip, fail}."""
    out = _ok_path(case_id, args_cli.out_dir)
    failed = _failed_path(case_id, args_cli.out_dir)
    if out.is_file() and not args_cli.force:
        return (case_id, "skip", f"exists: {out.name}")
    if failed.is_file() and not args_cli.retry_failed and not args_cli.force:
        return (case_id, "skip", f"prior failure (use --retry-failed): {failed.name}")

    pdf = args_cli.pdf_dir / f"{case_id}.pdf"
    if not pdf.is_file():
        return (case_id, "fail", f"PDF missing: {pdf}")

    try:
        text = _extract_with_cache(case_id, pdf, args_cli.text_cache,
                                   vlm_model=args_cli.vlm_model,
                                   base_url=args_cli.base_url,
                                   api_key_env=args_cli.api_key_env)
    except Exception as exc:  # noqa: BLE001
        _write_failure(failed, case_id, "extract_any", exc)
        return (case_id, "fail", f"extract: {type(exc).__name__}: {exc}")

    doc = {
        "event_description": text,
        "metadata": {"source": str(pdf), "name": case_id},
    }
    src_path = Path(f"{case_id}.pdf")

    try:
        raw = scene_call(scene_args, doc)
        scene_seed = scene_normalize(raw, src_path, args_cli.model)
    except Exception as exc:  # noqa: BLE001
        _write_failure(failed, case_id, "scene_call/normalize", exc)
        return (case_id, "fail", f"infer: {type(exc).__name__}: {exc}")

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(scene_seed, ensure_ascii=False, indent=2) + "\n",
                   encoding="utf-8")
    if failed.is_file():
        failed.unlink()  # clear any stale failure record on success
    status = (scene_seed or {}).get("status", "?")
    return (case_id, "ok", f"status={status}")


def _write_failure(path: Path, case_id: str, stage: str, exc: BaseException) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "case_id": case_id,
        "stage": stage,
        "exception_type": type(exc).__name__,
        "exception_message": str(exc),
        "traceback": traceback.format_exception_only(type(exc), exc),
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--eval-set", type=Path, default=DEFAULT_EVAL_SET)
    ap.add_argument("--pdf-dir", type=Path, default=DEFAULT_PDF_DIR)
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    ap.add_argument("--text-cache", type=Path, default=DEFAULT_TEXT_CACHE)
    ap.add_argument("--env", type=Path, default=DEFAULT_ENV)
    ap.add_argument("--model", default="deepseek/deepseek-v4-pro")
    ap.add_argument("--base-url", default="https://openrouter.ai/api/v1")
    ap.add_argument("--api-key-env", default="OPENROUTER_API_KEY")
    ap.add_argument("--vlm-model", default="xiaomi/mimo-v2.5",
                    help="used only if PDF needs VLM OCR (rare for these reports)")
    ap.add_argument("--workers", type=int, default=4,
                    help="parallel LLM calls; OpenRouter handles burst fine")
    ap.add_argument("--smoke", type=int, default=0,
                    help="only run first N cases (0 = all)")
    ap.add_argument("--force", action="store_true",
                    help="re-run even if outputs/<case>.json already exists")
    ap.add_argument("--retry-failed", action="store_true",
                    help="retry cases that have a .failed.json on disk")
    ap.add_argument("--dry-run", action="store_true",
                    help="print plan + env check, do not call LLM")
    args = ap.parse_args()

    load_env(args.env)
    if not args.dry_run:
        import os
        if not os.environ.get(args.api_key_env):
            print(f"FATAL: env var {args.api_key_env} not set "
                  f"(checked .env at {args.env})", file=sys.stderr)
            return 2

    es = json.loads(args.eval_set.read_text(encoding="utf-8"))
    cases = [c["case_id"] for c in es["cases"]]
    if args.smoke:
        cases = cases[:args.smoke]

    # Status banner
    print(f"=== batch_infer_scene_eval_set ===")
    print(f"  eval_set     : {args.eval_set}  ({len(es['cases'])} total)")
    print(f"  PDF dir      : {args.pdf_dir}")
    print(f"  out dir      : {args.out_dir}")
    print(f"  model        : {args.model}")
    print(f"  workers      : {args.workers}")
    print(f"  smoke limit  : {args.smoke or 'OFF (all)'}")
    print(f"  force        : {args.force}")
    print(f"  retry-failed : {args.retry_failed}")
    print(f"  dry-run      : {args.dry_run}")

    already_ok = sum(1 for c in cases if _ok_path(c, args.out_dir).is_file())
    prior_fail = sum(1 for c in cases if _failed_path(c, args.out_dir).is_file())
    print(f"\n  to process   : {len(cases)} cases")
    print(f"     existing ok : {already_ok}")
    print(f"     prior fail  : {prior_fail}")

    if args.dry_run:
        print("\n[dry-run] — no LLM calls issued. Sample plan:")
        for c in cases[:5]:
            kind = "SKIP(exists)" if _ok_path(c, args.out_dir).is_file() else \
                   "SKIP(prior-fail)" if (_failed_path(c, args.out_dir).is_file()
                                          and not args.retry_failed) else "RUN"
            print(f"    [{kind:>16s}] {c}")
        if len(cases) > 5:
            print(f"    ... +{len(cases)-5} more")
        return 0

    scene_args = make_args(args.model, args.base_url, args.api_key_env)

    t0 = time.time()
    results: list[tuple[str, str, str]] = []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futs = {pool.submit(_process_one, c, args, scene_args): c for c in cases}
        for i, fut in enumerate(as_completed(futs), 1):
            cid, outcome, detail = fut.result()
            results.append((cid, outcome, detail))
            tag = {"ok": "✓", "skip": "·", "fail": "✗"}[outcome]
            print(f"  [{i:>3d}/{len(cases)}] {tag} {cid:<40s}  {detail}")
    elapsed = time.time() - t0

    n_ok = sum(1 for _, o, _ in results if o == "ok")
    n_skip = sum(1 for _, o, _ in results if o == "skip")
    n_fail = sum(1 for _, o, _ in results if o == "fail")
    print(f"\n=== done in {elapsed:.1f}s ===")
    print(f"  ok={n_ok}  skip={n_skip}  fail={n_fail}")
    if n_fail:
        print("  failed cases:")
        for cid, o, det in results:
            if o == "fail":
                print(f"    - {cid} :: {det}")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
