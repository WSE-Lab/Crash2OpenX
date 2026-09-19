#!/usr/bin/env python3
"""Batch-infer road_seed for a subset of eval cases (mirror of batch_infer_scene_eval_set).

Why this exists:
  batch_infer_scene_eval_set covers the scene leg only. After 2026-06-25's
  unsupported-case re-run we also need to re-infer the road leg for cases whose
  prior road_seed is stale (Bucket D — asymmetric lanes, schema v1 era) or whose
  source PDF was empty before the VLM upgrade (Bucket A). This script reuses the
  same VLM PDF cache + .env loading as the scene batch and writes
  outputs/road_seed/<case>.json, idempotent and parallel-safe.

Usage:
    uv run --python 3.12 \
        --with openai --with pymupdf --with "markitdown[pdf]" \
        python tools/batch_infer_road_eval_set.py --cases 018 089 021 522 685 038 593
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

from tools.api_infer_road_seed import call_model as road_call, normalize_output as road_normalize  # noqa: E402
from tools.coordinator import load_env, make_args  # noqa: E402
from tools.extract_source import extract_any  # noqa: E402

DEFAULT_EVAL_SET = REPO_ROOT / "paper/eval_set.json"
DEFAULT_PDF_DIR = REPO_ROOT / "inputs/dmv_reports"
DEFAULT_OUT_DIR = REPO_ROOT / "outputs/road_seed"
DEFAULT_TEXT_CACHE = REPO_ROOT / "outputs/extracted_text"
DEFAULT_ENV = REPO_ROOT / ".env.local"


def _ok_path(case_id: str, out_dir: Path) -> Path:
    return out_dir / f"{case_id}.json"


def _failed_path(case_id: str, out_dir: Path) -> Path:
    return out_dir / f"{case_id}.failed.json"


def _extract_with_cache(case_id: str, pdf: Path, cache_dir: Path,
                        *, vlm_model: str, base_url: str, api_key_env: str) -> str:
    cache = cache_dir / f"{case_id}.txt"
    if cache.is_file() and cache.stat().st_size > 0:
        return cache.read_text(encoding="utf-8")
    text = extract_any(pdf, vlm_model=vlm_model, vlm_base_url=base_url,
                       vlm_api_key_env=api_key_env)
    cache_dir.mkdir(parents=True, exist_ok=True)
    cache.write_text(text, encoding="utf-8")
    return text


def _resolve_cases(eval_set: Path, requested: list[str]) -> list[str]:
    """Map short tokens (e.g. '018') to full case_ids in eval_set."""
    es = json.loads(eval_set.read_text(encoding="utf-8"))
    all_ids = [c["case_id"] for c in es["cases"]]
    if not requested:
        return all_ids
    out: list[str] = []
    for tok in requested:
        if tok in all_ids:
            out.append(tok)
            continue
        match = [c for c in all_ids if c.startswith(tok + "_") or c == tok]
        if len(match) == 1:
            out.append(match[0])
        elif not match:
            raise SystemExit(f"unknown case token: {tok!r}")
        else:
            raise SystemExit(f"ambiguous case token {tok!r}: matches {match}")
    return out


def _process_one(case_id: str, args_cli: argparse.Namespace,
                 road_args: SimpleNamespace) -> tuple[str, str, str]:
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
        raw = road_call(road_args, doc)
        road_seed = road_normalize(raw, src_path, args_cli.model)
    except Exception as exc:  # noqa: BLE001
        _write_failure(failed, case_id, "road_call/normalize", exc)
        return (case_id, "fail", f"infer: {type(exc).__name__}: {exc}")

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(road_seed, ensure_ascii=False, indent=2) + "\n",
                   encoding="utf-8")
    if failed.is_file():
        failed.unlink()
    status = (road_seed or {}).get("status", "?")
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
    ap.add_argument("--cases", nargs="+", default=[],
                    help="case_id prefixes (e.g. 018) or full ids; empty = all eval_set")
    ap.add_argument("--eval-set", type=Path, default=DEFAULT_EVAL_SET)
    ap.add_argument("--pdf-dir", type=Path, default=DEFAULT_PDF_DIR)
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    ap.add_argument("--text-cache", type=Path, default=DEFAULT_TEXT_CACHE)
    ap.add_argument("--env", type=Path, default=DEFAULT_ENV)
    ap.add_argument("--model", default="deepseek-flash")
    ap.add_argument("--base-url", default="https://api.deepseek.com")
    ap.add_argument("--api-key-env", default="DEEPSEEK_API_KEY")
    ap.add_argument("--vlm-model", default="deepseek-flash")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--retry-failed", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    load_env(args.env)
    if not args.dry_run:
        import os
        if not os.environ.get(args.api_key_env):
            print(f"FATAL: env var {args.api_key_env} not set", file=sys.stderr)
            return 2

    cases = _resolve_cases(args.eval_set, args.cases)

    print(f"=== batch_infer_road_eval_set ===")
    print(f"  out dir      : {args.out_dir}")
    print(f"  cases        : {len(cases)}")
    print(f"  force        : {args.force}")
    print(f"  retry-failed : {args.retry_failed}")
    print(f"  dry-run      : {args.dry_run}")

    if args.dry_run:
        for c in cases:
            kind = "SKIP(exists)" if _ok_path(c, args.out_dir).is_file() and not args.force else "RUN"
            print(f"    [{kind:>16s}] {c}")
        return 0

    road_args = make_args(args.model, args.base_url, args.api_key_env)

    t0 = time.time()
    results: list[tuple[str, str, str]] = []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futs = {pool.submit(_process_one, c, args, road_args): c for c in cases}
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
        for cid, o, det in results:
            if o == "fail":
                print(f"    - {cid} :: {det}")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
