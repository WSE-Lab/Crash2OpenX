#!/usr/bin/env python3
"""Batch-infer sig_v1 signatures for the full PDF corpus.

Why this exists:
  scene_seed_v2 is too heavy to run over the 700+ DMV/NHTSA reports — each
  PDF burns ~4 k tokens for description_zh / evidence / params that the
  cluster metric never reads. signature_v1 strips those out and gives us
  the 4 structural axes (sut_maneuver / npc_count / npc_sig / collision_pair)
  the cluster + saturation tools actually consume. Run this once over the
  whole inputs/dmv_reports/ tree to retire the n=100 sample.

Pipeline per case (no road / no XODR / no CARLA):
  1. inputs/dmv_reports/<case>.pdf  --extract_any-->  raw text (cached)
  2. text  --signature_call (cached!)-->  raw LLM JSON
  3. signature_normalize(...)  -->  sig_v1 dict
  4. write outputs/signature/<case>.json

Idempotent:
  - skips cases whose outputs/signature/<case>.json already exists
  - LLM cache hits via outputs/signature_cache/ even if outputs/signature/<case>.json
    is deleted, so re-running after schema-incompatible edits is cheap
  - failures land in outputs/signature/<case>.failed.json (re-run with --retry-failed)

Usage:
    uv run --python 3.12 \\
        --with openai --with pymupdf --with "markitdown[pdf]" \\
        python tools/batch_infer_signatures.py --dry-run
    uv run ... python tools/batch_infer_signatures.py --smoke 5
    uv run ... python tools/batch_infer_signatures.py --workers 5
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

from tools.api_infer_signature import (  # noqa: E402
    DEFAULT_CACHE_DIR,
    SCHEMA_VERSION,
    infer_one,
)
from tools.coordinator import load_env  # noqa: E402
from tools.extract_source import extract_any  # noqa: E402

DEFAULT_PDF_DIR = REPO_ROOT / "inputs/dmv_reports"
DEFAULT_OUT_DIR = REPO_ROOT / "outputs/signature"
DEFAULT_TEXT_CACHE = REPO_ROOT / "outputs/extracted_text"
DEFAULT_ENV = REPO_ROOT / ".env.local"


def _ok_path(case_id: str, out_dir: Path) -> Path:
    return out_dir / f"{case_id}.json"


def _failed_path(case_id: str, out_dir: Path) -> Path:
    return out_dir / f"{case_id}.failed.json"


def _list_cases(pdf_dir: Path, only: list[str] | None) -> list[str]:
    pdfs = sorted(pdf_dir.glob("*.pdf"))
    cases = [p.stem for p in pdfs]
    if only:
        s = set(only)
        cases = [c for c in cases if c in s]
    return cases


def _extract_with_cache(case_id: str, pdf: Path, cache_dir: Path,
                        *, vlm_model: str, base_url: str, api_key_env: str,
                        max_retries: int = 3) -> str:
    cache = cache_dir / f"{case_id}.txt"
    if cache.is_file():
        text = cache.read_text(encoding="utf-8")
        if text.strip():
            return text
    last_err: Exception | None = None
    for attempt in range(1, max_retries + 1):
        try:
            text = extract_any(pdf, vlm_model=vlm_model, vlm_base_url=base_url,
                               vlm_api_key_env=api_key_env)
            cache_dir.mkdir(parents=True, exist_ok=True)
            cache.write_text(text, encoding="utf-8")
            return text
        except Exception as exc:  # noqa: BLE001
            last_err = exc
            if attempt == max_retries:
                break
            time.sleep(2 ** (attempt - 1))  # 1s, 2s, 4s ...
    assert last_err is not None
    raise last_err


def _make_signature_args(args_cli: argparse.Namespace) -> SimpleNamespace:
    return SimpleNamespace(
        model=args_cli.model,
        base_url=args_cli.base_url,
        api_key_env=args_cli.api_key_env,
        temperature=0.0,
        max_tokens=args_cli.max_tokens,
        cache_dir=args_cli.cache_dir,
        no_cache=args_cli.no_cache,
    )


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


def _process_one(case_id: str, args_cli: argparse.Namespace,
                 sig_args: SimpleNamespace) -> tuple[str, str, str]:
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

    if not text.strip():
        _write_failure(failed, case_id, "extract_any", RuntimeError("empty text"))
        return (case_id, "fail", "extract: empty text")

    try:
        payload = infer_one(case_id, text, sig_args)
    except Exception as exc:  # noqa: BLE001
        _write_failure(failed, case_id, "signature_call", exc)
        return (case_id, "fail", f"infer: {type(exc).__name__}: {exc}")

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                   encoding="utf-8")
    if failed.is_file():
        failed.unlink()
    return (case_id, "ok", f"status={payload.get('status', '?')}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--pdf-dir", type=Path, default=DEFAULT_PDF_DIR)
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    ap.add_argument("--text-cache", type=Path, default=DEFAULT_TEXT_CACHE)
    ap.add_argument("--cache-dir", type=Path, default=DEFAULT_CACHE_DIR)
    ap.add_argument("--env", type=Path, default=DEFAULT_ENV)
    ap.add_argument("--model", default="deepseek/deepseek-v4-pro")
    ap.add_argument("--base-url", default="https://openrouter.ai/api/v1")
    ap.add_argument("--api-key-env", default="OPENROUTER_API_KEY")
    ap.add_argument("--vlm-model", default="xiaomi/mimo-v2.5",
                    help="used only if PDF needs VLM OCR (rare for these reports)")
    ap.add_argument("--max-tokens", type=int, default=12000,
                    help="reasoning models burn the chain in this budget; default 12000 covers outliers")
    ap.add_argument("--workers", type=int, default=3,
                    help="parallel LLM calls; keep low (2-3) — high concurrency causes proxy resets during PDF upload")
    ap.add_argument("--smoke", type=int, default=0,
                    help="only run first N cases (0 = all)")
    ap.add_argument("--only", nargs="*",
                    help="restrict to these case_ids (overrides --smoke ordering)")
    ap.add_argument("--force", action="store_true",
                    help="re-run even if outputs/signature/<case>.json already exists")
    ap.add_argument("--retry-failed", action="store_true",
                    help="retry cases that have a .failed.json on disk")
    ap.add_argument("--no-cache", action="store_true",
                    help="bypass LLM cache lookup (still writes to cache after success)")
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

    cases = _list_cases(args.pdf_dir, args.only)
    if args.smoke:
        cases = cases[:args.smoke]

    print("=== batch_infer_signatures ===")
    print(f"  PDF dir       : {args.pdf_dir}  ({len(list(args.pdf_dir.glob('*.pdf')))} PDFs total)")
    print(f"  out dir       : {args.out_dir}")
    print(f"  text cache    : {args.text_cache}")
    print(f"  LLM cache     : {args.cache_dir}")
    print(f"  schema_version: {SCHEMA_VERSION}")
    print(f"  model         : {args.model}")
    print(f"  workers       : {args.workers}")
    print(f"  smoke limit   : {args.smoke or 'OFF (all)'}")
    print(f"  --only        : {len(args.only) if args.only else 'OFF'}")
    print(f"  force         : {args.force}")
    print(f"  retry-failed  : {args.retry_failed}")
    print(f"  no-cache      : {args.no_cache}")
    print(f"  dry-run       : {args.dry_run}")

    already_ok = sum(1 for c in cases if _ok_path(c, args.out_dir).is_file())
    prior_fail = sum(1 for c in cases if _failed_path(c, args.out_dir).is_file())
    print(f"\n  to consider   : {len(cases)} cases")
    print(f"     existing ok : {already_ok}")
    print(f"     prior fail  : {prior_fail}")

    if args.dry_run:
        print("\n[dry-run] — no LLM calls issued. Sample plan:")
        for c in cases[:5]:
            kind = ("SKIP(exists)" if _ok_path(c, args.out_dir).is_file() else
                    "SKIP(prior-fail)" if (_failed_path(c, args.out_dir).is_file()
                                            and not args.retry_failed) else "RUN")
            print(f"    [{kind:>16s}] {c}")
        if len(cases) > 5:
            print(f"    ... +{len(cases)-5} more")
        return 0

    sig_args = _make_signature_args(args)

    t0 = time.time()
    results: list[tuple[str, str, str]] = []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futs = {pool.submit(_process_one, c, args, sig_args): c for c in cases}
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

    # Status histogram over successfully written signatures (read disk to avoid
    # double-counting in concurrent runs)
    status_hist: dict[str, int] = {}
    for c in cases:
        p = _ok_path(c, args.out_dir)
        if p.is_file():
            try:
                d = json.loads(p.read_text(encoding="utf-8"))
                status_hist[d.get("status", "?")] = status_hist.get(d.get("status", "?"), 0) + 1
            except (OSError, json.JSONDecodeError):
                pass
    if status_hist:
        print("  signature status histogram:")
        for k in sorted(status_hist):
            print(f"    {k:<18s}  {status_hist[k]}")

    if n_fail:
        print("  failed cases:")
        for cid, o, det in results:
            if o == "fail":
                print(f"    - {cid} :: {det}")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
