#!/usr/bin/env python3
"""Lightweight signature extractor (sig_v1) for cluster/saturation only.

Decoupled from scene_seed_v2: only emits the 4 structural axes the cluster
distance metric uses (sut_maneuver / npc_count / npc_sig / collision_pair),
plus the same {supported, skipped, needs_extension} status. No params, no
description_zh, no evidence, no environment. The vocabulary is tied to
schemas/signature_schema.md and the constants in api_infer_scene_seed_v2.

Cache key: sha256(pdf_text)[:16] + sha256(system_prompt)[:16] + model + sig_v1.
Cache lives at outputs/signature_cache/<key>.json with a sidecar _index.json
mapping case_id -> key for retrieval and selective invalidation.

Usage (single case):
    uv run --python 3.12 --with openai \\
        python tools/api_infer_signature.py \\
        --pdf-text outputs/extracted_text/021_Zoox_Inc._March_28_2025.txt \\
        --case-id 021_Zoox_Inc._March_28_2025 \\
        --out outputs/signature/021_Zoox_Inc._March_28_2025.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.api_infer_scene_seed_v2 import (  # noqa: E402
    BLOCKS_BY_KIND,
    KINDS,
    POSITIONS_BY_KIND,
    SUT_MANEUVERS,
)

SCHEMA_VERSION = "sig_v1"
DEFAULT_CACHE_DIR = REPO_ROOT / "outputs/signature_cache"


from tools.model_transport import complete_chat_completion


def system_prompt() -> str:
    """sig_v1 prompt — structural-only extraction. Keep enums in sync with
    api_infer_scene_seed_v2.py; any vocabulary edit must bump SCHEMA_VERSION."""
    return f"""你是事故场景"分类签名"抽取器(sig_v1)。从事故文本只抽取结构性 4 维,不输出参数/描述/证据。

只返回一个 JSON object,不要 Markdown / 代码块 / 解释。

三种结局(与完整 scene_seed 同语义):
- (A) status="supported": 能用现有 position+behavior 词表表达 → 输出 signature。
- (B) status="skipped": 倒车/泊车/单车冲出路缘/无第二参与者/全静止/非碰撞 → signature=null + reason。
- (C) status="needs_extension": 真双方碰撞但词表表达不了 → signature=null + reason。

SUT(ego) = 报告中"任意一辆 car、正在移动、有测试价值"的车,不必是 AV。其余参与者(含 AV)作为 NPC 相对 ego 描述。

关键规则 (容易误判,务必遵守):
1. **被撞方静止 ≠ skip**: 若报告里 AV/Vehicle1 静止、另一辆车移动并撞过来,选**移动的那辆**为 ego,
   把静止方作 NPC(用 stopped_ahead / static_hold / opposing_leg 等)。例: "前车停住、后车驶来追尾"
   应选**后车**为 ego。只有在**所有车都静止**或**唯一动作是被动被撞**时才 skip。
2. **多连环碰撞(>2 方)**: 抽出**最关键的一对双方冲突**按 supported 表达,把第三方忽略。例:
   "Tesla 变道侧擦 BMW,然后追尾静止 Zoox" → 抽 (Tesla cut_in BMW) 这对,Tesla=v_npc,
   BMW=ego(右车道直行),不要因为有第三车就判 needs_extension。**只有连两车对都抽不出**时才 ne。
3. **AV 当 NPC 时**: id 用 "av"。其他 NPC 按 v1/v2/p1/c1/s1... 编号。

允许的 sut.maneuver: {sorted(SUT_MANEUVERS)}
(ego 必须移动;overtake_oncoming = 跨对向道超车)

每个 NPC 必须有:
- kind ∈ {sorted(KINDS)}
- position ∈ POSITIONS_BY_KIND[kind]
- behavior.block ∈ BLOCKS_BY_KIND[kind]

POSITIONS_BY_KIND:
  vehicle: {sorted(POSITIONS_BY_KIND['vehicle'])}
  cyclist: {sorted(POSITIONS_BY_KIND['cyclist'])}
  pedestrian: {sorted(POSITIONS_BY_KIND['pedestrian'])}
  static: {sorted(POSITIONS_BY_KIND['static'])}

BLOCKS_BY_KIND:
  vehicle: {sorted(BLOCKS_BY_KIND['vehicle'])}
  cyclist: {sorted(BLOCKS_BY_KIND['cyclist'])}
  pedestrian: {sorted(BLOCKS_BY_KIND['pedestrian'])}
  static: {sorted(BLOCKS_BY_KIND['static'])}

NPC id 约定: 第一个 NPC 用 v1/p1/c1/s1...,第二个用 v2/p2/...,以此类推。
若报告把 AV 当作 NPC(因为 ego 是另一辆移动 car),AV 的 id 用 "av"。
collision: {{"a": <actor_id>, "b": <actor_id>}},通常一端是 "ego";另一端可能是 v1/v2/av/debris/...。

supported 输出 schema:
{{
  "status": "supported",
  "signature": {{
    "sut_maneuver": "...",
    "npc_count": 整数,
    "npc_sig": ["<position>|<block>", ...],   // 每个 NPC 贡献一个;输出已排序去重的集合
    "collision_pair": "ego↔<other>"           // 字面量含 "↔"(U+2194),非 ascii
  }}
}}

skipped/needs_extension 输出 schema:
{{
  "status": "skipped" | "needs_extension",
  "signature": null,
  "reason": "一句中文说明"
}}

不要输出 schema_version / case_id / pipeline 字段(下游补)。不要输出 params / description_zh / evidence。
"""


def user_prompt(pdf_text: str) -> str:
    return ("请把下面事故文本抽成 sig_v1 分类签名 JSON。返回 JSON only。\n\n"
            "事故文本:\n" + pdf_text)


def _sha16(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()[:16]


def _model_slug(model: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_.-]+", "_", model)


def cache_key(pdf_text: str, system_prompt_str: str, model: str) -> str:
    pdf_sha = _sha16(pdf_text.encode("utf-8"))
    pmt_sha = _sha16(system_prompt_str.encode("utf-8"))
    return f"{pdf_sha}_{pmt_sha}_{_model_slug(model)}_{SCHEMA_VERSION}"


def cache_get(cache_dir: Path, key: str) -> dict[str, Any] | None:
    p = cache_dir / f"{key}.json"
    if not p.is_file():
        return None
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def cache_put(cache_dir: Path, key: str, payload: dict[str, Any], case_id: str) -> None:
    cache_dir.mkdir(parents=True, exist_ok=True)
    (cache_dir / f"{key}.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    idx_path = cache_dir / "_index.json"
    idx: dict[str, str] = {}
    if idx_path.is_file():
        try:
            idx = json.loads(idx_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            idx = {}
    idx[case_id] = key
    idx_path.write_text(json.dumps(idx, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8")


def extract_json(text: str) -> dict[str, Any]:
    """Best-effort JSON extraction from LLM output. Tolerates code fences,
    leading/trailing prose, and stray BOMs. Raises ValueError with the raw
    text attached so the caller can persist it for diagnosis."""
    raw = text
    text = (text or "").strip().lstrip("﻿")
    if not text:
        raise ValueError(f"empty LLM response (raw={raw!r})")
    if text.startswith("```"):
        text = re.sub(r"^```[a-zA-Z]*\n?", "", text)
        text = re.sub(r"\n?```\s*$", "", text)
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        m = re.search(r"\{.*\}", text, flags=re.DOTALL)
        if m:
            try:
                return json.loads(m.group(0))
            except json.JSONDecodeError:
                pass
        snippet = text[:400] + ("..." if len(text) > 400 else "")
        raise ValueError(f"LLM output is not valid JSON; raw={snippet!r}") from None


def normalize(raw: dict[str, Any], case_id: str, *, model: str,
              prompt_sha16: str, pdf_sha16: str) -> dict[str, Any]:
    """Validate and clamp LLM output against the sig_v1 vocabulary."""
    pipeline = {
        "stage": "signature_v1",
        "model": model,
        "prompt_sha16": prompt_sha16,
        "pdf_sha16": pdf_sha16,
    }
    status = raw.get("status")
    if status not in {"supported", "skipped", "needs_extension"}:
        # malformed: treat as needs_extension so it surfaces in audit
        return {
            "schema_version": SCHEMA_VERSION,
            "case_id": case_id,
            "status": "needs_extension",
            "signature": None,
            "reason": f"LLM returned bad status={status!r}",
            "pipeline": pipeline,
        }
    if status != "supported":
        return {
            "schema_version": SCHEMA_VERSION,
            "case_id": case_id,
            "status": status,
            "signature": None,
            "reason": str(raw.get("reason") or ""),
            "pipeline": pipeline,
        }
    sig = raw.get("signature") or {}
    maneuver = sig.get("sut_maneuver")
    if maneuver not in SUT_MANEUVERS:
        return {
            "schema_version": SCHEMA_VERSION,
            "case_id": case_id,
            "status": "skipped",
            "signature": None,
            "reason": f"sut_maneuver {maneuver!r} not in {sorted(SUT_MANEUVERS)}",
            "pipeline": pipeline,
        }
    npc_sig_raw = sig.get("npc_sig") or []
    if not isinstance(npc_sig_raw, list):
        npc_sig_raw = []
    cleaned: set[str] = set()
    for item in npc_sig_raw:
        if not isinstance(item, str) or "|" not in item:
            continue
        pos, blk = item.split("|", 1)
        pos = pos.strip()
        blk = blk.strip()
        # accept the pair if it appears in ANY kind's allowed sets — we don't
        # carry kind into the signature, so the pair is valid as long as some
        # kind's (positions × blocks) cell admits it.
        ok = any(pos in POSITIONS_BY_KIND[k] and blk in BLOCKS_BY_KIND[k]
                 for k in KINDS)
        if ok:
            cleaned.add(f"{pos}|{blk}")
    npc_sig = sorted(cleaned)
    npc_count = sig.get("npc_count")
    if not isinstance(npc_count, int) or npc_count < 0:
        npc_count = len(npc_sig)
    coll = sig.get("collision_pair")
    if not isinstance(coll, str) or "↔" not in coll:
        coll = "-"
    return {
        "schema_version": SCHEMA_VERSION,
        "case_id": case_id,
        "status": "supported",
        "signature": {
            "sut_maneuver": maneuver,
            "npc_count": npc_count,
            "npc_sig": npc_sig,
            "collision_pair": coll,
        },
        "pipeline": pipeline,
    }


def call_model(args: argparse.Namespace, pdf_text: str) -> dict[str, Any]:
    from openai import OpenAI
    api_key = os.environ.get(args.api_key_env)
    if not api_key:
        raise RuntimeError(f"Missing {args.api_key_env}; export it (e.g. source .env.local) first.")
    client = OpenAI(api_key=api_key, base_url=args.base_url)
    sp = system_prompt()
    last_err: Exception | None = None
    for attempt in (1, 2):
        resp = complete_chat_completion(client,
            model=args.model,
            messages=[
                {"role": "system", "content": sp},
                {"role": "user", "content": user_prompt(pdf_text)},
            ],
            temperature=args.temperature,
            max_tokens=args.max_tokens,
            stream=False,
        )
        txt = resp.choices[0].message.content or ""
        try:
            return extract_json(txt)
        except ValueError as exc:
            last_err = exc
            if attempt == 2:
                break
    assert last_err is not None
    raise last_err


def infer_one(case_id: str, pdf_text: str, args: argparse.Namespace) -> dict[str, Any]:
    """Cached single-case inference. Returns the normalized payload."""
    sp = system_prompt()
    key = cache_key(pdf_text, sp, args.model)
    if not args.no_cache:
        hit = cache_get(args.cache_dir, key)
        if hit is not None:
            return hit
    raw = call_model(args, pdf_text)
    norm = normalize(
        raw, case_id,
        model=args.model,
        prompt_sha16=_sha16(sp.encode("utf-8")),
        pdf_sha16=_sha16(pdf_text.encode("utf-8")),
    )
    if not args.no_cache:
        cache_put(args.cache_dir, key, norm, case_id)
    return norm


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--pdf-text", type=Path, required=True,
                    help="path to outputs/extracted_text/<case>.txt")
    ap.add_argument("--case-id", required=True)
    ap.add_argument("--out", type=Path, required=True,
                    help="path to write outputs/signature/<case>.json")
    ap.add_argument("--cache-dir", type=Path, default=DEFAULT_CACHE_DIR)
    ap.add_argument("--model", default="deepseek-flash")
    ap.add_argument("--base-url", default="https://api.deepseek.com")
    ap.add_argument("--api-key-env", default="DEEPSEEK_API_KEY")
    ap.add_argument("--temperature", type=float, default=0.0)
    ap.add_argument("--max-tokens", type=int, default=12000,
                    help="reasoning models (dsv4-pro) burn 1.5-5k tokens on the chain "
                         "before emitting content; 12000 leaves headroom for outliers")
    ap.add_argument("--no-cache", action="store_true",
                    help="bypass cache (still writes to cache after success)")
    ap.add_argument("--print-prompt", action="store_true",
                    help="print system_prompt and exit")
    return ap.parse_args(argv)


def main() -> int:
    args = parse_args()
    if args.print_prompt:
        sp = system_prompt()
        print(json.dumps({
            "schema_version": SCHEMA_VERSION,
            "model": args.model,
            "system_prompt": sp,
            "system_prompt_sha16": _sha16(sp.encode("utf-8")),
            "system_prompt_chars": len(sp),
        }, ensure_ascii=False, indent=2))
        return 0
    pdf_text = args.pdf_text.read_text(encoding="utf-8")
    payload = infer_one(args.case_id, pdf_text, args)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8")
    print(f"wrote {args.out}  status={payload['status']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
