#!/usr/bin/env python3
"""Fidelity LLM scoring for the 18 medoid scenarios.

For each medoid: feed the original PDF text + the generated scene_seed/road_seed
to a judge LLM. Get a 1–5 score on whether the DSL faithfully captures the
report's accident geometry. Output:
  - paper/medoid_fidelity.json  (per-case score + rationale + element-level
                                  pass/miss flags)
  - paper/medoid_fidelity.md    (markdown summary table for paper §)

This is a separate LLM pass from the generator — same model is fine because the
judge sees the OUTPUT and reasons about consistency with source, not from
scratch.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

from openai import OpenAI

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.coordinator import load_env  # noqa: E402

JUDGE_SYSTEM = """你是事故场景 DSL 的 fidelity 评审。
输入是一份事故报告原文（VLM 抽取后的纯文本，可能包含表单字段、叙述、几何描述）+ pipeline 生成的两份 JSON（road_seed 描述道路几何、scene_seed 描述参与者与行为）。
你的任务：判断 DSL 是否如实反映报告里**真实发生的事故几何与交互**。

只输出一个 JSON 对象，键如下：
- score: 1-5 整数。
  5 = DSL 与报告高度吻合（topology、lanes、ego 角色、NPC 类型/位置/行为、碰撞对都对得上）
  4 = 主要元素都对，1-2 个次要参数偏差（如 weather 没采到、lateral 默认）
  3 = 主轮廓对，关键参数有 1 个明显偏差或漏抓（如 NPC 行为类别选错但相邻、车道数偏 1）
  2 = 多个关键元素错配（topology 错、NPC 行为大类错、参与者角色错）
  1 = 完全捕获错了（场景全错，或漏掉了核心交互）
- rationale: 一句中文说明（≤80 字）为什么打这分
- matched: 字符串列表，DSL 命中的关键元素（如 "topology=cross_intersection"、"ego turns left at stop sign"）
- missed_or_wrong: 字符串列表，报告里有但 DSL 漏掉或错配的元素（每条 ≤30 字）

注意：
- 报告里的 ADS 厂商车辆有时是 ego、有时是 NPC（pipeline 会选"驾驶决策最值得测试的那辆"），不要因为角色互换就扣分。
- 报告里的 weather/time_of_day 若未明确，DSL 填 "unknown" 或近似值不扣分。
- 默认参数（如 gap=15, lateral=3.5）若报告无量化信息，不扣分。
- 只输出 JSON object，不要 Markdown 代码块。
"""


from tools.model_transport import complete_chat_completion


def _judge_one(client: OpenAI, model: str, cid: str,
               pdf_text: str, road: dict, scene: dict) -> dict:
    # Truncate PDF text to keep prompt manageable (3-5k chars is plenty)
    if len(pdf_text) > 4000:
        pdf_text = pdf_text[:4000] + "\n[...truncated...]"

    user = f"""## 事故报告原文 ({cid})

{pdf_text}

---

## road_seed.json

{json.dumps(road, ensure_ascii=False, indent=2)}

---

## scene_seed.json (只看 scene + description_zh + evidence)

{json.dumps({k: scene.get(k) for k in ("status", "scene", "description_zh", "evidence")},
            ensure_ascii=False, indent=2)}

按系统提示输出 JSON。"""

    resp = complete_chat_completion(client,
        model=model,
        messages=[
            {"role": "system", "content": JUDGE_SYSTEM},
            {"role": "user", "content": user},
        ],
        stream=False,
        temperature=0.0,
        max_tokens=2400,
        # Shared transport supplies high reasoning and thinking, with enough
        # output budget and incomplete-response retries for native DeepSeek.
    )
    raw = (resp.choices[0].message.content or "").strip()
    # DeepSeek reasoning models occasionally emit a leading <thinking>…</thinking>
    # block, a Markdown fence, or chatter before the JSON object. Be robust:
    # strip thinking + fences, then take the LAST balanced {…} object.
    raw = re.sub(r"<think(?:ing)?>.*?</think(?:ing)?>", "", raw, flags=re.DOTALL)
    raw = re.sub(r"^```(?:json)?\s*", "", raw)
    raw = re.sub(r"\s*```$", "", raw).strip()
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        # Find the OUTERMOST {…} block — walk char-by-char counting braces.
        depth = 0
        start = -1
        for i, c in enumerate(raw):
            if c == "{":
                if depth == 0:
                    start = i
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0 and start >= 0:
                    candidate = raw[start:i + 1]
                    try:
                        return json.loads(candidate)
                    except json.JSONDecodeError:
                        continue
        raise ValueError(f"could not extract JSON from response: {raw[:200]!r}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--medoids", type=Path, default=ROOT / "paper/eval_medoids_n100.json")
    ap.add_argument("--out-json", type=Path, default=ROOT / "paper/medoid_fidelity.json")
    ap.add_argument("--out-md", type=Path, default=ROOT / "paper/medoid_fidelity.md")
    ap.add_argument("--model", default="deepseek-flash")
    ap.add_argument("--base-url", default="https://api.deepseek.com")
    ap.add_argument("--api-key-env", default="DEEPSEEK_API_KEY")
    ap.add_argument("--workers", type=int, default=4)
    args = ap.parse_args()

    load_env(ROOT / ".env.local")
    if not os.environ.get(args.api_key_env):
        print(f"FATAL: {args.api_key_env} not set", file=sys.stderr)
        return 2

    client = OpenAI(api_key=os.environ[args.api_key_env], base_url=args.base_url)
    meds = json.loads(args.medoids.read_text())

    rows: dict[str, dict] = {}
    def _job(med):
        cid = med["medoid"]
        text_p = ROOT / "outputs/extracted_text" / f"{cid}.txt"
        scene_p = ROOT / "outputs/scene_seed" / f"{cid}.json"
        road_p = ROOT / "outputs/road_seed" / f"{cid}.json"
        if not (text_p.is_file() and scene_p.is_file() and road_p.is_file()):
            return cid, {"error": "missing source artifacts"}
        try:
            verdict = _judge_one(client, args.model, cid,
                                 text_p.read_text(encoding="utf-8"),
                                 json.loads(road_p.read_text()),
                                 json.loads(scene_p.read_text()))
            return cid, verdict
        except Exception as exc:  # noqa: BLE001
            return cid, {"error": f"{type(exc).__name__}: {exc}"}

    t0 = time.time()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futs = {pool.submit(_job, m): m for m in meds["medoids"]}
        for i, fut in enumerate(as_completed(futs), 1):
            cid, v = fut.result()
            rows[cid] = v
            score = v.get("score", "?")
            print(f"  [{i:>2}/{len(meds['medoids'])}] {cid:<55}  score={score}  {v.get('rationale', v.get('error','-'))[:80]}")
    elapsed = time.time() - t0

    # Stable-order output by cluster_id
    ordered = {}
    for med in meds["medoids"]:
        cid = med["medoid"]
        ordered[cid] = {"cluster_id": med["cluster_id"], "size": med["size"],
                        **rows.get(cid, {"error": "missing"})}

    args.out_json.write_text(json.dumps(ordered, ensure_ascii=False, indent=2),
                             encoding="utf-8")
    print(f"\nwrote {args.out_json.relative_to(ROOT)}")

    # Markdown summary
    lines = ["# Medoid fidelity report (LLM judge, n=18)\n",
             f"- judge model: `{args.model}`",
             f"- elapsed: {elapsed:.1f}s",
             "",
             "| # | case_id | size | score | rationale |",
             "|---|---|---|---|---|"]
    scores = []
    for med in meds["medoids"]:
        cid = med["medoid"]
        v = ordered[cid]
        s = v.get("score")
        scores.append(s if isinstance(s, int) else None)
        rat = v.get("rationale") or v.get("error", "")
        lines.append(f"| {med['cluster_id']} | `{cid}` | {med['size']} | {s} | {rat} |")

    valid = [s for s in scores if s is not None]
    if valid:
        lines.append("")
        lines.append(f"**Distribution**: mean={sum(valid)/len(valid):.2f}, "
                     f"n5={valid.count(5)}, n4={valid.count(4)}, "
                     f"n3={valid.count(3)}, n2={valid.count(2)}, n1={valid.count(1)}")
        lines.append("")
        # Per-case missed_or_wrong
        lines.append("## Per-case mismatches")
        for med in meds["medoids"]:
            cid = med["medoid"]
            v = ordered[cid]
            miss = v.get("missed_or_wrong") or []
            if miss:
                lines.append(f"### `{cid}` (score {v.get('score','?')})")
                for m in miss:
                    lines.append(f"- {m}")
                lines.append("")
    args.out_md.write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote {args.out_md.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
