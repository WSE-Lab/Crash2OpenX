#!/usr/bin/env python3
"""Fidelity judging for the frozen 42-medoid set, rubric-scored and multi-judge.

Answers reviewer R3-1 ("the generated scenarios are not compared ... to ensure
they describe the same situation"): how often, and in what respect, does the
generated scenario actually match the crash report?

Rigor changes over tools/medoid_fidelity.py (n=18, single text judge):

  1. Rubric, not vibes. The judge returns a verdict per fixed element
     (topology / lanes / actors / positions / behavior / collision / environment
     / traffic control); the 1-5 score is then computed *in Python* from those
     verdicts. The judge never gets to pick the aggregate, so the number is
     auditable and reproducible from the element table.
  2. Grounded. Every element verdict must carry a verbatim quote from the report
     (or the explicit string NO_EVIDENCE). Unsupported "matched" claims become
     visible instead of blending into prose.
  3. Closed-set defect tags, so the failure profile aggregates automatically --
     this is what produces the traffic-control breakdown rather than us
     hand-reading rationales.
  4. Traffic control is scored but held out of the core score, and reported as a
     separate delta. It is a known architectural gap (the compilers drop
     scene.control), not a per-case extraction failure; folding it in would just
     subtract a near-constant and hide the per-case variance. Reporting
     score_core and score_full separately makes the cost of the limitation the
     headline number instead of a footnote.
  5. Independent judges. The v1 judge was the same model that produced the seeds.
     Pass several models to --judges; the script reports each judge separately
     plus their agreement (exact-match rate on elements, mean |dscore|).
  6. Optional VLM mode (--vlm). The judge additionally sees the rendered
     keyframes (start/trigger/pre_collision/collision/post_collision), which is
     what Step 5 of the paper actually claims the judge does. Text-only judging
     cannot tell whether the *simulation* matches the report, only whether the
     JSON does.
  7. Full report text (v1 truncated to 4000 chars; the narratives are 6-7 KB, so
     truncation could cut the crash description itself).
  8. Stratified output: signal-dependent cases vs the rest.

Inputs:  data/eval/baseline_42.json                  (frozen case list)
         data/eval/baseline_direct/reports/<c>.txt    (VLM-extracted narrative)
         data/eval/fidelity_42/{road_seed,scene_seed}/<c>.json
         outputs/medoid_runs/<c>/keyframes/*.png      (--vlm only)
Outputs: outputs/fidelity_v2/<judge>/<case>.json
         outputs/fidelity_v2/results.json + results.md

    uv run python tools/medoid_fidelity_v2.py --probe                 # check models
    uv run python tools/medoid_fidelity_v2.py --limit 2 --vlm         # smoke test
    uv run python tools/medoid_fidelity_v2.py --vlm \
        --judges deepseek/deepseek-v4-pro,openai/gpt-5.1
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import re
import statistics
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

from openai import OpenAI

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.coordinator import load_env  # noqa: E402

BASELINE_42 = ROOT / "data" / "eval" / "baseline_42.json"
REPORT_DIR = ROOT / "data" / "eval" / "baseline_direct" / "reports"
SEED_DIR = ROOT / "data" / "eval" / "fidelity_42"
RUN_DIR = ROOT / "outputs" / "medoid_runs"

# Rendered phases, in temporal order. Named frames beat a contact sheet: the
# judge can be told which frame is which moment.
KEYFRAMES = ["start.png", "trigger.png", "pre_collision.png",
             "collision.png", "post_collision.png"]

# element -> (weight class, human label). "critical" elements decide whether the
# scenario is the same crash at all; "secondary" ones are parameter fidelity.
ELEMENTS: dict[str, tuple[str, str]] = {
    "topology":    ("critical",  "road topology"),
    "actors":      ("critical",  "participant set (kinds, count, incl. secondary parties)"),
    "behavior":    ("critical",  "critical pre-crash event / behavior"),
    "collision":   ("critical",  "collision pair AND its direction (who strikes whom)"),
    "lanes":       ("secondary", "lane counts / one-way"),
    "positions":   ("secondary", "initial relative configuration"),
    "environment": ("secondary", "weather, time of day, surface"),
}
# Judged, reported, but excluded from score_core -- see docstring point 4.
CONTROL_ELEMENT = "traffic_control"

VERDICTS = ("match", "minor", "wrong", "missing")
PENALTY = {("critical", "match"): 0.0, ("critical", "minor"): 0.5,
           ("critical", "wrong"): 1.5, ("critical", "missing"): 1.5,
           ("secondary", "match"): 0.0, ("secondary", "minor"): 0.25,
           ("secondary", "wrong"): 0.5, ("secondary", "missing"): 0.5}

DEFECT_TAGS = ("traffic_control", "secondary_actor", "topology",
               "collision_direction", "maneuver", "behavior_block",
               "environment", "lane_count", "role_assignment", "other")

SIGNAL_CONTROLS = ("traffic_light", "stop_sign", "yield")

_ELEMENT_DOC = "\n".join(
    f"  - {k}: {label}" for k, (_, label) in ELEMENTS.items()
) + f"\n  - {CONTROL_ELEMENT}: traffic control governing the conflict " \
    "(signal, stop/yield sign, stop line)"

JUDGE_SYSTEM = f"""You are auditing whether a generated driving scenario describes
the SAME crash as the source report. You are an adversarial reviewer, not a
cheerleader: your default assumption is that something is missing, and you must
be argued out of it by evidence in the report.

You receive:
  - the crash report (VLM-extracted text of an NHTSA/DMV PDF; contains form
    fields and a narrative),
  - road_seed.json (road configuration) and scene_seed.json (actors, behavior),
  - and possibly rendered simulation keyframes, labelled with the moment each
    one shows.

Judge these elements independently:
{_ELEMENT_DOC}

For each element return one verdict:
  "match"   - the generated scenario reproduces what the report describes
  "minor"   - reproduced in substance; a non-decisive parameter differs
  "wrong"   - reproduced, but incorrectly (wrong value, inverted, mismatched)
  "missing" - the report specifies this and the generated scenario omits it

Rules, applied strictly:
  - EVIDENCE IS MANDATORY. Every element carries "evidence": a verbatim span
    copied from the report supporting your verdict. If the report says nothing
    about that element, the evidence must be exactly NO_EVIDENCE and the verdict
    must be "match" (nothing to contradict) -- never "missing".
  - Which real vehicle became the ego is a free choice: do NOT penalise a
    swapped ego/NPC assignment under "actors" or "collision" as long as the
    interaction is preserved. But an INVERTED collision direction (striker and
    struck exchanged relative to the report) is always "wrong" under
    "collision", regardless of role assignment.
  - A dropped participant who was involved in the crash sequence is "missing"
    under "actors", even if the two main parties are correct.
  - Defaults for values the report never quantifies (gap, lateral offset,
    trigger distance) are "match", not "minor".
  - weather/time_of_day of "unknown" when the report is silent is "match".
  - Judge traffic_control on the generated scenario as a whole: if the report
    names a signal, sign, or stop line governing the conflict and neither seed
    model encodes it, that is "missing".
  - If keyframes are provided, they are the ground truth for what was actually
    simulated. When the JSON says one thing and the frames show another, judge
    the FRAMES and say so in the rationale.

Return ONLY a JSON object, no code fence, no commentary:
{{
  "elements": {{
    "<element>": {{"verdict": "match|minor|wrong|missing",
                   "evidence": "<verbatim report span, or NO_EVIDENCE>",
                   "note": "<=20 words, what differs"}},
    ...  // all {len(ELEMENTS) + 1} elements, none omitted
  }},
  "defects": [{{"tag": "<one of: {', '.join(DEFECT_TAGS)}>",
                "detail": "<=20 words"}}],
  "holistic_score": <integer 1-5, your own overall impression>,
  "rationale": "<=40 words"
}}
"""


from tools.model_transport import complete_chat_completion


def b64_image(path: Path) -> str:
    return base64.b64encode(path.read_bytes()).decode()


def build_user_content(cid: str, report: str, road: dict, scene: dict,
                       frames: list[Path]) -> list[dict] | str:
    scene_view = {k: scene.get(k) for k in ("status", "scene", "description_zh")
                  if scene.get(k) is not None}
    text = (
        f"## Crash report ({cid})\n\n{report}\n\n---\n\n"
        f"## road_seed.json\n\n{json.dumps(road, ensure_ascii=False, indent=2)}\n\n---\n\n"
        f"## scene_seed.json\n\n{json.dumps(scene_view, ensure_ascii=False, indent=2)}\n"
    )
    if not frames:
        return text + "\nReturn the JSON object described in the system prompt."

    parts: list[dict] = [{"type": "text", "text": text}]
    parts.append({"type": "text", "text":
                  "\n## Rendered simulation keyframes, in temporal order:"})
    for fp in frames:
        moment = fp.stem.replace("_", " ")
        parts.append({"type": "text", "text": f"Frame -- {moment}:"})
        parts.append({"type": "image_url", "image_url":
                      {"url": f"data:image/png;base64,{b64_image(fp)}"}})
    parts.append({"type": "text", "text":
                  "Return the JSON object described in the system prompt."})
    return parts


def extract_json(raw: str) -> dict:
    raw = re.sub(r"<think(?:ing)?>.*?</think(?:ing)?>", "", raw or "", flags=re.DOTALL)
    raw = re.sub(r"^```(?:json)?\s*", "", raw.strip())
    raw = re.sub(r"\s*```$", "", raw).strip()
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        pass
    depth, start = 0, -1
    for i, c in enumerate(raw):
        if c == "{":
            if depth == 0:
                start = i
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0 and start >= 0:
                try:
                    return json.loads(raw[start:i + 1])
                except json.JSONDecodeError:
                    continue
    raise ValueError(f"no JSON object in response: {raw[:200]!r}")


def score_from_elements(elements: dict) -> dict:
    """The score is a function of the verdicts -- the judge does not set it."""
    core = 5.0
    for name, (weight, _) in ELEMENTS.items():
        v = (elements.get(name) or {}).get("verdict")
        if v not in VERDICTS:
            v = "missing"  # an unanswered element is not a free pass
        core -= PENALTY[(weight, v)]
    ctrl = (elements.get(CONTROL_ELEMENT) or {}).get("verdict")
    ctrl = ctrl if ctrl in VERDICTS else "missing"
    full = core - PENALTY[("critical", ctrl)]
    return {"score_core": round(max(1.0, min(5.0, core)), 2),
            "score_full": round(max(1.0, min(5.0, full)), 2),
            "control_verdict": ctrl}


def audit(elements: dict) -> list[str]:
    """Protocol violations by the judge itself -- reported, not silently fixed."""
    problems = []
    expected = set(ELEMENTS) | {CONTROL_ELEMENT}
    for name in sorted(expected - set(elements)):
        problems.append(f"missing element: {name}")
    for name, body in elements.items():
        if name not in expected:
            problems.append(f"unknown element: {name}")
            continue
        v = (body or {}).get("verdict")
        if v not in VERDICTS:
            problems.append(f"{name}: bad verdict {v!r}")
        ev = ((body or {}).get("evidence") or "").strip()
        if not ev:
            problems.append(f"{name}: no evidence field")
        elif ev == "NO_EVIDENCE" and v == "missing":
            problems.append(f"{name}: 'missing' with NO_EVIDENCE")
    return problems


def judge_case(client: OpenAI, model: str, cid: str, report: str, road: dict,
               scene: dict, frames: list[Path], max_tokens: int,
               retries: int = 2) -> dict:
    content = build_user_content(cid, report, road, scene, frames)
    last = None
    for attempt in range(retries + 1):
        try:
            resp = complete_chat_completion(client,
                model=model, temperature=0.0, max_tokens=max_tokens, stream=False,
                messages=[{"role": "system", "content": JUDGE_SYSTEM},
                          {"role": "user", "content": content}],
            )
            verdict = extract_json(resp.choices[0].message.content or "")
            elements = verdict.get("elements") or {}
            out = {"case_id": cid, "judge": model,
                   "saw_frames": bool(frames),
                   "elements": elements,
                   "defects": verdict.get("defects") or [],
                   "holistic_score": verdict.get("holistic_score"),
                   "rationale": verdict.get("rationale"),
                   "protocol_problems": audit(elements)}
            out.update(score_from_elements(elements))
            return out
        except Exception as exc:  # noqa: BLE001
            last = exc
            if attempt < retries:
                time.sleep(5 * (attempt + 1))
    return {"case_id": cid, "judge": model,
            "error": f"{type(last).__name__}: {str(last)[:200]}"}


def load_case(cid: str, use_vlm: bool) -> dict | None:
    report_p = REPORT_DIR / f"{cid}.txt"
    road_p = SEED_DIR / "road_seed" / f"{cid}.json"
    scene_p = SEED_DIR / "scene_seed" / f"{cid}.json"
    if not (report_p.is_file() and road_p.is_file() and scene_p.is_file()):
        print(f"[warn] {cid}: missing report or seed artifacts, skipping")
        return None
    frames: list[Path] = []
    if use_vlm:
        kf = RUN_DIR / cid / "keyframes"
        frames = [kf / n for n in KEYFRAMES if (kf / n).is_file()]
        if not frames:
            print(f"[warn] {cid}: no keyframes, judging text-only")
    scene = json.loads(scene_p.read_text())
    control = ((scene.get("scene") or scene).get("control") or "unknown")
    return {"case_id": cid, "report": report_p.read_text(encoding="utf-8",
                                                         errors="replace"),
            "road": json.loads(road_p.read_text()), "scene": scene,
            "frames": frames, "control": control,
            "signal_dependent": control in SIGNAL_CONTROLS}


def probe(client: OpenAI, models: list[str], use_vlm: bool) -> int:
    """Which candidate judges actually answer? Gemini has been key-blocked and
    mimo has returned empty content on this account before."""
    sample = next((RUN_DIR / c / "keyframes" / "start.png"
                   for c in os.listdir(RUN_DIR)
                   if (RUN_DIR / c / "keyframes" / "start.png").is_file()), None)
    for m in models:
        content: list[dict] | str = "Reply with exactly: OK"
        if use_vlm and sample:
            content = [{"type": "text", "text":
                        "Reply with exactly one word naming what this image shows."},
                       {"type": "image_url", "image_url":
                        {"url": f"data:image/png;base64,{b64_image(sample)}"}}]
        try:
            r = complete_chat_completion(client,
                model=m, max_tokens=64, temperature=0.0,
                messages=[{"role": "user", "content": content}])
            txt = (r.choices[0].message.content or "").strip()
            print(f"  {m:<40} {'OK' if txt else 'EMPTY RESPONSE'}  {txt[:60]!r}")
        except Exception as exc:  # noqa: BLE001
            print(f"  {m:<40} FAIL  {type(exc).__name__}: {str(exc)[:100]}")
    return 0


def agreement(by_judge: dict[str, dict[str, dict]]) -> dict:
    """Cross-judge agreement on elements and on the derived score."""
    judges = sorted(by_judge)
    if len(judges) < 2:
        return {}
    out: dict[str, dict] = {}
    for i, a in enumerate(judges):
        for b in judges[i + 1:]:
            shared = [c for c in by_judge[a]
                      if c in by_judge[b]
                      and "error" not in by_judge[a][c]
                      and "error" not in by_judge[b][c]]
            if not shared:
                continue
            hits = total = 0
            deltas = []
            for c in shared:
                ea, eb = by_judge[a][c]["elements"], by_judge[b][c]["elements"]
                for name in list(ELEMENTS) + [CONTROL_ELEMENT]:
                    va = (ea.get(name) or {}).get("verdict")
                    vb = (eb.get(name) or {}).get("verdict")
                    if va in VERDICTS and vb in VERDICTS:
                        total += 1
                        hits += va == vb
                deltas.append(abs(by_judge[a][c]["score_core"]
                                  - by_judge[b][c]["score_core"]))
            out[f"{a} vs {b}"] = {
                "n_cases": len(shared),
                "element_exact_agreement": round(hits / total, 3) if total else None,
                "mean_abs_score_delta": round(statistics.fmean(deltas), 2),
                "max_abs_score_delta": round(max(deltas), 2),
            }
    return out


def stratify(rows: list[dict], cases: dict[str, dict]) -> dict:
    ok = [r for r in rows if "error" not in r]

    def stats(subset: list[dict]) -> dict | None:
        if not subset:
            return None
        core = [r["score_core"] for r in subset]
        full = [r["score_full"] for r in subset]
        return {"n": len(subset),
                "mean_core": round(statistics.fmean(core), 2),
                "mean_full": round(statistics.fmean(full), 2),
                "median_core": statistics.median(core),
                "min_core": min(core), "max_core": max(core),
                "control_missing": sum(1 for r in subset
                                       if r["control_verdict"] in ("missing", "wrong"))}

    sig = [r for r in ok if cases[r["case_id"]]["signal_dependent"]]
    non = [r for r in ok if not cases[r["case_id"]]["signal_dependent"]]
    tags: dict[str, int] = {}
    for r in ok:
        for d in r.get("defects") or []:
            t = d.get("tag") if isinstance(d, dict) else None
            if t:
                tags[t] = tags.get(t, 0) + 1
    elem_fail: dict[str, int] = {}
    for r in ok:
        for name in list(ELEMENTS) + [CONTROL_ELEMENT]:
            v = (r["elements"].get(name) or {}).get("verdict")
            if v in ("wrong", "missing"):
                elem_fail[name] = elem_fail.get(name, 0) + 1
    holistic_gap = [abs(r["holistic_score"] - r["score_core"]) for r in ok
                    if isinstance(r.get("holistic_score"), (int, float))]
    return {
        "all": stats(ok), "signal_dependent": stats(sig), "non_signal": stats(non),
        "defect_tags": dict(sorted(tags.items(), key=lambda kv: -kv[1])),
        "element_failures": dict(sorted(elem_fail.items(), key=lambda kv: -kv[1])),
        "mean_holistic_vs_rubric_gap": (round(statistics.fmean(holistic_gap), 2)
                                        if holistic_gap else None),
        "n_protocol_problems": sum(1 for r in ok if r.get("protocol_problems")),
        "n_errors": len(rows) - len(ok),
    }


def write_markdown(summary: dict, path: Path, judges: list[str], use_vlm: bool) -> None:
    L = ["# Fidelity of generated scenarios vs. crash reports (42-medoid set)", "",
         f"- judges: {', '.join(f'`{j}`' for j in judges)}",
         "- judge input: report + road_seed + scene_seed"
         + (" + rendered keyframes (VLM)" if use_vlm else " (text only)"),
         "- score is derived in Python from per-element verdicts, not chosen by the judge",
         "- `score_core` excludes traffic control; `score_full` includes it", ""]
    for judge in judges:
        s = summary["per_judge"].get(judge)
        if not s:
            continue
        L += [f"## `{judge}`", "",
              "| Stratum | n | mean score_core | mean score_full | control missing/wrong |",
              "|---|---|---|---|---|"]
        for label, key in (("All", "all"),
                           ("Signal-dependent", "signal_dependent"),
                           ("No traffic control", "non_signal")):
            st = s.get(key)
            if st:
                L.append(f"| {label} | {st['n']} | {st['mean_core']} | "
                         f"{st['mean_full']} | {st['control_missing']}/{st['n']} |")
        L += ["", f"- element failures (wrong/missing): {s['element_failures']}",
              f"- defect tags: {s['defect_tags']}",
              f"- judge's own holistic score vs rubric score, mean gap: "
              f"{s['mean_holistic_vs_rubric_gap']}",
              f"- cases where the judge broke protocol: {s['n_protocol_problems']}"
              f"; API errors: {s['n_errors']}", ""]
    if summary.get("agreement"):
        L += ["## Cross-judge agreement", "",
              "| Pair | n | element exact agreement | mean abs score delta | max |",
              "|---|---|---|---|---|"]
        for pair, a in summary["agreement"].items():
            L.append(f"| {pair} | {a['n_cases']} | {a['element_exact_agreement']} | "
                     f"{a['mean_abs_score_delta']} | {a['max_abs_score_delta']} |")
        L.append("")
    path.write_text("\n".join(L))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--judges", default="deepseek-flash",
                    help="comma-separated judge models; use >=2 for an "
                         "independence claim (the v1 judge was the generator model)")
    ap.add_argument("--vlm", action="store_true",
                    help="also show the judge the rendered keyframes")
    ap.add_argument("--base-url", default="https://api.deepseek.com")
    ap.add_argument("--api-key-env", default="DEEPSEEK_API_KEY")
    ap.add_argument("--out-root", type=Path, default=ROOT / "outputs" / "fidelity_v2")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--max-tokens", type=int, default=3000)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--probe", action="store_true",
                    help="check that each judge model answers, then exit")
    args = ap.parse_args()

    load_env(ROOT / ".env.local")
    if not os.environ.get(args.api_key_env):
        print(f"[error] {args.api_key_env} not set")
        return 2
    client = OpenAI(api_key=os.environ[args.api_key_env], base_url=args.base_url)
    judges = [m.strip() for m in args.judges.split(",") if m.strip()]

    if args.probe:
        print(f"=== probing {len(judges)} model(s), vlm={args.vlm} ===")
        return probe(client, judges, args.vlm)

    ids = [c["case_id"] for c in json.loads(BASELINE_42.read_text())["cases"]]
    if args.limit:
        ids = ids[: args.limit]
    cases = {}
    for cid in ids:
        c = load_case(cid, args.vlm)
        if c:
            cases[cid] = c
    if not cases:
        print("[error] no judgeable cases")
        return 2
    n_sig = sum(1 for c in cases.values() if c["signal_dependent"])
    print(f"=== fidelity v2: n={len(cases)} ({n_sig} signal-dependent), "
          f"judges={judges}, vlm={args.vlm} ===")

    by_judge: dict[str, dict[str, dict]] = {}
    for judge in judges:
        jdir = args.out_root / re.sub(r"[^A-Za-z0-9._-]", "_", judge)
        jdir.mkdir(parents=True, exist_ok=True)
        rows: dict[str, dict] = {}

        def one(c: dict, _judge=judge, _jdir=jdir) -> dict:
            cached = _jdir / f"{c['case_id']}.json"
            if args.resume and cached.is_file():
                r = json.loads(cached.read_text())
                r["skipped"] = True
                return r
            r = judge_case(client, _judge, c["case_id"], c["report"], c["road"],
                           c["scene"], c["frames"], args.max_tokens)
            cached.write_text(json.dumps(r, ensure_ascii=False, indent=2))
            return r

        print(f"\n--- judge: {judge} ---")
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futs = {pool.submit(one, c): c["case_id"] for c in cases.values()}
            for i, fut in enumerate(as_completed(futs), 1):
                cid = futs[fut]
                r = fut.result()
                rows[cid] = r
                if "error" in r:
                    print(f"  [{i:>2}/{len(cases)}] {cid:<45} ERROR {r['error'][:70]}")
                else:
                    flag = "!" if r["protocol_problems"] else " "
                    print(f"  [{i:>2}/{len(cases)}] {cid:<45} "
                          f"core={r['score_core']:<4} full={r['score_full']:<4} "
                          f"ctrl={r['control_verdict']:<8}{flag}")
        by_judge[judge] = rows

    summary = {
        "benchmark": "frozen 42-medoid set (data/eval/baseline_42.json)",
        "judges": judges, "vlm": args.vlm,
        "scoring": {"elements": {k: v[0] for k, v in ELEMENTS.items()},
                    "control_element_excluded_from_core": CONTROL_ELEMENT,
                    "penalties": {f"{w}/{v}": p for (w, v), p in PENALTY.items()}},
        "per_judge": {j: stratify(list(rows.values()), cases)
                      for j, rows in by_judge.items()},
        "agreement": agreement(by_judge),
        "rows": {j: rows for j, rows in by_judge.items()},
    }
    args.out_root.mkdir(parents=True, exist_ok=True)
    (args.out_root / "results.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2))
    write_markdown(summary, args.out_root / "results.md", judges, args.vlm)

    print("\n=== summary ===")
    for judge, s in summary["per_judge"].items():
        a, sig, non = s["all"], s["signal_dependent"], s["non_signal"]
        if not a:
            continue
        print(f"  {judge}")
        print(f"    all (n={a['n']}):            core {a['mean_core']}  full {a['mean_full']}")
        if sig:
            print(f"    signal-dep (n={sig['n']}):     core {sig['mean_core']}  "
                  f"full {sig['mean_full']}  control missing {sig['control_missing']}/{sig['n']}")
        if non:
            print(f"    no control (n={non['n']}):     core {non['mean_core']}  "
                  f"full {non['mean_full']}")
    for pair, ag in summary["agreement"].items():
        print(f"  agreement {pair}: elements {ag['element_exact_agreement']}, "
              f"mean |dscore| {ag['mean_abs_score_delta']}")
    print(f"  -> {args.out_root / 'results.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
