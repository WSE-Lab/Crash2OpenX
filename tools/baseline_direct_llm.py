#!/usr/bin/env python3
"""Baseline: prompt LLM directly for OpenDRIVE + OpenSCENARIO XML, no DSL.

This is the §5 baseline cell for Crash2OpenX: how often does a direct
"NL crash report → emit OpenSCENARIO XML" prompt produce something that
even passes XSD validation? Spoiler: an LLM writing 200+ lines of XML
free-hand has lots of opportunities to drop a closing tag, mis-nest
ManeuverGroup, or invent attributes the schema doesn't know.

Crash2OpenX, by contrast, is XSD-conformant by construction (the
scenariogeneration python lib enforces the structure). The §5 claim is
the GAP between the two pass rates.

Inputs: extracted from outputs/scene_seed/*.json's evidence.source_snippets.
Outputs: outputs/baseline/<case>/{response.json, baseline.xodr, baseline.xosc}
         + outputs/baseline/results.json (aggregate XSD pass rates).

    uv run python tools/baseline_direct_llm.py
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

import xmlschema
from openai import OpenAI

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

XODR_XSD = ROOT / "xsd" / "OpenDRIVE_1.5M.xsd"
XOSC_XSD = ROOT / "xsd" / "OpenSCENARIO.xsd"


SYSTEM_PROMPT = """You are an expert in OpenSCENARIO 1.0 and OpenDRIVE 1.5M.

Given a natural-language traffic-crash report, emit one OpenDRIVE XML defining
the road geometry, and one OpenSCENARIO 1.0 XML defining the dynamic scenario,
such that the pair can be loaded and replayed in CARLA's scenario_runner.

Output a single JSON object with two string keys:
  - "xodr": a complete OpenDRIVE 1.5M XML document (starts with <?xml ...?>)
  - "xosc": a complete OpenSCENARIO 1.0 XML document (starts with <?xml ...?>)

The xosc must reference the road via:
  <RoadNetwork><LogicFile filepath="map.xodr"/></RoadNetwork>

Minimum required OpenDRIVE structure:
  <OpenDRIVE><header revMajor="1" revMinor="5" .../>
    <road id="1" length="N"><link/>
      <planView><geometry s="0" x="0" y="0" hdg="0" length="N"><line/></geometry></planView>
      <lanes><laneSection s="0">
        <left><lane id="1" type="driving" level="false"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></left>
        <center><lane id="0" type="none"/></center>
        <right><lane id="-1" type="driving" level="false"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right>
      </laneSection></lanes>
    </road>
  </OpenDRIVE>

Minimum required OpenSCENARIO structure:
  <OpenSCENARIO>
    <FileHeader description="" author="" revMajor="1" revMinor="0" date="2024-01-01T00:00:00"/>
    <CatalogLocations/>
    <RoadNetwork><LogicFile filepath="map.xodr"/></RoadNetwork>
    <Entities><ScenarioObject name="hero"><Vehicle name="hero" vehicleCategory="car">...</Vehicle></ScenarioObject></Entities>
    <Storyboard>
      <Init>...</Init>
      <Story name="s1"><Act name="a1"><ManeuverGroup name="grp" maximumExecutionCount="1">
        <Actors selectTriggeringEntities="false"><EntityRef entityRef="hero"/></Actors>
      </ManeuverGroup>
      <StartTrigger><ConditionGroup><Condition name="start" delay="0" conditionEdge="rising">
        <ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan"/></ByValueCondition>
      </Condition></ConditionGroup></StartTrigger>
      </Act></Story>
      <StopTrigger/>
    </Storyboard>
  </OpenSCENARIO>

Vehicle requires <BoundingBox>, <Performance>, <Axles> (FrontAxle + RearAxle),
<Properties>. ScenarioObject must have a Vehicle or Pedestrian.

Output ONLY the JSON object. No markdown fences. No explanation.
"""


from tools.model_transport import complete_chat_completion


def call_llm(text: str, model: str, base_url: str, api_key_env: str) -> dict:
    api_key = os.environ.get(api_key_env)
    if not api_key:
        raise RuntimeError(f"Missing {api_key_env}")
    client = OpenAI(api_key=api_key, base_url=base_url)
    resp = complete_chat_completion(client,
        model=model, temperature=0.0, max_tokens=16000, stream=False,
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user",
             "content": f"Reproduce the following crash in CARLA. Emit xodr+xosc:\n\n{text}"},
        ],
    )
    raw = resp.choices[0].message.content or ""
    raw = raw.strip()
    if raw.startswith("```"):
        raw = re.sub(r"^```(?:json)?\s*", "", raw)
        raw = re.sub(r"\s*```$", "", raw)
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        s, e = raw.find("{"), raw.rfind("}")
        if s >= 0 and e > s:
            return json.loads(raw[s:e + 1])
        raise


def validate(xml_text: str, xsd_path: Path) -> tuple[bool, str | None]:
    try:
        schema = xmlschema.XMLSchema(str(xsd_path))
        schema.validate(xml_text)
        return True, None
    except Exception as exc:
        return False, f"{type(exc).__name__}: {str(exc)[:240]}"


def load_env_local() -> None:
    env = ROOT / ".env.local"
    if not env.is_file():
        return
    for line in env.read_text().splitlines():
        if "=" in line and not line.startswith("#"):
            k, _, v = line.partition("=")
            os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))


def collect_inputs() -> list[dict]:
    """Pull (case_id, text) from outputs/scene_seed/*.json's evidence.source_snippets."""
    out: list[dict] = []
    for fp in sorted((ROOT / "outputs" / "scene_seed").glob("*.json")):
        try:
            d = json.loads(fp.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        snips = (d.get("evidence") or {}).get("source_snippets") or []
        if not snips:
            continue
        text = " ".join(s.strip() for s in snips if isinstance(s, str))
        out.append({"case_id": fp.stem, "text": text})
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="deepseek-flash")
    ap.add_argument("--base-url", default="https://api.deepseek.com")
    ap.add_argument("--api-key-env", default="DEEPSEEK_API_KEY")
    ap.add_argument("--out-root", type=Path, default=ROOT / "outputs" / "baseline")
    ap.add_argument("--limit", type=int, default=0, help="cap N inputs (0 = no cap)")
    args = ap.parse_args()

    load_env_local()
    inputs = collect_inputs()
    if args.limit:
        inputs = inputs[: args.limit]
    if not inputs:
        print("[error] no inputs found in outputs/scene_seed/")
        return 2
    print(f"=== Direct-LLM baseline on n={len(inputs)} cases ===")

    args.out_root.mkdir(parents=True, exist_ok=True)
    rows: list[dict] = []
    for i, item in enumerate(inputs, 1):
        case = item["case_id"]
        case_dir = args.out_root / case
        case_dir.mkdir(parents=True, exist_ok=True)
        t0 = time.time()
        try:
            resp = call_llm(item["text"], args.model, args.base_url, args.api_key_env)
        except Exception as exc:
            row = {"case_id": case, "llm_error": f"{type(exc).__name__}: {str(exc)[:160]}",
                   "xodr_valid": False, "xosc_valid": False,
                   "xodr_error": "no llm output", "xosc_error": "no llm output",
                   "duration_s": round(time.time() - t0, 1)}
            rows.append(row); print(f"  [{i}/{len(inputs)}] {case}: LLM ERROR — {row['llm_error']}"); continue

        (case_dir / "response.json").write_text(json.dumps(resp, ensure_ascii=False, indent=2))
        xodr_str = resp.get("xodr") or ""
        xosc_str = resp.get("xosc") or ""
        (case_dir / "baseline.xodr").write_text(xodr_str)
        (case_dir / "baseline.xosc").write_text(xosc_str)

        xodr_ok, xodr_err = validate(xodr_str, XODR_XSD) if xodr_str else (False, "empty xodr")
        xosc_ok, xosc_err = validate(xosc_str, XOSC_XSD) if xosc_str else (False, "empty xosc")
        row = {"case_id": case,
               "xodr_valid": xodr_ok, "xosc_valid": xosc_ok,
               "xodr_error": xodr_err, "xosc_error": xosc_err,
               "duration_s": round(time.time() - t0, 1)}
        rows.append(row)
        marks = ("✓" if xodr_ok else "✗", "✓" if xosc_ok else "✗")
        print(f"  [{i}/{len(inputs)}] {case}: xodr {marks[0]}  xosc {marks[1]}  ({row['duration_s']}s)")

    n = len(rows)
    xodr_pass = sum(1 for r in rows if r["xodr_valid"])
    xosc_pass = sum(1 for r in rows if r["xosc_valid"])
    both_pass = sum(1 for r in rows if r["xodr_valid"] and r["xosc_valid"])
    summary = {
        "model": args.model, "n": n,
        "xodr_pass": xodr_pass, "xodr_pass_rate": round(xodr_pass / n, 3) if n else 0,
        "xosc_pass": xosc_pass, "xosc_pass_rate": round(xosc_pass / n, 3) if n else 0,
        "both_pass": both_pass, "both_pass_rate": round(both_pass / n, 3) if n else 0,
        "rows": rows,
    }
    (args.out_root / "results.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    print()
    print(f"=== Aggregate (n={n}) ===")
    print(f"  XODR XSD pass: {xodr_pass}/{n}  ({summary['xodr_pass_rate']:.1%})")
    print(f"  XOSC XSD pass: {xosc_pass}/{n}  ({summary['xosc_pass_rate']:.1%})")
    print(f"  Both pass:     {both_pass}/{n}  ({summary['both_pass_rate']:.1%})")
    print(f"  vs Crash2OpenX: 100% by construction (deterministic compile after seed extraction)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
