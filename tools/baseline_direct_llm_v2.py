#!/usr/bin/env python3
"""Baseline: prompt an LLM directly for OpenDRIVE + OpenSCENARIO, no seed models.

Answers reviewer R1: how often does a direct "NL crash report -> ASAM XML"
prompt produce schema-valid artifacts, compared with Crash2OpenX's
seed-model route (XSD-conformant by construction)?

Differences from v1 (tools/baseline_direct_llm.py):
  - Two stages instead of one JSON blob: stage A emits the XODR from the
    report; stage B emits the XOSC from the report *plus the stage-A XODR*,
    so the baseline gets the same road context our compiler gets.
  - Scoped to the frozen 42-medoid benchmark (data/eval/baseline_42.json),
    matching every other experiment in the paper.
  - Raw XML out, not JSON-wrapped: v1 lost one case to a JSON parse error,
    which is an artifact of the harness, not of the baseline's ability.
  - Three-tier grading per artifact: well-formed / XSD-valid / #violations,
    so a single trivial slip is not scored the same as a broken document.
  - Optional repair rounds: the XSD errors are fed back, mirroring the
    OCL-feedback loop Crash2OpenX gives its own inference layer. Reported
    separately (round 0 vs final) so both numbers come out of one run.

Inputs:  data/eval/baseline_direct/reports/<case>.txt (same VLM-extracted
         narrative the pipeline's inference layer consumes)
Outputs: outputs/baseline_direct_v2/<case>/{stage_a.xml,stage_b.xml,row.json,...}
         outputs/baseline_direct_v2/results.json + results.md

    uv run python tools/baseline_direct_llm_v2.py                # full run
    uv run python tools/baseline_direct_llm_v2.py --dry-run      # no API calls
    uv run python tools/baseline_direct_llm_v2.py --limit 3      # smoke test
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import xmlschema
from openai import OpenAI

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.coordinator import load_env  # noqa: E402

XODR_XSD = ROOT / "xsd" / "OpenDRIVE_1.5M.xsd"
XOSC_XSD = ROOT / "xsd" / "OpenSCENARIO.xsd"
BASELINE_42 = ROOT / "data" / "eval" / "baseline_42.json"
REPORT_DIR = ROOT / "data" / "eval" / "baseline_direct" / "reports"

# Schemas are expensive to build and safe to share read-only; build once.
_SCHEMA_CACHE: dict[str, xmlschema.XMLSchema] = {}
_SCHEMA_LOCK = threading.Lock()

_STRUCTURE_XODR = """Minimum required OpenDRIVE structure:
  <OpenDRIVE><header revMajor="1" revMinor="5" name="" version="1.00" date="" north="0" south="0" east="0" west="0"/>
    <road name="" id="1" length="N" junction="-1"><link/>
      <planView><geometry s="0" x="0" y="0" hdg="0" length="N"><line/></geometry></planView>
      <lanes><laneSection s="0">
        <left><lane id="1" type="driving" level="false"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></left>
        <center><lane id="0" type="none"/></center>
        <right><lane id="-1" type="driving" level="false"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right>
      </laneSection></lanes>
    </road>
  </OpenDRIVE>"""

_STRUCTURE_XOSC = """Minimum required OpenSCENARIO structure:
  <OpenSCENARIO>
    <FileHeader description="" author="" revMajor="1" revMinor="0" date="2024-01-01T00:00:00"/>
    <ParameterDeclarations/><CatalogLocations/>
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

Vehicle requires <BoundingBox>, <Performance>, <Axles> (FrontAxle + RearAxle) and
<Properties>. Every ScenarioObject must hold a Vehicle or a Pedestrian."""

SYSTEM_XODR = f"""You are an expert in ASAM OpenDRIVE 1.5M.

Given a natural-language traffic-crash report, emit one OpenDRIVE XML document
describing the road on which the crash happened, so that CARLA can load it as a
standalone map.

{_STRUCTURE_XODR}

Output ONLY the OpenDRIVE XML document, starting with <?xml. No markdown fences,
no JSON wrapper, no explanation."""

SYSTEM_XOSC = f"""You are an expert in ASAM OpenSCENARIO 1.0.

You are given a natural-language traffic-crash report and the OpenDRIVE map that
was generated for it. Emit one OpenSCENARIO 1.0 XML document that reproduces the
crash on that map, so that CARLA's scenario_runner can replay it.

Reference the map as <RoadNetwork><LogicFile filepath="map.xodr"/></RoadNetwork>,
and use only road ids, lane ids and s-coordinates that actually exist in the
OpenDRIVE you were given.

{_STRUCTURE_XOSC}

Output ONLY the OpenSCENARIO XML document, starting with <?xml. No markdown
fences, no JSON wrapper, no explanation."""

SYSTEM_REPAIR = """You are an expert in ASAM {standard}.

You produced the XML document below, and a schema validator rejected it. Emit a
corrected, complete {standard} XML document that fixes every reported error while
still describing the same scenario.

Output ONLY the corrected XML document, starting with <?xml. No markdown fences,
no explanation."""


from tools.model_transport import complete_chat_completion


def schema_for(xsd_path: Path) -> xmlschema.XMLSchema:
    key = str(xsd_path)
    with _SCHEMA_LOCK:
        if key not in _SCHEMA_CACHE:
            _SCHEMA_CACHE[key] = xmlschema.XMLSchema(key)
        return _SCHEMA_CACHE[key]


def strip_to_xml(raw: str) -> str:
    """Pull the XML document out of a model response."""
    raw = (raw or "").strip()
    if raw.startswith("```"):
        raw = re.sub(r"^```[a-zA-Z]*\s*", "", raw)
        raw = re.sub(r"\s*```$", "", raw).strip()
    start = raw.find("<?xml")
    if start < 0:
        for root in ("<OpenDRIVE", "<OpenSCENARIO"):
            start = raw.find(root)
            if start >= 0:
                break
    if start > 0:
        raw = raw[start:]
    # Drop any trailing prose after the closing root tag.
    for root in ("</OpenDRIVE>", "</OpenSCENARIO>"):
        end = raw.rfind(root)
        if end >= 0:
            return raw[: end + len(root)]
    return raw


ERROR_CODES = [
    (r"missing required attribute", "missing-attribute"),
    (r"doesn't match any pattern|value doesn't match", "bad-value-pattern"),
    (r"Unexpected child|not allowed here|invalid child", "unexpected-child"),
    (r"is not an element of the set|invalid value.*enumeration", "bad-enum"),
    (r"attribute .*: .*not allowed|unexpected attribute", "unknown-attribute"),
    (r"missing child element|The content of element.*is not complete", "missing-child"),
    (r"not well-formed|mismatched tag|no element found|syntax error", "not-well-formed"),
]


def classify(msg: str) -> str:
    for pattern, code in ERROR_CODES:
        if re.search(pattern, msg, re.IGNORECASE):
            return code
    return "other"


def grade(xml_text: str, xsd_path: Path) -> dict:
    """Three tiers: well-formed / XSD-valid / how badly invalid."""
    if not xml_text.strip():
        return {"well_formed": False, "xsd_valid": False, "n_violations": None,
                "error": "empty output", "error_code": "empty"}
    schema = schema_for(xsd_path)
    try:
        errors = list(schema.iter_errors(xml_text))
    except Exception as exc:  # not parseable as XML at all
        msg = f"{type(exc).__name__}: {str(exc)[:240]}"
        return {"well_formed": False, "xsd_valid": False, "n_violations": None,
                "error": msg, "error_code": classify(msg)}
    if not errors:
        return {"well_formed": True, "xsd_valid": True, "n_violations": 0,
                "error": None, "error_code": None}
    first = str(errors[0].reason or errors[0])[:240]
    return {"well_formed": True, "xsd_valid": False, "n_violations": len(errors),
            "error": first, "error_code": classify(first),
            "error_codes": sorted({classify(str(e.reason or e)) for e in errors})}


def call_llm(client: OpenAI, model: str, system: str, user: str,
             max_tokens: int, retries: int = 2) -> str:
    last = None
    for attempt in range(retries + 1):
        try:
            resp = complete_chat_completion(client,
                model=model, temperature=0.0, max_tokens=max_tokens, stream=False,
                messages=[{"role": "system", "content": system},
                          {"role": "user", "content": user}],
            )
            content = (resp.choices[0].message.content or "").strip()
            if content:
                return content
            last = RuntimeError("empty response content")
        except Exception as exc:  # transient upstream errors are common
            last = exc
        if attempt < retries:
            time.sleep(5 * (attempt + 1))
    raise RuntimeError(f"{type(last).__name__}: {str(last)[:160]}")


def run_stage(client: OpenAI, model: str, system: str, user: str, xsd: Path,
              standard: str, case_dir: Path, tag: str, repair_rounds: int,
              max_tokens: int) -> dict:
    """One artifact: initial generation plus optional schema-feedback repairs."""
    xml = strip_to_xml(call_llm(client, model, system, user, max_tokens))
    (case_dir / f"{tag}.round0.xml").write_text(xml)
    grades = [grade(xml, xsd)]

    for r in range(repair_rounds):
        if grades[-1]["xsd_valid"]:
            break
        detail = grades[-1].get("error") or "unknown validation error"
        n = grades[-1].get("n_violations")
        feedback = (f"Validator reported {n if n is not None else 'a fatal'} error(s). "
                    f"First error: {detail}\n\n--- your XML ---\n{xml}")
        xml = strip_to_xml(call_llm(
            client, model, SYSTEM_REPAIR.format(standard=standard), feedback, max_tokens))
        (case_dir / f"{tag}.round{r + 1}.xml").write_text(xml)
        grades.append(grade(xml, xsd))

    (case_dir / f"{tag}.xml").write_text(xml)
    return {"round0": grades[0], "final": grades[-1], "rounds_used": len(grades) - 1}


def process_case(case_id: str, text: str, args, out_root: Path) -> dict:
    case_dir = out_root / case_id
    case_dir.mkdir(parents=True, exist_ok=True)
    row_path = case_dir / "row.json"
    if args.resume and row_path.is_file():
        row = json.loads(row_path.read_text())
        row["skipped"] = True
        return row

    t0 = time.time()
    row: dict = {"case_id": case_id}
    try:
        if args.dry_run:
            xodr = {"round0": grade("<OpenDRIVE/>", XODR_XSD),
                    "final": grade("<OpenDRIVE/>", XODR_XSD), "rounds_used": 0}
            xosc = {"round0": grade("<OpenSCENARIO/>", XOSC_XSD),
                    "final": grade("<OpenSCENARIO/>", XOSC_XSD), "rounds_used": 0}
            (case_dir / "stage_a.xml").write_text("<OpenDRIVE/>")
        else:
            client = OpenAI(api_key=os.environ[args.api_key_env], base_url=args.base_url)
            xodr = run_stage(
                client, args.model, SYSTEM_XODR,
                f"Crash report:\n\n{text}\n\nEmit the OpenDRIVE map for this crash.",
                XODR_XSD, "OpenDRIVE 1.5M", case_dir, "stage_a",
                args.repair_rounds, args.max_tokens)

            # Stage B sees the report *and* the road produced in stage A, even
            # when that road is schema-invalid: gating stage B on stage A would
            # make the two rates non-comparable.
            road_xml = (case_dir / "stage_a.xml").read_text()
            truncated = len(road_xml) > args.max_map_chars
            if truncated:
                road_xml = road_xml[: args.max_map_chars]
            row["map_truncated"] = truncated
            xosc = run_stage(
                client, args.model, SYSTEM_XOSC,
                f"Crash report:\n\n{text}\n\n"
                f"OpenDRIVE map generated for this crash (filepath map.xodr):\n\n"
                f"{road_xml}\n\nEmit the OpenSCENARIO scenario for this crash.",
                XOSC_XSD, "OpenSCENARIO 1.0", case_dir, "stage_b",
                args.repair_rounds, args.max_tokens)
    except Exception as exc:
        row.update({"error": f"{type(exc).__name__}: {str(exc)[:200]}",
                    "duration_s": round(time.time() - t0, 1)})
        row_path.write_text(json.dumps(row, ensure_ascii=False, indent=2))
        return row

    row.update({"xodr": xodr, "xosc": xosc, "duration_s": round(time.time() - t0, 1)})
    row_path.write_text(json.dumps(row, ensure_ascii=False, indent=2))
    return row


def collect_inputs(limit: int) -> list[tuple[str, str]]:
    cases = json.loads(BASELINE_42.read_text())["cases"]
    items: list[tuple[str, str]] = []
    for entry in cases:
        case_id = entry["case_id"]
        fp = REPORT_DIR / f"{case_id}.txt"
        if not fp.is_file():
            print(f"[warn] no report text for {case_id}, skipping")
            continue
        items.append((case_id, fp.read_text(encoding="utf-8", errors="replace")))
    return items[:limit] if limit else items


def summarize(rows: list[dict], args) -> dict:
    ok = [r for r in rows if "error" not in r]
    n = len(rows)

    def count(art: str, stage: str, field: str) -> int:
        return sum(1 for r in ok if r[art][stage][field])

    def rate(k: int) -> float:
        return round(k / n, 3) if n else 0.0

    cells = {}
    for stage in ("round0", "final"):
        xodr_v, xosc_v = count("xodr", stage, "xsd_valid"), count("xosc", stage, "xsd_valid")
        both = sum(1 for r in ok if r["xodr"][stage]["xsd_valid"] and r["xosc"][stage]["xsd_valid"])
        cells[stage] = {
            "xodr_well_formed": count("xodr", stage, "well_formed"),
            "xosc_well_formed": count("xosc", stage, "well_formed"),
            "xodr_xsd_valid": xodr_v, "xodr_xsd_rate": rate(xodr_v),
            "xosc_xsd_valid": xosc_v, "xosc_xsd_rate": rate(xosc_v),
            "both_xsd_valid": both, "both_xsd_rate": rate(both),
        }

    codes: dict[str, int] = {}
    for r in ok:
        for art in ("xodr", "xosc"):
            code = r[art]["final"].get("error_code")
            if code:
                codes[f"{art}:{code}"] = codes.get(f"{art}:{code}", 0) + 1
    # Only invalid artifacts: including the valid ones (n_violations == 0) would
    # drag the median to 0 and misreport how far off the invalid files actually are.
    viol = sorted(v for r in ok for art in ("xodr", "xosc")
                  if not r[art]["final"]["xsd_valid"]
                  and (v := r[art]["final"].get("n_violations")) is not None)

    return {
        "model": args.model, "protocol": "two-stage (XODR, then XOSC given the XODR)",
        "benchmark": "frozen 42-medoid set (data/eval/baseline_42.json)",
        "repair_rounds": args.repair_rounds,
        "n": n, "n_harness_errors": n - len(ok),
        "round0": cells["round0"], "final": cells["final"],
        "failure_codes": dict(sorted(codes.items(), key=lambda kv: -kv[1])),
        "violations_median": viol[len(viol) // 2] if viol else None,
        "rows": rows,
    }


def write_markdown(summary: dict, path: Path) -> None:
    n = summary["n"]
    lines = [
        "# Direct-LLM baseline (two-stage, 42-medoid set)", "",
        f"- model: `{summary['model']}`",
        f"- protocol: {summary['protocol']}",
        f"- repair rounds: {summary['repair_rounds']} (XSD errors fed back, mirroring the OCL loop)",
        f"- n = {n}" + (f" ({summary['n_harness_errors']} harness/API errors)"
                        if summary["n_harness_errors"] else ""),
        "",
        "| Artifact | XSD-valid, no feedback | XSD-valid, after repair |",
        "|---|---|---|",
    ]
    r0, fin = summary["round0"], summary["final"]
    for label, key in (("OpenDRIVE", "xodr"), ("OpenSCENARIO", "xosc"), ("Both", "both")):
        a = key if key == "both" else key
        k0 = r0[f"{a}_xsd_valid"] if key != "both" else r0["both_xsd_valid"]
        kf = fin[f"{a}_xsd_valid"] if key != "both" else fin["both_xsd_valid"]
        lines.append(f"| {label} | {k0}/{n} ({k0 / n:.0%}) | {kf}/{n} ({kf / n:.0%}) |")
    lines += [
        "", f"- median schema violations per invalid file (final): {summary['violations_median']}",
        f"- dominant failure modes: {summary['failure_codes']}",
        "",
        "Crash2OpenX on the same 42 cases: 42/42 XSD-valid by construction",
        "(see `data/eval/execution_reference.md`).", "",
    ]
    path.write_text("\n".join(lines))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="deepseek-flash")
    ap.add_argument("--base-url", default="https://api.deepseek.com")
    ap.add_argument("--api-key-env", default="DEEPSEEK_API_KEY")
    ap.add_argument("--out-root", type=Path,
                    default=ROOT / "outputs" / "baseline_direct_v2")
    ap.add_argument("--repair-rounds", type=int, default=1,
                    help="schema-feedback repair attempts per artifact (0 = naive prompt only)")
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--max-tokens", type=int, default=16000)
    ap.add_argument("--max-map-chars", type=int, default=60000,
                    help="cap on the XODR text handed to stage B")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--resume", action="store_true",
                    help="reuse cases that already have row.json")
    ap.add_argument("--dry-run", action="store_true",
                    help="exercise the harness without calling the API")
    args = ap.parse_args()

    load_env(ROOT / ".env.local")
    if not args.dry_run and not os.environ.get(args.api_key_env):
        print(f"[error] {args.api_key_env} not set (.env.local or environment)")
        return 2

    items = collect_inputs(args.limit)
    if not items:
        print(f"[error] no report texts under {REPORT_DIR}")
        return 2
    args.out_root.mkdir(parents=True, exist_ok=True)
    print(f"=== direct-LLM baseline: n={len(items)}, model={args.model}, "
          f"repair_rounds={args.repair_rounds}, workers={args.workers} ===")

    schema_for(XODR_XSD)  # build once, before threads race for it
    schema_for(XOSC_XSD)

    rows: list[dict] = []
    done = 0
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(process_case, cid, txt, args, args.out_root): cid
                   for cid, txt in items}
        for fut in as_completed(futures):
            cid = futures[fut]
            done += 1
            try:
                row = fut.result()
            except Exception as exc:
                row = {"case_id": cid, "error": f"{type(exc).__name__}: {str(exc)[:200]}"}
            rows.append(row)
            if "error" in row:
                print(f"  [{done}/{len(items)}] {cid}: ERROR — {row['error']}")
            else:
                mark = lambda g: "OK" if g["xsd_valid"] else f"x({g['n_violations']})"  # noqa: E731
                print(f"  [{done}/{len(items)}] {cid}: "
                      f"xodr {mark(row['xodr']['round0'])}->{mark(row['xodr']['final'])}  "
                      f"xosc {mark(row['xosc']['round0'])}->{mark(row['xosc']['final'])}  "
                      f"({row.get('duration_s', '?')}s)"
                      + ("  [resumed]" if row.get("skipped") else ""))

    rows.sort(key=lambda r: r["case_id"])
    summary = summarize(rows, args)
    (args.out_root / "results.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2))
    write_markdown(summary, args.out_root / "results.md")

    n, r0, fin = summary["n"], summary["round0"], summary["final"]
    print(f"\n=== aggregate (n={n}) ===")
    print(f"  no feedback:   XODR {r0['xodr_xsd_valid']}/{n}  "
          f"XOSC {r0['xosc_xsd_valid']}/{n}  both {r0['both_xsd_valid']}/{n}")
    print(f"  after repair:  XODR {fin['xodr_xsd_valid']}/{n}  "
          f"XOSC {fin['xosc_xsd_valid']}/{n}  both {fin['both_xsd_valid']}/{n}")
    print("  Crash2OpenX:   42/42 XSD-valid by construction")
    print(f"  -> {args.out_root / 'results.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
