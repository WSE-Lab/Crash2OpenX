#!/usr/bin/env python3
"""Coordinator: dispatch a report (file or raw text) to the road + scene agents
in parallel, then compile the road_seed into OpenDRIVE + an HTML preview.

This replaces the linear ``pdf_md_to_seed.py`` with a real coordinator:
    - road and scene inference run concurrently (independent LLM calls)
    - a supported road_seed is compiled to XODR with a self-contained SVG preview
    - the current stage stops at XODR + visualization + scene_seed (no XOSC/CARLA)

    uv run --python 3.12 \
        --with openai --with xmlschema --with pymupdf --with "markitdown[pdf]" \
        --with ./scenariogeneration \
        python tools/coordinator.py --input "一段中文事故描述" --name demo

Outputs under ``<out_root>/<name>/``:
    road_seed.json, scene_seed.json, <name>.xodr, <name>.html
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.api_infer_road_seed import call_model as road_call, normalize_output as road_normalize
from tools.api_infer_scene_seed_v2 import call_model as scene_call, normalize as scene_normalize
from tools.build_road_seed_opendrive import build as build_road, read_seed
from tools.extract_source import extract_any, load_extraction_checkpoint, pdf_narrative_coverage
from tools.ocl_constraints import violations as ocl_check
from tools.qa_agent import run_qa
from tools.scene_outcome import check_scene_outcome as _shared_check_scene_outcome

DEFAULT_ENV = REPO_ROOT / ".env.local"
DEFAULT_OUT_ROOT = REPO_ROOT / "outputs/web_runs"
DEFAULT_XSD = REPO_ROOT / "xsd" / "OpenDRIVE_1.5M.xsd"
DEFAULT_QA_MAX_RETRIES = 1  # initial pass + up to 1 retry => at most 2 rounds total


def load_env(env_path: Path) -> None:
    """Minimal KEY=VALUE .env loader (no quoting magic)."""
    if not env_path.exists():
        return
    for line in env_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        key, value = key.strip(), value.strip().strip('"').strip("'")
        if key and key not in os.environ:
            os.environ[key] = value


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


PROVENANCE_SCHEMA_VERSION = "1.0"


def _sha256_first16(payload: bytes | None) -> str | None:
    if payload is None:
        return None
    return hashlib.sha256(payload).hexdigest()[:16]


def _git_revision() -> str | None:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short=12", "HEAD"],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=2,
        )
        rev = out.stdout.strip()
        return rev or None
    except (FileNotFoundError, subprocess.SubprocessError):
        return None


def _attach_provenance(
    seed: dict[str, Any],
    *,
    stage: str,
    args: SimpleNamespace,
    src_path: Path,
) -> dict[str, Any]:
    """Stamp ``seed`` with a provenance fingerprint so a downstream researcher
    can locate the exact NL→DSL pass that produced it.

    Captures (a) the LLM identity + decoding params, (b) the schema-prompt file
    sha (when used; the scene agent has a built-in system prompt and reports
    `null` here), (c) the input file sha, (d) the output sha for tamper
    detection, and (e) the repo git revision so the surrounding code is
    pin-pointed.
    """
    schema_md_sha: str | None = None
    schema_md = getattr(args, "schema_md", None)
    if isinstance(schema_md, Path) and schema_md.exists():
        schema_md_sha = _sha256_first16(schema_md.read_bytes())

    src_sha: str | None = None
    try:
        if src_path.exists() and src_path.is_file():
            src_sha = _sha256_first16(src_path.read_bytes())
    except OSError:
        pass

    # Output sha is computed BEFORE the provenance block is added (otherwise it
    # would be self-referential).
    seed_payload = json.dumps(seed, sort_keys=True, ensure_ascii=False).encode("utf-8")
    seed["provenance"] = {
        "schema_version": PROVENANCE_SCHEMA_VERSION,
        "stage": stage,
        "llm": {
            "model": getattr(args, "model", None),
            "base_url": getattr(args, "base_url", None),
            "temperature": getattr(args, "temperature", None),
            "reasoning_effort": getattr(args, "reasoning_effort", None),
            "enable_thinking": getattr(args, "enable_thinking", None),
        },
        "schema_md_sha256_16": schema_md_sha,
        "input": {
            "path": str(src_path),
            "sha256_16": src_sha,
        },
        "output_sha256_16": _sha256_first16(seed_payload),
        "timestamp_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "git_revision": _git_revision(),
    }
    return seed


def make_args(model: str, base_url: str, api_key_env: str,
              *, fix_hint: str = "") -> SimpleNamespace:
    """Shared LLM args consumed by both agents' call_model.

    The scene agent uses its own built-in system_prompt() and ignores schema_md,
    so a single namespace works for both. ``fix_hint`` is folded in by each
    agent's call_model as an extra user turn so QA-driven retries don't have to
    rebuild the prompt chain themselves.
    """
    return SimpleNamespace(
        model=model,
        base_url=base_url,
        api_key_env=api_key_env,
        schema_md=REPO_ROOT / "schemas/road_seed_schema.md",
        temperature=0.0,
        reasoning_effort="high",
        enable_thinking=True,
        fix_hint=fix_hint,
    )


def _infer_road(args: SimpleNamespace, doc: dict[str, Any], src_path: Path, model: str) -> dict[str, Any]:
    raw = road_call(args, doc)
    return road_normalize(raw, src_path, model)


def _infer_scene(args: SimpleNamespace, doc: dict[str, Any], src_path: Path, model: str) -> dict[str, Any]:
    args.normalization_attempts = []
    initial_hint = getattr(args, 'fix_hint', '') or ''
    for attempt in range(2):
        raw = scene_call(args, doc)
        record = {'attempt': attempt + 1, 'raw_output': raw}
        args.normalization_attempts.append(record)
        try:
            return scene_normalize(raw, src_path, model, source_context=doc)
        except ValueError as exc:
            record['normalization_error'] = str(exc)
            if attempt == 1:
                raise
            args.fix_hint = (initial_hint + '\n上次输出未通过词汇/结构验证：' + str(exc)
                             + '\n请根据原文重新推理，保留所有阶段；若词汇无法表达须needs_extension。'
                             + '\n上次输出：' + json.dumps(raw, ensure_ascii=False))


def _run_one_round(
    *,
    doc: dict[str, Any],
    src_path: Path,
    name: str,
    run_dir: Path,
    xsd_path: Path,
    model: str,
    base_url: str,
    api_key_env: str,
    fix_hint_road: str,
    fix_hint_scene: str,
) -> dict[str, Any]:
    """One pass: parallel road+scene inference + (if road=supported) XODR compile.

    Returns a per-round dict with road_seed / scene_seed / html_path / xodr_path /
    summary / errors. Files are overwritten in-place across rounds so the final
    on-disk artifacts match the final round.
    """
    errors: dict[str, str] = {}
    road_seed: dict[str, Any] | None = None
    scene_seed: dict[str, Any] | None = None
    # A failed retry must not leave the preceding round's compiled map looking
    # like its output. Callers may archive prior rounds before starting another.
    for filename in ("road_seed.json", "scene_seed.json", "road_generation.json", "scene_inference_attempts.json",
                     f"{name}.xodr", f"{name}.html", f"{name}.xosc"):
        (run_dir / filename).unlink(missing_ok=True)

    args_road = make_args(model, base_url, api_key_env, fix_hint=fix_hint_road)
    args_scene = make_args(model, base_url, api_key_env, fix_hint=fix_hint_scene)
    with ThreadPoolExecutor(max_workers=2) as pool:
        f_road = pool.submit(_infer_road, args_road, doc, src_path, model)
        f_scene = pool.submit(_infer_scene, args_scene, doc, src_path, model)
        try:
            road_seed = f_road.result()
        except Exception as exc:  # noqa: BLE001
            errors["road"] = f"{type(exc).__name__}: {exc}"
        try:
            scene_seed = f_scene.result()
        except Exception as exc:  # noqa: BLE001
            errors["scene"] = f"{type(exc).__name__}: {exc}"

    if getattr(args_scene, 'normalization_attempts', None):
        write_json(run_dir / 'scene_inference_attempts.json', args_scene.normalization_attempts)

    if road_seed is not None:
        _attach_provenance(road_seed, stage="road_seed", args=args_road, src_path=src_path)
        write_json(run_dir / "road_seed.json", road_seed)
    if scene_seed is not None:
        _attach_provenance(scene_seed, stage="scene_seed", args=args_scene, src_path=src_path)
        write_json(run_dir / "scene_seed.json", scene_seed)

    html_path: Path | None = None
    xodr_path: Path | None = None
    xodr_schema_valid: bool | None = None
    summary: dict[str, Any] | None = None
    road_skipped: str | None = None

    if road_seed is None:
        road_skipped = "road inference failed"
    elif road_seed.get("status") != "supported":
        road_skipped = road_seed.get("description_zh") or road_seed.get("reason") or "road unsupported"
    else:
        xodr_path_candidate = run_dir / f"{name}.xodr"
        html_path_candidate = run_dir / f"{name}.html"
        compile_started = datetime.now(timezone.utc).isoformat()
        try:
            br = build_road(read_seed(run_dir / "road_seed.json"),
                            xodr_path_candidate, html_path_candidate, xsd_path)
            xodr_path = xodr_path_candidate
            html_path = html_path_candidate
            xodr_schema_valid = br.get("xodr_schema_valid")
            summary = {k: br.get(k) for k in (
                "road_count", "junction_count", "connection_count",
                "lane_link_count", "driving_lanes_per_direction",
                "total_driving_lanes_on_approach")}
            write_json(run_dir / "road_generation.json", {
                "operation": "build_road_seed_opendrive.build",
                "started_at": compile_started,
                "completed_at": datetime.now(timezone.utc).isoformat(),
                "seed_path": str(run_dir / "road_seed.json"),
                "seed_sha256": hashlib.sha256((run_dir / "road_seed.json").read_bytes()).hexdigest(),
                "xodr_path": str(xodr_path),
                "xodr_sha256": hashlib.sha256(xodr_path.read_bytes()).hexdigest(),
                "compiler_sha256": hashlib.sha256(
                    (REPO_ROOT / "tools/build_road_seed_opendrive.py").read_bytes()
                ).hexdigest(),
                "existing_xodr_used": False,
            })
        except Exception as exc:  # noqa: BLE001 - e.g. NotImplementedError topology
            errors["compile"] = f"{type(exc).__name__}: {exc}"
            road_skipped = f"compile failed: {exc}"

    return {
        "road_seed": road_seed,
        "scene_seed": scene_seed,
        "xodr_path": str(xodr_path) if xodr_path else None,
        "html_path": str(html_path) if html_path else None,
        "xodr_schema_valid": xodr_schema_valid,
        "summary": summary,
        "road_skipped": road_skipped,
        "errors": errors,
    }


def _check_scene_outcome(scene_seed: dict | None, carla_summary: dict | None,
                         sim_feedback_path: Path) -> dict[str, Any]:
    # CARLA flags "collision_count=1" as long as anything hit anything — that's not enough
    # to call the scenario successful. A PCLA swerve that ends with hero clipping v2 from
    # the opposing lane still bumps the counter, but the rear-end narrative the scene_seed
    # asked for didn't actually happen. Diff intent (scene_seed.collision) against outcome
    # (sim_feedback collision actors + impact_area + lane-discipline criteria) here so
    # downstream filtering / training-data curation can drop these drifted runs.
    out: dict[str, Any] = {
        "aligned": None, "verdict": "skipped", "issues": [],
        "expected_collision": None, "actual_collision_actors": None,
        "lane_invasion_count": None, "wrong_lane_count": None, "impact_area": None,
    }
    expected = ((scene_seed or {}).get("scene") or {}).get("collision") or {}
    if expected.get("a") and expected.get("b"):
        # scene_seed schema uses the SUT alias 'ego'; the xosc/sim_feedback layer renames
        # it to 'hero'. Normalize before comparing or the alignment check is always false.
        def _canon(a: str) -> str:
            return "hero" if a == "ego" else a
        out["expected_collision"] = sorted([_canon(str(expected["a"])), _canon(str(expected["b"]))])

    feedback: dict[str, Any] | None = None
    if sim_feedback_path.exists():
        try:
            feedback = json.loads(sim_feedback_path.read_text())
        except (OSError, json.JSONDecodeError):
            feedback = None
    if not feedback:
        out["verdict"] = "no_feedback"
        return out

    fsum = feedback.get("summary") or {}
    actors = sorted(str(a) for a in (fsum.get("collision_actors") or []))
    out["actual_collision_actors"] = actors or None
    out["impact_area"] = (feedback.get("behavior_checks") or {}).get("impact_area_estimated")
    out["lane_invasion_count"] = (carla_summary or {}).get("lane_invasion_count")
    out["wrong_lane_count"] = next(
        (int(c.get("actual_value") or 0)
         for c in ((carla_summary or {}).get("criteria") or [])
         if c.get("name") == "WrongLaneTest"),
        None,
    )

    issues: list[str] = []
    if out["expected_collision"]:
        if not fsum.get("collision_detected"):
            issues.append("expected collision but none occurred")
        elif actors and actors != out["expected_collision"]:
            issues.append(f"collision actors {actors} != expected {out['expected_collision']}")
        if out["impact_area"] and out["impact_area"] not in ("front", "rear", "side"):
            # 'unknown' often means hero impact angle didn't match a clean rear-end geometry
            issues.append(f"impact area '{out['impact_area']}' did not match a clean rear-end geometry")
    if (out["wrong_lane_count"] or 0) > 0:
        issues.append(f"hero entered wrong lane (WrongLaneTest={out['wrong_lane_count']})")
    if (out["lane_invasion_count"] or 0) > 2:
        issues.append(f"excessive lane invasions ({out['lane_invasion_count']})")

    out["issues"] = issues
    out["aligned"] = not issues
    out["verdict"] = "aligned" if out["aligned"] else "drifted"
    return out


def dispatch(
    source: str | Path,
    name: str,
    out_root: Path = DEFAULT_OUT_ROOT,
    *,
    model: str | None = None,
    base_url: str | None = None,
    api_key_env: str | None = None,
    vlm_model: str | None = None,
    xsd_path: Path = DEFAULT_XSD,
    qa_max_retries: int = DEFAULT_QA_MAX_RETRIES,
) -> dict[str, Any]:
    """Full coordinator: VLM-extract → (parallel road+scene → compile → VLM QA) loop.

    Up to ``qa_max_retries`` extra rounds are run when QA returns verdict=fail
    AND it gave at least one actionable fix_hint. The on-disk seeds + xodr +
    html always reflect the final round; each QA round's artifacts (map png,
    scene schematic, composite, report) live in ``run_dir/qa/r{N}/`` and the
    full per-round history is on result["qa"]["rounds"].
    """
    from tools.model_transport import model_settings
    defaults = model_settings()
    model, vlm_model = model or defaults["model"], vlm_model or defaults["vlm_model"]
    base_url, api_key_env = base_url or defaults["base_url"], api_key_env or defaults["api_key_env"]
    text = extract_any(
        source,
        vlm_model=vlm_model,
        vlm_base_url=base_url,
        vlm_api_key_env=api_key_env,
    )
    src_label = str(source) if isinstance(source, (str, Path)) else name
    src_path = Path(src_label) if os.sep in str(src_label) else Path(f"{name}.txt")
    doc = {"event_description": text, "metadata": {"source": src_label, "name": name}}

    run_dir = out_root / name
    run_dir.mkdir(parents=True, exist_ok=True)

    fix_hint_road = ""
    fix_hint_scene = ""
    qa_history: list[dict[str, Any]] = []
    round_result: dict[str, Any] = {}
    qa_final: dict[str, Any] | None = None
    total_rounds = qa_max_retries + 1

    for round_idx in range(total_rounds):
        round_result = _run_one_round(
            doc=doc, src_path=src_path, name=name, run_dir=run_dir, xsd_path=xsd_path,
            model=model, base_url=base_url, api_key_env=api_key_env,
            fix_hint_road=fix_hint_road, fix_hint_scene=fix_hint_scene,
        )

        # Cannot QA without both seeds + a rendered map -> exit the loop.
        if not round_result["html_path"] or not round_result["road_seed"] or not round_result["scene_seed"]:
            qa_history.append({
                "round": round_idx, "verdict": "skipped",
                "reason": round_result["road_skipped"] or "missing inputs for QA",
                "errors": round_result["errors"],
            })
            break

        try:
            qa = run_qa(
                html_path=Path(round_result["html_path"]),
                event_text=text,
                road_seed=round_result["road_seed"],
                scene_seed=round_result["scene_seed"],
                out_dir=run_dir / "qa" / f"r{round_idx}",
                round_idx=round_idx,
                vlm_model=vlm_model, base_url=base_url, api_key_env=api_key_env,
            )
        except Exception as exc:  # noqa: BLE001 - keep legacy CLI runs inspectable
            qa = {
                "round": round_idx,
                "verdict": "error",
                "issues": [{
                    "type": "qa_runtime",
                    "severity": "high",
                    "description": f"{type(exc).__name__}: {str(exc)[:500]}",
                }],
                "fix_hint_road": "",
                "fix_hint_scene": "",
                "artifacts": {},
            }
            qa_history.append(qa)
            qa_final = qa
            break
        qa_history.append(qa)
        qa_final = qa

        if qa["verdict"] == "pass":
            break
        # No more retries allowed, or QA didn't give actionable hints -> stop.
        if round_idx >= qa_max_retries:
            break
        if not qa["fix_hint_road"] and not qa["fix_hint_scene"]:
            break
        fix_hint_road = qa["fix_hint_road"]
        fix_hint_scene = qa["fix_hint_scene"]

    result: dict[str, Any] = {
        "name": name,
        "text_chars": len(text),
        "extracted_text": text,
        "input_kind": "file" if isinstance(source, Path) else "text",
        "run_dir": str(run_dir),
        "road_status": (round_result.get("road_seed") or {}).get("status"),
        "scene_status": (round_result.get("scene_seed") or {}).get("status"),
        "road_seed": round_result.get("road_seed"),
        "scene_seed": round_result.get("scene_seed"),
        "xodr_path": round_result.get("xodr_path"),
        "html_path": round_result.get("html_path"),
        "xodr_schema_valid": round_result.get("xodr_schema_valid"),
        "summary": round_result.get("summary"),
        "errors": round_result.get("errors", {}),
        "road_skipped": round_result.get("road_skipped"),
        "qa": {
            "final_verdict": (qa_final or {}).get("verdict")
                             if qa_final else (qa_history[-1].get("verdict") if qa_history else None),
            "rounds_run": len(qa_history),
            "max_retries": qa_max_retries,
            "rounds": qa_history,
        },
    }

    write_json(run_dir / "result.json", result)
    return result


def _format_extract_failure_hint(err: str, xodr_text: str | None = None) -> str:
    """Synthesise a fix_hint_road from a CARLA extract / load failure.

    The extract phase pushes the freshly generated XODR to the remote server
    and runs CARLA's OpenDRIVE loader. When the loader segfaults, throws
    invalid-geometry, or the runner script exits non-zero, the message lands
    in our SSH stdout. We pattern-match a few well-known cases so the next
    inference round receives an actionable hint instead of a raw stack trace.
    """
    err_lower = err.lower()
    if "s <= road->getlength" in err_lower or "exception thrown: s" in err_lower:
        return (
            "CARLA 加载新生成的 XODR 时几何越界（'s <= road->GetLength()'），"
            "解析器直接 segfault。这意味着 planView 中至少一段 geometry 的"
            "累积 s 超过其声明的 road length，常见原因：相邻 spiral/arc 段的"
            "curvature 端点 (curvStart/curvEnd) 不连续、length 字段对不上几何累计长度、"
            "或 junction connector 起止点没贴在主路上。请保留原报告的道路拓扑，"
            "检查编译器生成的 geometry 长度与连接端点；不要通过把弯道改成直路、"
            "删除路口或编造 road_seed schema 之外的字段来掩盖编译问题。"
        )
    if "invalid lane connection" in err_lower or "lane_link" in err_lower:
        return (
            "CARLA 拒收 XODR 的 lane connection（'invalid lane connection'）。"
            "请检查 junction.connection.laneLink 的 from/to id 是否都存在于对应 road 的 lanes 区段，"
            "以及 contactPoint 是否与连接的 incoming/connecting road 末端方向一致。"
        )
    if "is not a valid roadid" in err_lower or "road id" in err_lower and "invalid" in err_lower:
        return "CARLA 报 XODR 引用了不存在的 roadId。检查 junction.connection 和 laneLink 的 road 引用是否都在 <road> 列表里。"
    snippet = err.strip().splitlines()[-1] if err.strip() else "(empty)"
    return (
        f"CARLA 加载 XODR 失败（远端 runner 退出非零，最后一行：'{snippet[:200]}'）。"
        "保留原报告支持的拓扑、车道方向与数量。只修正有原文依据的 seed 错误；"
        "编译器或运行环境问题应保留失败记录，不得把事故改为更简单的道路。"
    )


def _format_xosc_failure_hint(err: str) -> str:
    """Synthesise a fix_hint_scene from an XOSC compile failure."""
    err_str = str(err)
    if "BlockUnsupported" in err_str:
        return (
            f"osc_blocks 不支持当前 scene 形态：{err_str[:300]}。"
            "请保留原文的参与者、方向、动作次序和碰撞关系。只有与原文等价的块组合才可采用；"
            "若编译器尚不能表达，应保留 unsupported，不得改写事故类型以求通过。"
        )
    if "no routes in roadgraph for ego" in err_str:
        return (
            "Ego 在 XODR 中找不到 maneuver 对应的 route。"
            "请检查生成路网是否包含原文机动所需的连接；不得把原文的转弯改为直行。"
        )
    if "junction scene needs roadgraph_map_cache" in err_str:
        return (
            "scene 需要 junction roadgraph 但 cache 未就绪 — 通常是 XODR 里的 junction 在 CARLA "
            "extract 阶段没解析出来。应修复路网提取或编译，不得删除原文中的路口。"
        )
    return (
        f"XOSC 编译失败：{err_str[:400]}。"
        "只修正有原文依据的字段错误，保留参与者与完整事故动作；"
        "不能表达时明确报告限制，不得删除参与者或动作来获得通过。"
    )


def dispatch_full(
    source: str | Path,
    name: str,
    out_root: Path = DEFAULT_OUT_ROOT,
    *,
    model: str | None = None,
    base_url: str | None = None,
    api_key_env: str | None = None,
    vlm_model: str | None = None,
    xsd_path: Path = DEFAULT_XSD,
    qa_max_retries: int = DEFAULT_QA_MAX_RETRIES,
    carla_enabled: bool = True,
    pcla_agent: str = "tfv6_regnet",
    carla_max_seconds: int = 60,
    carla_timeout: float = 1800.0,
    progress=None,
    carla_client=None,
    generation_only: bool = False,
    extraction_checkpoint: Path | None = None,
) -> dict[str, Any]:
    """End-to-end orchestrator with downstream-feedback fix loop.

    Phases (each emits ``phase_start`` / ``phase_done`` / ``phase_error`` via
    the ``progress`` callback):
      1. ``input``      — extract text from PDF / pass-through
      2. ``road``       — LLM → road_seed.json
      3. ``scene``      — LLM → scene_seed.json   (parallel with road)
      4. ``xodr``       — build XODR + HTML preview
      5. ``qa``         — VLM scene-vs-map sanity check
      6. ``roadgraph``  — push XODR to CARLA, extract waypoints (validates XODR)
      7. ``xosc``       — osc_blocks → XOSC, grounded in extracted waypoints
      8. ``carla``      — run scenario remotely, pull frame_states / events / mp4

    The first six phases live inside a retry loop bounded by ``qa_max_retries``.
    Any of QA verdict=fail, ``roadgraph`` failure, or ``xosc`` failure formats
    an actionable ``fix_hint_road`` / ``fix_hint_scene`` and re-rolls inference
    on the next round. ``carla`` (the actual scenario run) lives outside the
    loop — run-time failures are usually environmental, retrying with a
    different seed rarely helps, so we surface the error and stop.

    Result dict adds (relative to ``dispatch()``):
      roadgraph_status, roadgraph_selfcheck, roadgraph_cache_dir,
      xosc_path, xosc_status, carla_status, carla_summary, carla_run_dir.
    """
    from tools import osc_blocks
    from tools.carla_client import get_carla_client, CarlaRemoteClient, CarlaRemoteError
    from tools.model_transport import model_settings
    defaults = model_settings()
    model, vlm_model = model or defaults["model"], vlm_model or defaults["vlm_model"]
    base_url, api_key_env = base_url or defaults["base_url"], api_key_env or defaults["api_key_env"]

    # Master wall-clock so every phase timing can be plotted on a single
    # 0..N seconds axis. Each entry: {phase, round, t_start, t_end, duration_s, status}.
    t_global0 = time.time()
    phase_timings: list[dict[str, Any]] = []
    phase_t_starts: dict[tuple, float] = {}  # (phase, round) -> wall-clock t at phase_start

    def _emit(event: str, payload: dict[str, Any]) -> None:
        # Auto-record timing entries whenever a phase opens/closes. Frontend
        # gets per-node duration + a Gantt-able t_start/t_end without us
        # littering ``_record(...)`` calls at every callsite below.
        phase = payload.get("phase")
        round_idx = payload.get("round", -1)
        if phase:
            if event == "phase_start":
                phase_t_starts[(phase, round_idx)] = time.time()
            elif event in ("phase_done", "phase_error"):
                t_start = phase_t_starts.pop((phase, round_idx), time.time())
                t_end = time.time()
                status = ("ok" if event == "phase_done" else "error")
                if "status" in payload and payload["status"]:
                    status = str(payload["status"])
                if "verdict" in payload and payload["verdict"]:
                    status = str(payload["verdict"])
                entry = {
                    "phase": phase,
                    "round": round_idx,
                    "t_start": round(t_start - t_global0, 3),
                    "t_end": round(t_end - t_global0, 3),
                    "duration_s": round(t_end - t_start, 3),
                    "status": status,
                }
                phase_timings.append(entry)
                # Also enrich the outgoing SSE payload so the frontend Gantt
                # gets t_start/t_end on the same clock as the server's
                # phase_timings list.
                payload.setdefault("t_start", entry["t_start"])
                payload.setdefault("t_end", entry["t_end"])
                payload.setdefault("duration_s", entry["duration_s"])
        if progress is not None:
            try:
                progress(event, payload)
            except Exception:
                pass

    # --- phase: input ---
    _emit("phase_start", {"phase": "input"})
    t = time.time()
    src_label = str(source) if isinstance(source, (str, Path)) else name
    src_path = Path(src_label) if os.sep in str(src_label) else Path(f"{name}.txt")
    extraction_record = None
    if extraction_checkpoint is not None:
        text, extraction_record = load_extraction_checkpoint(
            Path(source), extraction_checkpoint, model=vlm_model, base_url=base_url)
        extraction_record = {**extraction_record, "source": src_label,
            "resumed_from": str(extraction_checkpoint.resolve()),
            "resumed_at": datetime.now(timezone.utc).isoformat()}
    else:
        text = extract_any(source, vlm_model=vlm_model, vlm_base_url=base_url,
                           vlm_api_key_env=api_key_env)
    doc = {"event_description": text, "metadata": {"source": src_label, "name": name}}
    run_dir = out_root / name
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "source_text.txt").write_text(text, encoding="utf-8")
    write_json(run_dir / "source_extraction.json", extraction_record or {
        "source": src_label,
        "source_sha256": hashlib.sha256(src_path.read_bytes()).hexdigest() if src_path.is_file() else None,
        "text_sha256": hashlib.sha256(text.encode("utf-8")).hexdigest(),
        "pdf_narrative_coverage": pdf_narrative_coverage(src_path, text) if src_path.is_file() else None,
        "vlm_model": vlm_model,
        "base_url": base_url,
        "extracted_at": datetime.now(timezone.utc).isoformat(),
    })
    _emit("phase_done", {"phase": "input", "duration_s": round(time.time() - t, 2),
                          "text_chars": len(text)})

    # --- retry loop covers: road + scene + xodr + qa + roadgraph + xosc ---
    fix_hint_road = ""
    fix_hint_scene = ""
    qa_history: list[dict[str, Any]] = []
    round_result: dict[str, Any] = {}
    qa_final: dict[str, Any] | None = None
    total_rounds = qa_max_retries + 1

    # Per-round state that bubbles out at the end:
    roadgraph_status: str = "pending"
    roadgraph_selfcheck: dict[str, Any] | None = None
    roadgraph_cache_dir: str | None = None
    xosc_path: str | None = None
    xosc_status: str = "pending"
    modeling_status: str = "pending"
    modeling_dir: str | None = None
    scene_model_summary: dict[str, Any] | None = None
    ocl_status: str = "pending"
    rounds_taken = 0
    retry_reason: str | None = None  # set when a round is being retried

    for round_idx in range(total_rounds):
        rounds_taken = round_idx + 1
        if round_idx > 0:
            _emit("phase_retry", {"round": round_idx,
                                   "reason": retry_reason or "fix_hint provided",
                                   "fix_hint_road": fix_hint_road,
                                   "fix_hint_scene": fix_hint_scene})

        _emit("phase_start", {"phase": "road", "round": round_idx})
        _emit("phase_start", {"phase": "scene", "round": round_idx})
        t = time.time()
        round_result = _run_one_round(
            doc=doc, src_path=src_path, name=name, run_dir=run_dir, xsd_path=xsd_path,
            model=model, base_url=base_url, api_key_env=api_key_env,
            fix_hint_road=fix_hint_road, fix_hint_scene=fix_hint_scene,
        )
        elapsed = round(time.time() - t, 2)
        errs = round_result.get("errors", {})
        _emit("phase_done" if not errs.get("road") else "phase_error",
              {"phase": "road", "round": round_idx, "duration_s": elapsed,
               "status": (round_result.get("road_seed") or {}).get("status"),
               "error": errs.get("road")})
        _emit("phase_done" if not errs.get("scene") else "phase_error",
              {"phase": "scene", "round": round_idx, "duration_s": elapsed,
               "status": (round_result.get("scene_seed") or {}).get("status"),
               "error": errs.get("scene")})

        # --- phase: ocl (Table 1: I1–I9 per-model at extraction return,
        # P1–P6 on the SeedPair before compilation; violations go back to the
        # extraction step as textual hints, exactly like a QA fail) ---
        if any((round_result.get(key) or {}).get('status') in {'unsupported', 'needs_extension', 'skipped'}
               for key in ('road_seed', 'scene_seed')):
            ocl_status = 'skipped_unsupported_seed'
            qa_history.append({'round': round_idx, 'verdict': 'skipped',
                               'reason': 'source requires an unsupported seed vocabulary or is outside scope'})
            _emit('phase_done', {'phase': 'ocl', 'round': round_idx, 'status': ocl_status})
            _emit('phase_done', {'phase': 'qa', 'round': round_idx, 'verdict': 'skipped'})
            break
        if round_result.get("road_seed") and round_result.get("scene_seed"):
            _emit("phase_start", {"phase": "ocl", "round": round_idx})
            _road_m = (round_result["road_seed"] or {}).get("road") or round_result["road_seed"] or {}
            _scene_m = (round_result["scene_seed"] or {}).get("scene") or round_result["scene_seed"] or {}
            ocl_viol = ocl_check(_road_m, _scene_m)
            if ocl_viol:
                ocl_status = "violation: " + ",".join(ocl_viol)
                _emit("phase_error", {"phase": "ocl", "round": round_idx,
                                       "violations": ocl_viol})
                if round_idx < qa_max_retries:
                    hint = ("OCL constraints violated: " + ", ".join(ocl_viol)
                            + ". Revise the seed models so the road-scene pair "
                              "satisfies these invariants.")
                    fix_hint_road = hint
                    fix_hint_scene = hint
                    retry_reason = f"ocl violations: {','.join(ocl_viol)}"
                    continue
                break
            ocl_status = "pass"
            _emit("phase_done", {"phase": "ocl", "round": round_idx,
                                  "violations": []})

        if round_result.get("xodr_path"):
            _emit("phase_done", {"phase": "xodr", "round": round_idx,
                                  "duration_s": elapsed,
                                  "schema_valid": round_result.get("xodr_schema_valid"),
                                  "summary": round_result.get("summary")})
        else:
            _emit("phase_error", {"phase": "xodr", "round": round_idx,
                                   "error": round_result.get("road_skipped") or errs.get("compile")})

        # --- phase: modeling (scene_seed -> typed Scenario + UML, platform-independent) ---
        # Promotes the synthesis output into the explicit metamodel (M1 instance)
        # and renders class/instance/activity diagrams. Runs whenever scene_seed
        # exists -- decoupled from QA so the modeling artifacts always reflect
        # the latest scene_seed even on failed rounds.
        if round_result.get("scene_seed"):
            _emit("phase_start", {"phase": "modeling", "round": round_idx})
            t_m = time.time()
            try:
                from tools import scene_modeling
                sc_m, m_artifacts = scene_modeling.render_all(
                    round_result["scene_seed"], name=name)
                _modeling_dir = run_dir / "modeling"
                _modeling_dir.mkdir(parents=True, exist_ok=True)
                for fname, text in m_artifacts.items():
                    (_modeling_dir / fname).write_text(text, encoding="utf-8")
                modeling_dir = str(_modeling_dir)
                modeling_status = "ok"
                scene_model_summary = {
                    "scenario_type": sc_m.type,
                    "sut_maneuver": sc_m.sut_maneuver,
                    "control": sc_m.control,
                    "actor_count": len(sc_m.actors),
                    "relation_count": len(sc_m.relations),
                    "phase_count": len(sc_m.phases),
                    "relation_types": sorted({r.type for r in sc_m.relations}),
                }
                _emit("phase_done", {
                    "phase": "modeling", "round": round_idx,
                    "duration_s": round(time.time() - t_m, 2),
                    **scene_model_summary,
                    "modeling_dir": modeling_dir,
                })
            except Exception as exc:  # noqa: BLE001
                modeling_status = f"error: {type(exc).__name__}: {exc}"
                _emit("phase_error", {"phase": "modeling", "round": round_idx,
                                       "error": str(exc)[:500]})
        else:
            if modeling_status == "pending":
                modeling_status = "skipped_no_scene"
            _emit("phase_done", {"phase": "modeling", "round": round_idx,
                                  "status": "skipped_no_scene"})

        if (not round_result["html_path"] or not round_result["road_seed"]
                or not round_result["scene_seed"]):
            qa_history.append({
                "round": round_idx, "verdict": "skipped",
                "reason": round_result["road_skipped"] or "missing inputs for QA",
                "errors": round_result["errors"],
            })
            _emit("phase_done", {"phase": "qa", "round": round_idx, "verdict": "skipped"})
            break  # nothing downstream can help

        _emit("phase_start", {"phase": "qa", "round": round_idx})
        t = time.time()
        try:
            qa = run_qa(
                html_path=Path(round_result["html_path"]),
                event_text=text,
                road_seed=round_result["road_seed"],
                scene_seed=round_result["scene_seed"],
                out_dir=run_dir / "qa" / f"r{round_idx}",
                round_idx=round_idx,
                vlm_model=vlm_model, base_url=base_url, api_key_env=api_key_env,
            )
        except Exception as exc:  # noqa: BLE001 - persist a diagnosable failed run
            qa = {
                "round": round_idx,
                "verdict": "error",
                "issues": [{
                    "type": "qa_runtime",
                    "severity": "high",
                    "description": f"{type(exc).__name__}: {str(exc)[:500]}",
                }],
                "fix_hint_road": "",
                "fix_hint_scene": "",
                "artifacts": {},
            }
            qa_history.append(qa)
            qa_final = qa
            _emit("phase_error", {"phase": "qa", "round": round_idx,
                                   "error": qa["issues"][0]["description"]})
            break
        qa_history.append(qa)
        qa_final = qa
        _emit("phase_done", {"phase": "qa", "round": round_idx,
                              "verdict": qa["verdict"],
                              "duration_s": round(time.time() - t, 2)})

        if qa["verdict"] != "pass":
            # QA fail → use QA's own fix_hints; retry if budget left.
            if round_idx >= qa_max_retries:
                break
            if not qa.get("fix_hint_road") and not qa.get("fix_hint_scene"):
                break
            fix_hint_road = qa.get("fix_hint_road", "")
            fix_hint_scene = qa.get("fix_hint_scene", "")
            retry_reason = f"qa verdict={qa['verdict']}"
            continue

        if generation_only:
            roadgraph_status = "deferred"
            xosc_status = "deferred"
            break

        # --- phase: roadgraph (CARLA extract — validates XODR) ---
        _emit("phase_start", {"phase": "roadgraph", "round": round_idx})
        t = time.time()
        try:
            if carla_client is None:
                carla_client = get_carla_client()
            extract_res = carla_client.extract_roadgraph(
                Path(round_result["xodr_path"]), name=name,
            )
            roadgraph_status = "ok"
            roadgraph_selfcheck = extract_res.selfcheck
            roadgraph_cache_dir = str(extract_res.local_dir)
            _emit("phase_done", {"phase": "roadgraph", "round": round_idx,
                                  "duration_s": round(time.time() - t, 2),
                                  "cached": extract_res.cached,
                                  "continuity_pass": (extract_res.selfcheck or {}).get("continuity_pass"),
                                  "waypoint_count": (extract_res.selfcheck or {}).get("waypoint_count"),
                                  "junction_count": (extract_res.selfcheck or {}).get("junction_count")})
        except CarlaRemoteError as exc:
            err_str = str(exc)
            roadgraph_status = f"error: {err_str[:240]}"
            _emit("phase_error", {"phase": "roadgraph", "round": round_idx,
                                   "error": err_str[:500]})
            if round_idx < qa_max_retries:
                fix_hint_road = _format_extract_failure_hint(err_str)
                fix_hint_scene = ""
                retry_reason = "carla_extract rejected the XODR"
                continue
            else:
                break

        # --- phase: xosc ---
        _emit("phase_start", {"phase": "xosc", "round": round_idx})
        t = time.time()
        xosc_path_candidate = run_dir / f"{name}.xosc"
        try:
            scene_payload = round_result["scene_seed"].get("scene", round_result["scene_seed"])
            # Pass road_seed so osc_blocks._check_wf can enforce WF6/WF7 at compile.
            road_seed_payload = round_result.get("road_seed")
            # auto_extract=False because roadgraph phase above already populated
            # outputs/map_cache/<name>/. If for some reason it's still missing we
            # want a hard error, not a silent second extract.
            osc_blocks.build_xosc(scene_payload, round_result["xodr_path"], xosc_path_candidate,
                                  name=name, auto_extract=False,
                                  road_seed=road_seed_payload)
            xosc_path = str(xosc_path_candidate)
            xosc_status = "ok"
            _emit("phase_done", {"phase": "xosc", "round": round_idx,
                                  "duration_s": round(time.time() - t, 2),
                                  "xosc_path": xosc_path})
        except Exception as exc:  # noqa: BLE001 - includes BlockUnsupported
            err_str = f"{type(exc).__name__}: {exc}"
            xosc_status = f"error: {err_str[:240]}"
            _emit("phase_error", {"phase": "xosc", "round": round_idx,
                                   "error": err_str[:500]})
            if round_idx < qa_max_retries:
                fix_hint_scene = _format_xosc_failure_hint(err_str)
                fix_hint_road = ""
                retry_reason = "xosc compile failed"
                continue
            else:
                break

        # All in-loop phases passed for this round.
        break

    # --- assemble base result ---
    qa_verdict = (qa_final or {}).get("verdict") if qa_final else (qa_history[-1].get("verdict") if qa_history else None)
    result: dict[str, Any] = {
        "name": name,
        "text_chars": len(text),
        "extracted_text": text,
        "input_kind": "file" if isinstance(source, Path) else "text",
        "run_dir": str(run_dir),
        "road_status": (round_result.get("road_seed") or {}).get("status"),
        "scene_status": (round_result.get("scene_seed") or {}).get("status"),
        "road_seed": round_result.get("road_seed"),
        "scene_seed": round_result.get("scene_seed"),
        "xodr_path": round_result.get("xodr_path"),
        "html_path": round_result.get("html_path"),
        "xodr_schema_valid": round_result.get("xodr_schema_valid"),
        "summary": round_result.get("summary"),
        "errors": dict(round_result.get("errors", {})),
        "road_skipped": round_result.get("road_skipped"),
        "qa": {
            "final_verdict": qa_verdict,
            "rounds_run": len(qa_history),
            "max_retries": qa_max_retries,
            "rounds": qa_history,
        },
        "rounds_taken": rounds_taken,
        "fix_loop_final_hints": {
            "road": fix_hint_road,
            "scene": fix_hint_scene,
        },
        "phase_timings": phase_timings,
        "ocl_status": ocl_status,
        "roadgraph_status": roadgraph_status,
        "roadgraph_selfcheck": roadgraph_selfcheck,
        "roadgraph_cache_dir": roadgraph_cache_dir,
        "xosc_path": xosc_path,
        "xosc_status": xosc_status,
        "modeling_status": modeling_status,
        "modeling_dir": modeling_dir,
        "scene_model_summary": scene_model_summary,
        "carla_status": "pending",
        "carla_summary": None,
        "carla_run_dir": None,
        "carla_elapsed_s": None,
        "pcla_agent": pcla_agent,
    }

    # --- phase: carla (run scenario, OUTSIDE retry loop) ---
    if qa_verdict != "pass":
        # Mark downstream cleanly when QA never reached pass on any round.
        if roadgraph_status == "pending":
            result["roadgraph_status"] = "skipped_by_qa"
            _emit("phase_done", {"phase": "roadgraph", "status": "skipped_by_qa"})
        if xosc_status == "pending":
            result["xosc_status"] = "skipped_by_qa"
            _emit("phase_done", {"phase": "xosc", "status": "skipped_by_qa"})
        result["carla_status"] = "skipped_by_qa"
        _emit("phase_done", {"phase": "carla", "status": "skipped_by_qa"})
    elif generation_only:
        result["carla_status"] = "deferred_generation_only"
        _emit("phase_done", {"phase": "carla", "status": "deferred_generation_only"})
    elif roadgraph_status != "ok":
        if xosc_status == "pending":
            result["xosc_status"] = "skipped_roadgraph_failed"
            _emit("phase_done", {"phase": "xosc", "status": "skipped_roadgraph_failed"})
        result["carla_status"] = "skipped_roadgraph_failed"
        _emit("phase_done", {"phase": "carla", "status": "skipped_roadgraph_failed"})
    elif xosc_status != "ok":
        result["carla_status"] = "skipped_xosc_failed"
        _emit("phase_done", {"phase": "carla", "status": "skipped_xosc_failed"})
    elif not carla_enabled:
        result["carla_status"] = "disabled"
        _emit("phase_done", {"phase": "carla", "status": "disabled"})
    else:
        _emit("phase_start", {"phase": "carla"})
        t = time.time()
        try:
            if carla_client is None:
                carla_client = get_carla_client()
            run = carla_client.run_scenario(
                Path(round_result["xodr_path"]), Path(xosc_path),
                name=name, out_dir=run_dir,
                pcla_agent=pcla_agent,
                max_seconds=carla_max_seconds, timeout=carla_timeout,
                scene_seed=result.get("scene_seed"),
            )
            result["carla_run_dir"] = str(run.local_dir)
            result["carla_summary"] = run.summary
            result["carla_status"] = "ok"
            result["carla_elapsed_s"] = round(run.elapsed_s, 2)
            _emit("phase_done", {"phase": "carla", "duration_s": round(time.time() - t, 2),
                                  "termination": (run.summary or {}).get("termination_reason"),
                                  "collisions": (run.summary or {}).get("collision_count"),
                                  "lane_invasions": (run.summary or {}).get("lane_invasion_count")})
            result["scene_outcome"] = _shared_check_scene_outcome(
                result.get("scene_seed"), result.get("carla_summary"),
                Path(run.local_dir) / "sim_feedback.json")
            _emit("phase_done", {"phase": "scene_outcome",
                                  "verdict": result["scene_outcome"].get("verdict"),
                                  "issues": result["scene_outcome"].get("issues")})
        except Exception as exc:  # noqa: BLE001
            result["carla_status"] = f"error: {type(exc).__name__}: {exc}"
            result["errors"]["carla"] = result["carla_status"]
            _emit("phase_error", {"phase": "carla", "error": str(exc)})

    # Persist once so the demo acceptance gate can inspect the exact on-disk
    # artifact set, then attach its auditable pass/fail report to result.json.
    write_json(run_dir / "result.json", result)
    try:
        from tools.validate_demo_run import validate as validate_demo_run
        result["demo_readiness"] = validate_demo_run(run_dir)
    except Exception as exc:  # noqa: BLE001 - readiness must not hide pipeline output
        result["demo_readiness"] = {
            "run_dir": str(run_dir),
            "passed": False,
            "checks": [{
                "name": "validator_runtime",
                "passed": False,
                "detail": f"{type(exc).__name__}: {str(exc)[:500]}",
            }],
        }
    write_json(run_dir / "result.json", result)
    _emit("done", result)
    return result


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Coordinator: report -> road+scene (parallel) -> XODR")
    ap.add_argument("--input", required=True, help="PDF/office/txt/md path OR a raw paragraph")
    ap.add_argument("--name", help="run name / output stem (default: input stem or 'run')")
    ap.add_argument("--out-root", type=Path, default=DEFAULT_OUT_ROOT)
    ap.add_argument("--env", type=Path, default=DEFAULT_ENV)
    ap.add_argument("--model", default=None)
    ap.add_argument("--base-url", default=None)
    ap.add_argument("--api-key-env", default=None)
    ap.add_argument("--vlm-model", default=None,
                    help="Vision model used to read PDF pages (image-in -> text)")
    ap.add_argument("--qa-max-retries", type=int, default=DEFAULT_QA_MAX_RETRIES,
                    help="Extra retry rounds after the initial pass when QA verdict=fail "
                         "(0=no QA loop, 1=initial+1 retry, ...)")
    ap.add_argument("--xsd", type=Path, default=DEFAULT_XSD)
    return ap.parse_args()


def _derive_name(raw_input: str, explicit: str | None) -> str:
    if explicit:
        return explicit
    p = Path(raw_input)
    if p.suffix and (os.sep in raw_input or p.exists()):
        return p.stem
    return "run"


def main() -> int:
    args = parse_args()
    load_env(args.env)

    name = _derive_name(args.input, args.name)
    source: str | Path = args.input
    candidate = Path(args.input)
    if candidate.exists():
        source = candidate if candidate.is_absolute() else REPO_ROOT / candidate

    out_root = args.out_root if args.out_root.is_absolute() else REPO_ROOT / args.out_root
    result = dispatch(
        source, name, out_root,
        model=args.model, base_url=args.base_url,
        api_key_env=args.api_key_env, vlm_model=args.vlm_model,
        xsd_path=args.xsd, qa_max_retries=args.qa_max_retries,
    )
    # Drop the bulky seed bodies from stdout; they're on disk.
    printable = {k: v for k, v in result.items() if k not in {"road_seed", "scene_seed"}}
    print(json.dumps(printable, ensure_ascii=False, indent=2))
    return 0 if not result.get("errors") else 1


if __name__ == "__main__":
    raise SystemExit(main())
