#!/usr/bin/env python3
"""Infer minimal road_seed JSON from facts or semantic scene JSON via API."""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path
from typing import Any

from openai import OpenAI


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


SCHEMA_MD = REPO_ROOT / "schemas/road_seed_schema.md"

TOPOLOGIES = {
    "straight",
    "curve",
    "cross_intersection",
    "t_junction",
    "y_junction",
    "merge",
    "fork",
}
ROAD_TYPES = {
    "town",
    "lowSpeed",
    "rural",
    "motorway",
    "townArterial",
    "townCollector",
    "townLocal",
}
FORWARD_LANES = {1, 2, 3, 4, 5}
BACKWARD_LANES = {0, 1, 2, 3, 4, 5}
CENTER_LINES = {"solid", "broken"}


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def display_path(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def compact_input(data: dict[str, Any]) -> dict[str, Any]:
    """Keep only road-relevant source context while supporting facts or semantic input."""
    return {
        "metadata": data.get("metadata"),
        "event_description": data.get("event_description"),
        "environment": data.get("environment"),
        "road_context": data.get("road_context"),
        "road_model": data.get("road_model"),
        "collision": data.get("collision"),
        "entities": data.get("entities"),
        "actors": data.get("actors"),
        "participant_actions": data.get("participant_actions"),
        "incident": data.get("incident"),
        "raw_widgets": data.get("raw", {}).get("widgets") if isinstance(data.get("raw"), dict) else None,
    }


def system_prompt(schema_md: str) -> str:
    return f"""你是道路 seed 推理器。你的任务是从事故 facts 或 semantic scene JSON 中推理最小 road seed。

只返回一个 JSON object。不要返回 Markdown、代码块、解释文字或多余前后缀。

核心原则：
1. 只推理道路，不推理 actor、轨迹、碰撞时间、OpenDRIVE ID、laneLink 或 waypoint。
2. road seed 必须严格使用 Road Seed Schema v2 中的 topology/type/lanes/center_line。
3. 如果支持的 topology 无法覆盖该场景，不要强行选择最近项。必须返回 unsupported 特殊标准，并说明原因。
4. 不要输出 parking_lot、parking_area、parking_access、driveway_connection、custom_topology 或 allowed_maneuvers。
   也不要输出 roundabout（环岛）：当前 OpenDRIVE 生成工具链不支持环岛几何，遇到环岛场景请返回 status=unsupported。
5. cross_intersection 天然隐含四向 approach 和所有合法直行/左转/右转，不要额外列 allowed maneuvers。
6. 不要输出 left_turn/right_turn。转弯是机动行为，不是道路几何：路口处的左转/右转用 cross_intersection 或 t_junction/y_junction 表达（路口已隐含所有转向 connector）；只有当道路本身是一段独立弯曲的连接道路、且不构成路口时，才用 curve。转弯方向和进出路由属于后续机动/路由阶段，绝不在 road seed 中表达。
7. lanes 是方向感知对象 {{"forward": N, "backward": M}}，相对 ego 的行驶方向：forward = 同向车道数（>=1），backward = 对向车道数（0..5）。
   - 单行道（one-way street）：backward = 0（明确只有一个行驶方向、无对向车道时）。
   - 不对称（如北向1条、南向2条）：forward/backward 取各自实际值。
   - 事实不明确时默认双向各一条：{{"forward": 1, "backward": 1}}；明确 right/left lane、multiple lanes、arterial 时该方向可用 2；four lanes 用 4；five lanes 用 5；不超过 5。
8. center_line ∈ {{"solid","broken"}}：表示能否跨越中心线借用对向车道（如绕过障碍物）。
   - double yellow / solid centerline / 禁止跨越 → "solid"。
   - broken / dashed / 允许借道或超车 → "broken"。
   - 事实不明确时默认 "broken"（保守地保留绕行空间）。单行道（backward=0）时 center_line 仍填，但无实际意义。
9. type 直接使用 OpenDRIVE 1.5M road type 白名单；事实不明确时默认 town。
10. 必须额外输出 description_zh：一句简洁中文，描述该道路的几何（topology/type/lanes/center_line）及判断依据，供人工快速浏览。unsupported 时也要给出一句中文说明为什么无法覆盖。

支持输出格式一：supported
{{
  "status": "supported",
  "road": {{
    "topology": "cross_intersection",
    "type": "town",
    "lanes": {{"forward": 1, "backward": 1}},
    "center_line": "broken"
  }},
  "description_zh": "一句中文描述该道路几何与判断依据",
  "evidence": {{
    "source_snippets": ["从输入中摘录或紧密转述的道路证据"],
    "reason": "为什么选择该 topology/type/lanes/center_line"
  }}
}}

支持输出格式二：unsupported
{{
  "status": "unsupported",
  "road": null,
  "unsupported_topology": "简短描述输入需要的道路结构",
  "nearest_supported_topology": "straight | curve | cross_intersection | t_junction | y_junction | merge | fork | none",
  "description_zh": "一句中文说明输入需要什么道路结构以及为什么无法覆盖",
  "reason": "为什么 Road Seed Schema v2 不能覆盖，必须源于输入事实",
  "evidence": {{
    "source_snippets": ["从输入中摘录或紧密转述的道路证据"]
  }}
}}

Road Seed Schema v2:
{schema_md}
"""


def user_prompt(data: dict[str, Any]) -> str:
    return (
        "请将下面输入推理成一个 road_seed JSON。"
        "如果 Road Seed Schema v2 可以覆盖，返回 status=supported。"
        "如果不能覆盖，返回 status=unsupported 特殊标准并说明理由。"
        "返回 JSON only。\n\n"
        "Input JSON:\n"
        + json.dumps(compact_input(data), ensure_ascii=False, indent=2)
    )


def extract_json(text: str) -> dict[str, Any]:
    text = text.strip()
    if text.startswith("```"):
        text = re.sub(r"^```(?:json)?\s*", "", text)
        text = re.sub(r"\s*```$", "", text)
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        start = text.find("{")
        end = text.rfind("}")
        if start >= 0 and end > start:
            return json.loads(text[start : end + 1])
        raise


def normalize_supported(data: dict[str, Any], input_path: Path, model: str) -> dict[str, Any]:
    road = data.get("road")
    if not isinstance(road, dict):
        raise ValueError("supported output must contain road object")

    # lanes: accept v2 object {forward, backward}; tolerate a bare int (legacy
    # model output) as a symmetric two-way road.
    raw_lanes = road.get("lanes", {"forward": 1, "backward": 1})
    if isinstance(raw_lanes, dict):
        forward = raw_lanes.get("forward", 1)
        backward = raw_lanes.get("backward", forward)
    else:
        forward = backward = raw_lanes
    center_line = road.get("center_line", "broken")

    normalized = {
        "topology": road.get("topology"),
        "type": road.get("type", "town"),
        "lanes": {"forward": forward, "backward": backward},
        "center_line": center_line,
    }
    if normalized["topology"] not in TOPOLOGIES:
        raise ValueError(f"unsupported road.topology: {normalized['topology']!r}")
    if normalized["type"] not in ROAD_TYPES:
        raise ValueError(f"unsupported road.type: {normalized['type']!r}")
    if forward not in FORWARD_LANES:
        raise ValueError(f"unsupported road.lanes.forward: {forward!r}")
    if backward not in BACKWARD_LANES:
        raise ValueError(f"unsupported road.lanes.backward: {backward!r}")
    if center_line not in CENTER_LINES:
        raise ValueError(f"unsupported road.center_line: {center_line!r}")
    return {
        "status": "supported",
        "road": normalized,
        "description_zh": str(data.get("description_zh") or ""),
        "evidence": data.get("evidence") if isinstance(data.get("evidence"), dict) else {},
        "pipeline": {
            "stage": "road_seed",
            "generation_mode": "api_deepseek",
            "model": model,
            "input": display_path(input_path),
        },
    }


def normalize_unsupported(data: dict[str, Any], input_path: Path, model: str) -> dict[str, Any]:
    nearest = data.get("nearest_supported_topology", "none")
    if nearest not in TOPOLOGIES and nearest != "none":
        nearest = "none"
    return {
        "status": "unsupported",
        "road": None,
        "unsupported_topology": str(data.get("unsupported_topology") or "unknown"),
        "nearest_supported_topology": nearest,
        "description_zh": str(data.get("description_zh") or ""),
        "reason": str(data.get("reason") or "Road Seed Schema v2 cannot cover this road structure."),
        "evidence": data.get("evidence") if isinstance(data.get("evidence"), dict) else {},
        "pipeline": {
            "stage": "road_seed",
            "generation_mode": "api_deepseek",
            "model": model,
            "input": display_path(input_path),
        },
    }


def normalize_output(data: dict[str, Any], input_path: Path, model: str) -> dict[str, Any]:
    status = data.get("status")
    if status == "unsupported":
        return normalize_unsupported(data, input_path, model)
    if status == "supported" or isinstance(data.get("road"), dict):
        return normalize_supported(data, input_path, model)
    raise ValueError("model output must be status=supported with road, or status=unsupported")


def call_model(args: argparse.Namespace, data: dict[str, Any]) -> dict[str, Any]:
    api_key = os.environ.get(args.api_key_env)
    if not api_key:
        raise RuntimeError(f"Missing {args.api_key_env}; export it before running this script.")
    schema_md = args.schema_md.read_text(encoding="utf-8")
    client = OpenAI(api_key=api_key, base_url=args.base_url)
    messages: list[dict[str, Any]] = [
        {"role": "system", "content": system_prompt(schema_md)},
        {"role": "user", "content": user_prompt(data)},
    ]
    fix_hint = (getattr(args, "fix_hint", "") or "").strip()
    if fix_hint:
        # QA agent flagged the previous attempt — fold its guidance in as an
        # extra user turn so the model can revise without losing its system rules.
        messages.append({"role": "user",
                         "content": f"上一次推理被 QA 评审判定不合理,请按以下指导调整后重新推理(仍只输出 JSON):\n{fix_hint}"})
    kwargs: dict[str, Any] = {
        "model": args.model,
        "messages": messages,
        "stream": False,
        "temperature": args.temperature,
        "max_tokens": getattr(args, "max_tokens", 8000),
    }
    if args.reasoning_effort:
        kwargs["reasoning_effort"] = args.reasoning_effort
    if args.enable_thinking:
        kwargs["extra_body"] = {"thinking": {"type": "enabled"}}
    response = client.chat.completions.create(**kwargs)
    return extract_json(response.choices[0].message.content or "")


def default_output_path(input_path: Path, output_dir: Path) -> Path:
    return output_dir / f"{input_path.stem}.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Infer road_seed JSON from facts or semantic scene JSON via API")
    parser.add_argument("--input", required=True, type=Path, help="Facts or semantic scene JSON")
    parser.add_argument("--output", type=Path, help="Output road_seed JSON path")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/road_seed"))
    parser.add_argument("--schema-md", type=Path, default=SCHEMA_MD)
    parser.add_argument("--model", default="deepseek/deepseek-v4-pro")
    parser.add_argument("--base-url", default="https://openrouter.ai/api/v1")
    parser.add_argument("--api-key-env", default="OPENROUTER_API_KEY")
    parser.add_argument("--reasoning-effort", default="high")
    parser.add_argument("--enable-thinking", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--temperature", type=float, default=0.0)
    parser.add_argument("--dry-run", action="store_true", help="Print prompt payload without calling the API")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_path = args.input
    if not input_path.is_absolute():
        input_path = REPO_ROOT / input_path
    data = read_json(input_path)
    output_path = args.output or default_output_path(input_path, REPO_ROOT / args.output_dir)
    if not output_path.is_absolute():
        output_path = REPO_ROOT / output_path

    if args.dry_run:
        payload = {
            "model": args.model,
            "base_url": args.base_url,
            "system_prompt": system_prompt(args.schema_md.read_text(encoding="utf-8")),
            "user_prompt": user_prompt(data),
            "output": display_path(output_path),
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0

    raw = call_model(args, data)
    normalized = normalize_output(raw, input_path, args.model)
    write_json(output_path, normalized)
    print(json.dumps({"output": display_path(output_path), **normalized}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
