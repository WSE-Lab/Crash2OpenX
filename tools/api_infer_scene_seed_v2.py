#!/usr/bin/env python3
"""Infer the two-axis scene_seed (v2) from Stage 1 facts via API.

Output per NPC = position (axis 1) x behavior block (axis 2) + optional params.
The LLM only makes the DISCRETE choices (which actor is the SUT/ego; each NPC's kind,
position, behavior block) and may optionally override a few params when the report is
explicit. All other params default from the block catalog and are mutation knobs — so a
single inference is enough; no second params pass.

    uv run python tools/api_infer_scene_seed_v2.py --input outputs/facts_skill/<name>.json
"""

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

# ---- catalog (kept in sync with openscenarios/blocks/block_catalog.md) ----
KINDS = {"vehicle", "pedestrian", "cyclist", "static"}
SIDES = {"left", "right", "none"}
SUT_MANEUVERS = {"straight", "left", "right", "overtake_oncoming"}  # ego must be MOVING (no stationary)
POSITIONS = {"ahead_same_lane", "behind_same_lane", "adjacent", "oncoming",
             "roadside", "cross", "opposing_leg"}
CONTROLS = {"traffic_light", "stop_sign", "yield", "none", "unknown"}
# Schema v2.2: environment field. Both axes are controlled vocabularies (LLM
# may not invent values); friction_scale is optional 0.1..1.0 float and is
# only filled if the report explicitly mentions wet/icy/oily surface.
WEATHERS = {"clear", "rain", "snow", "fog", "cloudy", "unknown"}
TIMES_OF_DAY = {"dawn", "morning", "afternoon", "evening", "dusk", "night", "unknown"}

# kind -> allowed behavior blocks
BLOCKS_BY_KIND = {
    "vehicle": {"front_brake", "rear_hit", "cut_in", "oncoming", "stopped_ahead",
                "static_hold",
                "junction_cross", "junction_turn", "light_change_start"},
    "cyclist": {"cut_in", "oncoming", "junction_cross", "cross", "light_change_start"},
    "pedestrian": {"cross", "walk_along", "light_change_start"},
    "static": {"static_block"},
}
# kind -> allowed positions
POSITIONS_BY_KIND = {
    "vehicle": {"ahead_same_lane", "behind_same_lane", "adjacent", "oncoming", "cross", "opposing_leg"},
    "cyclist": {"adjacent", "oncoming", "cross", "roadside"},
    "pedestrian": {"roadside", "ahead_same_lane"},
    "static": {"ahead_same_lane", "roadside"},
}
# block -> allowed param keys (anything else is dropped)
BLOCK_PARAMS = {
    "front_brake": {"speed", "trig_dist", "trig_simtime", "decel", "end_speed", "brake_t"},
    "rear_hit": {"speed", "closing_speed"},
    "cut_in": {"speed", "trig_ttc"},
    "oncoming": {"speed", "encroach"},
    "stopped_ahead": set(),
    "static_hold": set(),
    "light_change_start": {"speed", "trig_simtime"},
    "junction_cross": {"speed", "trig_ttc"},
    "junction_turn": {"speed", "trig_ttc"},
    "cross": {"speed", "trig_dist"},
    "walk_along": {"speed"},
    "static_block": set(),
}
# position -> allowed shared params (gap/lateral/long)
POS_PARAMS = {"gap", "lateral", "long"}
# fallback position per kind (LLM sometimes puts a block name in the position field)
DEFAULT_POSITION = {"vehicle": "ahead_same_lane", "cyclist": "adjacent",
                    "pedestrian": "roadside", "static": "ahead_same_lane"}


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


def compact_input(d: dict[str, Any]) -> dict[str, Any]:
    return {k: d.get(k) for k in ("metadata", "event_description", "environment", "entities",
                                  "participant_actions", "road_context", "collision",
                                  "impact_or_damage", "incident")}


def system_prompt() -> str:
    return f"""你是事故场景 seed 推理器。从事故 facts 推理一个"两轴" scene seed。

只返回一个 JSON object,不要 Markdown / 代码块 / 解释。

定义:
- SUT(ego):场景中**任意一辆 car 类型、正在移动、且具测试价值**的车辆,**不必是报告里的那辆 AV**;从所有移动的 car 里选"驾驶决策最值得测试"的一辆作为 ego(固定 id="ego", kind=vehicle)。其余参与者(含报告里的 AV,若它不是 ego)都作为 NPC,相对 ego 描述。

你只有三种结局,**严禁临时发明新的 block / position**:
- (A) status=supported:能用现有词汇表(下方 position / block 枚举)映射 → 输出 scene。
- (B) status=skipped:属于**可辩护地排除在范围外**的事故类型(见淘汰规则),不是词汇表的缺陷。
- (C) status=needs_extension:这**是一场真实的、双方交互的碰撞**(ego 在移动、有第二参与者),本应能被一个事故场景 DSL 表达,但现有 block/position 里**找不到贴切的原语**。此时不要硬塞最近项、也不要判 skip,而是返回 reason(为什么现有词汇表表达不了)+ proposed(用自然语言描述缺失的 block 或 position 概念,**只描述、不要把新词写进 scene**),供人工 review 后离线扩充词汇表。

淘汰规则(命中任一 → 返回 status=skipped + reason,这些是可辩护的 out-of-scope,不要推位置/行为):
1. 核心机动涉及倒车/后退(reverse/backing) —— CARLA 无法实现,且属 NHTSA 非 V2V 子集。
2. 场景中**没有任何一辆移动的 car 具备测试价值**(例如所有车都静止、或唯一动作是被动被撞)→ skip。
   注意:被撞方若是静止的,可把"主动移动的那辆车"选为 ego —— 例如"前车停住、后车驶来追尾",应选**后车**为 ego(测它能否及时刹停),而不是判 skip。
3. 停车场/私有区域(泊车机动)、单车冲出路缘/失控(无第二参与者)、non-collision(无碰撞目标)。

skipped 与 needs_extension 的判别:**能不能找到第二个交互参与者 + 是不是 ego 主动驾驶决策**。
有 → 若现有积木表达不了,用 needs_extension;无(单车/倒车/泊车/非碰撞) → 用 skipped。
多连环碰撞(>2 方):若能抽出最关键的一对双方冲突 → 按该对表达;实在抽不出 → needs_extension 说明原因。

- 每个 NPC 用两根轴描述:
  - position(相对 ego 的位置):{sorted(POSITIONS)}  (+ side: left/right/none)
  - behavior.block(行为原语,按 kind 限定,见下)
- 不要输出底层坐标/waypoint/时间/触发;触发与具体数值由下游默认+变异决定。
- params 抽取:**这是关键步骤,不是"可省"**。下方「params 抽取规则」列出量化语/定性语/空间保真三类信号,看到任一就换算后填入对应键;完全无信号才走默认。

kind -> 允许的 behavior.block:
  vehicle: front_brake(前车急刹) rear_hit(后车追尾) cut_in(侧向切入) oncoming(对向/越线) stopped_ahead(同车道前方静止) static_hold(任意位置静止;用于"相邻车道停车""对向路口停等"等非同车道前方的静止车辆,与 stopped_ahead 互斥) junction_cross(路口直行冲突) junction_turn(路口转弯冲突) light_change_start(同向前车在路口光变后从 0 启动加速;**仅当 NPC 唯一动作是同向直行启动、无横向切入/转弯时用**;有横向动作优先 cut_in,有路口转向优先 junction_turn)
  时序优先:若报告说前车先在行驶,随后 slowed/braked abruptly 并在碰撞时已停住,必须用 front_brake;只有前车在 ego 接近前就已经静止时才用 stopped_ahead。
  cyclist: cut_in oncoming junction_cross cross light_change_start
  pedestrian: cross(横穿) walk_along(沿路行走) light_change_start
  static: static_block(静止障碍)

kind -> 允许的 position:
  vehicle: ahead_same_lane behind_same_lane adjacent oncoming cross opposing_leg
  cyclist: adjacent oncoming cross roadside
  pedestrian: roadside ahead_same_lane
  static: ahead_same_lane roadside
注意:行人/骑行的 position 是"起点位置"(通常 roadside),`cross` 是 behavior.block 不是 position;别把 block 名填进 position。

每个 block 可选 params 键(填了才会用,否则默认):
  front_brake: speed trig_dist trig_simtime decel end_speed brake_t
  rear_hit: speed closing_speed
  cut_in: speed trig_ttc ; oncoming: speed encroach(stay|cross)
  junction_cross/junction_turn: speed trig_ttc
  cross: speed trig_dist ; walk_along: speed ; stopped_ahead/static_block/static_hold: 无
  light_change_start: speed trig_simtime(秒,默认 2;NPC 在 sim t=trig_simtime 时从 0 启动到 speed)
  位置类通用 params(可选): gap lateral long

params 抽取规则(看到任一信号 → 必须换算后填入,**不许偷懒留空**):

  量化语(单位换算后直接填):
    "X mph" → speed = X * 0.447 (m/s)
    "X km/h" → speed = X / 3.6
    "X feet ahead/behind / X ft 前/后" → gap = X * 0.305
    "X seconds before contact / X 秒后" → 触发时延 → trig_ttc = X;动作时长 → brake_t = X
    "X feet from curb / X ft lateral offset / X 米外" → lateral = X * 0.305

  定性语(按下表查值,**不要凭印象**):
    前车 abruptly / suddenly / 急停 / 突然刹停 → front_brake, trig_simtime = 2.0; brake_t = 0.2; end_speed = 0
    其他 abruptly / suddenly / unexpectedly / 突然 → trig_ttc = 1.5
    gradually / slowly / cautiously / 缓慢 / 减速 → trig_dist = 30; brake_t = 2.5
    hard brake / slammed brakes / 急刹 → brake_t = 0.6; end_speed = 0
    rolled to a stop / decelerating to stop / 减速停下 → brake_t = 2.0; end_speed = 0
    high speed / fast / accelerated / 加速 → speed = 18
    low speed / crawl / 缓行 → speed = 5
    ego stopped at light / waiting at red → sut.params.speed 不填(由 hero_cruise 处理)

  空间保真(横向距离最易被默认 3.5 m 毁掉,必须特别看):
    "side mirror contact / mirror to mirror / 擦碰侧镜" → lateral = 0.1
    "one lane offset / 并排 / adjacent lane" → lateral = 1.5
    "shoulder / off the road / 路肩外 / 草地上" → lateral = 4.0

  报告完全无量化语 + 无强定性词 → 不填该键,走 osc_blocks 默认值。
  填的键必须在该 block 的允许集(见上方"每个 block 可选 params"),否则会被丢弃。

sut:{{"id":"ego","kind":"vehicle","maneuver":"straight|left|right|overtake_oncoming","params":{{"speed":可选}}}}  (ego 必须移动;**overtake_oncoming** = ego 跨对向道超车,需 road.lanes.backward≥1 且 center_line=broken,WF7;典型场景:前方有静止/慢车,ego 越双黄借对向道超车)

支持输出:
{{
  "status":"supported",
  "scene":{{
    "sut":{{"id":"ego","kind":"vehicle","maneuver":"straight"}},
    "npcs":[{{"id":"v2","kind":"vehicle","position":"ahead_same_lane","side":"none","behavior":{{"block":"front_brake"}}}}],
    "collision":{{"a":"v2","b":"ego"}},
    "control":"traffic_light|stop_sign|yield|none|unknown",
    "environment":{{"weather":"clear|rain|snow|fog|cloudy|unknown","time_of_day":"dawn|morning|afternoon|evening|dusk|night|unknown","friction_scale":可选}}
  }},
  "description_zh":"一句中文描述场景与判断依据",
  "evidence":{{"source_snippets":["..."],"reason":"..."}}
}}

环境(environment)抽取规则:
  weather:报告里的明确天气词 → clear / rain / snow / fog / cloudy;无提及 → unknown。
    "raining / wet pavement / damp / drizzle" → rain
    "snowing / snow-covered" → snow
    "fog / foggy / low visibility" → fog
    "overcast / cloudy / gray sky" → cloudy
    "sunny / clear / bright" → clear
  time_of_day:按报告里的时间或定性词 → dawn/morning/afternoon/evening/dusk/night;无提及 → unknown。
    "approximately HH:MM" 或 ISO 时间 → 按小时归档(05:30-06:59→dawn,07:00-11:59→morning,12:00-16:59→afternoon,17:00-18:59→evening,19:00-20:30→dusk,其余→night)
    "in the morning / 上午" → morning;"after dark / at night / 夜间" → night;"sunset / dusk" → dusk
  friction_scale(可选,仅明确提及湿/冰/油 时填):wet=0.5,snow/icy=0.3,oily=0.4,otherwise 不填(默认 1.0)。

(B) 可辩护排除(倒车/泊车/单车冲出/非碰撞/无测试价值)时:
{{"status":"skipped","scene":null,"description_zh":"为何排除","reason":"属于哪类 out-of-scope","evidence":{{"source_snippets":["..."]}}}}

(C) 真实双方碰撞但现有词汇表表达不了时(供人工 review 扩词,**不要把新词写进 scene**):
{{"status":"needs_extension","scene":null,"description_zh":"一句话描述这场碰撞","reason":"现有 block/position 为何表达不了这场双方交互碰撞","proposed":"自然语言描述缺失的 block 或 position 概念(例如:需要一个表示'前车突然向左变道后急停'的复合原语)","evidence":{{"source_snippets":["..."]}}}}

params 抽取范例(每例 params 都是从报告原文换算或查表得到,不是默认值):

范例 A — 量化语 + 急刹(LVD 类)
  报告:"the AV was traveling at approximately 25 mph behind a lead vehicle that suddenly slammed on its brakes from about 30 ft ahead"
  关键 sut:{{"params":{{"speed":11.2}}}}
  关键 npc:{{"id":"v2","kind":"vehicle","position":"ahead_same_lane","side":"none","behavior":{{"block":"front_brake","params":{{"speed":11.2,"trig_dist":9.0,"brake_t":0.6,"end_speed":0,"gap":9.0}}}}}}
  推导:25 mph→11.2(ego 与前车同向、跟车速度相近)、30 ft→9.0、"slammed"→brake_t=0.6 + end_speed=0

范例 B — 侧镜擦碰(空间保真)
  报告:"the AV passed a parked vehicle and made contact with its driver-side mirror"
  关键 npc:{{"id":"parked","kind":"static","position":"roadside","side":"right","behavior":{{"block":"static_block","params":{{"lateral":0.1,"gap":12}}}}}}
  推导:"mirror contact"→lateral=0.1(默认 3.5 会让车根本碰不到)、gap=12 给 ego 留出反应距离

范例 C — 红绿灯起步 + 突然加速冲入(混合)
  报告:"the AV was stopped at a red light; after the light turned green the AV proceeded; the cyclist abruptly veered left and accelerated into the AV's path"
  关键字段:control = traffic_light
  关键 npc:{{"id":"cyclist","kind":"cyclist","position":"adjacent","side":"right","behavior":{{"block":"cut_in","params":{{"trig_ttc":1.5,"speed":6}}}}}}
  推导:"abruptly"→trig_ttc=1.5、"accelerated"→cyclist 速度高于慢行(6 m/s)、控制信号 traffic_light

范例 D — 相邻车道停车擦碰(static_hold + adjacent)
  报告:"the AV was passing a vehicle that had stopped in the adjacent lane and made contact with its side mirror"
  关键 npc:{{"id":"v2","kind":"vehicle","position":"adjacent","side":"left","behavior":{{"block":"static_hold","params":{{"lateral":0.1,"long":2.0}}}}}}
  推导:NPC 是 vehicle 而不是 static(报告说是车而不是路边设施)+ 静止在相邻车道(不是 ahead_same_lane,所以用 static_hold 而不是 stopped_ahead)+ "mirror contact"→lateral=0.1

范例 E — 对向路口停车被左转撞(static_hold + opposing_leg)
  报告:"the AV was making a left turn at the intersection and made contact with a stopped vehicle that was waiting at the stop sign on the opposing leg"
  关键 sut:maneuver = "left"
  关键 npc:{{"id":"v2","kind":"vehicle","position":"opposing_leg","side":"none","behavior":{{"block":"static_hold"}}}}
  推导:NPC 在路口对面停止线等待(opposing_leg + static_hold);ego 左转切入其位置导致碰撞——驾驶决策与位置由 ego 转弯路径自然产生
"""


def user_prompt(d: dict[str, Any]) -> str:
    return ("请把下面事故 facts 推理成两轴 scene_seed JSON。能表达返回 status=supported,否则 skipped。返回 JSON only。\n\n"
            "Input JSON:\n" + json.dumps(compact_input(d), ensure_ascii=False, indent=2))


def extract_json(text: str) -> dict[str, Any]:
    text = text.strip()
    if text.startswith("```"):
        text = re.sub(r"^```(?:json)?\s*", "", text)
        text = re.sub(r"\s*```$", "", text)
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        s, e = text.find("{"), text.rfind("}")
        if s >= 0 and e > s:
            return json.loads(text[s:e + 1])
        raise


def _filter_params(block: str, params: Any) -> dict[str, Any]:
    if not isinstance(params, dict):
        return {}
    allowed = BLOCK_PARAMS.get(block, set()) | POS_PARAMS
    return {k: v for k, v in params.items() if k in allowed}


def _normalize_npc(npc: dict[str, Any]) -> dict[str, Any]:
    nid = str(npc.get("id") or "").strip()
    kind = npc.get("kind")
    pos = npc.get("position")
    side = npc.get("side", "none")
    beh = npc.get("behavior") or {}
    block = beh.get("block")
    if not nid:
        raise ValueError("npc.id required")
    if kind not in KINDS:
        raise ValueError(f"bad kind {kind!r}")
    if pos not in POSITIONS_BY_KIND.get(kind, set()):
        pos = DEFAULT_POSITION.get(kind, "ahead_same_lane")  # tolerate position/block confusion
    if side not in SIDES:
        side = "none"
    if block not in BLOCKS_BY_KIND.get(kind, set()):
        raise ValueError(f"block {block!r} not allowed for kind {kind}")
    return {
        "id": nid, "kind": kind, "position": pos, "side": side,
        "behavior": {"block": block, "params": _filter_params(block, beh.get("params"))},
    }


def _normalize_abrupt_lead_stop(
    npcs: list[dict[str, Any]],
    data: dict[str, Any],
    source_context: dict[str, Any] | None = None,
) -> bool:
    """Correct final-state-only inference when the evidence gives a brake sequence."""
    evidence = data.get("evidence") if isinstance(data.get("evidence"), dict) else {}
    snippets = evidence.get("source_snippets") if isinstance(evidence.get("source_snippets"), list) else []
    context = " ".join([
        *(str(item) for item in snippets),
        str(data.get("description_zh") or ""),
        str(evidence.get("reason") or ""),
        json.dumps(source_context or {}, ensure_ascii=False),
    ]).lower()
    lead_braked = any(term in context for term in (
        "slowed abruptly", "braked abruptly", "suddenly braked", "突然刹", "急刹",
    ))
    stopped = any(term in context for term in ("complete stop", "stopped", "停住", "停下"))
    rear_end = any(term in context for term in (
        "striking the rear", "rear end", "rear-end", "追尾",
    ))
    if not (lead_braked and stopped and rear_end):
        return False
    changed = False
    for npc in npcs:
        if (npc.get("kind") == "vehicle"
                and npc.get("position") == "ahead_same_lane"
                and (npc.get("behavior") or {}).get("block") == "stopped_ahead"):
            npc["behavior"] = {
                "block": "front_brake",
                "params": {"trig_simtime": 2.0, "brake_t": 0.2, "end_speed": 0},
            }
            changed = True
    return changed


def normalize(
    data: dict[str, Any],
    input_path: Path,
    model: str,
    source_context: dict[str, Any] | None = None,
) -> dict[str, Any]:
    pipeline = {"stage": "scene_seed_v2", "generation_mode": "api_deepseek", "model": model,
                "input": display_path(input_path)}
    if data.get("status") == "needs_extension":
        # real two-party collision the current vocabulary cannot express -> flag for
        # human review + offline controlled vocabulary extension (never invent at runtime).
        return {
            "status": "needs_extension", "scene": None,
            "description_zh": str(data.get("description_zh") or ""),
            "reason": str(data.get("reason") or "current block/position vocabulary cannot express this collision"),
            "proposed": str(data.get("proposed") or data.get("missing_concept") or ""),
            "evidence": data.get("evidence") if isinstance(data.get("evidence"), dict) else {},
            "pipeline": pipeline,
        }
    if data.get("status") == "skipped" or not isinstance(data.get("scene"), dict):
        return {
            "status": "skipped", "scene": None,
            "description_zh": str(data.get("description_zh") or ""),
            "reason": str(data.get("reason") or "cannot be expressed by the block catalog"),
            "evidence": data.get("evidence") if isinstance(data.get("evidence"), dict) else {},
            "pipeline": pipeline,
        }
    scene = data["scene"]
    sut = scene.get("sut") or {}
    maneuver = sut.get("maneuver", "straight")
    if maneuver not in SUT_MANEUVERS:
        # ego not moving / no testable driving decision -> reject per filter rule 2
        return {
            "status": "skipped", "scene": None,
            "description_zh": str(data.get("description_zh") or "被测车非移动,无测试价值"),
            "reason": f"SUT must be a moving vehicle with test value (got maneuver={maneuver!r})",
            "evidence": data.get("evidence") if isinstance(data.get("evidence"), dict) else {},
            "pipeline": {"stage": "scene_seed_v2", "generation_mode": "api_deepseek", "model": model,
                         "input": display_path(input_path)},
        }
    npcs = [_normalize_npc(n) for n in (scene.get("npcs") or [])]
    temporal_rule_applied = _normalize_abrupt_lead_stop(npcs, data, source_context)
    ids = ["ego"] + [n["id"] for n in npcs]
    if len(ids) != len(set(ids)):
        raise ValueError(f"duplicate ids {ids}")
    col = scene.get("collision") or {}
    a, b = str(col.get("a") or "").strip(), str(col.get("b") or "ego").strip()
    if a and a not in ids:
        raise ValueError(f"collision.a {a!r} not an actor")
    control = scene.get("control", "unknown")
    if control not in CONTROLS:
        control = "unknown"
    sut_params = {k: v for k, v in (sut.get("params") or {}).items() if k == "speed"}
    # Schema v2.2: environment block (weather + time_of_day + optional friction_scale).
    # Filter to controlled vocabulary; never echo back invented values.
    env_in = scene.get("environment") or {}
    env_out = {
        "weather": env_in.get("weather") if env_in.get("weather") in WEATHERS else "unknown",
        "time_of_day": env_in.get("time_of_day") if env_in.get("time_of_day") in TIMES_OF_DAY else "unknown",
    }
    if "friction_scale" in env_in:
        try:
            fs = float(env_in["friction_scale"])
            if 0.1 <= fs <= 1.0:
                env_out["friction_scale"] = fs
        except (TypeError, ValueError):
            pass
    return {
        "status": "supported",
        "scene": {
            "sut": {"id": "ego", "kind": "vehicle", "maneuver": maneuver, "params": sut_params},
            "npcs": npcs,
            "collision": {"a": a or (npcs[0]["id"] if npcs else ""), "b": b or "ego"},
            "control": control,
            "environment": env_out,
        },
        "description_zh": str(data.get("description_zh") or ""),
        "evidence": data.get("evidence") if isinstance(data.get("evidence"), dict) else {},
        "pipeline": {
            "stage": "scene_seed_v2", "generation_mode": "api_deepseek", "model": model,
            "input": display_path(input_path),
            "normalization_rules": ["abrupt_lead_stop_to_front_brake"] if temporal_rule_applied else [],
        },
    }


def call_model(args: argparse.Namespace, data: dict[str, Any]) -> dict[str, Any]:
    api_key = os.environ.get(args.api_key_env)
    if not api_key:
        raise RuntimeError(f"Missing {args.api_key_env}; export it (e.g. source .env.local) first.")
    client = OpenAI(api_key=api_key, base_url=args.base_url)
    messages: list[dict[str, Any]] = [
        {"role": "system", "content": system_prompt()},
        {"role": "user", "content": user_prompt(data)},
    ]
    fix_hint = (getattr(args, "fix_hint", "") or "").strip()
    if fix_hint:
        messages.append({"role": "user",
                         "content": f"上一次推理被 QA 评审判定不合理,请按以下指导调整后重新推理(仍只输出 JSON):\n{fix_hint}"})
    kwargs: dict[str, Any] = {
        "model": args.model, "stream": False, "temperature": args.temperature,
        "max_tokens": getattr(args, "max_tokens", 8000),
        "messages": messages,
    }
    if args.reasoning_effort:
        kwargs["reasoning_effort"] = args.reasoning_effort
    if args.enable_thinking:
        kwargs["extra_body"] = {"thinking": {"type": "enabled"}}
    resp = client.chat.completions.create(**kwargs)
    return extract_json(resp.choices[0].message.content or "")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Infer two-axis scene_seed (v2) from facts via API")
    ap.add_argument("--input", required=True, type=Path)
    ap.add_argument("--output", type=Path)
    ap.add_argument("--output-dir", type=Path, default=Path("outputs/scene_seed"))
    ap.add_argument("--model", default="deepseek/deepseek-v4-pro")
    ap.add_argument("--base-url", default="https://openrouter.ai/api/v1")
    ap.add_argument("--api-key-env", default="OPENROUTER_API_KEY")
    ap.add_argument("--reasoning-effort", default="high")
    ap.add_argument("--enable-thinking", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--temperature", type=float, default=0.0)
    ap.add_argument("--dry-run", action="store_true")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    inp = args.input if args.input.is_absolute() else REPO_ROOT / args.input
    data = read_json(inp)
    out = args.output or (REPO_ROOT / args.output_dir / f"{inp.stem}.json")
    if not out.is_absolute():
        out = REPO_ROOT / out
    if args.dry_run:
        print(json.dumps({"model": args.model, "system_prompt": system_prompt(),
                          "user_prompt": user_prompt(data), "output": display_path(out)},
                         ensure_ascii=False, indent=2))
        return 0
    raw = call_model(args, data)
    norm = normalize(raw, inp, args.model)
    write_json(out, norm)
    print(json.dumps({"output": display_path(out), **norm}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
