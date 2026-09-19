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
import math
import os
import re
import sys
from pathlib import Path
from typing import Any

from openai import OpenAI

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.scene_actor_catalog import actor_attributes
from tools.scene_contacts import normalize_contacts
from tools.scene_sequences import normalize_steps, validate_sequence_targets
from tools.scene_positions import ordered_npcs

# ---- catalog (kept in sync with openscenarios/blocks/block_catalog.md) ----
KINDS = {"vehicle", "pedestrian", "cyclist", "static"}
SIDES = {"left", "right", "none"}
SUT_MANEUVERS = {"straight", "left", "right", "overtake_oncoming", "overtake_solid_centerline"}
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
                "static_hold", "cruise", "sequence", "partial_lane_intrusion",
                "junction_cross", "junction_turn", "junction_merge", "light_change_start"},
    "cyclist": {"cut_in", "oncoming", "junction_cross", "cross", "light_change_start", "cruise"},
    "pedestrian": {"cross", "walk_along", "light_change_start"},
    "static": {"static_block"},
}
# kind -> allowed positions
POSITIONS_BY_KIND = {
    "vehicle": {"ahead_same_lane", "behind_same_lane", "adjacent", "oncoming", "cross", "opposing_leg", "roadside"},
    "cyclist": {"adjacent", "oncoming", "cross", "roadside"},
    "pedestrian": {"roadside", "ahead_same_lane"},
    "static": {"ahead_same_lane", "roadside"},
}
# block -> allowed param keys (anything else is dropped)
BLOCK_PARAMS = {
    "sequence": set(),
    "cruise": {"speed"},
    "front_brake": {"speed", "trig_dist", "trig_simtime", "decel", "end_speed", "brake_t"},
    "rear_hit": {"speed", "closing_speed"},
    "cut_in": {"speed", "trig_ttc"},
    "partial_lane_intrusion": {"speed", "offset_fraction", "max_lateral_acceleration"},
    "oncoming": {"speed", "encroach"},
    "stopped_ahead": set(),
    "static_hold": set(),
    "light_change_start": {"speed", "trig_simtime"},
    "junction_cross": {"speed", "trig_ttc"},
    "junction_turn": {"speed", "trig_ttc"},
    "junction_merge": {"speed"},
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
- SUT(ego):场景中**任意一辆具有主动驾驶行为和测试价值的道路车辆**（包括货车）,允许初始静止后起步,**不必是报告里的那辆 AV**;选"驾驶决策最值得测试"的一辆作为 ego(固定 id="ego", kind=vehicle)。其余参与者(含报告里的 AV,若它不是 ego)都作为 NPC,相对 ego 描述。必须保留各自 vehicle_class 和 evidence 中的原报告角色，不得把重型卡车替换成普通轿车。

你只有三种结局,**严禁临时发明新的 block / position**:
- (A) status=supported:能用现有词汇表(下方 position / block 枚举)映射 → 输出 scene。
- (B) status=skipped:属于**可辩护地排除在范围外**的事故类型(见淘汰规则),不是词汇表的缺陷。
- (C) status=needs_extension:这**是一场真实的、双方交互的碰撞**(ego 在移动、有第二参与者),本应能被一个事故场景 DSL 表达,但现有 block/position 里**找不到贴切的原语**。此时不要硬塞最近项、也不要判 skip,而是返回 reason(为什么现有词汇表表达不了)+ proposed(用自然语言描述缺失的 block 或 position 概念,**只描述、不要把新词写进 scene**),供人工 review 后离线扩充词汇表。

淘汰规则(命中任一 → 返回 status=skipped + reason,这些是可辩护的 out-of-scope,不要推位置/行为):
1. 核心机动涉及倒车/后退(reverse/backing) —— CARLA 无法实现,且属 NHTSA 非 V2V 子集。
2. 场景中**没有任何一辆移动的 car 具备测试价值**(例如所有车都静止、或唯一动作是被动被撞)→ skip。
   注意:被撞方若是静止的,可把"主动移动的那辆车"选为 ego —— 例如"前车停住、后车驶来追尾",应选**后车**为 ego(测它能否及时刹停),而不是判 skip。
3. 停车场/私有区域(泊车机动，不包括公共道路路边停车带行进)、单车冲出路缘/失控(无第二参与者)、non-collision(无碰撞目标)。

skipped 与 needs_extension 的判别:**能不能找到第二个交互参与者 + 是不是 ego 主动驾驶决策**。
有 → 若现有积木表达不了,用 needs_extension;无(单车/倒车/泊车/非碰撞) → 用 skipped。
原文完整性优先：不能为匹配词汇表而只保留最后一个动作、最关键的一次接触，或删除导致避让的骑行者/前车。多次接触、分离后再次撞击、先越过再并回、部分侵入邻道等完整动作若当前字段无法表达，必须返回 needs_extension，并明确列出缺失的时序或参与者关系，不能把降级后的单次冲突标为 supported。

- 每个 NPC 用两根轴描述:
  - position(相对 ego 的位置):{sorted(POSITIONS)}  (+ side: left/right/none)
  - behavior.block(行为原语,按 kind 限定,见下)
- 不要输出底层坐标/waypoint/时间/触发;触发与具体数值由下游默认+变异决定。
- 车辆类别必须保留：kind=vehicle 的 SUT/NPC 可用 vehicle_class=car/suv/pickup/van/truck/bus/ambulance。原文明确 heavy truck/bus/van/SUV 等时必须填写，不能都简化成乘用车；未明确则省略。停放或停止的汽车仍是 kind=vehicle，不能改成 static；static 只用于非车辆障碍物。
- 路侧停放车辆用 position=roadside、behavior.block=static_hold，并按原文设置 parked_heading=parallel/opposite/perpendicular（相对 ego 的车道行驶方向，未说明时省略，车辆默认为 parallel）。人行横穿的默认朝向不能套在停放车辆上。
- straight 表示沿当前道路行驶，包括沿弯曲道路转向。只有在交叉口转入另一条道路才选 left/right；没有路口证据时不能把沿道路弯曲行驶改为路口转弯。
- 所有position按ego转弯前的起点道路解释：oncoming只表示ego当前道路的对向车道；opposing_leg表示路口正对面的入口；cross表示左右横向道路的入口，side按ego起始朝向判断。若ego转入横向道路后才面对停等车辆，该车辆须用cross + static_hold，不能用oncoming将其错放在转弯前的道路上。static_hold也适用于cross，表示横向道路停止线前静止等待，并不让该车横穿路口。
- NPC可选relative_to指定位置参考参与者id，省略为ego。队列中的前车可为position=ahead_same_lane、relative_to=middle，middle再相对ego描述；不能把队列里多车都放成相对ego的同一默认位置。引用必须来自本场景且不能成环。relative_to只改变位置参考，不改变碰撞对或控制目标。
- 如果原文只描述某车被后车撞后被动向前推，该被动车不应被选为有主动驾驶决策的ego；应选择原文有主动行驶行为的参与者。禁止用ego默认起步速度替代事故引起的被动位移。
- vehicle/cyclist 可用 behavior.block=cruise 表示在当前车道持续行进，无切入、无额外急刹。导致别车避让的直行骑行者也必须保留为 cruise，不能为符合旧词汇而改成 cut_in；cruise 仅可带 speed 及位置参数，数值仍须有原文依据。
- vehicle 可用 partial_lane_intrusion 表示从相邻车道部分侵入 ego 所在车道：position=adjacent，side 表示初始侧，运动朝 ego；保持部分侵入，不完成整次换道。参数仅 speed、offset_fraction（0到0.5之间）、max_lateral_acceleration，缺省按新地图车道宽度的0.4和0.8m/s²编译，是公开默认值，不是原文事实。若报告里的AV需要该动作，可将AV作为此NPC，选择另一辆具有主动驾驶行为的车辆为SUT；必须披露角色，不改变道路、接触对象和前后关系。为其让行的同向骑行者保留为 relative_to=该NPC、position=roadside、side=right/left、block=cruise，不能删掉骑行者或改成骑行者切入。骑行者路边cruise缺省与参照车并排，并保持路侧偏移；明确纵向距离应保留。含返回、停车等后续阶段时仍需完整表达，不能压缩成持续部分侵入。
- vehicle 还可用 position=adjacent、behavior.block=junction_merge：NPC 与 ego 从同一道路的相邻同向车道进入路口，转向同一出口并汇入 ego 出口车道。仅 ego 为 left/right 且原文明确双方同向转弯并切入时使用；与普通直线换道 cut_in 区别是整个转弯路径来自生成地图的路口连接，不能用 cut_in 省掉转弯。junction_merge 的行为参数仅 speed，不能表达先反方向换道等额外阶段。
- scene.collisions 可用有序数组列出原文全部接触，如 [{{"a":"rear","b":"middle"}},{{"a":"middle","b":"ego"}}]；顺序表示首次接触先后，每对内部 a/b 无序。兼容字段 collision 必须等于数组第一对。三车物理链式接触可用该数组加完整的三个车辆及其初始行为表达，不必为被撞后的被动运动发明强制轨迹。它只是期望，不能强制碰撞。相同两车分离再撞可重复列出，但单一 rear_hit 仍不能表达减速分离和再次加速；若缺行为阶段仍返回 needs_extension，不能只加数组就认可完整性。
- vehicle 的 behavior.block=sequence 可显式描述纵向及横向多阶段过程，behavior.steps 为至少2项数组；每项结构为 {{"action":"drive|match_speed|brake|lane_change","params":{{...}},"when":{{"condition":"start|contact|after_previous|separated_and_target_stopped",...}}}}。所有步骤按顺序执行，只有第1步用start。drive的params允许speed/duration；brake允许end_speed/duration；match_speed只允许duration，且步骤顶层target为匹配速度的参与者id。contact与separated_and_target_stopped须在when内给target。when可选delay(秒)，separated_and_target_stopped另可选clearance(净空米)、standstill_duration(秒)。参数仅据原文量化/定性信号填写，缺省由编译器公开默认给出。典型多次接触可为drive→contact触发match_speed→after_previous触发brake→separated_and_target_stopped触发drive；必须同时在collisions保留两次接触。匹配速度是控制目标，不保证持续接触；真实轨迹另验收。lane_change的params仅允许direction(left/right)、lanes(整数1..5，缺省1)、distance(米，缺省20)，须先drive再换道。可用drive→lane_change(left,lanes=2)→lane_change(right,lanes=1)表达从右侧车道跨过中间车道进入左侧停车带、再返回中间车道，所有目标车道须在新地图中存在；停车带类型由RoadSeed表示，不得当成普通行车道。连续换道用after_previous按顺序，不能用单个cut_in省略超车过程。停车带行进不是停车场泊车，不应因此skip。不支持强制碰撞或人为旋转。
- params 抽取:**这是关键步骤,不是"可省"**。下方「params 抽取规则」列出量化语/定性语/空间保真三类信号,看到任一就换算后填入对应键;完全无信号才走默认。

kind -> 允许的 behavior.block:
  vehicle: front_brake(前车急刹) rear_hit(后车追尾) cut_in(侧向切入) oncoming(对向/越线) stopped_ahead(同车道前方静止) static_hold(任意位置静止;用于"相邻车道停车""对向路口停等"等非同车道前方的静止车辆,与 stopped_ahead 互斥) junction_cross(路口直行冲突) junction_turn(路口转弯冲突) light_change_start(同向前车在路口光变后从 0 启动加速;**仅当 NPC 唯一动作是同向直行启动、无横向切入/转弯时用**;有横向动作优先 cut_in,有路口转向优先 junction_turn)
  时序优先:若报告说前车先在行驶,随后 slowed/braked abruptly 并在碰撞时已停住,必须用 front_brake;只有前车在 ego 接近前就已经静止时才用 stopped_ahead。
  cyclist: cut_in oncoming junction_cross cross light_change_start
  pedestrian: cross(横穿) walk_along(沿路行走) light_change_start
  static: static_block(静止障碍)

kind -> 允许的 position:
  vehicle: ahead_same_lane behind_same_lane adjacent oncoming cross opposing_leg roadside
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
  pedestrian cross 是完整横穿初始道路，编译器按新XODR车道/路肩宽度确定起终点，在对侧路缘外停止；默认路缘净空0.75米不是原文事实，不要自行写入params。明确lateral保留。部分横穿或另有中途动作不能压缩成cross，应needs_extension。 原文未说明起始侧时保留side=none，编译器默认从右侧横穿，并在C2XPedestrianCrossings中记录默认假设；不得把默认方向当作原文事实。
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
    选定的 ego 初始静止、随后起步 → sut.params.initial_speed_mps = 0;与后续 speed 分开记录。

  空间保真(横向距离最易被默认 3.5 m 毁掉,必须特别看):
    lateral 是 NPC 中心相对 ego 所在车道中心的横向距离，单位米，不是两车车身或后视镜之间的间隙。
    "side mirror contact / mirror to mirror / 擦碰侧镜" 本身不能推导 lateral 数值，禁止据此填写 0.1。只保留接触语义与证据；初始位置由编译器默认或明确的原文距离给出。
    "one lane offset / 并排 / adjacent lane" → position=adjacent，用车道拓扑定位，不填写任意 lateral 数值。
    相邻车道的 behind / 后方 必须同时保留position=adjacent和params.long<0；若只有定性“后方”没有距离，用公开默认long=-12米，不得省略后变成默认前方。该数值为仿真初始化假设，不是报告测量，须在evidence注明。long放在behavior.params中（不能放在steps动作参数里）。
    "shoulder / off the road / 路肩外 / 草地上" → position=roadside，未提供中心距离时不填 lateral。
    路侧参与者若明确 "approaching from behind / 从后方接近"，须保留 position=roadside 并填 gap<0；只有定性后方而无距离时可用公开初始化默认gap=-12米，并在evidence中注明非报告测量。roadside不等于后方；未填gap通常会成为前方25米（路侧cyclist cruise例外，缺省0米并排）。不要把相邻位置的long误用于路侧gap，也不能只在description_zh描述后方而不编码。

  报告完全无量化语 + 无强定性词 → 不填该键,走 osc_blocks 默认值。
  填的键必须在该 block 的允许集(见上方"每个 block 可选 params"),否则会被丢弃。

sut:{{"id":"ego","kind":"vehicle","maneuver":"straight|left|right|overtake_oncoming|overtake_solid_centerline","params":{{"speed":可选,"initial_speed_mps":可选}}}}  (ego 在场景中有主动驾驶行为,允许初始静止后起步;速度单位 m/s,有限且非负。initial_speed_mps 优先用于初始化,其后由 PCLA 自主控制。overtake_oncoming 表示虚线处借对向车道超车,需 backward≥1、center_line=broken；overtake_solid_centerline 专门记录原文明确越双黄/实线借对向道超车,需 backward≥1、center_line=solid。后者是报告中已发生的违规机动描述,不是道路许可；evidence必须引用原文越线语句,禁止为了通过约束把实线改成虚线。两种机动都保留同向目标车道,由PCLA自主决定避让或停车,是否实际越线/擦碰须验收轨迹。)

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
  关键 npc:{{"id":"parked","kind":"vehicle","vehicle_class":"car","position":"roadside","side":"right","parked_heading":"parallel","behavior":{{"block":"static_hold"}}}}
  推导:停放汽车仍保留车辆类别，平行路边停放；mirror contact 不是距车道中心 0.1 米的证据。碰撞是否重现须由真实运行另行验证。

范例 C — 红绿灯起步 + 突然加速冲入(混合)
  报告:"the AV was stopped at a red light; after the light turned green the AV proceeded; the cyclist abruptly veered left and accelerated into the AV's path"
  关键字段:control = traffic_light
  关键 npc:{{"id":"cyclist","kind":"cyclist","position":"adjacent","side":"right","behavior":{{"block":"cut_in","params":{{"trig_ttc":1.5,"speed":6}}}}}}
  推导:"abruptly"→trig_ttc=1.5、"accelerated"→cyclist 速度高于慢行(6 m/s)、控制信号 traffic_light

范例 D — 相邻车道停车擦碰(static_hold + adjacent)
  报告:"the AV was passing a vehicle that had stopped in the adjacent lane and made contact with its side mirror"
  关键 npc:{{"id":"v2","kind":"vehicle","position":"adjacent","side":"left","behavior":{{"block":"static_hold"}}}}
  推导:NPC 是 vehicle 而不是 static，停在相邻车道，用 static_hold；镜面接触不能凭空量化初始中心位置。

范例 E — 对向路口停车被左转撞(static_hold + opposing_leg)
  报告:"the AV was making a left turn at the intersection and made contact with a stopped vehicle that was waiting at the stop sign on the opposing leg"
  关键 sut:maneuver = "left"
  关键 npc:{{"id":"v2","kind":"vehicle","position":"opposing_leg","side":"none","behavior":{{"block":"static_hold"}}}}
  推导:NPC 在路口对面停止线等待(opposing_leg + static_hold);ego 左转切入其位置导致碰撞——驾驶决策与位置由 ego 转弯路径自然产生
"""


def user_prompt(d: dict[str, Any]) -> str:
    return ("请把下面事故 facts 完整推理成两轴 scene_seed JSON。能完整表达返回 supported；真实交互因词汇表缺失无法表达返回 needs_extension；只有明确符合排除规则才返回 skipped。不得删除参与者、前序动作或后续接触来获得 supported。返回 JSON only。\n\n"
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
    position_params = npc.get('params') or {}
    if not isinstance(position_params, dict) or set(position_params)-POS_PARAMS:
        raise ValueError('NPC top-level params may only contain gap/lateral/long')
    params = _filter_params(block, beh.get('params'))
    for key, value in position_params.items():
        if key in params and params[key] != value:
            raise ValueError(f'conflicting NPC position parameter {key}')
        params[key] = value
    return {
        "id": nid, "kind": kind, "position": pos, "side": side,
        "behavior": {"block": block, "params": params},
        **actor_attributes(npc),
        **({'relative_to': npc['relative_to']} if 'relative_to' in npc else {}),
    }


def _normalize_abrupt_lead_stop(
    npcs: list[dict[str, Any]],
    data: dict[str, Any],
    source_context: dict[str, Any] | None = None,
) -> bool:
    """Correct final-state-only inference when the evidence gives a brake sequence."""
    evidence = data.get("evidence") if isinstance(data.get("evidence"), dict) else {}
    snippets = evidence.get("source_snippets") if isinstance(evidence.get("source_snippets"), list) else []
    # Model explanations can negate an action ("非先行驶后急刹") while still
    # mentioning its keyword. They are not source facts and must not rewrite
    # a correctly inferred stationary lead into an abrupt-braking vehicle.
    context = (json.dumps(source_context, ensure_ascii=False) if source_context
               else " ".join(str(item) for item in snippets)).lower()
    lead_braked = any(term in context for term in (
        "slowed abruptly", "braked abruptly", "suddenly braked", "abrupt stop",
        "stopped abruptly", "sudden stop", "stopped suddenly", "突然刹", "急刹", "急停",
    ))
    stopped = any(term in context for term in ("complete stop", "stopped", "to a stop", "停住", "停下"))
    rear_end = any(term in context for term in (
        "striking the rear", "struck its rear", "struck the rear", "rear end", "rear-end", "追尾",
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
    for original, npc in zip(scene.get('npcs') or [], npcs):
        if 'ads_trigger' in (original.get('behavior') or {}):
            from tools.ads_trigger import normalize_ads_trigger
            npc['behavior']['ads_trigger'] = normalize_ads_trigger(original)
        if npc['behavior']['block'] == 'sequence':
            npc['behavior']['steps'] = normalize_steps((original.get('behavior') or {}).get('steps'))
    temporal_rule_applied = _normalize_abrupt_lead_stop(npcs, data, source_context)
    ids = ["ego"] + [n["id"] for n in npcs]
    if len(ids) != len(set(ids)):
        raise ValueError(f"duplicate ids {ids}")
    validate_sequence_targets(npcs, set(ids))
    ordered_npcs(npcs)  # Reject missing or cyclic position references.
    contacts = normalize_contacts(scene, set(ids))
    control = scene.get("control", "unknown")
    if control not in CONTROLS:
        control = "unknown"
    sut_params = {k: v for k, v in (sut.get("params") or {}).items()
                  if k in ("speed", "initial_speed_mps")}
    for key, value in sut_params.items():
        if (not isinstance(value, (int, float)) or isinstance(value, bool)
                or not math.isfinite(value) or value < 0):
            raise ValueError(f"sut.params.{key} must be finite and non-negative")
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
            "sut": {"id": "ego", "kind": "vehicle", "maneuver": maneuver, "params": sut_params,
                    **actor_attributes({**sut, "kind": "vehicle"})},
            "npcs": npcs,
            "collision": contacts[0],
            **({"collisions": contacts} if "collisions" in scene else {}),
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
    from tools.model_transport import complete_chat_completion
    resp = complete_chat_completion(client, **kwargs)
    return extract_json(resp.choices[0].message.content or "")


def parse_args() -> argparse.Namespace:
    from tools.coordinator import load_env
    from tools.model_transport import model_settings
    load_env(REPO_ROOT / ".env.local")
    defaults = model_settings()
    ap = argparse.ArgumentParser(description="Infer two-axis scene_seed (v2) from facts via API")
    ap.add_argument("--input", required=True, type=Path)
    ap.add_argument("--output", type=Path)
    ap.add_argument("--output-dir", type=Path, default=Path("outputs/scene_seed"))
    ap.add_argument("--model", default=defaults["model"])
    ap.add_argument("--base-url", default=defaults["base_url"])
    ap.add_argument("--api-key-env", default=defaults["api_key_env"])
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
