# Scene Seed Schema v2 — two-axis (simple)

> **v2.2（本版）补**：新增顶层 `scene.environment`（weather + time_of_day + 可选 friction_scale）。三者均为受控词汇表，由 LLM 从报告天气/时间/路面信号抽出，编译期落到 OpenSCENARIO `EnvironmentAction`（TimeOfDay / Weather / RoadCondition）。详见 `vocabulary_derivation.md §4.5`。
>
> **v2.1 补词**：新增 `sut.maneuver=overtake_oncoming`（ego 跨对向道超车，引入 WF7：`backward≥1` 且 `center_line=broken`，见 `road_topology_derivation.md §5`）+ `behavior.block=light_change_start`（同向前车在路口光变后从静止启动加速，LVA 子集，无横向动作时使用）。两条新词均按 `vocabulary_derivation.md §4` 离线受控补词协议升格，详见该文件 v2.1 freeze 记录。

当前场景推理脚本 `tools/api_infer_scene_seed_v2.py` 实际使用的 scene seed 定义。
与 road seed 对称：road 描述**道路几何**，scene 描述**动态语义**（SUT + NPC + 碰撞 + 管制 + 环境）。

核心原则：**SUT 绝对、NPC 相对**。每个 NPC 用**两根轴**描述——`position`（相对 ego 的位置）+ `behavior.block`（行为原语）。
不含底层量：禁止 XY / waypoint / road_id / lane_id / 速度具体值之外的坐标 / 绝对时间 / 触发条件（由下游默认值 + fuzz 决定）。

## 结构

```json
{
  "status": "supported",
  "scene": {
    "sut": { "id": "ego", "kind": "vehicle", "maneuver": "straight" },
    "npcs": [
      { "id": "v2", "kind": "vehicle", "position": "ahead_same_lane",
        "side": "none", "behavior": { "block": "front_brake" } }
    ],
    "collision": { "a": "v2", "b": "ego" },
    "control": "traffic_light"
  },
  "description_zh": "一句中文描述场景与判断依据",
  "evidence": { "source_snippets": ["..."], "reason": "..." }
}
```

## 顶层字段

| 字段 | 必填 | 含义 |
| --- | --- | --- |
| `status` | 是 | `supported` 或 `skipped` |
| `scene` | supported 时必填 | 场景体；skipped 时为 null |
| `description_zh` | 是 | 一句中文描述/判断依据 |
| `evidence` | 是 | `{source_snippets[], reason}` |

## `scene.sut`（ego，必须在移动）

| 字段 | 取值 | 说明 |
| --- | --- | --- |
| `id` | 固定 `"ego"` | — |
| `kind` | 固定 `"vehicle"` | — |
| `maneuver` | `straight` / `left` / `right` / `overtake_oncoming` | **无 stationary**，ego 必须移动；`overtake_oncoming` = ego 跨对向道超车（WF7：需 `road.lanes.backward≥1` 且 `center_line=broken`） |
| `params` | 可选 `{speed}` | 仅报告明确时填 |

> ego 选取：从所有“移动的 car”里选**驾驶决策最值得测**的一辆，不必是报告里的 AV；其余参与者（含 AV）作为 NPC。

## `scene.npcs[]`（两轴描述）

| 字段 | 取值 | 说明 |
| --- | --- | --- |
| `id` | 字符串 | 全局唯一 |
| `kind` | `vehicle` / `pedestrian` / `cyclist` / `static` | — |
| `position` | 见下表（按 kind 限定） | 相对 ego 的**起点位置** |
| `side` | `left` / `right` / `none` | 默认 `none` |
| `behavior.block` | 见下表（按 kind 限定） | 行为原语 |
| `behavior.params` | 见下表（按 block 限定） | 仅报告明确时填，否则省略 |

### kind → 允许的 `position`

| kind | position |
| --- | --- |
| vehicle | ahead_same_lane, behind_same_lane, adjacent, oncoming, cross, opposing_leg |
| cyclist | adjacent, oncoming, cross, roadside |
| pedestrian | roadside, ahead_same_lane |
| static | ahead_same_lane, roadside |

> 全局 position 枚举：`ahead_same_lane, behind_same_lane, adjacent, oncoming, roadside, cross, opposing_leg`。
> 注意：行人/骑行的 `cross` 是 **block** 不是 position；position 填起点（通常 roadside）。

### kind → 允许的 `behavior.block`

| kind | block |
| --- | --- |
| vehicle | front_brake, rear_hit, cut_in, oncoming, stopped_ahead, junction_cross, junction_turn, light_change_start |
| cyclist | cut_in, oncoming, junction_cross, cross, light_change_start |
| pedestrian | cross, walk_along, light_change_start |
| static | static_block |

### block → 允许的 `params` 键（其余被丢弃）

| block | params |
| --- | --- |
| front_brake | speed, trig_dist, decel, end_speed, brake_t |
| rear_hit | speed, closing_speed |
| cut_in | speed, trig_ttc |
| oncoming | speed, encroach |
| junction_cross | speed, trig_ttc |
| junction_turn | speed, trig_ttc |
| cross | speed, trig_dist |
| walk_along | speed |
| light_change_start | speed, trig_simtime |
| stopped_ahead / static_block | （无） |

> 位置类通用 params（可选，任意 block 可带）：`gap`, `lateral`, `long`。

## `scene.collision`

`{ "a": <id>, "b": <id> }`：a = 主动/肇事方，b = 被撞方。a、b ∈ {ego} ∪ {npc ids}，且 a ≠ b。

## `scene.control`

`traffic_light` / `stop_sign` / `yield` / `none` / `unknown`（默认 unknown）。
当前 seed XODR 不编码信号灯几何，`control` 仅映射为演员行为。

## `scene.environment`（v2.2 新增）

| 字段 | 取值 | 默认 | 说明 |
| --- | --- | --- | --- |
| `weather` | `clear` / `rain` / `snow` / `fog` / `cloudy` / `unknown` | `unknown` | 天气受控词汇；`unknown` 编译为晴天 |
| `time_of_day` | `dawn` / `morning` / `afternoon` / `evening` / `dusk` / `night` / `unknown` | `unknown` | 定性时段；编译期映射到 ISO 时刻供 OpenSCENARIO `TimeOfDay`（dawn=06:00、morning=09:00、afternoon=14:00、evening=18:00、dusk=19:30、night=22:00），太阳位置由 `_build_env_action` 按小时自动推导 |
| `friction_scale` | float `0.1..1.0` 可选 | 1.0 | 仅当报告明确提到湿/雪/冰/油 时填（wet=0.5、snow/icy=0.3、oily=0.4）；编译为 `RoadCondition.frictionScaleFactor` |

**抽取来源效度**：取值受控、运行时禁造词；与 `weather` / `time_of_day` 对应的报告语模式列在 `tools/api_infer_scene_seed_v2.py` system prompt 的"环境(environment)抽取规则"段。`unknown` 而不是默认值——区分"报告没说"与"报告说是晴天"。

## 三种结局（运行时只能映射/排除/标记，严禁造词）

| status | 含义 |
| --- | --- |
| `supported` | 能用现有 position/block 词汇表映射 → 输出 scene |
| `skipped` | 可辩护地排除在范围外（见下方淘汰规则），非词汇表缺陷 |
| `needs_extension` | **真实双方交互碰撞**，本应可表达，但现有 block/position 无贴切原语 → 返回 `reason` + `proposed`（自然语言描述缺失概念），供人工 review 后**离线**扩词，见 `vocabulary_derivation.md §4` |

> `needs_extension` 输出 `{status, scene:null, description_zh, reason, proposed, evidence}`，**不把新词写进 scene**。
> 判别 skipped vs needs_extension：有无第二个交互参与者 + 是否 ego 主动驾驶决策。有→needs_extension；无（单车/倒车/泊车/非碰撞）→skipped。

## skip 规则（命中任一 → `status: "skipped"`，可辩护 out-of-scope）

1. 核心机动涉及倒车/后退（CARLA 无法实现，且属 NHTSA 非 V2V 子集）。
2. 没有任何一辆“移动的 car”具备测试价值（全静止 / 唯一动作是被动被撞）。
   - 例外：前车停住、后车驶来追尾 → 选**后车**为 ego，不算 skip。
3. 停车场 / 私有区域（泊车）、单车冲出路缘/失控（无第二参与者）、non-collision（无碰撞目标）。

## 示例

```json
{ "status": "supported", "scene": {
  "sut": { "id": "ego", "kind": "vehicle", "maneuver": "straight" },
  "npcs": [ { "id": "v2", "kind": "vehicle", "position": "ahead_same_lane",
             "side": "none", "behavior": { "block": "front_brake", "params": { "trig_dist": 15 } } } ],
  "collision": { "a": "ego", "b": "v2" }, "control": "none" } }
```

```json
{ "status": "supported", "scene": {
  "sut": { "id": "ego", "kind": "vehicle", "maneuver": "left" },
  "npcs": [ { "id": "ped", "kind": "pedestrian", "position": "roadside",
             "side": "right", "behavior": { "block": "cross" } } ],
  "collision": { "a": "ego", "b": "ped" }, "control": "traffic_light" } }
```
