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
| `maneuver` | `straight` / `left` / `right` / `overtake_oncoming` / `overtake_solid_centerline` | **无 stationary**；后两种分别表示虚线借道超车、原文明确的越实线借道超车，均需对向车道，分别要求 broken / solid |
| `params` | 可选 `{speed, initial_speed_mps}` | 单位 m/s，有限且非负；`initial_speed_mps` 明确初始状态，优先于 `speed` 初始化，随后由 PCLA 控制 |
| `vehicle_class` | 可选 `car / suv / pickup / van / truck / bus / ambulance` | 保留原文明确的车辆类别；由通用目录映射至 CARLA 模型，不由 LLM 指定资产名 |

> ego 选取：从所有“移动的 car”里选**驾驶决策最值得测**的一辆，不必是报告里的 AV；其余参与者（含 AV）作为 NPC。
> 初始静止、随后主动起步的 ego 仍可测试；用 `initial_speed_mps: 0` 表示初始状态。省略所有速度字段时才使用编译器的默认初始化速度。
> `overtake_solid_centerline` 是原文违规机动的显式记录，不是跨线许可，必须在 evidence 中保留越实线/双黄线的原句。P3/WF7 保持旧 `overtake_oncoming` 的虚线约束，对新值检查 `backward≥1 && center_line=solid`。编译器保留同向目标车道和 PCLA 外部控制，在 `C2XSourceManeuver` 参数中记录期望机动；不设置对向车道远端目标，不强制越线或碰撞，实际执行另行验收。

## `scene.npcs[]`（两轴描述）

| 字段 | 取值 | 说明 |
| --- | --- | --- |
| `id` | 字符串 | 全局唯一 |
| `kind` | `vehicle` / `pedestrian` / `cyclist` / `static` | — |
| `position` | 见下表（按 kind 限定） | 相对 ego 的**起点位置** |
| `side` | `left` / `right` / `none` | 默认 `none` |
| `relative_to` | 可选，已声明参与者id | 位置参考对象，默认ego；可表达相对中间车辆的位置，引用必须无环 |
| `behavior.block` | 见下表（按 kind 限定） | 行为原语 |
| `behavior.params` | 见下表（按 block 限定） | 仅报告明确时填，否则省略 |
| `vehicle_class` | 同 SUT，可选；仅 `kind=vehicle` | 卡车、公交车、厢式车和救护车等不能静默简化为乘用车 |
| `parked_heading` | 可选 `parallel / opposite / perpendicular` | 路侧静止车辆相对 ego 所在车道行驶方向的朝向；默认 parallel |

`cyclist` 编译为 OpenSCENARIO `VehicleCategory=bicycle`，CARLA 模型为
`vehicle.diamondback.century`；必须保留两轮参与者，不能只给汽车附加骑行者标签。

### kind → 允许的 `position`

| kind | position |
| --- | --- |
| vehicle | ahead_same_lane, behind_same_lane, adjacent, oncoming, cross, opposing_leg, roadside |
| cyclist | adjacent, oncoming, cross, roadside |
| pedestrian | roadside, ahead_same_lane |
| static | ahead_same_lane, roadside |

> 全局 position 枚举：`ahead_same_lane, behind_same_lane, adjacent, oncoming, roadside, cross, opposing_leg`。
> 所有位置以 ego **转弯前**的道路和朝向为参考。`oncoming` 是同一道路的对向车道，`opposing_leg` 是路口正对面的入口，`cross` 是左右横向道路入口。ego 转弯后的出口对向停等车应使用 `cross + static_hold`，side 取该入口相对 ego 初始朝向的方向；不能用 `oncoming` 将其放在原道路上。`cross` 是起点位置，不要求车辆移动。
> 注意：行人/骑行的 `cross` 是 **block** 不是 position；position 填起点（通常 roadside）。

`roadside` 只表示横向处于路侧，纵向使用有符号 `params.gap`，正数在参照车前、负数在后（`relative_to`省略时参照ego）。默认通常为前方25米，路侧cyclist的cruise默认并排0米。原文明示从后方接近时必须编码负gap；缺精确距离可采用公开初始化假设-12米并在evidence中注明，不能仅在文字描述中保留后方。`adjacent`的纵向参数则为`long`。

### kind → 允许的 `behavior.block`

| kind | block |
| --- | --- |
| vehicle | front_brake, rear_hit, cut_in, partial_lane_intrusion, oncoming, stopped_ahead, static_hold, junction_cross, junction_turn, junction_merge, light_change_start, cruise, sequence |
| cyclist | cut_in, oncoming, junction_cross, cross, light_change_start, cruise |
| pedestrian | cross, walk_along, light_change_start |
| static | static_block |

### block → 允许的 `params` 键（其余被丢弃）

| block | params |
| --- | --- |
| front_brake | speed, trig_dist, decel, end_speed, brake_t, max_brake |
| rear_hit | speed, closing_speed |
| cut_in | speed, trig_ttc |
| partial_lane_intrusion | speed, offset_fraction, max_lateral_acceleration |
| oncoming | speed, encroach |
| junction_cross | speed, trig_ttc, approach_distance_m (ADS-triggered variants only) |
| junction_turn | speed, trig_ttc |
| cross | speed, trig_dist |
| walk_along | speed |
| cruise | speed |
| junction_merge | speed |
| light_change_start | speed, trig_simtime |
| stopped_ahead / static_block / static_hold | （无行为参数，可携带下述通用位置参数） |

> 位置类通用 params（可选，任意 block 可带）：`gap`, `lateral`, `long`。

`lateral` 表示 NPC 中心距 ego 所在车道中心的横向距离，单位米，不能当成车身或后视镜间隙。
行人 `cross` 表示横穿初始道路：编译器从本次生成 XODR 读取实际车道及路肩宽度，缺省从近侧路缘外0.75米走到对侧路缘外0.75米后停止；0.75米是公开的几何初始化默认值，不是原文事实。原文明确的 `lateral` 保留。需有可容纳行人的已生成路肩/人行道；不支持的几何须拒绝，不能让行人无限走出地图。若原文仅部分横穿或中途另有动作，应报告 needs_extension，不能用完整横穿替代。 原文未说明起始侧时保留side=none，编译器默认从右侧横穿，并在C2XPedestrianCrossings中记录默认假设；不得把默认方向当作原文事实。
`cruise` 表示车辆或骑行者沿所在车道持续前进，无额外横向动作；被其他车辆绕行的直行骑行者不能替换为 `cut_in`。
`partial_lane_intrusion` 用于车辆 NPC 从相邻同向车道部分侵入 ego 车道；position=adjacent，side 是初始侧，relative_to=ego。目标偏移默认为初始车道宽的0.4倍（offset_fraction 必须在0与0.5之间），在启动1.5秒后按正弦目标偏移过渡并保持；max_lateral_acceleration 缺省0.8m/s²约束目标曲线，实际车辆响应另行测量。默认值不作为原文事实。编译器从本次XODR读取车道宽度，将计划保存在C2XPartialLaneIntrusions，用标准持续LaneOffsetAction表达，不执行完整LaneChangeAction，不指定碰撞。部分车身是否进入邻道需检查真实包围盒。
路侧直行骑行者可用relative_to指向所伴行车辆、position=roadside、block=cruise；缺省gap=0并保持lateral侧向偏移。原文明确的间距保留。若AV是需要此机动的NPC，可选另一辆有主动驾驶行为的道路车辆（包括货车）作为PCLA SUT；报告必须披露这种角色分配并保留原文车辆类别、前后关系和骑行者。
SUT 的 `straight` 表示沿道路行驶，在 `curve` 上即跟随道路弯曲；`left/right` 用于路口转入另一道路。
`junction_merge` 仅用于 `position=adjacent` 的车辆：双方从相邻同向车道同向转弯，NPC 路线与 ego 汇入同一出口车道。编译器必须从本次生成地图的 CARLA 路网中找到这两条路线，否则拒绝编译。它不包含强制碰撞或按个例指定的 XY 轨迹，也不表示转弯前的额外换道阶段。
只根据“侧镜擦碰”不能推导初始横向中心距离为 0.1 米；没有明确距离时省略，由编译器设置初始距离并在运行中检验接触情况。
路边停放汽车用 `kind=vehicle, position=roadside, behavior.block=static_hold`，保留车辆类别和与车道平行的默认朝向。
`kind=static` 只描述非车辆障碍。横穿行人/骑行者仍面向道路，不能把该默认朝向用于停放车辆。

### 车辆类别目录

`car` → Tesla Model 3；`suv` → Nissan Patrol；`pickup` → Cybertruck；`van` → Mercedes Sprinter；
`truck` → European HGV；`bus` → Mitsubishi Fuso Rosa；`ambulance` → Ford Ambulance。
这些是同类仿真代表车型，不声称逐一还原原报告的品牌、外形或传感器附件。
CARLA 使用相应蓝图的真实网格、碰撞形状和动力学；运行验收须核对 trace 的 `type_id`。
OpenSCENARIO 文件中的通用 BoundingBox 元数据不作为 CARLA 实际外形尺寸的测量证据。

## `scene.collision`

`{ "a": <id>, "b": <id> }`：碰撞参与者无序对。a、b ∈ {ego} ∪ {npc ids}，且 a ≠ b。

### `scene.collisions`：有序接触扩展

可选非空数组，每项仍为 `{a,b}`，列出原文发生的全部接触顺序；同一对可出现多次。每对内部 a/b 无序，不编码责任或冲量方向。为兼容现有调用，单数 `collision` 与数组第一对一致。I6 检查每一对的参与者引用和非自碰撞条件，不能只检查第一对。

链式碰撞可以表示为 `[{"a":"rear","b":"middle"},{"a":"middle","b":"ego"}]`，并保留全部车辆及初始行为。被撞后的运动由 CARLA 物理计算，数组本身不驱动演员、不生成冲量。编译器把完整列表作为 OpenSCENARIO 字符串参数 `C2XExpectedContactSequence` 保留；记录器看到多次接触期望时，继续记录后续过程，不在第一次 SUT 接触后提前结束。

运行检查以真实碰撞传感器事件的参与者和顺序为依据，不能用总碰撞计数或全部参与者的并集代替。接触间隔只用于区分传感器事件段；相同车辆的多次接触，还需要轨迹证明中间确实分离及完成原文动作。尚无行为原语表达的分离、再加速等阶段，仍须标为 `needs_extension`，不能只增加接触数组就视为已完整支持。

### `behavior.block=sequence`：有序动作序列

vehicle 可用 `behavior.steps` 明确至少两个步骤。每步有 `action`、`params`、`when`。允许动作：`drive`（参数speed/duration）、`brake`（end_speed/duration）、`match_speed`（duration及步骤顶层target）。三者编译为标准OSC速度动作；match_speed使用相对速度factor=1，不强制位置或接触。

`when.condition` 为 start/contact/after_previous/separated_and_target_stopped；仅第一步为start，后续步骤都等待前一步完成。contact需要when.target；separated_and_target_stopped需要目标停止且与NPC有净空，不能用经过固定秒数代替分离。when.delay是可选额外延迟。默认drive速度8m/s、brake终速0、动作时长1秒、净空2米、目标静止0.5秒；这些是编译器默认，不冒充报告测量。

纵向接触序列例：drive → 接触后match_speed → brake分离 → 目标停止且有净空后drive。SUT继续由PCLA自主控制；若其行为没有触发后续条件，应如实记录未发生的阶段。2026-09-18扩展：lane_change动作的params允许direction(left/right)、lanes(1..5整数，默认1)、distance(正米数，默认20)。首步必须drive，换道后续用after_previous；生成XOSC的RelativeLaneChangeAction以NPC为参照，沿新地图的相邻车道执行，停车带须在RoadSeed显式声明并生成为parking类型。不得省去跨越中间车道/停车带往返阶段，也不支持人为施加碰撞后的旋转。

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
> `supported` 必须覆盖完整事故过程。不能通过省略诱发避让的参与者、连续动作的前序阶段、后续碰撞，或把部分侵道替换成完整变道来获得通过。当前单 block / 单 collision 无法表达的关系应标明 `needs_extension`，供扩展框架后重新推理。

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
# Runtime initialization parameters

`sut.params.initial_speed_mps` optionally specifies the SUT's initial speed
in metres per second. It must be finite and non-negative. Use `0` when the
report participant is initially stopped. This initializes the vehicle only;
PCLA retains control during the episode. When omitted, the compiler retains
its legacy inferred takeover speed.

The storyboard Act starts on simulation time, independently of SUT travel.
This allows a moving NPC to approach a stationary SUT (for example, a stopped
AV being rear-ended). An ego-distance Act trigger would deadlock this case.

## ADS-relative stress variants (2026-09-19)

An optional `behavior.ads_trigger` replaces only the hazardous action's onset:

```json
{"block":"cut_in","params":{"speed":3,"long":24,"cut_dist":7.5},
 "ads_trigger":{"distance_m":3,"min_clearance_m":0.5,
                "min_ego_speed_mps":1,"min_npc_speed_mps":2}}
```

Supported blocks: `front_brake`, `cut_in`, `partial_lane_intrusion`, `cross`,
`junction_cross`, `junction_turn`. All predicates must hold **on the same tick**:
`min_clearance_m < body clearance to hero < distance_m`, ADS speed above its
minimum, and NPC speed above its minimum when required. There is no timer
fallback. Defaults are 0.5 m minimum clearance, 1 m/s ADS speed, and 2 m/s NPC
speed for lateral actions (zero for waiting crossing actors and braking).
`min_closing_speed_mps` optionally adds an ADS-minus-NPC **speed magnitude**
condition for same-direction lead/adjacent actors only; it is not a radial
closing speed or a general junction TTC. Do not apply it to opposing traffic.

Lateral actions require an NPC speed gate of at least 1 m/s. Waiting crossing
actors cannot require a positive NPC speed. Unknown keys, nonfinite values,
unsupported blocks, and simultaneous legacy `trig_*` parameters are rejected.
The compiler emits standard OpenSCENARIO entity conditions and records the
policy in `C2XADSTriggers`; `C2XADSActor=hero` declares the tested actor.
The CARLA runtime needs `ads_conditions.py` and its live-group hook: the stock
ScenarioRunner latches predicates independently and approximates side clearance
by subtracting vehicle half-lengths. Our tagged groups use actual projected
body polygons and simultaneous evaluation. Other conditions keep their behavior.

Junction routes are installed immediately; an explicit ADS gate holds the NPC
at its generated spawn before departure. This is an experimental waiting-start
variant, not a claim of conflict-point arrival synchronization or source fidelity.
For these variants, `junction_cross` and `junction_turn` optionally accept
`params.approach_distance_m` (finite, at least 5 m). The compiler trims only the
unused approach and chooses an existing sampled waypoint at least that far
before the junction entrance. Spawn, assigned route and controller route share
this anchor; connector and departure geometry stay unchanged. A missing or too
short sampled approach is rejected. The actual conflict may lie well beyond
the entrance, so a short fixed Cartesian trigger window can be unreachable.
`tools/ads_junction_timing.py` estimates the trigger from the common point of
the compiled routes and documented speed/acceleration assumptions; this still
requires an observed runtime onset and interaction before acceptance.
No scripted ADS trajectory, collision impulse, teleport after Init, or forced
ADS delay is added. The source SUT need not be the report's AV; identify the
tested role separately in publications.

Use `tools/ads_stress.py` to preserve the source and emit documented variants.
Its calibration option uses measured pre-hazard ADS speed to parameterize a
slower NPC and a nominal 2.5 s lane change. Requested braking is bounded by
`min(6, 0.8*mu*9.81)` m/s² (mu=1 when unspecified); actual vehicle dynamics and
whether the trigger was reached must be checked in runtime traces.
