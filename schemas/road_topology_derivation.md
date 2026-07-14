# Road 元模型推导协议（Road Metamodel Derivation Protocol）

本文件定义 `road_seed` 四个字段（`topology` / `type` / `lanes` / `center_line`）的**来源、推导、补充、冻结**规则。
与 `schemas/vocabulary_derivation.md`（scene 词汇表）平级，回答 reviewer 对 road 维度的同一质疑——
「这些道路类型凭什么是这些?」「会不会是随手一想?」「能涵盖语料吗?」

配套文件：`schemas/vocabulary_derivation.md`（scene 两轴）、`schemas/nhtsa_mapping.md`（覆盖度）、
`schemas/road_seed_schema.md`（字段定义）。

---

## 0. 一句话协议

> road 维度**不与 NHTSA 同构**（道路不是事故类型学的分类轴），改用**双层依据**：
> 取值**下界**由已论证的 scene 词汇表的几何承载需求**演绎决定**（逻辑必要，非品味）；
> 取值的**切分与命名**锚定 **OpenDRIVE 几何模型 + 交通工程交叉口分类**（外部权威，非自创）。
> 当前枚举 = 两层交集：**既不能少**（少则有 block 无处承载）**也不多**（每个值都有承载证据 + 标准出处）。
> `type`/`lanes`/`center_line` 不是分类轴，是 **model-to-text 的实例化参数**，依据来自目标语言（OpenDRIVE）与 scene 承载需求。
> 第三重保证：**归纳覆盖**——枚举对 split A 语料的实际覆盖率 + 饱和曲线 + 坦白的表达力边界（§7），证明「能涵盖」而非自洽空想。

---

## 1. 为什么 road 不能照搬 NHTSA 同构论证

`scene_seed` 的 `position` / `behavior.block` 能锚 NHTSA，是因为它们**就是** NHTSA 类型学的两个核心变量
（Movement Prior to Critical Event / Critical Event，见 `vocabulary_derivation.md` §1）。

道路几何**不是** NHTSA 的主分类轴——它是事故发生的**几何容器**。若硬把 road 也写成「NHTSA 演绎子集」，
恰恰会被 reviewer 识别为事后拼凑。因此 road 需要一套**性质不同**的论证。

并且 `road_seed` 三字段**性质不同，必须分开证**：

| 字段 | 性质 | 论证方式 |
|---|---|---|
| `topology` | 几何拓扑 | 双层依据（§2–§3）：演绎下界 + 外部标准上界 |
| `type` | OpenDRIVE 功能道路分级 | 目标语言锚 + 降级为参数（§4.1） |
| `lanes` | 方向感知车道数 `{forward, backward}` | scene 反推下界 + 工程区间，归参数（§4.2） |
| `center_line` | 中心线虚实 solid/broken | scene 反推（绕行可行性）+ OpenDRIVE roadMark 锚，归参数（§4.3） |

---

## 2. topology 的双层依据

### 2.1 第一层（演绎下界）——决定「必须有哪些」，逻辑必要而非品味

由已论证的 scene 词汇表**反推** topology 的**必要集合**：不是「我想要交叉口」，而是
「为承载 `junction_cross` 这个已论证 block，几何上**必须**存在一个交叉口拓扑，否则该 block 无处落地」。

形式化为 `position/block → required_geometry` 映射。该映射**不是新编的**，全部来自现有编译器代码：

- `osc_blocks.resolve_position()`（tools/osc_blocks.py:186-204）：每个 position 解析成的几何放置。
- `osc_blocks.block_events()`（tools/osc_blocks.py:247-272）：每个 block 装配成的事件 / 动作。
- `JUNCTION_POSITIONS = {cross, opposing_leg}`、`JUNCTION_BLOCKS = {junction_cross, junction_turn}`
  （tools/osc_blocks.py:97-98）+ `scene_needs_junction()`（:129-133）：哪些 scene 构件**强制要求**路口拓扑 + RoadGraph。

**检验标准（防「随手多塞」）：无孤儿 topology。** 任何找不到对应承载 block/position 的拓扑，要么删除，
要么补出它独有承载的构件——否则它就是「随手一想」。本检验已据此删除孤儿 `left_turn` / `right_turn`（§6）。

### 2.2 第二层（外部标准上界）——决定「切分与命名是否标准认可」

topology 的**具体切分粒度与命名**（为什么是 straight/curve，为什么 junction 分 cross/t/y）锚到外部权威，
证明这些是标准范畴而非自创：

- **OpenDRIVE 1.5**：`junction` 是其一等结构概念；几何 primitive（line / arc / spiral）直接决定
  `straight` / `curve` 的合法性。本项目 topology = OpenDRIVE 几何元素的**高层模板**
  （编译落点见 `road_seed_to_trace()` 的 layout 映射，build_road_seed_opendrive.py:106-135）。
- **交通工程**：交叉口按进路数分 3-leg（T / Y）/ 4-leg（cross）/ roundabout，是道路设计标准分类。
- **PEGASUS 6-layer / ISO 34502**（可选背书）：Layer 1 = road network/geometry，是场景描述的标准几何层。

---

## 3. topology 溯源表（single source of truth，代码可追溯）

> 每行的「第一层」列引用 tools/osc_blocks.py 的真实处理；「编译」列引用
> tools/build_road_seed_opendrive.py 的真实落点。状态：✅ 可确定性编译 · ⏳ 需 RoadGraph（option B）· ❌ 未实现

| topology | 第一层：承载的 position / block（演绎依据 + 代码） | 第二层：外部标准 | 编译 |
|---|---|---|---|
| `straight` | ahead/behind_same_lane, oncoming, roadside（resolve_position:186-201）；front_brake / rear_hit / oncoming / stopped_ahead / cross / walk_along / static_block（block_events:252-269） | OpenDRIVE line geom | ✅ `straight_corridor`（:101） |
| `curve` | 同 `straight`（弯道实例化，translation-invariant 不变） | OpenDRIVE arc / spiral | ✅ `curved_corridor`（:103） |
| `cross_intersection` | cross / opposing_leg（resolve_position:202-203 → BlockUnsupported 需 RoadGraph）；junction_cross / junction_turn（block_events:270-271）；scene_needs_junction:129 | OpenDRIVE junction + 4-leg | ✅ `common_cross_junction`（:99-100），运行时需 roadgraph_map_cache |
| `t_junction` | junction_turn / junction_cross（同上 JUNCTION_BLOCKS） | OpenDRIVE junction + 3-leg T | ✅ `t_junction`（:107-108） |
| `y_junction` | junction_turn | OpenDRIVE junction + 3-leg Y | ✅ `y_junction`（:109-110） |
| `merge` | ✅ 语料归纳逼出 **16/649**（高速汇流冲突，§7）；第一层 scene 承载（强制 ramp 的 block/position）仍待补，属下一轮 | OpenDRIVE direct junction + 匝道 | ✅ `ramp_merge`（DirectJunctionCreator，:132） |
| `fork` | ✅ 语料逼出 **1/649**（匝道分流 / off-ramp）；第一层承载待补 | OpenDRIVE direct junction + 匝道 | ✅ `ramp_fork`（DirectJunctionCreator，:134） |
| `roundabout` | 语料逼出 2/649，但**工具链不支持** | 环岛标准 / JunctionGroup(roundabout) | 🚫 已移出枚举：scenariogeneration 无环岛生成器（仅有 JunctionGroup 数据结构），真环岛需手搓闭环 road + 多 junction，CARLA 导入易碎。决策为表达力边界，环岛场景返回 `unsupported` |

> **`left_turn` / `right_turn` 已删除（§6 已消解）**：转弯是机动/路由行为，不是道路几何。路口左/右转由
> `cross_intersection` / `t_junction` / `y_junction`（隐含全部转向 connector）+ `scene_seed.sut.maneuver ∈ {left,right}`
> 在路由层选定（osc_blocks.py:286-292）；非路口的独立急转归 `curve`。两者均无独有承载构件，按「无孤儿」检验删除。

**lanes / center_line 维度（非 topology，但属同一第一层演绎）**：v2 起 `lanes` 是方向感知对象 `{forward, backward}`。
- `adjacent` position（RelativeLanePosition ±1，osc_blocks.py:195）与 `cut_in` block（RelativeLaneChangeAction，:262）
  **强制要求同向 ≥2 车道** → `forward ≥ 2`，即跨模型约束 **WF5**（§5）的代码来源。
- `oncoming`（RelativeLanePosition 2 + h=π，:197）**要求存在对向车道** → `backward ≥ 1`，即 **WF6**（§5）。
- **单向道路**（`backward = 0`）由语料归纳逼出（§7：8/64 unsupported 是 SF 单行道），旧 schema 的「lanes=1 即双向」
  假设表达不了，强行套双向会**凭空生成不存在的对向车流**，损伤 construct validity——这是 v1→v2 升级的直接依据。
- `center_line`（solid/broken）由「障碍物绕行可行性」反推：scene 若需借对向道绕行，几何上必须 `center_line=broken`
  且 `backward ≥ 1`；否则编译出的道路无旁路、车辆卡死（语料动机，§4.3 / §7）。

---

## 4. type / lanes：降级为实例化参数（不是分类轴）

### 4.1 `type`（town / lowSpeed / rural / motorway / townArterial / townCollector / townLocal）

- **几何闭包反推不出它**：scene 词汇表不要求「必须有 motorway vs town」。
- **唯一硬依据 = OpenDRIVE e_roadType 枚举**：白名单 `ALLOWED_ROAD_TYPES`（build_road_seed_opendrive.py:34-44）
  是 OpenDRIVE 1.5 road type 的子集，纯**输出兼容性**锚定。
- **编译里它就是一张参数查找表**：`ROAD_TYPE_DEFAULTS`（:50-58）把每个 type 映射为
  `{lane_width_m, road_length_m, junction_radius_m, speed_mps}` 四个几何/限速参数，**不参与任何拓扑或事故语义选择**
  （`patch_road_type` 仅写入 XODR 的 `<type>`/`<speed>`，:162+）。
- **结论 → 定位为 variability / 实例化参数**，不与 `topology` 并列为分类轴。这种「主动降级」本身是
  construct validity 的加分项，也契合 FRAMEWORK_ROADMAP §2.6 的极简性红线。

### 4.2 `lanes`（v2：`{forward: 1..5, backward: 0..5}`，相对 ego 行驶方向）

- **下界由 scene 反推**（第一层演绎，见 §3）：cut_in / adjacent ⇒ `forward ≥ 2`（WF5）；oncoming ⇒ `backward ≥ 1`（WF6）。
- **单向（`backward = 0`）** 不是工程区间任选，而是**语料逼出的必要表达**（§7：单行道 8 例）；反过来，
  无对向车道时 oncoming 类 block 必须被禁（下一轮 WF6 下游强制）。
- **上限 5** 是编译可实现 + 常见道路设计的工程区间（`ALLOWED_FORWARD_LANES` / `ALLOWED_BACKWARD_LANES`，:46-47）——属参数区间。
- **结论 → 归 variability / 参数**，下界受 scene 约束，区间受工程约束；方向感知本身由语料覆盖证据驱动（非凭空细化）。

### 4.3 `center_line`（v2 新增：`solid` / `broken`）

- **几何闭包/topology 反推不出它**，但 **scene 可行性反推得出**：当 scene 需要「借对向道绕过障碍物」这类机动，
  几何上必须满足 `center_line = broken`（可跨）且 `backward ≥ 1`（有对向道）。否则编译出的道路无旁路、
  车辆只能停在障碍物前——**这是 v2 新增该字段的直接动机**（语料中障碍物场景被退化推成单向后卡死，§7）。
- **外部标准锚**：OpenDRIVE `<roadMark>` 的 `type`（solid / broken）是标准车道线属性，本字段是其高层二元抽象。
- **结论 → 同样归 variability / 参数**：取值二元、不增分类轴；下界（何时必须 broken）由 scene 机动可行性约束。
  事实不明默认 `broken`（保守保留绕行空间）。

---

## 5. 跨模型约束（WF4 / WF5）= 第一层依据的形式化

第一层演绎不是散文，已是（或应是）可声明、可校验的跨元模型约束（见 FRAMEWORK_ROADMAP §2.2）：

- **WF4**（junction 类 block/position ⇒ topology ∈ {cross_intersection, t_junction, y_junction}）
  代码来源：`scene_needs_junction()` + `JUNCTION_POSITIONS/BLOCKS`（osc_blocks.py:97-98, 129）；
  缺 roadgraph 即 `BlockUnsupported`。
- **WF5**（cut_in / adjacent ⇒ `lanes.forward ≥ 2`）
  代码来源：RelativeLanePosition(±1)（:195）/ RelativeLaneChangeAction（:262）。✅ 已 code-enforced。
- **WF6**（oncoming ⇒ `lanes.backward ≥ 1`；等价地：`backward = 0` 单向路上禁用 oncoming 类 block）
  代码来源：RelativeLanePosition(2, h=π)（:197）。✅ **已 code-enforced**（`osc_blocks._check_wf`，2026-06-23）：
  build_xosc 收 road_seed kwarg 时，oncoming 块/位置 + backward=0 → raise WFViolation。
- **WF7**（v2.1 新增）：`sut.maneuver=overtake_oncoming ⇒ lanes.backward ≥ 1 AND center_line=broken`
  代码来源：早先曾在 build_xosc 翻 ego goal 车道，2026-06-23 因 PCLA 把"对向道远端 goal"读成 U-turn 而回退。
  改由 `osc_blocks._check_wf` 在 build_xosc 入口检查并 raise WFViolation；overtake_oncoming 退化为
  schema-级注解 + WF7 形式化保证。✅ **已 code-enforced**。
  推导依据：见 `vocabulary_derivation.md §4.4` v2.1 freeze 记录与 008 触发 case；以及 §4.6（如有）的 lessons-learned。

→ 这四条把「road 取值由 scene 需求反推」从口头论证落成**可执行的一致性 gate**，是论文「跨元模型良构性约束」贡献的直接证据。
   现状：WF4 ✅ code-enforced（junction routing），WF5 ✅ code-enforced（RelativeLane），WF6 ✅ code-enforced（v2.2），WF7 ✅ code-enforced（v2.2）。

---

## 6. 审查发现的不一致（schema ↔ code）——已消解

construct validity 审查（本协议「无孤儿 topology」检验）曾暴露以下 schema 与代码不一致，现已按协议消解：

1. **`right_turn`（已删除）**：曾在 `ALLOWED_TOPOLOGIES` 与编译内（编译为 `orthogonal_turn_right`），但
   `road_seed_schema.md` 枚举无此项，且与 `sut.maneuver=right` 概念重叠（道路几何 vs ego 动作）。
   裁定：转弯归机动/路由层，删除该拓扑。
2. **`left_turn`（已删除）**：曾在白名单内但编译走 `NotImplementedError`，无任何承载构件——典型孤儿，按本协议删除。
3. **schema「已实现」清单**：`road_seed_schema.md` §「下游可实现性」已同步为 straight/curve/cross/t/y（不含已删除的 turn）。

→ 处理依据 `vocabulary_derivation.md` §4：**离线受控修正 → 更新 schema/本表 → 重新冻结版本号 → 全量重跑**。
本次修正已落地编译器（build_road_seed_opendrive.py）与两份 schema；版本冻结与全量重跑按 §7 执行。

---

## 7. 归纳验证与来源效度（沿用 vocabulary 协议）

- **归纳（bottom-up）**：对语料（split A）统计 topology 的实际分布 + 饱和曲线；与 §3 演绎集合比对。
  语料若反复逼出某拓扑且成模式 → 升格为补编译目标（finding）。已据此实现 `merge`(16/649)/`fork`(1/649)；
  `roundabout`(2/649) 虽被逼出但因工具链不支持，定为表达力边界（§3）。
- **运行时禁造词**：road agent 只能映射到现有 topology 或返回 `status:"unsupported"`
  （api_infer_road_seed.py 已如此），严禁临时发明拓扑。
- **来源效度红线**：topology 集合的导出/修正只在 split A；最终覆盖率报在 hold-out 的 split B。
  词汇表/拓扑表**版本冻结**，论文报告的覆盖率须标注所用版本。

### 7.1 归纳覆盖证据（split A 实测，n=713，回答「能涵盖吗」）

> 统计口径：扫 `outputs/road_seed/*.json` 的 `topology` 与 `unsupported_topology` 字段。
> 注：此快照在 v1 schema 下产出，`roundabout`(2) 当时记为 supported，现已重分类为表达力边界（§3）。

**覆盖率**：supported **649 / 713 = 91.0%**，unsupported 64（9.0%）。

**topology 实际分布（supported）**：

| topology | 计数 | 占比 |
|---|---|---|
| cross_intersection | 346 | 53.3% |
| straight | 262 | 40.4% |
| merge | 16 | 2.5% |
| curve | 10 | 1.5% |
| t_junction | 10 | 1.5% |
| y_junction | 2 | 0.3% |
| roundabout（→ 已转边界） | 2 | 0.3% |
| fork | 1 | 0.2% |

**饱和曲线**（各 topology 在语料中首次出现的 case 序号）：cross@1 · straight@4 · curve@18 · y@71 · t@76 · merge@155 · fork@644。
→ 主拓扑在前 ~160 例即饱和；长尾（fork@644）说明枚举不是凭空扩张，而是被语料逐步逼出。

**unsupported 64 条分类（= 坦白的表达力边界 + 改进驱动）**：

| 类别 | 约计 | 处置 |
|---|---|---|
| 停车场 / driveway / 车库接入 | ~45 | **表达力边界**：非公共道路几何，坦白排除（§8 不强塞） |
| 单向 / 方向不对称车道 | ~9 | **驱动 v1→v2 升级**：`lanes.{forward,backward}` + 单向 `backward=0`（§4.2） |
| 其它（six-way、cul-de-sac、slip lane、onramp 等长尾） | ~10 | 暂留 unsupported，未成模式 |

（`roundabout` 不在此 64 条内——它在 v1 快照里记为 supported(2)，现重分类为表达力边界，见上表与 §3。）

**三段论闭合**：① 不是凭空——每个 topology 有承载证据（§3 第一层）+ 标准出处（§2.2 第二层）；
② 有依据——`lanes`/`center_line` 的方向感知由语料 unsupported 反推逼出（§4.2/§4.3），非品味细化；
③ 能涵盖——公共道路几何覆盖率 91%，边界（停车场/环岛）坦白排除并计入分母，覆盖率随版本冻结可复核。

---

## 8. 反例（论文里要避免）

| ❌ 凭空 / 事后 | ✅ 双层推导 / 验证 |
|---|---|
| 「我选了这几种道路类型」 | 「下界由 scene 承载需求反推（无孤儿），切分锚 OpenDRIVE + 交通工程」 |
| 把 `type` 当成与 topology 并列的事故分类轴 | 坦白 `type` 是 OpenDRIVE 实例化参数（ROAD_TYPE_DEFAULTS 查找表），主动降级 |
| 保留 `left_turn` 这种「白名单里有但不能编译也无承载」的孤儿 | 无孤儿 topology 检验：删除或补全 |
| road 与 scene 各自独立拍枚举 | road 取值经 WF4/WF5/WF6 由 scene 反推，一致性 gate 形式化 |
| 「枚举自洽就行」/ 不报覆盖 | 报 split A 实测覆盖率 91%（§7.1）+ 饱和曲线 + 坦白边界（停车场/环岛计入分母） |
| 为「几何完整」加 `center_line`/单向 | 由 unsupported 语料（单行道、障碍物卡死）反推逼出，非凭空细化（§4.2/§4.3） |

---

## 来源

- OpenDRIVE 1.5 / 1.5M Format Specification（road `type`、`junction` 结构、line/arc/spiral 几何）。
- AASHTO, *A Policy on Geometric Design of Highways and Streets*（交叉口 3-leg/4-leg/roundabout 分类）。
- ISO 34502 场景分类；PEGASUS 6-layer model（Layer 1 road network）（可选背书）。
- 配套：`schemas/vocabulary_derivation.md`、`schemas/nhtsa_mapping.md`、`schemas/road_seed_schema.md`。
