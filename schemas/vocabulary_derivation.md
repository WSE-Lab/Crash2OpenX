# 词汇表演化协议(Vocabulary Derivation & Evolution Protocol)

本文件定义 `scene_seed` / `road_seed` 词汇表(position / block / topology 等枚举)的
**来源、推导、补充、冻结**规则。目的:回答 reviewer 的核心质疑——
「这些构件凭什么是这些?」「会不会是事后拼凑?」

配套文件:`schemas/nhtsa_mapping.md`(覆盖度映射表)。

---

## 0. 一句话协议

> 从 NHTSA 37 类(施加「可确定性执行 + 双方交互」两准则)**演绎**出初始词汇表
> → 跑语料,把 `skipped` 当**信号** → **离线、人工把关**地补词并**重新冻结**
> → 直到**饱和**;**运行时只做「映射到现有词汇 或 skip」,绝不造词。**

---

## 1. 设计同构:两轴 = NHTSA 两变量(非拍脑袋)

NHTSA 识别 37 类所用核心变量:**Movement Prior to Critical Event** + **Critical Event**
(辅以 Accident Type)。本词汇表的两轴与之同构:

| scene_seed 轴 | NHTSA 变量 |
|---|---|
| `position`(相对态势/起点几何) | Movement Prior to Critical Event |
| `behavior.block`(行为原语) | Critical Event |
| `collision (a,b)`(肇事/被撞) | Accident Type(有向化) |

→ 两轴分解是**沿用事故学标准的描述维度**,不是个人偏好。

---

## 2. 演绎起点(top-down):词汇表 = 标准的可执行子集

词汇表**不是**「设计出来的」,而是对 NHTSA 37 类施加两条**显式准则**筛得的子集:

- **准则 C1(可执行性)**:该场景能在 CARLA 中**确定性**编译执行(seed→OpenX→可跑)。
- **准则 C2(交互性)**:涉及**两个交互参与者**(ego + 至少一个 NPC/对象)。

由 C1/C2 推导:
- **position 取值** ← 各保留场景的「相对态势」归一化:
  ahead_same_lane / behind_same_lane / adjacent / oncoming / cross / opposing_leg / roadside
- **block 取值** ← 各保留场景的 critical event 算子化:
  front_brake(LVD) / stopped_ahead(LVS) / rear_hit / cut_in / oncoming / junction_cross / junction_turn /(VRU)cross / walk_along /(对象)static_block
- **被 C1/C2 排除的**:失控(#2/3)、倒车(#8/15)、停车(#17)、单车冲出(#6/7)、
  non-collision(#34)等。其中失控/倒车/停车**恰好是 NHTSA 自身剔除出 V2V 子集的类**。

→ 「覆盖 22/36 类」是**按构造得到**,不是巧合。

---

## 3. 归纳验证(bottom-up):语料证明没漏

- 对语料(split A)抽样做 **open coding**,让类目自然涌现,绘制**饱和曲线**。
- 与 §2 演绎得到的词汇表比对:
  - 一致 → 互证(标准背书 + 数据验证)。
  - 语料逼出 NHTSA 没有的构件(如更细的 VRU 行为)→ 记为**发现(finding)**,
    写成「真实事故语料要求比 NHTSA 更细的行为粒度」,是加分贡献而非任意选择。
- 报告**编码者间一致性(Cohen's κ)**。

---

## 4. 补充机制:离线语言演化,运行时禁造词

### 4.1 运行时(每份报告推理)——只允许两种结局
- ✅ 映射到**现有**词汇表;或
- ✅ 标 `status:"skipped"` + 记录 `reason`(成因)。
- ❌ **严禁** LLM 临时发明新 block/position。
  理由:(a) 新词无编译规则 → 破坏下游确定性;(b) 破坏「冻结词汇表」→ 覆盖率失效。

### 4.2 离线(批次之间)——受控补词循环
1. 汇总 `skipped` 报告,按 `reason` 聚类。
2. 若某成因**反复出现且成模式**(非长尾),才考虑升格为新构件。
3. 新构件必须**同时**满足 C1(给出 seed→OpenX 确定性编译规则)+ C2。
4. 更新元模型与 `nhtsa_mapping.md` → **重新冻结**词汇表版本号 → 全量重跑。
5. 一次性长尾、无法确定性编译的 → 维持 skip,计入「表达力边界」。

### 4.3 终止条件:饱和
- 持续补到「新报告几乎不再逼出新构件」(饱和曲线趋平)。
- **饱和曲线本身是论文证据**:词汇表对本领域够用且收敛。

### 4.4 v2.1 freeze 记录(2026-06-23 第二次受控补词)

按 §4.2 协议升格的两条新词,对应 schema 版本 v2 → **v2.1**(minor bump:词汇表新增,无破坏性变更):

| 新词 | 轴 | 触发 case | reason | C1 编译规则 | C2 双方交互 |
|---|---|---|---|---|---|
| `overtake_oncoming` | `sut.maneuver` 新值 | 008 (Waymo Mar 2025): bus 越双黄借对向道超 stopped Waymo,擦碰其左后侧。原 seed 被压成 stopped_ahead+adjacent,**丢"主动越线"critical event** | 对原 NHTSA #20 (Maneuver-Opposite-Direction) 的"做机动方"语义,既有 `oncoming` block(NPC 视角) 不能干净表达"ego 主动越线" | `osc_blocks.build_xosc`: 当 `sut.maneuver=overtake_oncoming` 时,`ego_goal_pos.lane_id` 翻号(-1→+1),让 ADS 测试是否能跨 broken centerline 到达对向道 goal | ✓ ego(主动越线方) + 静止 NPC,二者擦碰 |
| `light_change_start` | `behavior.block` 新值 | **无诊断 case 直接逼出**;为 LVA 子集(NHTSA #23 真缺口)备用,合成 LVA-after-green-light case 验证可命中 | NHTSA #23 (LVA) 是已知 ✗ 真缺口,且语料里"路口光变后前车启动"是常见小事故 critical event | `osc_blocks.block_events`: NPC 在 t=0 hold 速度 0,在 t=trig_simtime(默认 2 s) 起步到 cruise speed | ✓ 同向前车 + 后车 ego,后车追尾 |

**WF7(新)**:`sut.maneuver=overtake_oncoming ⇒ road.lanes.backward ≥ 1 AND road.center_line=broken`。
代码来源:lane_id 翻号需要对向车道存在;broken centerline 是合法跨界前提。
形式化锚点:见 `road_topology_derivation.md §5` WF7 行(已 docs-enforced;code-enforce 待下一轮 R2/R5 的整体 WF gate 一并)。

**坦白(防 reviewer 拍 light_change_start 是事后拼凑)**:
- 本批 5 个 supported case (001/002/004/008 + 一个 skipped) 中,002 的"光变 + cyclist 加速冲入"在 R3 (params 抽取) 加持下被 cut_in + control=traffic_light + trig_ttc=1.5 + speed=6 完整表达,**不需要 light_change_start**。
- light_change_start 的真验证 case 是合成的"LVA-after-green-light",非真实语料逼出。属**前瞻性补词**(为 split B 中可能遇到的纯 LVA 启动场景备用)。
- 处置:**先入 schema、不删**,但在 §6 反例里增加一行"前瞻性补词需在 split B 实测验证;若饱和曲线显示该 block 长期零命中,下一版本删除"。
- 对照 §4.2 准则:`overtake_oncoming` 完整满足"反复出现且成模式"+"C1+C2"两条;`light_change_start` 仅满足 C1+C2,模式性需要 split B 兑现。

**版本冻结**:`schemas/scene_seed_schema_v2.md` 标题保持文件名 v2,内容标 v2.1;`api_infer_scene_seed_v2.py` 与 `osc_blocks.py` 的 BLOCKS_BY_KIND/BLOCK_PARAMS/SUT_MANEUVERS/build_xosc 已同步;`nhtsa_mapping.md` 第 1 节 #20 ~→✓、#23 ✗→~、统计行已更新。

### 4.5 v2.2 freeze 记录(2026-06-23 第三次受控扩展 — 新轴而非新词)

新增**顶层 `scene.environment` 块**(weather + time_of_day + 可选 friction_scale),对应 schema 版本 v2.1 → **v2.2**(新轴 minor bump)。

| 字段 | 受控词汇表 | 触发理由 | C1 编译规则 | 来源效度 |
|---|---|---|---|---|
| `weather` | clear/rain/snow/fog/cloudy/unknown | NHTSA / DMV crash 报告约 30% 提到非晴天("rainy"/"foggy"/"snow-covered"等),原 schema 全部压成默认 sun 失真 | `osc_blocks._env_for_compile` 透传给 `stage4_xosc._build_env_action` 的子串匹配("rain" → CloudState.rainy + Precipitation rain 0.8) | 受控集 6 值,LLM 不能造词;映射规则在 prompt 中显式给出 |
| `time_of_day` | dawn/morning/afternoon/evening/dusk/night/unknown | 夜间事故约 25%,原 schema 全部压成正午,太阳位置错 | `_env_for_compile` 把定性词翻译成 ISO datetime("night"→`2024-06-15T22:00:00`),`_build_env_action` 据此推 sun azimuth/elevation;`night` 时太阳 elevation=-0.3(地平线下) | 同上;ISO 翻译表见 `osc_blocks._TIME_QUAL_TO_ISO` |
| `friction_scale` | float 0.1..1.0(可选) | 湿滑/结冰路面对 collision 物理影响显著;原 schema 默认 1.0 干燥 | `RoadCondition.frictionScaleFactor`,直接由 OpenSCENARIO 解释 | 仅在报告明确说湿/雪/冰/油 时填(wet=0.5/snow=0.3/oily=0.4),否则不填走默认 |

**为什么是新轴而不是新词**:weather 和 time_of_day 都不是 NHTSA 37 类的分类轴,也不与 scene 的 position × behavior.block 同构;它们是**正交于动态语义的环境上下文**。本协议 §1(设计同构)只要求两轴(position × block)沿用 NHTSA 变量;environment 是**编译目标(OpenSCENARIO)原生支持的环境层**,并非新分类轴。因此走"新轴 minor bump"而非"新词受控补"流程。

**WF8 候选(待形式化)**:`scene.environment.weather=snow ⇒ road.lane_width 应放宽 + scene.params 的 trig_dist 应延长(冰雪路面制动距离)`。当前未 code-enforce,记入 backlog。

**端到端验证(2026-06-23)**:合成 case "At approximately 22:30 on a rainy night with wet pavement, ... slammed on its brakes from 30 ft ahead":
- LLM 抽出 `environment={weather:rain, time_of_day:night, friction_scale:0.5}`
- osc_blocks 编译出 `EnvironmentAction` 含 `cloudState=rainy`、`dateTime=2024-06-15T22:00:00`、`Sun elevation=-0.3`、`Precipitation rain 0.8`、`frictionScaleFactor=0.5`
- XSD-valid
- evidence.reason 显式列出推导链 — 可审查性继承 R3。

**版本冻结**:`api_infer_scene_seed_v2.py` 加 `WEATHERS`/`TIMES_OF_DAY` 受控集 + normalize() 加 environment 输出 + prompt 加抽取规则;`osc_blocks.py` 加 `_env_for_compile` + `_TIME_QUAL_TO_ISO` 转换;`scene_seed_schema_v2.md` 加 §scene.environment 段;之前所有有效 scene_seed.json (无 environment 字段) 在重抽时会得到 `weather:unknown, time_of_day:unknown` 默认。

---

## 5. 来源效度红线(防过拟合)

- **语料切分**:导出+补词只在 **split A**;最终覆盖率报在从未参与补词的 **split B(hold-out)**。
- 严禁用 hold-out 反复补词(= 自己喂自己,循环论证)。
- 词汇表**版本冻结**:每次补词产生新版本号;论文报告的覆盖率必须标注所用词汇表版本。

---

## 6. 反例(踩坑写法,论文里要避免)

| ❌ 凭空 / 事后 | ✅ 推导 / 验证 |
|---|---|
| 「我设计了这 7 个 block」 | 「对 NHTSA 37 类施加 C1/C2 得到的子集」 |
| 「跑完发现覆盖了 22 类」(post-hoc) | 「覆盖 22 类是按 C1/C2 构造的结果,并经 split B 实测」 |
| 运行时 LLM 自由造词 | 运行时只映射或 skip;补词为离线受控演化 |
| 同一批语料既导出又评估 | split A 导出/补词,split B 评估 |
| skip 当失败藏起来 | skip 是表达力边界 + 补词信号,公开统计 |

---

## 来源

- Najm, W.G., Smith, J.D., Yanagisawa, M. (2007). *Pre-Crash Scenario Typology for Crash
  Avoidance Research.* DOT HS 810 767, NHTSA / Volpe Center.(37 类 + GES 变量)
- DOT HS 811 731 (2013) *Description of Light-Vehicle Pre-Crash Scenarios.*
- 方法论:grounded theory / open coding;DSL 迭代语言工程与饱和。
