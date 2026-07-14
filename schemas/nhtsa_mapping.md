# Scene Seed 词汇表 ↔ NHTSA 37 类 pre-crash 场景映射

本文件把当前 `scene_seed`(block × position × side)+ `road_seed`(topology)的词汇表,
映射到 NHTSA pre-crash scenario typology(DOT HS 810 767, Najm et al. 2007)的 37 类。
用途:论文里 DSL「代表性 / 覆盖度 / 表达力边界」的 single source of truth。

> ⚠️ 现状声明:下表为**人工映射**(基于 schema 文本判定),尚未在语料上实测,
> 也尚未做编码者间一致性。正式投稿前需:(1) 双人独立映射 + 报告 κ;
> (2) 在 hold-out 报告集上实测覆盖率(见文末「来源效度」)。

状态图例:✓ 完全 · ~ 部分 · ✗ 真缺口 · ⊘ 主动排除(可辩护) · – 出范围 · n/a

---

## 1. 全 37 类覆盖表

| # | NHTSA 场景 | 状态 | 映射到的构件 / 排除理由 |
|---|---|---|---|
| 1 | Vehicle Failure | ✗ | 无车辆故障语义(机械故障非 ADS 决策对象) |
| 2 | Control Loss — with prior action | ⊘ | ego 永远干净执行机动;NHTSA 自身划为非 V2V |
| 3 | Control Loss — without prior action | ⊘ | 同上 |
| 4 | Running Red Light | ~ | `control=traffic_light` + junction block(NPC 闯灯) |
| 5 | Running Stop Sign | ~ | `control=stop_sign` + junction block |
| 6 | Road Edge Departure — with maneuver | ✗ | 单车冲出路缘,破「双方碰撞」前提 |
| 7 | Road Edge Departure — without maneuver | ✗ | 同上 |
| 8 | Road Edge Departure — while backing | ⊘ | 倒车,已明确 skip |
| 9 | Animal — with maneuver | ✗ | 无 `animal` kind |
| 10 | Animal — without maneuver | ✗ | 同上 |
| 11 | Pedestrian Crash — with maneuver | ✓ | pedestrian `cross` / `walk_along` |
| 12 | Pedestrian Crash — without maneuver | ✓ | pedestrian `cross` / `walk_along` |
| 13 | Pedalcyclist Crash — with maneuver | ✓ | cyclist `cut_in/oncoming/junction_cross/cross` |
| 14 | Pedalcyclist Crash — without maneuver | ✓ | cyclist blocks |
| 15 | Backing Up Into Another Vehicle | ⊘ | 倒车,已明确 skip |
| 16 | Vehicle(s) Turning — Same Direction | ~ | `junction_turn` 近似(弱) |
| 17 | Vehicle(s) Parking — Same Direction | ⊘ | 停车场,已明确 skip |
| 18 | Changing Lanes — Same Direction | ✓ | `cut_in` (adjacent) |
| 19 | Drifting — Same Direction | ~ | `cut_in` 近似(drift ≠ 主动并线) |
| 20 | Maneuver — Opposite Direction | ✓ | `sut.maneuver=overtake_oncoming`(v2.1) ego 跨对向道超车，本身就是 #20 的"做机动方"语义 |
| 21 | Not Making Maneuver — Opposite Direction | ✓ | `oncoming`(正面对撞) |
| 22 | Following Vehicle Making a Maneuver | ~ | `rear_hit` (behind_same_lane) |
| 23 | Lead Vehicle Accelerating (LVA) | ~ | v2.1 `light_change_start` 覆盖路口光变后启动子集；通用 LVA（连续加速）仍缺，标 ~ |
| 24 | Lead Vehicle Moving — lower constant speed (LVM) | ~ | `front_brake` 近似(实为减速非匀速慢行) |
| 25 | Lead Vehicle Decelerating (LVD) | ✓ | `front_brake` (ahead_same_lane) |
| 26 | Lead Vehicle Stopped (LVS) | ✓ | `stopped_ahead` (ahead_same_lane) |
| 27 | LTAP/OD — Signalized Junctions | ✓ | `junction_turn` + ego left + `control` |
| 28 | Turning Right — Signalized Junctions | ✓ | `junction_turn` + ego right |
| 29 | LTAP/OD — Non-Signalized Junctions | ✓ | `junction_turn` + ego left |
| 30 | Straight Crossing Paths (SCP) — Non-Signalized | ✓ | `junction_cross` (cross / opposing_leg) |
| 31 | Turning — Non-Signalized Junctions | ✓ | `junction_turn` |
| 32 | Evasive Action — with maneuver | ✗ | 无规避机动语义(NHTSA 自身亦常排除) |
| 33 | Evasive Action — without maneuver | ✗ | 同上 |
| 34 | Non-Collision Incident | – | 模型要求必有 collision,出范围 |
| 35 | Object Crash — with maneuver | ✓ | `static_block` |
| 36 | Object Crash — without maneuver | ✓ | `static_block` |
| 37 | Other | n/a | — |

---

## 2. 统计

| 状态 | 数量 | 类号 |
|---|---|---|
| ✓ 完全覆盖 | 16 | 11,12,13,14,18,20,21,25,26,27,28,29,30,31,35,36 |
| ~ 部分覆盖 | 7 | 4,5,16,19,22,23,24 |
| ✗ 真缺口 | 7 | 1,6,7,9,10,32,33 |
| ⊘ 主动排除(可辩护) | 5 | 2,3,8,15,17 |
| – 出范围 | 1 | 34 |
| n/a | 1 | 37 |

- **有覆盖(完全+部分)= 23 / 36 类**(#37 Other 不计)。
- v2.1 补词后：#20 (Maneuver-OD) ~→✓ 因 `overtake_oncoming`；#23 (LVA) ✗→~ 因 `light_change_start` 覆盖光变启动子集。
- **主动排除的 5 类(#2/3 失控、#8/15 倒车、#17 停车)恰好落在 NHTSA 主动剔除出 V2V 子集的那批**(NHTSA 将其划给「车端自治系统更合适」)。NHTSA V2V 适用子集为 22 类,与本词汇表覆盖数高度吻合 —— 论文最强辩护点。

---

## 3. 真缺口(按补齐价值排序)

1. **#23 LVA / #24 LVM** —— 追尾族是最高频组,现仅有 LVD(`front_brake`)+ LVS(`stopped_ahead`),
   缺「前车加速 / 匀速慢行」。族内 under-resolve。补 `lead_slow` / `lead_accel` block 成本低、回报高。
2. **#6/#7 Road Edge Departure** —— 单车冲出路缘,占比不低,但需突破「双方碰撞」模型前提,需权衡。
3. **#9/#10 Animal** —— 加 `animal` kind 即可,低成本。
4. **#32/#33 Evasive Action / #1 Vehicle Failure** —— 低频,NHTSA 自身亦常排除,优先级最低。

---

## 4. 超出 37 类的表达力(论文要主动讲)

1. **VRU 行为分辨率高于 NHTSA**:NHTSA 把行人/骑行各压成 2 类(with/without prior maneuver);
   本词汇表给行人 2 block、骑行 4 block。VRU 上比标准更细。
2. **生成式组合空间 ≫ 37**:词汇表是 `block × position × topology × side × params` 的笛卡尔积;
   同一 `front_brake` 在 straight / curve / 路口入口是不同可执行实例。
   **37 类是该空间的一个粗商(coarse quotient)**,可实例化数量远多于 37。
3. **显式 side 横向性**(left/right):NHTSA 不枚举。
4. **二元有向碰撞对 `(a,b)` + 肇事/被撞角色**:NHTSA 为「主车视角」单边描述,本模型为二元有向,语义更明确。

---

## 5. 词汇表的来源效度(provenance / construct validity)

> 解决「这些定义不能凭空产生」的核心。**纯 post-hoc 映射 ≠ 推导**,reviewer 会识别为事后拼凑。
> 需要做成「演绎 + 归纳」双向三角验证:

- **演绎(top-down)**:把词汇表重述为「NHTSA 37 类(+ ISO 34502 / EURO NCAP 可选)中
  **可确定性执行的交互式子集**」。排除准则显式化:倒车/停车/失控 = NHTSA 非 V2V 集;
  单车冲出 = 无第二参与者;non-collision = 无碰撞目标。
  → 这样「覆盖」是**按构造得到**的,不是巧合。
- **归纳(bottom-up)**:对语料抽样做 open coding,让类目自然涌现,测饱和曲线;
  与演绎得到的词汇表比对。若语料逼出 NHTSA 没有的构件(如更细的 VRU 行为),
  那是一个**发现(finding)**,不是任意选择。
- **来源效度红线(防过拟合)**:导出词汇表的报告子集 与 评估覆盖率的报告集 **必须切分**
  (hold-out)。否则同一批既导出又评估 = 循环论证。

---

## 来源

- Najm, W.G., Smith, J.D., Yanagisawa, M. (2007). *Pre-Crash Scenario Typology for Crash
  Avoidance Research.* DOT HS 810 767, NHTSA / Volpe Center.
- DOT HS 811 731 (2013) *Description of Light-Vehicle Pre-Crash Scenarios.*
- (可补)ISO 34502 场景分类;EURO NCAP 测试场景;PEGASUS 6-layer model;ISO 21448 SOTIF。
