# crash2openx Fuzz 模块

## 目标

从 42 个**已 OCL/WF/XSD/QA 全部校验通过**的 medoid 种子出发，通过**参数变异**在 SceneSeed 层探索邻域，扩充测试场景库到数百个多样化的变体，**不再调用 LLM**——所有变异都是确定性数值扰动，享受种子的 provenance，同时用现有 gate 做合法性守卫。

## 为什么在 SceneSeed 层做 Fuzz？

Crash2OpenX 流水线的分层结构决定了 fuzz 有一个**天然接入点**：

```
input → road(LLM) → scene(LLM) → OCL → xodr(det) → qa(VLM) → roadgraph → xosc(det) → carla
                                  ↑
                          Fuzz 在这里注入
```

- 上游（LLM）产出的种子已通过语义校验，具备现实合理性
- 下游（XODR/XOSC/CARLA）是**确定性变换**，seed 一变，产物立即跟随
- OCL 15 条约束 + WF gate 天然拦截无效变异 → **免费的物理合法性守卫**

## 变异操作（Mutation Operators）

变异**只**作用在 SceneSeed 的可数值字段上（不动 seed 结构）。5 类算子：

| 算子 | 变异对象 | 边界 | 影响 |
| --- | --- | --- | --- |
| `mutate_trigger_dist` | `behavior.params.trig_dist` (m) | [3, 40] | 前车刹车触发距离 → 追尾时机 |
| `mutate_trigger_ttc` | `behavior.params.trig_ttc` (s) | [0.5, 5.0] | cut-in/junction 触发时距 → 危险度 |
| `mutate_target_speed` | `behavior.params.speed` (m/s) | [0, 25] | NPC 速度 → 相对动能 |
| `mutate_environment` | `environment.friction_scale` | [0.1, 1.0] | 路面摩擦 → 制动能力 |
| `mutate_environment_weather_tod` | `environment.weather / time_of_day` | 枚举 | 光照/能见度 → 感知失效 |

**分布**：数值算子采用**截断高斯**（μ=当前值，σ=0.2×取值范围）；枚举算子采用**均匀离散**。

## 合法性守卫（Validation Gates）

变异后**在提交到编译器之前**顺序过 4 道 gate：

1. **OCL I1–I9 + P1–P6**（`tools.ocl_constraints.violations`）—— 无外部依赖，毫秒级
2. **XSD**（编译 XODR/XOSC 时 `scenariogeneration` 自动校验）
3. **WF gates**（`osc_blocks._check_wf`）—— 跨元模型约束
4. **CARLA extract**（可选，需 GPU 主机；离线模式跳过）

任一 gate 拒绝 → 丢弃变体，不进入 archive。

## 评估函数（Evaluator）

由于本毕设的运行环境是 macOS（无 CARLA），fuzz 循环采用**离线代理评分**：

$$\text{score} = w_1 \cdot \underbrace{\text{danger}}_{\text{触发距离小、速度大、摩擦低}} + w_2 \cdot \underbrace{\text{diversity}}_{\text{相对已 archive 变体的新颖度}}$$

- **Danger 代理**：$\frac{1}{\text{trig\_dist} + 1} \cdot v_{\text{npc}} \cdot (2 - \mu_{\text{friction}})$
- **Diversity**：以 (block, trig_dist_bin, speed_bin, weather) 为坐标，MAP-Elites archive 每格保留最高分变体

线上（有 CARLA）时应替换为**真实指标**：最小 TTC / 碰撞发生率 / 车道入侵计数（毕设开题第 4.3.2 节的"多维评价体系"）。

## Archive 策略

**MAP-Elites**：把行为特征空间分格，每格保留一个"精英"变体。特征轴：

| 轴 | 分桶 |
| --- | --- |
| block 类型 | 8 个（front_brake, cut_in, oncoming, ...） |
| trig_dist_bin | `<5`, `5-15`, `15-30`, `>30` |
| target_speed_bin | `<5`, `5-15`, `15-25`, `>25` (m/s) |
| weather | 5 个枚举 |

理论上限约 `8 × 4 × 4 × 5 = 640` 格。实际因 OCL 约束，可填格数远小于此。

## 使用

### 单 case 演示（离线，无 CARLA）

```bash
uv run python -m tools.fuzzer.runner \
    --seed data/seeds/scene_seed/122_Waymo_February_18_2024.json \
    --road data/seeds/road_seed/122_Waymo_February_18_2024.json \
    --budget 100 \
    --out outputs/fuzz_runs/case122
```

### 批量（42 medoid）

```bash
uv run python -m tools.fuzzer.runner --batch data/eval/baseline_42.json --budget 50 --out outputs/fuzz_runs/batch
```

### 可视化

```bash
uv run python -m tools.fuzzer.visualize --run outputs/fuzz_runs/case122
```

产出：
- `coverage_curve.png` — 每步 archive 填充格数 vs 预算
- `danger_curve.png` — 累计最大 danger score vs 预算
- `archive_heatmap.png` — MAP-Elites 网格填充率

## 与毕设开题的对应

| 开题内容 | Fuzz 模块实现 |
| --- | --- |
| §3.2.2「Fuzzing 变异种子场景」| `mutator.py` 5 类算子 |
| §3.2.2「双重合法性校验」| OCL/WF gate 物理层 + LLM 语义层（离线暂省，线上可接） |
| §4.2「降维搜索 + 关键实体锁定」| 只变异 `behavior.params` + `environment`，不变异结构 |
| §4.2「多目标优化」| danger + diversity 双目标，MAP-Elites 精英保留 |
| §4.3「多维评价体系」| Evaluator 接口预留 safety/effectiveness/compliance/comfort 四维（线上启用） |

## 后续（学位论文延续）

- **在线评估**：Linux+GPU 主机接 CARLA，把离线 danger 代理换成实测 min-TTC / 碰撞 / 违规 / jerk 四维分数
- **多 ADS 对比**：同一 archive 在 TransFuser vs InterFuser vs `carl_*` 上重跑，统计失效差异
- **与基线对比**：接入 ScenarioFuzz、DeepCollision，统计相同预算下的失效发现率
