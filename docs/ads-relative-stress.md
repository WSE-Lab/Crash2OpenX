# 用 ADS 相对触发生成更紧迫的场景

本次增加的是可审计的压力变体：原始事故种子保留，派生场景明确记录改过的速度、动作和触发参数。`hero` 是交给 PCLA 的 ADS 被测车；它未必等同于事故报告中的 AV。

## 为什么以前看起来有充分反应时间

1. 部分行为按固定时刻启动，与 ADS 实际走到哪里无关。
2. 旧 cut-in 的 `trig_ttc` 没被编译器用于触发，实际走距离条件加 1.5 秒启动保护。新实验请使用 `ads_trigger`，不要用这个旧参数调危险度。
3. 相对速度不匹配时，NPC 一直远离 ADS。本次第一组实测中，NPC 目标 8 m/s，而 InterFuser 稳态约 5 m/s，两档窗口都没进入。
4. 变道“距离”必须结合实际速度理解。NPC 3 m/s 行驶 22 m 才完成变道，过程会持续约 7 秒；这本身就给了 ADS 充分应对时间。
5. 执行器原有的逐条件锁存，以及用中心距离减两个半车长计算侧向净空，都不适合精确的相对触发窗口。

## 新的控制方式

```json
{
  "id": "merging_car",
  "kind": "vehicle",
  "position": "adjacent",
  "side": "left",
  "behavior": {
    "block": "cut_in",
    "params": {"speed": 3, "long": 24, "cut_dist": 7.5},
    "ads_trigger": {
      "distance_m": 3,
      "min_clearance_m": 0.5,
      "min_ego_speed_mps": 1,
      "min_npc_speed_mps": 2
    }
  }
}
```

NPC 先在自己的车道行驶，实际车身净空进入 0.5–3 m、ADS 和 NPC 同时达到运动门槛后才切入。没有“等久了自动触发”的兜底。事件开始后控制器照常执行动作，不反复重置窗口。ADS 的感知、规划和制动自由度保留。

适用于急刹、切入、部分侵入、行人横穿，以及路口 NPC 等待后出发。路口出发模式目前不是精确的冲突点到达时刻匹配；这一轮真实闭环验证的是切入，其他新分支完成了编译/回归验证。

## 实测对照（2026-09-19）

同一生成道路、同一 InterFuser、同一初始化和动作参数：ADS 初始 5 m/s，NPC 目标 3 m/s，相邻车道前方 24 m，变道距离 7.5 m。只改变窗口上界。每档一次运行，不能据此宣称统计上的失效率提升。

| 测量项目 | 提前切入 | 近距离切入 |
|---|---:|---:|
| 配置触发净空 | 16 m | 3 m |
| 事件启动时实测净空 | 15.97 m | 2.99 m |
| 启动时 ADS 速度 | 4.88 m/s | 4.85 m/s |
| 净空 / ADS 速度（时机指标，非 TTC） | 3.27 s | 0.62 s |
| 事件后 6 s 内最小正 TTC | 2.75 s | 0.80 s |
| 事件后 6 s 内最小车身净空 | 5.79 m | 1.03 m |
| NPC 可观测横向运动滞后 | 0.65 s | 0.65 s |
| 事件后 6 s 内 NPC 最大横向加速度 | 3.39 m/s² | 3.78 m/s² |
| 实际碰撞传感器接触 | 无 | 无 |

这里的 TTC 使用双方真实包围盒、位置和速度，通过固定朝向、匀速外推计算。尚在不同车道平行行驶时可能没有有限 TTC，不用中心距离除速度差制造一个虚假的碰撞倒计时。

近距离切入造成更紧迫的冲突，ADS 最终成功避让。可以用它展示“事故语义约束下，框架能生成接近安全边界的测试”；不能把未碰撞的视频称为已发现 ADS 失效。完整运行最终均因地图末端阻塞停止（`blocked_exit`），因此这里只比较明确的事件后 6 秒窗口，不把后续地图末端行为算成危险场景收益。

## 复现

从示例种子生成三档变体；新目录保留源种子、地图和全部参数差异：

```bash
uv run python tools/ads_stress.py \
  --scene examples/ads_relative_cut_in/scene_seed.json \
  --road examples/ads_relative_cut_in/road_seed.json \
  --out outputs/my_ads_stress
```

对于已有场景，可加 `--actors NPC_ID` 选定危险行为、`--xodr PATH` 保持原地图；路口还需用 `--map-name NAME` 指向对应道路缓存。可加 `--calibrate-from RUN_DIR`，从以前运行的前 2–10 秒、事件前、运动且未制动的样本估计 ADS 稳态速度，把 NPC 目标设为该速度的 0.6 倍，并设置 2.5 秒名义切入时间。所有校准变化单独写进 manifest；缺样本时拒绝猜测。这个比值是公开实验假设，不是对所有 ADS 的最优参数。

CARLA 主机的独立运行目录先装入支持模块和 hook（在该主机的仓库执行）：

```bash
python tools/enable_ads_conditions.py --runtime-root /path/to/carla_runtime
```

新编译的 XOSC 使用标准实体条件，但本项目执行器需要这个 hook 才能保证同时满足条件和正确的侧向净空；仅通过 XSD 不等于运行语义正确。照常用 CARLA/PCLA runner 运行各档 XOSC，ADS 不接管成脚本车辆。

从真实运行产物生成审阅数据和标注视频：

```bash
uv run python tools/review_ads_stress.py \
  --run outputs/my_run \
  --xosc outputs/my_run/scenario.runtime.xosc \
  --out outputs/my_review --video
```

视频标识从相机参数、逐帧实际车辆姿态和包围盒投影得到，青色为 ADS，橙色为 NPC。视频与轨迹相差超过 75 ms 时不画车辆框；原视频保留。`review.json` 同时区分未触发、缺少观测、实际碰撞、动作开始与实际运动开始，避免把脚本计划当作已实现效果。

远程 CARLA 客户端在收到完整 RGB、相机参数和轨迹时，会自动生成 `ads_review/`。网页运行详情优先播放已完成的 ADS 标注版；原视频仍在 `carla_rgb.mp4`，视频接口附加 `?raw=1` 可播放原版。旧运行缺乏所需观测时继续保留原视频，不猜测演员身份或事件。
