# 项目总览：从 `src/demo.py` 到 OpenSCENARIO、PCLA 与指标采集

本文按当前仓库里的实际代码，概括整条运行链路。重点说明四件事：

- `src/demo.py` 如何驱动一次场景测试
- 运行时指标是怎么采、怎么分层存储的
- PCLA 现在能加载哪些 agent
- 现有配置层分别能控制什么

本文描述的是当前仓库状态，不包含“未来想兼容更多 OpenSCENARIO 示例”的扩展方案。

---

## 1. 项目结构总览

和主流程最相关的目录/文件如下：

- 入口脚本：[src/demo.py](/home/server/workspace/LXJ/leaderboard_2.0/src/demo.py)
- 本地运行器封装：[src/scenario_runner_local.py](/home/server/workspace/LXJ/leaderboard_2.0/src/scenario_runner_local.py)
- 本地场景管理器：[src/scenario_manager_local.py](/home/server/workspace/LXJ/leaderboard_2.0/src/scenario_manager_local.py)
- OpenSCENARIO 配置解析：[src/openscenario_configuration.py](/home/server/workspace/LXJ/leaderboard_2.0/src/openscenario_configuration.py)
- OpenSCENARIO 场景树构建：[src/open_scenario.py](/home/server/workspace/LXJ/leaderboard_2.0/src/open_scenario.py)
- 运行时数据采集：[src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py)
- external actor 控制器：[scenario_runner/srunner/scenariomanager/actorcontrols/external_control.py](/home/server/workspace/LXJ/leaderboard_2.0/scenario_runner/srunner/scenariomanager/actorcontrols/external_control.py)
- PCLA 主入口：[PCLA/PCLA.py](/home/server/workspace/LXJ/leaderboard_2.0/PCLA/PCLA.py)
- PCLA agent 索引：[PCLA/agents.json](/home/server/workspace/LXJ/leaderboard_2.0/PCLA/agents.json)

---

## 2. 从 `src/demo.py` 开始的一次运行流程

### 2.1 参数解析

`demo.py` 暴露的命令行参数主要有：

- `--host` / `--port`：CARLA server 地址
- `--timeout`：CARLA client 超时
- `--trafficManagerPort`
- `--trafficManagerSeed`
- `--sync`：是否同步模式
- `--reloadWorld`：是否切换/重载地图
- `--record`：是否录制 CARLA log
- `--scenario`：要运行的 `.xosc`
- `--xodr` / `--map`：可选，自定义 OpenDRIVE `.xodr` 地图文件；指定后会把运行时 `.xosc` 的 `RoadNetwork/LogicFile` 指向该文件，并用 CARLA `generate_opendrive_world()` 先加载自定义地图
- `--output`：route 文件名
- `--collect-data`：是否开启采集
- `--data-output`：episode 输出目录
- `--no-rendering`：启用 CARLA `no_rendering_mode`，无画面渲染也能跑测试
- `--record-video`：根据采集到的 actor 轨迹生成俯视视频 `trajectory_video.mp4`
- `--video-fps` / `--video-frame-stride`：控制轨迹视频帧率和采样间隔

定义位置在 [src/demo.py](/home/server/workspace/LXJ/leaderboard_2.0/src/demo.py#L27)。

### 2.2 解析场景并生成运行时 route

当前 `demo.py` 假设你的 `.xosc` 满足这几个前提：

- ego 实体名是 `hero`
- 能从 `Storyboard/Story/Act/StopTrigger/.../WorldPosition` 中提取终点
- ego 的控制器属性里存在 `module=external_control`

在这个前提下，`demo.py` 会：

1. 读取 `.xosc`
2. 根据 `RoadNetwork/LogicFile` 确保 CARLA world 切到正确 town；如果传入 `--xodr` / `--map`，则先用该 `.xodr` 生成 `OpenDriveMap`
3. 读取 hero 起点
4. 读取 `ActEnd` 的终点
5. 用 PCLA 的全局规划工具从起点到终点生成统一 `route.xml`
6. 把生成后的 route 文件路径写回 ego controller 的 `route_file`
7. 把 `ActEnd` 同步改成“到达 route 终点 1m 范围内结束”
8. 输出一份 `route_preview.svg`

自定义地图与场景一起运行的最小命令：

```bash
python src/demo.py \
  --xodr /path/to/custom_map.xodr \
  --scenario /path/to/custom_scenario.xosc \
  --host localhost \
  --port 2000
```

这部分逻辑在 [src/demo.py](/home/server/workspace/LXJ/leaderboard_2.0/src/demo.py#L232) 到 [src/demo.py](/home/server/workspace/LXJ/leaderboard_2.0/src/demo.py#L284)。

### 2.3 建立 episode 输出目录

如果启用了 `--collect-data`，`demo.py` 会为本次测试创建一个独立目录。目录下目前至少会有：

- `metadata.json`
- `frame_states.jsonl`
- `events.jsonl`
- `summary.json`
- `route.xml`
- `route_preview.svg`
- 如果传了 `--record-video`，还会生成 `trajectory_video.mp4`

目录构造逻辑在 [src/demo.py](/home/server/workspace/LXJ/leaderboard_2.0/src/demo.py#L45) 和 [src/demo.py](/home/server/workspace/LXJ/leaderboard_2.0/src/demo.py#L404)。

### 2.4 启动 ScenarioRunner

随后 `demo.py` 会创建本地 `ScenarioRunner`：

- 传入命令行参数和 `xml_tree`
- 如果开启采集，就把 `DataCollector` 注入 runner
- 调用 `scenario_runner.run()`

入口在 [src/demo.py](/home/server/workspace/LXJ/leaderboard_2.0/src/demo.py#L431)。

### 2.5 ScenarioRunner 做什么

[src/scenario_runner_local.py](/home/server/workspace/LXJ/leaderboard_2.0/src/scenario_runner_local.py) 负责：

- 创建 CARLA client
- 校验 CARLA Python API 版本
- 创建 `ScenarioManager`
- 加载/切换 world
- spawn ego
- 构造 `OpenScenario`
- 启动场景运行循环
- 在结束时统一收口数据采集与资源清理

核心执行入口在 [src/scenario_runner_local.py](/home/server/workspace/LXJ/leaderboard_2.0/src/scenario_runner_local.py#L309)。

### 2.6 ScenarioManager 每 tick 做什么

[src/scenario_manager_local.py](/home/server/workspace/LXJ/leaderboard_2.0/src/scenario_manager_local.py) 的 `_tick_scenario()` 每个仿真 tick 会：

1. 更新 `GameTime`
2. 更新 `CarlaDataProvider`
3. 若存在 agent，则取一次控制命令并施加到 ego
4. 执行一次 scenario tree
5. 调用 `data_collector.collect_frame_data()`
6. 如果 collector 请求终止，则停止场景

关键位置在 [src/scenario_manager_local.py](/home/server/workspace/LXJ/leaderboard_2.0/src/scenario_manager_local.py#L164)。

---

## 3. OpenSCENARIO 与 external_control 的关系

当前工程并不是把 OpenSCENARIO 当作“ego 内部策略”，而是：

- OpenSCENARIO 负责定义场景、其他 actor、触发条件、结束条件
- ego 的控制权通过 controller property 交给 `external_control`
- `external_control` 再调用 PCLA 生成控制量

也就是：

- OpenSCENARIO 决定“场景怎么搭”和“何时结束”
- PCLA 决定“ego 怎么开”

`demo.py` 之所以会提前生成 route，并写回 `.xosc` 的 `route_file`，是为了让 `external_control` 初始化 PCLA 时能够直接拿到这条 route。

`route_file` 回写逻辑在 [src/demo.py](/home/server/workspace/LXJ/leaderboard_2.0/src/demo.py#L162)。  
`external_control` 读取 `route_file` 的逻辑在 [external_control.py](/home/server/workspace/LXJ/leaderboard_2.0/scenario_runner/srunner/scenariomanager/actorcontrols/external_control.py#L101)。

---

## 4. PCLA 是怎么被接进来的

### 4.1 external_control 如何初始化 PCLA

[external_control.py](/home/server/workspace/LXJ/leaderboard_2.0/scenario_runner/srunner/scenariomanager/actorcontrols/external_control.py) 的 `_setup_pcla()` 会：

1. 读取当前 actor、world、client
2. 读取 controller property 里的 `route_file`
3. 若 route 文件存在，直接读 route 的最后一个 waypoint 作为终点
4. 若 route 文件不存在，则退化成“从当前位置向前选一个 endpoint”再临时规划 route
5. 创建 `PCLA(agent=..., vehicle=actor, route=route_file, client=client)`

实现位置在 [external_control.py](/home/server/workspace/LXJ/leaderboard_2.0/scenario_runner/srunner/scenariomanager/actorcontrols/external_control.py#L50)。

### 4.2 PCLA 做什么

[PCLA/PCLA.py](/home/server/workspace/LXJ/leaderboard_2.0/PCLA/PCLA.py) 的职责可以分成三步：

1. `setup_agent()`  
   根据 agent 字符串找到 agent 文件和 config 路径，动态导入 agent 类并实例化

2. `setup_route()`  
   用 route XML 构造 route index，并将全局规划结果喂给 agent

3. `setup_sensors()`  
   按 agent `sensors()` 定义，把相机/雷达/GNSS/速度计等挂到 ego 车上

调用控制时，`get_action()` 会在当前 tick 更新 `GameTime`，然后执行 agent 推理并返回 `carla.VehicleControl`。

关键代码在 [PCLA.py](/home/server/workspace/LXJ/leaderboard_2.0/PCLA/PCLA.py#L30)。

---

## 5. 当前指标采集有哪些

当前运行时统计不再是“所有东西逐帧写一份”，而是分成三层：

- 逐 tick 状态
- 事件触发日志
- episode 聚合摘要

实现集中在 [src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py)。

### 5.1 逐 tick 采集：`frame_states.jsonl`

当前每个仿真 tick 记录的是 ego 基础状态。字段包括：

- `frame_id`
- `simulation_time`
- `wall_time`
- `delta_seconds`
- `x, y, z`
- `pitch, yaw, roll`
- `vx, vy, vz`
- `speed`
- `ax, ay, az`
- `longitudinal_accel`
- `lateral_accel`
- `angular_velocity_x/y/z`
- `throttle`
- `brake`
- `steer`
- `hand_brake`
- `reverse`
- `road_id`
- `lane_id`
- `lane_type`
- `is_junction`
- `speed_limit`
- `traffic_light_state`

提取逻辑在 [src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py#L272)。

### 5.2 事件触发采集：`events.jsonl`

当前事件流包含：

- `episode_started`
- `collision`
- `lane_invasion`
- `red_light_violation`
- `stop_sign_violation`
- `blocked_termination`
- `episode_finished`

其中：

- 碰撞由 `sensor.other.collision` 回调触发
- 压线由 `sensor.other.lane_invasion` 回调触发
- 红灯/stop sign 由逐 tick 规则判断触发

对应实现位置：

- 碰撞：[src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py#L419)
- 压线：[src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py#L447)
- 红灯：[src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py#L694)
- Stop sign：[src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py#L725)

### 5.3 在线聚合：`summary.json`

当前会在线更新并在 episode 结束时输出的聚合指标包括：

- `final_route_completion`
- `min_ttc`
- `min_distance`
- `max_long_jerk`
- `max_lat_jerk`
- `unsafe_exposure_time`
- `off_road_time`
- `opposite_lane_occupancy_time`
- `blocked_time`
- `collision_count`
- `lane_invasion_count`
- `red_light_violation_count`
- `stop_sign_violation_count`
- `termination_reason`
- `scenario_tree_status`
- `criteria`

这些指标的更新逻辑主要在：

- jerk：[src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py#L495)
- 最小 TTC / 最小距离：[src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py#L523)
- off-road：[src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py#L598)
- opposite lane：[src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py#L605)
- blocked：[src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py#L642)
- route completion：[src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py#L671)

### 5.4 当前终止机制

现在场景除了 OpenSCENARIO 自己的结束条件之外，还额外挂了两类主动退出：

- 碰撞立即退出：`collision_exit`
- 长时间 blocked 退出：`blocked_exit`

blocked 不是“静止 20 秒直接退出”，而是：

- 低速
- route progress 长时间无推进
- 且不是合法等待状态

当前合法等待状态包括：

- 红灯/黄灯
- stop sign 影响范围内
- 车辆处在 junction 内

这部分在 [src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py#L648) 和 [src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py#L677)。

---

## 6. PCLA 当前能加载哪些 agent

PCLA 的 agent 索引来自 [PCLA/agents.json](/home/server/workspace/LXJ/leaderboard_2.0/PCLA/agents.json)。

### 6.1 名称格式

PCLA 期望的 agent 字符串格式是：

- `<family>_<variant>`
- 或 `<family>_<variant>_<seed>`

解析逻辑在 [give_path.py](/home/server/workspace/LXJ/leaderboard_2.0/PCLA/pcla_functions/give_path.py#L5)。

例如：

- `tfv6_regnet`
- `simlingo_simlingo`
- `lmdrive_llava`
- `carl_roach_3`
- `tfv4_l6_2`

其中第三段 `seed` 会直接拼到 config 路径后面，用来选择不同 seed 的权重目录或配置。

### 6.2 当前 family 和 variant

当前 `agents.json` 一共定义了 12 个 family、35 个 variant：

- `tfv5`
  - `alltowns`
  - `notown13`
- `tfv6`
  - `regnet`
  - `resnet`
  - `4cameras`
  - `noradar`
  - `visiononly`
  - `notown13`
- `carl`
  - `carl`
  - `carlv11`
  - `roach`
  - `plant`
- `tfv3`
  - `tf`
  - `ltf`
  - `lf`
  - `gf`
- `lav`
  - `lav`
  - `fast`
- `lbc`
  - `lb`
  - `nc`
- `wor`
  - `lb`
  - `nc`
- `lmdrive`
  - `llava`
  - `vicuna`
  - `llama`
- `simlingo`
  - `simlingo`
- `tfv4`
  - `lav`
  - `aim`
  - `wp`
  - `l6`
- `neat`
  - `neat`
  - `aimbev`
  - `aim2dsem`
  - `aim2ddepth`
- `if`
  - `if`

### 6.3 实际 agent 文件在哪

每个条目在 `agents.json` 里会指定两样东西：

- `agent`：入口 Python 文件
- `config`：对应配置文件或权重目录

例如：

- `tfv6_regnet`
  - agent: `pcla_agents/transfuserv6/lead/inference/sensor_agent.py`
  - config: `pcla_agents/transfuserv6_pretrained/tfv6_regnety032`
- `if_if`
  - agent: `pcla_agents/interfuser/interfuser_agent.py`
  - config: `pcla_agents/interfuser/interfuser_config.py`
- `simlingo_simlingo`
  - agent: `pcla_agents/simlingo/agent_simlingo.py`
  - config: `pcla_agents/simlingo_pretrained/checkpoints/epoch=013.ckpt/pytorch_model.pt`

PCLA 在运行时会把 agent 文件动态导入，再调用该模块暴露的 `get_entry_point()` 找到类名，见 [PCLA.py](/home/server/workspace/LXJ/leaderboard_2.0/PCLA/PCLA.py#L51)。

---

## 7. “通过 config 能控制什么东西”

这里的 config 实际上分成四层，不是一个单一配置文件。

### 7.1 命令行参数层：控制运行方式

来自 `demo.py` 的 argparse。它控制的是：

- 连哪个 CARLA server
- 是否同步模式
- 是否重载 world
- TrafficManager 端口和随机种子
- 场景文件路径
- route 输出路径
- 是否采集数据
- episode 数据目录

这是“运行容器级”的控制层。

### 7.2 OpenSCENARIO `.xosc` 层：控制场景内容

`.xosc` 决定的是：

- 地图 town
- ego / other actors 的生成与初始位姿
- 天气、环境、交通参与者
- 行为树、触发条件、停止条件
- ego 控制器声明

对当前项目尤其重要的是这两类 property：

- actor property 中 `type=ego_vehicle`
- controller property 中 `module=external_control`

如果这两项不对，ego 的 spawn 和 external_control 接入就会失效。

### 7.3 controller properties 层：控制 external_control 的输入

当前最关键的是：

- `route_file`

`demo.py` 会把它回写到 hero 的 controller properties，external_control 再读出来初始化 PCLA。

这层本质上是“OpenSCENARIO 到外部控制器之间的参数桥”。

### 7.4 PCLA `agents.json` + agent 自身 config 层：控制策略模型

这一层控制的是：

- 选哪个 agent family
- 选哪个 variant
- 选哪个 seed
- 对应加载哪套权重/配置

同时 `give_path()` 还会按不同 family/variant 自动设置部分环境变量，例如：

- `ROUTES`
- `OPENBLAS_NUM_THREADS`
- `OMP_NUM_THREADS`
- `UNCERTAINTY_THRESHOLD`
- `STOP_CONTROL`
- `DIRECT`
- `SAMPLE_TYPE`

逻辑在 [give_path.py](/home/server/workspace/LXJ/leaderboard_2.0/PCLA/pcla_functions/give_path.py#L13)。

这说明当前工程里所谓“agent config”不只是一个 yaml，而是：

- `agents.json` 中的映射
- 具体 agent 代码内部读取的配置文件/权重
- `give_path()` 设置的环境变量

### 7.5 数据采集层：控制输出内容

当前 `DataCollector` 还没有完整做成命令行参数化，但代码里已经固定了若干采集/终止阈值，例如：

- `TTC_THRESHOLD_SECONDS`
- `BLOCKED_SPEED_THRESHOLD`
- `BLOCKED_PROGRESS_EPSILON`
- `BLOCKED_WARMUP_SECONDS`
- `BLOCKED_TERMINATION_SECONDS`

定义位置在 [src/data_collector.py](/home/server/workspace/LXJ/leaderboard_2.0/src/data_collector.py#L18)。

目前这些阈值是代码常量，不是 argparse 参数。

---

## 8. 当前工程的关键假设和限制

这部分非常重要，因为当前代码是“能跑你这条链”，不是“通吃任意 OpenSCENARIO”。

### 8.1 当前默认假设

- ego 实体名是 `hero`
- 场景能从 `ActEnd` 提取到终点
- ego controller 使用 `external_control`
- PCLA 需要一条 route 才能稳定工作

### 8.2 当前不通用的地方

对于 `scenario_runner/srunner/examples` 下很多官方 `.xosc`，这些假设并不总成立：

- 未必存在 `hero`
- 未必提供终点
- 未必存在 external control route_file

所以当前 `src/demo.py` 更适合“带明确 ego 与目标点的 ADS 测试场景”，而不是所有 OpenSCENARIO 示例。

---

## 9. 一句话总结当前项目

当前工程的主线是：

`src/demo.py` 先把 `.xosc` 场景改写成“hero 由 external_control + PCLA 按统一 route 驱动”，然后用本地 `ScenarioRunner` 执行 OpenSCENARIO，并由 `DataCollector` 输出分层的运行时数据和聚合指标。

如果只看当前实现，它更像一套“面向 ADS 测试场景的 OpenSCENARIO 运行与统计框架”，而不是一个通用 OpenSCENARIO 播放器。
