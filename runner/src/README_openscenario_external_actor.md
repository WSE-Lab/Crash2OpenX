# OpenSCENARIO 加载与 External Actor 控车流程（基于 `src/demo.py`）

本文从 `src/demo.py` 作为入口，说明如何加载**已经生成好的 OpenSCENARIO（`.xosc`）文件**并完成一次仿真测试；同时解释**被测车辆（ego/hero）由 external actor 控制**时，我在代码里是如何把控制链路接到 ScenarioRunner 的。

---

## 1. 入口与相关文件

入口脚本与关键链路如下（按“从外到内”的调用顺序）：

- 入口：`src/demo.py`
- 本地 runner 封装：`src/scenario_runner_local.py`
- OpenSCENARIO 配置解析：`src/openscenario_configuration.py`
- OpenSCENARIO 场景树构建：`src/open_scenario.py`
- 控制器解析（读 `.xosc` 里的 `ControllerAction`）：`scenario_runner/srunner/tools/openscenario_parser.py`
- 控制器装载（把 `module=external_control` 映射到 Python 类）：`scenario_runner/srunner/scenariomanager/actorcontrols/actor_control.py`
- 外部控车实现（external actor）：`scenario_runner/srunner/scenariomanager/actorcontrols/external_control.py`
- external actor 内部用到的 agent（这里是 PCLA）：`PCLA/PCLA.py`
- external actor 的目标点输入（单点目的地）：`src/route.xml`

> 运行目录建议使用 `src/`（见后文“常见问题：route.xml 找不到”）。

---

## 2. 一次完整的“加载 xosc → 跑仿真 → 出结果”流程

### 2.1 启动阶段（`demo.py`）

`demo.py` 做了三件关键事情：

1. 解析命令行参数（CARLA host/port、`.xosc` 路径、是否采集数据等）
2. `xml.etree.ElementTree.parse()` 读取 `.xosc` 得到 `xml_tree`
3. 创建并运行 `ScenarioRunner(args, xml_tree)`（来自 `src/scenario_runner_local.py`）

### 2.2 配置解析（`OpenScenarioConfiguration`）

`ScenarioRunner._run_openscenario()` 会创建：

```python
config = OpenScenarioConfiguration(xml_tree, client, openscenario_params)
```

`OpenScenarioConfiguration` 的核心工作：

- 用 XSD 校验 `.xosc` 是否符合 OpenSCENARIO 1.0（`OpenSCENARIO.xsd`）
- 从 `.xosc` 解析出：
  - 地图（`RoadNetwork/LogicFile filepath="TownXX"`）
  - Actors（`Entities/ScenarioObject`）
    - 通过 `Property name="type" value="ego_vehicle"` 标记 ego（被测车）
  - Init（初始位置/初速度/控制器绑定等，来自 `Storyboard/Init`）

### 2.3 加载地图与生成 actor（`ScenarioRunner._load_and_run_scenario`）

`ScenarioRunner._load_and_run_scenario(config)` 负责：

1. 加载/切换 CARLA world 到 `config.town`
2. 设置同步模式（sync）与 TrafficManager
3. 根据 `config.ego_vehicles` **spawn ego**
4. 创建 `OpenScenario(...)` 实例并交给 `ScenarioManager` 运行

### 2.4 构建 OpenSCENARIO 行为树（`OpenScenario`）

`src/open_scenario.py` 里，`OpenScenario._create_init_behavior()` 会遍历 `Init/Private`，对每个 actor 执行初始化动作，其中与“external actor 控车”最相关的是：

- 解析 `.xosc` 中该 actor 的 `ControllerAction`
- 调用 `OpenScenarioParser.get_controller(...)` 得到：
  - `module`（例如 `"external_control"`）
  - `args`（`Properties` 里除 `module` 外的其他参数）
- 创建一个 `ChangeActorControl(actor, control_py_module=module, args=args, ...)` 节点加入 init 行为树

### 2.5 逐帧执行（`ScenarioManager` + `UpdateAllActorControls`）

`src/scenario_manager_local.py` 在循环中做：

- 每 tick 更新 `CarlaDataProvider` 和 `GameTime`
- `scenario_tree.tick_once()` 执行一次 py_trees 行为树

而 `scenario_runner/srunner/scenarios/basic_scenario.py` 会把 `UpdateAllActorControls()` 挂到 `scenario_tree` 上；它每 tick 会：

- 从 Blackboard 读出 `ActorsWithController`
- 逐个调用 `ActorControl.run_step()`，从而驱动 external actor 的控制回路

### 2.6 结束与输出

场景结束后：

- `ScenarioRunner._analyze_scenario()` 输出 criteria 结果（SUCCESS/FAILURE/TIMEOUT）
- `ScenarioRunner.destroy()`/`ScenarioManager.cleanup()` 清理 actor、恢复 world 设置
- 如果启用了数据收集（`--collect-data`），会在最后保存 JSON（参考 `src/README_data_collector.md`）

---

## 3. external actor 控制被测车：我是怎么接入的

这里的“external actor”指：**被测车的纵向/横向控制不由 ScenarioRunner 内置 NPC 控制器或 leaderboard agent 直接给出**，而是由一个“外部控制模块”在每个 tick 计算控制量，并直接 `apply_control` 到 ego 上。

### 3.1 `.xosc` 里如何声明“用 external actor 控车”

关键是给 ego 的 `Init/Private` 加一个 `ControllerAction`，并在 `Properties` 里配置：

```xml
<Private entityRef="hero">
  ...
  <PrivateAction>
    <ControllerAction>
      <AssignControllerAction>
        <Controller name="HeroAgent">
          <Properties>
            <Property name="module" value="external_control" />
            <!-- 其他参数（可选）：<Property name="xxx" value="yyy" /> -->
          </Properties>
        </Controller>
      </AssignControllerAction>
      <OverrideControllerValueAction>
        <Throttle value="0" active="false" />
        <Brake value="0" active="false" />
        <SteeringWheel value="0" active="false" />
        ...
      </OverrideControllerValueAction>
    </ControllerAction>
  </PrivateAction>
</Private>
```

可直接参考示例：`scenario_runner/scenario_from_LLM.xosc`。

### 3.2 ScenarioRunner 内部如何把 `module=external_control` 变成“每帧控车”

链路是：

1. `OpenScenarioParser.get_controller()` 读取 `Properties`，得到 `module="external_control"` 和参数字典 `args`
2. `ChangeActorControl` 创建 `ActorControl(actor, control_py_module="external_control", args=...)`
3. `ActorControl` 用 `importlib.import_module("external_control")` 导入：
   - `scenario_runner/srunner/scenariomanager/actorcontrols/external_control.py`
   - 按规则把模块名映射成类名 `ExternalControl`
4. `BasicScenario` 每 tick 运行 `UpdateAllActorControls`，对 ego 调用 `ExternalControl.run_step()`

### 3.3 `ExternalControl` 里我是怎么实现 external actor 的

实现文件：`scenario_runner/srunner/scenariomanager/actorcontrols/external_control.py`

核心逻辑：

- 初始化阶段（`__init__` / `_setup_pcla`）：
  1. 通过 `CarlaDataProvider` 拿到 `world/client`
  2. 读取目的地：`src/route.xml`（只需要一个 `<waypoint .../>`）
  3. 以“ego 当前坐标 → 目的地”生成一条 waypoint 序列，并写出 `temp_route_for_external_control.xml`
  4. 用该 route 初始化 PCLA：`PCLA(agent=..., vehicle=actor, route=..., client=client)`
  5. （可选）挂一个俯视相机，并把 spectator 视角切到车上方跟随

- 每 tick（`run_step`）：
  1. `control = self._pcla_agent.get_action()`
  2. `self._actor.apply_control(control)`
  3. 更新 spectator 视角（俯视跟随）

> 注意：这里的 “external actor” 本质上是一个**控制回路实现**（此处用 PCLA 计算控制量），其输出是标准 `carla.VehicleControl`。

---

## 4. 如何运行（建议命令）

### 4.1 前置条件

- CARLA Server 正在运行，且版本满足 `0.9.16+`（`src/scenario_runner_local.py` 会检查）
- Python 环境能 `import carla`
- external actor（PCLA）相关权重/环境变量已按你的实际环境配置好（PCLA 内部可能依赖本地路径与权重目录）

### 4.2 推荐运行方式

从仓库根目录进入 `src/` 再运行（避免相对路径问题）：

```bash
cd src
python demo.py \
  --host localhost \
  --port 2000 \
  --scenario ../scenario_runner/scenario_from_LLM.xosc \
  --collect-data
```

如果你要测试自己生成的 `.xosc`，把 `--scenario` 换成对应路径即可。

### 4.3 设置 external actor 的目的地

编辑 `src/route.xml` 里唯一的 waypoint（x/y/z），它会被 `external_control.py` 当作目标点。

---

## 5. 常见问题（排错要点）

1. **报错：OpenSCENARIO 校验失败 / XSD validate error**
   - 说明生成的 `.xosc` 不符合 OpenSCENARIO 1.0 结构；先用最小可跑模板（如 `scenario_runner/scenario_from_LLM.xosc`）对齐结构再逐步扩展。

2. **地图不匹配（The CARLA server uses the wrong map）**
   - 检查 `.xosc` 的 `RoadNetwork/LogicFile filepath="TownXX"` 是否与你启动 CARLA 时加载的 town 一致（或启用 `--reloadWorld` 强制切换）。

3. **external actor 没生效（ego 不动）**
   - 检查 `.xosc` 的 ego 是否真的有 `ControllerAction`，且 `Property name="module" value="external_control"` 写对了。
   - 确认 `ScenarioObject` 对应 actor 的 `Properties` 里 `type=ego_vehicle`（否则可能不会被当作 ego spawn）。

4. **报错：找不到 `route.xml`**
   - 当前实现里 external control 读取目的地依赖相对路径，建议使用 `cd src` 后再 `python demo.py`（见上面的推荐命令）。

5. **PCLA 权重/路径相关报错**
   - `PCLA/PCLA.py` 内部对权重目录、agent 配置有硬编码/环境依赖；需要按你的机器实际路径调整（或在 `agents.json` + 环境变量层面配置）。
