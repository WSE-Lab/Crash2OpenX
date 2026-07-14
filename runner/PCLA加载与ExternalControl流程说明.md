# PCLA + ExternalControl 加载与运行流程说明

> 梳理从 `.xosc` 文件中声明 `external_control` 控制器，到 PCLA Agent 逐帧输出控制指令的完整链路。

---

## 一、整体架构

```
.xosc 文件
  └─ <ControllerAction> module="external_control"
        │
        ▼
  ActorControl (actor_control.py)         ← 控制器调度中枢
        │  importlib 加载 "external_control" 模块
        ▼
  ExternalControl (external_control.py)   ← 桥接层
        │  _setup_pcla()
        ▼
  PCLA (PCLA.py)                          ← ADS 框架
        ├─ setup_agent()   → 通过 agents.json 动态加载具体 ADS（如 interfuser）
        ├─ setup_route()   → 从路线文件构建全局规划，注入 agent.set_global_plan()
        └─ setup_sensors() → 在 ego 车上 spawn 传感器并注册回调
        │
        ▼  每帧 get_action()
  AutonomousAgent.__call__()              ← 调用 agent.run_step(sensor_data, timestamp)
        │
        ▼
  ExternalControl.run_step()              → actor.apply_control(control)
```

---

## 二、触发入口：xosc 声明控制器

在 `.xosc` 的 `<Init>` 段，ego vehicle 的 `<Private>` 节点中声明：

```xml
<ControllerAction>
  <AssignControllerAction>
    <Controller name="HeroAgent">
      <Properties>
        <Property name="module" value="external_control"/>
      </Properties>
    </Controller>
  </AssignControllerAction>
  <OverrideControllerValueAction>
    <Throttle value="0" active="false"/>
    ...
  </OverrideControllerValueAction>
</ControllerAction>
```

`module="external_control"` 告诉 ScenarioRunner：**用 `external_control` Python 模块来控制这个 Actor**。

---

## 三、控制器加载：ActorControl → ExternalControl

**文件**：`srunner/scenariomanager/actorcontrols/actor_control.py`

`open_scenario.py` 的 `_create_init_behavior()` 解析 `<ControllerAction>` 后，构造 `ChangeActorControl` 行为原子：

```python
# open_scenario.py
module, args = OpenScenarioParser.get_controller(controller_action, catalogs)
controller_atomic = ChangeActorControl(
    carla_actor,
    control_py_module=module,   # "external_control"
    args=args,
    scenario_file_path=os.path.dirname(config.filename)
)
```

`ChangeActorControl.update()` 在行为树首次 tick 时执行，将 `(actor_id, ActorControl实例)` 写入 `py_trees.Blackboard["ActorsWithController"]`。

**ActorControl 的加载逻辑**（`actor_control.py:65-89`）：

```python
# control_py_module = "external_control"
sys.path.append(os.path.dirname(__file__))          # actorcontrols/ 目录已在 path 中
module_control = importlib.import_module("external_control")
control_class_name = "ExternalControl"              # external_control → ExternalControl
self.control_instance = ExternalControl(actor, args)
```

---

## 四、ExternalControl 初始化

**文件**：`srunner/scenariomanager/actorcontrols/external_control.py`

```python
class ExternalControl(BasicControl):
    def __init__(self, actor, args=None):
        self._agent_name = "if_if"   # 默认使用 interfuser
        if actor:
            self._setup_pcla(actor)
```

### `_setup_pcla(actor)` 三步流程

**第一步：构造路线**

```python
start_location = actor.get_location()                      # ego 车当前位置（来自 xosc Init）
end_location = self._read_destination_from_route_xml("temp_route.xml")  # 读取目标点
waypoints = location_to_waypoint(client, start_location, end_location)  # GlobalRoutePlanner 规划
route_maker(waypoints, "temp_route_for_external_control.xml")            # 写成 leaderboard route XML
```

> `temp_route.xml` 是 `demo.py` 中 `save_route()` 预先写好的目标点文件（单个 waypoint）。

**第二步：实例化 PCLA**

```python
self._pcla_agent = PCLA(
    agent=self._agent_name,                     # e.g. "if_if"
    vehicle=actor,
    route="temp_route_for_external_control.xml",
    client=client
)
```

**第三步：架设俯视摄像头（辅助观察）**

```python
self._setup_overhead_camera(actor, world)       # 20m 高 RGB 摄像头 + Spectator 跟随
```

---

## 五、PCLA 内部初始化

**文件**：`PCLA/PCLA.py`

`PCLA.__init__` 调用 `set()` → 三个子步骤：

### 5.1 setup_agent()：动态加载 ADS

```python
# 1. 解析 agent 名称 "if_if" → agent_name="if", variant="if"
self.agentPath, self.configPath = give_path(agent, self.current_dir, routePath)
```

`give_path()` 查询 `PCLA/agents.json`：
```json
{
  "if": {
    "if": {
      "agent":  "pcla_agents/interfuser/interfuser_agent.py",
      "config": "pcla_agents/interfuser_pretrained/interfuser_baseline"
    }
  }
}
```

```python
# 2. 隔离加载：防止多 agent 间模块污染
spec = importlib.util.spec_from_file_location(module_key, self.agentPath)
module_agent = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module_agent)

# 3. 通过约定的入口函数获取类名并实例化
agent_class_name = module_agent.get_entry_point()       # e.g. "InterfuserAgent"
self.agent_instance = getattr(module_agent, agent_class_name)(self.configPath)
# → 调用 InterfuserAgent.__init__(configPath) → InterfuserAgent.setup()
```

**agent 隔离机制**：每次加载前清除 `sys.modules` 中的 `model`、`config`、`planner` 等通用模块名，避免不同 agent 互相污染。

### 5.2 setup_route()：注入全局路线

```python
route_indexer = RouteIndexer(self.routePath, no_scenarios_json, 1)
config = route_indexer.next()           # 读取 route XML 中的 waypoint 列表
gps_route, route = interpolate_trajectory(world, config.trajectory)
# GlobalRoutePlanner 将稀疏 waypoints 插值为 1m 间距的稠密路径
# 同时转换为 GPS 坐标（lat/lon）
self.agent_instance.set_global_plan(gps_route, route)
# → AutonomousAgent.set_global_plan() 降采样后存入 _global_plan / _global_plan_world_coord
```

**插值逻辑**（`route_manipulation.py:interpolate_trajectory`）：
- 使用 `GlobalRoutePlanner` 在地图拓扑上逐段 `trace_route()`
- 每段返回 `(carla.Waypoint, RoadOption)` 元组列表，hop 分辨率 1m
- 最后将世界坐标转换为 GPS 坐标（基于地图中 OpenDRIVE header 的经纬度参考点）

### 5.3 setup_sensors()：绑定传感器

```python
for sensor_spec in self.agent_instance.sensors():
    if sensor_spec['type'] == 'sensor.opendrive_map':
        sensor = OpenDriveMapReader(vehicle, reading_frequency)   # 伪传感器
    elif sensor_spec['type'] == 'sensor.speedometer':
        sensor = SpeedometerReader(vehicle, frame_rate)           # 伪传感器
    else:
        bp = bp_library.find(sensor_spec['type'])                 # 真实 CARLA 传感器
        sensor = world.spawn_actor(bp, transform, vehicle)        # 挂载到 ego 车
    sensor.listen(CallBack(sensor_spec['id'], sensor_spec['type'], sensor,
                           self.agent_instance.sensor_interface)) # 注册回调
world.tick()
CarlaDataProvider.register_actor(vehicle)
```

---

## 六、每帧执行链路

### 6.1 行为树驱动

`ScenarioManager._tick_scenario()` 每帧调用：

```python
self.scenario_tree.tick_once()
```

行为树中有一个持续 RUNNING 的 `UpdateAllActorControls` 节点（`BasicScenario` 在 `scenario_tree` 中加入），它在每帧 `update()` 里：

```python
# atomic_behaviors.py: UpdateAllActorControls.update()
actor_dict = py_trees.blackboard.Blackboard().ActorsWithController
for actor_id in actor_dict:
    actor_dict[actor_id].run_step()    # → ActorControl.run_step()
                                        # → ExternalControl.run_step()
```

### 6.2 ExternalControl.run_step()

```python
def run_step(self):
    control = self._pcla_agent.get_action()   # ① 调用 PCLA 获取决策
    self._actor.apply_control(control)         # ② 下发给 ego 车
    self._update_spectator_view()              # ③ 更新 Spectator 跟随视角
```

### 6.3 PCLA.get_action()

```python
def get_action(self):
    snapshot = self.world.get_snapshot()
    timestamp = snapshot.timestamp
    GameTime.on_carla_tick(timestamp)           # 更新 PCLA 侧游戏时钟
    return self.agent_instance(vehicle=self.vehicle)
```

### 6.4 AutonomousAgent.__call__()

```python
def __call__(self, sensors=None):
    input_data = self.sensor_interface.get_data()   # 从 CallBack 缓冲取传感器数据
    timestamp = GameTime.get_time()
    control = self.run_step(input_data, timestamp)  # ADS 推理：InterfuserAgent.run_step()
    control.manual_gear_shift = False
    return control                                  # carla.VehicleControl
```

---

## 七、完整调用链

```
ScenarioManager._tick_scenario(timestamp)
  │
  ├─ GameTime.on_carla_tick(timestamp)           # srunner 侧时钟
  ├─ CarlaDataProvider.on_carla_tick()
  │
  └─ scenario_tree.tick_once()
       │
       └─ UpdateAllActorControls.update()        # 遍历所有受控 Actor
            │
            └─ ActorControl.run_step()
                 │
                 └─ ExternalControl.run_step()
                      │
                      ├─ PCLA.get_action()
                      │    ├─ GameTime.on_carla_tick(timestamp)   # PCLA 侧时钟同步
                      │    └─ agent_instance()
                      │         ├─ sensor_interface.get_data()    # 读取传感器缓冲
                      │         └─ run_step(input_data, t)        # ADS 推理 → VehicleControl
                      │
                      └─ actor.apply_control(control)             # 下发控制
```

---

## 八、路线构建流程详解

```
demo.py: save_route(tf)
  └─ 写入 temp_route.xml（仅含终点 waypoint）

ExternalControl._setup_pcla(actor)
  ├─ 读 temp_route.xml 得到 end_location
  ├─ location_to_waypoint(client, start, end)
  │    └─ GlobalRoutePlanner.trace_route(start, end)  → 稀疏 waypoint 列表
  └─ route_maker(waypoints, "temp_route_for_external_control.xml")
       └─ 写成 leaderboard 路线 XML 格式

PCLA.setup_route()
  ├─ RouteIndexer 解析路线 XML → config.trajectory（carla.Location 列表）
  └─ interpolate_trajectory(world, trajectory, hop=1.0)
       ├─ GlobalRoutePlanner.trace_route() 段段连接，1m 分辨率
       ├─ 转换为 GPS 坐标（lat/lon）
       └─ 返回 (gps_route, world_coord_route)
            └─ agent.set_global_plan(gps_route, route)
                 └─ 降采样到每 50m 一个关键点存入 _global_plan
```

---

## 九、关键文件一览

| 文件 | 职责 |
|------|------|
| `PCLA/PCLA.py` | ADS 框架主类：加载 agent、构建路线、绑定传感器、获取决策 |
| `PCLA/pcla_functions/give_path.py` | 根据 agent 名+变体查 `agents.json` 返回脚本/配置路径 |
| `PCLA/pcla_functions/location_to_waypoint.py` | GlobalRoutePlanner 规划起终点路径 |
| `PCLA/pcla_functions/route_maker.py` | 将 waypoint 列表写成 leaderboard XML 格式 |
| `PCLA/leaderboard_codes/route_manipulation.py` | 稀疏路点插值（1m 精度）+ 世界坐标→GPS 转换 |
| `PCLA/leaderboard_codes/route_indexer.py` | 解析 route XML，生成 RouteScenarioConfiguration |
| `PCLA/leaderboard_codes/autonomous_agent_local.py` | ADS 基类：`sensors()` / `run_step()` / `set_global_plan()` |
| `PCLA/agents.json` | agent 名称 → (脚本路径, 配置路径) 的注册表 |
| `srunner/actorcontrols/external_control.py` | 桥接层：初始化 PCLA，每帧调用 `get_action()` 并 `apply_control()` |
| `srunner/actorcontrols/actor_control.py` | 控制器加载器：按模块名 import 并实例化对应控制类 |
| `srunner/scenarioatomics/atomic_behaviors.py` | `ChangeActorControl`（注册）+ `UpdateAllActorControls`（每帧驱动） |

---

## 十、当前默认 Agent

`ExternalControl.__init__` 中硬编码：

```python
self._agent_name = "if_if"   # interfuser，variant=if
```

对应 `agents.json` 中：
```json
"if": {
  "if": {
    "agent":  "pcla_agents/interfuser/interfuser_agent.py",
    "config": "pcla_agents/interfuser_pretrained/interfuser_baseline"
  }
}
```

要切换 ADS，修改 `self._agent_name` 即可，格式为 `<agent>_<variant>[_seed]`，如 `tfv6_regnet`、`carl_roach` 等。
