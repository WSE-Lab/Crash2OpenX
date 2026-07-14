# CARLA 数据收集器使用指南

## 概述

这个数据收集器可以在CARLA仿真运行时收集详细的数据，包括所有车辆和行人的位置、速度、控制命令等信息。数据最终会保存为JSON格式文件。

## 文件说明

- `data_collector.py` - 核心数据收集器类
- `demo.py` - 修改后的demo，支持数据收集
- `test_data_collection.py` - 独立的测试脚本
- `scenario_runner_local.py` - 修改后支持数据收集器
- `scenario_manager_local.py` - 修改后在每次tick时收集数据

## 使用方法

### 1. 通过demo.py使用

```bash
# 启用数据收集
python demo.py --collect-data --data-output my_data.json

# 不启用数据收集（正常运行）
python demo.py
```

### 2. 通过测试脚本使用

```bash
# 基本测试
python test_data_collection.py --collect-data

# 指定场景文件和输出文件
python test_data_collection.py \
    --scenario /path/to/your/scenario.xosc \
    --collect-data \
    --data-output my_test_data.json
```

### 3. 在自己的代码中使用

```python
from data_collector import DataCollector
from scenario_runner_local import ScenarioRunner

# 创建数据收集器
data_collector = DataCollector("my_scenario")

# 创建ScenarioRunner
scenario_runner = ScenarioRunner(args, xml_tree)

# 传递数据收集器
scenario_runner.set_data_collector(data_collector)

# 运行场景
result = scenario_runner.run()

# 保存数据
data_collector.save_data("output.json")
```

## 数据格式

输出的JSON文件包含以下结构：

```json
{
  "scenario_info": {
    "scenario_name": "FollowLeadingVehicle",
    "town_name": "Town01",
    "start_location": {"x": 100.0, "y": 200.0, "z": 0.5},
    "ego_vehicle_type": "vehicle.tesla.model3",
    "weather": {
      "cloudiness": 0.0,
      "precipitation": 0.0,
      "wind_intensity": 0.0
    },
    "start_timestamp": 1638360000.0
  },
  "frames": [
    {
      "frame": 0,
      "timestamp": 1638360001.0,
      "simulation_time": 10.5,
      "actors": [
        {
          "id": 123,
          "type": "vehicle",
          "type_id": "vehicle.tesla.model3",
          "location": {"x": 100.0, "y": 200.0, "z": 0.5},
          "rotation": {"pitch": 0.0, "yaw": 90.0, "roll": 0.0},
          "velocity": {"x": 10.0, "y": 0.0, "z": 0.0},
          "acceleration": {"x": 1.0, "y": 0.0, "z": 0.0},
          "is_hero": true,
          "control": {
            "throttle": 0.5,
            "steer": 0.0,
            "brake": 0.0,
            "hand_brake": false,
            "reverse": false
          }
        }
      ]
    }
  ],
  "summary": {
    "total_frames": 150,
    "duration_seconds": 15.0,
    "fps": 10.0
  }
}
```

## 收集的数据类型

### 场景信息 (scenario_info)
- 场景名称
- 城镇名称
- 起始位置
- 主角车辆类型
- 天气条件
- 开始时间戳

### 每帧数据 (frames)
- 帧编号
- 时间戳
- 仿真时间
- 所有Actor的信息：
  - ID和类型
  - 位置和旋转
  - 速度和加速度
  - 控制命令（车辆）
  - 是否为主角车辆

### 统计摘要 (summary)
- 总帧数
- 持续时间
- 平均FPS

## 技术细节

### 数据收集流程

1. **初始化**: 在ScenarioRunner启动时创建DataCollector
2. **场景初始化**: 收集场景基本信息（城镇、天气、起点等）
3. **实时收集**: 在每次tick时收集所有Actor数据
4. **保存数据**: 场景结束时保存为JSON文件

### 集成方式

数据收集器通过以下方式集成到现有代码中：

1. **ScenarioRunner**: 添加了`set_data_collector()`方法
2. **ScenarioManager**: 在`_tick_scenario()`中调用数据收集
3. **最小侵入**: 不影响现有逻辑，只是添加数据收集功能

### 性能考虑

- 数据收集在每次tick时进行，频率与仿真频率一致
- 跳过传感器等不必要的Actor以减少数据量
- 数据存储在内存中，场景结束时一次性保存

## 故障排除

### 常见问题

1. **CARLA连接失败**
   - 确保CARLA服务器正在运行
   - 检查host和port参数

2. **场景文件无法加载**
   - 检查场景文件路径是否正确
   - 确保XML文件格式正确

3. **数据文件无法保存**
   - 检查输出目录权限
   - 确保磁盘空间充足

### 调试技巧

1. 查看控制台输出了解收集进度
2. 检查生成的JSON文件结构
3. 使用较短的场景进行测试

## 示例命令

```bash
# 基本使用
python demo.py --collect-data

# 指定输出文件
python demo.py --collect-data --data-output my_scenario_data.json

# 使用自定义场景
python test_data_collection.py \
    --scenario /path/to/custom_scenario.xosc \
    --collect-data \
    --data-output custom_data.json \
    --host localhost \
    --port 2000

# 不启用数据收集（正常运行）
python demo.py
```

## 扩展功能

可以通过修改`DataCollector`类来添加更多功能：

1. **自定义数据字段**: 在`_extract_actor_data()`中添加更多信息
2. **数据过滤**: 只收集特定类型的Actor
3. **实时保存**: 定期保存数据而不是最后一次性保存
4. **数据压缩**: 减少文件大小
5. **可视化输出**: 生成图表或动画

## 依赖项

- Python 3.6+
- CARLA Python API
- 标准库：json, time, os
