# CARLA 无头服务器可视化框架

## 核心思路

CARLA server 可以无头运行，场景脚本继续负责推进仿真；另起一个轻量观察客户端连接同一个 `host:port` 做可视化。这样不会改动场景执行流程，也不会依赖 CARLA 服务端窗口。

这里提供的入口是：

```bash
python src/visualize_carla.py --host localhost --port 2000 --actor-role hero
```

## 两种模式

### 1. RGB 相机模式

适合服务端无显示器，但仍允许 GPU/offscreen 渲染的情况。

```bash
python src/visualize_carla.py \
  --host localhost \
  --port 2000 \
  --mode rgb \
  --actor-role hero \
  --width 1280 \
  --height 720
```

这个模式会在 `role_name=hero` 的 actor 上挂一个 `sensor.camera.rgb`，在客户端用 pygame 显示画面。

注意：如果运行场景时用了 `--no-rendering`，CARLA world 的 `no_rendering_mode=True`，RGB 相机通常会没有有效画面。此时要用 topdown 模式。

### 2. Top-down 状态模式

适合真正的 `no_rendering_mode=True`。它不依赖 CARLA 渲染器，只读取 actor 的位置、朝向、类型，在客户端画一个俯视图。

```bash
python src/visualize_carla.py \
  --host localhost \
  --port 2000 \
  --mode topdown \
  --actor-role hero \
  --topdown-range 90
```

## 推荐运行方式

### 一键运行并录真实 CARLA 画面

```bash
./scripts/run_headless_scene_record.sh
```

默认输出：

```text
runs/v2/001_Zoox_April_11_2025_<timestamp>/carla_rgb.mp4
```

这个视频来自 `sensor.camera.rgb` 的真实 CARLA 渲染画面。脚本默认按 `run_v2.py -1` 的逻辑跑：

- 场景：`v2/001_Zoox_April_11_2025.xosc`
- 地图：`v2/001_Zoox_April_11_2025.xodr`
- ADS：`--pcla-sut --agent tfv6_regnet --sut-actor v2`

脚本默认不会给 `demo.py` 传 `--no-rendering`，因为那会让 RGB 相机没有有效画面。

常用参数：

```bash
V2_CASE_INDEX=2 ./scripts/run_headless_scene_record.sh
RGB_WIDTH=1920 RGB_HEIGHT=1080 RGB_FPS=30 ./scripts/run_headless_scene_record.sh
SCENARIO=/path/to/a.xosc XODR=/path/to/a.xodr ./scripts/run_headless_scene_record.sh
```

相机默认放在 ADS 车后上方，参数是 `RGB_CAMERA_X=-16.0 RGB_CAMERA_Z=9.0 RGB_CAMERA_PITCH=-28.0`。如果想更高：

```bash
RGB_CAMERA_X=-22 RGB_CAMERA_Z=14 RGB_CAMERA_PITCH=-35 ./scripts/run_headless_scene_record.sh
```

如果还想同时生成原来的抽象轨迹俯视视频：

```bash
DEMO_RECORD_TRAJECTORY=1 ./scripts/run_headless_scene_record.sh
```

终端 1：启动无头 CARLA server。不要开 `no_rendering_mode` 时，RGB 可用；开了 `no_rendering_mode` 时，只用 topdown。

终端 2：运行场景。

```bash
python src/demo.py --case 001 --host localhost --port 2000 --real-time-factor 1.0
```

终端 3：运行可视化客户端。

```bash
python src/visualize_carla.py --host localhost --port 2000 --mode rgb --actor-role hero
```

或：

```bash
python src/visualize_carla.py --host localhost --port 2000 --mode topdown --actor-role hero
```

## 保存画面

两个模式都可以保存帧：

```bash
python src/visualize_carla.py \
  --mode topdown \
  --save-dir /tmp/carla_vis_frames \
  --save-every 5
```

## 参数速查

- `--mode rgb|topdown`：选择 RGB 传感器画面或无渲染俯视图。
- `--actor-role hero`：跟随哪个 actor 的 `role_name`。
- `--spectator`：同时更新 CARLA spectator。服务端无窗口时不是必需，但如果后续接 VNC/Pixel Streaming 会有用。
- `--fps 20`：客户端显示刷新率。
- `--save-dir DIR`：把显示帧保存到目录。

## 关键限制

- “CARLA 无头运行”不等于 `no_rendering_mode=True`。无头/offscreen 仍可渲染 RGB；`no_rendering_mode=True` 是主动关闭渲染，RGB 摄像头不可作为主可视化来源。
- 这个脚本只是观察客户端。它会 spawn 一个相机 sensor，因此 RGB 模式会轻微增加服务端负载；topdown 模式负载很低。
