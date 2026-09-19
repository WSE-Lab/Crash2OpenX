# Reproduce Crash2OpenX

## 1. Static checks and road generation (no GPU or API key)

```sh
git clone --recurse-submodules https://github.com/WSE-Lab/Crash2OpenX.git
cd Crash2OpenX
uv sync --locked --python 3.12
uv run --locked python -m pytest -q
uv run python tools/build_road_seed_opendrive.py \
  --seed examples/rainy_night_front_brake/road_seed.json \
  --xodr outputs/quickstart/map.xodr --html outputs/quickstart/map.html
```

The main environment uses Python 3.12+. The CARLA/PCLA execution environment
uses Python 3.10 as required by its upstream pins. Full scene grounding uses
CARLA's realized waypoints, so road-only compilation is the offline quickstart.

## 2. Build the isolated execution runtime

On a Linux NVIDIA GPU host, install Docker and the NVIDIA Container Toolkit as
explained in the README. Run:

```sh
scripts/setup_exec_env.sh
uv run python -m tools.prepare_runtime --output outputs/runtime
docker compose -f docker/docker-compose.yml up -d
```

`prepare_runtime` exports the pinned PCLA and ScenarioRunner sources, applies the
reviewed patch and verifies 13 patched dependency files against the recorded
runtime hashes. It also installs the current first-party runner and helper modules.
It refuses an existing output directory and never edits the submodules. Choose a
new output directory when rebuilding. The patch's source and base revisions are
in `runner/patches/manifest.json`.

Download pretrained weights following `outputs/runtime/PCLA/README.md`, placing
them in that runtime's PCLA tree. They are not bundled with the source release.
Set the following in `.env.local`, using your actual absolute runtime directory:

```ini
CARLA_MODE=local
CARLA_LOCAL_PROJECT_DIR=/absolute/path/to/Crash2OpenX/outputs/runtime
CARLA_LOCAL_MIN_FREE_GIB=10
```

For a remote host, build the runtime on that host, then set
`CARLA_REMOTE_PROJECT_DIR` to its isolated runtime and configure the other
`CARLA_REMOTE_*` values from `.env.example`. `CARLA_REMOTE_RUNS_ROOT` must be your
own dedicated directory. Deploy the wrapper with:

```sh
uv run python tools/carla_remote.py deploy-runner
```

The remote driver checks available bytes and inodes before uploading inputs.
It stops below the configured reserve and does not free space by deleting files.
Run evidence is retained by default. Never prune shared containers or delete
another user's files to make space.

## 3. Replay a portable scenario

Download the presentation/evidence release and unzip it. Select a case directory
under `evidence/latest_cases/`. Its `scenario.xosc` references `map.xodr` relatively.

```sh
CASE_DIR=/absolute/path/to/evidence/latest_cases/013_Zoox_February_19_2025
uv run python tools/carla_local.py run \
  --xodr "$CASE_DIR/map.xodr" --xosc "$CASE_DIR/scenario.xosc" \
  --pcla-agent if_if --sut-actor hero --rgb-actor-role hero --max-seconds 600 \
  --scene-seed "$CASE_DIR/scene_seed.json" --road-seed "$CASE_DIR/source/road_seed.json"
```

For SSH transport, use `tools/carla_remote.py` with the same run arguments.
Read each case's `manifest.json` and `quality_review.json` for source/variant
parameters and the measured interval. A new run may yield different ADS traces.

## 4. Generate from a new report

Copy `.env.example` to `.env.local` and supply your own model API key. The template
uses the configured DeepSeek-compatible endpoint. Set `MODEL_BASE_URL`,
`MODEL_API_KEY_ENV`, `TEXT_MODEL` and `VLM_MODEL` to use another supported endpoint.
Then run `./run.sh --no-carla` to start only the web client when the simulator is
already managed separately, or use `tools/coordinator.py` from the README.

The browser streams extraction, seed checks, road generation, scene grounding,
validation, simulation and recorded results. Archived demonstration assets are
recorded evidence, not a claim that a new model invocation returns identical seeds.
