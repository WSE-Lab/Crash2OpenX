# Crash2OpenX

**From natural-language crash reports to executable OpenDRIVE/OpenSCENARIO
scenarios via model transformation.**

Crash2OpenX is a model-transformation toolchain that turns a natural-language
crash report (NHTSA / California DMV style, in PDF or plain text) into a pair
of executable simulation artifacts — an OpenDRIVE 1.5 road network and a
matching OpenSCENARIO 1.0 scene — and runs the pair in CARLA under an
autonomous-driving agent loaded via PCLA. This repository is the
implementation accompanying the MODELS 2026 Tools & Demonstrations submission.

## Architecture

The tool has three layers; every artifact that crosses a layer boundary is
validated first.

| Layer | What it does | Key modules |
|---|---|---|
| **Inference** | Normalizes the report (plain text / Markdown / DOCX directly; PDFs are read page-by-page by a VLM), then two LLM agents extract the coupled **RoadSeed** and **SceneSeed** models in parallel. Plain scripts enforce the OCL constraints of the paper's Table 1 (I1–I9 per model, P1–P6 on the pair); violations go back to the agents as structured fix hints. | `tools/extract_source.py`, `tools/api_infer_road_seed.py`, `tools/api_infer_scene_seed_v2.py`, `tools/ocl_constraints.py`, `tools/coordinator.py` |
| **Compilation** | Deterministic compilers: RoadSeed → XODR (topology-templated via `scenariogeneration`); the XODR is loaded into CARLA to resolve the lane graph and qualitative NPC positions; SceneSeed → XOSC. Both files pass their XSD gates plus cross-file checks, then a VLM judge reviews a visual replay of the pair against the source narrative before execution. | `tools/build_road_seed_opendrive.py`, `tools/osc_blocks.py` (WF gates), `runner/extract_roadgraph_carla.py`, `tools/qa_agent.py`, `tools/replay_scene_tools.py` |
| **Execution** | CARLA runs in **Docker on the same machine**; the tool connects as a client. The ego vehicle is driven by a PCLA-packaged leaderboard agent (TransFuser family, InterFuser, …) selected by one config value. The run records traces and metrics (TTC, max speed, min distance to the other actor) and an execution gate compares the observed outcome against the declared collision intent. | `docker/docker-compose.yml`, `runner/runner_local.sh`, `tools/carla_local.py`, `runner/src/runs/run_scene.py`, `tools/scene_outcome.py` |

Repository layout:

```
crash2openx/
├── run.sh                  # one-command launcher (CARLA docker + web demo)
├── tools/                  # pipeline: inference, compilers, gates, web demo, batch eval
├── runner/                 # CARLA-side execution stack (extract roadgraph, run scenario, record)
├── schemas/                # seed metamodel docs + derivation notes (NHTSA mapping)
├── xsd/                    # OpenDRIVE 1.5 + OpenSCENARIO 1.0 schemas (L1 gate)
├── docker/                 # CARLA simulation host (docker compose)
├── external/               # git submodules (see below)
├── examples/               # paper running example (Sec. 3) as frozen seeds
├── data/                   # frozen evaluation set: 42-pattern benchmark (seeds + compiled XODR/XOSC)
├── inputs/                 # sample crash reports (incl. demo case-165 PDF)
├── scripts/                # setup + reproduction utilities
└── tests/                  # static regression tests (no CARLA needed)
```

## Submodules

| Path | Project | Pin |
|---|---|---|
| `external/scenariogeneration` | [pyoscx/scenariogeneration](https://github.com/pyoscx/scenariogeneration) — OpenDRIVE/OpenSCENARIO generation backend | `debb300` (v0.16.5) |
| `external/PCLA` | [MasoudJTehrani/PCLA](https://github.com/MasoudJTehrani/PCLA) — Pretrained CARLA Leaderboard Agents | latest |
| `external/scenario_runner` | [carla-simulator/scenario_runner](https://github.com/carla-simulator/scenario_runner) — OpenSCENARIO execution engine | `v0.9.16` |

## Installation

### 0. Prerequisites

- **git**, **[uv](https://docs.astral.sh/uv/)** (`curl -LsSf https://astral.sh/uv/install.sh | sh` or `brew install uv`)
- **ffmpeg** (video encoding of CARLA runs)
- For the execution layer: a **Linux x86_64 host with an NVIDIA GPU**,
  [Docker Engine](https://docs.docker.com/engine/install/) and the
  [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
  (On a machine without a GPU — e.g. a macOS laptop — everything up to and
  including XSD-valid XODR/XOSC compilation still works; only the CARLA
  phases need the GPU host.)

### 1. Clone with submodules

```bash
git clone --recurse-submodules <repo-url> crash2openx
cd crash2openx
# or, after a plain clone:
git submodule update --init --recursive
```

### 2. Python environment (uv-managed)

The whole toolchain environment is managed by uv: dependencies are declared
in `pyproject.toml`, locked in `uv.lock`, and installed into `.venv/`.
`scenariogeneration` is installed editable from the pinned submodule.

```bash
uv sync
```

Run anything with `uv run` (activates `.venv` and resolves `tools.*` imports):

```bash
uv run pytest            # static regression suite, no CARLA required
```

### 3. API key for the LLM/VLM front end

Seed extraction and the QA judge call an OpenAI-compatible endpoint
(default: OpenRouter; models are set per script via `--model`).

```bash
cp .env.example .env.local
# edit .env.local: OPENROUTER_API_KEY=sk-or-...
```

### 4. CARLA via Docker (same machine)

The simulation host is a Docker container, reproducible with a single
command. One-time host setup on the GPU machine:

```bash
# 4.1 Install Docker Engine, then the NVIDIA Container Toolkit:
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 4.2 Sanity check: the GPU is visible inside containers
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi

# 4.3 Pull + start the CARLA 0.9.16 server (headless, ports 2000-2002)
docker compose -f docker/docker-compose.yml up -d

# 4.4 Verify the simulator answers
docker ps --filter name=carla-0916
uv run python -c "import carla" 2>/dev/null \
  || echo "carla client comes with step 5"
```

Notes:
- The compose file starts `CarlaUE4.sh -RenderOffScreen -nosound` and caps
  container logs at 3×100 MB (a long-running CARLA can otherwise flood the
  Docker json log).
- `runner/runner_local.sh` manages the container automatically afterwards:
  it starts it when stopped and restarts it when the RPC stream pool
  degrades (a known long-uptime CARLA failure mode).

### 5. Execution environment (PCLA agents)

The execution layer needs PCLA's pinned dependency set (Python 3.10,
`carla==0.9.16`, torch, …). It lives in a second uv-managed venv so the main
environment stays light:

```bash
scripts/setup_exec_env.sh              # creates .venv-exec from PCLA's pins
uv run --python .venv-exec/bin/python \
    python external/PCLA/pcla_functions/download_weights.py   # pretrained agent weights
```

`runner/runner_local.sh` picks `.venv-exec/bin/python` automatically when it
exists.

## Running

### One command

```bash
./run.sh              # uv sync + CARLA container up + web demo on :5000
./run.sh --no-carla   # compile-only machine (no docker/GPU)
```

Open `http://127.0.0.1:5000`, paste a crash narrative (or upload a PDF, e.g.
`inputs/165_Mercedes_Benz_August_18_2023.pdf`), pick the PCLA agent, and
watch the pipeline stages stream in over SSE: extraction → OCL gate → XODR →
QA judge → XOSC → CARLA execution → intent verdict, with the replay and the
recorded CARLA video at the end.

### CLI

```bash
# Full pipeline on one report (same phases as the browser demo):
uv run python tools/coordinator.py --input inputs/demo_case165.txt --name case165

# Reproduce artifacts from a frozen seed pair (no LLM call, paper Sec. 3 example):
uv run python scripts/compile_from_seeds.py \
    --road examples/rainy_night_front_brake/road_seed.json \
    --scene examples/rainy_night_front_brake/scene_seed.json \
    --out outputs/repro/rainy_night

# Execute one compiled pair in the Dockerized CARLA under a PCLA agent:
uv run python tools/carla_local.py run \
    --xodr outputs/repro/rainy_night/rainy_night.xodr \
    --xosc outputs/repro/rainy_night/rainy_night.xosc \
    --pcla-agent if_if            # InterFuser; see external/PCLA/agents.json

# Batch-run the frozen 42-pattern evaluation set (data/eval, artifacts in
# data/compiled) and collect execution-gate verdicts:
uv run python tools/batch_run_medoids.py

# Direct-LLM baseline (prompt the same model for raw XML, Sec. 5 of the paper):
uv run python tools/baseline_direct_llm.py
```

The legacy ssh driver for a remote CARLA host is still available with
`CARLA_MODE=remote` (see `.env.example`).

## Validation layers

1. **OCL Table 1** — `tools/ocl_constraints.py`: I1–I9 per-model invariants at
   extraction return, P1–P6 pair rules before compilation; violations are fed
   back to the extraction agents as textual hints (bounded retry loop).
2. **XSD** — every emitted file is validated against
   `xsd/OpenDRIVE_1.5M.xsd` / `xsd/OpenSCENARIO.xsd`; schema violations abort
   compilation.
3. **Cross-metamodel WF gates** — `tools/osc_blocks._check_wf` and the road
   graph builder raise `WFViolation` on road×scene combinations that are
   individually valid but jointly meaningless (oncoming NPC on a one-way
   road, overtake across a solid centerline, …).
4. **VLM judge** — before execution, a visual replay of the compiled pair is
   rendered and judged against the source narrative; rejections return
   seed-level fix suggestions.
5. **Execution gate** — after the CARLA run, the observed trajectory and
   collision events are compared against the declared `collision(striker,
   struck)` intent (`tools/scene_outcome.py`).

All gate regressions are covered by the static test suite:

```bash
uv run pytest    # 43 tests, < 30 s, no CARLA and no API key needed
```

## Troubleshooting

- **`carla` import fails on macOS** — expected: the CARLA client wheel is
  Linux/Windows only. Compilation, XSD validation, the OCL suite and the web
  demo (up to the execution phases) all run without it.
- **First scenario on a cold CARLA times out** — complex generated maps can
  take >30 s to bake; the runner already waits 90 s and retries once on the
  silent-spawn pattern.
- **`carl_*` agents crash at the first tick** — the runner pre-bakes the
  required BEV raster automatically; make sure the run goes through
  `runner/runner_local.sh` (or `tools/carla_local.py`) rather than calling
  `run_scene.py` directly.
