#!/usr/bin/env bash
# Single remote entry point invoked by tools/carla_remote.py over SSH.
#
#   runner.sh extract <run_id> [-- extra flags forwarded to extract_roadgraph_carla.py]
#   runner.sh run     <run_id> [-- extra flags forwarded to src/demo.py]
#
# Layout per run on the remote:
#   $RUNS_ROOT/$RUN_ID/inputs/   <- client uploads map.xodr (+ scenario.xosc for run mode)
#   $RUNS_ROOT/$RUN_ID/outputs/  <- script writes here; client scp -r back
#   $RUNS_ROOT/$RUN_ID/runner.log
set -Eeuo pipefail

MODE="${1:?need mode (extract|run)}"
RUN_ID="${2:?need run_id}"
shift 2

RUNS_ROOT="${RUNS_ROOT:-${HOME}/crash2openx_runs}"
RUN_DIR="${RUNS_ROOT}/${RUN_ID}"
INPUT_DIR="${RUN_DIR}/inputs"
OUTPUT_DIR="${RUN_DIR}/outputs"
LOG="${RUN_DIR}/runner.log"

PROJECT_DIR="${PROJECT_DIR:-${HOME}/leaderboard_2.0}"
PYTHON="${PYTHON:-${HOME}/miniconda3/envs/PCLA/bin/python}"
CONTAINER="${CONTAINER:-carla-0916}"
CARLA_PORT="${CARLA_PORT:-2000}"

mkdir -p "${INPUT_DIR}" "${OUTPUT_DIR}"
exec > >(tee -a "${LOG}") 2>&1

echo "==== runner.sh mode=${MODE} run_id=${RUN_ID} ts=$(date -Iseconds) ===="

ensure_carla() {
  local state
  state=$(docker inspect "${CONTAINER}" --format '{{.State.Status}}' 2>/dev/null || echo missing)
  if [[ "${state}" != "running" ]]; then
    echo "Starting CARLA container ${CONTAINER} (was ${state})..."
    docker start "${CONTAINER}" >/dev/null
  fi
  _wait_for_port_and_rpc || return 1
  # Even after the cheap get_server_version ping passes, the simulator can be in
  # a long-running degraded state (stream-pool exhaustion after ~8h of uptime
  # manifests as `Invalid session: no stream available` floods + 30s timeouts on
  # generate_opendrive_world). Probe the real RPC the caller will issue next.
  if ! _probe_world_gen; then
    echo "CARLA RPC pings ok but world-gen probe failed; restarting container ${CONTAINER}..."
    docker restart "${CONTAINER}" >/dev/null
    # After restart the simulator needs 20-40s to finish the UE4 boot before
    # generate_opendrive_world can succeed. Sleep, then re-wait for port + RPC.
    sleep 30
    _wait_for_port_and_rpc || return 1
    # Give the freshly-booted server up to 3 attempts (with backoff) before
    # giving up — a single restart sometimes lands while UE4 is still loading
    # the default Town map and the first world-gen call still times out.
    local probe_attempt
    for probe_attempt in 1 2 3; do
      if _probe_world_gen; then
        echo "CARLA recovered after restart (probe ${probe_attempt})"
        return 0
      fi
      echo "world-gen probe ${probe_attempt}/3 still failing; sleeping 20s..."
      sleep 20
    done
    echo "ERROR: world-gen probe still failing after restart" >&2
    return 1
  fi
  return 0
}

_wait_for_port_and_rpc() {
  for i in $(seq 1 24); do
    if (ss -tln 2>/dev/null || netstat -tln 2>/dev/null) | grep -qE ":${CARLA_PORT}\b"; then
      # Port up != server ready. Ping with the PCLA carla 0.9.16 client which
      # matches the simulator version; get_server_version is a cheap RPC that
      # fails if the world hasn't finished spinning up yet.
      if "${PYTHON}" -c "import carla; c=carla.Client('127.0.0.1', ${CARLA_PORT}); c.set_timeout(5.0); c.get_server_version()" >/dev/null 2>&1; then
        echo "CARLA port ${CARLA_PORT} ready (server pings back)"
        return 0
      fi
    fi
    sleep 5
  done
  echo "ERROR: CARLA port ${CARLA_PORT} not listening after 120s" >&2
  return 1
}

_probe_world_gen() {
  # Cheap-ish liveness check: regenerate an empty OpenDRIVE world. This is the
  # exact RPC that fails first when the stream pool degrades, so it's a good
  # proxy for "the next real run won't time out at warmup."
  "${PYTHON}" -u - "${CARLA_PORT}" >/dev/null 2>&1 <<'PY'
import sys, carla
client = carla.Client("127.0.0.1", int(sys.argv[1])); client.set_timeout(20.0)
xodr = (
  '<?xml version="1.0" encoding="UTF-8"?>'
  '<OpenDRIVE><header revMajor="1" revMinor="5" name="" version="1.00"/>'
  '<road name="r" length="20.0" id="1" junction="-1">'
  '<planView><geometry s="0" x="0" y="0" hdg="0" length="20.0"><line/></geometry></planView>'
  '<lanes><laneSection s="0"><left><lane id="1" type="driving" level="false">'
  '<width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></left>'
  '<center><lane id="0" type="none" level="false"/></center>'
  '<right><lane id="-1" type="driving" level="false">'
  '<width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right>'
  '</laneSection></lanes></road></OpenDRIVE>'
)
params = carla.OpendriveGenerationParameters(
    vertex_distance=2.0, max_road_length=500.0, wall_height=0.0,
    additional_width=0.6, smooth_junctions=True, enable_mesh_visibility=False,
)
client.generate_opendrive_world(xodr, params)
PY
}

ensure_carla

case "${MODE}" in
  extract)
    XODR="${INPUT_DIR}/map.xodr"
    [[ -f "${XODR}" ]] || { echo "missing ${XODR}" >&2; exit 2; }
    cd "${PROJECT_DIR}"
    "${PYTHON}" -u extract_roadgraph_carla.py \
        --xodr "${XODR}" \
        --out  "${OUTPUT_DIR}" \
        --host 127.0.0.1 \
        --port "${CARLA_PORT}" \
        "$@"
    ;;
  run)
    XODR="${INPUT_DIR}/map.xodr"
    XOSC="${INPUT_DIR}/scenario.xosc"
    [[ -f "${XODR}" && -f "${XOSC}" ]] || { echo "missing inputs in ${INPUT_DIR}" >&2; exit 2; }
    cd "${PROJECT_DIR}"

    # Parse our wrapper-level flags out of "$@". Pass-through demo args are dropped
    # because src/runs/run_scene.py owns the demo.py invocation now.
    SUT_ACTOR=""
    PCLA_AGENT="tfv6_regnet"
    RGB_ACTOR_ROLE=""
    PAPER_RENDER=0
    MAX_SECONDS=60
    while (( "$#" )); do
      case "$1" in
        --sut-actor) SUT_ACTOR="$2"; shift 2 ;;
        --pcla-agent) PCLA_AGENT="$2"; shift 2 ;;
        --rgb-actor-role) RGB_ACTOR_ROLE="$2"; shift 2 ;;
        --paper-render) PAPER_RENDER=1; shift ;;
        --max-seconds) MAX_SECONDS="$2"; shift 2 ;;
        --no-rendering|--pcla-sut|--agent|--record-rgb|--record-video) shift ;;
        *) shift ;;
      esac
    done

    # Warmup: docker-fresh CARLA defaults to Town01, where host 0.9.15 client's get_world()
    # times out under version mismatch. generate_opendrive_world via PCLA 0.9.16 client leaves
    # the server on OpenDriveMap, after which detection in run_headless_scene_record.sh works.
    # Timeout 90s (was 30s): complex generated OpenDRIVE (t_junction with forward=2/backward=2,
    # fork topology, etc.) needs >30s to bake on a cold CARLA — observed 2026-06-26 with 153
    # (4-lane t_junction) and 246 (fork) hitting `time-out of 30000ms while waiting for the
    # simulator` even after server restart. Bumping the cap is cheap and only stretches the
    # ceiling — fast maps still warmup in <5s.
    "${PYTHON}" -u - "${XODR}" "${CARLA_PORT}" <<'PY'
import sys, carla
xodr_path, port = sys.argv[1], int(sys.argv[2])
client = carla.Client("127.0.0.1", port); client.set_timeout(90.0)
with open(xodr_path) as f:
    xodr = f.read()
params = carla.OpendriveGenerationParameters(
    vertex_distance=2.0, max_road_length=500.0, wall_height=0.0,
    additional_width=0.6, smooth_junctions=True, enable_mesh_visibility=False,
)
client.generate_opendrive_world(xodr, params)
print("warmup OK", flush=True)
PY

    # Roach/CaRL (carl_*) agents need a pre-baked BEV raster of the CURRENT map:
    # bev_observation.py opens maps_low_res/<map.name>.h5 at first tick and dies
    # with FileNotFoundError otherwise (observed 2026-07-03, whole roach batch).
    # Official towns ship pre-baked; our per-case OpenDRIVE loads as
    # "OpenDriveMap", so bake it here right after warmup while the world is live.
    if [[ "${PCLA_AGENT}" == carl_* ]]; then
      "${PYTHON}" -u - "${CARLA_PORT}" "${PROJECT_DIR}" <<'PY'
import importlib, os, sys
import carla, h5py
port, project_dir = int(sys.argv[1]), sys.argv[2]
carl_dir = os.path.join(project_dir, "PCLA/pcla_agents/carl")
bev_dir = os.path.join(carl_dir, "birds_eye_view")
# carl_dir: traffic_light.py imports `birds_eye_view.transforms` (package-style);
# bev_dir: birdview_map_opencv.py imports `traffic_light` (flat).
sys.path.insert(0, carl_dir)
sys.path.insert(0, bev_dir)
client = carla.Client("127.0.0.1", port); client.set_timeout(60.0)
world = client.get_world()
cmap = world.get_map()
name = cmap.name.rsplit("/", 1)[-1]
mod = importlib.import_module("birds_eye_view.birdview_map_opencv")
try:
    from traffic_light import TrafficLightHandler
    TrafficLightHandler.reset(world)
except Exception as exc:  # generated maps have no signals; stopline layer stays empty
    print(f"TrafficLightHandler reset skipped: {exc}", flush=True)
masks = mod.MapImage.draw_map_image(cmap, 5.0)
out = os.path.join(bev_dir, "maps_low_res", name + ".h5")
with h5py.File(out, "w") as hf:
    hf.attrs["pixels_per_meter"] = 5.0
    hf.attrs["world_offset_in_meters"] = masks["world_offset"]
    hf.attrs["width_in_meters"] = masks["width_in_meters"]
    hf.attrs["width_in_pixels"] = masks["width_in_pixels"]
    for k in ("road", "shoulder", "parking", "sidewalk", "stopline",
              "lane_marking_all", "lane_marking_yellow_broken",
              "lane_marking_yellow_solid", "lane_marking_white_broken",
              "lane_marking_white_solid"):
        hf.create_dataset(k, data=masks[k], compression="gzip", compression_opts=4)
print(f"BEV h5 baked: {out} ({masks['width_in_pixels']}px)", flush=True)
PY
    fi

    # Activate PCLA env then delegate to src/runs/run_scene.py. process_timeout caps wall
    # clock; SIGTERM trap in run_headless_scene_record.sh still encodes captured frames into
    # carla_rgb.mp4 even on timeout. Artifacts land in $OUTPUT_DIR/scene/out/.
    PROCESS_TIMEOUT=$(( MAX_SECONDS + 10 ))
    SCENE_OUT="${OUTPUT_DIR}/scene"
    mkdir -p "${SCENE_OUT}"
    # Derive conda.sh from the PCLA env interpreter (…/miniconda3/envs/PCLA/bin/python
    # -> …/miniconda3/etc/profile.d/conda.sh); override with CONDA_SH if needed.
    source "${CONDA_SH:-${PYTHON%%/envs/*}/etc/profile.d/conda.sh}"
    conda activate PCLA
    # PCLA_AGENT is propagated to src/runs/run_scene.py via the AGENT env var
    # — run_scene.py's argparse has no --agent flag (verified 2026-06-28),
    # but it forwards env["AGENT"] through to PCLA.PCLA(). Without this
    # export, every run silently used the default tfv6_regnet regardless of
    # what the client passed. Empty PCLA_AGENT falls back to tfv6_regnet
    # (the function-arg default in run_scene.run_scene).
    export AGENT="${PCLA_AGENT}"
    # Dense route samples materially reduce corner-cutting on generated arcs.
    # The downstream recorder keeps its own default for legacy callers; this
    # pipeline explicitly requests 2 m spacing to match roadgraph extraction.
    export SUT_ROUTE_SPACING="${SUT_ROUTE_SPACING:-2.0}"

    PAPER_FRAME_DIR="${OUTPUT_DIR}/paper_frames"
    PAPER_LOG="${OUTPUT_DIR}/paper_recorder.log"
    PAPER_VIDEO="${OUTPUT_DIR}/carla_paper.mp4"
    PAPER_PID=""

    stop_paper_recorder() {
      if [[ -n "${PAPER_PID}" ]] && kill -0 "${PAPER_PID}" >/dev/null 2>&1; then
        kill "${PAPER_PID}" >/dev/null 2>&1 || true
        wait "${PAPER_PID}" >/dev/null 2>&1 || true
      fi
      PAPER_PID=""
    }

    encode_paper_video() {
      if [[ "${PAPER_RENDER}" != "1" || ! -d "${PAPER_FRAME_DIR}" ]]; then
        return
      fi
      local frame_count
      frame_count=$(find "${PAPER_FRAME_DIR}" -maxdepth 1 -name 'frame_*.jpg' | wc -l)
      if (( frame_count == 0 )); then
        echo "paper recorder produced no frames; inspect ${PAPER_LOG}" >&2
        return
      fi
      ffmpeg -y -framerate 20 -start_number 0 \
        -i "${PAPER_FRAME_DIR}/frame_%06d.jpg" \
        -c:v libx264 -crf 18 -pix_fmt yuv420p "${PAPER_VIDEO}" >>"${PAPER_LOG}" 2>&1 || true
      # The final frame often follows actor teardown.  The temporal midpoint is
      # a more stable paper still: both the SUT and lead NPC are normally alive
      # and the interaction has developed enough to be visually legible.
      local still_index still_frame
      still_index=$(( frame_count / 2 ))
      still_frame=$(printf '%s/frame_%06d.jpg' "${PAPER_FRAME_DIR}" "${still_index}")
      [[ -f "${still_frame}" ]] && cp "${still_frame}" "${OUTPUT_DIR}/carla_paper.png"
      echo "paper render produced ${frame_count} frames + ${PAPER_VIDEO}"
    }

    if [[ "${PAPER_RENDER}" == "1" ]]; then
      [[ -f "${INPUT_DIR}/paper_meta.json" ]] || { echo "missing paper_meta.json" >&2; exit 2; }
      mkdir -p "${PAPER_FRAME_DIR}"
      SDL_VIDEODRIVER=dummy "${PYTHON}" -u "${PROJECT_DIR}/src/visualize_carla_paper.py" \
        --host 127.0.0.1 \
        --port "${CARLA_PORT}" \
        --actor-role "${RGB_ACTOR_ROLE:-${SUT_ACTOR}}" \
        --width 1920 \
        --height 1080 \
        --fps 20 \
        --meta "${INPUT_DIR}/paper_meta.json" \
        --save-dir "${PAPER_FRAME_DIR}" >"${PAPER_LOG}" 2>&1 &
      PAPER_PID=$!
      echo "paper recorder started (pid=${PAPER_PID})"
    fi

    set +e
    python -u "${PROJECT_DIR}/src/runs/run_scene.py" \
        --map "${XODR}" \
        --scenario "${XOSC}" \
        --output-root "${SCENE_OUT}" \
        --run-name out \
        --port "${CARLA_PORT}" \
        --timeout "${MAX_SECONDS}" \
        --process-timeout "${PROCESS_TIMEOUT}" \
        --require-existing-carla \
        --no-conda \
        ${SUT_ACTOR:+--sut-actor "${SUT_ACTOR}"} \
        ${RGB_ACTOR_ROLE:+--rgb-actor-role "${RGB_ACTOR_ROLE}"}
    SCENE_RC=$?
    set -e
    stop_paper_recorder
    encode_paper_video

    # Promote artifacts up to ${OUTPUT_DIR}/ so client's scp pulls a flat layout.
    if [[ -d "${SCENE_OUT}/out" ]]; then
      mv "${SCENE_OUT}/out"/* "${OUTPUT_DIR}/" 2>/dev/null || true
      rmdir "${SCENE_OUT}/out" 2>/dev/null || true
      rmdir "${SCENE_OUT}" 2>/dev/null || true
    fi

    # run_scene returns non-zero on process_timeout (which is expected by design); accept
    # the run as long as a meaningful artifact set was produced. carla_rgb.mp4 alone is
    # not enough — PCLA can crash early after recording a handful of frames; require
    # summary.json with total_ticks > 0 so a sut-actor mismatch fails loudly instead
    # of getting rubber-stamped.
    if [[ -f "${OUTPUT_DIR}/carla_rgb.mp4" && -f "${OUTPUT_DIR}/summary.json" ]]; then
      TICKS=$("${PYTHON}" -c "import json,sys; print(json.load(open('${OUTPUT_DIR}/summary.json')).get('total_ticks', 0))" 2>/dev/null || echo 0)
      if (( TICKS > 0 )); then
        echo "run_scene produced carla_rgb.mp4 + summary.json (ticks=${TICKS}, rc=${SCENE_RC} accepted)"
        exit 0
      fi
      echo "summary.json present but total_ticks=${TICKS}; treating as failure"
    else
      echo "missing carla_rgb.mp4 or summary.json; treating as failure"
    fi
    exit ${SCENE_RC}
    ;;
  *)
    echo "unknown mode: ${MODE}" >&2
    exit 2
    ;;
esac

echo "==== runner.sh done rc=$? ===="
