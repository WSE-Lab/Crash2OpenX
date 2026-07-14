#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CARLA_ROOT="${CARLA_ROOT:-/home/server/carla}"
CARLA_SH="${CARLA_SH:-${CARLA_ROOT}/CarlaUE4.sh}"
HOST="${HOST:-localhost}"
PORT="${PORT:-2000}"
TM_PORT="${TM_PORT:-8000}"
CASE="${CASE:-}"
SCENARIO="${SCENARIO:-}"
XODR="${XODR:-}"
V2_CASE_INDEX="${V2_CASE_INDEX:-1}"
AGENT="${AGENT:-tfv6_regnet}"
SUT_ACTOR="${SUT_ACTOR:-}"
SUT_SPAWN_Z="${SUT_SPAWN_Z:-0.5}"
SUT_START_OFFSET="${SUT_START_OFFSET:-0.0}"
SUT_ROUTE_MODE="${SUT_ROUTE_MODE:-endpoints}"
SUT_ROUTE_SPACING="${SUT_ROUTE_SPACING:-5.0}"
INPUT_ROOT="${INPUT_ROOT:-${ROOT_DIR}/input}"
OUTPUT_ROOT="${OUTPUT_ROOT:-${ROOT_DIR}/runs}"
REAL_TIME_FACTOR="${REAL_TIME_FACTOR:-1.0}"
TIMEOUT="${TIMEOUT:-180.0}"
WAIT_TIMEOUT="${WAIT_TIMEOUT:-180}"
CARLA_READY_SLEEP="${CARLA_READY_SLEEP:-5}"
POST_RUN_HOLD="${POST_RUN_HOLD:-0.0}"
VIDEO_FPS="${VIDEO_FPS:-10}"
VIDEO_FRAME_STRIDE="${VIDEO_FRAME_STRIDE:-2}"
DEMO_NO_RENDERING="${DEMO_NO_RENDERING:-0}"
DEMO_RECORD_TRAJECTORY="${DEMO_RECORD_TRAJECTORY:-0}"
DEMO_SYNC_ROUTE="${DEMO_SYNC_ROUTE:-0}"
CARLA_RENDER_BACKEND="${CARLA_RENDER_BACKEND:-offscreen}"
RECORD_RGB="${RECORD_RGB:-1}"
RGB_ACTOR_ROLE="${RGB_ACTOR_ROLE:-}"
RGB_WIDTH="${RGB_WIDTH:-1280}"
RGB_HEIGHT="${RGB_HEIGHT:-720}"
RGB_FPS="${RGB_FPS:-20}"
RGB_FOV="${RGB_FOV:-90}"
RGB_SAVE_EVERY="${RGB_SAVE_EVERY:-1}"
RGB_CAMERA_X="${RGB_CAMERA_X:--16.0}"
RGB_CAMERA_Y="${RGB_CAMERA_Y:-0.0}"
RGB_CAMERA_Z="${RGB_CAMERA_Z:-9.0}"
RGB_CAMERA_PITCH="${RGB_CAMERA_PITCH:--28.0}"
RGB_CAMERA_YAW="${RGB_CAMERA_YAW:-0.0}"
CONDA_ENV="${CONDA_ENV:-}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
KEEP_CARLA="${KEEP_CARLA:-0}"
USE_EXISTING_CARLA="${USE_EXISTING_CARLA:-1}"
CARLA_DISPLAY="${CARLA_DISPLAY:-${DISPLAY:-:0}}"
REQUIRE_EXISTING_CARLA="${REQUIRE_EXISTING_CARLA:-0}"

V2_CASES=(
  "001_Zoox_April_11_2025"
  "004_Pony.ai_March_29_2025"
  "008_Waymo_March_20_2025"
  "022_Waymo_March_27_2025"
  "072_Zoox_September_3_2024"
)
V2_SUT_ACTORS=(
  "v2"
  "hero"
  "bus_1"
  "v2"
  "v2"
)

if [[ -z "${SCENARIO}" && -z "${CASE}" && -n "${V2_CASE_INDEX}" ]]; then
  if [[ "${V2_CASE_INDEX}" -lt 1 || "${V2_CASE_INDEX}" -gt "${#V2_CASES[@]}" ]]; then
    echo "Invalid V2_CASE_INDEX=${V2_CASE_INDEX}; use 1-${#V2_CASES[@]}." >&2
    exit 2
  fi
  v2_array_index="$((V2_CASE_INDEX - 1))"
  CASE_NAME="${V2_CASES[${v2_array_index}]}"
  SCENARIO="${ROOT_DIR}/v2/${CASE_NAME}.xosc"
  XODR="${ROOT_DIR}/v2/${CASE_NAME}.xodr"
  if [[ -z "${SUT_ACTOR}" ]]; then
    SUT_ACTOR="${V2_SUT_ACTORS[${v2_array_index}]}"
  fi
  RUNS_SUBDIR="v2/${CASE_NAME}"
else
  CASE_NAME="${CASE:-$(basename "${SCENARIO:-scenario}" .xosc)}"
  RUNS_SUBDIR="headless_${CASE_NAME}"
fi

if [[ -z "${RGB_ACTOR_ROLE}" ]]; then
  RGB_ACTOR_ROLE="${SUT_ACTOR:-hero}"
fi

timestamp="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${RUN_DIR:-${OUTPUT_ROOT}/${RUNS_SUBDIR}_${timestamp}}"
CARLA_LOG="${RUN_DIR}/carla.log"
DEMO_LOG="${RUN_DIR}/demo.log"
RGB_LOG="${RUN_DIR}/rgb_recorder.log"
RGB_FRAME_DIR="${RUN_DIR}/rgb_frames"
RGB_VIDEO="${RUN_DIR}/carla_rgb.mp4"
ENCODED_RGB_VIDEO=0

mkdir -p "${RUN_DIR}"

stop_rgb_recorder() {
  if [[ -n "${RGB_PID:-}" ]]; then
    if kill -0 "${RGB_PID}" >/dev/null 2>&1; then
      echo "[cleanup] stopping RGB recorder pid=${RGB_PID}"
      kill "${RGB_PID}" >/dev/null 2>&1 || true
      wait "${RGB_PID}" >/dev/null 2>&1 || true
    fi
    RGB_PID=""
  fi
}

encode_rgb_video() {
  if [[ "${RECORD_RGB}" != "1" || "${ENCODED_RGB_VIDEO}" == "1" ]]; then
    return
  fi
  ENCODED_RGB_VIDEO=1

  if [[ ! -d "${RGB_FRAME_DIR}" ]]; then
    return
  fi

  local frame_count
  frame_count="$(find "${RGB_FRAME_DIR}" -maxdepth 1 -name 'frame_*.jpg' | wc -l)"
  if [[ "${frame_count}" -gt 0 ]]; then
    echo "Encoding real CARLA RGB video from ${frame_count} frames..."
    local ffmpeg_command=(
      ffmpeg
      -y
      -framerate "${RGB_FPS}"
      -start_number 0
      -i "${RGB_FRAME_DIR}/frame_%06d.jpg"
      -c:v libx264
      -pix_fmt yuv420p
      "${RGB_VIDEO}"
    )
    if command -v setsid >/dev/null 2>&1; then
      setsid "${ffmpeg_command[@]}" >>"${RGB_LOG}" 2>&1 || echo "ffmpeg failed; inspect ${RGB_LOG}" >&2
    else
      "${ffmpeg_command[@]}" >>"${RGB_LOG}" 2>&1 || echo "ffmpeg failed; inspect ${RGB_LOG}" >&2
    fi
  else
    echo "No RGB frames were captured; inspect ${RGB_LOG}" >&2
  fi
}

cleanup() {
  stop_rgb_recorder
  encode_rgb_video
  if [[ "${KEEP_CARLA}" != "1" && -n "${CARLA_PID:-}" ]]; then
    if kill -0 "${CARLA_PID}" >/dev/null 2>&1; then
      echo "[cleanup] stopping CARLA pid=${CARLA_PID}"
      kill "${CARLA_PID}" >/dev/null 2>&1 || true
      wait "${CARLA_PID}" >/dev/null 2>&1 || true
    fi
  fi
}
trap cleanup EXIT

if [[ -n "${CONDA_ENV}" ]]; then
  if [[ -f "/home/server/Software/miniconda3/etc/profile.d/conda.sh" ]]; then
    # shellcheck disable=SC1091
    source "/home/server/Software/miniconda3/etc/profile.d/conda.sh"
    conda activate "${CONDA_ENV}"
  else
    echo "CONDA_ENV=${CONDA_ENV} was set, but conda.sh was not found." >&2
    exit 2
  fi
fi

export CARLA_ROOT
export PYTHONPATH="${ROOT_DIR}/scenario_runner:${ROOT_DIR}/PCLA:${ROOT_DIR}/src:${CARLA_ROOT}/PythonAPI/carla:${PYTHONPATH:-}"

CARLA_ALREADY_RUNNING=0
if [[ "${USE_EXISTING_CARLA}" == "1" ]]; then
  if "${PYTHON_BIN}" - "${HOST}" "${PORT}" "${CARLA_ROOT}" >/dev/null 2>&1 <<'PY'
import glob
import os
import sys

host = sys.argv[1]
port = int(sys.argv[2])
carla_root = sys.argv[3]

try:
    import carla
except ImportError:
    api_root = os.path.join(carla_root, "PythonAPI", "carla")
    sys.path.append(api_root)
    sys.path.extend(glob.glob(os.path.join(api_root, "dist", "carla-*py3*.egg")))
    import carla

client = carla.Client(host, port)
client.set_timeout(2.0)
world = client.get_world()
world.get_snapshot()
PY
  then
    CARLA_ALREADY_RUNNING=1
  fi
fi

carla_args=(
  "-carla-rpc-port=${PORT}"
  "-nosound"
)

case "${CARLA_RENDER_BACKEND}" in
  offscreen)
    carla_args+=("-RenderOffScreen")
    ;;
  nullrhi)
    carla_args+=("-nullrhi")
    ;;
  window)
    ;;
  *)
    echo "Invalid CARLA_RENDER_BACKEND=${CARLA_RENDER_BACKEND}; use offscreen, nullrhi, or window." >&2
    exit 2
    ;;
esac

if [[ "${RECORD_RGB}" == "1" && "${CARLA_RENDER_BACKEND}" == "nullrhi" ]]; then
  echo "RECORD_RGB=1 cannot work with CARLA_RENDER_BACKEND=nullrhi; use offscreen or window." >&2
  exit 2
fi

if [[ "${RECORD_RGB}" == "1" && "${DEMO_NO_RENDERING}" == "1" ]]; then
  echo "RECORD_RGB=1 requires DEMO_NO_RENDERING=0, otherwise CARLA RGB sensors will be blank." >&2
  exit 2
fi

if [[ "${CARLA_ALREADY_RUNNING}" == "1" ]]; then
  echo "[1/4] reusing existing CARLA at ${HOST}:${PORT}"
  echo "  set USE_EXISTING_CARLA=0 to force this script to start its own server"
else
  if [[ "${REQUIRE_EXISTING_CARLA}" == "1" ]]; then
    echo "CARLA is not reachable at ${HOST}:${PORT}, and REQUIRE_EXISTING_CARLA=1." >&2
    echo "Start CARLA yourself first, then rerun this script." >&2
    exit 2
  fi

  if [[ ! -x "${CARLA_SH}" ]]; then
    echo "CARLA launcher not found or not executable: ${CARLA_SH}" >&2
    exit 2
  fi

  echo "[1/4] starting headless CARLA"
  echo "  launcher: ${CARLA_SH}"
  echo "  args: ${carla_args[*]}"
  if [[ "${CARLA_RENDER_BACKEND}" == "offscreen" ]]; then
    echo "  display: ${CARLA_DISPLAY}"
  fi
  echo "  log: ${CARLA_LOG}"
  if [[ "${CARLA_RENDER_BACKEND}" == "offscreen" ]]; then
    (
      cd "${CARLA_ROOT}"
      DISPLAY="${CARLA_DISPLAY}" bash "${CARLA_SH}" "${carla_args[@]}"
    ) >"${CARLA_LOG}" 2>&1 &
  else
    (
      cd "${CARLA_ROOT}"
      DISPLAY= SDL_VIDEODRIVER=offscreen bash "${CARLA_SH}" "${carla_args[@]}"
    ) >"${CARLA_LOG}" 2>&1 &
  fi
  CARLA_PID="$!"
  echo "  pid: ${CARLA_PID}"
fi

echo "[2/4] waiting for CARLA ${HOST}:${PORT}"
"${PYTHON_BIN}" - "${HOST}" "${PORT}" "${WAIT_TIMEOUT}" "${CARLA_ROOT}" "${CARLA_PID:-}" "${CARLA_LOG}" <<'PY'
import glob
import os
import signal
import sys
import time

host = sys.argv[1]
port = int(sys.argv[2])
timeout = float(sys.argv[3])
carla_root = sys.argv[4]
carla_pid = int(sys.argv[5]) if sys.argv[5] else None
carla_log = sys.argv[6]

try:
    import carla
except ImportError:
    api_root = os.path.join(carla_root, "PythonAPI", "carla")
    sys.path.append(api_root)
    sys.path.extend(glob.glob(os.path.join(api_root, "dist", "carla-*py3*.egg")))
    import carla

deadline = time.time() + timeout
last_error = None
while time.time() < deadline:
    if carla_pid is not None:
        try:
            os.kill(carla_pid, 0)
        except ProcessLookupError:
            print(f"CARLA process exited before opening port {host}:{port}.", file=sys.stderr)
            if os.path.exists(carla_log):
                print(f"--- {carla_log} ---", file=sys.stderr)
                with open(carla_log, "r", encoding="utf-8", errors="replace") as handle:
                    print(handle.read()[-4000:], file=sys.stderr)
            sys.exit(1)
        except PermissionError:
            pass
    try:
        client = carla.Client(host, port)
        client.set_timeout(5.0)
        world = client.get_world()
        world.get_snapshot()
        print(f"  connected: preloaded_map={world.get_map().name}, frame={world.get_snapshot().frame}")
        print("  scenario/map will be loaded in step [4/4] by demo.py")
        sys.exit(0)
    except Exception as exc:
        last_error = exc
        time.sleep(2.0)

print(f"Timed out waiting for CARLA at {host}:{port}: {last_error}", file=sys.stderr)
sys.exit(1)
PY

if [[ "${CARLA_READY_SLEEP}" != "0" ]]; then
  echo "  waiting ${CARLA_READY_SLEEP}s for CARLA render/server warmup"
  sleep "${CARLA_READY_SLEEP}"
fi

if [[ "${RECORD_RGB}" == "1" ]]; then
  mkdir -p "${RGB_FRAME_DIR}"
  echo "[3/4] starting real CARLA RGB recorder"
  echo "  actor role: ${RGB_ACTOR_ROLE}"
  echo "  frames: ${RGB_FRAME_DIR}"
  echo "  log: ${RGB_LOG}"
  (
    cd "${ROOT_DIR}"
    SDL_VIDEODRIVER=dummy "${PYTHON_BIN}" src/visualize_carla.py \
      --host "${HOST}" \
      --port "${PORT}" \
      --mode rgb \
      --actor-role "${RGB_ACTOR_ROLE}" \
      --width "${RGB_WIDTH}" \
      --height "${RGB_HEIGHT}" \
      --fps "${RGB_FPS}" \
      --fov "${RGB_FOV}" \
      --camera-x "${RGB_CAMERA_X}" \
      --camera-y "${RGB_CAMERA_Y}" \
      --camera-z "${RGB_CAMERA_Z}" \
      --camera-pitch "${RGB_CAMERA_PITCH}" \
      --camera-yaw "${RGB_CAMERA_YAW}" \
      --save-dir "${RGB_FRAME_DIR}" \
      --save-every "${RGB_SAVE_EVERY}"
  ) >"${RGB_LOG}" 2>&1 &
  RGB_PID="$!"
  echo "  pid: ${RGB_PID}"
fi

demo_args=(
  "--host" "${HOST}"
  "--port" "${PORT}"
  "--timeout" "${TIMEOUT}"
  "--trafficManagerPort" "${TM_PORT}"
  "--input-root" "${INPUT_ROOT}"
  "--data-output" "${RUN_DIR}"
  "--real-time-factor" "${REAL_TIME_FACTOR}"
  "--post-run-hold" "${POST_RUN_HOLD}"
  "--video-fps" "${VIDEO_FPS}"
  "--video-frame-stride" "${VIDEO_FRAME_STRIDE}"
)

if [[ "${DEMO_RECORD_TRAJECTORY}" == "1" ]]; then
  demo_args+=("--record-video")
fi

if [[ "${DEMO_NO_RENDERING}" == "1" ]]; then
  demo_args+=("--no-rendering")
fi

if [[ "${DEMO_SYNC_ROUTE}" == "1" ]]; then
  demo_args+=("--sync-route" "--agent" "${AGENT}")
fi

if [[ -n "${SCENARIO}" ]]; then
  if [[ ! -f "${SCENARIO}" ]]; then
    echo "Scenario file not found: ${SCENARIO}" >&2
    exit 2
  fi
  demo_args+=("--scenario" "${SCENARIO}")
else
  demo_args+=("--case" "${CASE}")
fi

if [[ -n "${XODR}" ]]; then
  if [[ ! -f "${XODR}" ]]; then
    echo "OpenDRIVE file not found: ${XODR}" >&2
    exit 2
  fi
  demo_args+=("--xodr" "${XODR}")
fi

if [[ -n "${SUT_ACTOR}" ]]; then
  route_output="${RUN_DIR}/${SUT_ACTOR}_pcla_route.xml"
  demo_args+=(
    "--output" "${route_output}"
    "--pcla-sut"
    "--agent" "${AGENT}"
    "--sut-actor" "${SUT_ACTOR}"
    "--sut-spawn-z" "${SUT_SPAWN_Z}"
    "--sut-start-offset" "${SUT_START_OFFSET}"
    "--sut-route-mode" "${SUT_ROUTE_MODE}"
    "--sut-route-spacing" "${SUT_ROUTE_SPACING}"
  )
fi

echo "[4/4] running scenario"
echo "  output: ${RUN_DIR}"
if [[ -n "${SUT_ACTOR}" ]]; then
  echo "  ADS: agent=${AGENT}, sut_actor=${SUT_ACTOR}"
fi
if [[ -n "${SCENARIO}" ]]; then
  echo "  scenario: ${SCENARIO}"
fi
if [[ -n "${XODR}" ]]; then
  echo "  opendrive: ${XODR}"
fi
echo "  log: ${DEMO_LOG}"
(
  cd "${ROOT_DIR}"
  "${PYTHON_BIN}" src/demo.py "${demo_args[@]}"
) 2>&1 | tee "${DEMO_LOG}"

stop_rgb_recorder
encode_rgb_video

echo
echo "Done."
echo "  run dir: ${RUN_DIR}"
if [[ -f "${RGB_VIDEO}" ]]; then
  echo "  real carla rgb video: ${RGB_VIDEO}"
elif [[ -f "${RUN_DIR}/trajectory_video.mp4" ]]; then
  echo "  trajectory video: ${RUN_DIR}/trajectory_video.mp4"
elif [[ -f "${RUN_DIR}/trajectory_video.gif" ]]; then
  echo "  trajectory video fallback: ${RUN_DIR}/trajectory_video.gif"
else
  echo "  video not found; inspect ${DEMO_LOG}"
fi
echo "  carla log: ${CARLA_LOG}"
echo "  demo log: ${DEMO_LOG}"
if [[ "${RECORD_RGB}" == "1" ]]; then
  echo "  rgb recorder log: ${RGB_LOG}"
  echo "  rgb frames: ${RGB_FRAME_DIR}"
fi
