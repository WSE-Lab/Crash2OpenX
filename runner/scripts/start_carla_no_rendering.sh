#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CARLA_ROOT="${CARLA_ROOT:-${HOME}/carla}"
CARLA_SH="${CARLA_SH:-${CARLA_ROOT}/CarlaUE4.sh}"
export CARLA_ROOT
HOST="${HOST:-localhost}"
PORT="${PORT:-2000}"
WAIT_TIMEOUT="${WAIT_TIMEOUT:-120}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/runs/carla_no_rendering}"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/carla_${PORT}.log}"
PID_FILE="${PID_FILE:-${LOG_DIR}/carla_${PORT}.pid}"

# BACKEND=nullrhi disables Unreal rendering. Use BACKEND=offscreen if you need
# RGB/camera sensors while still running without a visible desktop window.
BACKEND="${BACKEND:-nullrhi}"
QUALITY_LEVEL="${QUALITY_LEVEL:-Low}"
EXTRA_CARLA_ARGS="${EXTRA_CARLA_ARGS:-}"

mkdir -p "${LOG_DIR}"

if [[ ! -x "${CARLA_SH}" ]]; then
  echo "CARLA launcher not found or not executable: ${CARLA_SH}" >&2
  exit 2
fi

carla_args=(
  "-carla-rpc-port=${PORT}"
  "-quality-level=${QUALITY_LEVEL}"
  "-nosound"
)

case "${BACKEND}" in
  nullrhi)
    carla_args+=("-nullrhi")
    ;;
  offscreen)
    carla_args+=("-RenderOffScreen")
    ;;
  window)
    ;;
  *)
    echo "Invalid BACKEND=${BACKEND}; use nullrhi, offscreen, or window." >&2
    exit 2
    ;;
esac

if [[ -n "${EXTRA_CARLA_ARGS}" ]]; then
  # shellcheck disable=SC2206
  extra_args=(${EXTRA_CARLA_ARGS})
  carla_args+=("${extra_args[@]}")
fi

is_carla_ready() {
  "${PYTHON_BIN}" - "${HOST}" "${PORT}" "${CARLA_ROOT}" <<'PY' >/dev/null 2>&1
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
client.get_world().get_snapshot()
PY
}

if is_carla_ready; then
  echo "CARLA is already reachable at ${HOST}:${PORT}"
  exit 0
fi

if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(cat "${PID_FILE}")"
  if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" >/dev/null 2>&1; then
    echo "PID file exists and process is still running: ${PID_FILE} pid=${old_pid}" >&2
    echo "Remove the stale server or use another PORT." >&2
    exit 1
  fi
fi

echo "Starting CARLA:"
echo "  launcher: ${CARLA_SH}"
echo "  host/port: ${HOST}:${PORT}"
echo "  backend: ${BACKEND}"
echo "  log: ${LOG_FILE}"

if [[ "${BACKEND}" == "offscreen" ]]; then
  DISPLAY= SDL_VIDEODRIVER=offscreen bash "${CARLA_SH}" "${carla_args[@]}" >"${LOG_FILE}" 2>&1 &
else
  DISPLAY= SDL_VIDEODRIVER=dummy bash "${CARLA_SH}" "${carla_args[@]}" >"${LOG_FILE}" 2>&1 &
fi

carla_pid="$!"
echo "${carla_pid}" >"${PID_FILE}"

deadline="$((SECONDS + WAIT_TIMEOUT))"
while (( SECONDS < deadline )); do
  if ! kill -0 "${carla_pid}" >/dev/null 2>&1; then
    echo "CARLA exited before becoming ready. Inspect ${LOG_FILE}" >&2
    exit 1
  fi
  if is_carla_ready; then
    echo "CARLA ready at ${HOST}:${PORT} pid=${carla_pid}"
    exit 0
  fi
  sleep 2
done

echo "Timed out waiting for CARLA at ${HOST}:${PORT}. pid=${carla_pid}, log=${LOG_FILE}" >&2
exit 1
