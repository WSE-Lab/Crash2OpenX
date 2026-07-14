#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BATCH_DIR="${BATCH_DIR:-${ROOT_DIR}/carla_test_batch}"
SELECTOR="${1:-1}"

usage() {
  cat <<'EOF'
Usage:
  scripts/run_carla_test_batch_rgb_video.sh list
  scripts/run_carla_test_batch_rgb_video.sh 1
  scripts/run_carla_test_batch_rgb_video.sh 004
  scripts/run_carla_test_batch_rgb_video.sh 004_Pony.ai_March_29_2025
  scripts/run_carla_test_batch_rgb_video.sh all

Useful environment overrides:
  BATCH_DIR=/path/to/carla_test_batch
  OUTPUT_ROOT=/path/to/runs
  HOST=localhost PORT=2000 TM_PORT=8000
  CONDA_ENV=PCLA AGENT=if_if
  RGB_WIDTH=1280 RGB_HEIGHT=720 RGB_FPS=20
  REQUIRE_EXISTING_CARLA=1  # fail if no CARLA server is already running

The script loads the matching .xodr map, runs the matching .xosc scenario, and
writes a real CARLA RGB video named carla_rgb.mp4 under OUTPUT_ROOT.
EOF
}

load_cases() {
  if [[ ! -d "${BATCH_DIR}" ]]; then
    echo "Batch directory not found: ${BATCH_DIR}" >&2
    exit 2
  fi

  mapfile -t CASES < <(find "${BATCH_DIR}" -maxdepth 1 -type f -name '*.xosc' -printf '%f\n' | sort)
  if [[ "${#CASES[@]}" -eq 0 ]]; then
    echo "No .xosc files found in ${BATCH_DIR}" >&2
    exit 2
  fi
}

print_cases() {
  local index=1
  local file stem
  for file in "${CASES[@]}"; do
    stem="${file%.xosc}"
    printf "%2d  %s\n" "${index}" "${stem}"
    index=$((index + 1))
  done
}

resolve_case() {
  local selector="$1"
  local index file stem matches=()

  if [[ "${selector}" =~ ^[0-9]+$ ]]; then
    if (( selector >= 1 && selector <= ${#CASES[@]} )); then
      printf '%s\n' "${CASES[$((selector - 1))]%.xosc}"
      return 0
    fi
  fi

  for file in "${CASES[@]}"; do
    stem="${file%.xosc}"
    if [[ "${stem}" == "${selector}" || "${file}" == "${selector}" || "${stem}" == "${selector}"* ]]; then
      matches+=("${stem}")
    fi
  done

  if [[ "${#matches[@]}" -eq 1 ]]; then
    printf '%s\n' "${matches[0]}"
    return 0
  fi

  if [[ "${#matches[@]}" -gt 1 ]]; then
    echo "Selector '${selector}' matched multiple cases:" >&2
    printf '  %s\n' "${matches[@]}" >&2
    exit 2
  fi

  echo "Unknown case selector: ${selector}" >&2
  echo "Available cases:" >&2
  print_cases >&2
  exit 2
}

run_one_case() {
  local stem="$1"
  local scenario="${BATCH_DIR}/${stem}.xosc"
  local xodr="${BATCH_DIR}/${stem}.xodr"

  if [[ ! -f "${scenario}" ]]; then
    echo "Scenario file not found: ${scenario}" >&2
    exit 2
  fi
  if [[ ! -f "${xodr}" ]]; then
    echo "OpenDRIVE file not found: ${xodr}" >&2
    exit 2
  fi

  echo
  echo "=== Running ${stem} ==="
  echo "Scenario: ${scenario}"
  echo "OpenDRIVE: ${xodr}"

  export CONDA_ENV="${CONDA_ENV:-PCLA}"
  export SCENARIO="${scenario}"
  export XODR="${xodr}"
  export CASE="${stem}"
  export OUTPUT_ROOT="${OUTPUT_ROOT:-${ROOT_DIR}/runs/carla_test_batch}"

  export AGENT="${AGENT:-if_if}"
  export SUT_ACTOR="${BATCH_SUT_ACTOR:-}"
  export RGB_ACTOR_ROLE="${RGB_ACTOR_ROLE:-hero}"

  export DEMO_SYNC_ROUTE="${DEMO_SYNC_ROUTE:-1}"
  export DEMO_NO_RENDERING="${DEMO_NO_RENDERING:-0}"
  export DEMO_RECORD_TRAJECTORY="${DEMO_RECORD_TRAJECTORY:-0}"
  export RECORD_RGB="${RECORD_RGB:-1}"
  export CARLA_RENDER_BACKEND="${CARLA_RENDER_BACKEND:-offscreen}"

  export RGB_WIDTH="${RGB_WIDTH:-1280}"
  export RGB_HEIGHT="${RGB_HEIGHT:-720}"
  export RGB_FPS="${RGB_FPS:-20}"
  export RGB_SAVE_EVERY="${RGB_SAVE_EVERY:-1}"
  export RGB_CAMERA_X="${RGB_CAMERA_X:--16.0}"
  export RGB_CAMERA_Y="${RGB_CAMERA_Y:-0.0}"
  export RGB_CAMERA_Z="${RGB_CAMERA_Z:-9.0}"
  export RGB_CAMERA_PITCH="${RGB_CAMERA_PITCH:--28.0}"
  export RGB_CAMERA_YAW="${RGB_CAMERA_YAW:-0.0}"

  export REAL_TIME_FACTOR="${REAL_TIME_FACTOR:-1.0}"
  export TIMEOUT="${TIMEOUT:-240.0}"
  export WAIT_TIMEOUT="${WAIT_TIMEOUT:-180}"

  "${ROOT_DIR}/scripts/run_headless_scene_record.sh"
}

if [[ "${SELECTOR}" == "-h" || "${SELECTOR}" == "--help" ]]; then
  usage
  exit 0
fi

load_cases

case "${SELECTOR}" in
  list)
    print_cases
    ;;
  all)
    for case_file in "${CASES[@]}"; do
      run_one_case "${case_file%.xosc}"
    done
    ;;
  *)
    run_one_case "$(resolve_case "${SELECTOR}")"
    ;;
esac
