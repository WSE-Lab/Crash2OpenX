#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export CONDA_ENV="${CONDA_ENV:-PCLA}"
export SCENARIO="${SCENARIO:-${ROOT_DIR}/new/001_Zoox_April_11_2025_rearend_ads.xosc}"
export XODR="${XODR:-${ROOT_DIR}/opendrive_seed/001_Zoox_April_11_2025.xodr}"
export AGENT="${AGENT:-if_if}"
export SUT_ACTOR="${SUT_ACTOR:-external_control}"
export RGB_ACTOR_ROLE="${RGB_ACTOR_ROLE:-external_control}"

# Headless means no visible desktop window. Keep CARLA rendering enabled so the
# exported visualization is real sensor video from the CARLA scene.
export DEMO_NO_RENDERING="${DEMO_NO_RENDERING:-0}"
export DEMO_RECORD_TRAJECTORY="${DEMO_RECORD_TRAJECTORY:-0}"
export DEMO_SYNC_ROUTE="${DEMO_SYNC_ROUTE:-0}"
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

export VIDEO_FPS="${VIDEO_FPS:-10}"
export VIDEO_FRAME_STRIDE="${VIDEO_FRAME_STRIDE:-2}"
export REAL_TIME_FACTOR="${REAL_TIME_FACTOR:-1.0}"
export TIMEOUT="${TIMEOUT:-180.0}"
export WAIT_TIMEOUT="${WAIT_TIMEOUT:-180}"

exec "${ROOT_DIR}/scripts/run_headless_scene_record.sh" "$@"
