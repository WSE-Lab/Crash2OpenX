#!/usr/bin/env bash
# One-command launcher for the Crash2OpenX browser demo.
#
#   ./run.sh                # sync env, start CARLA (Docker), serve the web demo
#   ./run.sh --no-carla     # compile-only machine: skip the CARLA container
#   ./run.sh --port 8080    # web port (default 5000)
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${REPO_ROOT}"

PORT=5000
HOST=127.0.0.1
START_CARLA=1
while (( "$#" )); do
  case "$1" in
    --no-carla) START_CARLA=0; shift ;;
    --port) PORT="$2"; shift 2 ;;
    --host) HOST="$2"; shift 2 ;;
    *) echo "unknown flag: $1" >&2; exit 2 ;;
  esac
done

command -v uv >/dev/null || {
  echo "uv not found. Install: curl -LsSf https://astral.sh/uv/install.sh | sh" >&2
  exit 2
}

if [[ ! -f external/scenariogeneration/pyproject.toml ]]; then
  echo "[run] submodules missing; fetching (git submodule update --init --recursive)"
  git submodule update --init --recursive
fi

echo "[run] syncing python env (uv sync)"
uv sync

if [[ ! -f .env.local ]]; then
  echo "[run] WARN: .env.local not found — copy .env.example and set OPENROUTER_API_KEY," >&2
  echo "      otherwise seed inference (the LLM front end) will fail." >&2
fi

if [[ "${START_CARLA}" == "1" ]]; then
  if command -v docker >/dev/null && docker info >/dev/null 2>&1; then
    echo "[run] starting CARLA container (docker compose -f docker/docker-compose.yml up -d)"
    docker compose -f docker/docker-compose.yml up -d
  else
    echo "[run] WARN: docker unavailable; skipping CARLA. Compilation + XSD validation" >&2
    echo "      still work; execution phases will fail until CARLA is reachable." >&2
  fi
fi

echo "[run] starting web demo on http://${HOST}:${PORT}"
exec uv run python tools/web_app.py --host "${HOST}" --port "${PORT}"
