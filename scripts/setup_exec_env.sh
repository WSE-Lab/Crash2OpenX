#!/usr/bin/env bash
# Build the uv-managed EXECUTION environment (.venv-exec) for the CARLA
# execution layer: PCLA's pinned dependency set (python 3.10, carla==0.9.16,
# torch, ...) + scenario_runner requirements.
#
# Run this on the machine that talks to the CARLA Docker container (Linux
# x86_64 with an NVIDIA GPU). The main .venv (uv sync) stays light and is all
# you need for inference + compilation + XSD validation + the web demo.
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

ENV_DIR="${ENV_DIR:-.venv-exec}"
PCLA_ENV_YML="external/PCLA/environment.yml"
SR_REQS="external/scenario_runner/requirements.txt"

[[ -f "${PCLA_ENV_YML}" ]] || {
  echo "missing ${PCLA_ENV_YML}; run: git submodule update --init --recursive" >&2
  exit 2
}

echo "[1/3] creating ${ENV_DIR} (python 3.10, uv-managed)"
uv venv --python 3.10 "${ENV_DIR}"

echo "[2/3] extracting pip pins from PCLA environment.yml"
REQS_TMP="$(mktemp -t pcla_reqs_XXXX).txt"
python3 - "${PCLA_ENV_YML}" "${REQS_TMP}" <<'PY'
import sys
src, dst = sys.argv[1], sys.argv[2]
pins, in_pip = [], False
for raw in open(src, encoding="utf-8"):
    line = raw.rstrip("\n")
    stripped = line.strip()
    if stripped == "- pip:":
        in_pip = True
        continue
    if in_pip:
        if stripped.startswith("- "):
            pins.append(stripped[2:].strip())
        elif stripped and not raw.startswith((" ", "\t")):
            in_pip = False
open(dst, "w", encoding="utf-8").write("\n".join(pins) + "\n")
print(f"{len(pins)} pins -> {dst}")
PY

echo "[3/3] installing PCLA pins + scenario_runner requirements into ${ENV_DIR}"
uv pip install --python "${ENV_DIR}/bin/python" -r "${REQS_TMP}"
uv pip install --python "${ENV_DIR}/bin/python" -r "${SR_REQS}"
rm -f "${REQS_TMP}"

echo
echo "Done. runner/runner_local.sh picks ${ENV_DIR}/bin/python automatically."
echo "Agent weights: see external/PCLA/README.md (download pretrained weights"
echo "into external/PCLA/pcla_agents/ before the first run)."
