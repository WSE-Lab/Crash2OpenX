#!/usr/bin/env python3
"""Factory for the CARLA execution client.

CARLA_MODE=local  (default) -> CarlaLocalClient: Dockerized CARLA on this
                   machine (docker/docker-compose.yml), runner/runner_local.sh.
CARLA_MODE=remote -> CarlaRemoteClient: legacy ssh/scp driver against a
                   remote CARLA host (tools/carla_remote.py, CARLA_REMOTE_*).
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.carla_remote import CarlaRemoteClient, CarlaRemoteError  # noqa: E402,F401
from tools.carla_local import CarlaLocalClient  # noqa: E402,F401

# Callers catch this one name regardless of transport.
CarlaClientError = CarlaRemoteError


def get_carla_client():
    mode = os.environ.get("CARLA_MODE", "local").strip().lower()
    if mode == "remote":
        return CarlaRemoteClient()
    if mode == "local":
        return CarlaLocalClient()
    raise ValueError(f"CARLA_MODE must be 'local' or 'remote', got {mode!r}")
