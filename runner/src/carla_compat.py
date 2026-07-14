"""CARLA Python API import helpers.

Prefer the package installed in the active Python environment. If it is not
installed, fall back to CARLA_ROOT/PythonAPI so scripts still work with a
downloaded CARLA distribution.
"""

from __future__ import annotations

import glob
import os
import sys
from pathlib import Path
from typing import List, Optional


def _carla_api_root(carla_root: Optional[str] = None) -> Optional[Path]:
    root_value = carla_root or os.environ.get("CARLA_ROOT")
    if not root_value:
        return None
    root = Path(root_value).expanduser()
    return root / "PythonAPI" / "carla"


def add_carla_agents_path(carla_root: Optional[str] = None) -> List[str]:
    api_root = _carla_api_root(carla_root)
    if api_root is None:
        return []
    candidate = str(api_root)
    if os.path.exists(candidate) and candidate not in sys.path:
        sys.path.append(candidate)
        return [candidate]
    return []


def add_carla_python_api_paths(carla_root: Optional[str] = None) -> List[str]:
    api_root = _carla_api_root(carla_root)
    if api_root is None:
        return []

    candidates = [str(api_root)]
    candidates.extend(glob.glob(str(api_root / "dist" / "carla-*py3*.egg")))

    added = []
    for candidate in candidates:
        if os.path.exists(candidate) and candidate not in sys.path:
            sys.path.append(candidate)
            added.append(candidate)
    return added


def ensure_carla_importable(carla_root: Optional[str] = None):
    try:
        import carla  # pylint: disable=import-error,import-outside-toplevel
    except ImportError:
        add_carla_python_api_paths(carla_root)
        import carla  # pylint: disable=import-error,import-outside-toplevel
    add_carla_agents_path(carla_root)
    return carla
