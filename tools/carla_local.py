#!/usr/bin/env python3
"""Local CARLA client: run scenarios against a Dockerized CARLA on this machine.

Paper Sec. 4 (execution layer): CARLA runs in Docker so the simulation host is
reproducible with a single command; the rest of the tool runs on the client
side and connects to it. This client keeps the exact operation logic of
``tools.carla_remote.CarlaRemoteClient`` (content-addressed extract cache,
silent-spawn retry, merge mode, L3 behavior-vs-intent gate, paper render) and
swaps only the transport: no ssh/scp, plain local subprocess + file copies.

Operations:
  * extract_roadgraph(xodr) -> outputs/map_cache/<name>/{waypoints,junctions,
        legs,route_candidates,roadgraph_selfcheck,manifest}.json
  * run_scenario(xodr, xosc) -> outputs/web_runs/<name>_<ts>/{frame_states.jsonl,
        events.jsonl,summary.json,behavior_check.json,...}

The heavy lifting is runner/runner_local.sh (extract|run), which manages the
CARLA Docker container (docker compose up / start / degraded-state restart)
and delegates to runner/extract_roadgraph_carla.py and runner/src/runs/run_scene.py
with PCLA + scenario_runner on PYTHONPATH (symlinked from external/).
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.carla_remote import CarlaRemoteClient, CarlaRemoteError  # noqa: E402


def _default_python() -> str:
    exec_py = ROOT / ".venv-exec" / "bin" / "python"
    if exec_py.is_file():
        return str(exec_py)
    venv_py = ROOT / ".venv" / "bin" / "python"
    return str(venv_py) if venv_py.is_file() else sys.executable


@dataclass(frozen=True)
class LocalCfg:
    """Duck-typed stand-in for RemoteCfg; only runs_root/runner_path are used
    by the (inherited) operation logic, the rest feeds runner_local.sh env."""
    runs_root: str = str(ROOT / "outputs" / "runner_runs")
    runner_path: str = str(ROOT / "runner" / "runner_local.sh")
    project_dir: str = str(ROOT / "runner")
    python_bin: str = _default_python()
    container_name: str = "carla-0916"
    rpc_port: int = 2000
    compose_file: str = str(ROOT / "docker" / "docker-compose.yml")

    @classmethod
    def from_env(cls) -> "LocalCfg":
        d = cls()
        return cls(
            runs_root=os.environ.get("CARLA_LOCAL_RUNS_ROOT", d.runs_root),
            runner_path=os.environ.get("CARLA_LOCAL_RUNNER", d.runner_path),
            project_dir=os.environ.get("CARLA_LOCAL_PROJECT_DIR", d.project_dir),
            python_bin=os.environ.get("CARLA_LOCAL_PYTHON", d.python_bin),
            container_name=os.environ.get("CARLA_LOCAL_CONTAINER", d.container_name),
            rpc_port=int(os.environ.get("CARLA_LOCAL_RPC_PORT", d.rpc_port)),
            compose_file=os.environ.get("CARLA_LOCAL_COMPOSE_FILE", d.compose_file),
        )


class CarlaLocalClient(CarlaRemoteClient):
    """CarlaRemoteClient with local transport (subprocess + file copies)."""

    def __init__(self, cfg: LocalCfg | None = None):
        self.cfg = cfg or LocalCfg.from_env()  # type: ignore[assignment]

    # ---- transport primitives (local overrides) ------------------------------

    def _runner_env(self) -> dict:
        env = dict(os.environ)
        env.update({
            "RUNS_ROOT": self.cfg.runs_root,
            "PROJECT_DIR": self.cfg.project_dir,
            "PYTHON": self.cfg.python_bin,
            "CONTAINER": self.cfg.container_name,
            "CARLA_PORT": str(self.cfg.rpc_port),
            "COMPOSE_FILE": self.cfg.compose_file,
        })
        return env

    def _ssh(self, remote_cmd: str, *, timeout: float | None = None) -> subprocess.CompletedProcess:
        return subprocess.run(
            ["bash", "-c", remote_cmd],
            capture_output=True, text=True, timeout=timeout, check=False,
            env=self._runner_env(),
        )

    def _push(self, local: Path, remote: str) -> None:
        dst = Path(remote)
        dst.parent.mkdir(parents=True, exist_ok=True)
        try:
            shutil.copy2(str(local), str(dst))
        except OSError as exc:
            raise CarlaRemoteError(f"local push failed: {exc}")

    def _pull(self, remote_glob: str, local_dir: Path) -> None:
        # scp -r semantics: a directory source lands as <local_dir>/<basename>.
        src = Path(remote_glob)
        local_dir.mkdir(parents=True, exist_ok=True)
        if src.is_dir():
            shutil.copytree(src, local_dir / src.name, dirs_exist_ok=True)
        elif src.is_file():
            shutil.copy2(str(src), str(local_dir / src.name))
        else:
            raise CarlaRemoteError(f"local pull failed: no such path {src}")

    # ---- runner deployment: nothing to ship locally ---------------------------

    def deploy_runner(self) -> None:
        runner = Path(self.cfg.runner_path)
        if not runner.is_file():
            raise FileNotFoundError(runner)
        runner.chmod(runner.stat().st_mode | 0o111)


if __name__ == "__main__":
    # Reuse the parent CLI but force the local client.
    import tools.carla_remote as _remote
    _remote.CarlaRemoteClient = CarlaLocalClient  # type: ignore[misc]
    raise SystemExit(_remote._cli())
