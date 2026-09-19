"""Callable wrapper for running one OpenDRIVE map and one OpenSCENARIO scene.

The heavy lifting stays in scripts/run_headless_scene_record.sh so the Python
entry point shares the same CARLA startup, RGB recording, and mp4 encoding path
used by the existing command-line workflow.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import signal
import subprocess
import time
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterator, List, Mapping, Optional, Union


PathLike = Union[str, os.PathLike]

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "src" / "runs"
RUN_SCRIPT = REPO_ROOT / "scripts" / "run_headless_scene_record.sh"


class RunSceneError(RuntimeError):
    """Raised when a scenario run fails or does not produce an mp4."""


@dataclass(frozen=True)
class RunSceneResult:
    """Result metadata for a completed scene run."""

    mp4_path: str
    run_dir: str
    scenario_path: str
    map_path: str
    log_path: str
    return_code: int
    elapsed_seconds: float

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


def run_map_scenario(
    map_path: PathLike,
    scenario_path: PathLike,
    **kwargs: object,
) -> str:
    """Run one map and one scenario, returning the produced mp4 path.

    This is the simplest remote-call surface:

        mp4_path = run_map_scenario("/path/map.xodr", "/path/scene.xosc")
    """

    return run_scene(map_path=map_path, scenario_path=scenario_path, **kwargs).mp4_path


def run_map_scenario_bytes(
    map_path: PathLike,
    scenario_path: PathLike,
    **kwargs: object,
) -> bytes:
    """Run one map and one scenario, returning mp4 bytes.

    Prefer returning the path for large videos. This helper is useful when the
    remote caller expects a binary payload directly.
    """

    mp4_path = run_map_scenario(map_path=map_path, scenario_path=scenario_path, **kwargs)
    with open(mp4_path, "rb") as handle:
        return handle.read()


def run_scene(
    map_path: PathLike,
    scenario_path: PathLike,
    *,
    output_root: Optional[PathLike] = None,
    run_name: Optional[str] = None,
    host: str = "127.0.0.1",
    port: int = 2000,
    traffic_manager_port: int = 8000,
    timeout: float = 240.0,
    wait_timeout: float = 180.0,
    process_timeout: Optional[float] = None,
    conda_env: Optional[str] = "PCLA",
    agent: str = "tfv6_regnet",
    sut_actor: Optional[str] = None,
    rgb_actor_role: Optional[str] = None,
    rgb_width: int = 1280,
    rgb_height: int = 720,
    rgb_fps: int = 20,
    rgb_save_every: int = 1,
    rgb_camera_x: float = -16.0,
    rgb_camera_y: float = 0.0,
    rgb_camera_z: float = 9.0,
    rgb_camera_pitch: float = -28.0,
    rgb_camera_yaw: float = 0.0,
    real_time_factor: float = 1.0,
    post_run_hold: float = 0.0,
    record_rgb: bool = True,
    record_trajectory: bool = False,
    no_rendering: bool = False,
    use_existing_carla: bool = True,
    require_existing_carla: bool = False,
    keep_carla: bool = False,
    lock_port: bool = True,
    extra_env: Optional[Mapping[str, object]] = None,
) -> RunSceneResult:
    """Run one OpenDRIVE map plus one OpenSCENARIO scene and return metadata.

    Args:
        map_path: OpenDRIVE ``.xodr`` file to load into CARLA.
        scenario_path: OpenSCENARIO ``.xosc`` file to run.
        output_root: Parent directory for run artifacts. Defaults to
            ``src/runs``.
        run_name: Optional deterministic run directory name. Defaults to
            ``<scenario_stem>_<timestamp>``.
        process_timeout: Optional wall-clock timeout for the wrapper process.
        lock_port: Serialize calls per CARLA port. Keep this enabled for remote
            services unless each request gets its own CARLA port.
        extra_env: Additional environment variables passed to the run script.
    """

    scenario_file = _resolve_existing_file(scenario_path, ".xosc", "OpenSCENARIO")
    map_file = _resolve_existing_file(map_path, ".xodr", "OpenDRIVE")
    output_dir = Path(output_root).expanduser().resolve() if output_root else DEFAULT_OUTPUT_ROOT
    output_dir.mkdir(parents=True, exist_ok=True)

    scenario_stem = scenario_file.stem
    resolved_run_name = run_name or f"{scenario_stem}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
    run_dir = (output_dir / _safe_run_name(resolved_run_name)).resolve()
    run_dir.mkdir(parents=True, exist_ok=False)
    wrapper_log = run_dir / "run_scene_wrapper.log"

    env = _build_env(
        map_file=map_file,
        scenario_file=scenario_file,
        run_dir=run_dir,
        host=host,
        port=port,
        traffic_manager_port=traffic_manager_port,
        timeout=timeout,
        wait_timeout=wait_timeout,
        conda_env=conda_env,
        agent=agent,
        sut_actor=sut_actor,
        rgb_actor_role=rgb_actor_role,
        rgb_width=rgb_width,
        rgb_height=rgb_height,
        rgb_fps=rgb_fps,
        rgb_save_every=rgb_save_every,
        rgb_camera_x=rgb_camera_x,
        rgb_camera_y=rgb_camera_y,
        rgb_camera_z=rgb_camera_z,
        rgb_camera_pitch=rgb_camera_pitch,
        rgb_camera_yaw=rgb_camera_yaw,
        real_time_factor=real_time_factor,
        post_run_hold=post_run_hold,
        record_rgb=record_rgb,
        record_trajectory=record_trajectory,
        no_rendering=no_rendering,
        use_existing_carla=use_existing_carla,
        require_existing_carla=require_existing_carla,
        keep_carla=keep_carla,
        extra_env=extra_env,
    )

    command = ["bash", str(RUN_SCRIPT)]
    started_at = time.monotonic()
    with _optional_port_lock(port, lock_port):
        return_code = _run_command(command, env=env, log_path=wrapper_log, timeout=process_timeout)
    elapsed = time.monotonic() - started_at

    mp4_path = _find_mp4(run_dir)
    if return_code != 0 or mp4_path is None:
        tail = _read_tail(wrapper_log)
        raise RunSceneError(
            "Scenario run failed or did not produce an mp4. "
            f"return_code={return_code}, run_dir={run_dir}, log={wrapper_log}\n{tail}"
        )

    return RunSceneResult(
        mp4_path=str(mp4_path),
        run_dir=str(run_dir),
        scenario_path=str(scenario_file),
        map_path=str(map_file),
        log_path=str(wrapper_log),
        return_code=return_code,
        elapsed_seconds=elapsed,
    )


def _resolve_existing_file(path: PathLike, expected_suffix: str, label: str) -> Path:
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"{label} file not found: {resolved}")
    if resolved.suffix.lower() != expected_suffix:
        raise ValueError(f"{label} file must be {expected_suffix}: {resolved}")
    return resolved


def _safe_run_name(value: str) -> str:
    safe = []
    for char in value:
        if char.isalnum() or char in ("-", "_", ".", "@"):
            safe.append(char)
        else:
            safe.append("_")
    name = "".join(safe).strip("._")
    if not name:
        raise ValueError("run_name must contain at least one safe character")
    return name


def _build_env(
    *,
    map_file: Path,
    scenario_file: Path,
    run_dir: Path,
    host: str,
    port: int,
    traffic_manager_port: int,
    timeout: float,
    wait_timeout: float,
    conda_env: Optional[str],
    agent: str,
    sut_actor: Optional[str],
    rgb_actor_role: Optional[str],
    rgb_width: int,
    rgb_height: int,
    rgb_fps: int,
    rgb_save_every: int,
    rgb_camera_x: float,
    rgb_camera_y: float,
    rgb_camera_z: float,
    rgb_camera_pitch: float,
    rgb_camera_yaw: float,
    real_time_factor: float,
    post_run_hold: float,
    record_rgb: bool,
    record_trajectory: bool,
    no_rendering: bool,
    use_existing_carla: bool,
    require_existing_carla: bool,
    keep_carla: bool,
    extra_env: Optional[Mapping[str, object]],
) -> Dict[str, str]:
    env = os.environ.copy()
    env.update(
        {
            "SCENARIO": str(scenario_file),
            "XODR": str(map_file),
            "CASE": scenario_file.stem,
            "RUN_DIR": str(run_dir),
            "OUTPUT_ROOT": str(run_dir.parent),
            "HOST": host,
            "PORT": str(int(port)),
            "TM_PORT": str(int(traffic_manager_port)),
            "TIMEOUT": str(float(timeout)),
            "WAIT_TIMEOUT": str(float(wait_timeout)),
            "AGENT": agent,
            "REAL_TIME_FACTOR": str(float(real_time_factor)),
            "POST_RUN_HOLD": str(float(post_run_hold)),
            "RECORD_RGB": "1" if record_rgb else "0",
            "DEMO_RECORD_TRAJECTORY": "1" if record_trajectory else "0",
            "DEMO_NO_RENDERING": "1" if no_rendering else "0",
            "USE_EXISTING_CARLA": "1" if use_existing_carla else "0",
            "REQUIRE_EXISTING_CARLA": "1" if require_existing_carla else "0",
            "KEEP_CARLA": "1" if keep_carla else "0",
            "CARLA_RENDER_BACKEND": "offscreen",
            "RGB_ACTOR_ROLE": rgb_actor_role or sut_actor or "hero",
            "RGB_WIDTH": str(int(rgb_width)),
            "RGB_HEIGHT": str(int(rgb_height)),
            "RGB_FPS": str(int(rgb_fps)),
            "RGB_SAVE_EVERY": str(int(rgb_save_every)),
            "RGB_CAMERA_X": str(float(rgb_camera_x)),
            "RGB_CAMERA_Y": str(float(rgb_camera_y)),
            "RGB_CAMERA_Z": str(float(rgb_camera_z)),
            "RGB_CAMERA_PITCH": str(float(rgb_camera_pitch)),
            "RGB_CAMERA_YAW": str(float(rgb_camera_yaw)),
        }
    )
    if conda_env:
        env["CONDA_ENV"] = conda_env
    else:
        env.pop("CONDA_ENV", None)
    if sut_actor:
        env["SUT_ACTOR"] = sut_actor
    else:
        env.pop("SUT_ACTOR", None)
    if extra_env:
        env.update({str(key): str(value) for key, value in extra_env.items()})
    return env


@contextmanager
def _optional_port_lock(port: int, enabled: bool) -> Iterator[None]:
    if not enabled:
        yield
        return

    lock_path = Path("/tmp") / f"leaderboard_2_run_scene_{int(port)}.lock"
    with open(lock_path, "w", encoding="utf-8") as lock_file:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def _run_command(
    command: List[str],
    *,
    env: Mapping[str, str],
    log_path: Path,
    timeout: Optional[float],
) -> int:
    with open(log_path, "w", encoding="utf-8", errors="replace") as log_file:
        process = subprocess.Popen(
            command,
            cwd=str(REPO_ROOT),
            env=dict(env),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        try:
            return process.wait(timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            _terminate_process_group(process.pid)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                pass
            raise RunSceneError(f"Scenario run timed out after {timeout} seconds. log={log_path}") from exc


def _terminate_process_group(pid: int) -> None:
    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        try:
            os.killpg(pid, 0)
        except ProcessLookupError:
            return
        time.sleep(0.2)
    try:
        os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def _find_mp4(run_dir: Path) -> Optional[Path]:
    candidates = [
        run_dir / "carla_rgb.mp4",
        run_dir / "trajectory_video.mp4",
    ]
    for candidate in candidates:
        if candidate.is_file() and candidate.stat().st_size > 0:
            return candidate
    return None


def _read_tail(path: Path, max_chars: int = 4000) -> str:
    if not path.is_file():
        return ""
    data = path.read_text(encoding="utf-8", errors="replace")
    return data[-max_chars:]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run one OpenDRIVE map and one OpenSCENARIO scene.")
    parser.add_argument("--map", "--xodr", dest="map_path", required=True, help="OpenDRIVE .xodr path")
    parser.add_argument("--scenario", required=True, help="OpenSCENARIO .xosc path")
    parser.add_argument("--output-root", default=None, help="Parent directory for run artifacts")
    parser.add_argument("--run-name", default=None, help="Optional run directory name")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--wait-timeout", type=float, default=180.0)
    parser.add_argument("--process-timeout", type=float, default=None)
    parser.add_argument("--conda-env", default="PCLA")
    parser.add_argument("--agent", default=os.environ.get("AGENT", "tfv6_regnet"),
                        help="PCLA agent id (e.g. tfv6_regnet, if_if, carl_roach); env AGENT overrides default")
    parser.add_argument("--sut-actor", default=None)
    parser.add_argument("--rgb-actor-role", default=None)
    parser.add_argument("--no-lock", action="store_true", help="Do not serialize by CARLA port")
    parser.add_argument("--no-conda", action="store_true", help="Do not activate a conda env in the shell wrapper")
    parser.add_argument("--require-existing-carla", action="store_true")
    parser.add_argument("--keep-carla", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    result = run_scene(
        map_path=args.map_path,
        scenario_path=args.scenario,
        output_root=args.output_root,
        run_name=args.run_name,
        host=args.host,
        port=args.port,
        timeout=args.timeout,
        wait_timeout=args.wait_timeout,
        process_timeout=args.process_timeout,
        conda_env=None if args.no_conda else args.conda_env,
        agent=args.agent,
        sut_actor=args.sut_actor,
        rgb_actor_role=args.rgb_actor_role,
        require_existing_carla=args.require_existing_carla,
        keep_carla=args.keep_carla,
        lock_port=not args.no_lock,
    )
    print(json.dumps(result.to_dict(), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
