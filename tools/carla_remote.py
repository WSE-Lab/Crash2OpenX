#!/usr/bin/env python3
"""Remote CARLA client: ship XODR/XOSC to the leaderboard_2.0 server, run there, fetch results.

Two operations:
  * extract_roadgraph(xodr) -> outputs/map_cache/<name>/{waypoints,junctions,legs,route_candidates,roadgraph_selfcheck,manifest}.json
  * run_scenario(xodr, xosc) -> outputs/web_runs/<name>_<ts>/{frame_states.jsonl,events.jsonl,summary.json,...}

Transport: ssh/scp via subprocess (no paramiko dep). Auth: project-scoped ed25519 key.
Cache: extract is content-addressed by xodr sha256; identical XODR + passing selfcheck => skip remote.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import tempfile
import time
import uuid
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
DEFAULT_OUTPUTS = ROOT / "outputs"


def _load_env_local() -> None:
    """Best-effort .env.local loader (no python-dotenv dep)."""
    env_file = ROOT / ".env.local"
    if not env_file.is_file():
        return
    for line in env_file.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        os.environ.setdefault(k.strip(), v.strip())


_load_env_local()


@dataclass(frozen=True)
class RemoteCfg:
    # No real host is baked in: set CARLA_REMOTE_* in .env.local (see
    # .env.example) when CARLA_MODE=remote.
    host: str = ""
    port: int = 22
    user: str = "carla"
    key: Path = Path("~/.ssh/id_ed25519").expanduser()
    project_dir: str = "/home/carla/leaderboard_2.0"
    python_bin: str = "/home/carla/miniconda3/envs/PCLA/bin/python"
    container_name: str = "carla-0916"
    runs_root: str = "/home/carla/crash2openx_runs"
    runner_path: str = "/home/carla/crash2openx_runs/runner.sh"
    rpc_port: int = 2000

    @classmethod
    def from_env(cls) -> "RemoteCfg":
        defaults = cls()
        return cls(
            host=os.environ.get("CARLA_REMOTE_HOST", defaults.host),
            port=int(os.environ.get("CARLA_REMOTE_PORT", defaults.port)),
            user=os.environ.get("CARLA_REMOTE_USER", defaults.user),
            key=Path(os.environ.get("CARLA_REMOTE_KEY", str(defaults.key))).expanduser(),
            project_dir=os.environ.get("CARLA_REMOTE_PROJECT_DIR", defaults.project_dir),
            python_bin=os.environ.get("CARLA_REMOTE_PYTHON", defaults.python_bin),
            container_name=os.environ.get("CARLA_REMOTE_CONTAINER", defaults.container_name),
            runs_root=os.environ.get("CARLA_REMOTE_RUNS_ROOT", defaults.runs_root),
            runner_path=os.environ.get("CARLA_REMOTE_RUNNER", defaults.runner_path),
            rpc_port=int(os.environ.get("CARLA_REMOTE_RPC_PORT", defaults.rpc_port)),
        )


@dataclass
class ExtractResult:
    xodr_sha256: str
    local_dir: Path
    files: dict[str, Path]
    selfcheck: dict
    elapsed_s: float
    cached: bool


@dataclass
class RunResult:
    xodr_sha256: str
    xosc_sha256: str
    local_dir: Path
    files: dict[str, Path]
    summary: dict | None
    elapsed_s: float
    # Layer-3 behavior-vs-intent verdict (None when scene_seed wasn't passed
    # to run_scenario, in which case behavior verification is skipped).
    behavior: dict | None = None


class CarlaRemoteError(RuntimeError):
    pass


class CarlaRemoteClient:
    def __init__(self, cfg: RemoteCfg | None = None):
        self.cfg = cfg or RemoteCfg.from_env()
        if not self.cfg.host:
            raise CarlaRemoteError(
                "CARLA_MODE=remote but CARLA_REMOTE_HOST is not set — "
                "configure the CARLA_REMOTE_* variables in .env.local (see .env.example)"
            )

    # ---- transport primitives ------------------------------------------------

    def _ssh_base(self) -> list[str]:
        return [
            "ssh",
            "-i", str(self.cfg.key),
            "-p", str(self.cfg.port),
            "-o", "IdentitiesOnly=yes",
            "-o", "BatchMode=yes",
            "-o", "ServerAliveInterval=30",
            "-o", "ServerAliveCountMax=10",
            f"{self.cfg.user}@{self.cfg.host}",
        ]

    def _scp_base(self) -> list[str]:
        return [
            "scp",
            "-i", str(self.cfg.key),
            "-P", str(self.cfg.port),
            "-o", "IdentitiesOnly=yes",
            "-o", "BatchMode=yes",
            "-q",
        ]

    def _ssh(self, remote_cmd: str, *, timeout: float | None = None) -> subprocess.CompletedProcess:
        return subprocess.run(
            self._ssh_base() + [remote_cmd],
            capture_output=True, text=True, timeout=timeout, check=False,
        )

    def _push(self, local: Path, remote: str) -> None:
        cp = subprocess.run(
            self._scp_base() + [str(local), f"{self.cfg.user}@{self.cfg.host}:{remote}"],
            capture_output=True, text=True, check=False,
        )
        if cp.returncode != 0:
            raise CarlaRemoteError(f"scp push failed: {cp.stderr.strip()}")

    def _pull(self, remote_glob: str, local_dir: Path) -> None:
        local_dir.mkdir(parents=True, exist_ok=True)
        cp = subprocess.run(
            self._scp_base() + ["-r", f"{self.cfg.user}@{self.cfg.host}:{remote_glob}", str(local_dir)],
            capture_output=True, text=True, check=False,
        )
        if cp.returncode != 0:
            raise CarlaRemoteError(f"scp pull failed: {cp.stderr.strip()}")

    # ---- runner deployment ---------------------------------------------------

    def deploy_runner(self) -> None:
        runner_local = ROOT / "scripts" / "remote_legacy" / "runner.sh"
        paper_renderer_local = ROOT / "scripts" / "remote_legacy" / "visualize_carla_paper.py"
        if not runner_local.is_file():
            raise FileNotFoundError(runner_local)
        if not paper_renderer_local.is_file():
            raise FileNotFoundError(paper_renderer_local)
        self._ssh(f"mkdir -p {shlex.quote(self.cfg.runs_root)}").check_returncode()
        self._push(runner_local, self.cfg.runner_path)
        self._ssh(f"chmod +x {shlex.quote(self.cfg.runner_path)}").check_returncode()
        remote_renderer = f"{self.cfg.project_dir}/src/visualize_carla_paper.py"
        self._push(paper_renderer_local, remote_renderer)
        self._ssh(f"chmod +x {shlex.quote(remote_renderer)}").check_returncode()

    def _runner_env(self) -> str:
        """Env assignments prefixed to every runner.sh invocation, so the
        remote-side script never has to bake in deployment-specific paths."""
        pairs = {
            "RUNS_ROOT": self.cfg.runs_root,
            "PROJECT_DIR": self.cfg.project_dir,
            "PYTHON": self.cfg.python_bin,
            "CONTAINER": self.cfg.container_name,
            "CARLA_PORT": str(self.cfg.rpc_port),
        }
        return " ".join(f"{k}={shlex.quote(v)}" for k, v in pairs.items())

    # ---- operations ----------------------------------------------------------

    def extract_roadgraph(
        self,
        xodr_path: Path,
        *,
        name: str | None = None,
        spacing: float = 2.0,
        force: bool = False,
        out_root: Path | None = None,
        timeout: float = 300.0,
    ) -> ExtractResult:
        xodr_path = Path(xodr_path).resolve()
        if not xodr_path.is_file():
            raise FileNotFoundError(xodr_path)
        name = name or xodr_path.stem
        xodr_bytes = xodr_path.read_bytes()
        xodr_sha = hashlib.sha256(xodr_bytes).hexdigest()

        out_root = out_root or (DEFAULT_OUTPUTS / "map_cache")
        local_dir = out_root / name

        if not force:
            cached = _load_extract_cache(local_dir, xodr_sha, xodr_path)
            if cached is not None:
                return cached

        run_id = f"extract_{name}_{uuid.uuid4().hex[:8]}"
        remote_run_dir = f"{self.cfg.runs_root}/{run_id}"

        t0 = time.time()
        self._ssh(
            f"mkdir -p {shlex.quote(remote_run_dir)}/inputs {shlex.quote(remote_run_dir)}/outputs"
        ).check_returncode()
        self._push(xodr_path, f"{remote_run_dir}/inputs/map.xodr")

        cmd = (
            f"{self._runner_env()} bash {shlex.quote(self.cfg.runner_path)} extract "
            f"{shlex.quote(run_id)} --spacing {spacing}"
        )
        cp = self._ssh(cmd, timeout=timeout)
        if cp.returncode != 0:
            raise CarlaRemoteError(
                f"remote extract failed (rc={cp.returncode})\n"
                f"--- last stdout ---\n{cp.stdout[-2000:]}\n"
                f"--- last stderr ---\n{cp.stderr[-2000:]}"
            )

        if local_dir.exists():
            subprocess.run(["rm", "-rf", str(local_dir)], check=False)
        self._pull(f"{remote_run_dir}/outputs", local_dir.parent)
        # scp -r outputs <parent> -> parent/outputs ; rename to <name>
        outputs_dir = local_dir.parent / "outputs"
        if outputs_dir.exists():
            outputs_dir.rename(local_dir)

        self._ssh(f"rm -rf {shlex.quote(remote_run_dir)}", timeout=15)

        # Side-car cache key: byte-level hash of the local XODR we sent.
        # The script's manifest.xodr_sha256 hashes normalized text and won't match this.
        (local_dir / ".input_sha256").write_text(xodr_sha + "\n")

        files = {p.name: p for p in local_dir.iterdir() if p.is_file()}
        selfcheck = _load_json(files.get("roadgraph_selfcheck.json")) or {}

        return ExtractResult(
            xodr_sha256=xodr_sha,
            local_dir=local_dir,
            files=files,
            selfcheck=selfcheck,
            elapsed_s=time.time() - t0,
            cached=False,
        )

    def run_scenario(
        self,
        xodr_path: Path,
        xosc_path: Path,
        *,
        name: str | None = None,
        pcla_agent: str = "tfv6_regnet",
        sut_actor: str | None = None,
        rgb_actor_role: str | None = None,
        max_seconds: int = 60,
        out_root: Path | None = None,
        out_dir: Path | None = None,
        timeout: float = 1800.0,
        scene_seed: dict | None = None,
        road_seed: dict | None = None,
        paper_render: bool = False,
        retries: int = 1,
    ) -> RunResult:
        """Run a scenario; retry once on PCLA silent-spawn fail.

        Silent-spawn pattern: remote runner reports spawn_success=true and the
        runtime xosc loaded cleanly, but total_ticks==0 because PCLA's
        setup_sensors failed to attach to a hero actor that CARLA quietly
        destroyed at spawn (typically z-collision with the road mesh). A single
        retry on a freshly-spawned world tends to clear it.
        """
        last: RunResult | None = None
        for attempt in range(retries + 1):
            last = self._run_scenario_once(
                xodr_path, xosc_path,
                name=name, pcla_agent=pcla_agent, sut_actor=sut_actor,
                rgb_actor_role=rgb_actor_role, max_seconds=max_seconds,
                out_root=out_root, out_dir=out_dir, timeout=timeout,
                scene_seed=scene_seed, road_seed=road_seed,
                paper_render=paper_render,
            )
            if not _is_silent_spawn_fail(last.summary):
                return last
            if attempt < retries:
                print(f"[carla_remote] silent-spawn fail (spawn_success=true, "
                      f"total_ticks=0); retrying ({attempt+1}/{retries})...", flush=True)
        return last  # type: ignore[return-value]

    def _run_scenario_once(
        self,
        xodr_path: Path,
        xosc_path: Path,
        *,
        name: str | None = None,
        pcla_agent: str = "tfv6_regnet",
        sut_actor: str | None = None,
        rgb_actor_role: str | None = None,
        max_seconds: int = 60,
        out_root: Path | None = None,
        out_dir: Path | None = None,
        timeout: float = 1800.0,
        scene_seed: dict | None = None,
        road_seed: dict | None = None,
        paper_render: bool = False,
    ) -> RunResult:
        xodr_path = Path(xodr_path).resolve()
        xosc_path = Path(xosc_path).resolve()
        for p in (xodr_path, xosc_path):
            if not p.is_file():
                raise FileNotFoundError(p)
        name = name or xosc_path.stem

        # P0: auto-detect SUT when the caller didn't pin it. Both the old
        # ADS-test xosc (SUT=v2) and the new osc_blocks output (SUT=hero) tag
        # the externally-driven actor the same way, so we don't need separate
        # heuristics per format.
        if sut_actor is None:
            hits = _detect_sut_actor(xosc_path)
            if len(hits) == 1:
                sut_actor = hits[0]
            else:
                raise CarlaRemoteError(
                    "could not auto-detect SUT actor in xosc "
                    f"({xosc_path.name}): found {hits!r} with "
                    "module=external_control. Pass --sut-actor explicitly."
                )

        xodr_sha = hashlib.sha256(xodr_path.read_bytes()).hexdigest()
        xosc_sha = hashlib.sha256(xosc_path.read_bytes()).hexdigest()

        # Two layout modes:
        # - out_dir set (merge mode, used by dispatch_full): products are merged
        #   into the existing run dir, no ts suffix, no rename-or-clobber.
        # - out_dir unset (legacy CLI mode): products land in <out_root>/<name>_<ts>/.
        if out_dir is not None:
            local_dir = Path(out_dir).resolve()
            local_dir.mkdir(parents=True, exist_ok=True)
            merge_mode = True
        else:
            out_root = out_root or (DEFAULT_OUTPUTS / "web_runs")
            ts = time.strftime("%Y%m%d_%H%M%S")
            local_dir = out_root / f"{name}_{ts}"
            merge_mode = False

        run_id = f"run_{name}_{uuid.uuid4().hex[:8]}"
        remote_run_dir = f"{self.cfg.runs_root}/{run_id}"

        t0 = time.time()
        self._ssh(
            f"mkdir -p {shlex.quote(remote_run_dir)}/inputs {shlex.quote(remote_run_dir)}/outputs"
        ).check_returncode()
        self._push(xodr_path, f"{remote_run_dir}/inputs/map.xodr")
        self._push(xosc_path, f"{remote_run_dir}/inputs/scenario.xosc")

        paper_meta_payload: dict | None = None
        if paper_render:
            resolved_road_seed = road_seed or _load_json(xodr_path.parent / "road_seed.json") or {}
            resolved_scene_seed = scene_seed or _load_json(xosc_path.parent / "scene_seed.json") or {}
            paper_meta_payload = {
                "road": resolved_road_seed.get("road", resolved_road_seed),
                "scene": resolved_scene_seed.get("scene", resolved_scene_seed),
            }
            with tempfile.NamedTemporaryFile("w", suffix=".json", encoding="utf-8") as handle:
                json.dump(paper_meta_payload, handle, ensure_ascii=False, indent=2)
                handle.flush()
                self._push(Path(handle.name), f"{remote_run_dir}/inputs/paper_meta.json")

        extra: list[str] = ["--pcla-agent", pcla_agent, "--max-seconds", str(max_seconds)]
        if sut_actor:
            extra += ["--sut-actor", sut_actor]
        if rgb_actor_role:
            extra += ["--rgb-actor-role", rgb_actor_role]
        if paper_render:
            extra += ["--paper-render"]
        extra_str = " ".join(shlex.quote(a) for a in extra)

        cmd = (
            f"{self._runner_env()} bash {shlex.quote(self.cfg.runner_path)} run "
            f"{shlex.quote(run_id)} {extra_str}"
        ).rstrip()
        cp = self._ssh(cmd, timeout=timeout)
        if cp.returncode != 0:
            # P1: rescue partial artifacts + runner.log before razing the remote
            # dir so failures are debuggable. In merge mode we stash them under
            # <run_dir>/_failed_<ts>/ to avoid clobbering coordinator products.
            if merge_mode:
                failed_dir = local_dir / f"_failed_{time.strftime('%Y%m%d_%H%M%S')}"
            else:
                failed_dir = local_dir.with_name(local_dir.name + "_FAILED")
            try:
                self._pull(f"{remote_run_dir}/outputs", failed_dir.parent)
                outputs_dir = failed_dir.parent / "outputs"
                if outputs_dir.exists():
                    if failed_dir.exists():
                        subprocess.run(["rm", "-rf", str(failed_dir)], check=False)
                    outputs_dir.rename(failed_dir)
                else:
                    failed_dir.mkdir(parents=True, exist_ok=True)
                self._pull(f"{remote_run_dir}/runner.log", failed_dir)
            except CarlaRemoteError:
                pass
            self._ssh(f"rm -rf {shlex.quote(remote_run_dir)}", timeout=15)
            raise CarlaRemoteError(
                f"remote run failed (rc={cp.returncode}); partial artifacts at {failed_dir}\n"
                f"--- last stdout ---\n{cp.stdout[-2000:]}\n"
                f"--- last stderr ---\n{cp.stderr[-2000:]}"
            )

        self._pull(f"{remote_run_dir}/outputs", local_dir.parent)
        outputs_dir = local_dir.parent / "outputs"
        if outputs_dir.exists():
            if merge_mode:
                # Merge pulled outputs/ into the existing run dir (same-name overwrites,
                # subdir contents merged). Avoids clobbering coordinator products.
                _merge_into(outputs_dir, local_dir)
                shutil.rmtree(outputs_dir, ignore_errors=True)
            else:
                outputs_dir.rename(local_dir)

        self._ssh(f"rm -rf {shlex.quote(remote_run_dir)}", timeout=15)

        if paper_meta_payload is not None:
            (local_dir / "paper_meta.json").write_text(
                json.dumps(paper_meta_payload, ensure_ascii=False, indent=2), encoding="utf-8")

        # Drop a copy of the XODR alongside outputs so the replay viewer can
        # render the road background without reaching back to opendrive_seed/.
        shutil.copy2(xodr_path, local_dir / "map.xodr")

        files = {p.name: p for p in local_dir.iterdir() if p.is_file()}
        summary = _load_json(files.get("summary.json"))

        # Layer-3 behavior-vs-intent gate. When the caller passes scene_seed (or
        # we can find scene_seed.json next to the xosc), diff intent against the
        # actual CARLA outcome. Saves <run_dir>/behavior_check.json so direct
        # CLI runs catch the same kinematics-deadlock / U-turn anti-patterns
        # that earlier slipped through XSD-valid + no-error scenarios.
        from tools.scene_outcome import check_scene_outcome
        seed_for_check = scene_seed
        if seed_for_check is None:
            sibling = xosc_path.parent / "scene_seed.json"
            if sibling.is_file():
                try:
                    seed_for_check = json.loads(sibling.read_text())
                except Exception:
                    seed_for_check = None
        behavior = None
        if seed_for_check is not None:
            sim_fb_path = files.get("sim_feedback.json") or (local_dir / "sim_feedback.json")
            behavior = check_scene_outcome(seed_for_check, summary, sim_fb_path)
            (local_dir / "behavior_check.json").write_text(
                json.dumps(behavior, ensure_ascii=False, indent=2), encoding="utf-8")
            files["behavior_check.json"] = local_dir / "behavior_check.json"

        return RunResult(
            xodr_sha256=xodr_sha,
            xosc_sha256=xosc_sha,
            local_dir=local_dir,
            files=files,
            summary=summary,
            elapsed_s=time.time() - t0,
            behavior=behavior,
        )


# ---- helpers ----------------------------------------------------------------


def _is_silent_spawn_fail(summary: dict | None) -> bool:
    """Detect the PCLA silent-spawn pattern in a summary.json dict.

    Symptom: scenario reportedly completed (termination_reason=scenario_completed
    or anything with total_ticks==0) AND no actor was actually driven. Distinguish
    from a legitimate max-seconds timeout (signal:15 with ticks>0).
    """
    if not summary:
        return False
    if summary.get("total_ticks", 0) > 0:
        return False
    term = summary.get("termination_reason") or ""
    # An immediate failure of any kind that produced zero ticks is the spawn pattern.
    # Even "scenario_completed" with 0 ticks means OpenScenario init threw silently.
    return term in ("scenario_completed", "") or "exception" in term.lower()


def _load_json(path: Path | None) -> dict | None:
    if path is None or not path.is_file():
        return None
    try:
        return json.loads(path.read_text())
    except Exception:
        return None


def _merge_into(src: Path, dst: Path) -> None:
    """Move every entry from ``src`` into ``dst``, overwriting same-name targets.

    Used by ``run_scenario`` when called in merge mode so CARLA products land in
    the existing coordinator run dir without clobbering road_seed.json etc.
    """
    dst.mkdir(parents=True, exist_ok=True)
    for entry in src.iterdir():
        target = dst / entry.name
        if entry.is_dir():
            if target.exists() and target.is_dir():
                shutil.copytree(entry, target, dirs_exist_ok=True)
                shutil.rmtree(entry, ignore_errors=True)
            else:
                if target.exists():
                    target.unlink()
                shutil.move(str(entry), str(target))
        else:
            if target.exists():
                target.unlink()
            shutil.move(str(entry), str(target))


def _detect_sut_actor(xosc_path: Path) -> list[str]:
    """Scan xosc for Private/AssignControllerAction with module=external_control.

    Returns the list of entityRef names so the caller can distinguish 0/1/many.
    The PCLA convention (shared by both old ADS-test xosc and the new osc_blocks
    output) is to mark the SUT with a Controller carrying
    Property name="module" value="external_control".
    """
    try:
        root = ET.parse(xosc_path).getroot()
    except ET.ParseError:
        return []
    hits: list[str] = []
    for private in root.iter("Private"):
        entity = private.attrib.get("entityRef")
        if not entity:
            continue
        for prop in private.iter("Property"):
            if (prop.attrib.get("name") == "module"
                    and prop.attrib.get("value") == "external_control"):
                hits.append(entity)
                break
    return hits


def _load_extract_cache(local_dir: Path, xodr_sha: str, xodr_path: Path) -> ExtractResult | None:
    sha_file = local_dir / ".input_sha256"
    if not sha_file.is_file() or sha_file.read_text().strip() != xodr_sha:
        return None
    selfcheck = _load_json(local_dir / "roadgraph_selfcheck.json")
    if not selfcheck or not selfcheck.get("continuity_pass", False):
        return None
    files = {p.name: p for p in local_dir.iterdir() if p.is_file()}
    return ExtractResult(
        xodr_sha256=xodr_sha,
        local_dir=local_dir,
        files=files,
        selfcheck=selfcheck,
        elapsed_s=0.0,
        cached=True,
    )


# ---- CLI --------------------------------------------------------------------


def _cli() -> int:
    ap = argparse.ArgumentParser(description="Remote CARLA client (extract + run)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("deploy-runner", help="scp scripts/remote_legacy/runner.sh to the server")

    ex = sub.add_parser("extract", help="extract roadgraph map_cache from an XODR")
    ex.add_argument("--xodr", required=True, type=Path)
    ex.add_argument("--name", default=None)
    ex.add_argument("--spacing", type=float, default=2.0)
    ex.add_argument("--force", action="store_true", help="bypass local sha256 cache")

    rn = sub.add_parser("run", help="run a scenario via src/runs/run_scene.py (real CARLA RGB + ADS)")
    rn.add_argument("--xodr", required=True, type=Path)
    rn.add_argument("--xosc", required=True, type=Path)
    rn.add_argument("--name", default=None)
    rn.add_argument("--pcla-agent", default="tfv6_regnet",
                    help="PCLA agent id; default tfv6_regnet")
    rn.add_argument("--sut-actor", default=None,
                    help="role_name of the SUT vehicle; if omitted, auto-detected "
                         "from the xosc Private with module=external_control")
    rn.add_argument("--rgb-actor-role", default=None,
                    help="POV actor for the RGB camera; default same as --sut-actor")
    rn.add_argument("--max-seconds", type=int, default=60,
                    help="hard wall-clock cap per scenario (default 60s)")
    rn.add_argument("--paper-render", action="store_true",
                    help="also record a 1920x1080 annotated metamodel view")
    rn.add_argument("--road-seed", type=Path, default=None,
                    help="road_seed.json used by --paper-render; defaults next to XODR")
    rn.add_argument("--scene-seed", type=Path, default=None,
                    help="scene_seed.json used by --paper-render; defaults next to XOSC")

    args = ap.parse_args()
    client = CarlaRemoteClient()

    if args.cmd == "deploy-runner":
        client.deploy_runner()
        print(json.dumps({"deployed_to": client.cfg.runner_path}, indent=2))
        return 0

    if args.cmd == "extract":
        result = client.extract_roadgraph(
            args.xodr, name=args.name, spacing=args.spacing, force=args.force,
        )
        manifest = _load_json(result.files.get("manifest.json")) or {}
        print(json.dumps({
            "xodr_sha256": result.xodr_sha256[:16] + "...",
            "local_dir": str(result.local_dir),
            "elapsed_s": round(result.elapsed_s, 3),
            "cached": result.cached,
            "continuity_pass": result.selfcheck.get("continuity_pass"),
            "waypoint_count": manifest.get("waypoint_count"),
            "junction_count": manifest.get("junction_count"),
            "route_count": manifest.get("route_count"),
            "files": sorted(result.files.keys()),
        }, indent=2))
        return 0

    if args.cmd == "run":
        road_seed = _load_json(args.road_seed) if args.road_seed else None
        scene_seed = _load_json(args.scene_seed) if args.scene_seed else None
        result = client.run_scenario(
            args.xodr, args.xosc, name=args.name,
            pcla_agent=args.pcla_agent,
            sut_actor=args.sut_actor,
            rgb_actor_role=args.rgb_actor_role,
            max_seconds=args.max_seconds,
            road_seed=road_seed,
            scene_seed=scene_seed,
            paper_render=args.paper_render,
        )
        print(json.dumps({
            "xosc_sha256": result.xosc_sha256[:16] + "...",
            "local_dir": str(result.local_dir),
            "elapsed_s": round(result.elapsed_s, 3),
            "summary_keys": sorted((result.summary or {}).keys()),
            "termination_reason": (result.summary or {}).get("termination_reason"),
            "collision_count": (result.summary or {}).get("collision_count"),
            "files_count": len(result.files),
        }, indent=2))
        return 0

    return 2


if __name__ == "__main__":
    raise SystemExit(_cli())
