"""Exercise the public runtime assembly using the pinned Git dependencies."""
import hashlib
import json
import subprocess
import sys

import pytest

from tools.prepare_runtime import ROOT, prepare


def test_runtime_build_is_complete_and_refuses_overwrite(tmp_path):
    output = tmp_path / "runtime"
    result = prepare(output)
    assert result["dependency_patches"] == 13
    manifest = json.loads((output / "crash2openx_runtime_manifest.json").read_text())
    for name in ("extract_roadgraph_carla.py", "src/runs/run_scene.py",
                 "scripts/run_headless_scene_record.sh", "src/demo.py"):
        path = output / name
        assert path.is_file(), f"Runner entry point missing: {name}"
        assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest["files"][name]
    assert (output / "extract_roadgraph_carla.py").read_bytes() == (
        ROOT / "runner/extract_roadgraph_carla.py"
    ).read_bytes()
    help_result = subprocess.run(
        [sys.executable, str(output / "src/runs/run_scene.py"), "--help"],
        capture_output=True, text=True, check=True,
    )
    assert "--scenario" in help_result.stdout
    before = (output / "crash2openx_runtime_manifest.json").read_bytes()
    with pytest.raises(FileExistsError, match="Refusing to replace"):
        prepare(output)
    assert (output / "crash2openx_runtime_manifest.json").read_bytes() == before
