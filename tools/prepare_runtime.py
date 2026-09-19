"""Build an isolated CARLA runtime from pinned dependencies and reviewed patches.

Never edits the external submodules or an existing runtime directory.
"""
from __future__ import annotations
import argparse
import hashlib
import io
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def prepare(output: Path) -> dict:
    output = output.resolve()
    if output.exists():
        raise FileExistsError(f'Refusing to replace existing runtime: {output}')
    manifest = json.loads((ROOT/'runner/patches/manifest.json').read_text())
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='c2x-runtime-', dir=output.parent) as temporary:
        stage = Path(temporary)/'runtime'
        stage.mkdir()
        for name, revision in manifest['submodules'].items():
            dependency = ROOT/'external'/name
            actual = subprocess.check_output(['git', '-C', str(dependency), 'rev-parse', 'HEAD'], text=True).strip()
            if actual != revision:
                raise ValueError(f'{name}: expected pinned revision {revision}, got {actual}')
            archive = subprocess.check_output(['git', '-C', str(dependency), 'archive', 'HEAD'])
            destination = stage/name
            destination.mkdir()
            with tarfile.open(fileobj=io.BytesIO(archive)) as bundle:
                bundle.extractall(destination, filter='data')
        # A temporary git root prevents git apply from resolving the parent checkout.
        subprocess.run(['git', 'init', '-q', str(stage)], check=True)
        patch = ROOT/'runner/patches/showcase-runtime.patch'
        subprocess.run(['git', 'apply', '--check', str(patch)], cwd=stage, check=True)
        subprocess.run(['git', 'apply', str(patch)], cwd=stage, check=True)
        shutil.rmtree(stage/'.git')  # only the temporary directory created above
        for name in ('src', 'scripts'):
            shutil.copytree(ROOT/'runner'/name, stage/name,
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '.DS_Store', 'runs'))
        shutil.copy2(ROOT/'runner/extract_roadgraph_carla.py', stage/'extract_roadgraph_carla.py')
        (stage/'src/runs').mkdir(exist_ok=True)
        for name in ('run_scene.py', '__init__.py'):
            shutil.copy2(ROOT/'runner/src/runs'/name, stage/'src/runs'/name)
        for name in ('guarded_conditions', 'lane_coordinates', 'lane_offset_motion', 'standstill_condition'):
            shutil.copy2(ROOT/'runner/runtime_overrides'/f'{name}.py', stage/'src'/f'{name}.py')
        for name in ('opendrive_repair', 'scene_contacts'):
            shutil.copy2(ROOT/'tools'/f'{name}.py', stage/'src'/f'{name}.py')
        for name, expected in manifest['files'].items():
            actual = hashlib.sha256((stage/name).read_bytes()).hexdigest()
            if actual != expected['runtime_sha256']:
                raise ValueError('Runtime patch hash mismatch: '+name)
        report = {'submodules': manifest['submodules'], 'files': {
            str(p.relative_to(stage)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(stage.rglob('*')) if p.is_file()
        }}
        (stage/'crash2openx_runtime_manifest.json').write_text(json.dumps(report, indent=2)+'\n')
        stage.rename(output)
    return {'runtime': str(output), 'verified_files': len(report['files']),
            'dependency_patches': len(manifest['files'])}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    print(json.dumps(prepare(parser.parse_args().output)))
