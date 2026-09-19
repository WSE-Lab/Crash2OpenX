import json
import sys

from tools.carla_local import CarlaLocalClient, LocalCfg


def test_local_extraction_uses_local_process_environment_and_preserves_evidence(tmp_path):
    runner = tmp_path / 'runner.sh'
    runner.write_text('''#!/bin/sh
set -eu
out="$RUNS_ROOT/$2/outputs"
mkdir -p "$out"
printf '{"checked":true}' > "$out/roadgraph_selfcheck.json"
cp "$RUNS_ROOT/$2/inputs/map.xodr" "$out/map.xodr"
''')
    source = tmp_path/'input.xodr'
    source.write_text('<OpenDRIVE/>')
    client = CarlaLocalClient(LocalCfg(runner_path=str(runner), runs_root=str(tmp_path/'runs'),
                                      python_bin=sys.executable, min_free_gib=.001))
    result = client.extract_roadgraph(source, force=True, out_root=tmp_path/'cache')
    assert result.selfcheck == {'checked': True}
    assert (result.local_dir/'map.xodr').read_text() == source.read_text()
    assert len(list((tmp_path/'runs').glob('extract_*/inputs/map.xodr'))) == 1
    assert client._runner_env() == ''
