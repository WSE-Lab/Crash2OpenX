import hashlib
import json

from tools import coordinator


def test_scene_schema_retry_retains_raw_failure_without_changing_source(tmp_path, monkeypatch):
    from types import SimpleNamespace
    args = SimpleNamespace(fix_hint='original QA guidance')
    raw = iter([{'invalid': 'step trigger'}, {'status': 'needs_extension', 'scene': None}])
    hints = []
    def call(a, doc):
        hints.append(a.fix_hint)
        assert doc == {'event_description': 'original source'}
        return next(raw)
    def normalize(value, *args, **kwargs):
        if 'invalid' in value:
            raise ValueError('step 2 needs a supported condition')
        return value
    monkeypatch.setattr(coordinator, 'scene_call', call)
    monkeypatch.setattr(coordinator, 'scene_normalize', normalize)
    result = coordinator._infer_scene(args, {'event_description': 'original source'}, tmp_path/'source.pdf', 'test')
    assert result['status'] == 'needs_extension'
    assert 'step 2 needs a supported condition' in hints[1]
    assert args.normalization_attempts[0]['raw_output'] == {'invalid': 'step trigger'}
    assert len(args.normalization_attempts) == 2


def test_failed_retry_cannot_reuse_previous_generated_map(tmp_path, monkeypatch):
    source = tmp_path / "source.txt"
    source.write_text("A rear-end collision on a straight two-way road.")
    road = {"status": "supported", "road": {"topology": "straight", "type": "town",
            "lanes": {"forward": 1, "backward": 1}, "center_line": "broken"}}
    monkeypatch.setattr(coordinator, "road_call", lambda args, doc: road)
    monkeypatch.setattr(coordinator, "road_normalize", lambda raw, *args: raw.copy())
    monkeypatch.setattr(coordinator, "scene_call", lambda args, doc: {"status": "supported", "scene": {}})
    monkeypatch.setattr(coordinator, "scene_normalize", lambda raw, *args, **kwargs: raw.copy())
    arguments = dict(doc={"event_description": source.read_text()}, src_path=source,
                     name="fresh", run_dir=tmp_path, xsd_path=coordinator.DEFAULT_XSD,
                     model="test", base_url="unused", api_key_env="unused",
                     fix_hint_road="", fix_hint_scene="")
    first = coordinator._run_one_round(**arguments)
    assert first["xodr_schema_valid"]
    proof = json.loads((tmp_path / "road_generation.json").read_text())
    assert proof["xodr_sha256"] == hashlib.sha256((tmp_path / "fresh.xodr").read_bytes()).hexdigest()
    assert proof["seed_sha256"] == hashlib.sha256((tmp_path / "road_seed.json").read_bytes()).hexdigest()

    monkeypatch.setattr(coordinator, "road_call", lambda args, doc: {"status": "unsupported", "road": None})
    second = coordinator._run_one_round(**arguments)
    assert second["xodr_path"] is None
    assert not (tmp_path / "road_generation.json").exists()
    assert not (tmp_path / "fresh.xodr").exists()
    assert not (tmp_path / "fresh.html").exists()
