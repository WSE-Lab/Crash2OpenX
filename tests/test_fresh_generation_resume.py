import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import fitz

from tools import coordinator
from tools.resume_fresh_framework_batch import resume, verify_generation


@pytest.fixture
def generated(tmp_path, monkeypatch):
    attempt = tmp_path / "fresh_original"
    attempt.mkdir()
    source = attempt / "source.pdf"
    with fitz.open() as document:
        document.new_page().insert_text((40, 40), "PDF input to mocked VLM")
        document.save(source)
    road = {"status": "supported", "road": {"topology": "straight", "type": "town",
        "lanes": {"forward": 1, "backward": 1}, "center_line": "broken"}}
    scene = {"status": "supported", "scene": {
        "sut": {"id": "ego", "kind": "vehicle", "maneuver": "straight", "params": {}},
        "npcs": [{"id": "v1", "kind": "vehicle", "position": "ahead_same_lane", "side": "none",
                  "behavior": {"block": "stopped_ahead", "params": {}}}],
        "collision": {"a": "ego", "b": "v1"}, "control": "none", "environment": {}}}
    monkeypatch.setattr(coordinator, "extract_any", lambda *a, **k: "A stopped car was rear-ended.")
    monkeypatch.setattr(coordinator, "road_call", lambda *a: copy.deepcopy(road))
    monkeypatch.setattr(coordinator, "scene_call", lambda *a: copy.deepcopy(scene))
    monkeypatch.setattr(coordinator, "road_normalize", lambda raw, *a: raw)
    monkeypatch.setattr(coordinator, "scene_normalize", lambda raw, *a, **k: raw)
    monkeypatch.setattr(coordinator, "run_qa", lambda **kwargs: {"verdict": "pass", "issues": []})

    class NoCarla:
        def __getattr__(self, name):
            raise AssertionError("Generation-only must not call CARLA: " + name)

    result = coordinator.dispatch_full(source, "fixture", attempt / "pipeline",
        generation_only=True, carla_client=NoCarla(), qa_max_retries=0)
    assert result["carla_status"] == "deferred_generation_only"
    assert result["roadgraph_status"] == "deferred"
    (attempt / "framework_code").mkdir()
    (attempt / "framework_invocation.json").write_text(json.dumps({
        "entrypoint": "tools.coordinator.dispatch_full",
        "source_pdf_sha256": hashlib.sha256(source.read_bytes()).hexdigest()}))
    return attempt


def test_offline_generation_resumes_with_real_compiler(generated, tmp_path):
    old_run, original = verify_generation(generated)
    original_bytes = (old_run / "result.json").read_bytes()
    calls = []

    class Client:
        def extract_roadgraph(self, xodr, *, name):
            calls.append(name)
            assert hashlib.sha256(xodr.read_bytes()).hexdigest() == hashlib.sha256(
                (old_run / "fixture.xodr").read_bytes()).hexdigest()
            return SimpleNamespace(selfcheck={"continuity_pass": True}, local_dir=tmp_path / "graph")

        def run_scenario(self, *args, **kwargs):
            raise AssertionError("Compile-only resume must not run PCLA")

    result = resume(generated, tmp_path / "fresh_resumed", Client(), execute=False, max_seconds=60)
    assert result["xosc_status"] == "ok", result["errors"]
    assert result["carla_status"] == "disabled"
    assert len(calls) == 1
    assert (old_run / "result.json").read_bytes() == original_bytes


def test_changed_scene_is_rejected_before_rpc(generated, tmp_path):
    scene_path = generated / "pipeline/fixture/scene_seed.json"
    altered = json.loads(scene_path.read_text())
    altered["scene"]["collision"] = {"a": "ego", "b": "missing"}
    scene_path.write_text(json.dumps(altered))

    class NoCarla:
        def __getattr__(self, name):
            raise AssertionError("Tampered generation must not reach CARLA")

    with pytest.raises(ValueError, match="unverified generation"):
        resume(generated, tmp_path / "fresh_resumed", NoCarla(), execute=True, max_seconds=60)
    assert not (tmp_path / "fresh_resumed").exists()


def test_needs_extension_is_a_rejected_generation_not_an_audit_crash(generated):
    from tools.audit_fresh_framework_batch import audit_attempt
    path = generated / 'pipeline/fixture/result.json'
    result = json.loads(path.read_text())
    result['scene_seed'] = {'status': 'needs_extension', 'scene': None,
                            'reason': 'Ordered secondary contact cannot yet be represented.'}
    path.write_text(json.dumps(result))
    audited = audit_attempt(generated)
    assert audited['generation_pass'] is False
    assert audited['generation_checks']['current_ocl_pass'] is False
    with pytest.raises(ValueError, match='unverified generation'):
        verify_generation(generated)


def test_equivalent_road_rebuild_keeps_parent_and_records_new_compilation(generated, tmp_path):
    from tools.audit_fresh_framework_batch import audit_attempt
    old_run, original = verify_generation(generated)
    before = (old_run / 'result.json').read_bytes()

    class Client:
        def extract_roadgraph(self, xodr, *, name):
            assert xodr.read_bytes() != (old_run / 'fixture.xodr').read_bytes()
            return SimpleNamespace(selfcheck={}, local_dir=tmp_path / 'graph')

    dest = tmp_path / 'fresh_recompiled'
    result = resume(generated, dest, Client(), execute=False, max_seconds=60, recompile_road=True)
    assert result['xosc_status'] == 'ok', result['errors']
    assert audit_attempt(dest)['generation_pass']
    run = Path(result['run_dir'])
    evidence = json.loads((run / 'road_recompile_equivalence.json').read_text())
    assert evidence['equivalent'] and evidence['vlm_qa_rerun'] is False
    assert (old_run / 'result.json').read_bytes() == before
    # Even unchanged XML cannot inherit QA if the recorded parent was modified.
    parent = json.loads(before)
    parent['qa']['final_verdict'] = 'fail'
    (old_run / 'result.json').write_text(json.dumps(parent))
    assert not audit_attempt(dest)['generation_pass']


def test_rebuild_connectivity_change_is_rejected_before_rpc(generated, tmp_path, monkeypatch):
    from tools import resume_fresh_framework_batch as module
    from tools.audit_fresh_framework_batch import audit_attempt
    from xml.etree import ElementTree as ET
    original_build = module.build_road

    def changed_build(seed, xodr, html, xsd):
        result = original_build(seed, xodr, html, xsd)
        tree = ET.parse(xodr)
        lane = tree.getroot().find('.//lane[@type="driving"]')
        link = lane.find('link')
        if link is None:
            link = ET.SubElement(lane, 'link')
        ET.SubElement(link, 'successor', id='999')
        tree.write(xodr)
        return result

    monkeypatch.setattr(module, 'build_road', changed_build)

    class NoCarla:
        def __getattr__(self, name):
            raise AssertionError('Changed connectivity must not reach CARLA')

    dest = tmp_path / 'fresh_rejected'
    result = resume(generated, dest, NoCarla(), execute=False, max_seconds=60, recompile_road=True)
    assert 'new VLM QA required' in result['errors']['road_compile']
    assert not audit_attempt(dest)['generation_pass']
