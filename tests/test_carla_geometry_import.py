import importlib.util
import math
import sys
import json
from pathlib import Path
from types import SimpleNamespace as NS
from xml.etree import ElementTree as ET

import pytest

from tools.opendrive_repair import repair_opendrive_xml
from tools.build_road_seed_opendrive import build


def spiral_xml(start, end, length=20):
    return f'''<OpenDRIVE><road id="1" length="{length}" junction="-1">
      <planView><geometry s="0" x="0" y="0" hdg="0" length="{length}">
      <spiral curvStart="{start}" curvEnd="{end}"/></geometry></planView>
      </road></OpenDRIVE>'''


@pytest.mark.parametrize('start,end,kind', [
    (-0.04606323107392255, -0.04606323107392257, 'arc'),
    (0, 0, 'line'), (0, 0.02, 'spiral'),
    (0.02, 0.02000000001, 'spiral')])
def test_only_degenerate_spirals_are_canonicalized(start, end, kind):
    fixed, _ = repair_opendrive_xml(spiral_xml(start, end), straighten=False)
    geometry = ET.fromstring(fixed).find('.//geometry')
    assert geometry[0].tag == kind
    assert geometry.get('length') == '20'
    again, changes = repair_opendrive_xml(fixed, straighten=False)
    assert again == fixed and changes == 0


def test_generated_junction_already_has_importable_arcs(tmp_path):
    path = tmp_path / 'new.xodr'
    root = Path(__file__).resolve().parents[1]
    build({'road': {'topology': 'cross_intersection', 'type': 'town',
                   'lanes': {'forward': 1, 'backward': 1}, 'center_line': 'broken'}},
          path, tmp_path/'new.html', root/'xsd/OpenDRIVE_1.5M.xsd')
    tree = ET.parse(path)
    assert tree.findall('.//arc')
    assert tree.findall('.//spiral')
    for sp in tree.findall('.//spiral'):
        assert not math.isclose(float(sp.get('curvStart')), float(sp.get('curvEnd')), rel_tol=1e-12, abs_tol=1e-15)
    _, changes = repair_opendrive_xml(path.read_text())
    assert changes == 0


@pytest.fixture
def extractor(monkeypatch):
    monkeypatch.setitem(sys.modules, 'carla', NS())
    path = Path(__file__).resolve().parents[1] / 'runner/extract_roadgraph_carla.py'
    spec = importlib.util.spec_from_file_location('geometry_extractor_test', path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def test_extractor_keeps_native_positions_and_rejects_importer_disagreement(extractor, monkeypatch):
    point = NS(road_id=1, lane_id=-1, s=10,
               transform=NS(location=NS(x=3, y=4), rotation=NS(yaw=-30)))
    monkeypatch.setattr(extractor, 'xodr_lane_pose', lambda *args: (20, -4, 30))
    assert extractor.pose_for_wp(point, {}) == (3, -4, 30)
    check = extractor.check_imported_geometry({'p': point}, {})
    assert not check['geometry_consistency_pass']
    assert check['geometry_max_distance_m'] == 17
    assert check['geometry_mismatch_count'] == 1
    monkeypatch.setattr(extractor, 'xodr_lane_pose', lambda *args: (3, -4, 30))
    assert extractor.check_imported_geometry({'p': point}, {})['geometry_consistency_pass']


def test_unavailable_analytic_geometry_is_explicitly_unchecked(extractor, monkeypatch):
    point = NS(road_id=1, lane_id=-1, s=10)
    monkeypatch.setattr(extractor, 'xodr_lane_pose', lambda *args: None)
    check = extractor.check_imported_geometry({'p': point}, {})
    assert not check['geometry_consistency_pass']
    assert check['geometry_unchecked'] == ['p']


def test_pre_geometry_check_cache_is_not_reused(tmp_path):
    from tools.carla_remote import _load_extract_cache
    (tmp_path/'.input_sha256').write_text('map_digest\n')
    check = tmp_path/'roadgraph_selfcheck.json'
    check.write_text(json.dumps({'continuity_pass': True}))
    assert _load_extract_cache(tmp_path, 'map_digest', tmp_path/'map.xodr') is None
    check.write_text(json.dumps({'continuity_pass': True, 'geometry_check_version': 1,
                                 'geometry_consistency_pass': True}))
    assert _load_extract_cache(tmp_path, 'map_digest', tmp_path/'map.xodr').cached
