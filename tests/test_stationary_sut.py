import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from tools.osc_blocks import _hero_cruise, build_xosc
from tools.build_road_seed_opendrive import build


def test_stopped_report_av_can_be_approached_by_rear_vehicle(tmp_path):
    """A distance-gated Act never activated the rear car when the AV stood still."""
    scene = {
        "sut": {"id": "ego", "kind": "vehicle", "maneuver": "straight",
                "params": {"initial_speed_mps": 0.0}},
        "npcs": [{"id": "rear", "kind": "vehicle", "position": "behind_same_lane",
                  "side": "none", "behavior": {"block": "rear_hit", "params": {"speed": 4}}}],
        "collision": {"a": "rear", "b": "ego"},
    }
    repo = Path(__file__).resolve().parents[1]
    xodr = tmp_path / "fresh_road.xodr"
    road = {"road": {"topology": "straight", "type": "town",
                     "lanes": {"forward": 1, "backward": 1}, "center_line": "broken"}}
    build(road, xodr, tmp_path / "fresh_road.html", repo / "xsd/OpenDRIVE_1.5M.xsd")
    output = tmp_path / "stopped_av.xosc"
    build_xosc(scene, str(xodr), output)
    root = ET.parse(output).getroot()
    speed = root.find("./Storyboard/Init/Actions/Private[@entityRef='hero']//AbsoluteTargetSpeed")
    assert float(speed.get("value")) == 0
    start = root.find("./Storyboard/Story/Act/StartTrigger")
    assert start.find(".//SimulationTimeCondition") is not None
    assert start.find(".//TraveledDistanceCondition") is None
    assert root.find(".//Event[@name='rear_go']") is not None


@pytest.mark.parametrize("speed", [-1, math.nan, math.inf])
def test_invalid_initial_speed_rejected(speed):
    with pytest.raises(ValueError):
        _hero_cruise({"sut": {"params": {"initial_speed_mps": speed}}})


def test_source_review_has_exact_frozen_case_membership():
    repo = Path(__file__).resolve().parents[1]
    frozen = json.loads((repo / "data/eval/baseline_42.json").read_text())["cases"]
    reviewed = json.loads((repo / "data/eval/baseline_42_source_review.json").read_text())["cases"]
    assert len(reviewed) == len(frozen) == 42
    assert {c["id"] for c in reviewed} == {c["case_id"].split("_")[0] for c in frozen}
