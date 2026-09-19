import importlib.util
import math
from pathlib import Path

import pytest

path = Path(__file__).resolve().parents[1] / 'runtime_overrides/lane_coordinates.py'
spec = importlib.util.spec_from_file_location('lane_coordinates', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize('yaw,lane,offset,expected', [
    (0, -1, -3.5, (0, 3.5)),  # Right of eastbound travel.
    (180, 1, 3.5, (0, -3.5)),  # Right of westbound travel.
    (-90, -1, -3.5, (3.5, 0)),  # Right of northbound travel.
    (90, 1, 3.5, (-3.5, 0)),  # Right of southbound travel.
    (0, -1, 3.5, (0, -3.5)),  # Positive OpenDRIVE t is left.
])
def test_standard_offsets_follow_road_basis(yaw, lane, offset, expected):
    assert module.lane_offset_delta(yaw, lane, offset) == pytest.approx(expected)


def test_native_carla_offsets_keep_their_existing_convention():
    assert module.lane_offset_delta(0, -1, 3.5, True) == pytest.approx((0, 3.5))
@pytest.mark.parametrize('lane_yaw,heading,absolute,expected', [
    (30, 0, False, 30),
    (180, 0, False, 180),
    (30, math.pi / 2, False, -60),
    (30, math.pi / 2, True, -90),
])
def test_orientation_follows_target_lane_tangent(lane_yaw, heading, absolute, expected):
    from runner.runtime_overrides.lane_coordinates import lane_heading
    assert lane_heading(lane_yaw, heading, absolute) == pytest.approx(expected)
