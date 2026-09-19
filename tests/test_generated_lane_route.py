from types import SimpleNamespace as NS

import pytest

from runner.runtime_overrides.generated_lane_route import resolve_generated_lane_route


class Map:
    def __init__(self):
        self.calls=[]
    def to_opendrive(self):
        return '<OpenDRIVE><road id="9" length="20"/></OpenDRIVE>'
    def get_waypoint(self, *args, **kwargs):
        raise AssertionError('ambiguous nearest-lane projection must not be used')
    def get_waypoint_xodr(self, road, lane, s):
        self.calls.append((road,lane,s))
        return NS(road_id=road,lane_id=lane,s=s,transform=NS(
            location=NS(x=s,y=1.75),rotation=NS(yaw=0)))


def records():
    return [{'road_id':9,'lane_id':-1,'s':s,'x':s,'y':-1.75,'yaw':0} for s in (0,10,20)]


def test_route_preserves_lane_identity_and_clamps_only_rounded_endpoints():
    road_map=Map();points=resolve_generated_lane_route(road_map,records())
    assert [(p.road_id,p.lane_id) for p in points]==[(9,-1)]*3
    assert [p.s for p in points]==[0.001,10,19.999]


@pytest.mark.parametrize('field,value', [('x',50),('yaw',90),('s',40)])
def test_wrong_map_or_pose_is_rejected(field,value):
    data=records();data[1][field]=value
    with pytest.raises(ValueError):
        resolve_generated_lane_route(Map(),data)
