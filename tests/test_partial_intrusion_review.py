import pytest

from tools.review_partial_intrusion import route_intrusion_review, straight_route_projection


def state(y):
    return {'x':5.,'y':y,'bounding_box_world_vertices':[[x,yy,0] for x in (3.,7.) for yy in (y-1.,y+1.)]}


def test_body_crosses_boundary_while_centre_remains_inside_lane():
    points=[{'x':0.,'y':0.},{'x':10.,'y':0.}]
    frames=[{'simulation_time':t,'actors':{'car':state(y)}} for t,y in [(0,0),(1,-1.4),(2,-2.)]]
    plan={'lane_id':-1,'lane_width_m':3.5,'target_offset_xodr_m':1.4}
    result=route_intrusion_review(points,frames,'car',plan)
    assert result['partial_body_intrusion_samples']==1
    assert result['first_partial_intrusion']['body_intrusion_m']==pytest.approx(.65)
    assert result['centre_outside_route_lane_samples']==1


def test_positive_lane_direction_reverses_xodr_offset_sign():
    points=[{'x':10.,'y':0.},{'x':0.,'y':0.}]
    frames=[{'simulation_time':0,'actors':{'car':state(1.4)}}]
    plan={'lane_id':1,'lane_width_m':3.5,'target_offset_xodr_m':-1.4}
    result=route_intrusion_review(points,frames,'car',plan)
    assert result['target_offset_left_m']==pytest.approx(1.4)
    assert result['partial_body_intrusion_samples']==1


def test_projection_does_not_accept_points_beyond_route_or_curved_route():
    points=[{'x':0.,'y':0.},{'x':10.,'y':0.}]
    actor=state(0);actor['x']=12
    assert not straight_route_projection(points,actor)['within_compiled_endpoints']
    with pytest.raises(ValueError,match='collinear'):
        straight_route_projection([points[0],{'x':5,'y':2},points[1]],actor)
