import math

import pytest

from tools.xodr_corridor_review import CorridorMap


def make_map(tmp_path, geometry='<line/>', hdg=0, width='a="3.5" b="0"', parking=False):
    p = tmp_path/'map.xodr'
    park = '<lane id="-1" type="parking"><width sOffset="0" a="2.5"/></lane>' if parking else ''
    lid = '-2' if parking else '-1'
    p.write_text(f'''<OpenDRIVE><road id="1"><planView><geometry s="0" x="0" y="0" hdg="{hdg}" length="20">{geometry}</geometry></planView>
      <lanes><laneSection s="0"><right>{park}<lane id="{lid}" type="driving"><width sOffset="0" {width}/></lane></right></laneSection></lanes></road></OpenDRIVE>''')
    return CorridorMap(p)


def test_line_membership_respects_lane_width_endpoints_and_parking(tmp_path):
    m=make_map(tmp_path, parking=True)
    assert m.project(10, -4.25)['classification'] == 'inside'
    assert m.project(10, -4.25)['lane_id'] == -2
    assert m.project(10, -1)['classification'] == 'outside'  # parking is not driving
    assert m.project(21, -4.25)['classification'] == 'outside'  # beyond the road end


@pytest.mark.parametrize('geometry', ['<arc curvature="0.03"/>', '<spiral curvStart="0.01" curvEnd="0.05"/>'])
def test_curved_lane_normal_projection_uses_analytic_geometry(tmp_path, geometry):
    from pyclothoids import Clothoid
    m=make_map(tmp_path, geometry=geometry, hdg=.7)
    c=Clothoid.StandardParams(0, 0, .7, .03 if geometry.startswith('<arc') else .01,
                             0 if geometry.startswith('<arc') else .002, 20)
    s=8; h=c.Theta(s)
    inside=m.project(c.X(s)+1.75*math.sin(h), c.Y(s)-1.75*math.cos(h))
    assert inside['classification'] == 'inside'
    assert inside['s'] == pytest.approx(s, abs=1e-5)
    assert inside['center_distance_m'] < 1e-5
    assert m.project(c.X(s)+4*math.sin(h), c.Y(s)-4*math.cos(h))['classification'] == 'outside'


def test_nonconstant_width_is_unresolved_instead_of_false_pass(tmp_path):
    m=make_map(tmp_path, width='a="3.5" b="0.1"')
    assert m.project(10, -1.75)['classification'] == 'unresolved'
    assert m.unsupported


def test_raw_projection_error_is_retained_but_not_treated_as_proven_excursion(tmp_path):
    import json
    from tools.scene_outcome import check_scene_outcome
    make_map(tmp_path)
    feedback = tmp_path/'sim_feedback.json'
    feedback.write_text(json.dumps({'summary': {'collision_detected': True,
        'collision_actors': ['hero', 'npc']}, 'behavior_checks': {'impact_area_estimated': 'rear'}}))
    rows = [{'simulation_time': i*.05, 'actors': {'hero': {
        'x': 5+i*.1, 'y': 1.75, 'road_id': 1, 'lane_id': -1,
        'nearest_driving_waypoint_distance_m': 16}}} for i in range(10)]
    trace = tmp_path/'sim_trace_raw.jsonl'
    trace.write_text('\n'.join(json.dumps(row) for row in rows))
    summary = {'off_road_time': 2, 'lane_invasion_count': 0}
    scene = {'scene': {'collision': {'a': 'ego', 'b': 'npc'}}}
    result = check_scene_outcome(scene, summary, feedback)
    assert result['off_road_time'] == 2
    assert result['max_lateral_excess_m'] == 14.25  # Raw projection remains visible.
    assert result['lane_corridor_review']['inside_samples'] == 10
    assert any('conflicts' in w for w in result['warnings'])
    assert not any('off-road trajectory excursion' in issue for issue in result['issues'])
    # Missing observations must not establish complete containment.
    rows[4]['actors'] = {}
    trace.write_text('\n'.join(json.dumps(row) for row in rows))
    result = check_scene_outcome(scene, summary, feedback)
    assert result['lane_corridor_review']['missing_actor_samples'] == 1
    assert any('off-road trajectory excursion' in issue for issue in result['issues'])
