from xml.etree import ElementTree as ET

import pytest

from tools.ads_junction_timing import departure_window


def network(npc_x=50):
    root = ET.Element('OpenSCENARIO')
    for actor, points in [('hero', [(0, 0), (100, 0)]), ('crossing', [(npc_x, -50), (npc_x, 50)])]:
        entity = ET.SubElement(root, 'ScenarioObject', name=actor)
        box = ET.SubElement(entity, 'BoundingBox')
        ET.SubElement(box, 'Center', x='0', y='0')
        ET.SubElement(box, 'Dimensions', length='4', width='2')
        group = ET.SubElement(root, 'ManeuverGroup')
        ET.SubElement(ET.SubElement(group, 'Actors'), 'EntityRef', entityRef=actor)
        route = ET.SubElement(ET.SubElement(group, 'AssignRouteAction'), 'Route')
        for x, y in points:
            ET.SubElement(ET.SubElement(route, 'Waypoint'), 'WorldPosition', x=str(x), y=str(y))
    return root


def test_wide_junction_window_uses_conflict_arrival_instead_of_fixed_distance():
    result = departure_window(network(), 'crossing')
    assert result['distance_m'] > 50
    assert 48 <= result['npc_route_distance_to_conflict_m'] <= 50
    assert 10 < result['ego_onset_route_s_m'] < 15
    assert result['estimated_npc_arrival_s'] > 7
    earlier = departure_window(network(), 'crossing', time_offset=1)
    assert earlier['ego_onset_route_s_m'] == result['ego_onset_route_s_m']-5
    assert earlier['distance_m'] > result['distance_m']


def test_divergent_routes_and_unreachable_start_are_rejected():
    with pytest.raises(ValueError, match='no common conflict'):
        departure_window(network(150), 'crossing')
    with pytest.raises(ValueError, match='precedes'):
        departure_window(network(), 'crossing', ego_speed=50)
