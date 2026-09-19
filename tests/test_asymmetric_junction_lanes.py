from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
import xmlschema

from tools.build_road_seed_opendrive import build

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('topology', ['cross_intersection', 't_junction', 'y_junction'])
@pytest.mark.parametrize('forward,backward', [(2, 1), (1, 2), (3, 2)])
def test_asymmetric_incoming_lanes_have_real_connectors(topology, forward, backward, tmp_path):
    road = {'road': {'topology': topology, 'type': 'town',
                    'lanes': {'forward': forward, 'backward': backward}, 'center_line': 'broken'}}
    output = tmp_path / 'new.xodr'
    build(road, output, tmp_path / 'new.html', ROOT / 'xsd/OpenDRIVE_1.5M.xsd')
    tree = ET.parse(output)
    for incoming in tree.findall('./road[@junction="-1"]'):
        lane_ids = {int(l.get('id')) for l in incoming.findall('./lanes/laneSection/right/lane[@type="driving"]')}
        assert lane_ids == set(range(-forward, 0))
        connections = tree.findall(f'./junction/connection[@incomingRoad="{incoming.get("id")}"]')
        connected = {int(l.get('from')) for c in connections for l in c.findall('laneLink')}
        assert lane_ids <= connected, (incoming.get('id'), lane_ids, connected)
        for connection in connections:
            connector = tree.find(f'./road[@id="{connection.get("connectingRoad")}"]')
            actual = {l.get('id') for l in connector.findall('.//lane[@type="driving"]')}
            assert all(link.get('to') in actual for link in connection.findall('laneLink'))
    xmlschema.XMLSchema(ROOT / 'xsd/OpenDRIVE_1.5M.xsd').validate(output)
