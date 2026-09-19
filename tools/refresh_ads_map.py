#!/usr/bin/env python3
"""Recompile a derived test's source RoadSeed and validate native CARLA geometry."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import sys
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.build_road_seed_opendrive import build
from tools.carla_remote import CarlaRemoteClient
from tools.osc_blocks import build_xosc
from tools.replay_scene_tools import validate_xosc
from tools.run_fresh_framework_batch import SerializedCarlaClient, digest, write


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--config', type=Path, default=ROOT/'outputs/ads_stress_20260919/runtime_config.json')
    args = parser.parse_args()
    folder = args.folder.resolve()
    manifest = json.loads((folder/'manifest.json').read_text())
    if (folder/'execution.json').exists():
        raise ValueError('Cannot replace a map after execution')
    staging = folder/'native_map_rebuild'
    staging.mkdir(exist_ok=False)
    road = json.loads((folder/'source_road_seed.json').read_text())
    scene = json.loads((folder/'scene_seed.json').read_text())['scene']
    build(road, staging/'map.xodr', staging/'map.html', ROOT/'xsd/OpenDRIVE_1.5M.xsd')
    os.environ.update(json.loads(args.config.read_text()))
    client = SerializedCarlaClient(CarlaRemoteClient(), ROOT/'outputs/.carla_rpc.lock')
    name = 'ads_expansion_'+manifest['case_id'][:3]+'_native'
    extracted = client.extract_roadgraph(staging/'map.xodr', name=name)
    if not extracted.selfcheck.get('geometry_consistency_pass'):
        raise ValueError('Native geometry check failed: '+str(extracted.selfcheck))
    build_xosc(scene, str(staging/'map.xodr'), staging/'scenario.xosc', name=name, road_seed=road)
    tree = ET.parse(staging/'scenario.xosc')
    tree.find('.//RoadNetwork/LogicFile').set('filepath', 'map.xodr')
    prior = ET.parse(folder/'scenario.xosc')
    guard = prior.find("./Storyboard/StopTrigger/ConditionGroup/Condition[@name='experiment_duration']/..")
    for stop in [tree.find('./Storyboard/StopTrigger'), *tree.findall('./Storyboard/Story/Act/StopTrigger')]:
        stop.append(copy.deepcopy(guard))
    tree.write(staging/'scenario.xosc', encoding='utf-8', xml_declaration=True)
    valid, error = validate_xosc(staging/'scenario.xosc', ROOT/'xsd/OpenSCENARIO.xsd')
    if not valid:
        raise ValueError(error)
    for name in ['map.xodr', 'scenario.xosc', 'map.html']:
        shutil.copy2(folder/name, staging/('previous_'+name))
        shutil.copy2(staging/name, folder/name)
    manifest.update(map_preserved=False, map_sha256=digest(folder/'map.xodr'),
                    scenario_sha256=digest(folder/'scenario.xosc'), roadgraph_cache_dir=str(extracted.local_dir),
                    road_recompiled_from_source_seed=True, native_geometry_check=extracted.selfcheck)
    write(folder/'manifest.json', manifest)
    print(json.dumps({'case':manifest['case_id'], 'native_geometry':True, 'map_sha256':manifest['map_sha256']}))


if __name__ == '__main__':
    main()
