"""Install bounded ScenarioRunner support for requested parking-lane changes.

Run under the endpoint lock, with no active scenario. Saves original sources
and hashes; no CARLA restart, actor repositioning or collision intervention.
"""
import argparse
import hashlib
import json
from pathlib import Path


def patched_helper(source):
    old = 'lane_changes=1, step_distance=2):'
    new = 'lane_changes=1, step_distance=2, allowed_lane_types=None):'
    assert source.count(old) == 1
    source = source.replace(old, new)
    old = 'if not side_wp or side_wp.lane_type != carla.LaneType.Driving:'
    new = ('if not side_wp or side_wp.lane_type not in (allowed_lane_types or (carla.LaneType.Driving,)):\n'
           '            return None, None\n'
           '        if (side_wp.lane_type == carla.LaneType.Parking or next_wp.lane_type == carla.LaneType.Parking) and side_wp.lane_id * next_wp.lane_id <= 0:')
    assert source.count(old) == 1
    return source.replace(old, new)


def patched_atomics(source):
    start = source.index('class ChangeActorLateralMotion(')
    end = source.index('class ChangeActorLaneOffset(', start)
    part = source[start:end]
    old = 'get_waypoint(CarlaDataProvider.get_location(self._actor))'
    new = 'get_waypoint(CarlaDataProvider.get_location(self._actor), lane_type=carla.LaneType.Driving | carla.LaneType.Parking)'
    assert part.count(old) == 1
    part = part.replace(old, new)
    old = 'get_waypoint(self._actor.get_location())'
    new = 'get_waypoint(self._actor.get_location(), lane_type=carla.LaneType.Driving | carla.LaneType.Parking)'
    assert part.count(old) == 1
    part = part.replace(old, new)
    old = 'check=False, lane_changes=self._lane_changes)'
    new = ('check=False, lane_changes=self._lane_changes,\n'
           '            allowed_lane_types=(carla.LaneType.Driving, carla.LaneType.Parking))')
    assert part.count(old) == 1
    part = part.replace(old, new)
    # The first projection can already be in the destination lane after a
    # coarse tick; retain a defined distance origin in that case.
    part = part.replace('# calculate plan with scenario_helper function',
                        'self._pos_before_lane_change = position_actor.transform.location\n\n'
                        '        # calculate plan with scenario_helper function')
    return source[:start] + part + source[end:]


def install(root, evidence):
    evidence.mkdir(parents=True, exist_ok=True)
    files = {'tools/scenario_helper.py': patched_helper,
             'scenariomanager/scenarioatomics/atomic_behaviors.py': patched_atomics}
    prepared = []
    for name, transform in files.items():
        path = root / 'scenario_runner/srunner' / name
        original = path.read_text()
        changed = transform(original)
        compile(changed, str(path), 'exec')
        prepared.append((path, original, changed))
    hashes = {}
    for path, original, changed in prepared:
        (evidence / (path.name + '.before')).write_text(original)
        path.write_text(changed)
        (evidence / (path.name + '.after')).write_text(changed)
        hashes[str(path.relative_to(root))] = {
            'before_sha256': hashlib.sha256(original.encode()).hexdigest(),
            'after_sha256': hashlib.sha256(changed.encode()).hexdigest()}
    (evidence / 'deployment.json').write_text(json.dumps(hashes, indent=2))
    print(json.dumps(hashes, indent=2))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, required=True)
    ap.add_argument('--evidence', type=Path, required=True)
    args = ap.parse_args()
    install(args.root, args.evidence)
