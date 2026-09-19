"""Install the same narrow live-window hook in an existing ScenarioRunner."""


def patched_scenario(source):
    if 'from ads_conditions import live_ads_condition_group' in source:
        return source
    old = '        for condition_group in node.iter("ConditionGroup"):\n'
    addition = '''            from ads_conditions import live_ads_condition_group
            live = live_ads_condition_group(condition_group, lambda condition:
                OpenScenarioParser.convert_condition_to_atomic(condition, self.other_actors + self.ego_vehicles))
            if live is not None:
                parallel_condition_groups.add_child(live)
                continue
'''
    if source.count(old) != 1:
        raise ValueError('Expected exactly one ScenarioRunner condition group loop')
    changed = source.replace(old, old+addition)
    compile(changed, 'open_scenario.py', 'exec')
    return changed


def main():
    import argparse
    import hashlib
    import json
    import shutil
    from pathlib import Path
    parser = argparse.ArgumentParser(description='Install live ADS-relative triggers into a local CARLA runtime copy')
    parser.add_argument('--runtime-root', type=Path, required=True)
    args = parser.parse_args()
    root = args.runtime_root.resolve()
    source_root = Path(__file__).resolve().parents[1]/'runner/src'
    target = root/'src/open_scenario.py'
    source = target.read_text()
    changed = patched_scenario(source)
    for name in ('ads_conditions.py', 'ads_geometry.py'):
        destination = root/'src'/name
        if (source_root/name).resolve() != destination.resolve():
            shutil.copy2(source_root/name, destination)
    if changed != source:
        backup = target.with_suffix('.py.before_ads_relative')
        if not backup.exists():
            backup.write_text(source)
        temporary = target.with_suffix('.py.ads_tmp')
        temporary.write_text(changed)
        temporary.replace(target)
    print(json.dumps({'runtime': str(root), 'open_scenario_sha256': hashlib.sha256(target.read_bytes()).hexdigest()}))


if __name__ == '__main__':
    main()
