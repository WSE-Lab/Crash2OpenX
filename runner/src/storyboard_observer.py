"""Read ScenarioRunner story transitions without changing controls or triggers."""
import math
from pathlib import Path
from xml.etree import ElementTree as ET


class StoryboardObserver:
    def __init__(self, scenario_path):
        self.elements = []
        self.seen = set()
        if scenario_path and Path(scenario_path).is_file():
            root = ET.parse(scenario_path).getroot()
            self.elements = [(kind.upper(), node.get('name'))
                             for kind in ('Event', 'Action')
                             for node in root.findall('.//' + kind) if node.get('name')]

    def sample(self, blackboard, scenario_time, frame, simulation_time):
        records = []
        for kind, name in self.elements:
            for state in ('START', 'END', 'CANCEL'):
                key = f'({kind}){name}-{state}'
                try:
                    value = blackboard.get(key)
                except KeyError:
                    continue
                if (not isinstance(value, (int, float)) or isinstance(value, bool)
                        or not math.isfinite(value) or not 0 <= value <= scenario_time):
                    continue
                identity = (key, value)
                if identity in self.seen:
                    continue
                self.seen.add(identity)
                records.append({'event_type': 'storyboard_transition', 'frame': frame,
                                'simulation_time': simulation_time,
                                'payload': {'source': 'scenario_runner_blackboard',
                                            'element_type': kind, 'element_name': name,
                                            'transition': state, 'scenario_time': value}})
        return records
