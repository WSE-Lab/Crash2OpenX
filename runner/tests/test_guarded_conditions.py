import importlib.util
from pathlib import Path
from xml.etree import ElementTree as ET
import unittest

import py_trees

try:
    from guarded_conditions import guarded_condition_group
except ModuleNotFoundError:
    spec=importlib.util.spec_from_file_location('guarded',Path(__file__).resolve().parents[1]/'runtime_overrides/guarded_conditions.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    guarded_condition_group=module.guarded_condition_group


class GuardedConditionsTest(unittest.TestCase):
    def test_spawn_standstill_cannot_satisfy_a_later_restart(self):
        xml=ET.fromstring('''<ConditionGroup>
          <Condition name="previous"><ByValueCondition><StoryboardElementStateCondition state="completeState"/></ByValueCondition></Condition>
          <Condition name="separated"><ByEntityCondition/></Condition>
          <Condition name="stopped"><ByEntityCondition/></Condition>
        </ConditionGroup>''')
        state={'previous':False,'separated':True,'stopped':True}; ticks=[]
        class Probe(py_trees.behaviour.Behaviour):
            def update(self):
                ticks.append(self.name)
                return py_trees.common.Status.SUCCESS if state[self.name] else py_trees.common.Status.RUNNING
        group=guarded_condition_group(xml,lambda c:Probe(c.get('name')))
        list(group.tick())
        self.assertEqual(ticks,['previous'])
        state.update(previous=True,stopped=False);ticks.clear();list(group.tick())
        self.assertEqual(group.status,py_trees.common.Status.RUNNING)
        # Predicates must be simultaneously true, not remembered at different times.
        state.update(separated=False,stopped=True);list(group.tick())
        self.assertEqual(group.status,py_trees.common.Status.RUNNING)
        state.update(separated=True);list(group.tick())
        self.assertEqual(group.status,py_trees.common.Status.SUCCESS)

    def test_non_sequential_trigger_uses_existing_path(self):
        xml=ET.fromstring('<ConditionGroup><Condition name="start"><ByValueCondition><SimulationTimeCondition/></ByValueCondition></Condition></ConditionGroup>')
        self.assertIsNone(guarded_condition_group(xml,lambda _:None))


if __name__ == '__main__':
    unittest.main()
