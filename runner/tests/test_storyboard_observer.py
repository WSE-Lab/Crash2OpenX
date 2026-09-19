import importlib.util
from pathlib import Path
import unittest
import tempfile

spec=importlib.util.spec_from_file_location('observer',Path(__file__).resolve().parents[1]/'src/storyboard_observer.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class StoryboardObserverTest(unittest.TestCase):
    def test_start_end_cancel_are_observations_and_zero_time_is_not_missing(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'scene.xosc'
            path.write_text('<OpenSCENARIO><Event name="rear_step_1"><Action name="drive"/></Event></OpenSCENARIO>')
            observer=module.StoryboardObserver(path)
            blackboard={'(EVENT)rear_step_1-START':0, '(ACTION)drive-START':0,
                        '(EVENT)rear_step_1-END':1, '(ACTION)drive-CANCEL':2}
            first=observer.sample(blackboard,.5,10,20)
            self.assertEqual([e['payload']['transition'] for e in first],['START','START'])
            self.assertEqual(observer.sample(blackboard,.5,11,20.05),[])
            later=observer.sample(blackboard,2,12,21.5)
            self.assertEqual([e['payload']['transition'] for e in later],['END','CANCEL'])
            self.assertEqual(later[0]['payload']['scenario_time'],1)
            self.assertEqual(later[0]['simulation_time'],21.5)

    def test_missing_scenario_has_no_transitions(self):
        observer=module.StoryboardObserver(None)
        self.assertEqual(observer.sample({},0,1,0),[])


if __name__ == '__main__':
    unittest.main()
