import unittest
from types import SimpleNamespace
from unittest.mock import patch

import py_trees
import standstill_condition as condition


class ObservedStandstillTest(unittest.TestCase):
    def setUp(self):
        self.actor = SimpleNamespace(id=7)
        self.atomic = condition.ObservedStandStill(self.actor, 'target stopped', .5)
        self.atomic.initialise()

    def sample(self, time, speed):
        with patch.object(condition.GameTime, 'get_time', return_value=time), \
             patch.object(condition.CarlaDataProvider, 'get_velocity', return_value=speed):
            return self.atomic.update()

    def test_full_window_starts_at_first_stationary_observation(self):
        self.sample(1., .6)
        self.sample(1.05, 0.)
        self.assertEqual(self.sample(1.50, 0.), py_trees.common.Status.RUNNING)
        self.assertEqual(self.sample(1.56, 0.), py_trees.common.Status.SUCCESS)

    def test_new_motion_resets_the_entire_window(self):
        self.sample(1., 0.)
        self.sample(1.3, .01)
        self.sample(1.35, 0.)
        self.assertEqual(self.sample(1.8, 0.), py_trees.common.Status.RUNNING)
        self.assertEqual(self.sample(1.86, 0.), py_trees.common.Status.SUCCESS)

    def test_missing_and_nonfinite_samples_do_not_count_as_stationary(self):
        for missing in (None, float('nan'), float('inf')):
            self.atomic.initialise()
            self.sample(1., 0.)
            self.sample(1.4, missing)
            self.sample(1.45, 0.)
            self.assertEqual(self.sample(1.8, 0.), py_trees.common.Status.RUNNING)

    def test_reinitialisation_does_not_reuse_previous_window(self):
        self.sample(1., 0.)
        self.sample(1.6, 0.)
        self.atomic.initialise()
        self.assertEqual(self.sample(5., 0.), py_trees.common.Status.RUNNING)


if __name__ == '__main__':
    unittest.main()
