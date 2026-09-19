"""Time OpenSCENARIO standstill from the first stationary observation."""
import math

import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import EPSILON, StandStill
from srunner.scenariomanager.timer import GameTime


class ObservedStandStill(StandStill):
    """Require a complete observed stationary interval, not time since movement.

    The native condition starts its timer at the last moving tick. That can
    credit one unobserved tick of standstill at the moving-to-stopped boundary.
    Keep its speed threshold; only change when the interval begins.
    """
    def initialise(self):
        super(ObservedStandStill, self).initialise()
        self._stationary_since = None
        self._reported = False

    def update(self):
        now = GameTime.get_time()
        speed = CarlaDataProvider.get_velocity(self._actor)
        if speed is None or not math.isfinite(speed) or abs(speed) > EPSILON:
            self._stationary_since = None
            return py_trees.common.Status.RUNNING
        if self._stationary_since is None:
            self._stationary_since = now
        if now-self._stationary_since >= self._duration:
            if not self._reported:
                print('C2X_OBSERVED_STANDSTILL actor_id={} condition={} start={:.9f} '
                      'time={:.9f} duration={} speed_threshold={}'.format(
                          self._actor.id, self.name, self._stationary_since, now,
                          self._duration, EPSILON), flush=True)
                self._reported = True
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
