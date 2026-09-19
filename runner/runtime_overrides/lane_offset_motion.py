"""Continuous sinusoidal lane-offset targets for the stock vehicle controller."""
import math
import operator

import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ChangeActorLaneOffset
from srunner.scenariomanager.timer import GameTime


def sinusoidal_offset(start, target, acceleration, elapsed):
    if not all(math.isfinite(v) for v in (start, target, acceleration, elapsed)) or acceleration <= 0:
        raise ValueError('lane-offset motion needs finite values and positive acceleration')
    duration = math.pi * math.sqrt(abs(target-start)/(2*acceleration))
    fraction = min(1.0, max(0.0, elapsed/duration)) if duration else 1.0
    return start + (target-start)*(.5-.5*math.cos(math.pi*fraction))


class SinusoidalLaneOffset(ChangeActorLaneOffset):
    """Ramp a lane-relative target and hold it; NPC physics remain controller-driven.

    maxLateralAcc bounds the target ramp, not a guarantee about the physical
    vehicle response. Measured dynamics remain part of trajectory validation.
    """
    def __init__(self, actor, offset, max_lateral_acceleration, continuous=True,
                 use_carla_coordinates=False, name='SinusoidalLaneOffset'):
        if not continuous:
            raise ValueError('sinusoidal lane offset currently requires continuous=true')
        location = actor.get_location()
        waypoint = CarlaDataProvider.get_map().get_waypoint(location)
        if waypoint is None:
            raise ValueError('lane-offset actor has no initial driving lane')
        target = float(offset)
        if not use_carla_coordinates:
            target *= 1 if waypoint.lane_id > 0 else -1
        self._acceleration = float(max_lateral_acceleration)
        sinusoidal_offset(0, target, self._acceleration, 0)
        super(SinusoidalLaneOffset, self).__init__(actor, target, continuous=True, name=name)

    def _controller(self):
        return operator.attrgetter('ActorsWithController')(py_trees.blackboard.Blackboard())[self._actor.id]

    def initialise(self):
        location = self._actor.get_location()
        waypoint = self._map.get_waypoint(location)
        right = waypoint.transform.get_right_vector()
        delta = location-waypoint.transform.location
        self._initial_offset = delta.x*right.x+delta.y*right.y
        super(SinusoidalLaneOffset, self).initialise()
        self._controller().update_offset(self._initial_offset)

    def update(self):
        status = super(SinusoidalLaneOffset, self).update()
        if status == py_trees.common.Status.RUNNING:
            value = sinusoidal_offset(self._initial_offset, self._offset, self._acceleration,
                                      GameTime.get_time()-self._start_time)
            self._controller().update_offset(value)
        return status
