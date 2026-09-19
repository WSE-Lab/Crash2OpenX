#!/usr/bin/env python

# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides an example control for vehicles
"""

import math
import json

import carla
from agents.navigation.basic_agent import LocalPlanner
from agents.navigation.local_planner import RoadOption

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.actorcontrols.basic_control import BasicControl


def terminal_road_speed(plan, location, target_speed, current_speed, front_extent):
    """Brake before the physical end of a finite generated road.

    A normal route endpoint may be followed by another OSC action. Only limit
    speed when the last planned waypoint has no road continuation. Follow the
    planner's existing path; never choose a new branch or move the actor.
    """
    if not plan or plan[-1][0].next(2.0):
        return target_speed, None
    remaining = 0.0
    previous = location
    for waypoint, _ in plan:
        point = waypoint.transform.location
        remaining += previous.distance(point)
        previous = point
    # Keep the front bumper on the mesh and allow for controller/tick latency.
    clearance = max(0.0, remaining - front_extent - 2.0 - 0.3 * current_speed)
    # 2 m/s² is a conservative deceleration for the stock planner's brake cap.
    return min(target_speed, math.sqrt(4.0 * clearance)), remaining


class NpcVehicleControl(BasicControl):

    """
    Controller class for vehicles derived from BasicControl.

    The controller makes use of the LocalPlanner implemented in CARLA.

    Args:
        actor (carla.Actor): Vehicle actor that should be controlled.
    """

    _args = {'K_P': 1.0, 'K_D': 0.01, 'K_I': 0.0, 'dt': 0.05}

    def __init__(self, actor, args=None):
        super(NpcVehicleControl, self).__init__(actor)
        self._generated_lane_route = json.loads((args or {}).get('C2XGeneratedLaneRoute', 'null'))
        self._generated_lane_route_installed = False

        self._local_planner = LocalPlanner(  # pylint: disable=undefined-variable
            self._actor, opt_dict={
                'target_speed': self._target_speed * 3.6,
                'max_brake': float((args or {}).get('C2XMaxBrake', 0.3)),
                'lateral_control_dict': self._args})
        if 'C2XInitialLaneOffset' in (args or {}):
            self._offset = float(args['C2XInitialLaneOffset'])
            self._update_offset()

        if self._waypoints:
            self._update_plan()

        self._brake_lights_active = False
        self._terminal_braking_logged = False

    def _update_plan(self):
        """
        Update the plan (waypoint list) of the LocalPlanner
        """
        if self._generated_lane_route is not None and not self._generated_lane_route_installed:
            from srunner.scenariomanager.actorcontrols.generated_lane_route import resolve_generated_lane_route
            points = resolve_generated_lane_route(CarlaDataProvider.get_map(), self._generated_lane_route)
            self._local_planner.set_global_plan([(point, RoadOption.LANEFOLLOW) for point in points])
            self._generated_lane_route_installed = True
            print('C2X_GENERATED_LANE_ROUTE actor_id={} points={} source=compiled_CARLA_roadgraph'.format(
                self._actor.id, len(points)), flush=True)
            return
        plan = []
        for transform in self._waypoints:
            waypoint = CarlaDataProvider.get_map().get_waypoint(
                transform.location, project_to_road=True, lane_type=carla.LaneType.Any)
            plan.append((waypoint, RoadOption.LANEFOLLOW))
        self._local_planner.set_global_plan(plan)

    def _update_offset(self):
        """
        Update the plan (waypoint list) of the LocalPlanner
        """
        self._local_planner._vehicle_controller._lat_controller._offset = self._offset   # pylint: disable=protected-access

    def reset(self):
        """
        Reset the controller
        """
        if self._actor and self._actor.is_alive:
            if self._local_planner:
                self._local_planner.reset_vehicle()
                self._local_planner = None
            self._actor = None

    def run_step(self):
        """
        Execute on tick of the controller's control loop

        Note: Negative target speeds are not yet supported.
              Try using simple_vehicle_control or vehicle_longitudinal_control.

        If _waypoints are provided, the vehicle moves towards the next waypoint
        with the given _target_speed, until reaching the final waypoint. Upon reaching
        the final waypoint, _reached_goal is set to True.

        If _waypoints is empty, the vehicle moves in its current direction with
        the given _target_speed.

        If _init_speed is True, the control command is post-processed to ensure that
        the initial actor velocity is maintained independent of physics.
        """
        self._reached_goal = False

        if self._waypoints_updated:
            self._waypoints_updated = False
            self._update_plan()

        if self._offset_updated:
            self._offset_updated = False
            self._update_offset()

        target_speed = self._target_speed
        # If target speed is negavite, raise an exception
        if target_speed < 0:
            raise NotImplementedError("Negative target speeds are not yet supported")

        if not self._actor.is_alive:
            return
        velocity = self._actor.get_velocity()
        current_speed = math.hypot(velocity.x, velocity.y)
        box = self._actor.bounding_box
        target_speed, remaining = terminal_road_speed(
            self._local_planner.get_plan(), self._actor.get_location(),
            target_speed, current_speed, box.extent.x + abs(box.location.x))
        terminal_braking = target_speed < self._target_speed
        if terminal_braking and not self._terminal_braking_logged:
            print("NPC_TERMINAL_ROAD_BRAKING actor_id={} remaining_m={:.3f} speed_mps={:.3f}".format(
                self._actor.id, remaining, current_speed), flush=True)
            self._terminal_braking_logged = True
        self._local_planner.set_speed(target_speed * 3.6)
        control = self._local_planner.run_step(debug=False)

        # Check if the actor reached the end of the plan
        if self._local_planner.done():
            self._reached_goal = True

        self._actor.apply_control(control)

        current_speed = math.sqrt(self._actor.get_velocity().x**2 + self._actor.get_velocity().y**2)

        if self._init_speed and not terminal_braking and not self._reached_goal:

            # If _init_speed is set, and the PID controller is not yet up to the point to take over,
            # we manually set the vehicle to drive with the correct velocity
            if abs(target_speed - current_speed) > 3:
                yaw = self._actor.get_transform().rotation.yaw * (math.pi / 180)
                vx = math.cos(yaw) * target_speed
                vy = math.sin(yaw) * target_speed
                self._actor.set_target_velocity(carla.Vector3D(vx, vy, 0))

        # Change Brake light state
        if (current_speed > target_speed or target_speed < 0.2) and not self._brake_lights_active:
            light_state = self._actor.get_light_state()
            light_state |= carla.VehicleLightState.Brake
            self._actor.set_light_state(carla.VehicleLightState(light_state))
            self._brake_lights_active = True

        if self._brake_lights_active and current_speed < target_speed:
            self._brake_lights_active = False
            light_state = self._actor.get_light_state()
            light_state &= ~carla.VehicleLightState.Brake
            self._actor.set_light_state(carla.VehicleLightState(light_state))
