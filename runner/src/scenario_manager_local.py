#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the ScenarioManager implementation.
It must not be modified and is for reference only!
"""

from __future__ import print_function
import sys
import time

import py_trees

import carla

from srunner.autoagents.agent_wrapper import AgentWrapper
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.result_writer import ResultOutputProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog


class ScenarioManager(object):

    """
    Basic scenario manager class. This class holds all functionality
    required to start, and analyze a scenario.

    The user must not modify this class.

    To use the ScenarioManager:
    1. Create an object via manager = ScenarioManager()
    2. Load a scenario via manager.load_scenario()
    3. Trigger the execution of the scenario manager.run_scenario()
       This function is designed to explicitly control start and end of
       the scenario execution
    4. Trigger a result evaluation with manager.analyze_scenario()
    5. If needed, cleanup with manager.stop_scenario()
    """

    def __init__(self, debug_mode=False, sync_mode=False, timeout=2.0):
        """
        Setups up the parameters, which will be filled at load_scenario()

        """
        self.scenario = None
        self.scenario_tree = None
        self.ego_vehicles = None
        self.other_actors = None

        self._debug_mode = debug_mode
        self._agent = None
        self._sync_mode = sync_mode
        self._watchdog = None
        self._timeout = timeout

        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None

        
        # 添加数据收集器支持
        self.data_collector = None
        self.disable_traffic_lights = False
        self.follow_ego_camera = True
        self._spectator_location = None
        self.real_time_factor = 1.0
        self.post_run_hold = 0.0


    def _reset(self):
        """
        Reset all parameters
        """
        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        self._spectator_location = None
        GameTime.restart()

    def cleanup(self):
        """
        This function triggers a proper termination of a scenario
        """

        if self._watchdog is not None:
            self._watchdog.stop()
            self._watchdog = None

        if self.scenario is not None:
            self.scenario.terminate()

        if self._agent is not None:
            self._agent.cleanup()
            self._agent = None

        CarlaDataProvider.cleanup()

    def load_scenario(self, scenario, agent=None):
        """
        Load a new scenario
        """
        self._reset()
        self._agent = AgentWrapper(agent) if agent else None
        if self._agent is not None:
            self._sync_mode = True
        self.scenario = scenario
        self.scenario_tree = self.scenario.scenario_tree
        self.ego_vehicles = scenario.ego_vehicles
        self.other_actors = scenario.other_actors

        # To print the scenario tree uncomment the next line
        # py_trees.display.render_dot_tree(self.scenario_tree)

        if self._agent is not None:
            self._agent.setup_sensors(self.ego_vehicles[0], self._debug_mode)
            
    def set_data_collector(self, data_collector):
        """
        设置数据收集器
        
        Args:
            data_collector: DataCollector实例
        """
        self.data_collector = data_collector

    def set_disable_traffic_lights(self, disable_traffic_lights):
        """
        控制是否在每个 tick 全局强制绿灯。
        """
        self.disable_traffic_lights = disable_traffic_lights

    def _force_all_traffic_lights_green(self):
        """
        Force all traffic lights to green and freeze them.
        """
        world = CarlaDataProvider.get_world()
        if not world:
            return

        for traffic_light in world.get_actors().filter('traffic.traffic_light*'):
            try:
                traffic_light.set_state(carla.TrafficLightState.Green)
                traffic_light.set_green_time(999999.0)
                traffic_light.freeze(True)
            except RuntimeError:
                pass

    def run_scenario(self):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        print("ScenarioManager: Running scenario {}".format(self.scenario_tree.name))
        self.start_system_time = time.time()
        start_game_time = GameTime.get_time()

        self._watchdog = Watchdog(float(self._timeout))
        self._watchdog.start()
        self._running = True

        while self._running:
            timestamp = None
            world = CarlaDataProvider.get_world()
            if world:
                snapshot = world.get_snapshot()
                if snapshot:
                    timestamp = snapshot.timestamp
            if timestamp:
                self._tick_scenario(timestamp)

        self.end_system_time = time.time()
        end_game_time = GameTime.get_time()

        self.scenario_duration_system = self.end_system_time - \
            self.start_system_time
        self.scenario_duration_game = end_game_time - start_game_time
        print(
            "ScenarioManager: duration system={:.2f}s, simulation={:.2f}s, real_time_factor={:.2f}".format(
                self.scenario_duration_system,
                self.scenario_duration_game,
                self.scenario_duration_game / max(self.scenario_duration_system, 0.001)
            )
        )

        if self.scenario_tree.status == py_trees.common.Status.FAILURE:
            print("ScenarioManager: Terminated due to failure")

        if self.post_run_hold > 0.0:
            print("ScenarioManager: holding final scene for {:.1f}s before cleanup".format(self.post_run_hold))
            time.sleep(self.post_run_hold)

        self.cleanup()

    def _tick_scenario(self, timestamp):
        """
        Run next tick of scenario and the agent.
        If running synchornously, it also handles the ticking of the world.
        """

        tick_wall_start = time.time()

        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds

            self._watchdog.update()

            if self._debug_mode:
                print("\n--------- Tick ---------\n")

            # Update game time and actor information
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()

            if self.disable_traffic_lights:
                self._force_all_traffic_lights_green()

            if self._agent is not None:
                ego_action = self._agent()  # pylint: disable=not-callable

            if self._agent is not None:
                self.ego_vehicles[0].apply_control(ego_action)

            # Tick scenario
            self.scenario_tree.tick_once()
            self._update_spectator_camera()

            # 数据收集 - 在每次tick时收集数据
            if self.data_collector:
                self.data_collector.collect_frame_data()
                if self.data_collector.should_terminate():
                    self._running = False
            
            if self._debug_mode:
                print("\n")
                py_trees.display.print_ascii_tree(self.scenario_tree, show_status=True)
                sys.stdout.flush()

            if self.scenario_tree.status != py_trees.common.Status.RUNNING:
                self._running = False

        if self._sync_mode and self._running and self._watchdog.get_status():
            self._sleep_for_realtime_pacing(tick_wall_start)
            CarlaDataProvider.get_world().tick()

    def _sleep_for_realtime_pacing(self, tick_wall_start):
        """
        In synchronous mode, CARLA advances one fixed simulation step whenever
        world.tick() is called. Without pacing, the scenario runs as fast as the
        CPU can tick, so 60 simulated seconds can complete in a few wall seconds.
        """
        world = CarlaDataProvider.get_world()
        if world is None:
            return

        fixed_delta = world.get_settings().fixed_delta_seconds
        if not fixed_delta:
            return

        target_wall_delta = fixed_delta / max(self.real_time_factor, 0.01)
        elapsed_wall = time.time() - tick_wall_start
        sleep_time = target_wall_delta - elapsed_wall
        if sleep_time > 0.0:
            time.sleep(sleep_time)

    def _update_spectator_camera(self):
        """
        Keep the CARLA spectator camera high above the ego vehicle.
        This is a visualization camera, not a spawned sensor, so it does not
        affect the scenario actors or data collection.
        """
        if not self.follow_ego_camera or not self.ego_vehicles:
            return

        ego_vehicle = self.ego_vehicles[0]
        if ego_vehicle is None or not ego_vehicle.is_alive:
            return

        world = CarlaDataProvider.get_world()
        if world is None:
            return

        ego_transform = ego_vehicle.get_transform()
        camera_height = 60.0
        target_location = carla.Location(
            x=ego_transform.location.x,
            y=ego_transform.location.y,
            z=camera_height,
        )
        if self._spectator_location is None:
            self._spectator_location = target_location
        else:
            alpha = 0.25
            self._spectator_location = carla.Location(
                x=self._spectator_location.x + (target_location.x - self._spectator_location.x) * alpha,
                y=self._spectator_location.y + (target_location.y - self._spectator_location.y) * alpha,
                z=camera_height,
            )
        camera_rotation = carla.Rotation(
            pitch=-90.0,
            yaw=0.0,
            roll=0.0,
        )
        world.get_spectator().set_transform(carla.Transform(self._spectator_location, camera_rotation))

    def get_running_status(self):
        """
        returns:
           bool:  False if watchdog exception occured, True otherwise
        """
        return self._watchdog.get_status()

    def stop_scenario(self):
        """
        This function is used by the overall signal handler to terminate the scenario execution
        """
        self._running = False

    def analyze_scenario(self, stdout, filename, junit, json):
        """
        This function is intended to be called from outside and provide
        the final statistics about the scenario (human-readable, in form of a junit
        report, etc.)
        """

        failure = False
        timeout = False
        result = "SUCCESS"

        criteria = self.scenario.get_criteria()
        if len(criteria) == 0:
            print("Nothing to analyze, this scenario has no criteria")
            return True

        for criterion in criteria:
            if (not criterion.optional and
                    criterion.test_status != "SUCCESS" and
                    criterion.test_status != "ACCEPTABLE"):
                failure = True
                result = "FAILURE"
            elif criterion.test_status == "ACCEPTABLE":
                result = "ACCEPTABLE"

        if self.scenario.timeout_node.timeout and not failure:
            timeout = True
            result = "TIMEOUT"

        output = ResultOutputProvider(self, result, stdout, filename, junit, json)
        output.write()

        return failure or timeout
