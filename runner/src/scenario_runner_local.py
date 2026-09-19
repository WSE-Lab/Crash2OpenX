#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Welcome to CARLA scenario_runner

This is the main script to be executed when running a scenario.
It loads the scenario configuration, loads the scenario and manager,
and finally triggers the scenario execution.
"""

from __future__ import print_function

import glob
import traceback
import argparse
from argparse import RawTextHelpFormatter
from datetime import datetime
from distutils.version import LooseVersion
import importlib
import inspect
import os
import signal
import sys
import time
import json
import pkg_resources
import xml.etree.ElementTree as ET

try:
    from carla_compat import ensure_carla_importable
except ImportError:
    from src.carla_compat import ensure_carla_importable

carla = ensure_carla_importable()

from openscenario_configuration import OpenScenarioConfiguration
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from scenario_manager_local import ScenarioManager
from open_scenario import OpenScenario


# Version of scenario_runner
VERSION = '0.9.16'


class ScenarioRunner(object):

    """
    This is the core scenario runner module. It is responsible for
    running (and repeating) a single scenario or a list of scenarios.

    Usage:
    scenario_runner = ScenarioRunner(args)
    scenario_runner.run()
    del scenario_runner
    """

    ego_vehicles = []

    # Tunable parameters
    client_timeout = 10.0  # in seconds
    wait_for_world = 20.0  # in seconds
    frame_rate = 20.0      # in Hz

    # CARLA world and scenario handlers
    world = None
    manager = None

    finished = False

    additional_scenario_module = None

    agent_instance = None
    module_agent = None


    def __init__(self, args,xml_tree):
        """
        Setup CARLA client and world
        Setup ScenarioManager
        """
        self._args = args
        self.xml_tree=xml_tree
        
        # 添加数据收集器支持
        self.data_collector = None

        if args.timeout:
            self.client_timeout = float(args.timeout)

        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        self.client = carla.Client(args.host, int(args.port))
        self.client.set_timeout(self.client_timeout)
        dist = pkg_resources.get_distribution("carla")
        if LooseVersion(dist.version) < LooseVersion('0.9.16'):
            raise ImportError("CARLA version 0.9.16 or newer required. CARLA version found: {}".format(dist))


        # Create the ScenarioManager
        self.manager = ScenarioManager(False, self._args.sync, self._args.timeout)
        self.manager.set_disable_traffic_lights(getattr(self._args, 'disable_traffic_lights', False))
        self.manager.real_time_factor = max(float(getattr(self._args, 'real_time_factor', 1.0)), 0.01)
        self.manager.post_run_hold = max(float(getattr(self._args, 'post_run_hold', 0.0)), 0.0)

        # Create signal handler for SIGINT
        self._shutdown_requested = False
        if sys.platform != 'win32':
            signal.signal(signal.SIGHUP, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._start_wall_time = datetime.now()
        
    def set_data_collector(self, data_collector):
        """
        设置数据收集器
        
        Args:
            data_collector: DataCollector实例
        """
        self.data_collector = data_collector
        # 将数据收集器也传递给manager
        if self.manager:
            self.manager.set_data_collector(data_collector)

    def destroy(self):
        """
        Cleanup and delete actors, ScenarioManager and CARLA world
        """

        self._cleanup()
        if self.manager is not None:
            del self.manager
        if self.world is not None:
            del self.world
        if self.client is not None:
            del self.client

    def _signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt
        """
        self._shutdown_requested = True
        if self.manager:
            self.manager.stop_scenario()
            self._finalize_data_collection(termination_reason=f"signal:{signum}")
            self._cleanup()
            if not self.manager.get_running_status():
                raise RuntimeError("Timeout occurred during scenario execution")

    def _finalize_data_collection(self, termination_reason=None):
        if not self.data_collector:
            return

        scenario_tree_status = str(self.manager.scenario_tree.status) if self.manager and self.manager.scenario_tree else None
        criteria = self.manager.scenario.get_criteria() if self.manager and self.manager.scenario else []

        resolved_reason = termination_reason
        if resolved_reason is None:
            collector_reason = self.data_collector.get_requested_termination_reason()
            if collector_reason:
                resolved_reason = collector_reason
        if resolved_reason is None:
            resolved_reason = "scenario_completed"
            if self._shutdown_requested:
                resolved_reason = "shutdown_requested"
            elif scenario_tree_status and "FAILURE" in scenario_tree_status:
                resolved_reason = "scenario_failure"
            elif scenario_tree_status and "SUCCESS" not in scenario_tree_status:
                resolved_reason = f"scenario_finished:{scenario_tree_status}"

        self.data_collector.finalize(
            termination_reason=resolved_reason,
            scenario_tree_status=scenario_tree_status,
            criteria=criteria,
        )

    def _cleanup(self):
        """
        Remove and destroy all actors
        """
        if self.finished:
            return

        self.finished = True
        self._finalize_data_collection()

        # Simulation still running and in synchronous mode?
        if self.world is not None and self._args.sync:
            try:
                # Reset to asynchronous mode
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                if getattr(self._args, 'no_rendering', False):
                    settings.no_rendering_mode = False
                self.world.apply_settings(settings)
                self.client.get_trafficmanager(int(self._args.trafficManagerPort)).set_synchronous_mode(False)
            except RuntimeError:
                sys.exit(-1)
        elif self.world is not None and getattr(self._args, 'no_rendering', False):
            try:
                settings = self.world.get_settings()
                settings.no_rendering_mode = False
                self.world.apply_settings(settings)
            except RuntimeError:
                sys.exit(-1)

        self.manager.cleanup()

        CarlaDataProvider.cleanup()

        for i, _ in enumerate(self.ego_vehicles):
            if self.ego_vehicles[i]:
                if  self.ego_vehicles[i] is not None and self.ego_vehicles[i].is_alive:
                    print("Destroying ego vehicle {}".format(self.ego_vehicles[i].id))
                    self.ego_vehicles[i].destroy()
                self.ego_vehicles[i] = None
        self.ego_vehicles = []

        if self.agent_instance:
            self.agent_instance.destroy()
            self.agent_instance = None

    def _prepare_ego_vehicles(self, ego_vehicles):
        """
        Spawn or update the ego vehicles
        """

        for vehicle in ego_vehicles:
            self.ego_vehicles.append(CarlaDataProvider.request_new_actor(vehicle.model,
                                                                            vehicle.transform,
                                                                            vehicle.rolename,
                                                                            random_location=vehicle.random_location,
                                                                            color=vehicle.color,
                                                                            actor_category=vehicle.category))
        # sync state
        if CarlaDataProvider.is_sync_mode():
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def _analyze_scenario(self, config):
        """
        Provide feedback about success/failure of a scenario
        """

        # Create the filename
        current_time = str(datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        junit_filename = None
        json_filename = None
        config_name = config.name
        if self._args.output != '':
            config_name = os.path.join(self._args.outputDir, config_name)

        if self._args.junit:
            junit_filename = config_name + current_time + ".xml"
        if self._args.json:
            json_filename = config_name + current_time + ".json"
        filename = None
        if self._args.file:
            filename = config_name + current_time + ".txt"

        if not self.manager.analyze_scenario(self._args.output, filename, junit_filename, json_filename):
            print("All scenario tests were passed successfully!")
        else:
            print("Not all scenario tests were successful")
            if not (self._args.output or filename or junit_filename):
                print("Please run with --output for further information")

    def _record_criteria(self, criteria, name):
        """
        Filter the JSON serializable attributes of the criterias and
        dumps them into a file. This will be used by the metrics manager,
        in case the user wants specific information about the criterias.
        """
        file_name = name[:-4] + ".json"

        # Filter the attributes that aren't JSON serializable
        with open('temp.json', 'w', encoding='utf-8') as fp:

            criteria_dict = {}
            for criterion in criteria:

                criterion_dict = criterion.__dict__
                criteria_dict[criterion.name] = {}

                for key in criterion_dict:
                    if key != "name":
                        try:
                            key_dict = {key: criterion_dict[key]}
                            json.dump(key_dict, fp, sort_keys=False, indent=4)
                            criteria_dict[criterion.name].update(key_dict)
                        except TypeError:
                            pass

        os.remove('temp.json')

        # Save the criteria dictionary into a .json file
        with open(file_name, 'w', encoding='utf-8') as fp:
            json.dump(criteria_dict, fp, sort_keys=False, indent=4)

    @staticmethod
    def _is_opendrive_file(town):
        return bool(town) and str(town).lower().endswith(".xodr")

    @staticmethod
    def _read_opendrive_file(opendrive_path):
        with open(opendrive_path, 'r', encoding='utf-8') as od_file:
            data = od_file.read()

        index = data.find('<OpenDRIVE')
        if index == -1:
            raise RuntimeError("The provided OpenDRIVE file does not contain an <OpenDRIVE> root: {}".format(opendrive_path))
        return ScenarioRunner._prepare_opendrive_for_carla(data[index:], opendrive_path)

    @staticmethod
    def _prepare_opendrive_for_carla(opendrive_data, opendrive_path):
        """
        CARLA's OpenDRIVE importer can segfault on malformed-but-XML-valid maps,
        especially roads with an empty planView. Patch the common generated-map
        case in memory, and fail early for maps that still have no geometry.
        """
        try:
            root = ET.fromstring(opendrive_data)
        except ET.ParseError as exc:
            raise RuntimeError("The provided OpenDRIVE file is not valid XML: {} ({})".format(opendrive_path, exc))

        header = root.find('header')
        if header is None:
            header = ET.Element('header')
            root.insert(0, header)
        if header.find('geoReference') is None:
            geo_ref = ET.Element('geoReference')
            geo_ref.text = '+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs'
            header.append(geo_ref)

        repaired_planviews = 0
        roads = root.findall('road')
        for index, road in enumerate(roads):
            plan_view = road.find('planView')
            if plan_view is None:
                plan_view = ET.Element('planView')
                road.insert(1, plan_view)

            if plan_view.find('geometry') is not None:
                continue

            length = float(road.attrib.get('length', '0') or 0)
            if length <= 0.0:
                raise RuntimeError("OpenDRIVE road {} has no positive length in {}".format(
                    road.attrib.get('id', '<unknown>'), opendrive_path))

            geometry = ET.SubElement(plan_view, 'geometry')
            geometry.set('s', '0.0')
            if index % 3 == 0:
                geometry.set('x', str((index // 3) * 12.0))
                geometry.set('y', str(-length / 2.0))
                geometry.set('hdg', '1.5707963267948966')
            elif index % 3 == 1:
                geometry.set('x', str(-length / 2.0))
                geometry.set('y', '0.0')
                geometry.set('hdg', '0.0')
            else:
                geometry.set('x', str(12.0 + (index // 3) * 12.0))
                geometry.set('y', str(-length / 2.0))
                geometry.set('hdg', '1.5707963267948966')
            geometry.set('length', str(length))
            ET.SubElement(geometry, 'line')
            repaired_planviews += 1

        missing_geometry = [
            road.attrib.get('id', '<unknown>')
            for road in roads
            if road.find('planView/geometry') is None
        ]
        if missing_geometry:
            raise RuntimeError("OpenDRIVE roads have no planView geometry in {}: {}".format(
                opendrive_path, ', '.join(missing_geometry)))

        if repaired_planviews:
            print("Patched {} empty OpenDRIVE planView elements in memory for CARLA import".format(repaired_planviews))

        return ET.tostring(root, encoding='unicode')

    def _generate_opendrive_world(self, opendrive_path):
        data = self._read_opendrive_file(opendrive_path)
        print("Loading custom OpenDRIVE map: {}".format(opendrive_path))
        return self.client.generate_opendrive_world(
            str(data),
            carla.OpendriveGenerationParameters(
                vertex_distance=2.0,
                wall_height=0.0,
                additional_width=0.6,
                smooth_junctions=True,
                enable_mesh_visibility=True,
            )
        )

    def _load_and_wait_for_world(self, town, ego_vehicles=None):
        """
        Load a new CARLA world and provide data to CarlaDataProvider
        """

        if self._is_opendrive_file(town):
            if not os.path.isabs(town):
                town = os.path.abspath(town)
            if not os.path.exists(town):
                raise RuntimeError("The provided OpenDRIVE map does not exist: {}".format(town))
            if self._args.reloadWorld:
                self.world = self._generate_opendrive_world(town)
            else:
                self.world = self.client.get_world()
                current_map_name = self.world.get_map().name.split('/')[-1]
                if current_map_name != "OpenDriveMap":
                    self.world = self._generate_opendrive_world(town)
        elif self._args.reloadWorld:
            self.world = self.client.load_world(town)
        else:
            # if the world should not be reloaded, wait at least until all ego vehicles are ready
            ego_vehicle_found = False
            if self._args.waitForEgo:
                while not ego_vehicle_found and not self._shutdown_requested:
                    vehicles = self.client.get_world().get_actors().filter('vehicle.*')
                    for ego_vehicle in ego_vehicles:
                        ego_vehicle_found = False
                        for vehicle in vehicles:
                            if vehicle.attributes['role_name'] == ego_vehicle.rolename:
                                ego_vehicle_found = True
                                break
                        if not ego_vehicle_found:
                            print("Not all ego vehicles ready. Waiting ... ")
                            time.sleep(1)
                            break

        self.world = self.client.get_world()

        if os.environ.get("C2X_NATIVE_ROAD_MARKINGS") == "1":
            from road_markings import draw_opendrive_markings
            draw_opendrive_markings(self.world, brightness=float(
                os.environ.get('C2X_ROAD_MARKING_BRIGHTNESS', str(4.0/245.0))))

        if self._args.sync:
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 1.0 / self.frame_rate
            settings.no_rendering_mode = bool(getattr(self._args, 'no_rendering', False))
            self.world.apply_settings(settings)
        elif getattr(self._args, 'no_rendering', False):
            settings = self.world.get_settings()
            settings.no_rendering_mode = True
            self.world.apply_settings(settings)

        CarlaDataProvider.set_client(self.client)
        CarlaDataProvider.set_world(self.world)

        # Wait for the world to be ready
        if CarlaDataProvider.is_sync_mode():
            self.world.tick()
        else:
            self.world.wait_for_tick()

        map_name = CarlaDataProvider.get_map().name.split('/')[-1]
        expected_maps = ("OpenDriveMap",) if self._is_opendrive_file(town) else (town, "OpenDriveMap")
        if map_name not in expected_maps:
            print("The CARLA server uses the wrong map: {}".format(map_name))
            print("This scenario requires to use map: {}".format(town))
            return False

        if getattr(self._args, 'disable_traffic_lights', False):
            self._disable_all_traffic_lights()

        return True

    def _disable_all_traffic_lights(self):
        """
        Force all traffic lights to green and freeze them.
        """
        traffic_lights = self.world.get_actors().filter('traffic.traffic_light*')
        disabled_count = 0
        for traffic_light in traffic_lights:
            try:
                traffic_light.set_state(carla.TrafficLightState.Green)
                traffic_light.set_green_time(999999.0)
                traffic_light.freeze(True)
                disabled_count += 1
            except RuntimeError:
                pass

        print(f"Global traffic lights disabled: {disabled_count} lights forced to GREEN")

    def _load_and_run_scenario(self, config):
        """
        Load and run the scenario given by config
        """
        result = False
        if not self._load_and_wait_for_world(config.town, config.ego_vehicles):
            self._cleanup()
            return False


        CarlaDataProvider.set_traffic_manager_port(int(self._args.trafficManagerPort))
        tm = self.client.get_trafficmanager(int(self._args.trafficManagerPort))
        tm.set_random_device_seed(int(self._args.trafficManagerSeed))
        if self._args.sync:
            tm.set_synchronous_mode(True)

        # Prepare scenario
        print("Preparing scenario: " + config.name)
        try:
            self._prepare_ego_vehicles(config.ego_vehicles)
            scenario = OpenScenario(world=self.world,
                                        ego_vehicles=self.ego_vehicles,
                                        config=config,
                                        timeout=100000)
        except Exception as exception:                  # pylint: disable=broad-except
            print("The scenario cannot be loaded")
            traceback.print_exc()
            print(exception)
            self._cleanup()
            return False

        try:
            if self._args.record:
                recorder_name = "{}/{}/{}.log".format(
                    os.getenv('SCENARIO_RUNNER_ROOT', "./"), self._args.record, config.name)
                self.client.start_recorder(recorder_name, True)

            # Load scenario and run it
            self.manager.load_scenario(scenario, self.agent_instance)
            
            # 初始化数据收集器（如果存在）
            if self.data_collector:
                collector_actor = self.ego_vehicles[0] if self.ego_vehicles else None
                if getattr(self._args, 'pcla_sut', False):
                    sut_actor_name = getattr(self._args, 'sut_actor', None)
                    sut_actor = CarlaDataProvider.get_actor_by_name(sut_actor_name) if sut_actor_name else None
                    if sut_actor:
                        collector_actor = sut_actor
                    else:
                        print(f"Warning: 未找到 PCLA SUT actor {sut_actor_name}，数据采集回退到 ego_vehicles[0]")
                self.data_collector.initialize(self.world, collector_actor)
                print("数据收集器已初始化")
            
            self.manager.run_scenario()

            # Provide outputs if required
            # self._analyze_scenario(config)


            # Remove all actors, stop the recorder and save all criterias (if needed)
            scenario.remove_all_actors()
            if self._args.record:
                self.client.stop_recorder()
                self._record_criteria(self.manager.scenario.get_criteria(), recorder_name)

            self._finalize_data_collection()

            result = True

        except Exception as e:              # pylint: disable=broad-except
            traceback.print_exc()
            print(e)
            self._finalize_data_collection(termination_reason=f"exception:{type(e).__name__}")
            result = False

        self._cleanup()
        return result

    def _run_openscenario(self):
        """
        Run a scenario based on OpenSCENARIO
        """


        openscenario_params = {}
        # todo
        
        # if self._args.openscenarioparams is not None:
        #     for entry in self._args.openscenarioparams.split(','):
        #         [key, val] = [m.strip() for m in entry.split(':')]
        #         openscenario_params[key] = val
        config = OpenScenarioConfiguration(self.xml_tree, self.client, openscenario_params, self._args.scenario)

        result = self._load_and_run_scenario(config)
        self._cleanup()
        return result


    def run(self):
        """
        Run all scenarios according to provided commandline args
        """
        result = True
        result = self._run_openscenario()
        return result


# def eval(args,xml_tree):


#     # pylint: enable=line-too-long
#     arguments = args

#     scenario_runner = None
#     result = True
#     try:
#         scenario_runner = ScenarioRunner(arguments,xml_tree)
#         result = scenario_runner.run()
#         return  scenario_runner.manager.hero_metrics
#     except Exception:   # pylint: disable=broad-except
#         traceback.print_exc()

#     finally:
#         if scenario_runner is not None:
#             scenario_runner.destroy()
#             del scenario_runner
#     return not result


# if __name__ == "__main__":
#     sys.exit(main())
