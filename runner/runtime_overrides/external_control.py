#!/usr/bin/env python

# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides an example controller for actors, that use an external
software for longitudinal and lateral control command calculation.
Examples for external controls are: Autoware, CARLA manual_control, etc.

This module is not intended for modification.
"""

import carla
import math
import random
import sys
import os
import xml.etree.ElementTree as ET

# Add PCLA directory to path
pcla_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'PCLA'))
sys.path.insert(0, pcla_path)

from PCLA import PCLA, location_to_waypoint, route_maker

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.actorcontrols.basic_control import BasicControl


class ExternalControl(BasicControl):

    """
    Actor control class for actors, with externally implemented longitudinal and
    lateral controlers (e.g. Autoware).

    Args:
        actor (carla.Actor): Actor that should be controlled by the agent.
    """

    def __init__(self, actor, args=None):
        super(ExternalControl, self).__init__(actor)
        self._pcla_agent = None
        self._overhead_camera = None  # Initialize overhead camera variable
        self._controller_args = args or {}
        self._agent_name = self._resolve_agent_name()
        self._debug_life_time = 120.0
        self._ignore_traffic_lights = True
        self._last_overridden_light_id = None

        if actor:
            self._setup_pcla(actor)

    def _resolve_agent_name(self):
        """
        Resolve the PCLA agent name from controller properties.
        """
        configured_agent = (
            self._controller_args.get("pcla_agent")
            or self._controller_args.get("agent")
            or self._controller_args.get("agent_name")
        )
        if configured_agent:
            return str(configured_agent).strip()

        return "if_if"

    def _setup_pcla(self, actor):
        """
        Initializes the PCLA agent.
        """
        world = CarlaDataProvider.get_world()
        client = CarlaDataProvider.get_client()

        carla_map = world.get_map()
        actor_transform = actor.get_transform()
        start_wp = carla_map.get_waypoint(
            actor_transform.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        start_location = start_wp.transform.location

        configured_route_file = self._resolve_configured_route_file()
        configured_route_points = self._read_route_waypoints_from_xml(configured_route_file)
        configured_end = configured_route_points[-1] if configured_route_points else None

        if configured_end is not None:
            end_location = configured_end
            branch_decision = 'configured route file'
            route_file = configured_route_file
        else:
            end_location, branch_decision = self._select_route_endpoint(start_wp)
            waypoints = location_to_waypoint(client, start_location, end_location)
            route_file = "temp_route_for_external_control.xml"
            route_maker(waypoints, route_file)

        print(f"Hero road waypoint: pos=({start_location.x:.2f}, {start_location.y:.2f}), "
              f"yaw={start_wp.transform.rotation.yaw:.1f}°")
        print(f"Route endpoint selection: {branch_decision}")
        print(f"Endpoint used for routing: ({end_location.x:.2f}, {end_location.y:.2f})")
        if configured_route_file:
            print(f"Configured route file: {configured_route_file}")
        if configured_end is not None:
            print(f"Configured route endpoint: ({configured_end.x:.2f}, {configured_end.y:.2f})")

        if configured_route_file:
            debug_dir = os.path.dirname(configured_route_file)
            debug_name = f"{actor.attributes.get('role_name', 'actor')}_pcla_route_debug.jsonl"
            os.environ["PCLA_ROUTE_DEBUG"] = "1"
            os.environ["PCLA_ROUTE_DEBUG_FILE"] = os.path.join(debug_dir, debug_name)
            print(f"PCLA route debug file: {os.environ['PCLA_ROUTE_DEBUG_FILE']}")

        self._pcla_agent = PCLA(agent=self._agent_name, vehicle=actor, route=route_file, client=client)
        if os.environ.get("C2X_GNSS_COORDINATE_COMPAT") == "1":
            if self._agent_name != "if_if":
                raise ValueError("GNSS coordinate compatibility has only been validated for InterFuser")
            from srunner.scenariomanager.actorcontrols.gnss_compat import install_gnss_compat
            install_gnss_compat(self._pcla_agent.agent_instance, world, configured_route_points)
        print(f"PCLA Agent '{self._agent_name}' setup complete for ExternalControl.")
        print(f"Route: Start({start_location.x:.2f}, {start_location.y:.2f}) -> End({end_location.x:.2f}, {end_location.y:.2f})")

        self._debug_scene_layout(
            actor=actor,
            world=world,
            start_wp=start_wp,
            computed_end=end_location,
            configured_end=configured_end,
            configured_route_points=configured_route_points
        )

        self._setup_overhead_camera(actor, world)

    def _resolve_configured_route_file(self):
        """
        Resolve the route file declared in the OpenSCENARIO controller properties.
        """
        route_file = self._controller_args.get("route_file")
        if not route_file:
            return None

        candidates = [route_file]
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
        candidates.append(os.path.join(os.getcwd(), route_file))
        candidates.append(os.path.join(repo_root, route_file))

        for candidate in candidates:
            candidate = os.path.abspath(candidate)
            if os.path.exists(candidate):
                return candidate

        print(f"Warning: Configured route file not found: {route_file}")
        return None

    def _read_route_waypoints_from_xml(self, xml_file_path):
        """
        Read all waypoint locations from a leaderboard route XML.
        """
        if not xml_file_path:
            return []

        try:
            tree = ET.parse(xml_file_path)
            root = tree.getroot()
            waypoints = []
            for waypoint in root.findall('.//waypoint'):
                waypoints.append(carla.Location(
                    x=float(waypoint.get('x', 0.0)),
                    y=float(waypoint.get('y', 0.0)),
                    z=float(waypoint.get('z', 0.5))
                ))
            return waypoints
        except Exception as exc:
            print(f"Error reading route waypoints from {xml_file_path}: {exc}")
            return []

    def _normalize_yaw_delta(self, source_yaw, target_yaw):
        """
        Compute a signed smallest-angle difference in degrees.
        """
        return (target_yaw - source_yaw + 180.0) % 360.0 - 180.0

    def _advance_along_branch(self, waypoint, step_distance=8.0, steps=30):
        """
        Continue along one branch to obtain a route endpoint beyond the junction.
        """
        current = waypoint
        for _ in range(steps):
            next_candidates = current.next(step_distance)
            if not next_candidates:
                break
            current = next_candidates[0]
        return current.transform.location

    def _select_route_endpoint(self, start_wp):
        """
        Select a route endpoint that prefers the first turning branch over straight ahead.
        """
        preview_wp = start_wp
        step_distance = 8.0
        for preview_step in range(8):
            next_candidates = preview_wp.next(step_distance)
            if not next_candidates:
                break
            if len(next_candidates) > 1:
                current_yaw = preview_wp.transform.rotation.yaw
                ranked_candidates = []
                for candidate in next_candidates:
                    candidate_yaw = candidate.transform.rotation.yaw
                    delta_yaw = self._normalize_yaw_delta(current_yaw, candidate_yaw)
                    lane_continuity = 1 if candidate.lane_id == preview_wp.lane_id else 0
                    ranked_candidates.append((lane_continuity, -abs(delta_yaw), delta_yaw, candidate))
                ranked_candidates.sort(reverse=True)
                _, _, chosen_delta, chosen_wp = ranked_candidates[0]
                branch_side = 'lane-continuation' if abs(chosen_delta) <= 5.0 else 'right-turn/branch' if chosen_delta > 5.0 else 'left-turn/branch'
                end_location = self._advance_along_branch(chosen_wp)
                decision = (
                    f"branch step={preview_step}, chose road={chosen_wp.road_id}, lane={chosen_wp.lane_id}, "
                    f"delta_yaw={chosen_delta:.1f}°, decision={branch_side}"
                )
                return end_location, decision
            preview_wp = next_candidates[0]

        fallback_candidates = start_wp.next(300.0)
        if fallback_candidates:
            fallback_location = fallback_candidates[0].transform.location
            return fallback_location, 'no branch found, fallback=first lane continuation 300m'

        yaw_rad = math.radians(start_wp.transform.rotation.yaw)
        fallback_location = carla.Location(
            x=start_wp.transform.location.x + 300.0 * math.cos(yaw_rad),
            y=start_wp.transform.location.y + 300.0 * math.sin(yaw_rad),
            z=start_wp.transform.location.z
        )
        return fallback_location, 'no lane continuation found, fallback=manual forward projection'

    def _override_ego_traffic_light(self):
        """
        Force the traffic light affecting the ego vehicle to green.
        """
        if not self._ignore_traffic_lights or not self._actor or not self._actor.is_alive:
            return

        try:
            traffic_light = self._actor.get_traffic_light()
            if traffic_light is None:
                return

            if traffic_light.get_state() != carla.TrafficLightState.Green:
                traffic_light.set_state(carla.TrafficLightState.Green)
                traffic_light.set_green_time(999.0)
                traffic_light.freeze(True)
                if self._last_overridden_light_id != traffic_light.id:
                    print(f"Ignoring traffic light for ego: id={traffic_light.id}, forced GREEN")
                    self._last_overridden_light_id = traffic_light.id
        except Exception:
            pass

    def _lane_summary(self, carla_map, location):
        """
        Return a compact lane summary for a world location.
        """
        waypoint = carla_map.get_waypoint(
            location,
            project_to_road=False,
            lane_type=carla.LaneType.Any
        )
        if waypoint is None:
            return "no waypoint"

        return (
            f"road={waypoint.road_id}, section={waypoint.section_id}, lane={waypoint.lane_id}, "
            f"type={waypoint.lane_type}, junction={waypoint.is_junction}, "
            f"yaw={waypoint.transform.rotation.yaw:.1f}"
        )

    def _relative_position(self, ego_transform, other_location):
        """
        Project another location into the ego-vehicle local frame.
        """
        delta = other_location - ego_transform.location
        yaw_rad = math.radians(ego_transform.rotation.yaw)
        forward = carla.Vector3D(x=math.cos(yaw_rad), y=math.sin(yaw_rad), z=0.0)
        right = carla.Vector3D(x=math.cos(yaw_rad + math.pi / 2.0), y=math.sin(yaw_rad + math.pi / 2.0), z=0.0)

        longitudinal = delta.x * forward.x + delta.y * forward.y
        lateral = delta.x * right.x + delta.y * right.y
        distance = math.sqrt(delta.x ** 2 + delta.y ** 2 + delta.z ** 2)
        return longitudinal, lateral, distance

    def _classify_relative_pose(self, longitudinal, lateral):
        """
        Convert local-frame offsets into a readable spatial label.
        """
        front_back = "ahead" if longitudinal >= 0.0 else "behind"
        left_right = "right" if lateral >= 0.0 else "left"

        if abs(longitudinal) < 2.0:
            front_back = "alongside"
        if abs(lateral) < 1.5:
            left_right = "centered"

        if left_right == "centered":
            return front_back
        return f"{front_back}-{left_right}"

    def _draw_marker(self, world, location, text, color, life_time=None):
        """
        Draw a labeled point in the CARLA world.
        """
        debug = world.debug
        marker_location = carla.Location(location.x, location.y, location.z + 0.5)
        debug.draw_point(marker_location, size=0.15, color=color, life_time=life_time or self._debug_life_time)
        debug.draw_string(
            marker_location + carla.Location(z=0.6),
            text,
            draw_shadow=False,
            color=color,
            life_time=life_time or self._debug_life_time,
            persistent_lines=False
        )

    def _draw_transform_axes(self, world, transform, label, color):
        """
        Draw actor position and heading arrow.
        """
        start = transform.location + carla.Location(z=0.5)
        yaw_rad = math.radians(transform.rotation.yaw)
        end = start + carla.Location(x=2.5 * math.cos(yaw_rad), y=2.5 * math.sin(yaw_rad), z=0.0)
        world.debug.draw_arrow(start, end, thickness=0.08, arrow_size=0.2, color=color, life_time=self._debug_life_time)
        self._draw_marker(world, transform.location, label, color)

    def _draw_route(self, world, route_points, color, label_prefix):
        """
        Draw route points and connecting line segments.
        """
        if not route_points:
            return

        previous = None
        for index, location in enumerate(route_points):
            elevated = location + carla.Location(z=0.25)
            world.debug.draw_point(elevated, size=0.08, color=color, life_time=self._debug_life_time)
            if previous is not None:
                world.debug.draw_line(previous, elevated, thickness=0.05, color=color, life_time=self._debug_life_time)
            if index in (0, len(route_points) - 1):
                self._draw_marker(world, location, f"{label_prefix}{index}", color)
            previous = elevated

    def _collect_lane_preview(self, waypoint, step_distance=8.0, max_depth=6):
        """
        Follow the current lane forward and record branch points near intersections.
        """
        path = [waypoint]
        branches = []
        current = waypoint

        for _ in range(max_depth):
            next_candidates = current.next(step_distance)
            if not next_candidates:
                break

            if len(next_candidates) > 1:
                branches.append(next_candidates)

            current = next_candidates[0]
            path.append(current)

        return path, branches

    def _debug_scene_layout(self, actor, world, start_wp, computed_end, configured_end, configured_route_points):
        """
        Print and draw the ego lane, nearby actors, configured route and lane branches.
        """
        if os.environ.get("PCLA_DRAW_DEBUG", "0") != "1":
            return
        carla_map = world.get_map()
        ego_transform = actor.get_transform()
        ego_location = ego_transform.location

        print("=== Scene Debug Summary ===")
        print(f"Ego actor: id={actor.id}, type={actor.type_id}")
        print(f"Ego world position: ({ego_location.x:.2f}, {ego_location.y:.2f}, {ego_location.z:.2f}), yaw={ego_transform.rotation.yaw:.1f}°")
        print(f"Ego lane summary: {self._lane_summary(carla_map, ego_location)}")
        if configured_end is not None:
            print(
                "Route mismatch check: "
                f"configured_end=({configured_end.x:.2f}, {configured_end.y:.2f}) vs "
                f"computed_end=({computed_end.x:.2f}, {computed_end.y:.2f})"
            )

        self._draw_transform_axes(world, ego_transform, "ego", carla.Color(0, 128, 255))
        self._draw_marker(world, computed_end, "ego_route_end", carla.Color(0, 255, 255))
        if configured_end is not None:
            self._draw_marker(world, configured_end, "ego_route_end_cfg", carla.Color(0, 255, 255))
            self._draw_route(world, configured_route_points, carla.Color(0, 255, 255), "ego_route_")

        lane_preview, lane_branches = self._collect_lane_preview(start_wp)
        self._draw_route(
            world,
            [waypoint.transform.location for waypoint in lane_preview],
            carla.Color(255, 255, 0),
            "lane_preview_"
        )
        for branch_index, branch_candidates in enumerate(lane_branches):
            print(f"Lane branch {branch_index}: {len(branch_candidates)} candidates")
            for candidate_index, candidate_wp in enumerate(branch_candidates):
                candidate_location = candidate_wp.transform.location
                candidate_text = (
                    f"branch {branch_index}.{candidate_index}: "
                    f"road={candidate_wp.road_id}, lane={candidate_wp.lane_id}, "
                    f"yaw={candidate_wp.transform.rotation.yaw:.1f}"
                )
                print(candidate_text)
                self._draw_marker(world, candidate_location, f"b{branch_index}.{candidate_index}", carla.Color(255, 0, 255))

        nearby_descriptions = []
        for other_actor in world.get_actors():
            if other_actor.id == actor.id:
                continue
            if not isinstance(other_actor, (carla.Vehicle, carla.Walker)):
                continue
            if not other_actor.is_alive:
                continue

            other_location = other_actor.get_location()
            longitudinal, lateral, distance = self._relative_position(ego_transform, other_location)
            if distance > 80.0:
                continue

            relative_label = self._classify_relative_pose(longitudinal, lateral)
            lane_summary = self._lane_summary(carla_map, other_location)
            role_name = other_actor.attributes.get("role_name", "")
            display_name = role_name or other_actor.type_id

            nearby_descriptions.append((distance, (
                f"{display_name}: dist={distance:.1f}m, relative={relative_label}, "
                f"longitudinal={longitudinal:.1f}m, lateral={lateral:.1f}m, {lane_summary}"
            )))

            color = carla.Color(0, 255, 0) if isinstance(other_actor, carla.Vehicle) else carla.Color(255, 64, 64)
            self._draw_transform_axes(
                world,
                other_actor.get_transform(),
                f"{display_name} {relative_label}",
                color
            )

        if not nearby_descriptions:
            print("Nearby actors: none within 80m")
        else:
            print("Nearby actors:")
            for _, description in sorted(nearby_descriptions, key=lambda item: item[0]):
                print(f"  {description}")
        print("===========================")

    def _read_destination_from_route_xml(self, xml_file_path):
        """
        Read destination point from temp_route.xml file.
        The file should contain only one waypoint which will be used as the destination.
        
        Args:
            xml_file_path (str): Path to the XML file containing the destination point
            
        Returns:
            carla.Location: Destination location, or None if file cannot be read
        """
        try:
            if not os.path.exists(xml_file_path):
                print(f"Warning: Route file {xml_file_path} not found")
                return None

            tree = ET.parse(xml_file_path)
            root = tree.getroot()

            waypoint = root.find('.//waypoint')
            if waypoint is None:
                print(f"Warning: No waypoint found in {xml_file_path}")
                return None

            x = float(waypoint.get('x', 0))
            y = float(waypoint.get('y', 0))
            z = float(waypoint.get('z', 0.5))

            destination = carla.Location(x=x, y=y, z=z)
            print(f"Successfully read destination from {xml_file_path}: ({x:.2f}, {y:.2f}, {z:.2f})")

            return destination

        except Exception as e:
            print(f"Error reading destination from {xml_file_path}: {e}")
            return None

    def _setup_overhead_camera(self, actor, world):
        """
        Setup an overhead camera for observation
        """
        try:
            bp_library = world.get_blueprint_library()
            camera_bp = bp_library.find('sensor.camera.rgb')

            camera_bp.set_attribute('image_size_x', '800')
            camera_bp.set_attribute('image_size_y', '600')
            camera_bp.set_attribute('fov', '90')

            camera_transform = carla.Transform(
                carla.Location(x=0, y=0, z=50),
                carla.Rotation(pitch=-90, yaw=0, roll=0)
            )

            self._overhead_camera = world.spawn_actor(camera_bp, camera_transform, attach_to=actor)

            self._setup_spectator_view(actor, world)

            def save_camera_image(image):
                image.save_to_disk(f'/tmp/overhead_view_{image.frame}.png')

            # self._overhead_camera.listen(save_camera_image)

            print("Overhead observation camera setup complete")

        except Exception as e:
            print(f"Failed to setup overhead camera: {e}")
            self._overhead_camera = None

    def _setup_spectator_view(self, actor, world):
        """
        Set the spectator (observer window) to follow the actor from above
        """
        try:
            spectator = world.get_spectator()
            actor_location = actor.get_location()

            spectator_transform = carla.Transform(
                carla.Location(
                    x=actor_location.x,
                    y=actor_location.y,
                    z=actor_location.z + 50
                ),
                carla.Rotation(pitch=-90, yaw=0, roll=0)
            )

            spectator.set_transform(spectator_transform)

            print(f"Spectator view set to overhead position: {spectator_transform.location}")

        except Exception as e:
            print(f"Failed to setup spectator view: {e}")

    def reset(self):
        """
        Reset the controller
        """
        if self._pcla_agent:
            self._pcla_agent.cleanup()
            self._pcla_agent = None

        if self._overhead_camera and self._overhead_camera.is_alive:
            self._overhead_camera.stop()
            self._overhead_camera.destroy()
            self._overhead_camera = None

        if self._actor and self._actor.is_alive:
            self._actor = None

    def run_step(self):
        """
        The control loop and setting the actor controls is implemented externally.
        """
        if not self._pcla_agent:
            return

        self._override_ego_traffic_light()

        if not self._pcla_agent.sensors_ready():
            self._actor.apply_control(carla.VehicleControl())
            if self._init_speed:
                velocity = self._actor.get_velocity()
                current_speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2)
                if abs(self._target_speed - current_speed) > 3.0:
                    yaw = math.radians(self._actor.get_transform().rotation.yaw)
                    self._actor.set_target_velocity(carla.Vector3D(
                        math.cos(yaw) * self._target_speed,
                        math.sin(yaw) * self._target_speed,
                        0.0,
                    ))
            self._update_spectator_view()
            return

        control = self._pcla_agent.get_action()
        if control is None:
            control = carla.VehicleControl()

        self._actor.apply_control(control)

        self._update_spectator_view()

    def _update_spectator_view(self):
        """
        Update spectator view to continuously follow the actor from above
        """
        try:
            if self._actor and self._actor.is_alive:
                world = CarlaDataProvider.get_world()
                spectator = world.get_spectator()
                actor_location = self._actor.get_location()

                spectator_transform = carla.Transform(
                    carla.Location(
                        x=actor_location.x,
                        y=actor_location.y,
                        z=actor_location.z + 50
                    ),
                    carla.Rotation(pitch=-90, yaw=0, roll=0)
                )

                spectator.set_transform(spectator_transform)

        except Exception as e:
            pass
