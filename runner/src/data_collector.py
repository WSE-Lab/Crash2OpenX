import json
import math
import os
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import carla
import numpy as np
import shapely.geometry


class DataCollector:
    """Collect per-tick ego state, event logs, and episode-level aggregates."""

    TTC_THRESHOLD_SECONDS = 1.5
    BLOCKED_SPEED_THRESHOLD = 0.1
    BLOCKED_PROGRESS_EPSILON = 0.1
    BLOCKED_WARMUP_SECONDS = 20.0
    BLOCKED_TERMINATION_SECONDS = 20.0

    ROUTE_WINDOW_SIZE = 2
    ROUTE_SUCCESS_DISTANCE_THRESHOLD = 10.0
    ROUTE_SUCCESS_PERCENTAGE_THRESHOLD = 99.0

    OPPOSITE_ALLOWED_OUT_DISTANCE = 0.5
    OPPOSITE_MAX_VEHICLE_ANGLE = 120.0
    OPPOSITE_MAX_WAYPOINT_ANGLE = 150.0

    RED_LIGHT_DISTANCE = 15.0

    STOP_PROXIMITY_THRESHOLD = 4.0
    STOP_SPEED_THRESHOLD = 0.1
    STOP_WAYPOINT_STEP = 0.5

    def __init__(
        self,
        scenario_name: str = "unknown_scenario",
        output_dir: Optional[str] = None,
        scenario_path: Optional[str] = None,
        mid_model_path: Optional[str] = None,
        route_file_path: Optional[str] = None,
        frame_stride: int = 1,
        record_video: bool = False,
        video_fps: int = 10,
        video_frame_stride: int = 2,
    ):
        self.scenario_name = scenario_name
        self.scenario_path = scenario_path
        self.mid_model_path = mid_model_path
        self.route_file_path = route_file_path
        self.frame_stride = max(1, int(frame_stride))
        self.record_video = bool(record_video)
        self.video_fps = max(1, int(video_fps))
        self.video_frame_stride = max(1, int(video_frame_stride))

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_dir = output_dir or os.path.join(os.getcwd(), "eval", f"{scenario_name}_{timestamp}")
        self.output_dir = os.path.abspath(base_dir)
        self.paths = {
            "metadata": os.path.join(self.output_dir, "metadata.json"),
            "frame_states": os.path.join(self.output_dir, "frame_states.jsonl"),
            "sim_trace_raw": os.path.join(self.output_dir, "sim_trace_raw.jsonl"),
            "events": os.path.join(self.output_dir, "events.jsonl"),
            "summary": os.path.join(self.output_dir, "summary.json"),
            "sim_feedback": os.path.join(self.output_dir, "sim_feedback.json"),
            "path_plot": os.path.join(self.output_dir, "path_plot.png"),
            "contact_sheet": os.path.join(self.output_dir, "contact_sheet.jpg"),
            "keyframes_dir": os.path.join(self.output_dir, "keyframes"),
            "video": os.path.join(self.output_dir, "trajectory_video.mp4"),
        }

        self.world = None
        self.map = None
        self.hero_actor = None

        self.start_wall_time = None
        self.end_wall_time = None
        self._episode_start_sim_time = None
        self.fixed_delta_seconds = None
        self._last_sim_time = None
        self._last_local_accel = None
        self._last_route_completion = 0.0
        self._last_progress_time = None
        self._blocked_started_at = None
        self._last_ego_waypoint = None
        self._last_lane_id = None
        self._last_road_id = None
        self._wrong_lane_active = False
        self._offroad_active = False

        self._route_transforms: List[carla.Transform] = []
        self._route_accum_perc: List[float] = []
        self._route_index = 0
        self._route_target_location = None

        self._traffic_lights: List[Tuple[carla.Actor, carla.Location, List[carla.Waypoint]]] = []
        self._last_red_light_id = None

        self._stop_signs: List[carla.Actor] = []
        self._target_stop_sign = None
        self._stop_completed = False
        self._last_failed_stop = None

        self._collision_sensor = None
        self._lane_invasion_sensor = None

        self._frame_file = None
        self._trace_file = None
        self._event_file = None
        self._termination_requested = False
        self._requested_termination_reason = None
        self.expected_actors = self._parse_expected_actors()
        self._actor_name_by_id: Dict[int, str] = {}
        self._actor_category_by_name: Dict[str, str] = {
            actor["name"]: actor["category"] for actor in self.expected_actors
        }
        self._trace_records: List[Dict[str, Any]] = []
        self._event_records: List[Dict[str, Any]] = []
        self._offroad_detected = False
        self._teleport_detected = False
        self._previous_trace_state: Dict[str, Dict[str, Any]] = {}
        self._spawned_actor_names = set()
        self._synthetic_collision_pairs = set()
        self._static_contact_ignored = False

        self.summary = {
            "scenario_name": self.scenario_name,
            "episode_dir": self.output_dir,
            "total_ticks": 0,
            "stored_frame_rows": 0,
            "duration_seconds": 0.0,
            "final_route_completion": 0.0,
            "min_ttc": None,
            "min_distance": None,
            "max_long_jerk": None,
            "max_lat_jerk": None,
            "unsafe_exposure_time": 0.0,
            "off_road_time": 0.0,
            "opposite_lane_occupancy_time": 0.0,
            "blocked_time": 0.0,
            "collision_count": 0,
            "lane_invasion_count": 0,
            "red_light_violation_count": 0,
            "stop_sign_violation_count": 0,
            "termination_reason": None,
            "scenario_tree_status": None,
            "criteria": [],
            "video_path": None,
        }

    def initialize(self, world: carla.World, hero_actor: Optional[carla.Actor] = None):
        self.world = world
        self.map = world.get_map()
        self.hero_actor = hero_actor
        self.start_wall_time = time.time()
        self._episode_start_sim_time = self._get_snapshot_time()

        os.makedirs(self.output_dir, exist_ok=True)
        self._frame_file = open(self.paths["frame_states"], "w", encoding="utf-8")
        self._trace_file = open(self.paths["sim_trace_raw"], "w", encoding="utf-8")
        self._event_file = open(self.paths["events"], "w", encoding="utf-8")

        settings = world.get_settings()
        self.fixed_delta_seconds = settings.fixed_delta_seconds

        self._load_route()
        self._refresh_expected_actor_bindings()
        self._build_rule_monitors()
        self._setup_event_sensors()

        metadata = self._build_metadata()
        with open(self.paths["metadata"], "w", encoding="utf-8") as file_obj:
            json.dump(metadata, file_obj, indent=2, ensure_ascii=False)

        self._write_event(
            {
                "event_type": "episode_started",
                "frame": self._get_snapshot_frame(),
                "simulation_time": self._get_snapshot_time(),
                "payload": {"scenario_name": self.scenario_name},
            }
        )
        print(f"Data collector initialized: {self.output_dir}")

    def collect_frame_data(self):
        if not self.world or not self.hero_actor or not self.hero_actor.is_alive:
            return

        snapshot = self.world.get_snapshot()
        if snapshot is None:
            return

        sim_time = snapshot.timestamp.elapsed_seconds
        frame = snapshot.frame
        dt = self._resolve_delta_time(sim_time)

        ego_state = self._extract_ego_state(snapshot, sim_time, dt)
        actor_states = self._extract_expected_actor_states(snapshot, sim_time, dt)
        route_completion = self._compute_route_completion(ego_state["location_obj"])

        self._update_jerk(sim_time, frame, dt, ego_state)
        self._update_distance_metrics(sim_time, frame)
        self._check_synthetic_collisions(frame, sim_time)
        self._update_offroad_time(dt, ego_state["location_obj"])
        self._update_actor_feedback_flags(actor_states, dt)
        self._update_wrong_lane_time(dt, ego_state["location_obj"])
        self._update_blocked_time(
            sim_time,
            dt,
            route_completion,
            ego_state["speed"],
            ego_state["location_obj"],
        )
        self._check_red_light_violation(frame)
        self._check_stop_sign_violation(frame)

        self.summary["total_ticks"] += 1
        self.summary["final_route_completion"] = round(route_completion, 3)

        if self.summary["total_ticks"] % self.frame_stride == 0:
            self._write_frame_state(ego_state)
            self._write_trace_state(snapshot, sim_time, actor_states)
            self.summary["stored_frame_rows"] += 1

        self._last_sim_time = sim_time

    def finalize(
        self,
        termination_reason: Optional[str] = None,
        scenario_tree_status: Optional[str] = None,
        criteria: Optional[List[Any]] = None,
    ):
        if self.end_wall_time is not None:
            return

        self.end_wall_time = time.time()
        duration = 0.0
        if self.start_wall_time is not None:
            duration = self.end_wall_time - self.start_wall_time
        self.summary["duration_seconds"] = round(duration, 3)
        self.summary["termination_reason"] = termination_reason
        self.summary["scenario_tree_status"] = scenario_tree_status
        self.summary["criteria"] = self._serialize_criteria(criteria or [])

        if self.route_file_path and os.path.isfile(self.route_file_path):
            target_path = os.path.join(self.output_dir, os.path.basename(self.route_file_path))
            if os.path.abspath(target_path) != os.path.abspath(self.route_file_path):
                shutil.copy2(self.route_file_path, target_path)

        self._write_event(
            {
                "event_type": "episode_finished",
                "frame": self._get_snapshot_frame(),
                "simulation_time": self._get_snapshot_time(),
                "payload": {
                    "termination_reason": termination_reason,
                    "scenario_tree_status": scenario_tree_status,
                },
            }
        )

        self._write_sim_feedback()

        with open(self.paths["summary"], "w", encoding="utf-8") as file_obj:
            json.dump(self.summary, file_obj, indent=2, ensure_ascii=False)

        self._cleanup_sensors()
        if self._frame_file:
            self._frame_file.close()
            self._frame_file = None
        if self._trace_file:
            self._trace_file.close()
            self._trace_file = None
        if self._event_file:
            self._event_file.close()
            self._event_file = None

    def save_data(self, output_file: str = None):
        if output_file and os.path.abspath(output_file) != self.output_dir:
            print(f"Data output folder already fixed to: {self.output_dir}")
        if self.end_wall_time is None:
            self.finalize()
        print(f"Data artifacts saved under: {self.output_dir}")

    def get_summary(self) -> Dict[str, Any]:
        return dict(self.summary)

    def should_terminate(self) -> bool:
        return bool(self._termination_requested)

    def get_requested_termination_reason(self) -> Optional[str]:
        return self._requested_termination_reason

    def _build_metadata(self) -> Dict[str, Any]:
        weather = self.world.get_weather()
        hero_transform = self.hero_actor.get_transform() if self.hero_actor else None
        start_location = {
            "x": hero_transform.location.x if hero_transform else None,
            "y": hero_transform.location.y if hero_transform else None,
            "z": hero_transform.location.z if hero_transform else None,
        }
        return {
            "scenario_name": self.scenario_name,
            "scenario_path": self.scenario_path,
            "mid_model_path": self.mid_model_path,
            "route_file_path": self.route_file_path,
            "expected_actors": self.expected_actors,
            "town_name": self.map.name if self.map else None,
            "episode_dir": self.output_dir,
            "frame_stride": self.frame_stride,
            "record_video": self.record_video,
            "video_fps": self.video_fps,
            "video_frame_stride": self.video_frame_stride,
            "fixed_delta_seconds": self.fixed_delta_seconds,
            "start_wall_time": self.start_wall_time,
            "ego_vehicle_type": self.hero_actor.type_id if self.hero_actor else None,
            "ego_actor_id": self.hero_actor.id if self.hero_actor else None,
            "start_location": start_location,
            "weather": {
                "cloudiness": weather.cloudiness,
                "precipitation": weather.precipitation,
                "precipitation_deposits": weather.precipitation_deposits,
                "wind_intensity": weather.wind_intensity,
                "sun_azimuth_angle": weather.sun_azimuth_angle,
                "sun_altitude_angle": weather.sun_altitude_angle,
                "fog_density": weather.fog_density,
                "wetness": weather.wetness,
            },
        }

    def _extract_ego_state(self, snapshot, sim_time: float, dt: float) -> Dict[str, Any]:
        transform = self.hero_actor.get_transform()
        velocity = self.hero_actor.get_velocity()
        acceleration = self.hero_actor.get_acceleration()
        angular_velocity = self.hero_actor.get_angular_velocity()
        control = self.hero_actor.get_control()
        waypoint = self._resolve_waypoint(transform.location)
        traffic_light_state = self._get_traffic_light_state()

        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        local_accel = self._project_to_vehicle_frame(acceleration, transform.rotation.yaw)

        state = {
            "frame_id": snapshot.frame,
            "simulation_time": round(sim_time, 6),
            "wall_time": round(time.time(), 6),
            "delta_seconds": round(dt, 6),
            "x": round(transform.location.x, 6),
            "y": round(transform.location.y, 6),
            "z": round(transform.location.z, 6),
            "pitch": round(transform.rotation.pitch, 6),
            "yaw": round(transform.rotation.yaw, 6),
            "roll": round(transform.rotation.roll, 6),
            "vx": round(velocity.x, 6),
            "vy": round(velocity.y, 6),
            "vz": round(velocity.z, 6),
            "speed": round(speed, 6),
            "ax": round(acceleration.x, 6),
            "ay": round(acceleration.y, 6),
            "az": round(acceleration.z, 6),
            "longitudinal_accel": round(local_accel[0], 6),
            "lateral_accel": round(local_accel[1], 6),
            "angular_velocity_x": round(angular_velocity.x, 6),
            "angular_velocity_y": round(angular_velocity.y, 6),
            "angular_velocity_z": round(angular_velocity.z, 6),
            "throttle": round(control.throttle, 6),
            "brake": round(control.brake, 6),
            "steer": round(control.steer, 6),
            "hand_brake": bool(control.hand_brake),
            "reverse": bool(control.reverse),
            "road_id": waypoint.road_id if waypoint else None,
            "lane_id": waypoint.lane_id if waypoint else None,
            "lane_type": str(waypoint.lane_type) if waypoint else None,
            "is_junction": bool(waypoint.is_junction) if waypoint else None,
            "speed_limit": round(self.hero_actor.get_speed_limit(), 3),
            "traffic_light_state": traffic_light_state,
            "location_obj": transform.location,
        }
        return state

    def _parse_expected_actors(self) -> List[Dict[str, str]]:
        if not self.scenario_path or not os.path.isfile(self.scenario_path):
            return []

        actors = []
        try:
            root = ET.parse(self.scenario_path).getroot()
        except ET.ParseError:
            return actors

        for scenario_object in root.findall(".//Entities/ScenarioObject"):
            name = scenario_object.attrib.get("name")
            if not name:
                continue
            category = "unknown"
            model = None
            vehicle = scenario_object.find("Vehicle")
            pedestrian = scenario_object.find("Pedestrian")
            misc = scenario_object.find("MiscObject")
            if vehicle is not None:
                category = "vehicle"
                model = vehicle.attrib.get("name")
            elif pedestrian is not None:
                category = "pedestrian"
                model = pedestrian.attrib.get("model") or pedestrian.attrib.get("name")
            elif misc is not None:
                category = "misc"
                model = misc.attrib.get("name")
            actors.append({"name": name, "category": category, "model": model})
        return actors

    def _refresh_expected_actor_bindings(self):
        if not self.world:
            return
        expected_names = {actor["name"] for actor in self.expected_actors}
        for actor in self.world.get_actors():
            role_name = actor.attributes.get("role_name") if hasattr(actor, "attributes") else None
            if role_name in expected_names:
                self._actor_name_by_id[actor.id] = role_name
                self._spawned_actor_names.add(role_name)

    def _extract_expected_actor_states(self, snapshot, sim_time: float, dt: float) -> Dict[str, Dict[str, Any]]:
        self._refresh_expected_actor_bindings()
        states = {}
        expected_names = {actor["name"] for actor in self.expected_actors}
        for actor in self.world.get_actors():
            if not actor.is_alive:
                continue
            role_name = actor.attributes.get("role_name") if hasattr(actor, "attributes") else None
            if role_name not in expected_names:
                continue
            states[role_name] = self._extract_actor_state(actor, snapshot, sim_time, dt)
        return states

    def _extract_actor_state(self, actor: carla.Actor, snapshot, sim_time: float, dt: float) -> Dict[str, Any]:
        transform = actor.get_transform()
        velocity = actor.get_velocity()
        speed = self._get_speed(velocity)
        state = {
            "frame_id": snapshot.frame,
            "simulation_time": round(sim_time, 6),
            "actor_id": actor.id,
            "type_id": actor.type_id,
            "x": round(transform.location.x, 6),
            "y": round(transform.location.y, 6),
            "z": round(transform.location.z, 6),
            "yaw": round(transform.rotation.yaw, 6),
            "pitch": round(transform.rotation.pitch, 6),
            "roll": round(transform.rotation.roll, 6),
            "vx": round(velocity.x, 6),
            "vy": round(velocity.y, 6),
            "vz": round(velocity.z, 6),
            "speed_mps": round(speed, 6),
        }
        waypoint = self.map.get_waypoint(
            transform.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        ) if self.map else None
        if waypoint:
            wp_loc = waypoint.transform.location
            state["nearest_driving_waypoint_distance_m"] = round(transform.location.distance(wp_loc), 6)
            state["road_id"] = waypoint.road_id
            state["lane_id"] = waypoint.lane_id
        else:
            state["nearest_driving_waypoint_distance_m"] = None
            state["road_id"] = None
            state["lane_id"] = None
        return state

    def _write_frame_state(self, ego_state: Dict[str, Any]):
        record = dict(ego_state)
        record.pop("location_obj", None)
        self._frame_file.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _write_event(self, event: Dict[str, Any]):
        self._event_records.append(event)
        if not self._event_file:
            return
        self._event_file.write(json.dumps(event, ensure_ascii=False) + "\n")
        self._event_file.flush()

    def _write_trace_state(self, snapshot, sim_time: float, actor_states: Dict[str, Dict[str, Any]]):
        if self.hero_actor and self.hero_actor.is_alive and "hero" not in actor_states:
            actor_states = dict(actor_states)
            actor_states["hero"] = self._extract_actor_state(self.hero_actor, snapshot, sim_time, self._resolve_delta_time(sim_time))
        record = {
            "frame": snapshot.frame,
            "simulation_time": round(sim_time, 6),
            "actors": actor_states,
        }
        self._trace_records.append(record)
        if not self._trace_file:
            return
        self._trace_file.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._trace_file.flush()

    def _update_actor_feedback_flags(self, actor_states: Dict[str, Dict[str, Any]], dt: float):
        for actor_name, state in actor_states.items():
            distance = state.get("nearest_driving_waypoint_distance_m")
            category = self._actor_category_by_name.get(actor_name)
            if category == "vehicle" and distance is not None and distance > 2.5:
                self._offroad_detected = True

            previous = self._previous_trace_state.get(actor_name)
            if previous and dt > 0.0:
                dx = state["x"] - previous["x"]
                dy = state["y"] - previous["y"]
                dz = state["z"] - previous["z"]
                implied_speed = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
                if implied_speed > 30.0:
                    self._teleport_detected = True
            self._previous_trace_state[actor_name] = state

    def _load_route(self):
        if not self.route_file_path or not os.path.isfile(self.route_file_path):
            return

        route_tree = ET.parse(self.route_file_path)
        route_points = route_tree.getroot().findall(".//waypoint")
        self._route_transforms = []
        for point in route_points:
            transform = carla.Transform(
                carla.Location(
                    x=float(point.get("x", 0.0)),
                    y=float(point.get("y", 0.0)),
                    z=float(point.get("z", 0.0)),
                ),
                carla.Rotation(
                    pitch=float(point.get("pitch", 0.0)),
                    yaw=float(point.get("yaw", 0.0)),
                    roll=float(point.get("roll", 0.0)),
                ),
            )
            self._route_transforms.append(transform)

        if not self._route_transforms:
            return

        accum_meters = []
        prev_loc = self._route_transforms[0].location
        for index, transform in enumerate(self._route_transforms):
            distance = transform.location.distance(prev_loc)
            prev_total = 0.0 if index == 0 else accum_meters[index - 1]
            accum_meters.append(distance + prev_total)
            prev_loc = transform.location

        total_distance = accum_meters[-1] if accum_meters else 0.0
        if total_distance <= 0.0:
            self._route_accum_perc = [0.0 for _ in accum_meters]
        else:
            self._route_accum_perc = [value / total_distance * 100.0 for value in accum_meters]
        self._route_target_location = self._route_transforms[-1].location

    def _build_rule_monitors(self):
        if not self.world or not self.map or not self.hero_actor:
            return

        for actor in self.world.get_actors():
            if "traffic_light" in actor.type_id:
                center, waypoints = self._get_traffic_light_waypoints(actor)
                self._traffic_lights.append((actor, center, waypoints))
            elif "traffic.stop" in actor.type_id:
                self._stop_signs.append(actor)

        self._last_ego_waypoint = self.map.get_waypoint(self.hero_actor.get_location())
        if self._last_ego_waypoint:
            self._last_lane_id = self._last_ego_waypoint.lane_id
            self._last_road_id = self._last_ego_waypoint.road_id

    def _setup_event_sensors(self):
        blueprint_library = self.world.get_blueprint_library()

        collision_bp = blueprint_library.find("sensor.other.collision")
        self._collision_sensor = self.world.spawn_actor(collision_bp, carla.Transform(), attach_to=self.hero_actor)
        self._collision_sensor.listen(self._on_collision)

        lane_bp = blueprint_library.find("sensor.other.lane_invasion")
        self._lane_invasion_sensor = self.world.spawn_actor(lane_bp, carla.Transform(), attach_to=self.hero_actor)
        self._lane_invasion_sensor.listen(self._on_lane_invasion)

    def _cleanup_sensors(self):
        if self._collision_sensor is not None:
            self._collision_sensor.stop()
            self._collision_sensor.destroy()
            self._collision_sensor = None
        if self._lane_invasion_sensor is not None:
            self._lane_invasion_sensor.stop()
            self._lane_invasion_sensor.destroy()
            self._lane_invasion_sensor = None

    def _on_collision(self, event):
        actor_location = self.hero_actor.get_location()
        impulse = event.normal_impulse
        other_role = event.other_actor.attributes.get("role_name") if hasattr(event.other_actor, "attributes") else None
        other_type = event.other_actor.type_id
        ego_role = self.hero_actor.attributes.get("role_name") if hasattr(self.hero_actor, "attributes") else "hero"
        if other_type == "static.unknown":
            if not self._static_contact_ignored:
                elapsed = event.timestamp - (self._episode_start_sim_time or event.timestamp)
                self._write_event(
                    {
                        "event_type": "static_contact_ignored",
                        "frame": event.frame,
                        "simulation_time": round(event.timestamp, 6),
                        "payload": {
                            "other_actor_id": event.other_actor.id,
                            "other_actor_type_id": other_type,
                            "elapsed_since_episode_start": round(elapsed, 6),
                            "ego_location": {
                                "x": round(actor_location.x, 6),
                                "y": round(actor_location.y, 6),
                                "z": round(actor_location.z, 6),
                            },
                        },
                    }
                )
                self._static_contact_ignored = True
            return

        self.summary["collision_count"] += 1
        self._termination_requested = True
        self._requested_termination_reason = "collision_exit"
        self._write_event(
            {
                "event_type": "collision",
                "frame": event.frame,
                "simulation_time": round(event.timestamp, 6),
                "payload": {
                    "other_actor_id": event.other_actor.id,
                    "other_actor_type_id": other_type,
                    "other_actor_role_name": other_role,
                    "actors": [name for name in [other_role, ego_role] if name],
                    "ego_location": {
                        "x": round(actor_location.x, 6),
                        "y": round(actor_location.y, 6),
                        "z": round(actor_location.z, 6),
                    },
                    "normal_impulse": {
                        "x": round(impulse.x, 6),
                        "y": round(impulse.y, 6),
                        "z": round(impulse.z, 6),
                    },
                },
            }
        )

    def _on_lane_invasion(self, event):
        self.summary["lane_invasion_count"] += 1
        markings = []
        for marking in event.crossed_lane_markings:
            markings.append(
                {
                    "type": str(marking.type),
                    "color": str(marking.color),
                    "lane_change": str(marking.lane_change),
                    "width": round(marking.width, 6),
                }
            )
        self._write_event(
            {
                "event_type": "lane_invasion",
                "frame": event.frame,
                "simulation_time": round(event.timestamp, 6),
                "payload": {"crossed_lane_markings": markings},
            }
        )

    def _check_synthetic_collisions(self, frame: int, sim_time: float):
        if not self.hero_actor or not self.hero_actor.is_alive:
            return

        actors = self._get_expected_live_actors()
        for index, first_actor in enumerate(actors):
            for second_actor in actors[index + 1:]:
                pair_key = tuple(sorted((first_actor.id, second_actor.id)))
                if pair_key in self._synthetic_collision_pairs:
                    continue
                if not self._actors_overlap(first_actor, second_actor):
                    continue

                self._synthetic_collision_pairs.add(pair_key)
                sut_involved = self.hero_actor.id in pair_key
                if sut_involved:
                    self.summary["collision_count"] += 1
                    self._termination_requested = True
                    self._requested_termination_reason = "collision_exit"
                first_name = first_actor.attributes.get("role_name") if hasattr(first_actor, "attributes") else None
                second_name = second_actor.attributes.get("role_name") if hasattr(second_actor, "attributes") else None
                self._write_event(
                    {
                        "event_type": "collision",
                        "frame": frame,
                        "simulation_time": round(sim_time, 6),
                        "payload": {
                            "source": "synthetic_bounding_box_overlap",
                            "ignored_background_collision": not sut_involved,
                            "actors": [name for name in [first_name, second_name] if name],
                            "actor_ids": [first_actor.id, second_actor.id],
                            "actor_type_ids": [first_actor.type_id, second_actor.type_id],
                            "overlap_method": "2d_oriented_bbox_with_z_overlap",
                        },
                    }
                )

    def _get_expected_live_actors(self) -> List[carla.Actor]:
        self._refresh_expected_actor_bindings()
        expected_names = {actor["name"] for actor in self.expected_actors}
        actors = []
        for actor in self.world.get_actors():
            if not actor.is_alive:
                continue
            role_name = actor.attributes.get("role_name") if hasattr(actor, "attributes") else None
            if role_name in expected_names:
                actors.append(actor)
        return actors

    def _actors_overlap(self, first_actor: carla.Actor, second_actor: carla.Actor) -> bool:
        first_transform = first_actor.get_transform()
        second_transform = second_actor.get_transform()
        if not self._z_ranges_overlap(first_actor, first_transform, second_actor, second_transform):
            return False

        first_polygon = self._actor_bbox_polygon(first_actor, first_transform)
        second_polygon = self._actor_bbox_polygon(second_actor, second_transform)
        if first_polygon is None or second_polygon is None:
            return False
        return bool(first_polygon.intersects(second_polygon))

    @staticmethod
    def _z_ranges_overlap(first_actor: carla.Actor, first_transform: carla.Transform,
                          second_actor: carla.Actor, second_transform: carla.Transform) -> bool:
        first_box = first_actor.bounding_box
        second_box = second_actor.bounding_box
        first_center_z = first_transform.location.z + first_box.location.z
        second_center_z = second_transform.location.z + second_box.location.z
        first_min = first_center_z - first_box.extent.z
        first_max = first_center_z + first_box.extent.z
        second_min = second_center_z - second_box.extent.z
        second_max = second_center_z + second_box.extent.z
        return first_min <= second_max and second_min <= first_max

    @staticmethod
    def _actor_bbox_polygon(actor: carla.Actor, transform: carla.Transform):
        box = actor.bounding_box
        yaw = math.radians(transform.rotation.yaw)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        center_x = transform.location.x + box.location.x * cos_yaw - box.location.y * sin_yaw
        center_y = transform.location.y + box.location.x * sin_yaw + box.location.y * cos_yaw
        corners = []
        for local_x, local_y in (
                (box.extent.x, box.extent.y),
                (box.extent.x, -box.extent.y),
                (-box.extent.x, -box.extent.y),
                (-box.extent.x, box.extent.y)):
            x = center_x + local_x * cos_yaw - local_y * sin_yaw
            y = center_y + local_x * sin_yaw + local_y * cos_yaw
            corners.append((x, y))
        return shapely.geometry.Polygon(corners)

    def _resolve_delta_time(self, sim_time: float) -> float:
        if self._last_sim_time is not None and sim_time >= self._last_sim_time:
            delta = sim_time - self._last_sim_time
            if delta > 0.0:
                return delta
        if self.fixed_delta_seconds:
            return float(self.fixed_delta_seconds)
        return 0.0

    def _resolve_waypoint(self, location: carla.Location):
        waypoint = self.map.get_waypoint(location, project_to_road=False, lane_type=carla.LaneType.Driving)
        if waypoint is None:
            waypoint = self.map.get_waypoint(location, project_to_road=False, lane_type=carla.LaneType.Parking)
        if waypoint is None:
            waypoint = self.map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Any)
        return waypoint

    def _project_to_vehicle_frame(self, vector: carla.Vector3D, yaw_deg: float) -> Tuple[float, float]:
        yaw_rad = math.radians(yaw_deg)
        forward_x = math.cos(yaw_rad)
        forward_y = math.sin(yaw_rad)
        right_x = -math.sin(yaw_rad)
        right_y = math.cos(yaw_rad)
        longitudinal = vector.x * forward_x + vector.y * forward_y
        lateral = vector.x * right_x + vector.y * right_y
        return longitudinal, lateral

    def _update_jerk(self, sim_time: float, frame: int, dt: float, ego_state: Dict[str, Any]):
        current_local_accel = (
            ego_state["longitudinal_accel"],
            ego_state["lateral_accel"],
        )
        if self._last_local_accel is None or dt <= 0.0:
            self._last_local_accel = current_local_accel
            return

        long_jerk = (current_local_accel[0] - self._last_local_accel[0]) / dt
        lat_jerk = (current_local_accel[1] - self._last_local_accel[1]) / dt
        self._last_local_accel = current_local_accel

        self._maybe_update_peak(
            "max_long_jerk",
            abs(long_jerk),
            frame,
            sim_time,
            {"value": round(long_jerk, 6)},
        )
        self._maybe_update_peak(
            "max_lat_jerk",
            abs(lat_jerk),
            frame,
            sim_time,
            {"value": round(lat_jerk, 6)},
        )

    def _update_distance_metrics(self, sim_time: float, frame: int):
        hero_location = self.hero_actor.get_location()
        hero_velocity = self.hero_actor.get_velocity()
        hero_velocity_np = np.array([hero_velocity.x, hero_velocity.y, hero_velocity.z], dtype=float)

        min_distance = None
        min_distance_actor = None
        min_ttc = None
        min_ttc_actor = None

        for actor in self.world.get_actors():
            if actor.id == self.hero_actor.id:
                continue
            if not actor.is_alive:
                continue
            if "vehicle" not in actor.type_id and "walker" not in actor.type_id:
                continue

            other_location = actor.get_location()
            relative_position = np.array(
                [
                    other_location.x - hero_location.x,
                    other_location.y - hero_location.y,
                    other_location.z - hero_location.z,
                ],
                dtype=float,
            )
            distance = float(np.linalg.norm(relative_position))
            if distance <= 1e-6:
                continue

            if min_distance is None or distance < min_distance:
                min_distance = distance
                min_distance_actor = actor

            other_velocity = actor.get_velocity()
            relative_velocity = np.array(
                [
                    other_velocity.x - hero_velocity_np[0],
                    other_velocity.y - hero_velocity_np[1],
                    other_velocity.z - hero_velocity_np[2],
                ],
                dtype=float,
            )
            closing_speed = -float(np.dot(relative_position, relative_velocity) / distance)
            if closing_speed > 1e-6:
                ttc = distance / closing_speed
                if min_ttc is None or ttc < min_ttc:
                    min_ttc = ttc
                    min_ttc_actor = actor

        if min_distance is not None:
            self._maybe_update_minimum(
                "min_distance",
                min_distance,
                frame,
                sim_time,
                min_distance_actor,
            )

        if min_ttc is not None:
            self._maybe_update_minimum(
                "min_ttc",
                min_ttc,
                frame,
                sim_time,
                min_ttc_actor,
            )
            if min_ttc < self.TTC_THRESHOLD_SECONDS:
                dt = self._resolve_delta_time(sim_time)
                self.summary["unsafe_exposure_time"] = round(
                    self.summary["unsafe_exposure_time"] + dt,
                    6,
                )

    def _update_offroad_time(self, dt: float, location: carla.Location):
        driving_wp = self.map.get_waypoint(location, project_to_road=False, lane_type=carla.LaneType.Driving)
        parking_wp = self.map.get_waypoint(location, project_to_road=False, lane_type=carla.LaneType.Parking)
        self._offroad_active = driving_wp is None and parking_wp is None
        if self._offroad_active and dt > 0.0:
            self.summary["off_road_time"] = round(self.summary["off_road_time"] + dt, 6)

    def _update_wrong_lane_time(self, dt: float, location: carla.Location):
        waypoint = self.map.get_waypoint(location, lane_type=carla.LaneType.Driving)
        if waypoint is None:
            self._wrong_lane_active = False
            return

        if waypoint.is_junction:
            self._wrong_lane_active = False
        elif self._last_road_id != waypoint.road_id or self._last_lane_id != waypoint.lane_id:
            if self._last_ego_waypoint and self._last_ego_waypoint.is_junction:
                wp_yaw = waypoint.transform.rotation.yaw % 360.0
                actor_yaw = self.hero_actor.get_transform().rotation.yaw % 360.0
                angle = (wp_yaw - actor_yaw) % 360.0
                self._wrong_lane_active = not (
                    angle < self.OPPOSITE_MAX_VEHICLE_ANGLE
                    or angle > (360.0 - self.OPPOSITE_MAX_VEHICLE_ANGLE)
                )
            elif self._last_ego_waypoint:
                last_wp_yaw = self._last_ego_waypoint.transform.rotation.yaw % 360.0
                wp_yaw = waypoint.transform.rotation.yaw % 360.0
                angle = (last_wp_yaw - wp_yaw) % 360.0
                if (
                    angle > self.OPPOSITE_MAX_WAYPOINT_ANGLE
                    and angle < (360.0 - self.OPPOSITE_MAX_WAYPOINT_ANGLE)
                ):
                    self._wrong_lane_active = not bool(self._wrong_lane_active)

        if self._wrong_lane_active and dt > 0.0:
            self.summary["opposite_lane_occupancy_time"] = round(
                self.summary["opposite_lane_occupancy_time"] + dt,
                6,
            )

        self._last_ego_waypoint = waypoint
        self._last_lane_id = waypoint.lane_id
        self._last_road_id = waypoint.road_id

    def _update_blocked_time(
        self,
        sim_time: float,
        dt: float,
        route_completion: float,
        speed: float,
        location: carla.Location,
    ):
        if self._last_progress_time is None:
            self._last_progress_time = sim_time
            self._last_route_completion = route_completion
            return

        if route_completion > (self._last_route_completion + self.BLOCKED_PROGRESS_EPSILON):
            self._last_progress_time = sim_time
            self._last_route_completion = route_completion

        if self._is_legitimate_stop(location):
            self._last_progress_time = sim_time
            return

        no_progress_duration = sim_time - self._last_progress_time
        if speed < self.BLOCKED_SPEED_THRESHOLD and no_progress_duration >= self.BLOCKED_WARMUP_SECONDS:
            self.summary["blocked_time"] = round(self.summary["blocked_time"] + max(dt, 0.0), 6)
            if self.summary["blocked_time"] >= self.BLOCKED_TERMINATION_SECONDS and not self._termination_requested:
                self._termination_requested = True
                self._requested_termination_reason = "blocked_exit"
                self._write_event(
                    {
                        "event_type": "blocked_termination",
                        "frame": self._get_snapshot_frame(),
                        "simulation_time": round(self._get_snapshot_time(), 6),
                        "payload": {
                            "blocked_time": self.summary["blocked_time"],
                            "speed_threshold": self.BLOCKED_SPEED_THRESHOLD,
                            "progress_epsilon": self.BLOCKED_PROGRESS_EPSILON,
                        },
                    }
                )

    def _is_legitimate_stop(self, location: carla.Location) -> bool:
        traffic_light = self.hero_actor.get_traffic_light()
        if traffic_light is not None:
            state = traffic_light.state
            if state in (carla.TrafficLightState.Red, carla.TrafficLightState.Yellow):
                return True

        check_waypoints = self._get_stop_waypoints()
        if self._target_stop_sign and self._is_actor_affected_by_stop(check_waypoints, self._target_stop_sign):
            return True

        waypoint = self.map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if waypoint and waypoint.is_junction:
            return True

        return False

    def _compute_route_completion(self, location: carla.Location) -> float:
        if not self._route_transforms:
            return 0.0

        route_length = len(self._route_transforms)
        search_end = min(self._route_index + self.ROUTE_WINDOW_SIZE + 1, route_length)
        for index in range(self._route_index, search_end):
            route_transform = self._route_transforms[index]
            route_location = route_transform.location
            wp_dir = route_transform.get_forward_vector()
            wp_vehicle = location - route_location
            if wp_vehicle.dot(wp_dir) > 0:
                self._route_index = index

        completion = round(self._route_accum_perc[self._route_index], 3)
        if (
            completion > self.ROUTE_SUCCESS_PERCENTAGE_THRESHOLD
            and self._route_target_location is not None
            and location.distance(self._route_target_location) < self.ROUTE_SUCCESS_DISTANCE_THRESHOLD
        ):
            completion = 100.0
        return completion

    def _check_red_light_violation(self, frame: int):
        transform = self.hero_actor.get_transform()
        location = transform.location
        veh_extent = self.hero_actor.bounding_box.extent.x

        tail_close_pt = self._rotate_point(carla.Vector3D(-0.8 * veh_extent, 0, 0), transform.rotation.yaw)
        tail_close_pt = location + carla.Location(tail_close_pt)
        tail_far_pt = self._rotate_point(carla.Vector3D(-veh_extent - 1.0, 0, 0), transform.rotation.yaw)
        tail_far_pt = location + carla.Location(tail_far_pt)

        for traffic_light, center, waypoints in self._traffic_lights:
            if self._last_red_light_id == traffic_light.id:
                continue
            if center.distance(location) > self.RED_LIGHT_DISTANCE:
                continue
            if traffic_light.state != carla.TrafficLightState.Red:
                continue

            for waypoint in waypoints:
                tail_wp = self.map.get_waypoint(tail_far_pt)
                if tail_wp is None:
                    continue
                vehicle_direction = transform.get_forward_vector()
                waypoint_direction = waypoint.transform.get_forward_vector()
                same_lane = tail_wp.road_id == waypoint.road_id and tail_wp.lane_id == waypoint.lane_id
                if not same_lane or vehicle_direction.dot(waypoint_direction) <= 0:
                    continue

                yaw_wp = waypoint.transform.rotation.yaw
                lane_width = waypoint.lane_width
                location_wp = waypoint.transform.location
                left_wp = self._rotate_point(carla.Vector3D(0.6 * lane_width, 0, 0), yaw_wp + 90.0)
                right_wp = self._rotate_point(carla.Vector3D(0.6 * lane_width, 0, 0), yaw_wp - 90.0)
                left_wp = location_wp + carla.Location(left_wp)
                right_wp = location_wp + carla.Location(right_wp)

                if self._is_vehicle_crossing_line((tail_close_pt, tail_far_pt), (left_wp, right_wp)):
                    self.summary["red_light_violation_count"] += 1
                    self._last_red_light_id = traffic_light.id
                    light_location = traffic_light.get_transform().location
                    self._write_event(
                        {
                            "event_type": "red_light_violation",
                            "frame": frame,
                            "simulation_time": round(self._get_snapshot_time(), 6),
                            "payload": {
                                "traffic_light_id": traffic_light.id,
                                "location": {
                                    "x": round(light_location.x, 6),
                                    "y": round(light_location.y, 6),
                                    "z": round(light_location.z, 6),
                                },
                            },
                        }
                    )
                    return

    def _check_stop_sign_violation(self, frame: int):
        actor_transform = self.hero_actor.get_transform()
        check_waypoints = self._get_stop_waypoints()

        if not self._target_stop_sign:
            self._target_stop_sign = self._scan_for_stop_sign(actor_transform, check_waypoints)
            return

        if not self._stop_completed:
            current_speed = self._get_speed(self.hero_actor.get_velocity())
            if current_speed < self.STOP_SPEED_THRESHOLD:
                self._stop_completed = True

        if not self._is_actor_affected_by_stop(check_waypoints, self._target_stop_sign):
            if not self._stop_completed and self._last_failed_stop != self._target_stop_sign.id:
                stop_location = self._target_stop_sign.get_transform().location
                self.summary["stop_sign_violation_count"] += 1
                self._write_event(
                    {
                        "event_type": "stop_sign_violation",
                        "frame": frame,
                        "simulation_time": round(self._get_snapshot_time(), 6),
                        "payload": {
                            "stop_sign_id": self._target_stop_sign.id,
                            "location": {
                                "x": round(stop_location.x, 6),
                                "y": round(stop_location.y, 6),
                                "z": round(stop_location.z, 6),
                            },
                        },
                    }
                )
                self._last_failed_stop = self._target_stop_sign.id

            self._target_stop_sign = None
            self._stop_completed = False

    def _get_stop_waypoints(self):
        steps = int(self.STOP_PROXIMITY_THRESHOLD / self.STOP_WAYPOINT_STEP)
        wp_list = []
        waypoint = self.map.get_waypoint(self.hero_actor.get_location())
        if waypoint is None:
            return wp_list
        wp_list.append(waypoint)
        next_wp = waypoint
        for _ in range(steps):
            next_waypoints = next_wp.next(self.STOP_WAYPOINT_STEP)
            if not next_waypoints:
                break
            next_wp = next_waypoints[0]
            wp_list.append(next_wp)
        return wp_list

    def _scan_for_stop_sign(self, actor_transform, wp_list):
        if not wp_list:
            return None

        actor_direction = actor_transform.get_forward_vector()
        actor_velocity = self.hero_actor.get_velocity()
        if actor_velocity.dot(actor_direction) < -0.17:
            return None

        lane_direction = wp_list[0].transform.get_forward_vector()
        if actor_direction.dot(lane_direction) < -0.17:
            return None

        for stop_sign in self._stop_signs:
            if self._is_actor_affected_by_stop(wp_list, stop_sign):
                return stop_sign
        return None

    def _is_actor_affected_by_stop(self, wp_list, stop_sign):
        if not wp_list:
            return False

        stop_location = stop_sign.get_transform().transform(stop_sign.trigger_volume.location)
        actor_location = wp_list[0].transform.location
        if stop_location.distance(actor_location) > self.STOP_PROXIMITY_THRESHOLD:
            return False

        stop_extent = stop_sign.trigger_volume.extent
        for actor_wp in wp_list:
            if self._point_inside_boundingbox(actor_wp.transform.location, stop_location, stop_extent):
                return True
        return False

    def _point_inside_boundingbox(self, point, bb_center, bb_extent, multiplier=1.2):
        a = carla.Vector2D(bb_center.x - multiplier * bb_extent.x, bb_center.y - multiplier * bb_extent.y)
        b = carla.Vector2D(bb_center.x + multiplier * bb_extent.x, bb_center.y - multiplier * bb_extent.y)
        d = carla.Vector2D(bb_center.x - multiplier * bb_extent.x, bb_center.y + multiplier * bb_extent.y)
        m = carla.Vector2D(point.x, point.y)

        ab = b - a
        ad = d - a
        am = m - a
        am_ab = am.x * ab.x + am.y * ab.y
        ab_ab = ab.x * ab.x + ab.y * ab.y
        am_ad = am.x * ad.x + am.y * ad.y
        ad_ad = ad.x * ad.x + ad.y * ad.y
        return am_ab > 0 and am_ab < ab_ab and am_ad > 0 and am_ad < ad_ad

    def _is_vehicle_crossing_line(self, seg1, seg2):
        line1 = shapely.geometry.LineString([(seg1[0].x, seg1[0].y), (seg1[1].x, seg1[1].y)])
        line2 = shapely.geometry.LineString([(seg2[0].x, seg2[0].y), (seg2[1].x, seg2[1].y)])
        return not line1.intersection(line2).is_empty

    def _rotate_point(self, point, angle):
        x_value = math.cos(math.radians(angle)) * point.x - math.sin(math.radians(angle)) * point.y
        y_value = math.sin(math.radians(angle)) * point.x + math.cos(math.radians(angle)) * point.y
        return carla.Vector3D(x_value, y_value, point.z)

    def _get_traffic_light_waypoints(self, traffic_light):
        base_transform = traffic_light.get_transform()
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(traffic_light.trigger_volume.location)
        area_extent = traffic_light.trigger_volume.extent
        x_values = np.arange(-0.9 * area_extent.x, 0.9 * area_extent.x, 1.0)

        area = []
        for x_value in x_values:
            point = self._rotate_point(carla.Vector3D(x_value, 0, area_extent.z), base_rot)
            point_location = area_loc + carla.Location(x=point.x, y=point.y)
            area.append(point_location)

        initial_waypoints = []
        for point in area:
            waypoint = self.map.get_waypoint(point)
            if waypoint is None:
                continue
            if (
                not initial_waypoints
                or initial_waypoints[-1].road_id != waypoint.road_id
                or initial_waypoints[-1].lane_id != waypoint.lane_id
            ):
                initial_waypoints.append(waypoint)

        final_waypoints = []
        for waypoint in initial_waypoints:
            while not waypoint.is_intersection:
                next_waypoint = waypoint.next(0.5)[0]
                if next_waypoint and not next_waypoint.is_intersection:
                    waypoint = next_waypoint
                else:
                    break
            final_waypoints.append(waypoint)

        return area_loc, final_waypoints

    def _get_traffic_light_state(self):
        traffic_light = self.hero_actor.get_traffic_light()
        if traffic_light is None:
            return None
        return str(traffic_light.state)

    def _maybe_update_minimum(self, key: str, value: float, frame: int, sim_time: float, actor: Optional[carla.Actor]):
        current = self.summary.get(key)
        if current is not None and current["value"] <= value:
            return
        self.summary[key] = {
            "value": round(value, 6),
            "frame": frame,
            "simulation_time": round(sim_time, 6),
            "other_actor_id": actor.id if actor else None,
            "other_actor_type_id": actor.type_id if actor else None,
        }

    def _maybe_update_peak(self, key: str, magnitude: float, frame: int, sim_time: float, payload: Dict[str, Any]):
        current = self.summary.get(key)
        current_value = -1.0 if current is None else abs(current["value"])
        if magnitude <= current_value:
            return
        data = {
            "value": round(payload["value"], 6),
            "frame": frame,
            "simulation_time": round(sim_time, 6),
        }
        self.summary[key] = data

    def _serialize_criteria(self, criteria: List[Any]) -> List[Dict[str, Any]]:
        serialized = []
        for criterion in criteria:
            serialized.append(
                {
                    "name": getattr(criterion, "name", None),
                    "test_status": getattr(criterion, "test_status", None),
                    "actual_value": getattr(criterion, "actual_value", None),
                    "success_value": getattr(criterion, "success_value", None),
                    "units": getattr(criterion, "units", None),
                }
            )
        return serialized

    def _write_sim_feedback(self):
        os.makedirs(self.paths["keyframes_dir"], exist_ok=True)
        feedback = self._build_sim_feedback()
        try:
            self._render_feedback_artifacts(feedback)
        except Exception as exc:  # pylint: disable=broad-except
            feedback.setdefault("notes", []).append(f"artifact_render_failed: {type(exc).__name__}: {exc}")

        with open(self.paths["sim_feedback"], "w", encoding="utf-8") as file_obj:
            json.dump(feedback, file_obj, indent=2, ensure_ascii=False)

    def _build_sim_feedback(self) -> Dict[str, Any]:
        collision_event = self._first_event("collision")
        collision_time = collision_event.get("simulation_time") if collision_event else None
        collision_actors = self._collision_actor_names(collision_event)
        trigger_time, trigger_actor = self._find_motion_start()
        key_times = self._select_key_times(trigger_time, collision_time)
        key_states = [
            {
                "label": label,
                "time_s": round(time_value, 3),
                "actors": self._states_at_time(time_value),
            }
            for label, time_value in key_times
        ]
        min_distance = self._compute_min_distance(collision_actors)
        feedback_events = []
        if trigger_time is not None:
            feedback_events.append(
                {
                    "time_s": round(trigger_time, 3),
                    "type": "causal_trigger_motion_start",
                    "actors": [trigger_actor] if trigger_actor else [],
                    "details": "actor speed exceeded 0.2 m/s",
                }
            )
        if collision_event:
            feedback_events.append(
                {
                    "time_s": round(collision_time, 3),
                    "type": "collision",
                    "actors": collision_actors,
                    "details": "collision sensor event",
                }
            )

        scenario_tree_status = self.summary.get("scenario_tree_status")
        termination_reason = self.summary.get("termination_reason")
        timeout = bool(termination_reason and "timeout" in termination_reason.lower())
        timeout = timeout or bool(scenario_tree_status and "TIMEOUT" in scenario_tree_status)
        failed = bool(scenario_tree_status and "FAILURE" in scenario_tree_status)
        failed = failed or bool(termination_reason and termination_reason.startswith("exception:"))
        runner_status = "timeout" if timeout else "failed" if failed else "completed"
        missing_actors = [actor["name"] for actor in self.expected_actors if actor["name"] not in self._spawned_actor_names]
        notes = []
        if min_distance is not None:
            notes.append("min_distance_m uses actor center distance from sampled transforms; bounding-box distance is not available in the raw trace.")

        return {
            "scenario": self.scenario_name,
            "xosc_path": self.scenario_path,
            "mid_model_path": self.mid_model_path,
            "runner": {
                "carla_version": getattr(carla, "__version__", None),
                "scenario_runner_version": "0.9.16",
                "map": self.map.name.split("/")[-1] if self.map else None,
                "fixed_delta_seconds": self.fixed_delta_seconds,
                "status": runner_status,
            },
            "summary": {
                "spawn_success": len(missing_actors) == 0,
                "all_expected_actors_spawned": len(missing_actors) == 0,
                "missing_actors": missing_actors,
                "controller_failures": [],
                "collision_detected": bool(collision_event or self.summary.get("collision_count", 0) > 0),
                "collision_time_s": round(collision_time, 3) if collision_time is not None else None,
                "collision_actors": collision_actors,
                "min_distance_m": round(min_distance, 3) if min_distance is not None else None,
                "offroad_detected": bool(self._offroad_detected or self.summary.get("off_road_time", 0.0) > 0.0),
                "teleport_detected": bool(self._teleport_detected),
                "timeout": timeout,
            },
            "behavior_checks": {
                "hero_turn_visible": self._hero_turn_visible(),
                "hero_slowed_before_impact": self._slowed_before_impact("hero", collision_time),
                "causal_trigger_moved_before_collision": bool(
                    trigger_time is not None and (collision_time is None or trigger_time < collision_time)
                ),
                "initiator_continued_after_collision": self._initiator_continued_after_collision(collision_actors, collision_time),
                "target_stopped_or_slowed_before_collision": self._slowed_before_impact("hero", collision_time),
                "impact_area_estimated": self._estimate_impact_area(collision_actors, collision_time),
            },
            "events": feedback_events,
            "key_states": key_states,
            "trajectory_samples": self._trajectory_samples(),
            "artifacts": {
                "path_plot": self._relative_artifact_path(self.paths["path_plot"]),
                "contact_sheet": self._relative_artifact_path(self.paths["contact_sheet"]),
                "keyframes_dir": self._relative_artifact_path(self.paths["keyframes_dir"]),
                "video": self._relative_artifact_path(self.paths["video"]) if self.record_video else None,
            },
            "notes": notes,
        }

    def _relative_artifact_path(self, path: str) -> str:
        try:
            return os.path.relpath(path, os.getcwd())
        except ValueError:
            return path

    def _first_event(self, event_type: str) -> Optional[Dict[str, Any]]:
        for event in self._event_records:
            if event.get("event_type") == event_type:
                return event
        return None

    def _collision_actor_names(self, collision_event: Optional[Dict[str, Any]]) -> List[str]:
        if not collision_event:
            return []
        payload = collision_event.get("payload", {})
        actors = payload.get("actors") or []
        if actors:
            return actors
        other_name = payload.get("other_actor_role_name")
        return [name for name in [other_name, "hero"] if name]

    def _find_motion_start(self) -> Tuple[Optional[float], Optional[str]]:
        for record in self._trace_records:
            for actor_name, state in record.get("actors", {}).items():
                if actor_name == "hero":
                    continue
                if state.get("speed_mps", 0.0) > 0.2:
                    return record["simulation_time"], actor_name
        return None, None

    def _select_key_times(self, trigger_time: Optional[float], collision_time: Optional[float]) -> List[Tuple[str, float]]:
        start_time = self._trace_records[0]["simulation_time"] if self._trace_records else 0.0
        end_time = self._trace_records[-1]["simulation_time"] if self._trace_records else start_time
        if collision_time is None:
            collision_time = end_time
        return [
            ("start", start_time),
            ("trigger", trigger_time if trigger_time is not None else start_time),
            ("pre_collision", max(start_time, collision_time - 1.0)),
            ("collision", collision_time),
            ("post_collision", min(end_time, collision_time + 3.0)),
        ]

    def _nearest_record(self, time_s: float) -> Optional[Dict[str, Any]]:
        if not self._trace_records:
            return None
        return min(self._trace_records, key=lambda record: abs(record["simulation_time"] - time_s))

    def _states_at_time(self, time_s: float) -> Dict[str, Dict[str, float]]:
        record = self._nearest_record(time_s)
        if not record:
            return {}
        states = {}
        for actor_name, state in record.get("actors", {}).items():
            states[actor_name] = {
                "x": state["x"],
                "y": state["y"],
                "z": state["z"],
                "yaw": state["yaw"],
                "speed_mps": state["speed_mps"],
            }
        return states

    def _compute_min_distance(self, actor_names: List[str]) -> Optional[float]:
        names = actor_names or [actor["name"] for actor in self.expected_actors]
        if len(names) < 2:
            return None
        min_distance = None
        for record in self._trace_records:
            actors = record.get("actors", {})
            for idx, first_name in enumerate(names):
                first = actors.get(first_name)
                if not first:
                    continue
                for second_name in names[idx + 1:]:
                    second = actors.get(second_name)
                    if not second:
                        continue
                    distance = math.hypot(first["x"] - second["x"], first["y"] - second["y"])
                    if min_distance is None or distance < min_distance:
                        min_distance = distance
        return min_distance

    def _hero_turn_visible(self) -> bool:
        hero_states = [record.get("actors", {}).get("hero") for record in self._trace_records]
        hero_states = [state for state in hero_states if state]
        if len(hero_states) < 2:
            return False
        yaw_values = [state["yaw"] for state in hero_states]
        yaw_change = max(yaw_values) - min(yaw_values)
        xs = [state["x"] for state in hero_states]
        ys = [state["y"] for state in hero_states]
        lateral_span = min(max(xs) - min(xs), max(ys) - min(ys))
        return yaw_change > 35.0 and lateral_span > 1.0

    def _slowed_before_impact(self, actor_name: str, collision_time: Optional[float]) -> bool:
        if collision_time is None:
            return False
        window_states = [
            record.get("actors", {}).get(actor_name)
            for record in self._trace_records
            if collision_time - 5.0 <= record["simulation_time"] <= collision_time
        ]
        window_states = [state for state in window_states if state]
        if len(window_states) < 2:
            return False
        speeds = [state["speed_mps"] for state in window_states]
        return max(speeds) - speeds[-1] > 1.0

    def _initiator_continued_after_collision(self, actor_names: List[str], collision_time: Optional[float]) -> bool:
        if collision_time is None:
            return False
        candidates = [name for name in actor_names if name != "hero"] or [name for name in self._actor_category_by_name if name != "hero"]
        for actor_name in candidates:
            states = [
                record.get("actors", {}).get(actor_name)
                for record in self._trace_records
                if collision_time <= record["simulation_time"] <= collision_time + 4.0
            ]
            states = [state for state in states if state]
            if len(states) < 2:
                continue
            moved = math.hypot(states[-1]["x"] - states[0]["x"], states[-1]["y"] - states[0]["y"])
            if moved > 3.0 or max(state["speed_mps"] for state in states) > 1.0:
                return True
        return False

    def _estimate_impact_area(self, actor_names: List[str], collision_time: Optional[float]) -> str:
        if collision_time is None or "hero" not in actor_names:
            return "unknown"
        other_names = [name for name in actor_names if name != "hero"]
        if not other_names:
            return "unknown"
        states = self._states_at_time(collision_time)
        hero_state = states.get("hero")
        other_state = states.get(other_names[0])
        if not hero_state or not other_state:
            return "unknown"
        dx = other_state["x"] - hero_state["x"]
        dy = other_state["y"] - hero_state["y"]
        yaw_rad = math.radians(hero_state["yaw"])
        forward = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
        right = dx * -math.sin(yaw_rad) + dy * math.cos(yaw_rad)
        if forward > 1.2:
            return "front"
        if forward < -1.2:
            return "rear"
        if right > 0.8:
            return "right_side"
        if right < -0.8:
            return "left_side"
        return "unknown"

    def _trajectory_samples(self) -> Dict[str, Any]:
        samples = {"sample_rate_hz": 1, "actors": {}}
        next_sample_time: Dict[str, float] = {}
        for record in self._trace_records:
            time_s = record["simulation_time"]
            for actor_name, state in record.get("actors", {}).items():
                if time_s + 1e-6 < next_sample_time.get(actor_name, -1.0):
                    continue
                samples["actors"].setdefault(actor_name, []).append(
                    [round(time_s, 3), state["x"], state["y"], state["yaw"], state["speed_mps"]]
                )
                next_sample_time[actor_name] = time_s + 1.0
        return samples

    def _render_feedback_artifacts(self, feedback: Dict[str, Any]):
        from PIL import Image, ImageDraw, ImageFont

        records = self._trace_records
        if not records:
            return

        os.makedirs(self.paths["keyframes_dir"], exist_ok=True)
        bounds = self._plot_bounds(records)
        font = ImageFont.load_default()
        path_image = self._draw_topdown(records[-1], bounds, feedback, (1200, 900), font, draw_full_paths=True)
        path_image.save(self.paths["path_plot"])

        keyframe_paths = []
        for key_state in feedback.get("key_states", []):
            record = self._nearest_record(key_state["time_s"])
            if not record:
                continue
            image = self._draw_topdown(record, bounds, feedback, (900, 700), font, draw_full_paths=True)
            draw = ImageDraw.Draw(image)
            draw.rectangle((12, 12, 260, 42), fill=(255, 255, 255), outline=(30, 41, 59))
            draw.text((22, 22), f't={key_state["time_s"]:.2f}s {key_state["label"]}', fill=(15, 23, 42), font=font)
            frame_path = os.path.join(self.paths["keyframes_dir"], f'{key_state["label"]}.png')
            image.save(frame_path)
            keyframe_paths.append(frame_path)

        if keyframe_paths:
            thumbs = [Image.open(path).resize((360, 280)) for path in keyframe_paths]
            sheet = Image.new("RGB", (360 * len(thumbs), 280), (248, 250, 252))
            for idx, thumb in enumerate(thumbs):
                sheet.paste(thumb, (idx * 360, 0))
            sheet.save(self.paths["contact_sheet"], quality=92)

        if self.record_video:
            video_path = self._render_topdown_video(records, bounds, feedback, font)
            if video_path:
                self.summary["video_path"] = self._relative_artifact_path(video_path)
                feedback["artifacts"]["video"] = self._relative_artifact_path(video_path)

    def _render_topdown_video(self, records, bounds, feedback, font) -> Optional[str]:
        from PIL import Image

        frame_dir = os.path.join(self.output_dir, "_video_frames")
        os.makedirs(frame_dir, exist_ok=True)

        selected_records = records[::self.video_frame_stride]
        if records and selected_records[-1] is not records[-1]:
            selected_records.append(records[-1])
        if not selected_records:
            return None

        try:
            for index, record in enumerate(selected_records):
                image = self._draw_topdown(record, bounds, feedback, (1280, 720), font, draw_full_paths=True)
                frame_path = os.path.join(frame_dir, f"frame_{index:06d}.png")
                image.save(frame_path)

            if shutil.which("ffmpeg"):
                command = [
                    "ffmpeg",
                    "-y",
                    "-framerate",
                    str(self.video_fps),
                    "-i",
                    os.path.join(frame_dir, "frame_%06d.png"),
                    "-c:v",
                    "libx264",
                    "-pix_fmt",
                    "yuv420p",
                    self.paths["video"],
                ]
                completed = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False)
                if completed.returncode == 0 and os.path.isfile(self.paths["video"]):
                    print(f"Trajectory video saved to: {self.paths['video']}")
                    return self.paths["video"]

            gif_path = os.path.splitext(self.paths["video"])[0] + ".gif"
            images = [Image.open(os.path.join(frame_dir, f"frame_{index:06d}.png")) for index in range(len(selected_records))]
            if images:
                images[0].save(
                    gif_path,
                    save_all=True,
                    append_images=images[1:],
                    duration=int(1000 / self.video_fps),
                    loop=0,
                )
                self.paths["video"] = gif_path
                print(f"ffmpeg failed; trajectory GIF saved to: {gif_path}")
                return gif_path
        finally:
            shutil.rmtree(frame_dir, ignore_errors=True)

        return None

    def _plot_bounds(self, records: List[Dict[str, Any]]) -> Tuple[float, float, float, float]:
        xs = []
        ys = []
        for record in records:
            for state in record.get("actors", {}).values():
                xs.append(state["x"])
                ys.append(state["y"])
        if not xs or not ys:
            return 0.0, 1.0, 0.0, 1.0
        pad = 8.0
        return min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad

    def _draw_topdown(self, record, bounds, feedback, size, font, draw_full_paths=False):
        from PIL import Image, ImageDraw

        width, height = size
        image = Image.new("RGB", size, (248, 250, 252))
        draw = ImageDraw.Draw(image)
        min_x, max_x, min_y, max_y = bounds

        def project(x_value, y_value):
            scale = min((width - 90) / max(max_x - min_x, 1.0), (height - 90) / max(max_y - min_y, 1.0))
            px = 45 + (x_value - min_x) * scale
            py = height - (45 + (y_value - min_y) * scale)
            return px, py

        colors = {
            "hero": (37, 99, 235),
            "adversary": (220, 38, 38),
            "other_1": (220, 38, 38),
            "other_2": (22, 163, 74),
        }
        if draw_full_paths:
            for actor in self.expected_actors:
                actor_name = actor["name"]
                points = [
                    project(state["x"], state["y"])
                    for state in (trace.get("actors", {}).get(actor_name) for trace in self._trace_records)
                    if state
                ]
                if len(points) >= 2:
                    draw.line(points, fill=colors.get(actor_name, (100, 116, 139)), width=3)
                    draw.ellipse((points[0][0] - 5, points[0][1] - 5, points[0][0] + 5, points[0][1] + 5), fill=(15, 118, 110))
                    draw.rectangle((points[-1][0] - 5, points[-1][1] - 5, points[-1][0] + 5, points[-1][1] + 5), fill=(124, 45, 18))

        collision_time = feedback.get("summary", {}).get("collision_time_s")
        collision_state = self._states_at_time(collision_time) if collision_time is not None else {}
        collision_names = feedback.get("summary", {}).get("collision_actors") or []
        if collision_names:
            collision_state = {name: state for name, state in collision_state.items() if name in collision_names}
        if collision_state:
            xs = [state["x"] for state in collision_state.values()]
            ys = [state["y"] for state in collision_state.values()]
            cx, cy = project(sum(xs) / len(xs), sum(ys) / len(ys))
            draw.ellipse((cx - 9, cy - 9, cx + 9, cy + 9), outline=(0, 0, 0), width=3)
            draw.text((cx + 12, cy - 8), f'collision t={collision_time:.2f}s', fill=(0, 0, 0), font=font)

        for actor_name, state in record.get("actors", {}).items():
            px, py = project(state["x"], state["y"])
            color = colors.get(actor_name, (100, 116, 139))
            if self._actor_category_by_name.get(actor_name) == "pedestrian":
                draw.ellipse((px - 8, py - 8, px + 8, py + 8), fill=color, outline=(15, 23, 42))
            else:
                draw.rectangle((px - 10, py - 6, px + 10, py + 6), fill=color, outline=(15, 23, 42))
            draw.text((px + 12, py + 8), f'{actor_name} t={record["simulation_time"]:.1f}s', fill=(15, 23, 42), font=font)

        draw.text((18, height - 28), "top-down trajectory plot; points are sampled actor centers", fill=(71, 85, 105), font=font)
        return image

    def _get_snapshot_frame(self):
        if not self.world:
            return None
        snapshot = self.world.get_snapshot()
        return snapshot.frame if snapshot else None

    def _get_snapshot_time(self):
        if not self.world:
            return None
        snapshot = self.world.get_snapshot()
        return snapshot.timestamp.elapsed_seconds if snapshot else None

    def _get_speed(self, velocity: carla.Vector3D) -> float:
        return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
