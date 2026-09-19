"""Track authored OpenSCENARIO paths with CARLA vehicle physics.

Trajectory coordinates are steering references, never actor poses. This
controller only issues throttle/brake/steer, so impacts can deflect or stop an
actor. It must only control explicitly scripted participants, not a PCLA SUT.
"""
import bisect
import json
import math

import carla

from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
from srunner.scenariomanager.timer import GameTime


class PhysicsTrajectoryControl(BasicControl):
    def __init__(self, actor, args=None):
        super(PhysicsTrajectoryControl, self).__init__(actor)
        args = args or {}
        self._waypoints = []
        self._times = []
        self._start_time = None
        self._hold_brake = float(args.get("hold_brake", 1.0))
        self._max_speed = float(args.get("max_speed", 12.0))
        bicycle = actor.type_id in ("vehicle.bh.crossbike", "vehicle.diamondback.century", "vehicle.gazelle.omafiets")
        self._wheelbase = float(args.get("wheelbase", 1.1 if bicycle else 2.8))
        self._max_steer = float(args.get("max_steer_radians", .7))
        self._last_steer = 0.0
        self._last_time = None
        self._last_log_second = -1
        self._is_walker = actor.type_id.startswith("walker.")
        self._released = False
        self._contact_sensor = None
        if str(args.get("release_on_collision", "false")).lower() == "true":
            world = actor.get_world()
            self._contact_sensor = world.spawn_actor(world.get_blueprint_library().find("sensor.other.collision"), carla.Transform(), attach_to=actor)
            self._contact_sensor.listen(self._release_after_contact)
        print("PhysicsTrajectoryControl actor={} id={} (apply_control only)".format(
            actor.attributes.get("role_name"), actor.id), flush=True)

    def _release_after_contact(self, event):
        if event.other_actor.type_id.startswith("vehicle.") and not self._released:
            self._released = True
            print("PHYSICS_RELEASE " + json.dumps({"actor": self._actor.attributes.get("role_name"),
                  "simulation_time": event.timestamp, "other_actor_id": event.other_actor.id}), flush=True)

    def update_waypoints(self, waypoints, start_time=None, times=None):
        self._waypoints = list(waypoints or [])
        self._times = list(times or [])
        self._start_time = GameTime.get_time() if start_time is None else start_time
        self._reached_goal = False
        if self._times and (len(self._times) != len(self._waypoints) or any(
                b <= a for a, b in zip(self._times, self._times[1:]))):
            raise ValueError("Physical trajectory requires one strictly increasing time per vertex")
        if self._waypoints and not self._times:
            self._times = [0.0]
            for a, b in zip(self._waypoints, self._waypoints[1:]):
                self._times.append(self._times[-1] + max(.05, a.location.distance(b.location) / max(self._target_speed, 1.0)))

    def _sample(self, timestamp):
        idx = min(len(self._times) - 2, max(0, bisect.bisect_right(self._times, timestamp) - 1))
        a, b = self._waypoints[idx], self._waypoints[idx + 1]
        span = self._times[idx + 1] - self._times[idx]
        alpha = max(0.0, min(1.0, (timestamp - self._times[idx]) / span))
        dx, dy = b.location.x - a.location.x, b.location.y - a.location.y
        length = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx) if length > .001 else math.radians(a.rotation.yaw)
        return a.location.x + alpha * dx, a.location.y + alpha * dy, length / span, yaw

    def run_step(self):
        if self._actor is None or not self._actor.is_alive:
            return
        if self._released:
            self._actor.apply_control(carla.VehicleControl(throttle=0, brake=0, steer=0))
            return
        now = GameTime.get_time()
        dt = max(.01, min(.1, now - self._last_time)) if self._last_time is not None else .05
        self._last_time = now
        if len(self._waypoints) < 2:
            self._actor.apply_control(carla.WalkerControl(speed=0) if self._is_walker else carla.VehicleControl(throttle=0, brake=self._hold_brake))
            return
        elapsed = max(0.0, now - self._start_time)
        transform = self._actor.get_transform()
        position = transform.location
        velocity = self._actor.get_velocity()
        speed = math.hypot(velocity.x, velocity.y)
        x, y, desired_speed, heading = self._sample(elapsed)
        along_error = (x - position.x) * math.cos(heading) + (y - position.y) * math.sin(heading)
        if desired_speed < .02 or elapsed >= self._times[-1]:
            target_speed = 0.0
        else:
            # Limited schedule correction cannot make the actor jump forward or
            # gain arbitrary speed after an impact delays its planned motion.
            target_speed = min(self._max_speed, max(0.0, desired_speed + max(-2.0, min(2.0, .5 * along_error))))
        lookahead = max(2.0, min(6.0, speed * .65 + 1.5))
        tx, ty, _, _ = self._sample(elapsed + lookahead / max(desired_speed, 1.0))
        yaw = math.radians(transform.rotation.yaw)
        dx, dy = tx - position.x, ty - position.y
        if self._is_walker:
            distance = max(math.hypot(dx, dy), .001)
            self._actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(dx / distance, dy / distance, 0), speed=target_speed))
            return
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        steer_angle = math.atan2(2.0 * self._wheelbase * lateral, max(dx * dx + dy * dy, 1.0))
        wanted_steer = max(-1.0, min(1.0, steer_angle / self._max_steer))
        steer = max(self._last_steer - 2.5 * dt, min(self._last_steer + 2.5 * dt, wanted_steer))
        self._last_steer = steer
        error = target_speed - speed
        throttle = max(0.0, min(.8, .3 * error + (.08 if target_speed > 0 else 0.0)))
        brake = max(0.0, min(1.0, -.3 * error))
        if target_speed < .02:
            throttle, brake = 0.0, self._hold_brake
        self._actor.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=steer))
        if int(elapsed) != self._last_log_second:
            self._last_log_second = int(elapsed)
            print("PHYSICS_CONTROL " + json.dumps({"actor": self._actor.attributes.get("role_name"),
                  "game_time": round(now, 3), "trajectory_time": round(elapsed, 3),
                  "reference_speed": round(desired_speed, 3), "speed": round(speed, 3),
                  "target_speed": round(target_speed, 3), "along_error": round(along_error, 3),
                  "throttle": round(throttle, 3), "brake": round(brake, 3), "steer": round(steer, 3)}), flush=True)
        self._reached_goal = elapsed >= self._times[-1] and position.distance(self._waypoints[-1].location) < 2

    def reset(self):
        if self._contact_sensor is not None:
            try:
                self._contact_sensor.stop()
                self._contact_sensor.destroy()
            except RuntimeError:
                pass
            self._contact_sensor = None
        self._actor = None
