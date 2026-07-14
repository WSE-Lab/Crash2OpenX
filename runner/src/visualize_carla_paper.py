#!/usr/bin/env python3
"""Record a paper-ready CARLA view without changing simulation semantics.

The recorder attaches a second RGB sensor to the SUT, then composites live-map
lane geometry, actor identities, and RoadSeed/SceneSeed metadata onto each
frame.  It is intentionally separate from the raw RGB recorder used as the
experimental artifact.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record an annotated CARLA paper view")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--actor-role", default="hero")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--fov", type=float, default=78.0)
    parser.add_argument("--camera-x", type=float, default=-22.0)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument("--camera-z", type=float, default=14.0)
    parser.add_argument("--camera-pitch", type=float, default=-32.0)
    parser.add_argument("--camera-yaw", type=float, default=0.0)
    parser.add_argument("--save-dir", required=True)
    parser.add_argument("--save-every", type=int, default=1)
    parser.add_argument("--meta", default=None, help="RoadSeed/SceneSeed paper metadata JSON")
    parser.add_argument("--timeout", type=float, default=10.0)
    return parser.parse_args()


def _load_carla():
    try:
        import carla
        return carla
    except ImportError:
        root = os.environ.get("CARLA_ROOT", os.path.expanduser("~/carla"))
        sys.path.append(os.path.join(root, "PythonAPI", "carla"))
        import glob
        sys.path.extend(glob.glob(os.path.join(root, "PythonAPI", "carla", "dist", "carla-*py3*.egg")))
        import carla
        return carla


def load_meta(path: str | None) -> dict:
    if not path:
        return {}
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"paper metadata unavailable: {exc}", flush=True)
        return {}


def find_actor_by_role(world, role_name: str):
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == role_name:
            return actor
    for actor in world.get_actors():
        if actor.attributes.get("role_name") == role_name:
            return actor
    return None


def wait_for_actor(client, role_name: str):
    while True:
        world = client.get_world()
        actor = find_actor_by_role(world, role_name)
        if actor is not None:
            return world, actor
        print(f"waiting for actor role_name={role_name!r}", flush=True)
        time.sleep(0.5)


def configure_weather(carla, world, meta: dict) -> str:
    environment = ((meta.get("scene") or {}).get("environment") or {})
    weather = str(environment.get("weather") or "unknown").lower()
    time_of_day = str(environment.get("time_of_day") or "unknown").lower()

    # Weather and time are independent Environment fields.  Build one combined
    # CARLA setting so a rainy-night seed does not silently become rainy noon.
    preset = carla.WeatherParameters.ClearNoon
    if weather in {"rain", "rainy", "heavy_rain"}:
        preset.cloudiness = 90.0
        preset.precipitation = 70.0
        preset.precipitation_deposits = 80.0
        preset.wetness = 90.0
        preset.wind_intensity = 35.0
        weather_label = "rain"
    elif weather in {"fog", "foggy"}:
        preset.cloudiness = 80.0
        preset.fog_density = 35.0
        preset.fog_distance = 20.0
        weather_label = "fog"
    elif weather in {"cloudy", "overcast"}:
        preset.cloudiness = 80.0
        weather_label = "cloudy"
    elif weather in {"clear", "sunny"}:
        weather_label = "clear"
    else:
        weather_label = "unspecified"

    if time_of_day in {"night", "nighttime"}:
        preset.sun_altitude_angle = -20.0
        time_label = "night"
    elif time_of_day in {"evening", "sunset", "dusk"}:
        preset.sun_altitude_angle = 5.0
        time_label = "evening"
    elif time_of_day in {"dawn", "sunrise"}:
        preset.sun_altitude_angle = 8.0
        time_label = "dawn"
    elif time_of_day in {"day", "daytime", "noon"}:
        preset.sun_altitude_angle = 65.0
        time_label = "day"
    else:
        time_label = "unspecified"

    world.set_weather(preset)
    if time_label == "night":
        lights = carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam
        for vehicle in world.get_actors().filter("vehicle.*"):
            try:
                vehicle.set_light_state(carla.VehicleLightState(lights))
            except RuntimeError:
                pass
    if weather_label == "unspecified" and time_label == "unspecified":
        return "ClearNoon (visibility default)"
    return f"{weather_label}/{time_label}"


def _location_tuple(location) -> tuple[float, float, float]:
    return float(location.x), float(location.y), float(location.z)


def build_lane_segments(carla_map, spacing: float = 2.0) -> list[dict]:
    """Sample driving-lane boundaries from CARLA's live OpenDRIVE map."""
    grouped: dict[tuple[int, int, int], list] = defaultdict(list)
    for waypoint in carla_map.generate_waypoints(spacing):
        if str(waypoint.lane_type).lower().find("driving") < 0:
            continue
        grouped[(waypoint.road_id, waypoint.section_id, waypoint.lane_id)].append(waypoint)

    segments: list[dict] = []
    for (_, _, lane_id), waypoints in grouped.items():
        waypoints.sort(key=lambda wp: wp.s)
        previous = None
        for index, waypoint in enumerate(waypoints):
            if previous is None:
                previous = waypoint
                continue
            a = previous.transform.location
            b = waypoint.transform.location
            if a.distance(b) > spacing * 2.75:
                previous = waypoint
                continue
            for side, center_boundary in (("left", True), ("right", False)):
                endpoints = []
                for wp in (previous, waypoint):
                    yaw = math.radians(wp.transform.rotation.yaw)
                    left_x, left_y = -math.sin(yaw), math.cos(yaw)
                    sign = 1.0 if side == "left" else -1.0
                    loc = wp.transform.location
                    endpoints.append((
                        loc.x + sign * left_x * wp.lane_width * 0.5,
                        loc.y + sign * left_y * wp.lane_width * 0.5,
                        loc.z + 0.10,
                    ))
                segments.append({
                    "a": endpoints[0],
                    "b": endpoints[1],
                    "kind": "center" if center_boundary and abs(lane_id) == 1 else "lane",
                    "dash_index": index,
                })
            previous = waypoint
    return segments


def camera_intrinsics(width: int, height: int, fov: float) -> np.ndarray:
    focal = width / (2.0 * math.tan(math.radians(fov) / 2.0))
    return np.array([[focal, 0.0, width / 2.0], [0.0, focal, height / 2.0], [0.0, 0.0, 1.0]])


def project(location: tuple[float, float, float], inverse_matrix: np.ndarray, k: np.ndarray):
    world = np.array([location[0], location[1], location[2], 1.0])
    camera = inverse_matrix.dot(world)
    carla_camera = np.array([camera[1], -camera[2], camera[0]])
    if carla_camera[2] <= 0.25:
        return None
    pixel = k.dot(carla_camera)
    return float(pixel[0] / pixel[2]), float(pixel[1] / pixel[2]), float(carla_camera[2])


def _font(size: int, bold: bool = False):
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ]
    for candidate in candidates:
        if os.path.isfile(candidate):
            return ImageFont.truetype(candidate, size=size)
    return ImageFont.load_default()


def _text(draw, xy, value, font, fill, anchor=None):
    draw.text(xy, str(value), font=font, fill=fill, anchor=anchor)


def draw_lane_overlay(draw, lane_segments, inverse_matrix, k, width, height, center_line: str):
    for segment in lane_segments:
        if segment["kind"] == "center" and center_line == "broken" and segment["dash_index"] % 4 >= 2:
            continue
        a = project(segment["a"], inverse_matrix, k)
        b = project(segment["b"], inverse_matrix, k)
        if a is None or b is None:
            continue
        if max(a[2], b[2]) > 150.0:
            continue
        margin = 60
        if not (-margin <= a[0] <= width + margin and -margin <= a[1] <= height + margin):
            continue
        if not (-margin <= b[0] <= width + margin and -margin <= b[1] <= height + margin):
            continue
        if segment["kind"] == "center":
            color, line_width = (255, 197, 54, 235), 5
        else:
            color, line_width = (245, 248, 252, 220), 4
        draw.line([(a[0], a[1]), (b[0], b[1])], fill=color, width=line_width)


def actor_description(actor, meta: dict, sut_role: str) -> str:
    role = actor.attributes.get("role_name") or actor.type_id.rsplit(".", 1)[-1]
    scene = meta.get("scene") or {}
    if role == sut_role:
        sut = scene.get("sut") or {}
        return f"{role}  |  SUT · {sut.get('maneuver', 'vehicle')}"
    for npc in scene.get("npcs") or []:
        if npc.get("id") == role:
            behavior = (npc.get("behavior") or {}).get("block") or "NPC"
            return f"{role}  |  {npc.get('position', 'NPC')} · {behavior}"
    return role


def draw_actor_labels(draw, world, camera, meta, args, k):
    inverse = np.array(camera.get_transform().get_inverse_matrix())
    for actor in world.get_actors():
        if not actor.is_alive or not actor.type_id.startswith(("vehicle.", "walker.")):
            continue
        try:
            vertices = actor.bounding_box.get_world_vertices(actor.get_transform())
        except RuntimeError:
            continue
        points = [project(_location_tuple(vertex), inverse, k) for vertex in vertices]
        points = [point for point in points if point is not None and point[2] < 140.0]
        if len(points) < 4:
            continue
        left = max(0, min(point[0] for point in points))
        top = max(0, min(point[1] for point in points))
        right = min(args.width - 1, max(point[0] for point in points))
        bottom = min(args.height - 1, max(point[1] for point in points))
        if right <= left or bottom <= top:
            continue
        role = actor.attributes.get("role_name") or ""
        color = (45, 204, 211, 255) if role == args.actor_role else (255, 162, 55, 255)
        draw.rounded_rectangle((left, top, right, bottom), radius=5, outline=color, width=4)
        label = actor_description(actor, meta, args.actor_role)
        font = _font(22, bold=True)
        box = draw.textbbox((0, 0), label, font=font)
        label_width = box[2] - box[0] + 20
        label_top = max(4, top - 35)
        draw.rounded_rectangle((left, label_top, left + label_width, label_top + 30), radius=6, fill=(15, 24, 36, 225))
        _text(draw, (left + 10, label_top + 3), label, font, color)


def _road_summary(meta: dict) -> str:
    road = meta.get("road") or {}
    lanes = road.get("lanes") or {}
    forward = lanes.get("forward", "?") if isinstance(lanes, dict) else lanes
    backward = lanes.get("backward", "?") if isinstance(lanes, dict) else lanes
    return (
        f"{str(road.get('topology', 'unknown')).upper()}  ·  "
        f"{str(road.get('type', 'unknown')).upper()}  ·  "
        f"LANES {forward} forward / {backward} backward  ·  "
        f"{str(road.get('center_line', 'unknown')).upper()} CENTER LINE"
    )


def _scene_summary(meta: dict) -> str:
    scene = meta.get("scene") or {}
    sut = scene.get("sut") or {}
    npc_bits = []
    for npc in scene.get("npcs") or []:
        block = (npc.get("behavior") or {}).get("block", "unknown")
        npc_bits.append(f"{npc.get('id', 'npc')} {npc.get('position', '?')} / {block}")
    collision = scene.get("collision") or {}
    collision_text = f"collision {collision.get('a', '?')} ↔ {collision.get('b', '?')}"
    return f"SUT {sut.get('id', 'ego')} / {sut.get('maneuver', '?')}  ·  {'; '.join(npc_bits)}  ·  {collision_text}"


def draw_hud(draw, meta: dict, args, visual_weather: str, frame_number: int):
    road = meta.get("road") or {}
    environment = ((meta.get("scene") or {}).get("environment") or {})
    panel_x, panel_y = 34, 32
    panel_w, panel_h = min(args.width - 68, 1440), 158
    draw.rounded_rectangle(
        (panel_x, panel_y, panel_x + panel_w, panel_y + panel_h),
        radius=16,
        fill=(12, 20, 31, 222),
        outline=(105, 127, 151, 180),
        width=2,
    )
    _text(draw, (panel_x + 24, panel_y + 16), "EXECUTABLE METAMODEL VIEW", _font(23, True), (105, 224, 230, 255))
    _text(draw, (panel_x + 24, panel_y + 50), _road_summary(meta), _font(25, True), (246, 248, 252, 255))
    _text(draw, (panel_x + 24, panel_y + 87), _scene_summary(meta), _font(21), (224, 231, 239, 255))
    source_weather = environment.get("weather") or "unknown"
    source_time = environment.get("time_of_day") or "unknown"
    source_friction = environment.get("friction_scale")
    environment_note = f"ENVIRONMENT · source weather={source_weather} · time={source_time}"
    if source_friction is not None:
        environment_note += f" · friction={source_friction}"
    environment_note += f" · rendered {visual_weather}"
    if str(source_weather).lower() == "unknown" or str(source_time).lower() == "unknown":
        environment_note += " (unspecified values are not source evidence)"
    _text(draw, (panel_x + 24, panel_y + 119), environment_note, _font(18, True), (154, 222, 178, 255))

    legend = "cyan: SUT    orange: NPC    yellow: center line    white: lane boundary"
    legend_box = draw.textbbox((0, 0), legend, font=_font(18))
    legend_w = legend_box[2] - legend_box[0] + 30
    legend_y = args.height - 54
    draw.rounded_rectangle((34, legend_y, 34 + legend_w, legend_y + 34), radius=8, fill=(12, 20, 31, 205))
    _text(draw, (49, legend_y + 6), legend, _font(18), (230, 235, 242, 255))
    frame_label = f"CARLA 0.9.16 · live frame {frame_number}"
    _text(draw, (args.width - 34, args.height - 30), frame_label, _font(17), (235, 239, 245, 235), anchor="rs")


def draw_minimap(draw, lane_segments, world, followed, args):
    size = min(310, int(args.height * 0.29))
    x0, y0 = args.width - size - 34, 32
    draw.rounded_rectangle((x0, y0, x0 + size, y0 + size), radius=16, fill=(12, 20, 31, 220), outline=(105, 127, 151, 180), width=2)
    _text(draw, (x0 + 16, y0 + 12), "LIVE ROAD TOPOLOGY", _font(18, True), (235, 240, 247, 255))
    center = followed.get_transform().location
    yaw = math.radians(followed.get_transform().rotation.yaw)
    scale = size / 105.0

    def local_xy(point):
        dx, dy = point[0] - center.x, point[1] - center.y
        forward = dx * math.cos(yaw) + dy * math.sin(yaw)
        lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
        return x0 + size * 0.5 + lateral * scale, y0 + size * 0.76 - forward * scale

    for segment in lane_segments:
        a, b = local_xy(segment["a"]), local_xy(segment["b"])
        if not all(x0 + 8 <= p[0] <= x0 + size - 8 and y0 + 40 <= p[1] <= y0 + size - 8 for p in (a, b)):
            continue
        color = (255, 197, 54, 205) if segment["kind"] == "center" else (205, 215, 226, 170)
        draw.line((a, b), fill=color, width=2)
    for actor in world.get_actors():
        if not actor.is_alive or not actor.type_id.startswith(("vehicle.", "walker.")):
            continue
        point = local_xy(_location_tuple(actor.get_transform().location))
        if x0 + 8 <= point[0] <= x0 + size - 8 and y0 + 40 <= point[1] <= y0 + size - 8:
            role = actor.attributes.get("role_name") or ""
            color = (45, 204, 211, 255) if role == args.actor_role else (255, 162, 55, 255)
            radius = 7 if role == args.actor_role else 6
            draw.ellipse((point[0] - radius, point[1] - radius, point[0] + radius, point[1] + radius), fill=color)


def make_camera(world, actor, carla, args, meta):
    blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
    environment = ((meta.get("scene") or {}).get("environment") or {})
    is_night = str(environment.get("time_of_day") or "").lower() in {"night", "nighttime"}
    attributes = {
        "image_size_x": str(args.width),
        "image_size_y": str(args.height),
        "fov": str(args.fov),
        "sensor_tick": str(1.0 / max(args.fps, 1.0)),
        "exposure_mode": "manual",
        "iso": "400.0" if is_night else "100.0",
        "shutter_speed": "60.0" if is_night else "120.0",
        "fstop": "2.8" if is_night else "5.6",
        "gamma": "2.3" if is_night else "2.2",
        "exposure_compensation": "0.0",
        "bloom_intensity": "0.05",
        "lens_flare_intensity": "0.0",
        "motion_blur_intensity": "0.0",
    }
    for key, value in attributes.items():
        if blueprint.has_attribute(key):
            blueprint.set_attribute(key, value)
    transform = carla.Transform(
        carla.Location(x=args.camera_x, y=args.camera_y, z=args.camera_z),
        carla.Rotation(pitch=args.camera_pitch, yaw=args.camera_yaw, roll=0.0),
    )
    return world.spawn_actor(blueprint, transform, attach_to=actor)


def main() -> int:
    args = parse_args()
    meta = load_meta(args.meta)
    carla = _load_carla()
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world, followed = wait_for_actor(client, args.actor_role)
    print(f"paper recorder attached: map={world.get_map().name}, role={args.actor_role}", flush=True)
    visual_weather = configure_weather(carla, world, meta)
    lane_segments = build_lane_segments(world.get_map())
    print(f"paper recorder lane segments={len(lane_segments)}, weather={visual_weather}", flush=True)
    camera = make_camera(world, followed, carla, args, meta)
    image_queue: queue.Queue = queue.Queue(maxsize=2)

    def on_image(image):
        if image_queue.full():
            try:
                image_queue.get_nowait()
            except queue.Empty:
                pass
        image_queue.put(image)

    camera.listen(on_image)
    output_dir = Path(args.save_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    k = camera_intrinsics(args.width, args.height, args.fov)
    saved = 0
    observed = 0
    try:
        while True:
            try:
                image = image_queue.get(timeout=1.0)
            except queue.Empty:
                if not followed.is_alive:
                    break
                continue
            if observed % max(1, args.save_every) != 0:
                observed += 1
                continue
            array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
            frame = Image.fromarray(array[:, :, [2, 1, 0]], mode="RGB").convert("RGBA")
            overlay = Image.new("RGBA", frame.size, (0, 0, 0, 0))
            draw = ImageDraw.Draw(overlay, "RGBA")
            inverse = np.array(camera.get_transform().get_inverse_matrix())
            center_line = str((meta.get("road") or {}).get("center_line") or "solid").lower()
            draw_lane_overlay(draw, lane_segments, inverse, k, args.width, args.height, center_line)
            draw_actor_labels(draw, world, camera, meta, args, k)
            draw_hud(draw, meta, args, visual_weather, image.frame)
            draw_minimap(draw, lane_segments, world, followed, args)
            Image.alpha_composite(frame, overlay).convert("RGB").save(
                output_dir / f"frame_{saved:06d}.jpg", quality=94, subsampling=0
            )
            saved += 1
            observed += 1
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        camera.destroy()
    print(f"paper recorder saved {saved} frames", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
