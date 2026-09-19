#!/usr/bin/env python3
"""
Lightweight CARLA visualization client for a headless CARLA server.

This process is separate from the scenario runner. It connects to an already
running CARLA server, observes the current world, and either:

1. spawns an RGB camera sensor on the ego vehicle and displays the stream; or
2. draws a top-down actor view from simulator state, which still works when
   CARLA world settings have no_rendering_mode=True.
"""

import argparse
import json
import math
import os
import queue
import signal
import sys
import time
import xml.etree.ElementTree as ET

_STOP_REQUESTED = False


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scenario_runner"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "PCLA"))

try:
    from carla_compat import ensure_carla_importable
except ImportError:
    from src.carla_compat import ensure_carla_importable

ensure_carla_importable()


def import_runtime_dependencies():
    global carla, np, pygame

    import carla as carla_module
    import numpy as numpy_module
    import pygame as pygame_module

    carla = carla_module
    np = numpy_module
    pygame = pygame_module


def parse_args():
    parser = argparse.ArgumentParser(description="Visualize an already running CARLA simulation.")
    parser.add_argument("--host", default="localhost", help="CARLA server host")
    parser.add_argument("--port", type=int, default=2000, help="CARLA server port")
    parser.add_argument("--timeout", type=float, default=10.0, help="CARLA client timeout in seconds")
    parser.add_argument("--mode", choices=["rgb", "topdown"], default="rgb", help="Visualization backend")
    parser.add_argument("--actor-role", default="hero", help="Attach/follow actor role_name")
    parser.add_argument("--width", type=int, default=1280, help="Viewer width")
    parser.add_argument("--height", type=int, default=720, help="Viewer height")
    parser.add_argument("--fov", type=float, default=90.0, help="RGB camera FOV")
    parser.add_argument("--fps", type=float, default=20.0, help="Viewer refresh limit")
    parser.add_argument("--save-dir", default=None, help="Optional directory for saved frames")
    parser.add_argument("--save-every", type=int, default=1, help="Save every N displayed frames")
    parser.add_argument("--topdown-range", type=float, default=90.0, help="Top-down view span in meters")
    parser.add_argument("--camera-x", type=float, default=-16.0, help="RGB camera relative X offset")
    parser.add_argument("--camera-y", type=float, default=0.0, help="RGB camera relative Y offset")
    parser.add_argument("--camera-z", type=float, default=9.0, help="RGB camera relative Z offset")
    parser.add_argument("--camera-pitch", type=float, default=-28.0, help="RGB camera pitch in degrees")
    parser.add_argument("--camera-yaw", type=float, default=0.0, help="RGB camera yaw in degrees")
    parser.add_argument("--spectator", action="store_true", help="Also move CARLA spectator to the followed actor")
    return parser.parse_args()


def find_actor_by_role(world, role_name):
    actors = world.get_actors()
    for actor in actors.filter("vehicle.*"):
        if actor.attributes.get("role_name") == role_name:
            return actor
    for actor in actors:
        if actor.attributes.get("role_name") == role_name:
            return actor
    return None


def wait_for_actor(world, role_name):
    actor = find_actor_by_role(world, role_name)
    while actor is None and not _STOP_REQUESTED:
        print(f"Waiting for actor with role_name={role_name!r} ...")
        time.sleep(1.0)
        actor = find_actor_by_role(world, role_name)
    if actor is None:
        raise KeyboardInterrupt
    return actor


def make_window(width, height, title):
    pygame.init()
    pygame.display.set_caption(title)
    return pygame.display.set_mode((width, height), pygame.HWSURFACE | pygame.DOUBLEBUF)


def save_frame_if_needed(surface, save_dir, frame_index, save_every):
    if not save_dir or frame_index % max(save_every, 1) != 0:
        return
    os.makedirs(save_dir, exist_ok=True)
    pygame.image.save(surface, os.path.join(save_dir, f"frame_{frame_index:06d}.jpg"))


def pump_quit_events():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        if event.type == pygame.KEYUP and event.key in (pygame.K_ESCAPE, pygame.K_q):
            return True
    return False


def maybe_update_spectator(world, actor):
    transform = actor.get_transform()
    location = transform.location + carla.Location(z=45.0)
    rotation = carla.Rotation(pitch=-90.0, yaw=transform.rotation.yaw, roll=0.0)
    world.get_spectator().set_transform(carla.Transform(location, rotation))


def run_rgb_viewer(client, args):
    # demo.py reloads the map after the recorder starts. An Actor obtained
    # through the previous World proxy can have a stale is_alive snapshot.
    while True:
        if _STOP_REQUESTED:
            raise KeyboardInterrupt
        world = client.get_world()
        actor = find_actor_by_role(world, args.actor_role)
        if actor is not None:
            break
        print(f"Waiting for actor with role_name={args.actor_role!r} ...", flush=True)
        time.sleep(.25)
    settings = world.get_settings()
    if getattr(settings, "no_rendering_mode", False):
        print("WARNING: world.no_rendering_mode=True. RGB camera images may be blank; use --mode topdown.")

    blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
    blueprint.set_attribute("image_size_x", str(args.width))
    blueprint.set_attribute("image_size_y", str(args.height))
    blueprint.set_attribute("fov", str(args.fov))
    blueprint.set_attribute("sensor_tick", str(1.0 / max(args.fps, 1.0)))
    # Generated roads have no street-light assets. Preserve the world's night
    # weather and use a long-exposure observation camera for a legible recording.
    # This only changes the separate recorder, not the PCLA sensor configuration.
    source_night = False
    scenario_file = os.environ.get("SCENARIO")
    if scenario_file and os.path.isfile(scenario_file):
        sun = ET.parse(scenario_file).find(".//Environment/Weather/Sun")
        source_night = sun is not None and float(sun.get("elevation", "0")) < 0
    if source_night or world.get_weather().sun_altitude_angle < 0:
        for key, value in {"exposure_mode": "manual", "iso": "800",
                           "shutter_speed": "60", "fstop": "2.8",
                           "gamma": "2.2", "exposure_compensation": "0.0"}.items():
            if blueprint.has_attribute(key):
                blueprint.set_attribute(key, value)
        print("Night observation camera: manual ISO=800, shutter=1/60, f/2.8; PCLA sensors unchanged", flush=True)

    camera_transform = carla.Transform(
        carla.Location(x=args.camera_x, y=args.camera_y, z=args.camera_z),
        carla.Rotation(pitch=args.camera_pitch, yaw=args.camera_yaw, roll=0.0),
    )
    camera = world.spawn_actor(blueprint, camera_transform, attach_to=actor)
    timestamp_file = None
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
        timestamp_file = open(os.path.join(args.save_dir, "timestamps.jsonl"), "w", encoding="utf-8")
        with open(os.path.join(args.save_dir, "camera.json"), "w", encoding="utf-8") as handle:
            json.dump({"source": "CARLA sensor.camera.rgb", "sensor_id": camera.id,
                       "attached_actor_id": actor.id, "actor_role": args.actor_role,
                       "attributes": dict(camera.attributes),
                       "relative_pose": {"x": args.camera_x, "y": args.camera_y, "z": args.camera_z,
                                         "pitch": args.camera_pitch, "yaw": args.camera_yaw}}, handle, indent=2)
    image_queue = queue.Queue(maxsize=2)

    def on_image(image):
        if image_queue.full():
            try:
                image_queue.get_nowait()
            except queue.Empty:
                pass
        image_queue.put(image)

    camera.listen(on_image)
    print(f"RGB attached actor={actor.id}, camera={camera.id}, world={world.id}", flush=True)
    display = make_window(args.width, args.height, "CARLA RGB Viewer")
    clock = pygame.time.Clock()
    frame_index = 0

    try:
        while not _STOP_REQUESTED:
            if pump_quit_events():
                break
            if args.spectator and actor.is_alive:
                maybe_update_spectator(world, actor)
            try:
                image = image_queue.get(timeout=1.0)
            except queue.Empty:
                if client.get_world().id != world.id or world.get_actor(actor.id) is None:
                    print("Recorded actor removed; ending RGB stream", flush=True)
                    break
                continue

            if frame_index == 0:
                print(f"First RGB image: frame={image.frame}, time={image.timestamp:.3f}", flush=True)

            array = np.frombuffer(image.raw_data, dtype=np.uint8)
            array = array.reshape((image.height, image.width, 4))[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))
            pygame.display.flip()
            save_frame_if_needed(display, args.save_dir, frame_index, args.save_every)
            if timestamp_file and frame_index % max(args.save_every, 1) == 0:
                timestamp_file.write(json.dumps({"image_index": frame_index, "carla_frame": image.frame,
                                                "simulation_time": image.timestamp}) + "\n")
                timestamp_file.flush()
            frame_index += 1
            clock.tick(args.fps)
    finally:
        if timestamp_file:
            timestamp_file.close()
        try:
            camera.stop()
            camera.destroy()
        except RuntimeError:
            pass  # The scenario may already have destroyed the parent actor.
        pygame.quit()


def actor_color(actor):
    if actor.attributes.get("role_name") == "hero":
        return (235, 80, 70)
    if actor.type_id.startswith("vehicle."):
        return (64, 136, 220)
    if actor.type_id.startswith("walker."):
        return (44, 168, 112)
    if actor.type_id.startswith("traffic."):
        return (246, 188, 66)
    return (120, 120, 120)


def world_to_screen(location, center, pixels_per_meter, width, height):
    x = int(width * 0.5 + (location.x - center.x) * pixels_per_meter)
    y = int(height * 0.5 - (location.y - center.y) * pixels_per_meter)
    return x, y


def draw_actor(surface, actor, center, pixels_per_meter, width, height):
    transform = actor.get_transform()
    x, y = world_to_screen(transform.location, center, pixels_per_meter, width, height)
    yaw = math.radians(transform.rotation.yaw)

    if actor.type_id.startswith("vehicle."):
        length = max(10, int(4.5 * pixels_per_meter))
        width_px = max(5, int(2.0 * pixels_per_meter))
        points = [
            (length / 2, 0),
            (-length / 2, -width_px / 2),
            (-length / 2, width_px / 2),
        ]
        rotated = []
        for px, py in points:
            sx = x + px * math.cos(yaw) - py * math.sin(yaw)
            sy = y - (px * math.sin(yaw) + py * math.cos(yaw))
            rotated.append((int(sx), int(sy)))
        pygame.draw.polygon(surface, actor_color(actor), rotated)
        return

    radius = 5 if actor.type_id.startswith("walker.") else 4
    pygame.draw.circle(surface, actor_color(actor), (x, y), radius)


def draw_grid(surface, width, height, pixels_per_meter, step_m=10):
    background = (245, 247, 250)
    grid = (218, 224, 232)
    surface.fill(background)
    step = max(8, int(step_m * pixels_per_meter))
    for x in range(width // 2 % step, width, step):
        pygame.draw.line(surface, grid, (x, 0), (x, height), 1)
    for y in range(height // 2 % step, height, step):
        pygame.draw.line(surface, grid, (0, y), (width, y), 1)


def run_topdown_viewer(world, args):
    followed = wait_for_actor(world, args.actor_role)
    display = make_window(args.width, args.height, "CARLA Top-down Viewer")
    clock = pygame.time.Clock()
    frame_index = 0
    font = pygame.font.Font(None, 24)

    while not _STOP_REQUESTED:
        if pump_quit_events():
            break
        if not followed.is_alive:
            followed = wait_for_actor(world, args.actor_role)
        if args.spectator:
            maybe_update_spectator(world, followed)

        center = followed.get_transform().location
        pixels_per_meter = min(args.width, args.height) / max(args.topdown_range, 1.0)
        draw_grid(display, args.width, args.height, pixels_per_meter)

        actors = world.get_actors()
        for actor in actors:
            if actor.type_id.startswith(("vehicle.", "walker.", "traffic.traffic_light")):
                try:
                    draw_actor(display, actor, center, pixels_per_meter, args.width, args.height)
                except RuntimeError:
                    pass

        label = font.render(
            f"mode=topdown role={args.actor_role} frame={world.get_snapshot().frame}",
            True,
            (30, 35, 42),
        )
        display.blit(label, (12, 10))
        pygame.display.flip()
        save_frame_if_needed(display, args.save_dir, frame_index, args.save_every)
        frame_index += 1
        clock.tick(args.fps)

    pygame.quit()


def main():
    args = parse_args()
    import_runtime_dependencies()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    print(f"Connected to CARLA {args.host}:{args.port}, map={world.get_map().name}, mode={args.mode}")

    if args.mode == "rgb":
        run_rgb_viewer(client, args)
    else:
        run_topdown_viewer(world, args)


if __name__ == "__main__":
    def stop_on_signal(signum, frame):
        global _STOP_REQUESTED
        # Finish the current image/timestamp pair before leaving the loop.
        _STOP_REQUESTED = True

    signal.signal(signal.SIGTERM, stop_on_signal)
    try:
        main()
    except KeyboardInterrupt:
        print("CARLA recorder stopped; sensor and timestamp file closed", flush=True)
