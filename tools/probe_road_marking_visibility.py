"""Capture native RGB before/after OpenDRIVE debug markings on an idle server.

Rendering diagnostic only, not a scenario run. Restores weather/settings and
destroys only its own camera; does not reload the world or move vehicles.
"""
import argparse
import json
import queue
from pathlib import Path

import carla


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    client = carla.Client('127.0.0.1', 2000); client.set_timeout(20)
    world = client.get_world()
    original_settings, original_weather = world.get_settings(), world.get_weather()
    if list(world.get_actors().filter('vehicle.*')):
        raise RuntimeError('Probe requires an idle world without scenario vehicles')
    settings = world.get_settings(); settings.synchronous_mode = True
    settings.fixed_delta_seconds = .05; settings.no_rendering_mode = False
    camera = None
    try:
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        road_map = world.get_map()
        waypoints = [w for w in road_map.generate_waypoints(5) if not w.is_junction and w.lane_id < 0]
        wp = waypoints[len(waypoints)//2]
        transform = wp.transform
        location = transform.location+carla.Location(z=35)
        view = carla.Transform(location, carla.Rotation(pitch=-90, yaw=transform.rotation.yaw))
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', '960'); bp.set_attribute('image_size_y', '720')
        bp.set_attribute('fov', '80')
        bp.set_attribute('sensor_tick', '0.05')
        bp.set_attribute('exposure_mode', 'manual'); bp.set_attribute('iso', '100')
        bp.set_attribute('shutter_speed', '100'); bp.set_attribute('fstop', '8')
        camera = world.spawn_actor(bp, view)
        frames = queue.Queue(); camera.listen(frames.put)

        def capture(name):
            image = None
            for _ in range(20):
                world.tick()
                try:
                    while True:
                        image = frames.get(timeout=.1)
                except queue.Empty:
                    pass
            if image is None:
                raise RuntimeError('No native camera RGB frame')
            image.save_to_disk(str(args.output/name))
            return image.frame

        before = capture('before.png')
        from road_markings import draw_opendrive_markings
        count = draw_opendrive_markings(world)
        after = capture('after.png')
        (args.output/'probe.json').write_text(json.dumps({
            'kind': 'idle-world rendering diagnostic; not source-scenario evidence',
            'map': road_map.name, 'camera': dict(camera.attributes),
            'waypoint': {'road': wp.road_id, 'lane': wp.lane_id, 's': wp.s},
            'before_frame': before, 'after_frame': after, 'marking_segments': count,
            'temporary_weather': 'ClearNoon', 'original_sun_altitude': original_weather.sun_altitude_angle}, indent=2))
        print('probe frames:', before, after, 'marking segments:', count)
    finally:
        if camera is not None:
            camera.stop(); camera.destroy()
        world.set_weather(original_weather)
        world.apply_settings(original_settings)


if __name__ == '__main__':
    main()
