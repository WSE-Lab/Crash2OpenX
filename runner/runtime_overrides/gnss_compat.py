"""Calibrate native map GNSS into the GPS convention used by PCLA routes.

CARLA 0.9.16's generated tmerc maps can report increasing latitude for +Y,
whereas the leaderboard route converter assumes decreasing latitude for +Y.
Only measured GNSS is transformed; no vehicle ground truth enters the agent.
"""
import json
import math
import xml.etree.ElementTree as ET

import numpy as np


class MapGnssCalibration:
    def __init__(self, sample, bounds, latitude_reference=0.0, longitude_reference=0.0):
        self.lat_ref = latitude_reference
        self.lon_ref = longitude_reference
        xmin, xmax, ymin, ymax = bounds
        self.center = np.array([(xmin + xmax) / 2, (ymin + ymax) / 2])
        self.origin_gps = np.array(sample(*self.center), dtype=float)
        step = 50.0
        columns = []
        for axis in (np.array([step, 0.0]), np.array([0.0, step])):
            columns.append((np.array(sample(*(self.center + axis))) -
                            np.array(sample(*(self.center - axis)))) / (2 * step))
        self.native_jacobian = np.column_stack(columns)
        self.inverse = np.linalg.inv(self.native_jacobian)
        if not np.isfinite(self.inverse).all():
            raise ValueError("Nonfinite native GNSS calibration")
        errors = []
        for x in (xmin, (xmin + xmax) / 2, xmax):
            for y in (ymin, (ymin + ymax) / 2, ymax):
                errors.append(float(np.linalg.norm(self.world_xy(sample(x, y)) - [x, y])))
        self.max_probe_error_m = max(errors)
        if self.max_probe_error_m > .02:
            raise ValueError("Map GNSS calibration is nonlinear over route bounds: %.6f m" % self.max_probe_error_m)

    def world_xy(self, gps):
        return self.center + self.inverse.dot(np.asarray(gps[:2]) - self.origin_gps)

    def route_gps(self, gps):
        x, y = self.world_xy(gps)
        radius = 6378137.0
        scale = math.cos(math.radians(self.lat_ref))
        mx = scale * math.radians(self.lon_ref) * radius + x
        my = scale * radius * math.log(math.tan(math.radians(90 + self.lat_ref) / 2)) - y
        lon = math.degrees(mx / (radius * scale))
        lat = math.degrees(2 * math.atan(math.exp(my / (radius * scale)))) - 90
        return np.array([lat, lon])


def install_gnss_compat(agent, world, route_points):
    import carla

    if not route_points:
        raise ValueError("GNSS calibration requires a route for validation bounds")
    road_map = world.get_map()
    georef = ET.fromstring(road_map.to_opendrive()).findtext("header/geoReference", "")
    parameters = dict(part.split("=", 1) for part in georef.split() if "=" in part)

    def sample(x, y):
        geo = road_map.transform_to_geolocation(carla.Location(x=float(x), y=float(y)))
        return [geo.latitude, geo.longitude]

    bounds = (min(p.x for p in route_points) - 30, max(p.x for p in route_points) + 30,
              min(p.y for p in route_points) - 30, max(p.y for p in route_points) + 30)
    calibration = MapGnssCalibration(sample, bounds,
                                     float(parameters.get("+lat_0", 42)),
                                     float(parameters.get("+lon_0", 2)))
    tags = [s["id"] for s in agent.sensors() if s["type"] == "sensor.other.gnss"]
    original_get_data = agent.sensor_interface.get_data
    calls = 0

    def get_data():
        nonlocal calls
        data = original_get_data()
        for tag in tags:
            frame, raw = data[tag]
            converted = np.array(raw, copy=True)
            converted[:2] = calibration.route_gps(raw)
            data[tag] = (frame, converted)
            if calls % 20 == 0:
                print("PCLA_GNSS_ADAPTER_SAMPLE=" + json.dumps({
                    "frame": int(frame), "tag": tag,
                    "native_gps": np.asarray(raw).tolist(),
                    "route_convention_gps": converted.tolist(),
                }), flush=True)
        calls += 1
        return data

    agent.sensor_interface.get_data = get_data
    print("PCLA_GNSS_CALIBRATION=" + json.dumps({
        "georeference": georef, "validation_bounds_xy": bounds,
        "native_gps_jacobian_per_m": calibration.native_jacobian.tolist(),
        "max_probe_error_m": calibration.max_probe_error_m,
        "source": "Actual GNSS samples; fixed calibration from map coordinate transforms only",
        "noise_preserved": True, "vehicle_ground_truth_used": False,
    }), flush=True)
    return calibration
