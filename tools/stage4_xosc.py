#!/usr/bin/env python3
"""OpenSCENARIO entity and environment helpers for generated-road replay.

The current accident replay pipeline writes XOSC from trace scenes in
`tools/replay_scene_tools.py`. This module only contains reusable mechanical
helpers for CARLA-compatible inline entities and environment actions.
"""

from __future__ import annotations

import datetime as dt

import scenariogeneration.xosc as xosc


_CAR_BB = xosc.BoundingBox(2.1, 4.5, 1.8, 1.5, 0.0, 0.9)
_CAR_FA = xosc.Axle(0.5, 0.6, 1.8, 3.1, 0.3)
_CAR_RA = xosc.Axle(0.0, 0.6, 1.8, 0.0, 0.3)

_BUS_BB = xosc.BoundingBox(2.6, 12.0, 3.5, 5.0, 0.0, 1.75)
_BUS_FA = xosc.Axle(0.5, 0.8, 2.2, 9.0, 0.4)
_BUS_RA = xosc.Axle(0.0, 0.8, 2.2, 0.0, 0.4)

_PED_BB = xosc.BoundingBox(0.5, 0.5, 1.8, 0.0, 0.0, 0.9)


def _make_vehicle(name: str, entity_type: str, role: str = "simulation") -> xosc.Vehicle:
    """Create a CARLA-compatible inline vehicle object."""
    if entity_type == "bus":
        bb, fa, ra = _BUS_BB, _BUS_FA, _BUS_RA
        category = xosc.VehicleCategory.bus
        max_speed, max_accel, max_decel = 30.0, 10.0, 5.0
    else:
        bb, fa, ra = _CAR_BB, _CAR_FA, _CAR_RA
        category = xosc.VehicleCategory.car
        max_speed, max_accel, max_decel = 69.444, 200.0, 10.0

    vehicle = xosc.Vehicle(name, category, bb, fa, ra, max_speed, max_accel, max_decel)
    vehicle.add_property("type", role)
    return vehicle


def _make_pedestrian(name: str) -> xosc.Pedestrian:
    """Create a CARLA-compatible inline pedestrian object."""
    return xosc.Pedestrian(
        name,
        mass=75.0,
        category=xosc.PedestrianCategory.pedestrian,
        boundingbox=_PED_BB,
        model="walker.pedestrian.0001",
    )


def _build_env_action(scene: dict) -> xosc.EnvironmentAction:
    """Create an OpenSCENARIO EnvironmentAction from a facts/trace scene."""
    env = scene.get("environment") or {}

    time_text = str(env.get("time_of_day", "")).strip()
    try:
        time_value = dt.datetime.fromisoformat(time_text) if time_text else dt.datetime(2020, 10, 23, 12, 0, 0)
    except Exception:
        time_value = dt.datetime(2020, 10, 23, 12, 0, 0)

    time_of_day = xosc.TimeOfDay(
        False,
        time_value.year,
        time_value.month,
        time_value.day,
        time_value.hour,
        time_value.minute,
        time_value.second,
    )

    weather_text = str(env.get("weather", "")).lower()
    if "rain" in weather_text:
        cloud = xosc.CloudState.rainy
        precipitation = xosc.Precipitation(xosc.PrecipitationType.rain, 0.8)
    elif "snow" in weather_text:
        cloud = xosc.CloudState.overcast
        precipitation = xosc.Precipitation(xosc.PrecipitationType.snow, 0.6)
    elif "fog" in weather_text:
        cloud = xosc.CloudState.overcast
        precipitation = xosc.Precipitation(xosc.PrecipitationType.dry, 0.0)
    elif "cloud" in weather_text or "overcast" in weather_text:
        cloud = xosc.CloudState.cloudy
        precipitation = xosc.Precipitation(xosc.PrecipitationType.dry, 0.0)
    else:
        cloud = xosc.CloudState.free
        precipitation = xosc.Precipitation(xosc.PrecipitationType.dry, 0.0)

    night = time_value.hour < 6 or time_value.hour >= 19
    sun = xosc.Sun(0.0 if night else 0.85, 0.0, -0.3 if night else 1.31)
    weather = xosc.Weather(
        cloudstate=cloud,
        sun=sun,
        fog=xosc.Fog(100000.0),
        precipitation=precipitation,
    )
    road_condition = xosc.RoadCondition(float(env.get("friction_scale", 1.0)))
    environment = xosc.Environment(
        "Environment1",
        timeofday=time_of_day,
        weather=weather,
        roadcondition=road_condition,
    )
    return xosc.EnvironmentAction(environment)
