#!/usr/bin/env python3
"""Source-reviewed physical reconstructions and separate PCLA counterfactuals.

No dynamics are inferred from collision labels. Each supported case below has
an explicitly authored sequence, source review, and documented assumptions.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import xmlschema
from tools.carla_remote import CarlaRemoteClient, RemoteCfg
from tools.osc_blocks import _env_for_compile
from tools.replay_scene_tools import write_replay_xosc
from tools.source_runtime_evidence import evaluate, select_contact_episodes

STATIONARY_CASES = {"007", "065", "122", "440", "448"}
SUPPORTED = STATIONARY_CASES | {"162", "009", "444", "235", "638", "140", "263", "165", "203", "262", "533",
             "180", "639", "249", "008", "040", "491", "323", "113", "038", "274", "097", "013", "273", "157", "635", "032",
             "135", "035", "120", "259", "330", "021", "055", "293", "435", "144"}
TURN_CASES = {"135", "035", "120", "259", "330", "021", "055", "293", "435", "144"}


def save(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")


class RoadFrame:
    """Evaluate the original map's approach; local y is positive to the left."""
    def __init__(self, path, lane=-1):
        root = ET.parse(path).getroot()
        self.roads = {int(r.get("id")): r for r in root.findall("road")}
        roads = [r for r in root.findall("road") if r.get("junction") == "-1" and float(r.get("length")) > 60]
        self.road = roads[0]
        self.s0 = min(70.0, float(self.road.get("length")) * .5)
        section = self.road.find("lanes/laneSection")
        widths = [float(section.find("right/lane[@id='%d']/width" % k).get("a")) for k in range(-1, lane - 1, -1)]
        self.offset = -sum(widths) + widths[-1] / 2

    def point(self, t, x, y=0.0, h=None):
        s = self.s0 + x
        geometry = max((g for g in self.road.findall("planView/geometry") if float(g.get("s")) <= s), key=lambda g: float(g.get("s")))
        distance = s - float(geometry.get("s"))
        rx, ry, heading = (float(geometry.get(k)) for k in ("x", "y", "hdg"))
        arc = geometry.find("arc")
        if arc is not None:
            curvature = float(arc.get("curvature"))
            rx += (math.sin(heading + curvature * distance) - math.sin(heading)) / curvature
            ry -= (math.cos(heading + curvature * distance) - math.cos(heading)) / curvature
            heading += curvature * distance
        elif geometry.find("line") is not None:
            rx += distance * math.cos(heading)
            ry += distance * math.sin(heading)
        else:
            raise ValueError("Source approach needs a CARLA waypoint extraction for spiral geometry")
        offset = self.offset + y
        return {"t": float(t), "x": rx - offset * math.sin(heading), "y": ry + offset * math.cos(heading),
                "z": .5, "h": math.degrees(heading) + (h or 0)}

    def road_point(self, timestamp, road_id, lane_id, s):
        road = self.roads[road_id]
        g = max((g for g in road.findall("planView/geometry") if float(g.get("s")) <= s + 1e-6), key=lambda g: float(g.get("s")))
        length = max(0.0, s - float(g.get("s")))
        x, y, h = (float(g.get(k)) for k in ("x", "y", "hdg"))
        arc, spiral = g.find("arc"), g.find("spiral")
        if arc is not None:
            k = float(arc.get("curvature"))
            x += (math.sin(h + k * length) - math.sin(h)) / k
            y -= (math.cos(h + k * length) - math.cos(h)) / k
            h += k * length
        elif spiral is not None:
            k0 = float(spiral.get("curvStart")); rate = (float(spiral.get("curvEnd")) - k0) / float(g.get("length"))
            steps = max(1, math.ceil(length / .1)); ds = length / steps
            for i in range(steps):
                u = (i + .5) * ds; theta = h + k0 * u + .5 * rate * u * u
                x += ds * math.cos(theta); y += ds * math.sin(theta)
            h += k0 * length + .5 * rate * length * length
        else:
            x += length * math.cos(h); y += length * math.sin(h)
        sign = 1 if lane_id > 0 else -1
        side = "left" if lane_id > 0 else "right"
        widths = [float(road.find("lanes/laneSection/%s/lane[@id='%d']/width" % (side, sign * i)).get("a")) for i in range(1, abs(lane_id) + 1)]
        offset = sign * (sum(widths) - widths[-1] / 2)
        return {"t": float(timestamp), "x": x - offset * math.sin(h), "y": y + offset * math.cos(h),
                "z": .5, "h": math.degrees(h) + (180 if lane_id > 0 else 0)}


def authored_plan(prefix, frame=None):
    """Points are (seconds, forward metres, left metres, relative heading deg)."""
    if prefix in TURN_CASES:
        from tools.source_turn_plans import author_turn_plan
        return author_turn_plan(prefix, frame)
    hold = [(0, 0, 0, 0), (18, 0, 0, 0)]
    hero = {"points": hold, "blueprint": "vehicle.lincoln.mkz_2020", "role": "reported AV"}
    other = {"blueprint": "vehicle.toyota.prius", "role": "reported collision partner"}
    actors = {"hero": hero, "other": other}
    pairs = [["hero", "other"]]
    notes = []
    if prefix in STATIONARY_CASES:
        speed, gap = (2.2352, 8) if prefix == "448" else (4, 12)
        other["points"] = [(0, -gap, 0, 0), (1, -gap, 0, 0), (10, -gap + 9 * speed, 0, 0), (18, -gap + 9 * speed, 0, 0)]
        other["max_speed"] = speed + .2
        actors["queue_lead"] = {"points": [(0, 5.5, 0, 0), (18, 5.5, 0, 0)], "blueprint": "vehicle.toyota.prius", "role": "representative stopped traffic ahead of AV"}
        notes += ["The queue lead is contextual traffic; its 5.5 m center spacing is assumed. Traffic-signal assets are not validated."]
        if prefix == "448":
            notes += ["The reported 5 mph is the rear car's reference speed; impact speed is measured from CARLA."]
    elif prefix == "162":
        hero["points"] = [(0, 0, -1.8, 0), (14, 0, -1.8, 0)]
        other.update(blueprint="vehicle.volkswagen.t2_2021", points=[(0, -12, .5, 0), (6, 0, .5, 0), (8, 4, -.8, 0), (10, 8, -.8, 0), (14, 16, .5, 0)])
        notes += ["Parked AV is in the roadside shoulder; its off-driving-lane position is intentional.", "Stock van and AV body approximate the reported mirror contact; no custom mirror collider is claimed."]
        notes += ["The report says the car approached from the left; the same-direction passing path is a reconstruction assumption."]
    elif prefix == "009":
        other.update(blueprint="vehicle.tesla.model3", points=[(0, -15, 0, 0), (1, -15, 0, 0), (4, -8, 2.4, 0), (6, -3, 1.4, 0), (8, 2, 1.4, 0), (10, 7, 2.5, 0), (14, 18, 3, 0)])
        notes += ["Episode begins with the AV already stopped after the earlier accident; that earlier accident is outside this report's reconstruction."]
    elif prefix == "444":
        hero["points"] = [(0, -2, 0, 0), (2, 0, 0, 0), (18, 0, 0, 0)]
        other["points"] = [(0, 20, 3, 180), (4, 8, 2.6, 180), (6, 2, 1.4, 180), (8, -4, 1.4, 180), (10, -10, 2.8, 180), (14, -22, 3.5, 180)]
        notes += ["Reported mirror-to-sensor contact is approximated using stock vehicle collision hulls."]
    elif prefix == "235":
        hero["points"] = [(0, -2, 0, 0), (2, 0, 0, 0), (18, 0, 0, 0)]
        other.update(blueprint="vehicle.carlamotors.carlacola", points=[(0, 24, 3.7, 180), (6, 6, 3.1, 180), (8.5, -1.5, 2.8, 180), (10.5, -7.5, .2, 180), (13, -15, .2, 180), (18, -30, 3.5, 180)])
        other["release_on_collision"] = "true"
        notes += ["Stock truck hull approximates the truck-side-to-rear-sensor contact."]
        notes += ["Truck coasts with neutral inputs after first contact; this assumed response avoids continuously driving into the AV after the reported scrape."]
    elif prefix == "638":
        hero["points"] = [(0, 0, 0, 0), (28, 0, 0, 0)]
        other["points"] = [(0, -2.5, -2.4, 0), (5, -.265, -2.4, 0), (10, 1.97, -.95, 0), (18, 5.55, -.95, 0), (23, 7.78, -2.5, 0), (28, 10.02, -2.5, 0)]
        other["max_speed"] = .6
        notes += ["Longitudinal reference speed is the reported 1 mph (0.44704 m/s); actual contact speed is measured from CARLA.", "Stock hull contact is an approximation of protruding mirror/sensor geometry."]
    elif prefix == "140":
        actors.pop("other")
        actors["middle"] = {"points": [(0, -5.6, 0, 0), (18, -5.6, 0, 0)], "blueprint": "vehicle.toyota.prius", "hold_brake": 0, "role": "passenger car pushed into AV"}
        actors["pickup"] = {"points": [(0, -22, 0, 0), (2, -20, 0, 0), (6, -2, 0, 0), (10, 16, 0, 0), (18, 16, 0, 0)], "blueprint": "vehicle.tesla.cybertruck", "role": "pickup first strikes passenger car"}
        actors["queue_lead"] = {"points": [(0, 6.5, 0, 0), (18, 6.5, 0, 0)], "blueprint": "vehicle.toyota.prius", "role": "representative uncleared traffic ahead"}
        pairs = [["pickup", "middle"], ["middle", "hero"]]
        notes += ["Middle car is coasting with zero commanded throttle and brake; only physical impact may push it forward.", "No second impact is imposed by a trajectory or velocity assignment."]
    elif prefix == "263":
        hero["points"] = [(0, 0, 0, 0), (2, 0, 0, 0), (4, 2, 0, 0), (14, 22, 0, 0), (18, 30, 0, 0)]
        actors["lead"] = {"points": [(0, 7, 0, 0), (2, 7, 0, 0), (4, 12, 0, 0), (18, 40, 0, 0)], "blueprint": "vehicle.toyota.prius", "role": "departing traffic followed by AV"}
        other.update(blueprint="vehicle.tesla.cybertruck", points=[(0, -12, 0, 0), (5, -12, 0, 0), (7, -7, 0, 0), (16, 38, 0, 0), (18, 38, 0, 0)])
    elif prefix == "165":
        hero.update(blueprint="vehicle.mercedes.coupe_2020", points=[(0, -12, 0, 0), (3, 0, 0, 0), (4, 2, 0, 0), (5, 2.5, 0, 0), (18, 2.5, 0, 0)])
        actors["lead"] = {"points": [(0, -1, 0, 0), (2, 7, 0, 0), (3, 8.2, 0, 0), (18, 8.2, 0, 0)], "blueprint": "vehicle.toyota.prius", "role": "lead traffic vehicle abruptly stops"}
        other["points"] = [(0, -28, 0, 0), (2, -20, 0, 0), (7, 4, 0, 0), (10, 10, 0, 0), (18, 10, 0, 0)]
        notes += ["The source states the Mercedes was manually driven throughout; the reconstruction does not claim an autonomous Mercedes collision."]
    elif prefix in {"203", "262"}:
        hero["points"] = [(0, -12, 0, 0), (2, -5, 0, 0), (4, -.5, 0, 0), (6, 1, 0, 0), (18, 1, 0, 0)]
        speed = 4.4704 if prefix == "262" else 4.0
        other["points"] = [(0, -21, 0, 0), (2, -21 + speed * 2, 0, 0), (8, -21 + speed * 8, 0, 0), (18, -21 + speed * 8, 0, 0)]
        if prefix == "203":
            other["blueprint"] = "vehicle.mercedes.sprinter"
        else:
            notes += ["Reported following-vehicle speed is approximately 10 mph; reference speed is 4.4704 m/s, and actual impact speed is recorded separately."]
            other["max_speed"] = 4.7
    elif prefix == "533":
        hero["points"] = [(0, 0, 0, 0), (2, 0, 0, 0), (5, 2.1, 0, 0), (6, 2.3, 0, 0), (18, 2.3, 0, 0)]
        other["points"] = [(0, -11, 0, 0), (5, -11, 0, 0), (7, -7.87, 0, 0), (12, 7.78, 0, 0), (18, 7.78, 0, 0)]
        notes += ["Reported approximately 7 mph is the rear car's reference cruise speed, 3.12928 m/s; it first accelerates from rest."]
        other["max_speed"] = 3.4
    elif prefix in {"180", "639"}:
        other["points"] = [(0, -14, -.65, 0), (1, -14, -.65, 0), (6, 3.5, -.65, 0), (18, 3.5, -.65, 0)]
        actors["oncoming"] = {"points": [(0, 35, 3.5, 180), (18, -37, 3.5, 180)], "blueprint": "vehicle.toyota.prius", "role": "opposing traffic to which AV yields before left turn"}
        notes += ["AV waits to turn left; no turn is performed before the reported rear contact."]
        if prefix == "639":
            notes += ["The report specifies right-rear contact but not the BMW approach. The rear approach and lateral offset here are one possible reconstruction, not reported facts."]
    elif prefix == "249":
        other["points"] = [(0, -9, 0, 0), (3, -9, 0, 0), (5, -4, 0, 0), (8, 8, 0, 0), (18, 8, 0, 0)]
        actors["through_traffic"] = {"points": [(0, 1, -3.5, 0), (1, 1, -3.5, 0), (4, 9, -3.5, 0), (12, 41, -3.5, 0), (18, 41, -3.5, 0)], "blueprint": "vehicle.toyota.prius", "role": "adjacent through traffic departs while turn lane remains stopped"}
        notes += ["Phase sequence is represented by traffic motion; the generated map has no validated arrow-signal mesh."]
    elif prefix in {"008", "040"}:
        other.update(blueprint="vehicle.mitsubishi.fusorosa" if prefix == "008" else "vehicle.carlamotors.carlacola",
                     points=[(0, -18, 0, 0), (2, -18, 0, 0), (5, -11, 3.3, 0), (8, -3, 1.7 if prefix == "040" else 2.15, 0), (10, 3, 1.7 if prefix == "040" else 2.8, 0), (12, 9, 3.5, 0), (18, 27, 3.5, 0)])
        notes += ["The passing vehicle intentionally enters the opposing lane in the same travel direction; this is the reported maneuver."]
        if prefix == "040":
            notes += ["Report's unilluminated malfunctioning signal is represented as a stopped approach; an illuminated normal signal is not substituted."]
    elif prefix == "491":
        hero["points"] = [(0, 0, 0, 0), (14, 7, 0, 0), (18, 7, 0, 0)]
        other.update(blueprint="vehicle.carlamotors.carlacola", points=[(0, 26, 3.6, 180), (5, 11, 3.4, 180), (8, 2, 2.5, 180), (10, -4, .2, 180), (12, -10, .2, 180), (14, -16, 3.4, 180), (18, -28, 3.5, 180)])
        notes += ["AV's reported manual creep is modeled at an assumed 0.5 m/s; truck hull approximates the rear radar contact."]
    elif prefix == "323":
        hero["points"] = [(0, 0, 0, 0), (18, 0, 0, 0)]
        actors["following"] = {"points": [(0, -7, 0, 0), (18, -7, 0, 0)], "blueprint": "vehicle.toyota.prius", "role": "car queued behind AV, also overtaken"}
        other["points"] = [(0, -23, 0, 0), (3, -15, 3.5, 0), (6, -4, 3.3, 0), (8, 3, 1.2, 0), (10, 10.5, 1.2, 0), (12, 18, 3.5, 0), (18, 40, 3.5, 0)]
        notes += ["Overtaker travels in the AV's direction while occupying the opposing lane. AV's pre-turn wait is an assumed low-speed state because the source gives no precise speed."]
    elif prefix == "113":
        hero["points"] = [(0, -50, 0, 0), (1, -50, 0, 0), (7, -26, 0, 0), (13, 4, 0, 0), (20, 39, 0, 0), (22, 42, 0, 0), (32, 42, 0, 0)]
        other.update(blueprint="vehicle.tesla.model3", points=[(0, -62, 0, 0), (1, -62, 0, 0), (7, -28, 0, 0), (8, -23, 0, 0), (9, -22, 0, 0), (11, -22, 0, 0), (18, 12, 0, 0), (23, 28, .6, 0), (25, 28, .6, 0), (28, 43, .6, 0), (32, 43, .6, 0)])
        pairs = [["hero", "other"], ["hero", "other"]]
        notes += ["Two separate impacts require a measured gap without sensor contact, moving AV at first impact, and stopped AV at the second. Repeated callbacks from a single impact do not satisfy this requirement."]
    elif prefix == "038":
        hero["points"] = [(0, 0, 0, 0), (2, 8, 0, 0), (4, 16, 1.9, 0), (10, 40, 1.9, 0), (14, 56, 0, 0), (18, 65, 0, 0)]
        other.update(blueprint="vehicle.carlamotors.carlacola", points=[(0, -12, 3.5, 0), (2, -4, 3.5, 0), (10, 44, 3.5, 0), (18, 70, 3.5, 0)])
        actors["cyclist"] = {"type": "cyclist", "points": [(0, 5, -1.1, 0), (18, 50, -1.1, 0)], "blueprint": "vehicle.bh.crossbike", "role": "cyclist on AV's right causing left nudge"}
        notes += ["The AV nudges left; the truck remains in its lane. Cyclist speed and clearances are assumed."]
    elif prefix == "274":
        hero["points"] = [(0, 0, 0, 0), (2, 10, 0, 0), (6, 30, 0, 0), (8, 38, 0, 0), (10, 42, 0, 0), (18, 42, 0, 0)]
        other["points"] = [(0, -8, -3.5, 0), (2, -2, -3.5, 0), (4, 12, 3.5, 0), (6, 30, 3.5, 0), (7, 35.5, 1.2, 0), (8, 40, 1.2, 0), (10, 46, 0, 0), (18, 65, 0, 0)]
    elif prefix == "097":
        hero["points"] = [(0, -16, 0, 0), (4, 0, 0, 0), (6, 8, 0, 0), (8, 12, 0, 0), (18, 12, 0, 0)]
        other["points"] = [(0, 4, -3.5, 0), (3, 4, -3.5, 0), (5, 7, -.7, 0), (8, 17, 0, 0), (18, 40, 0, 0)]
        notes += ["Parked-to-moving transition is explicit; no initial adjacent-lane cruise substitutes for the parking exit."]
    elif prefix == "013":
        hero["points"] = [(0, 0, 0, 0), (10, 50, 0, 0), (18, 70, 0, 0)]
        other.update(blueprint="vehicle.dodge.charger_2020", points=[(0, 4, -3.5, 0), (3, 17, -3.5, 0), (4, 20, -1.3, 0), (6, 30, -1.3, 0), (10, 48, -3.5, 0), (18, 70, -3.5, 0)])
        actors["cyclist"] = {"type": "cyclist", "points": [(0, 22, -3.5, 0), (18, 58, -3.5, 0)], "blueprint": "vehicle.bh.crossbike", "role": "cyclist ahead of Camaro in lane 5"}
        notes += ["AV uses lane 4 and overtaking car starts in lane 5. Stock Dodge coupe approximates the Camaro asset."]
    elif prefix == "273":
        hero["points"] = [(0, 0, 0, 0), (4, 8, 3.5, 0), (8, 22, 3.5, 0), (10, 24, 1.2, 0), (18, 24, 1.2, 0)]
        actors["blocked_truck"] = {"points": [(0, 15, 0, 0), (18, 15, 0, 0)], "blueprint": "vehicle.carlamotors.carlacola", "role": "stationary truck being passed, not collision partner"}
        other.update(blueprint="vehicle.dodge.charger_2020", points=[(0, -16, 6.5, 0), (4, 0, 6.5, 0), (8, 16, 5.5, 0), (10, 24, 4.2, 0), (12, 28, 1.0, 0), (14, 32, 1.0, 0), (18, 44, 3.5, 0)])
        other["points"] = [other["points"][0]] + [(t + .5, x, y, h) for t, x, y, h in other["points"]]
        notes += ["The eighteen-wheeler is approximated by CARLA's shorter rigid truck; vehicle-length mismatch is retained as an asset limitation."]
    elif prefix == "157":
        hero["points"] = [(0, -4, 0, 0), (3, 4, 0, 0), (5, 4, 0, 0), (7, 8, 2.4, 0), (10, 14, 1.5, 0), (13, 21, 2.5, 0), (18, 32, 0, 0)]
        other["points"] = [(0, 12, 0, 0), (18, 12, 0, 0)]
        notes += ["AV's stop and later manually driven pass are separate phases; contact with the double-parked car occurs during the pass."]
    elif prefix == "635":
        hero["points"] = [(0, -20, 0, 0), (8, 20, 0, 0), (18, 55, 0, 0)]
        other["points"] = [(0, -18, -3.5, 0), (3, -3, -3.5, 0), (5, 7, -1.4, 0), (7, 17, -1.4, 0), (10, 32, 0, 0), (18, 63, 0, 0)]
        notes += ["Right-side approach is one reconstruction consistent with reported right-front contact; the source does not fully specify the Toyota's trajectory."]
    elif prefix == "032":
        hero["points"] = [(i * .5, i * .5 * 7.15264, 0, 0) for i in range(19)]
        hero["max_speed"] = 7.3
        other["points"] = [(0, 35, -1.6, 0), (10, 35, -1.6, 0)]
        notes += ["AV reference speed is the reported 16 mph; curved reference is sampled every 0.5 s rather than replacing the arc with a straight chord."]
        notes += ["Original curved OpenDRIVE segment supplies the right-hand curvature. Stock hulls approximate AV sensor pod and parked-car mirror."]
    else:
        raise ValueError(prefix)
    return actors, pairs, notes


def prepare(cid, output, mode):
    prefix = cid.split("_")[0]
    output.mkdir(parents=True, exist_ok=False)
    map_path, scenario_path = output / "map.xodr", output / "scenario.xosc"
    shutil.copy2(ROOT / "data/compiled/opendrive_seed" / (cid + ".xodr"), map_path)
    original = json.loads((ROOT / "data/seeds/scene_seed" / (cid + ".json")).read_text())
    review = next(c for c in json.loads((ROOT / "data/eval/baseline_42_source_review.json").read_text())["cases"] if c["id"] == prefix)
    frame = RoadFrame(map_path, lane={"140": -2, "038": -2, "013": -4, "273": -2}.get(prefix, -1))
    if prefix in STATIONARY_CASES:
        junction = frame.road.find("./link/successor[@elementType='junction']") is not None
        length = float(frame.road.get("length"))
        frame.s0 = length - 17 if junction else min(80, length * .5)
    if prefix in {"113", "635"} | TURN_CASES:
        frame.s0 = float(frame.road.get("length")) + 30
    actors, pairs, notes = authored_plan(prefix, frame)
    traces = {name: {"type": a.get("type", "vehicle"), "controller": "scripted", "role": a["role"],
                     "trace": a.get("world_trace") or [frame.point(*p) for p in a["points"]]} for name, a in actors.items()}
    duration = max(a["trace"][-1]["t"] for a in traces.values())
    if mode == "pcla":
        # demo.py removes trajectory actuation and assigns external_control.
        # Preserve moving/turning geometry and give the agent a destination
        # beyond the episode. A 1-4 m creep trace is not a usable navigation
        # route: InterFuser otherwise targets a point behind it after passing it.
        route = traces["hero"]["trace"]
        stationary_route = max(math.hypot(p["x"] - route[0]["x"], p["y"] - route[0]["y"]) for p in route) < .1
        last = dict(route[-1])
        heading = math.radians(last["h"])
        extension = max(120.0, 6 * duration)
        goal = dict(last, t=duration, x=last["x"] + extension * math.cos(heading), y=last["y"] + extension * math.sin(heading))
        route = [dict(route[0])] if stationary_route else [dict(p, t=p["t"] * .95) for p in route]
        traces["hero"]["trace"] = route + [goal]
        notes.append("PCLA-only navigation continues %.1f m beyond the reconstruction endpoint, preserving preceding turn geometry. Navigation timing is normalized inside the episode; the hero trajectory action is removed before autonomous execution." % extension)
    document = {"metadata": {"name": cid}, "environment": _env_for_compile(original["scene"].get("environment", {})),
                "duration_s": duration, "actors": traces, "road_generation": {"xodr_path": "map.xodr"}}
    write_replay_xosc(document, scenario_path)
    tree = ET.parse(scenario_path)
    for name, a in actors.items():
        vehicle = tree.find("./Entities/ScenarioObject[@name='%s']/Vehicle" % name)
        if vehicle is not None:
            vehicle.set("name", a["blueprint"])
        else:
            import copy
            action = tree.find(".//Private[@entityRef='hero']/PrivateAction/ControllerAction/..")
            tree.find(".//Private[@entityRef='%s']" % name).append(copy.deepcopy(action))
        properties = tree.find(".//Private[@entityRef='%s']//AssignControllerAction/Controller/Properties" % name)
        properties.find("Property[@name='module']").set("value", "physics_trajectory_control")
        for prop in ("hold_brake", "max_speed", "release_on_collision"):
            if prop in a:
                ET.SubElement(properties, "Property", name=prop, value=str(a[prop]))
    tree.find("./CatalogLocations").clear()
    tree.write(scenario_path, encoding="utf-8", xml_declaration=True)
    xmlschema.XMLSchema(str(ROOT / "xsd/OpenSCENARIO.xsd")).validate(scenario_path)
    xmlschema.XMLSchema(str(ROOT / "xsd/OpenDRIVE_1.5M.xsd")).validate(map_path)
    assumptions = ["Numerical positions, timing and unreported speeds are authored reconstruction assumptions, not recovered measurements.",
                   "Original representative OpenDRIVE geometry retained; no surveyed street geometry is claimed.",
                   "Scripted participants use apply_control with physics enabled. Collisions may change the planned trajectory."] + notes
    manifest = {"case_id": cid, "execution_mode": "pcla_autonomous_sut" if mode == "pcla" else "scripted_physical_reconstruction",
                "source_review": review, "expected_ordered_collision_pairs": pairs,
                "role_mapping": {name: a["role"] for name, a in actors.items()}, "actor_plans": actors,
                "reconstruction_assumptions": assumptions,
                "autonomous_result_note": "PCLA counterfactuals may avoid or alter the source collision; these are not reconstruction failures or scripted ADS successes."}
    save(output / "reconstruction.json", manifest)
    save(output / "trace_reference.json", document)
    save(output / "compile_validation.json", {"xodr_xsd": "pass", "xosc_xsd": "pass"})
    return manifest


def inspect_run(run, manifest):
    events = [json.loads(line) for line in (run / "events.jsonl").read_text().splitlines()]
    contacts = [e for e in events if e.get("event_type") == "collision" and e.get("payload", {}).get("source") == "carla_collision_sensor"]
    pair_first = {}
    for event in contacts:
        key = "/".join(sorted(event["payload"].get("actors", [])))
        pair_first.setdefault(key, event)
    selected = select_contact_episodes(contacts, manifest["expected_ordered_collision_pairs"])
    times = [e["simulation_time"] if e else None for e in selected]
    contact_order = all(t is not None for t in times) and all(a < b for a, b in zip(times, times[1:]))
    feedback = json.loads((run / "sim_feedback.json").read_text())
    result = {"physical_pair_first_contacts": pair_first, "ordered_contact_times": times, "ordered_physical_contacts_pass": contact_order,
              "trajectory_feedback": feedback.get("summary"), "video_visual_review": "pending", "source_sequence_review": "pending",
              "execution_mode": manifest["execution_mode"], "accepted": False,
              "sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in run.iterdir() if p.is_file() and p.suffix in {".mp4", ".xosc", ".xodr", ".log", ".jsonl", ".gz"}}}
    result["measured_source_sequence"] = evaluate(run, manifest["case_id"].split("_")[0], manifest["expected_ordered_collision_pairs"])
    save(run / "source_validation.json", result)
    return result


class SourceCaseClient(CarlaRemoteClient):
    def _runner_env(self):
        return super()._runner_env() + " CONTINUE_AFTER_COLLISION=1 REPLAY_CONTROL=npc SUT_ROUTE_MODE=full"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cases", nargs="+", required=True, choices=sorted(SUPPORTED))
    ap.add_argument("--attempt", required=True)
    ap.add_argument("--mode", choices=["reconstruction", "pcla"], required=True)
    ap.add_argument("--run", action="store_true")
    ap.add_argument("--wall-timeout", type=int, default=110, help="Wall-clock run timeout; does not change authored scenario duration.")
    args = ap.parse_args()
    if Path(args.attempt).name != args.attempt or args.attempt in {".", ".."}:
        ap.error("--attempt must be a directory name")
    frozen = json.loads((ROOT / "data/eval/baseline_42.json").read_text())["cases"]
    index = {c["case_id"].split("_")[0]: c["case_id"] for c in frozen}
    base = ROOT / "outputs/validated_42_20260917"
    client = SourceCaseClient(replace(RemoteCfg.from_env(), keep_remote_runs=True, compact_artifacts=True)) if args.run else None
    results = []
    for prefix in args.cases:
        cid = index[prefix]
        output = base / "cases" / cid / args.attempt
        manifest = prepare(cid, output, args.mode)
        result = {"case_id": cid, "mode": args.mode, "attempt": args.attempt}
        if client:
            try:
                client.run_scenario(output / "map.xodr", output / "scenario.xosc", name="source" + prefix + "_" + args.attempt,
                                    sut_actor="hero" if args.mode == "pcla" else "", rgb_actor_role="hero", pcla_agent="if_if",
                                    max_seconds=args.wall_timeout, out_dir=output / "run")
                validation = inspect_run(output / "run", manifest)
                result["physical_pairs_pass"] = validation["ordered_physical_contacts_pass"]
            except Exception as exc:
                result["error"] = str(exc)
                save(output / "run_failure.json", result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
        results.append(result)
        save(base / ("source_" + args.attempt + ".json"), results)
    return int(any("error" in r for r in results))


if __name__ == "__main__":
    raise SystemExit(main())
