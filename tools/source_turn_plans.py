"""Explicit junction reconstructions, with all unreported geometry assumed."""
import math


def arc(t0, speed, cx, cy, radius, a0, a1, step=5):
    count = max(2, math.ceil(abs(a1 - a0) / step))
    direction = 1 if a1 > a0 else -1
    points = []
    for i in range(count + 1):
        a = a0 + (a1 - a0) * i / count
        t = t0 + radius * abs(math.radians(a - a0)) / speed
        points.append((t, cx + radius * math.cos(math.radians(a)), cy + radius * math.sin(math.radians(a)), a + direction * 90))
    return points


def road_segment(frame, road, lane, s0, s1, t0, speed):
    n = max(1, math.ceil(abs(s1 - s0) / 2))
    return [frame.road_point(t0 + abs(s1 - s0) * i / n / speed, road, lane, s0 + (s1 - s0) * i / n) for i in range(n + 1)]


def join(*segments):
    points = []
    for segment in segments:
        for point in segment:
            if points and abs(points[-1]["t"] - point["t"]) < 1e-5:
                points[-1] = point
            else:
                points.append(point)
    return points


def author_turn_plan(prefix, frame):
    hero = {"blueprint": "vehicle.lincoln.mkz_2020", "role": "reported AV"}
    other = {"blueprint": "vehicle.toyota.prius", "role": "reported collision partner"}
    actors = {"hero": hero, "other": other}
    pairs = [["hero", "other"]]
    notes = ["Junction positions and turn radii are reconstruction assumptions on the retained representative map; they are not surveyed curb/stop-line locations.",
             "Stock vehicle assets approximate manufacturer models and protruding sensor hardware."]
    length = float(frame.road.get("length"))
    if prefix == "330":
        connector = float(frame.roads[100].get("length"))
        for a, gap, speed in [(hero, 8, 3), (other, 30, 5)]:
            t1 = gap / speed; t2 = t1 + connector / speed
            a["world_trace"] = join(road_segment(frame, 1, -1, length - gap, length, 0, speed),
                                    road_segment(frame, 100, -1, 0, connector, t1, speed),
                                    road_segment(frame, 2, 1, length, length - 20, t2, speed))
        notes += ["Both vehicles follow the actual OpenDRIVE right-turn connector; the following car closes on the AV during its turn."]
    elif prefix == "035":
        connector = float(frame.roads[103].get("length"))
        hero["world_trace"] = join(road_segment(frame, 1, -1, length - 5, length, 0, 3),
                                    road_segment(frame, 103, -1, 0, connector, 5 / 3, 3),
                                    road_segment(frame, 4, 1, length, length - 12, (5 + connector) / 3, 3))
        other["points"] = [(0, -5, 72.7, -90), (12, -5, 12.7, -90), (23, -5, -42.3, -90)]
        notes += ["Crossing car proceeds through the conflict as the AV follows the left-turn connector. Signal violation is semantic intent; signal assets are not validated."]
    elif prefix in {"021", "120"}:
        hero["points"] = [(0, -32, 0, 0), (20, -32, 0, 0)]
        if prefix == "120":
            hero["points"] = [(0, -32, 0, 0), (10, -32, 0, 0), (14, -30.5, -.4, 0), (22, -30.5, -.4, 0)]
            other["blueprint"] = "vehicle.mitsubishi.fusorosa"
            notes += ["Stock bus hull approximates the protruding triple-bike rack; the rack itself is absent and no exact rack collision is claimed."]
        end_y = 0 if prefix == "021" else 1.0
        curve = arc(6, 4, -30, 30 + end_y, 30, 0, -90)
        other["points"] = [(0, 0, 54 + end_y, -90)] + curve + [(curve[-1][0] + 4, -46, end_y, 180)]
    elif prefix == "435":
        hero["points"] = [(0, -36, 0, 0), (3, -32, -.6, 0), (40, -32, -.6, 0)]
        curve = arc(4, 2.2352, -30, -27.2, 30, 0, 90)
        end = curve[-1][0]
        other["points"] = [(0, 0, -36.14, 90)] + curve + [(end + 2.5, -35.59, 1.5, 180), (end + 5, -41.18, -.5, 180), (end + 8, -47.88, 1.8, 180)]
        other["max_speed"] = 2.5
        for name, x, y in [("parked_right", -47, -4.2), ("parked_left", -50, 6.0)]:
            actors[name] = {"blueprint": "vehicle.toyota.prius", "role": "parked traffic constraining the narrow road",
                            "points": [(0, x, y, 0), (40, x, y, 0)]}
    elif prefix == "055":
        hero["points"] = [(0, -30, 0, 0), (12, 33.6, 0, 0), (22, 70, 0, 0)]
        other["blueprint"] = "vehicle.ford.ambulance"
        curve = arc(2, 4.5, 30, -30, 28.4, 180, 90)
        end = curve[-1][0]
        other["points"] = [(0, 1.6, -39, 90)] + curve + [(end + 5, 52.5, -1.6, 0), (22, 70, -3.5, 0)]
        notes += ["Ambulance emergency lights and siren remain off, as described in the PDF."]
    elif prefix == "259":
        hero["points"] = [(0, -32, 0, 0), (20, -32, 0, 0)]
        other.update(blueprint="vehicle.carlamotors.carlacola", points=[(0, -43, 3.5, 0), (1, -43, 3.5, 0), (4, -31, 3.5, 0), (6, -26, .2, -50), (9, -24, -10, -80), (15, -6, -28, -90), (20, -6, -42, -90)])
        actors["pedestrian"] = {"type": "pedestrian", "role": "pedestrian crossing the AV's intended right turn",
                                 "points": [(0, -27, -9, 0), (12, -15, -9, 0), (20, -7, -9, 0)]}
    elif prefix == "293":
        hero["points"] = [(0, -25, 0, 0), (4, -5, 0, 0), (6, 3, 0, 0), (8, 6, 0, 0), (20, 6, 0, 0)]
        curve = arc(3, 3.5, 5, 8.8, 10, 180, 270)
        end = curve[-1][0]
        other["points"] = [(0, -5, 19.3, -90)] + curve + [(end + 4, 19, -1.2, 0), (20, 45, -1.2, 0)]
        notes += ["The PDF does not specify the Nissan's complete incoming path. A left turn from the crossing approach is one chosen reconstruction of its entry into the AV right of way."]
    elif prefix == "144":
        curve = arc(5, 2.5, -10, -8, 8, 90, 0)
        end = curve[-1][0]
        hero["points"] = [(0, -10, 0, 0)] + curve + [(end + 4, -2, -18, -90), (20, -2, -25, -90)]
        other.update(type="cyclist", blueprint="vehicle.bh.crossbike", points=[(0, -24, -4, 0), (12, 6, -4, 0), (20, 20, -4, 0)])
        actors["pedestrian"] = {"type": "pedestrian", "role": "pedestrian clears the side-road crosswalk before AV starts turning",
                                 "points": [(0, -5, -8, 0), (4, 6, -8, 0), (20, 6, -8, 0)]}
        notes += ["AV's manual takeover is represented by the physical reconstruction controller; this is not an autonomous PCLA collision.",
                  "Crosswalk placement inside the representative junction is assumed; cyclist is a physical CARLA bicycle, not a passenger car."]
    elif prefix == "135":
        actors.pop("other")
        hero["points"] = [(0, 1.1, -4.7, 90), (20, 1.1, -4.7, 90)]
        actors["suv"] = {"blueprint": "vehicle.audi.etron", "role": "red-running SUV that spins after crossing-car impact",
                          "points": [(0, -1.75, 30, -90), (10, -1.75, -30, -90), (20, -1.75, -30, -90)], "release_on_collision": "true"}
        actors["crossing_car"] = {"blueprint": "vehicle.tesla.model3", "role": "eastbound passenger car in the first collision",
                                   "points": [(0, -42, 0, 0), (12, 66, 0, 0), (20, 66, 0, 0)], "release_on_collision": "true"}
        pairs = [["suv", "crossing_car"], ["suv", "hero"]]
        notes += ["The report identifies a Toyota SUV but does not state its model. The stock Audi e-tron is a substitute, not an exact Toyota asset.",
                  "The AV stop position is assumed immediately behind the crossing-lane edge; the source does not provide a surveyed stop-line position."]
        notes += ["SUV releases throttle, brake and steering after its first physical vehicle contact. Any spin and secondary impact must result from CARLA physics; no post-impact pose, yaw or velocity is imposed."]
        notes += ["Crossing car also coasts after the first impact. Neutral driver inputs after impact are an explicit reconstruction assumption, preventing an unreported continued throttle push."]
    else:
        raise ValueError(prefix)
    return actors, pairs, notes
