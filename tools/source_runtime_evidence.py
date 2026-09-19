"""Evaluate source-relevant motion from measured CARLA states, not XOSC plans."""
import json
import math
from pathlib import Path


def planar_speed(state):
    return math.hypot(state.get("vx", 0), state.get("vy", 0))


def select_contact_episodes(contacts, expected_pairs, separation_seconds=.75):
    """Select successive contact episodes for a repeated required actor pair."""
    episodes, last, used = {}, {}, {}
    for event in sorted(contacts, key=lambda e: e["simulation_time"]):
        pair = tuple(sorted(event["payload"].get("actors", [])))
        timestamp = event["simulation_time"]
        if pair not in last or timestamp - last[pair] > separation_seconds:
            episodes.setdefault(pair, []).append(event)
        last[pair] = timestamp
    selected = []
    for pair in expected_pairs:
        pair = tuple(sorted(pair))
        index = used.get(pair, 0)
        candidates = episodes.get(pair, [])
        selected.append(candidates[index] if index < len(candidates) else None)
        used[pair] = index + 1
    return selected


def evaluate(run_dir, case_prefix, expected_pairs):
    run_dir = Path(run_dir)
    rows = [json.loads(line) for line in (run_dir / "sim_trace_raw.jsonl").read_text().splitlines()]
    events = [json.loads(line) for line in (run_dir / "events.jsonl").read_text().splitlines()]
    contacts = [e for e in events if e.get("event_type") == "collision" and e.get("payload", {}).get("source") == "carla_collision_sensor"]
    first = {}
    for event in contacts:
        first.setdefault(tuple(sorted(event["payload"].get("actors", []))), event)
    required = select_contact_episodes(contacts, expected_pairs)
    checks = {"required_physical_pairs": all(required)}
    times = [event["simulation_time"] for event in required if event]
    checks["required_contact_order"] = len(times) == len(required) and all(a < b for a, b in zip(times, times[1:]))
    expected = {tuple(sorted(pair)) for pair in expected_pairs}
    checks["no_unreported_collision_pairs_through_required_sequence"] = not any(
        tuple(sorted(event["payload"].get("actors", []))) not in expected
        for event in contacts if not times or event["simulation_time"] <= max(times) + .1)
    if not rows:
        return {"checks": checks, "pass": False, "error": "No measured trajectory"}
    start = rows[0]["simulation_time"]

    def at(timestamp):
        return min(rows, key=lambda r: abs(r["simulation_time"] - timestamp))["actors"]

    def speeds(actor, lo, hi):
        return [planar_speed(r["actors"][actor]) for r in rows if lo <= r["simulation_time"] <= hi and actor in r["actors"]]

    hero_contact = next((e for e in required if e and "hero" in e["payload"].get("actors", [])), None)
    metrics = {"contact_times_s": times, "episode_start_s": start}
    if hero_contact:
        t = hero_contact["simulation_time"]
        states = at(t - .05)
        hero = states["hero"]
        partner = next(name for name in hero_contact["payload"]["actors"] if name != "hero")
        other = states[partner]
        angle = math.radians(hero["yaw"])
        dx, dy = other["x"] - hero["x"], other["y"] - hero["y"]
        heading_difference = abs((other["yaw"] - hero["yaw"] + 180) % 360 - 180)
        metrics.update({"hero_speed_before_contact_mps": planar_speed(hero), "partner_speed_before_contact_mps": planar_speed(other),
                        "partner_relative_forward_m": dx * math.cos(angle) + dy * math.sin(angle),
                        "partner_relative_left_m": dx * math.sin(angle) - dy * math.cos(angle),
                        "heading_difference_deg": heading_difference,
                        "hero_max_planar_speed_before_contact_mps": max(speeds("hero", start + .5, t - .15) or [0])})
        stationary = {"007", "065", "122", "440", "448", "162", "009", "140", "638", "180", "639", "249", "008", "040", "323"}
        if case_prefix in stationary:
            checks["av_stationary_before_contact"] = metrics["hero_max_planar_speed_before_contact_mps"] < .2
        if case_prefix in {"444", "235", "491"}:
            checks["opposing_travel_direction"] = heading_difference > 140
            if case_prefix != "491":
                checks["av_stopped_before_pass"] = max(speeds("hero", t - 1, t - .15) or [999]) < .2
            else:
                checks["av_creeping_at_contact"] = .1 < planar_speed(hero) < 1.5
        if case_prefix in {"162", "009", "008", "040", "323", "638"}:
            checks["same_direction_pass"] = heading_difference < 35 and planar_speed(other) > .15
        if case_prefix in {"162", "009", "008", "040", "323", "444", "235", "491"}:
            checks["contact_partner_on_left"] = metrics["partner_relative_left_m"] > 1.0
        if case_prefix == "162":
            # CARLA t2_2021 bounding-box half-length, measured on this runtime.
            # An actor origin is not a contact point: its front can reach the
            # AV's mirror region while its center is still slightly behind.
            front = metrics["partner_relative_forward_m"] + 2.2211 * math.cos(math.radians(heading_difference))
            metrics["partner_front_forward_estimate_m"] = front
            checks["partner_front_reaches_av_front_side_region"] = front > 1
        if case_prefix in {"235", "491"}:
            front = metrics["partner_relative_forward_m"] + 2.6019 * math.cos(math.radians(heading_difference))
            metrics["partner_front_forward_estimate_m"] = front
            checks["passing_truck_front_reaches_av_rear_region"] = front < -1.5
        if case_prefix == "638":
            checks["contact_partner_on_right"] = metrics["partner_relative_left_m"] < -1
            checks["reported_1mph_contact_speed"] = abs(planar_speed(other) - .44704) <= .2
        if case_prefix == "263":
            checks["av_initially_stopped"] = max(speeds("hero", start + .5, start + 1.5) or [999]) < .2
            checks["av_moving_before_rear_impact"] = planar_speed(hero) > .3
        if case_prefix in {"203", "262"}:
            prior = max(speeds("hero", start + 1, t - .5) or [0])
            checks["av_decelerated_before_impact"] = prior > planar_speed(hero) + .5
            checks["av_not_yet_stationary_at_impact"] = planar_speed(hero) > .15
        if case_prefix == "533":
            checks["initial_stop"] = max(speeds("hero", start + .5, start + 1.5) or [999]) < .2
            checks["creep_phase"] = max(speeds("hero", start + 2, t - 1) or [0]) > .3
            checks["second_stop_before_impact"] = max(speeds("hero", t - 1, t - .15) or [999]) < .2
        if case_prefix == "165":
            checks["lead_vehicle_stopped"] = planar_speed(states["lead"]) < .2
            checks["av_stopped_after_moving"] = planar_speed(hero) < .2 and max(speeds("hero", start + 1, t - .5) or [0]) > 1
        if case_prefix == "140" and all(required):
            first_t = required[0]["simulation_time"]
            checks["middle_stationary_before_pickup_impact"] = max(speeds("middle", start + .5, first_t - .15) or [999]) < .2
            checks["middle_moved_between_impacts"] = max(speeds("middle", first_t + .1, t) or [0]) > .2
        if case_prefix == "113" and all(required):
            first_t, second_t = times
            checks["first_impact_with_moving_av"] = planar_speed(at(first_t - .05)["hero"]) > .3
            checks["second_impact_with_stopped_av"] = planar_speed(at(second_t - .05)["hero"]) < .2
            distances = [math.hypot(r["actors"]["hero"]["x"] - r["actors"]["other"]["x"],
                                    r["actors"]["hero"]["y"] - r["actors"]["other"]["y"])
                         for r in rows if first_t + .5 < r["simulation_time"] < second_t - .5]
            checks["vehicles_physically_separated_between_impacts"] = max(distances or [0]) > 5.5
        if case_prefix in {"007", "065", "122", "440", "448", "140", "263", "165", "203", "262", "533", "180", "639", "249"}:
            checks["rear_contact_geometry"] = metrics["partner_relative_forward_m"] < -2
        if case_prefix == "448":
            checks["reported_5mph_contact_speed"] = abs(planar_speed(other) - 2.2352) < .7
        if case_prefix == "262":
            checks["reported_10mph_contact_speed"] = abs(planar_speed(other) - 4.4704) < .7
        if case_prefix == "533":
            checks["reported_7mph_contact_speed"] = abs(planar_speed(other) - 3.12928) < .6
        # Source-specific phases are evaluated on measured motion, not merely
        # on the presence of the expected collision pair.
        initial = rows[0]["actors"]
        def relative(a, b):
            theta = math.radians(a["yaw"])
            dx, dy = b["x"] - a["x"], b["y"] - a["y"]
            return dx * math.cos(theta) + dy * math.sin(theta), dx * math.sin(theta) - dy * math.cos(theta)

        def displacement(name, timestamp):
            return relative(initial["hero"], at(timestamp)[name])

        def yaw_change(name, timestamp):
            return (at(timestamp)[name]["yaw"] - initial[name]["yaw"] + 180) % 360 - 180

        def stopped_for(name, duration, lo, hi):
            began = None
            for row in rows:
                stamp = row["simulation_time"]
                if not lo <= stamp <= hi or name not in row["actors"]:
                    continue
                if planar_speed(row["actors"][name]) < .2:
                    began = stamp if began is None else began
                    if stamp - began >= duration:
                        return True
                else:
                    began = None
            return False

        rel_x, rel_y = metrics["partner_relative_forward_m"], metrics["partner_relative_left_m"]
        if case_prefix in {"038", "013"}:
            checks["cyclist_present_on_av_right"] = "cyclist" in initial and relative(initial["hero"], initial["cyclist"])[1] < -1
        if case_prefix == "038":
            checks["av_nudges_left"] = displacement("hero", t - .1)[1] > .6
            checks["truck_stays_in_initial_lane"] = abs(displacement("other", t - .1)[1] - relative(initial["hero"], initial["other"])[1]) < .4
            checks["truck_contacts_left_rear_region"] = rel_x < -1 and rel_y > 1
        if case_prefix in {"013", "097", "635"}:
            initial_y = relative(initial["hero"], initial[partner])[1]
            checks["partner_initially_on_right"] = initial_y < -1
            checks["partner_moves_left_from_initial_position"] = displacement(partner, t - .1)[1] - initial_y > .6
            checks["right_front_or_side_contact"] = rel_x > 0 and rel_y < -.5
        if case_prefix == "097":
            checks["partner_initially_parked"] = stopped_for(partner, 1, start + .5, start + 3)
            checks["partner_moving_at_contact"] = planar_speed(other) > .5
        if case_prefix == "274":
            ys = [relative(initial["hero"], r["actors"][partner])[1] for r in rows if r["simulation_time"] < t]
            checks["partner_starts_in_right_lane_then_passes_left"] = relative(initial["hero"], initial[partner])[1] < -2 and max(ys or [0]) > 2
            checks["partner_returns_right_before_contact"] = max(ys or [0]) - displacement(partner, t - .1)[1] > .6
            checks["av_brakes_before_contact"] = max(speeds("hero", start + 1, t - .1) or [0]) > planar_speed(hero) + .5
            checks["contact_on_av_front_left"] = rel_x > 0 and rel_y > .5
        if case_prefix == "273":
            checks["blocker_present_and_stationary"] = "blocked_truck" in initial and max(speeds("blocked_truck", start + .5, t - .1) or [999]) < .2
            checks["av_passed_blocker"] = displacement("hero", t - .1)[0] > relative(initial["hero"], initial["blocked_truck"])[0]
            checks["av_stopped_before_pass_contact"] = stopped_for("hero", .5, start + 2, t - .1)
            checks["passing_car_contacts_av_front_left"] = rel_x > 0 and rel_y > 1
        if case_prefix in {"157", "032"}:
            checks["collision_partner_remained_parked"] = max(speeds(partner, start + .5, t - .15) or [999]) < .2
            checks["av_moving_during_pass"] = planar_speed(hero) > .5
            checks["parked_car_on_av_right"] = rel_y < -1
        if case_prefix == "157":
            checks["av_stopped_then_resumed_before_pass"] = stopped_for("hero", .6, start + 1, t - 1)
        if case_prefix == "032":
            checks["av_follows_right_curve"] = yaw_change("hero", t - .1) > 3
            checks["reported_16mph_speed"] = abs(planar_speed(hero) - 7.15264) < 1
        if case_prefix in {"035", "330", "144"}:
            delta = yaw_change("hero", t - .1)
            metrics["av_heading_change_before_contact_deg"] = delta
            checks["av_turns_in_reported_direction"] = delta < -15 if case_prefix == "035" else delta > 15
        if case_prefix == "035":
            front = rel_x + 2.2568 * math.cos(math.radians(heading_difference))
            metrics["partner_front_forward_estimate_m"] = front
            checks["crossing_partner_front_reaches_av_left_rear"] = front < -1 and rel_y > .5
        if case_prefix == "330":
            checks["rear_contact_during_turn"] = rel_x < -2 and planar_speed(hero) > .3
        if case_prefix in {"021", "120", "055", "259", "293", "435"}:
            delta = yaw_change(partner, t - .1)
            metrics["partner_heading_change_before_contact_deg"] = delta
            checks["partner_turns_in_reported_direction"] = delta < -25 if case_prefix in {"293", "435"} else delta > (10 if case_prefix == "259" else 25)
        if case_prefix == "021":
            checks["av_stopped_for_wide_turn"] = max(speeds("hero", start + .5, t - .15) or [999]) < .2
            checks["front_to_front_contact"] = rel_x > 2 and heading_difference > 140
        if case_prefix == "120":
            checks["av_initial_stop_then_creep"] = stopped_for("hero", 2, start + .5, t - 1) and max(speeds("hero", start + 2, t - .1) or [0]) > .2
            checks["bus_is_opposing_at_contact"] = heading_difference > 140
        if case_prefix == "259":
            checks["pedestrian_present_and_moving"] = "pedestrian" in initial and max(speeds("pedestrian", start + .5, t - .1) or [0]) > .5
            checks["av_yields_stationary"] = max(speeds("hero", start + .5, t - .15) or [999]) < .2
            front = rel_x + 2.6019 * math.cos(math.radians(heading_difference))
            metrics["partner_front_forward_estimate_m"] = front
            checks["truck_front_reaches_av_left_front"] = front > 1 and rel_y > .5
        if case_prefix == "055":
            checks["ambulance_merges_at_av_right_rear"] = rel_x < 0 and rel_y < -.5 and heading_difference < 45
        if case_prefix == "293":
            checks["av_brakes_before_turning_car_contact"] = max(speeds("hero", start + 1, t - .5) or [0]) > planar_speed(hero) + .5
            checks["turning_car_ahead_of_av"] = rel_x > 1
        if case_prefix == "435":
            checks["av_nudges_right_and_stops"] = displacement("hero", t - .1)[1] < -.2 and stopped_for("hero", 1, start + 1, t - .1)
            front = rel_x + 2.2568 * math.cos(math.radians(heading_difference))
            metrics["partner_front_forward_estimate_m"] = front
            checks["opposing_car_front_reaches_av_left_rear"] = front < -1.5 and rel_y > 1 and heading_difference > 140
            checks["reported_5mph_contact_speed"] = abs(planar_speed(other) - 2.2352) < .7
        if case_prefix == "144":
            checks["av_waits_before_turn"] = stopped_for("hero", 2, start + .5, t - 1)
            checks["cyclist_is_physical_bicycle"] = "crossbike" in str(other.get("type_id", other.get("blueprint", "")))
            checks["cyclist_contacts_passenger_rear_quarter"] = rel_x < 0 and rel_y < -.5
            checks["pedestrian_clears_before_turn"] = "pedestrian" in initial and displacement("pedestrian", start + 4)[0] > displacement("hero", start + 4)[0] + 5
        if case_prefix == "135" and all(required):
            first_t = required[0]["simulation_time"]
            first_states = at(first_t - .05)
            crossing = first_states["crossing_car"]
            suv = first_states["suv"]
            cross_yaw = math.radians(crossing["yaw"])
            first_forward = ((suv["x"] - crossing["x"]) * math.cos(cross_yaw)
                             + (suv["y"] - crossing["y"]) * math.sin(cross_yaw))
            metrics["first_contact_suv_forward_of_crossing_car_m"] = first_forward
            checks["crossing_car_front_participates_in_first_contact"] = first_forward > 1
            delta = (at(t - .05)["suv"]["yaw"] - at(first_t - .05)["suv"]["yaw"] + 180) % 360 - 180
            metrics["suv_rotation_between_impacts_deg"] = delta
            checks["suv_spins_after_first_collision"] = abs(delta) > 20
            checks["av_stationary_before_secondary_contact"] = max(speeds("hero", start + .5, t - .15) or [999]) < .2
    summary = json.loads((run_dir / "summary.json").read_text())
    feedback = json.loads((run_dir / "sim_feedback.json").read_text()).get("summary", {})
    checks["all_expected_actors_spawned"] = feedback.get("all_expected_actors_spawned") is True
    checks["no_controller_failures"] = not feedback.get("controller_failures")
    checks["completed_without_runtime_exception"] = not str(summary.get("termination_reason", "")).startswith("exception")
    checks["no_actor_fell_below_road"] = all(state.get("z", 0) > -1 for row in rows for state in row["actors"].values())
    return {"checks": checks, "metrics": metrics, "pass": all(checks.values()),
            "limits": "Body-extent and actor-origin geometry tests only approximate contact regions; CARLA collision events do not provide the precise contact point. These checks do not verify real-world dimensions, exact sensor/mirror contact, or missing traffic-signal assets; visual and source review remain necessary."}
