#!/usr/bin/env python3
"""Bounded physical calibration of the source-135 two-impact sequence.

Changes only pre-impact positions/speeds and stock passenger-car asset.
Both moving cars release actuation after their first physical contact.
"""
import copy
import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools import source_turn_plans
from tools.validate_source_reconstruction import prepare, inspect_run, SourceCaseClient
from tools.carla_remote import RemoteCfg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-grid", action="store_true", help="Try stock SUVs with different inertial properties; preserve the original grid.")
    parser.add_argument("--timing-grid", action="store_true", help="Refine first-contact timing with the stationary AV outside crossing traffic.")
    parser.add_argument("--momentum-grid", action="store_true", help="Try explicitly documented higher approach speeds using unchanged stock physics.")
    parser.add_argument("--front-contact-grid", action="store_true", help="Target the reported passenger-car front impact with a slower crossing approach.")
    parser.add_argument("--placement-grid", action="store_true", help="Test lawful AV stop positions inferred from a free post-impact SUV trajectory.")
    args = parser.parse_args()
    original = source_turn_plans.author_turn_plan
    cid = "135_Zoox_January_1_2024_(A)"
    base = ROOT / "outputs/validated_42_20260917/cases" / cid
    client = SourceCaseClient(replace(RemoteCfg.from_env(), compact_artifacts=True, keep_remote_runs=True))
    candidates = [(30, -42, 9), (31.5, -42, 9), (30, -46, 9), (31.5, -46, 9), (30, -40, 8), (31.5, -40, 8), (31.8, -42, 9), (32.1, -42, 9)]
    candidates = [(*candidate, 6) for candidate in candidates] + [(40, -42, 9, 10), (42, -42, 9, 10), (44, -42, 9, 10),
                                                                 (41, -42, 9, 10), (41.3, -42, 9, 10), (41.5, -42, 9, 10),
                                                                 (42.3, -42, 9, 10), (42.6, -42, 9, 10), (42.9, -42, 9, 10), (43.2, -42, 9, 10),
                                                                 (42.6, -42, 9, 10), (42.6, -42, 9, 10), (42.6, -42, 9, 10),
                                                                 (42.6, -42, 9, 10), (42.6, -42, 9, 10), (42.7, -42, 9, 10), (42.75, -42, 9, 10)]
    asset_candidates = [(asset, y, -42, 9, velocity)
                        for asset in ("vehicle.jeep.wrangler_rubicon", "vehicle.audi.etron")
                        for y, velocity in ((31.5, 6), (30, 6), (33, 6), (42.6, 10), (41.5, 10), (43.5, 10))]
    candidates = asset_candidates if args.asset_grid else [("vehicle.nissan.patrol_2021", *c) for c in candidates]
    if args.timing_grid:
        candidates = [("vehicle.nissan.patrol_2021", y, -42, 9, 10) for y in (42.3, 42.4, 42.5, 42.2, 42.1, 42.0)]
        candidates += [("vehicle.nissan.patrol_2021", y, -47, 10, 10) for y in (42.3, 42.6, 42.9, 43.2)]
    if args.momentum_grid:
        candidates = [("vehicle.nissan.patrol_2021", y, -42, 9, 14) for y in (46, 46.5, 45.5, 47, 45)]
        candidates += [("vehicle.nissan.patrol_2021", y, -60, 14, 14) for y in (55.5, 56.5, 54.5, 57.5, 53.5)]
    if args.front_contact_grid:
        candidates = [(asset, y, -24, 5, 10)
                      for asset in ("vehicle.nissan.patrol_2021", "vehicle.jeep.wrangler_rubicon")
                      for y in (40, 41, 42, 39, 43)]
    placements = [(1.1, -4.7), (1.1, -5.1), (1.6, -4.7), (1.6, -5.1), (2.1, -4.7), (2.1, -5.1)]
    if args.placement_grid:
        candidates = [("vehicle.audi.etron", 30, -42, 9, 6) for _ in placements]
    for i, (asset, suv_y, car_x, speed, suv_speed) in enumerate(candidates, 1):
        attempt = ("secondary_placement_v6_%02d" if args.placement_grid else "secondary_front_v5_%02d" if args.front_contact_grid else "secondary_momentum_v4_%02d" if args.momentum_grid else "secondary_timing_v3_%02d" if args.timing_grid else "secondary_asset_v2_%02d" if args.asset_grid else "secondary_grid_v1_%02d") % i
        output = base / attempt
        if output.exists():
            continue
        def authored(prefix, frame):
            actors, pairs, notes = copy.deepcopy(original(prefix, frame))
            if i >= 12 and not (args.asset_grid or args.timing_grid or args.momentum_grid):
                stop_y = -4.25 if i < 19 else -4.55 - .2 * (i - 19)
                if i >= 22:
                    stop_y = -6.25 if i == 22 else -7
                actors["hero"]["points"] = [(0, 1.1, stop_y, 90), (20, 1.1, stop_y, 90)]
                notes.append("Assumed northbound stop position keeps the AV front at the edge of the crossing driving lanes; no surveyed stop-line placement is claimed.")
            actors["suv"]["points"] = [(0, -1.75, suv_y, -90), (10, -1.75, suv_y - 10 * suv_speed, -90), (20, -1.75, suv_y - 10 * suv_speed, -90)]
            actors["suv"]["blueprint"] = asset
            notes.append("Stock SUV asset %s substitutes for the reported Toyota SUV (model unreported); manufacturer, dimensions and inertia do not reproduce the actual Toyota vehicle." % asset)
            actors["crossing_car"].update(blueprint="vehicle.tesla.model3", points=[(0, car_x, 0, 0), (12, car_x + 12 * speed, 0, 0), (20, car_x + 12 * speed, 0, 0)])
            if args.momentum_grid:
                actors["suv"]["max_speed"] = suv_speed + 2
                actors["crossing_car"]["max_speed"] = speed + 2
                actors["hero"]["points"] = [(0, 2.5, -5, 90), (20, 2.5, -5, 90)]
                notes.append("AV center is 2.5 m into the northbound lane and 5 m behind the local crossing origin; its front remains outside the crossing driving lane. The exact stop-line position is unreported.")
            if args.front_contact_grid:
                actors["hero"]["points"] = [(0, 1.1, -4.75, 90), (20, 1.1, -4.75, 90)]
                notes.append("AV's front is placed immediately behind the crossing-lane edge, using an assumed stop line; the reported crossing car approaches at an unreported assumed 5 m/s.")
            if args.placement_grid:
                stop_x, stop_y = placements[i - 1]
                actors["hero"]["points"] = [(0, stop_x, stop_y, 90), (20, stop_x, stop_y, 90)]
                notes.append("Assumed AV stop center (%s, %s) remains in the northbound lane behind the crossing edge. Candidate chosen from a stock SUV free post-impact trajectory, then validated by a new physical run." % (stop_x, stop_y))
            notes.append("Calibration candidate: unreported SUV approach coordinate %s m, passenger-car coordinate %s m, reference speed %s m/s. Stock Tesla sedan substitutes for the unspecified passenger car." % (suv_y, car_x, speed))
            notes.append("Unreported SUV reference speed: %s m/s; actual speed is measured, not imposed by velocity assignment." % suv_speed)
            return actors, pairs, notes
        source_turn_plans.author_turn_plan = authored
        manifest = prepare(cid, output, "reconstruction")
        try:
            client.run_scenario(output / "map.xodr", output / "scenario.xosc", name="source135_" + attempt,
                                sut_actor="", rgb_actor_role="hero", max_seconds=110, out_dir=output / "run")
            validation = inspect_run(output / "run", manifest)
            measured = validation["measured_source_sequence"]
            rows = [json.loads(line) for line in (output / "run/sim_trace_raw.jsonl").read_text().splitlines()]
            first = measured["metrics"].get("contact_times_s", [])
            spin = max((abs((r["actors"]["suv"]["yaw"] - 90 + 180) % 360 - 180) for r in rows if first and r["simulation_time"] > first[0]), default=0)
            print(json.dumps({"attempt": attempt, "checks": measured["checks"], "metrics": measured["metrics"], "max_suv_heading_change_deg": spin}), flush=True)
            if measured["pass"]:
                break
        except Exception as exc:
            (output / "run_failure.json").write_text(json.dumps({"error": str(exc)}, indent=2))
            print(json.dumps({"attempt": attempt, "error": str(exc)}), flush=True)


if __name__ == "__main__":
    main()
