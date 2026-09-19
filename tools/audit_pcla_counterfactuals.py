#!/usr/bin/env python3
"""Audit PCLA execution separately from source collision reproduction."""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "outputs/validated_42_20260917"


def main():
    frozen = json.loads((ROOT / "data/eval/baseline_42.json").read_text())["cases"]
    cases = []
    for item in frozen:
        cid = item["case_id"]
        attempts = []
        for path in (BASE / "cases" / cid).glob("pcla_v*/run/remote_run.json"):
            run = path.parent
            if not (run / "source_validation.json").exists():
                # The batch writes this last, after download and inspection.
                continue
            remote = json.loads(path.read_text())
            summary = json.loads((run / "summary.json").read_text())
            feedback = json.loads((run / "sim_feedback.json").read_text()).get("summary", {})
            log = (run / "demo.log").read_text(errors="replace")
            device = "cpu" if "PCLA_MODEL_PARAMETER_DEVICE=cpu" in log else "cuda"
            rows = [json.loads(line) for line in (run / "sim_trace_raw.jsonl").read_text().splitlines()]
            events = [json.loads(line) for line in (run / "events.jsonl").read_text().splitlines()]
            pairs = sorted({tuple(sorted(e["payload"].get("actors", []))) for e in events
                            if e.get("event_type") == "collision" and e.get("payload", {}).get("source") == "carla_collision_sensor"})
            hero_collision = any("hero" in pair for pair in pairs)
            outcome = summary.get("autonomous_test_outcome") or ("physical_collision_observed" if hero_collision else "no_hero_collision_within_recorded_window")
            controls = [r["actors"]["hero"].get("applied_control", {}) for r in rows if "hero" in r["actors"]]
            duration = rows[-1]["simulation_time"] - rows[0]["simulation_time"] if rows else 0
            reference = json.loads((run.parent / "trace_reference.json").read_text())
            expected_duration = reference["duration_s"]
            initial = reference["actors"]["hero"]["trace"][0]
            actual_initial = next((r["actors"]["hero"] for r in rows if "hero" in r["actors"]), {})
            initial_position_error = math.hypot(actual_initial.get("x", float("inf")) - initial["x"],
                                                actual_initial.get("y", float("inf")) + initial["y"])
            initial_heading_error = abs((actual_initial.get("yaw", float("inf")) + initial["h"] + 180) % 360 - 180)
            calibrations = [json.loads(line.split("=", 1)[1]) for line in log.splitlines()
                            if line.startswith("PCLA_GNSS_CALIBRATION=")]
            gnss_validated = bool(calibrations) and all(
                c["max_probe_error_m"] <= .02 and c["noise_preserved"] and not c["vehicle_ground_truth_used"]
                for c in calibrations)
            actor_min_z = {}
            for row in rows:
                for name, actor in row["actors"].items():
                    actor_min_z[name] = min(actor_min_z.get(name, float("inf")), actor.get("z", -999))
            checks = {
                "correct_control_identity": remote.get("execution_mode") == "pcla_autonomous_sut" and remote.get("sut_actor") == "hero" and remote.get("pcla_agent") == "if_if",
                "interfuser_loaded": "agent 'if_if' loaded" in log,
                "agent_sensors_ready": "sensors ready for agent 'if_if'" in log,
                "external_control_initialized": "setup complete for ExternalControl" in log,
                "all_expected_actors_spawned": feedback.get("all_expected_actors_spawned") is True,
                "no_controller_failures": not feedback.get("controller_failures"),
                "nonempty_simulation": summary.get("total_ticks", 0) > 0,
                "no_runtime_exception": not str(summary.get("termination_reason", "")).startswith("exception"),
                "real_rgb_video_present": (run / "carla_rgb.mp4").is_file(),
                "finite_controls_recorded": bool(controls) and all(math.isfinite(c.get(k, float("nan"))) for c in controls for k in ("throttle", "brake", "steer")),
                "no_unbounded_fall_below_map": bool(rows) and all(r["actors"]["hero"].get("z", -999) > -1 for r in rows if "hero" in r["actors"]),
            }
            measured_path = run / "measured_source_sequence.json"
            measured = json.loads(measured_path.read_text()) if measured_path.exists() else json.loads((run / "source_validation.json").read_text())["measured_source_sequence"]
            integration_checks = {
                "runtime_evidence_valid": all(checks.values()),
                "gnss_route_coordinates_calibrated": gnss_validated,
                "full_observation_window": duration >= expected_duration - .1,
                "scenario_completed": summary.get("termination_reason") == "scenario_completed",
                "no_map_surface_exit": outcome != "map_surface_exit_failure",
                "no_actor_falls_below_map": bool(actor_min_z) and all(z > -1 for z in actor_min_z.values()),
                "initial_pose_matches_authored_input": initial_position_error <= .001 and initial_heading_error <= .001,
            }
            attempts.append({"attempt": run.parent.name, "path": str(run.parent.relative_to(BASE)),
                             "inference_device": device,
                             "device_evidence": "explicit model parameter device log" if "PCLA_MODEL_PARAMETER_DEVICE=" in log else "original CUDA-only InterFuser agent",
                             "runtime_checks": checks, "runtime_pass": all(checks.values()),
                             "integration_checks": integration_checks,
                             "integration_pass": all(integration_checks.values()),
                             "gnss_calibrations": calibrations,
                             "actor_minimum_z_m": actor_min_z,
                             "initial_position_error_m": initial_position_error,
                             "initial_heading_error_deg": initial_heading_error,
                             "source_sequence_pass": measured["pass"],
                             "source_contact_times_s": measured.get("metrics", {}).get("contact_times_s", []),
                             "hero_max_planar_speed_mps": max((math.hypot(r["actors"]["hero"].get("vx", 0), r["actors"]["hero"].get("vy", 0)) for r in rows if "hero" in r["actors"]), default=0),
                             "ego_offroad_seconds": summary.get("off_road_time"),
                             "autonomous_outcome": outcome, "physical_collision_pairs": pairs,
                             "hero_collision_observed": hero_collision,
                             "recorded_simulation_duration_s": duration,
                             "expected_observation_duration_s": expected_duration,
                             "route_completion_percent": summary.get("final_route_completion"),
                             "termination_reason": summary.get("termination_reason")})
        cases.append({"case_id": cid, "attempts": attempts, "has_runtime_pass": any(a["runtime_pass"] for a in attempts),
                      "has_integration_pass": any(a["integration_pass"] for a in attempts)})
    result = {"case_count": 42, "recorded_cases": sum(bool(c["attempts"]) for c in cases),
              "runtime_passing_cases": sum(c["has_runtime_pass"] for c in cases),
              "integration_passing_cases": sum(c["has_integration_pass"] for c in cases),
              "integration_scope": "Authored initial pose preserved, calibrated GNSS/route coordinates, full authored observation window and no actor falling below the map or runtime failure. Does not mean the agent completed the extended navigation route or drove without a collision.",
              "note": "Runtime checks validate real InterFuser execution and bounded trace/video evidence. Map-exit failure terminations remain failures in autonomous_outcome. This is not a claim of safe driving, source collision reproduction, or completion of every driving task.",
              "cases": cases}
    (BASE / "audit/pcla_counterfactuals.json").write_text(json.dumps(result, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k != "cases"}))


if __name__ == "__main__":
    main()
