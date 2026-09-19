"""Snapshot the small runtime surface used for each CARLA validation run."""
import hashlib
import json
import os
import platform
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path


def snapshot(root, output):
    root, output = Path(root), Path(output)
    relative = ["src/demo.py", "src/data_collector.py", "src/scenario_manager_local.py", "src/scenario_runner_local.py",
                "src/open_scenario.py", "src/ads_conditions.py", "src/ads_geometry.py",
                "src/visualize_carla.py", "src/road_markings.py", "src/openscenario_configuration.py", "scripts/run_headless_scene_record.sh",
                "src/runs/run_scene.py", "src/lane_coordinates.py", "src/opendrive_repair.py", "src/scene_contacts.py",
                "src/storyboard_observer.py", "src/guarded_conditions.py", "src/lane_offset_motion.py", "src/standstill_condition.py", "scenario_runner/srunner/scenarios/open_scenario.py",
                "scenario_runner/srunner/scenariomanager/scenarioatomics/atomic_behaviors.py",
                "scenario_runner/srunner/scenariomanager/scenarioatomics/atomic_trigger_conditions.py",
                "scenario_runner/srunner/tools/openscenario_parser.py",
                "scenario_runner/srunner/tools/scenario_helper.py"]
    relative += ["scenario_runner/srunner/scenariomanager/actorcontrols/" + name + ".py" for name in
                 ["actor_control", "basic_control", "external_control", "gnss_compat", "physics_trajectory_control", "npc_vehicle_control", "generated_lane_route"]]
    relative += ["PCLA/pcla_agents/interfuser/interfuser_agent.py", "PCLA/pcla_agents/interfuser/interfuser_config.py",
                 "PCLA/device_selection_provenance.json"]
    manifest = {"created_utc": datetime.now(timezone.utc).isoformat(), "python": platform.python_version(),
                "runtime_root": str(root), "files": {},
                "rendering": {"markings_enabled": os.environ.get('C2X_NATIVE_ROAD_MARKINGS') == '1',
                              "marking_brightness": os.environ.get('C2X_ROAD_MARKING_BRIGHTNESS'),
                              "representation": 'CARLA debug lines; rendering approximation, no collision geometry'}}
    for name in relative:
        path = root / name
        if path.is_file():
            target = output / "runtime_code" / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
            manifest["files"][name] = hashlib.sha256(path.read_bytes()).hexdigest()
    (output / "runtime_manifest.json").write_text(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    snapshot(*sys.argv[1:3])
