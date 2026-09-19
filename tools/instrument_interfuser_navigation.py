#!/usr/bin/env python3
"""Add opt-in navigation telemetry to the task-local InterFuser copy."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("agent", type=Path)
    args = parser.parse_args()
    if args.agent.is_symlink():
        raise SystemExit("Refusing to modify a symlink: use the task-local copied agent.")
    original = args.agent.read_text()
    if "PCLA_NAV_AUDIT=" in original:
        raise SystemExit("Navigation telemetry already installed.")
    marker = '        tick_data = self.tick(input_data)\n'
    assert original.count(marker) == 1
    modified = original.replace(marker, marker + '''
        if os.environ.get("PCLA_NAV_AUDIT") == "1" and self.step % 20 == 0:
            transform = vehicle.get_transform() if vehicle is not None else None
            record = {
                "step": self.step,
                "timestamp": timestamp,
                "raw_gps": input_data["gps"][1][:2].tolist(),
                "compass_rad": float(tick_data["compass"]),
                "planner_position": tick_data["gps"].tolist(),
                "target_local": tick_data["target_point"].tolist(),
                "command": int(tick_data["next_command"]),
                "remaining_route_points": len(self._route_planner.route),
                "next_route_points": [p.tolist() for p, _ in list(self._route_planner.route)[:3]],
                "actual_transform": {
                    "x": transform.location.x, "y": transform.location.y,
                    "yaw": transform.rotation.yaw,
                } if transform is not None else None,
                "sensor_frames": {k: int(v[0]) for k, v in input_data.items()},
            }
            print("PCLA_NAV_AUDIT=" + json.dumps(record), flush=True)
''')
    compile(modified, str(args.agent), "exec")
    backup = args.agent.with_suffix(".pre_navigation_audit.py")
    if backup.exists():
        raise SystemExit("Backup already exists; inspect before changing.")
    backup.write_text(original)
    args.agent.write_text(modified)
    print(json.dumps({"original_sha256": hashlib.sha256(original.encode()).hexdigest(),
                      "modified_sha256": hashlib.sha256(modified.encode()).hexdigest(),
                      "scope": "Opt-in read-only telemetry after the existing tick call. No changes to inputs, inference or control."}))


if __name__ == "__main__":
    main()
