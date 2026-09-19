#!/usr/bin/env python3
"""Decode and time-check the current best RGB recording for each frozen case."""
import hashlib
import argparse
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "outputs/validated_42_20260917"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcla", action="store_true", help="Audit autonomous recordings separately.")
    args = parser.parse_args()
    mode = "pcla_autonomous_sut" if args.pcla else "scripted_physical_reconstruction"
    inventory = json.loads((BASE / "audit/runtime_inventory.json").read_text())
    selection_file = BASE / "audit/reviewed_selection.json"
    selections = {c["case_id"]: c for c in json.loads(selection_file.read_text())["cases"]} if selection_file.exists() else {}
    results = []
    for case in inventory["cases"]:
        options = [a for a in case["attempts"] if a.get("has_rgb_video") and a["mode"] == mode]
        if not options:
            continue
        chosen = max(options, key=lambda a: (False if args.pcla else a.get("measured_sequence_pass", False), (BASE / a["path"] / "run/carla_rgb.mp4").stat().st_mtime))
        if not args.pcla:
            review = selections.get(case["case_id"], {})
            selected_path = review.get("reconstruction") or review.get("candidate")
            chosen = next((a for a in options if a["path"] == selected_path), chosen)
        run = BASE / chosen["path"] / "run"
        video = run / "carla_rgb.mp4"
        probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                                "stream=width,height,avg_frame_rate,nb_frames,duration", "-of", "json", str(video)], capture_output=True, text=True, check=True)
        stream = json.loads(probe.stdout)["streams"][0]
        frames = int(stream["nb_frames"])
        decoded = subprocess.run(["ffmpeg", "-v", "error", "-i", str(video), "-f", "null", "-"], capture_output=True, text=True)
        source = run / "rgb_frames/timestamps.jsonl"
        original = [json.loads(line) for line in source.read_text().splitlines()] if source.is_file() else []
        # Before the recorder lifecycle fix, an orphan could append timestamps
        # after encoding. Preserve that raw file; derive only actual MP4 indices.
        aligned = [row for row in original if 0 <= row["image_index"] < frames]
        if aligned:
            (run / "video_frame_timestamps.jsonl").write_text("".join(json.dumps(row) + "\n" for row in aligned))
        times = chosen.get("metrics", {}).get("contact_times_s", [])
        termination_times = []
        if args.pcla:
            events = [json.loads(line) for line in (run / "events.jsonl").read_text().splitlines()]
            times, previous = [], {}
            for event in sorted(events, key=lambda e: e.get("simulation_time", 0)):
                if event.get("event_type") == "map_surface_exit":
                    termination_times.append(event["simulation_time"])
                if event.get("event_type") != "collision" or event.get("payload", {}).get("source") != "carla_collision_sensor":
                    continue
                pair = tuple(sorted(event["payload"].get("actors", [])))
                timestamp = event["simulation_time"]
                if pair not in previous or timestamp - previous[pair] > .75:
                    times.append(timestamp)
                previous[pair] = timestamp
        contacts = []
        for timestamp in times:
            nearest = min(aligned, key=lambda row: abs(row["simulation_time"] - timestamp)) if aligned else None
            contacts.append({"contact_time_s": timestamp, "nearest_video_frame": nearest,
                             "difference_seconds": abs(nearest["simulation_time"] - timestamp) if nearest else None})
        terminations = []
        for timestamp in termination_times:
            nearest = min(aligned, key=lambda row: abs(row["simulation_time"] - timestamp)) if aligned else None
            terminations.append({"termination_time_s": timestamp, "nearest_video_frame": nearest,
                                 "difference_seconds": abs(nearest["simulation_time"] - timestamp) if nearest else None})
        result = {"case_id": case["case_id"], "attempt": chosen["attempt"], "execution_mode": chosen["mode"],
                  "sha256": hashlib.sha256(video.read_bytes()).hexdigest(), "probe": stream,
                  "full_video_decode_pass": decoded.returncode == 0 and not decoded.stderr.strip(),
                  "decode_messages": decoded.stderr.strip(), "raw_timestamp_rows": len(original),
                  "video_aligned_timestamp_rows": len(aligned), "timestamp_rows_after_encoding": len(original) - len(aligned),
                  "encoded_frames_without_timestamp": max(0, frames - len(aligned)), "physical_contacts": contacts,
                  "encoded_timestamps_complete": [r["image_index"] for r in aligned] == list(range(frames)),
                  "recorded_contact_episodes": len(contacts),
                  "contact_timestamp_coverage_pass": (bool(contacts) or args.pcla) and all(c["difference_seconds"] is not None and c["difference_seconds"] <= .15 for c in contacts),
                  "map_exit_terminations": terminations,
                  "termination_timestamp_coverage_pass": all(c["difference_seconds"] is not None and c["difference_seconds"] <= .15 for c in terminations),
                  "note": "Raw timestamps are preserved. Derived timestamps select only image indices actually present in the MP4. Missing timestamps are never synthesized."}
        (run / "video_integrity.json").write_text(json.dumps(result, indent=2))
        results.append(result)
    destination = "audit/pcla_video_integrity.json" if args.pcla else "audit/video_integrity.json"
    (BASE / destination).write_text(json.dumps(results, indent=2))
    print(json.dumps({"videos": len(results), "decode_pass": sum(r["full_video_decode_pass"] for r in results),
                      "recordings_with_contacts": sum(bool(r["physical_contacts"]) for r in results),
                      "contact_timestamp_coverage": sum(r["contact_timestamp_coverage_pass"] for r in results),
                      "termination_timestamp_coverage": sum(r["termination_timestamp_coverage_pass"] for r in results)}))


if __name__ == "__main__":
    main()
