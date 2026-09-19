#!/usr/bin/env python3
"""Extract actual RGB samples and every contact episode for pending PCLA review."""
import argparse
import hashlib
import json
import math
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "outputs/validated_42_20260917"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt", default="pcla_v8_gnss")
    args = parser.parse_args()
    registry_path = BASE / "audit/pcla_reviewed_selection.json"
    registry = json.loads(registry_path.read_text()) if registry_path.exists() else {"cases": []}
    reviewed = {(r["case_id"], r["video_sha256"]) for r in registry["cases"] if r["visual_review"] == "reviewed"}
    audit = json.loads((BASE / "audit/pcla_counterfactuals.json").read_text())
    integration_passes = {r["path"] for c in audit["cases"] for r in c["attempts"] if r.get("integration_pass")}
    candidates = []
    for case in sorted((BASE / "cases").iterdir()):
        attempt = case / args.attempt
        run = attempt / "run"
        if not (run / "source_validation.json").exists():
            continue
        if str(attempt.relative_to(BASE)) not in integration_passes:
            continue
        video = run / "carla_rgb.mp4"
        video_hash = digest(video)
        if (case.name, video_hash) in reviewed:
            continue
        stream = json.loads(subprocess.check_output([
            "ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=nb_frames",
            "-of", "json", str(video)]))["streams"][0]
        n = int(stream["nb_frames"])
        timestamps = [json.loads(line) for line in (run / "rgb_frames/timestamps.jsonl").read_text().splitlines()]
        timestamps = [t for t in timestamps if 0 <= t["image_index"] < n]
        frames = {max(0, int(n * .1)): "early", n // 2: "middle", max(0, n - 2): "end"}
        previous_contact = {}
        episodes = []
        for line in (run / "events.jsonl").read_text().splitlines():
            event = json.loads(line)
            if event["event_type"] != "collision" or event.get("payload", {}).get("source") != "carla_collision_sensor":
                continue
            pair = tuple(sorted(event["payload"].get("actors", [])))
            t = event["simulation_time"]
            if pair not in previous_contact or t - previous_contact[pair] > .75:
                nearest = min(timestamps, key=lambda x: abs(x["simulation_time"] - t))
                frames[nearest["image_index"]] = "contact " + "/".join(pair)
                episodes.append({"simulation_time": t, "pair": pair, "frame": nearest["image_index"]})
            previous_contact[pair] = t
        summary = json.loads((run / "summary.json").read_text())
        panel = Image.new("RGB", (1080, 250 * math.ceil(len(frames) / 3)), "white")
        draw = ImageDraw.Draw(panel)
        images = []
        for j, (index, label) in enumerate(sorted(frames.items())):
            target = BASE / "audit" / f"{args.attempt}_{case.name[:3]}_frame_{index}.jpg"
            subprocess.run(["ffmpeg", "-v", "error", "-i", str(video), "-vf", f"select=eq(n\\,{index})",
                            "-frames:v", "1", "-y", str(target)], check=True)
            x, y = j % 3 * 360, j // 3 * 250
            draw.text((x + 4, y + 3), case.name[:3] + " | " + label + " | frame=" + str(index), fill="black")
            draw.text((x + 4, y + 19), summary["termination_reason"] + " / offroad=" + str(summary["off_road_time"]), fill="black")
            panel.paste(Image.open(target).resize((360, 202)), (x, y + 45))
            images.append(str(target.relative_to(BASE)))
        panel_path = BASE / "audit" / f"{args.attempt}_{case.name[:3]}_review.jpg"
        panel.save(panel_path)
        candidates.append({"case_id": case.name, "attempt": args.attempt, "path": str(attempt.relative_to(BASE)),
                           "video_sha256": video_hash, "input_sha256": {f: digest(attempt / f) for f in ("map.xodr", "scenario.xosc")},
                           "frame_indices": sorted(frames), "contact_episodes": episodes, "images": images,
                           "panel": str(panel_path.relative_to(BASE)), "visual_review": "pending"})
    (BASE / "audit/pcla_pending_visual_review.json").write_text(json.dumps(candidates, indent=2))
    print(json.dumps({"prepared": len(candidates), "panels": [c["panel"] for c in candidates]}))


if __name__ == "__main__":
    main()
