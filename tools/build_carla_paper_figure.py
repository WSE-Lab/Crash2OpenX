#!/usr/bin/env python3
"""Reframe a 1920x1080 paper-render frame as the compact paper figure."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frame", required=True, type=Path)
    parser.add_argument("--meta", required=True, type=Path)
    parser.add_argument("--out", type=Path, default=Path("paper/figures/carla_metamodel_environment.png"))
    return parser.parse_args()


def font(size: int, bold: bool = False):
    candidates = [
        Path("/System/Library/Fonts/Supplemental") / ("Arial Bold.ttf" if bold else "Arial.ttf"),
        Path("/usr/share/fonts/truetype/dejavu") / ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"),
    ]
    for candidate in candidates:
        if candidate.is_file():
            return ImageFont.truetype(str(candidate), size)
    return ImageFont.load_default()


def _upper(value, default="UNSPECIFIED") -> str:
    return str(value if value not in (None, "", "unknown") else default).upper()


def build(frame_path: Path, meta_path: Path, out_path: Path) -> None:
    image = Image.open(frame_path).convert("RGB")
    if image.size != (1920, 1080):
        raise ValueError(f"expected a 1920x1080 paper-render frame, got {image.size}")
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    road = meta.get("road") or {}
    scene_meta = meta.get("scene") or {}
    lanes = road.get("lanes") or {}
    sut = scene_meta.get("sut") or {}
    npcs = scene_meta.get("npcs") or []
    npc = npcs[0] if npcs else {}
    behavior = npc.get("behavior") or {}
    collision = scene_meta.get("collision") or {}
    environment = scene_meta.get("environment") or {}

    road_line = f"{_upper(road.get('topology'))} | {_upper(road.get('type'))}"
    lane_line = (
        f"FWD {lanes.get('forward', '?')} / BACK {lanes.get('backward', '?')}"
        f" | {_upper(road.get('center_line'))}"
    )
    sut_line = f"{sut.get('id', 'ego')} {_upper(sut.get('maneuver'))}"
    npc_line = f"{npc.get('id', 'npc')} {_upper(npc.get('position'))}"
    behavior_line = (
        f"{_upper(behavior.get('block'))} | collision "
        f"{collision.get('a', '?')}-{collision.get('b', '?')}"
    )
    weather = _upper(environment.get("weather"))
    time_of_day = _upper(environment.get("time_of_day"))
    friction = environment.get("friction_scale")
    environment_line = f"{weather} | {time_of_day}"
    if friction is not None:
        environment_line += f" | friction {friction}"
    if weather == "UNSPECIFIED" and time_of_day == "UNSPECIFIED":
        rendered_line = "rendered ClearNoon*"
    else:
        rendered_line = f"rendered {weather.lower()}/{time_of_day.lower()}"
    canvas = Image.new("RGB", (720, 270), "#111c2a")

    scene = image.crop((620, 178, 1520, 718)).resize((450, 270), Image.Resampling.LANCZOS)
    canvas.paste(scene, (0, 0))

    draw = ImageDraw.Draw(canvas)
    draw.rectangle((450, 0, 720, 270), fill="#111c2a")
    draw.text((466, 8), "METAMODEL VIEW", font=font(20, True), fill="#69e0e6")
    draw.text((466, 40), "RoadSeed", font=font(18, True), fill="#ffffff")
    draw.text((466, 62), road_line, font=font(16, True), fill="#ffffff")
    draw.text((466, 83), lane_line, font=font(13), fill="#e4eaf1")

    draw.text((466, 108), "SceneSeed", font=font(18, True), fill="#ffffff")
    draw.text((466, 130), sut_line, font=font(15), fill="#e4eaf1")
    draw.text((466, 149), npc_line, font=font(12, True), fill="#ffad4d")
    draw.text((466, 168), behavior_line, font=font(11), fill="#e4eaf1")

    draw.text((466, 194), "Environment", font=font(18, True), fill="#9adeb2")
    draw.text((466, 217), environment_line, font=font(12, True), fill="#ffffff")
    draw.text((466, 242), rendered_line, font=font(13), fill="#aebdcd")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out_path, optimize=True)


if __name__ == "__main__":
    args = parse_args()
    build(args.frame, args.meta, args.out)
