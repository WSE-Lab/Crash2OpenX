#!/usr/bin/env python3
"""VLM-driven QA agent for the road+scene pipeline.

One VLM call reads three things at once:
  - the rendered OpenDRIVE map (the coordinator's <name>.html, screenshot via playwright)
  - a schematic of the scene_seed (ego in center, NPCs placed by ``position`` semantics)
  - the VLM-extracted accident text (the event_description blob)

…and emits structured JSON: ``verdict``, ``issues``, ``fix_hint_road``,
``fix_hint_scene``. The coordinator can loop the road/scene inference with
those hints injected if verdict=fail.
"""

from __future__ import annotations

import base64
import io
import json
import os
import re
from pathlib import Path
from typing import Any


QA_SYSTEM_PROMPT = """你是事故场景流水线的 QA 评审员。你会收到:
- 一份事故事实文本(由 VLM 从原始 PDF 抽取);
- 一张拼接图:**左**是 OpenDRIVE 地图渲染(road_seed 实际编译出来的样子),**右**是该 scene_seed 的拓扑示意(ego 在中心,NPC 按 position 语义放置,标注 kind/position/behavior block);
- road_seed 与 scene_seed 的 JSON。

按下面三个维度评估,只能输出 JSON,不要 markdown 代码块、不要解释:

1. **consistency_with_input** — 地图拓扑(车道数、路口类型、交通控制)和 scene 描述(SUT 机动、NPC 类型/位置/行为、collision 双方)是否与事故事实文本一致。
2. **road_correctness** — 地图本身是否合理:topology 与事故所述路况搭得上吗;车道数和道路类型搭配合理吗;中心线规则合理吗;路口是否承载得了所述机动。
3. **scene_correctness** — scene 抽象是否合理:SUT 的 maneuver 与每个 NPC 的 behavior block 组合能产生 collision 里写的两方冲突吗;NPC 的 kind/position 与事故文本贴合吗;有没有错把"被动停下让行的车"选成了 ego(应选具备主动测试价值的车)。

严格 schema:
{
  "verdict": "pass" | "fail",
  "issues": [
    {"type": "consistency_with_input" | "road_correctness" | "scene_correctness",
     "severity": "high" | "medium" | "low",
     "description": "一句中文说明哪里不对、附上原文证据片段"}
  ],
  "fix_hint_road": "若 road 需改,给 road_seed 推理器的明确中文指导(例如'topology 应改成 t_junction,因为事故文本只提到 X 与 Y 两条街相交,Y 终止于 X');road 无需改则空字符串",
  "fix_hint_scene": "若 scene 需改,给 scene_seed 推理器的明确中文指导(例如'NPC v1 的 behavior.block 应改为 stopped_ahead 而非 front_brake,因为被追尾的 Zoox 已几乎停下让行人,不是急刹');scene 无需改则空字符串"
}

判定:
- 任一 issue 的 severity=high → verdict=fail。
- 全部 issue 都是 low,或无 issue → verdict=pass。
- fix_hint 必须可执行:点出该改哪个字段、改成什么、依据是文本里的哪句话。不要写"建议重新考虑"这种没信息量的话。
- 若该维度没问题就把对应 fix_hint 写成空字符串。
"""


def _extract_json(text: str) -> dict[str, Any]:
    text = (text or "").strip()
    if text.startswith("```"):
        text = re.sub(r"^```(?:json)?\s*", "", text)
        text = re.sub(r"\s*```$", "", text)
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        s, e = text.find("{"), text.rfind("}")
        if s >= 0 and e > s:
            return json.loads(text[s:e + 1])
        raise


def _render_map_png_xodr(
    html_path: Path,
    road_seed: dict | None,
    width: int,
    height: int,
) -> bytes:
    """Deterministic browser-free map render used when Chromium is unavailable."""
    from PIL import Image, ImageDraw
    from tools.replay_geom import parse_xodr_lines

    xodr_path = html_path.with_suffix(".xodr")
    roads = parse_xodr_lines(xodr_path)
    points = [point for road in roads for point in road["points"]]
    if not points:
        raise RuntimeError(f"no renderable line/arc/spiral geometry in {xodr_path}")

    img = Image.new("RGB", (width, height), "#f7f8fb")
    draw = ImageDraw.Draw(img)
    title_font = _load_font(18)
    label_font = _load_font(14)
    small_font = _load_font(12)

    margin, top = 52, 84
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    span_x = max(1.0, max(xs) - min(xs))
    span_y = max(1.0, max(ys) - min(ys))
    scale = min((width - 2 * margin) / span_x, (height - top - margin) / span_y)
    draw_w = span_x * scale
    draw_h = span_y * scale
    x_pad = (width - draw_w) / 2
    y_pad = top + (height - top - draw_h) / 2

    def project(point: tuple[float, float]) -> tuple[int, int]:
        x, y = point
        return (
            round(x_pad + (x - min(xs)) * scale),
            round(y_pad + (max(ys) - y) * scale),
        )

    road = (road_seed or {}).get("road") or {}
    lanes = road.get("lanes") or {}
    lane_count = max(1, int(lanes.get("forward") or 0) + int(lanes.get("backward") or 0))
    body_width = max(14, min(42, lane_count * 10 + 8))

    draw.rectangle([0, 0, width - 1, height - 1], outline="#c6ccd8", width=2)
    draw.text((18, 14), "OpenDRIVE deterministic geometry preview", fill="#202938", font=title_font)
    summary = (
        f"topology={road.get('topology', '?')} · type={road.get('type', '?')} · "
        f"lanes forward/backward={lanes.get('forward', '?')}/{lanes.get('backward', '?')} · "
        f"center_line={road.get('center_line', '?')}"
    )
    draw.text((18, 43), summary, fill="#536174", font=label_font)

    for item in roads:
        polyline = [project(point) for point in item["points"]]
        if len(polyline) < 2:
            continue
        draw.line(polyline, fill="#a8b0bd", width=body_width, joint="curve")
        draw.line(polyline, fill="#ffffff", width=2, joint="curve")
        mid = polyline[len(polyline) // 2]
        draw.text((mid[0] + 7, mid[1] + 5), f"road {item['road_id']}",
                  fill="#39465a", font=small_font)

    start = project(roads[0]["points"][0])
    end = project(roads[-1]["points"][-1])
    draw.ellipse([start[0] - 6, start[1] - 6, start[0] + 6, start[1] + 6],
                 fill="#2b6ff3", outline="white", width=2)
    draw.ellipse([end[0] - 6, end[1] - 6, end[0] + 6, end[1] + 6],
                 fill="#10a36a", outline="white", width=2)
    draw.text((18, height - 30),
              f"source={xodr_path.name} · sampled roads={len(roads)}",
              fill="#6c7788", font=small_font)
    return _png_bytes(img)


def render_map_png(
    html_path: Path,
    width: int = 1280,
    height: int = 800,
    wait_ms: int = 1500,
    road_seed: dict | None = None,
) -> bytes:
    """Render the OpenDRIVE preview, falling back to pure XODR geometry."""
    try:
        from playwright.sync_api import sync_playwright

        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True)
            page = browser.new_page(viewport={"width": width, "height": height})
            page.goto(html_path.resolve().as_uri(), wait_until="networkidle", timeout=30000)
            page.wait_for_timeout(wait_ms)
            png = page.screenshot(full_page=False, type="png")
            browser.close()
        return png
    except Exception as exc:  # noqa: BLE001 - browser is optional in restricted runtimes
        print(f"[qa] Chromium map render unavailable; using XODR fallback: "
              f"{type(exc).__name__}: {str(exc).splitlines()[0]}", flush=True)
        return _render_map_png_xodr(html_path, road_seed, width, height)


# Position semantics -> unit (dx, dy) in canvas coords (y grows down, ego at center).
_POSITION_DXDY = {
    "ahead_same_lane":  (0.0, -1.0),
    "behind_same_lane": (0.0,  1.0),
    "oncoming":         (0.0, -1.5),
    "adjacent":         (1.0,  0.0),   # side=left flips x
    "cross":            (1.5,  0.0),
    "opposing_leg":     (0.0, -1.8),
    "roadside":         (1.2,  0.4),
}

_KIND_COLOR = {
    "vehicle":    "#4f6ef7",
    "cyclist":    "#10a36a",
    "pedestrian": "#c47d00",
    "static":     "#777777",
}


def _png_bytes(img) -> bytes:
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


def _load_font(size: int):
    from PIL import ImageFont
    for path in ("/System/Library/Fonts/PingFang.ttc",
                 "/System/Library/Fonts/Supplemental/Arial Unicode.ttf",
                 "/Library/Fonts/Arial Unicode.ttf"):
        if Path(path).exists():
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                continue
    return ImageFont.load_default()


def render_scene_schematic(scene_seed: dict, width: int = 520, height: int = 520) -> bytes:
    """Top-down schematic of the scene_seed: ego in center, NPCs placed by position.

    No real coordinates exist at this stage — positions are purely semantic
    (ahead_same_lane / oncoming / cross …). The picture is meant for the VLM to
    cross-check the scene's relational layout against the actual map render
    on its left, not for geometric exactness.
    """
    from PIL import Image, ImageDraw

    scene = scene_seed.get("scene") if isinstance(scene_seed, dict) else scene_seed
    img = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(img)
    title_font = _load_font(13)
    label_font = _load_font(11)

    draw.rectangle([0, 0, width - 1, height - 1], outline="#999", width=1)
    draw.text((8, 6), "scene 拓扑示意 · ego 在中心,NPC 按 position 放置", fill="#444", font=title_font)

    cx, cy = width // 2, height // 2
    if not scene:
        draw.text((cx - 60, cy), "(no scene)", fill="#999", font=title_font)
        return _png_bytes(img)

    # Cross-hair to imply road axes (vertical = same-lane axis, horizontal = cross axis).
    draw.line([(cx, 36), (cx, height - 56)], fill="#e6e9f0", width=1)
    draw.line([(28, cy), (width - 28, cy)], fill="#e6e9f0", width=1)
    draw.text((cx + 4, 36), "前", fill="#c2c8d3", font=label_font)
    draw.text((cx + 4, height - 56), "后", fill="#c2c8d3", font=label_font)
    draw.text((28, cy - 14), "左", fill="#c2c8d3", font=label_font)
    draw.text((width - 38, cy - 14), "右", fill="#c2c8d3", font=label_font)

    # ego
    sut = scene.get("sut") or {}
    R = 20
    draw.ellipse([cx - R, cy - R, cx + R, cy + R], fill="#d63b46", outline="black", width=2)
    draw.text((cx - 12, cy - 7), "ego", fill="white", font=label_font)
    draw.text((cx - 36, cy + R + 4), f"maneuver: {sut.get('maneuver', '?')}",
              fill="#222", font=label_font)

    # NPCs
    SCALE = 100
    for npc in scene.get("npcs") or []:
        pos = npc.get("position", "ahead_same_lane")
        side = npc.get("side", "none")
        dx, dy = _POSITION_DXDY.get(pos, (1.0, -1.0))
        if side == "left":
            dx = -abs(dx) if dx else -0.5
        elif side == "right":
            dx = abs(dx) if dx else 0.5
        x = cx + int(dx * SCALE)
        y = cy + int(dy * SCALE)
        x = max(40, min(width - 40, x))
        y = max(60, min(height - 60, y))
        kind = npc.get("kind", "vehicle")
        block = (npc.get("behavior") or {}).get("block", "?")
        color = _KIND_COLOR.get(kind, "#4f6ef7")
        # Light connector showing relation to ego
        draw.line([(x, y), (cx, cy)], fill="#d8dce6", width=1)
        r = 15
        draw.ellipse([x - r, y - r, x + r, y + r], fill=color, outline="black", width=1)
        draw.text((x - 7, y - 6), str(npc.get("id", "?")), fill="white", font=label_font)
        draw.text((x - 70, y + r + 2), f"{kind}·{pos}", fill="#222", font=label_font)
        draw.text((x - 70, y + r + 16), f"block: {block}", fill="#444", font=label_font)

    col = scene.get("collision") or {}
    draw.text((8, height - 26),
              f"collision: {col.get('a', '?')} ↔ {col.get('b', '?')}    "
              f"control: {scene.get('control', '?')}",
              fill="#444", font=label_font)
    return _png_bytes(img)


def stitch_side_by_side(left_png: bytes, right_png: bytes, pad: int = 12) -> bytes:
    """Glue map (left) + scene schematic (right) so the VLM sees both in one image."""
    from PIL import Image

    left = Image.open(io.BytesIO(left_png))
    right = Image.open(io.BytesIO(right_png))
    h = max(left.height, right.height)
    w = left.width + pad + right.width
    canvas = Image.new("RGB", (w, h), "white")
    canvas.paste(left, (0, (h - left.height) // 2))
    canvas.paste(right, (left.width + pad, (h - right.height) // 2))
    return _png_bytes(canvas)


def _normalize_verdict(d: dict[str, Any]) -> dict[str, Any]:
    verdict = str(d.get("verdict") or "").strip().lower()
    if verdict not in {"pass", "fail"}:
        verdict = "fail"  # model didn't follow schema -> err on the safe side
    issues = d.get("issues") if isinstance(d.get("issues"), list) else []
    return {
        "verdict": verdict,
        "issues": issues,
        "fix_hint_road": str(d.get("fix_hint_road") or "").strip(),
        "fix_hint_scene": str(d.get("fix_hint_scene") or "").strip(),
    }


def call_qa_vlm(
    *,
    composite_png: bytes,
    event_text: str,
    road_seed: dict,
    scene_seed: dict,
    vlm_model: str,
    base_url: str,
    api_key_env: str,
    max_tokens: int = 4000,
) -> dict[str, Any]:
    """One QA call. Returns the parsed+normalized verdict JSON.

    If the model returns non-JSON (empty content, prose, malformed), we fall
    back to a structured ``fail`` verdict that captures the raw response so the
    QA loop can still proceed (and the user can see what the VLM said).
    """
    from openai import OpenAI

    api_key = os.environ.get(api_key_env)
    if not api_key:
        raise RuntimeError(f"Missing {api_key_env} for QA VLM")

    payload = {
        "event_text": (event_text or "")[:6000],
        "road_seed": (road_seed or {}).get("road"),
        "road_description_zh": (road_seed or {}).get("description_zh"),
        "scene_seed": (scene_seed or {}).get("scene"),
        "scene_description_zh": (scene_seed or {}).get("description_zh"),
    }
    b64 = base64.b64encode(composite_png).decode("ascii")
    client = OpenAI(api_key=api_key, base_url=base_url)
    messages = [
        {"role": "system", "content": QA_SYSTEM_PROMPT},
        {"role": "user", "content": [
            {"type": "text",
             "text": "事故事实文本与 road / scene JSON 如下,左侧是 OpenDRIVE 地图渲染,"
                     "右侧是 scene 抽象拓扑示意。严格按 schema 返回 **纯 JSON**(不要任何前后文字、不要 markdown 代码块)。\n\n"
                     + "```json\n" + json.dumps(payload, ensure_ascii=False, indent=2) + "\n```"},
            {"type": "image_url",
             "image_url": {"url": f"data:image/png;base64,{b64}"}},
        ]},
    ]
    raw = ""
    for attempt in range(2):
        resp = client.chat.completions.create(
            model=vlm_model, stream=False, temperature=0.0, max_tokens=max_tokens,
            messages=messages,
        )
        raw = (resp.choices[0].message.content or "").strip()
        if raw or attempt == 1:
            break
        print("[qa] VLM returned empty content; retrying once", flush=True)
    try:
        return _normalize_verdict(_extract_json(raw))
    except (json.JSONDecodeError, ValueError) as exc:
        # VLM didn't emit valid JSON -> degrade to a structured fail verdict
        # rather than crashing the coordinator. We surface the raw text so the
        # UI / logs explain why QA couldn't render a real verdict.
        snippet = raw[:300] if raw else "(empty response from VLM)"
        return {
            "verdict": "fail",
            "issues": [{
                "type": "consistency_with_input", "severity": "low",
                "description": f"QA VLM 返回的不是合法 JSON,无法解析。raw[:300]={snippet}",
            }],
            "fix_hint_road": "",
            "fix_hint_scene": "",
            "vlm_raw": raw,
            "vlm_parse_error": f"{type(exc).__name__}: {exc}",
        }


def run_qa(
    *,
    html_path: Path,
    event_text: str,
    road_seed: dict,
    scene_seed: dict,
    out_dir: Path,
    round_idx: int,
    vlm_model: str = "xiaomi/mimo-v2.5",
    base_url: str = "https://openrouter.ai/api/v1",
    api_key_env: str = "OPENROUTER_API_KEY",
) -> dict[str, Any]:
    """End-to-end: screenshot map → draw scene schematic → stitch → VLM → persist artifacts."""
    out_dir.mkdir(parents=True, exist_ok=True)
    if not html_path or not html_path.exists():
        # No map to evaluate -> hard fail with a road hint.
        stub = {
            "verdict": "fail",
            "issues": [{"type": "road_correctness", "severity": "high",
                        "description": f"OpenDRIVE preview HTML missing at {html_path}"}],
            "fix_hint_road": "road 推理结果未产生可编译 OpenDRIVE,请重新选择 topology / lanes 使其落入支持的 schema v2",
            "fix_hint_scene": "",
            "artifacts": {},
            "round": round_idx,
        }
        (out_dir / f"qa_report_r{round_idx}.json").write_text(
            json.dumps(stub, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return stub

    map_png = render_map_png(html_path, road_seed=road_seed)
    scene_png = render_scene_schematic(scene_seed or {})
    composite = stitch_side_by_side(map_png, scene_png)

    (out_dir / f"qa_map_r{round_idx}.png").write_bytes(map_png)
    (out_dir / f"qa_scene_r{round_idx}.png").write_bytes(scene_png)
    composite_path = out_dir / f"qa_composite_r{round_idx}.png"
    composite_path.write_bytes(composite)

    verdict = call_qa_vlm(
        composite_png=composite,
        event_text=event_text or "",
        road_seed=road_seed or {},
        scene_seed=scene_seed or {},
        vlm_model=vlm_model, base_url=base_url, api_key_env=api_key_env,
    )
    verdict["artifacts"] = {
        "map_png": str(out_dir / f"qa_map_r{round_idx}.png"),
        "scene_png": str(out_dir / f"qa_scene_r{round_idx}.png"),
        "composite_png": str(composite_path),
    }
    verdict["round"] = round_idx
    (out_dir / f"qa_report_r{round_idx}.json").write_text(
        json.dumps(verdict, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return verdict
