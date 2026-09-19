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

1. **consistency_with_input** — 地图拓扑(车道数、路口类型、交通控制)和 scene 描述(SUT 机动、NPC 类型/位置/行为、collision 双方)是否与完整事故事实一致。删除导致避让的参与者、只保留连续动作的最后一步、将部分侵道改成完整换道、将多次接触缩为一对冲突，都属于 high issue；词汇表不足应提示 needs_extension，不能认可有损简化。
2. **road_correctness** — 地图本身是否合理:topology 与事故所述路况搭得上吗;车道数和道路类型搭配合理吗;中心线规则合理吗;路口是否承载得了所述机动。
3. **scene_correctness** — scene 抽象是否合理:SUT 的 maneuver 与每个 NPC 的 behavior block 组合能产生 collision 里写的两方冲突吗;NPC 的 kind/position 与事故文本贴合吗;有没有错把"被动停下让行的车"选成了 ego(应选具备主动测试价值的车)。

建模约定(评审必须遵守,违反约定的"问题"不是问题):
- collision 的 {a, b} 是**无序对**:下游按无序集合与仿真结果比对,a/b 的先后不表达"谁撞谁"。只要双方 id 正确,顺序不同**不构成 issue**。
- 仿真中的 ego 是被测 ADS 车辆,按**主动测试价值**选择(通常是移动中、其驾驶行为可被考验的那辆车),它**不必是事故报告里的 AV/当事 SUT**。报告里静止被撞的车(即使是 AV)按约定建模为 NPC,这是正确做法,不是角色颠倒。
- ego 在场景中应有主动驾驶行为；允许初始静止后起步，使用 sut.params.initial_speed_mps=0。初速为零本身不是错误。
- 主动驾驶须按完整时序判断：原文明确自行行驶/通过路口后才停住的车辆，仍可作为ego；不能只因末次接触时静止就强制换角色。双方均有独立驾驶阶段时，可选择任一具测试价值者，但须披露角色分配并完整保留另一方的动作。全程静止或仅被碰撞推动的参与者仍不应作为ego。
- ego固定是外部PCLA控制，sut.id必须为ego，sequence/steps只适用于NPC；不要在fix_hint中要求给sut增加sequence、强制制动/加速/碰撞或用源车辆名替换固定ID。SUT的停车、接触等条件是待实际观测的结果，不能因不能保证重现而把动作转给错误参与者；描述声称已保证但seed未表达的动作仍须指出。
- road.topology 是高层模板(straight/curve/cross_intersection/t_junction/y_junction/merge/fork):`curve` 即"弯道走廊",是高速立交连接匝道(interchange connector)的正确抽象;只有事故动作本身是汇入/分流时才该用 merge/fork。不要因"真实立交更复杂/应该多车道"而 fail——评审标准是**词汇表内最贴切的可用值**,不是对真实世界的完全还原。
- 行人cross为完整横穿初始道路，编译器按新XODR车道/路肩宽度在对侧路缘外停止；0.75米路缘净空是公开几何默认值，不是源报告事实。部分横穿或其他中途动作不能判作等价完整cross。 原文未说明起始侧时保留side=none，编译器默认从右侧横穿，并在C2XPedestrianCrossings中记录默认假设；不得把默认方向当作原文事实。
- sut.maneuver 词汇表为 straight/left/right/overtake_oncoming(无 stationary):`straight` 意为"沿本车道行驶",在 curve 道路上即沿弯道行驶,与 topology=curve **不矛盾**。
- 另有 overtake_solid_centerline，专门记录原文明确越实线/双黄线借道超车的机动，要求道路保留 solid 和对向车道；不能因该机动违反交通规则而要求改成 broken。overtake_oncoming 仍要求 broken。二者是SUT预期机动描述，实际由PCLA自主控制，不能据此宣称越线或事故已复现。必须有原文对应的越线证据。
- scene.control 指交通控制设施(traffic_light/stop_sign/yield/none/unknown),与"手动/自动驾驶模式"无关,报告里的 Conventional/Autonomous Mode 不映射到该字段。
- cruise 表示车辆/骑行者沿其车道持续前进，没有切入；junction_merge 表示相邻同向 NPC 与 ego 同向转弯，在同一出口车道汇合。普通 cut_in 只表达侧向换道，不能替代完整路口转弯；多阶段行为也不能用 junction_merge 一概代替。
- partial_lane_intrusion 仅描述 adjacent 车辆向ego车道部分侵入并保持；初始side、源车辆类别、前后关系和致因骑行者必须保留。默认偏移0.4车道宽、正弦目标曲线最大加速度0.8m/s²和1.5秒启动均为公开默认，不是源报告事实；实际车身跨线、时序、接触部位另验收。不能当作完整换道，也不能省略额外返回/停止动作。允许将复杂机动的报告AV作为NPC、选择其他有主动驾驶行为的车辆为PCLA SUT（包括货车），但须披露角色分配。路侧同向骑行者cruise会保持侧向偏移，缺省与参照车辆并排，不能替换为自行车切入。
- scene.collisions 是按原文时序排列的接触数组，可表达三车物理链式接触；单数 collision 是第一对的兼容字段，每对内部a/b无序。数组中必须保留全部接触。相同两车分离后再撞，仅列两对相同id并不能表示中间的减速、分离、再加速；缺这些行为仍属于high问题。碰撞后的被动物理响应无需强制坐标轨迹，但须留待真实运行验证。
- sequence 的steps允许drive/match_speed/brake/lane_change依次执行，when.condition为start/contact/after_previous/separated_and_target_stopped。纵向可表达行驶、接触后匹配速度、减速分离、等待目标停稳后再加速。lane_change的params允许direction(left/right)、lanes(1..5整数，默认1)、distance(默认20米)，在先drive后依次换道的条件下可表达横向往返；必须核对初始右后/左后相对位置以及跨越的车道数，不能缩为一次cut_in。不能施加人为碰撞或旋转。所有steps和目标须对应原文；匹配速度不保证持续接触。
- road.parking为可选left/right布尔对象，表示路缘停车带，不计入lanes.forward/backward；当前straight支持，左侧停车带紧邻同向最左车道时要求backward=0。停车带生成OpenDRIVE parking类型，允许NPC按明确的lane_change指令进入与返回，不能要求把它算为普通行车道。
- event_text是完整PDF原文（包括天气/光照勾选），scene示意图和description_zh只是待核对的模型结果，不能拿示意图的简述当成原始事故全文。深夜和天气应从event_text相应字段核对。
- NPC.relative_to指定位置参考车，省略才是ego。判断三车队列时必须沿引用链理解前后顺序，不能把所有ahead_same_lane都当成ego前方同一点。原文仅有被后撞后被动前推的参与者，不能被当成具有主动起步决策的ego；应保留被动物理响应并选择明确主动行驶的车辆。
- compiled_initial_relations显示编译器实际采用的纵向初始关系，正数为参照车前方、负数为后方，默认值不是原文事实。roadside只表达路侧，不自带后方含义；通常缺省gap为前方25米，路侧cyclist cruise缺省为并排0米。若原文明示从后方接近而roadside未保留负gap，即使description_zh声称在后方也应列high问题；不能依据示意图臆造源关系。相邻位置用long、路侧位置用gap保留前后关系，缺距离的数值假设须公开记录。

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
- 任一 issue 的 severity=high 或 medium → verdict=fail，先完成纠正再验收。
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
    "roadside":         (1.2, -1.0),
}

_KIND_COLOR = {
    "vehicle":    "#4f6ef7",
    "cyclist":    "#10a36a",
    "pedestrian": "#c47d00",
    "static":     "#777777",
}


def initial_relations(scene_seed):
    """Expose compiler defaults to QA; do not infer or modify source positions."""
    from tools.osc_blocks import resolve_position, _params
    from tools.scene_positions import ordered_npcs
    scene = (scene_seed or {}).get('scene') or {}
    relations = []
    for npc in ordered_npcs(scene.get('npcs', [])):
        pos, side = npc.get('position'), npc.get('side', 'none')
        reference = npc.get('relative_to', 'ego')
        if pos in ('cross', 'opposing_leg'):
            dx, dy = _POSITION_DXDY[pos]
            if side == 'left':
                dx = -abs(dx)
            relations.append({'id': npc['id'], 'reference': reference, 'dx': dx, 'dy': dy,
                              'longitudinal_m': None, 'note': 'RoadGraph route required; no metric distance inferred'})
            continue
        element = resolve_position(npc, -1, reference).get_element().find('RelativeLanePosition')
        ds = float(element.get('ds'))
        longitudinal = -ds if pos == 'oncoming' else ds
        dx = -float(element.get('dLane'))
        if pos == 'roadside':
            dx = -float(element.get('offset', 0)) / 3.5
        key = 'long' if pos == 'adjacent' else 'gap'
        default = key not in _params(npc)
        relations.append({'id': npc['id'], 'reference': reference, 'dx': dx,
                          'dy': -longitudinal / 25.0, 'compiler_ds_m': ds,
                          'longitudinal_m': longitudinal, 'longitudinal_is_default': default,
                          'note': f"{key}={longitudinal:+g}m ({'compiler default' if default else 'seed parameter'})"})
    return relations


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
    relations = initial_relations(scene_seed) if scene else []
    height = max(height, 380 + 48*len(relations))
    diagram_bottom = height - 42 - 48*len(relations)
    img = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(img)
    title_font = _load_font(13)
    label_font = _load_font(11)

    draw.rectangle([0, 0, width - 1, height - 1], outline="#999", width=1)
    draw.text((8, 6), "scene 初始关系示意 · 编译默认值另注，非等比例", fill="#444", font=title_font)

    cx, cy = width // 2, (36 + diagram_bottom) // 2
    if not scene:
        draw.text((cx - 60, cy), "(no scene)", fill="#999", font=title_font)
        return _png_bytes(img)

    # Cross-hair to imply road axes (vertical = same-lane axis, horizontal = cross axis).
    draw.line([(cx, 36), (cx, diagram_bottom)], fill="#e6e9f0", width=1)
    draw.line([(28, cy), (width - 28, cy)], fill="#e6e9f0", width=1)
    draw.text((cx + 4, 36), "前", fill="#c2c8d3", font=label_font)
    draw.text((cx + 4, diagram_bottom - 14), "后", fill="#c2c8d3", font=label_font)
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
    by_id = {npc['id']: npc for npc in scene.get('npcs') or []}
    positions = {'ego': (cx, cy)}
    for number, relation in enumerate(relations, 1):
        npc = by_id[relation['id']]
        pos = npc.get("position", "ahead_same_lane")
        side = npc.get("side", "none")
        rx, ry = positions[relation['reference']]
        x = rx + int(relation['dx'] * SCALE)
        y = ry + int(relation['dy'] * SCALE)
        x = max(40, min(width - 40, x))
        y = max(60, min(diagram_bottom - 24, y))
        positions[npc['id']] = (x, y)
        kind = npc.get("kind", "vehicle")
        block = (npc.get("behavior") or {}).get("block", "?")
        color = _KIND_COLOR.get(kind, "#4f6ef7")
        # Light connector showing relation to ego
        draw.line([(x, y), (rx, ry)], fill="#d8dce6", width=1)
        r = 15
        draw.ellipse([x - r, y - r, x + r, y + r], fill=color, outline="black", width=1)
        draw.text((x - 4, y - 6), str(number), fill="white", font=label_font)
        for index, label in enumerate((f"{number}. {npc['id']} | {kind}·{pos} @ {npc.get('relative_to', 'ego')}",
                                       f"block: {block}", relation['note'])):
            draw.text((8, diagram_bottom + 6 + (number-1)*48 + index*14), label, fill="#444", font=label_font)

    from tools.scene_contacts import contact_sequence
    contacts = contact_sequence(scene)
    contact_label = ' -> '.join(f"{c.get('a', '?')} ↔ {c.get('b', '?')}" for c in contacts)
    draw.text((8, height - 26),
              f"contacts: {contact_label}    "
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
    if any(not isinstance(issue, dict) or issue.get("severity") != "low" for issue in issues):
        verdict = "fail"
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
    max_tokens: int = 8000,
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
        "event_text": event_text or "",
        "road_seed": (road_seed or {}).get("road"),
        "road_description_zh": (road_seed or {}).get("description_zh"),
        "scene_seed": (scene_seed or {}).get("scene"),
        "scene_description_zh": (scene_seed or {}).get("description_zh"),
        "compiled_initial_relations": initial_relations(scene_seed),
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
    for attempt in range(3):
        from tools.model_transport import chat_completion
        resp = chat_completion(client,
            model=vlm_model, stream=False, temperature=0.0, max_tokens=max_tokens * (2 ** attempt),
            messages=messages,
        )
        raw = (resp.choices[0].message.content or "").strip()
        finish = resp.choices[0].finish_reason
        if raw and finish != "length":
            break
        if attempt < 2:
            print(f"[qa] VLM incomplete (finish={finish}, chars={len(raw)}); retrying with larger output budget", flush=True)
        elif finish == "length":
            raw = ""  # A truncated verdict cannot certify the source semantics.
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
    vlm_model: str | None = None,
    base_url: str | None = None,
    api_key_env: str | None = None,
) -> dict[str, Any]:
    """End-to-end: screenshot map → draw scene schematic → stitch → VLM → persist artifacts."""
    from tools.model_transport import model_settings
    defaults = model_settings()
    vlm_model = vlm_model or defaults["vlm_model"]
    base_url, api_key_env = base_url or defaults["base_url"], api_key_env or defaults["api_key_env"]
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
    relations_path = out_dir / f"qa_initial_relations_r{round_idx}.json"
    relations_path.write_text(json.dumps(initial_relations(scene_seed), ensure_ascii=False, indent=2)+'\n')
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
        "initial_relations": str(relations_path),
    }
    verdict["round"] = round_idx
    (out_dir / f"qa_report_r{round_idx}.json").write_text(
        json.dumps(verdict, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return verdict
