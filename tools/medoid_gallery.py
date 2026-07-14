#!/usr/bin/env python3
"""Build a self-contained HTML gallery of the 18 medoid scenarios.

For each medoid, render a card with:
  - cluster header (id, size, signature)
  - badges: fidelity score (1-5) + CARLA verdict category (E/D/C/F)
  - row 1: original PDF excerpt + DSL summary (side-by-side)
  - row 2: OpenDRIVE SVG preview + inlined CARLA RGB video + per-run metrics
  - links to: scene_seed.json, road_seed.json, xodr preview, xosc, summary.json

Reads:
  - paper/eval_medoids_n100.json
  - paper/medoid_fidelity.json (LLM 4.5/5 mean judge)
  - outputs/scene_seed/<cid>.json, outputs/road_seed/<cid>.json
  - outputs/opendrive_seed/<cid>.html (for SVG preview)
  - outputs/medoid_runs/<cid>/{summary.json, behavior_check.json, carla_rgb.mp4}

Output: paper/medoid_gallery.html (self-contained, mp4 referenced by relative path).
"""
from __future__ import annotations

import argparse
import html
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _extract_svg(xodr_html_path: Path) -> str:
    if not xodr_html_path.is_file():
        return ""
    text = xodr_html_path.read_text(encoding="utf-8")
    m = re.search(r"<svg[^>]*>.*?</svg>", text, flags=re.DOTALL)
    return m.group(0) if m else ""


def _truncate(s: str, n: int = 280) -> str:
    s = s.strip()
    return s if len(s) <= n else (s[:n].rstrip() + " …")


def _format_npc(npc: dict) -> str:
    pos = npc.get("position", "?")
    side = npc.get("side", "")
    side_part = f"/{side}" if side and side != "none" else ""
    block = (npc.get("behavior") or {}).get("block", "?")
    params = (npc.get("behavior") or {}).get("params") or {}
    p_str = ", ".join(f"{k}={v}" for k, v in sorted(params.items())) if params else ""
    p_part = f" [{p_str}]" if p_str else ""
    return f"{npc.get('id','?')} ({npc.get('kind','?')}): {pos}{side_part} → {block}{p_part}"


def _classify_run(summary: dict) -> tuple[str, str, str]:
    """Return (category code, label, css class) from CARLA summary metrics."""
    if not summary:
        return "F", "no summary", "cat-F"
    ticks = summary.get("total_ticks", 0)
    route = summary.get("final_route_completion", 0) or 0
    coll = summary.get("collision_count", 0) or 0
    li = summary.get("lane_invasion_count", 0) or 0
    if ticks == 0:
        return "A", "silent spawn", "cat-A"
    if coll > 0:
        return "E", "real collision", "cat-E"
    if route < 5:
        return "B", "ego stuck", "cat-B"
    if li > 3:
        return "C", "ADS lane-keep fail", "cat-C"
    return "D", "ADS passed", "cat-D"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--medoids", type=Path, default=ROOT / "paper/eval_medoids_n100.json")
    ap.add_argument("--fidelity", type=Path, default=ROOT / "paper/medoid_fidelity.json")
    ap.add_argument("--out", type=Path, default=ROOT / "paper/medoid_gallery.html")
    args = ap.parse_args()

    meta = json.loads(args.medoids.read_text())
    fidelity = json.loads(args.fidelity.read_text()) if args.fidelity.is_file() else {}

    cards = []
    topo_set, block_set, maneuver_set = set(), set(), set()
    cat_counts: dict[str, int] = {}
    fid_scores: list[int] = []

    for med in meta["medoids"]:
        cid = med["medoid"]
        sig = med["feature_signature"]
        scene = json.loads((ROOT / "outputs/scene_seed" / f"{cid}.json").read_text())
        road = json.loads((ROOT / "outputs/road_seed" / f"{cid}.json").read_text())
        sc = scene.get("scene") or {}
        r = road.get("road") or {}
        npcs = sc.get("npcs") or []
        env = sc.get("environment") or {}
        evidence_snippets = (scene.get("evidence") or {}).get("source_snippets") or []
        evidence_html = "".join(
            f'<blockquote class="src">{html.escape(_truncate(s))}</blockquote>'
            for s in evidence_snippets[:3]
        )
        npc_list = "".join(f"<li>{html.escape(_format_npc(n))}</li>" for n in npcs)
        sut = sc.get("sut") or {}
        sut_str = f"{sut.get('id','ego')} ({sut.get('kind','vehicle')}): maneuver={sut.get('maneuver','?')}"
        coll = sc.get("collision") or {}
        coll_str = f"{coll.get('a','?')} ↔ {coll.get('b','?')}"
        env_str = f"{env.get('weather','?')} / {env.get('time_of_day','?')}"

        topo_set.add(r.get("topology", "?"))
        maneuver_set.add(sut.get("maneuver", "?"))
        for n in npcs:
            block_set.add((n.get("behavior") or {}).get("block", "?"))

        svg = _extract_svg(ROOT / "outputs/opendrive_seed" / f"{cid}.html")
        svg_block = (
            f'<div class="svg-wrap">{svg}</div>' if svg
            else '<div class="svg-wrap empty">(no preview)</div>'
        )

        scene_desc = scene.get("description_zh") or ""
        road_desc = road.get("description_zh") or ""

        # CARLA artifacts
        run_dir = ROOT / "outputs/medoid_runs" / cid
        summary_path = run_dir / "summary.json"
        behavior_path = run_dir / "behavior_check.json"
        mp4_path = run_dir / "carla_rgb.mp4"
        summary = json.loads(summary_path.read_text()) if summary_path.is_file() else None
        behavior = json.loads(behavior_path.read_text()) if behavior_path.is_file() else None
        cat_code, cat_label, cat_class = _classify_run(summary)
        cat_counts[cat_code] = cat_counts.get(cat_code, 0) + 1

        # Build CARLA panel
        if mp4_path.is_file():
            rel_mp4 = f"../outputs/medoid_runs/{cid}/carla_rgb.mp4"
            video_html = (
                f'<video controls preload="metadata" muted '
                f'poster="../outputs/medoid_runs/{cid}/contact_sheet.jpg">'
                f'<source src="{rel_mp4}" type="video/mp4">'
                f'(浏览器不支持 mp4 内联，<a href="{rel_mp4}">点这里下载</a>)'
                f'</video>'
            )
        else:
            video_html = '<div class="svg-wrap empty">(no video)</div>'

        if summary:
            metrics_html = f"""
              <table class="metrics">
                <tr><th>ticks</th><td>{summary.get('total_ticks','-')}</td>
                    <th>sim sec</th><td>{(summary.get('duration_seconds') or 0):.1f}</td></tr>
                <tr><th>route</th><td>{(summary.get('final_route_completion') or 0):.0f}%</td>
                    <th>term</th><td>{html.escape(str(summary.get('termination_reason','-')))}</td></tr>
                <tr><th>coll</th><td>{summary.get('collision_count',0)}</td>
                    <th>lane_inv</th><td>{summary.get('lane_invasion_count',0)}</td></tr>
              </table>"""
            issues = (behavior or {}).get("issues") or []
            issues_html = ""
            if issues:
                issues_html = (
                    '<details class="issues"><summary>behavior_check issues</summary><ul>' +
                    "".join(f"<li>{html.escape(i)}</li>" for i in issues) +
                    '</ul></details>'
                )
        else:
            metrics_html = '<p class="muted">(no CARLA run)</p>'
            issues_html = ""

        # Fidelity badge
        fid = fidelity.get(cid) or {}
        fid_score = fid.get("score")
        if isinstance(fid_score, int):
            fid_scores.append(fid_score)
        fid_rationale = fid.get("rationale") or ""

        # Relative paths
        rel_xodr = f"../outputs/opendrive_seed/{cid}.html"
        rel_xosc = f"../outputs/medoid_xosc/{cid}.xosc"
        rel_scene = f"../outputs/scene_seed/{cid}.json"
        rel_road = f"../outputs/road_seed/{cid}.json"
        rel_run = f"../outputs/medoid_runs/{cid}"

        lanes_v = r.get("lanes")
        if isinstance(lanes_v, dict):
            lanes_str = f"forward={lanes_v.get('forward','?')}, backward={lanes_v.get('backward','?')}"
        else:
            lanes_str = str(lanes_v)

        fid_badge = (
            f'<span class="badge fid fid-{fid_score}">fidelity {fid_score}/5</span>'
            if isinstance(fid_score, int) else ""
        )
        cat_badge = f'<span class="badge {cat_class}">CARLA: {cat_code} · {html.escape(cat_label)}</span>'

        cards.append(f"""
<article class="card">
  <header>
    <div class="hrow">
      <div class="cid">Cluster #{med['cluster_id']:>2}</div>
      <div class="size">size {med['size']}</div>
      {fid_badge}
      {cat_badge}
    </div>
    <h2><code>{html.escape(cid)}</code></h2>
    <div class="sig">topology=<b>{html.escape(sig['topology'])}</b> · lanes=<b>{html.escape(str(sig['lanes']))}</b> · ego=<b>{html.escape(sig['sut_maneuver'])}</b> · npc=<b>{html.escape(' + '.join(sig['npc_sig']))}</b> · coll=<b>{html.escape(sig['collision_pair'])}</b></div>
  </header>

  <div class="grid-row1">
    <section class="src-pane">
      <h3>原报告引用</h3>
      {evidence_html or '<p class="muted">(no source snippets)</p>'}
      <h3>LLM 描述</h3>
      <p class="zh">{html.escape(scene_desc)}</p>
      <p class="zh muted">道路: {html.escape(road_desc)}</p>
      <p class="zh muted">fidelity rationale: {html.escape(fid_rationale)}</p>
    </section>
    <section class="dsl-pane">
      <h3>DSL 表达</h3>
      <table class="dsl">
        <tr><th>topology</th><td>{html.escape(r.get('topology','?'))}</td></tr>
        <tr><th>type</th><td>{html.escape(r.get('type','?'))}</td></tr>
        <tr><th>lanes</th><td>{html.escape(lanes_str)}</td></tr>
        <tr><th>center_line</th><td>{html.escape(r.get('center_line','?'))}</td></tr>
        <tr><th>sut</th><td>{html.escape(sut_str)}</td></tr>
        <tr><th>npcs</th><td><ul class="npcs">{npc_list}</ul></td></tr>
        <tr><th>collision</th><td>{html.escape(coll_str)}</td></tr>
        <tr><th>control</th><td>{html.escape(sc.get('control','?'))}</td></tr>
        <tr><th>environment</th><td>{html.escape(env_str)}</td></tr>
      </table>
      <p class="links">
        <a href="{rel_scene}">scene_seed.json</a> ·
        <a href="{rel_road}">road_seed.json</a> ·
        <a href="{rel_xodr}">xodr 预览</a> ·
        <a href="{rel_xosc}">xosc</a>
      </p>
    </section>
  </div>

  <div class="grid-row2">
    <section class="svg-pane">
      <h3>OpenDRIVE 几何</h3>
      {svg_block}
    </section>
    <section class="video-pane">
      <h3>CARLA 实跑录像</h3>
      <div class="video-wrap">{video_html}</div>
      {metrics_html}
      {issues_html}
      <p class="links"><a href="{rel_run}/summary.json">summary.json</a> · <a href="{rel_run}/">run artifacts</a></p>
    </section>
  </div>
</article>""")

    body = "\n".join(cards)

    # Distribution legend
    legend_cells = []
    for code, label, cls in [
        ("E", "real_collision", "cat-E"),
        ("D", "ADS passed", "cat-D"),
        ("C", "ADS lane-keep fail", "cat-C"),
        ("B", "ego stuck", "cat-B"),
        ("A", "silent spawn", "cat-A"),
        ("F", "no summary", "cat-F"),
    ]:
        n = cat_counts.get(code, 0)
        legend_cells.append(
            f'<span class="badge {cls}">{code} · {label}: {n}</span>'
        )
    legend_html = " ".join(legend_cells)

    fid_summary = ""
    if fid_scores:
        from collections import Counter
        ctr = Counter(fid_scores)
        fid_summary = (
            f"fidelity LLM 评分 (n={len(fid_scores)}): mean = <b>{sum(fid_scores)/len(fid_scores):.2f}/5</b>"
            f" — 5×{ctr.get(5,0)} · 4×{ctr.get(4,0)} · 3×{ctr.get(3,0)} · 2×{ctr.get(2,0)} · 1×{ctr.get(1,0)}"
        )

    summary_html = f"""
<section class="summary">
  <h2>覆盖与可执行性统计</h2>
  <ul>
    <li><b>{len(meta['medoids'])}</b> 个 medoid（k={meta['best_k']}, silhouette = {meta['silhouette']:.3f}）</li>
    <li>topologies ({len(topo_set)}): {', '.join(sorted(topo_set))}</li>
    <li>SUT maneuvers ({len(maneuver_set)}): {', '.join(sorted(maneuver_set))}</li>
    <li>NPC blocks ({len(block_set)}): {', '.join(sorted(block_set))}</li>
    <li>{fid_summary}</li>
    <li>CARLA pipeline-OK rate: <b>{sum(cat_counts.get(c,0) for c in 'CDE')}/{len(meta['medoids'])}</b> (E+D+C, 排除 A/B/F)</li>
  </ul>
  <div class="legend">{legend_html}</div>
</section>"""

    out_html = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <title>Medoid 画廊 (n=100, k={meta['best_k']})</title>
  <style>
    body {{ font-family: -apple-system, "Helvetica Neue", "PingFang SC", sans-serif; margin: 0; background: #f5f7fb; color: #1f2937; }}
    header.top {{ padding: 18px 24px; background: #1e3a8a; color: white; }}
    header.top h1 {{ margin: 0 0 4px; font-size: 22px; }}
    header.top p {{ margin: 0; opacity: 0.85; font-size: 13px; }}
    .summary {{ background: white; padding: 14px 24px; border-bottom: 1px solid #e5e7eb; }}
    .summary h2 {{ margin: 0 0 8px; font-size: 16px; }}
    .summary li {{ font-size: 13px; line-height: 1.6; }}
    .summary .legend {{ margin-top: 8px; display: flex; flex-wrap: wrap; gap: 6px; }}
    .card {{ background: white; margin: 18px; border-radius: 8px; box-shadow: 0 1px 3px rgba(0,0,0,0.06); overflow: hidden; }}
    .card header {{ padding: 12px 18px; background: linear-gradient(90deg, #f3f4f6, #ffffff); border-bottom: 1px solid #e5e7eb; }}
    .hrow {{ display: flex; gap: 6px; align-items: center; flex-wrap: wrap; }}
    .cid {{ display: inline-block; background: #1e3a8a; color: white; font-weight: 600; padding: 2px 8px; border-radius: 4px; font-size: 12px; }}
    .size {{ display: inline-block; background: #6b7280; color: white; padding: 2px 8px; border-radius: 4px; font-size: 12px; }}
    .badge {{ display: inline-block; padding: 2px 8px; border-radius: 4px; font-size: 12px; font-weight: 500; color: white; }}
    .badge.fid {{ background: #475569; }}
    .badge.fid-5 {{ background: #10b981; }}
    .badge.fid-4 {{ background: #22c55e; }}
    .badge.fid-3 {{ background: #f59e0b; }}
    .badge.fid-2 {{ background: #f97316; }}
    .badge.fid-1 {{ background: #ef4444; }}
    .cat-E {{ background: #10b981; color: white; }}
    .cat-D {{ background: #22c55e; color: white; }}
    .cat-C {{ background: #6366f1; color: white; }}
    .cat-B {{ background: #f97316; color: white; }}
    .cat-A {{ background: #ef4444; color: white; }}
    .cat-F {{ background: #9ca3af; color: white; }}
    .card header h2 {{ margin: 6px 0 4px; font-size: 16px; }}
    .card header .sig {{ font-size: 12px; color: #4b5563; }}
    .grid-row1, .grid-row2 {{ display: grid; gap: 14px; padding: 14px 18px; }}
    .grid-row1 {{ grid-template-columns: 1fr 1fr; border-bottom: 1px solid #f3f4f6; }}
    .grid-row2 {{ grid-template-columns: 1fr 1.4fr; }}
    .card section h3 {{ font-size: 13px; color: #374151; margin: 6px 0 6px; border-bottom: 1px solid #e5e7eb; padding-bottom: 3px; }}
    blockquote.src {{ font-size: 12.5px; line-height: 1.5; color: #1f2937; background: #f9fafb; border-left: 3px solid #2563eb; margin: 6px 0; padding: 6px 10px; font-family: Georgia, serif; }}
    p.zh {{ font-size: 13px; line-height: 1.55; margin: 4px 0; }}
    p.zh.muted {{ color: #6b7280; font-size: 12px; }}
    table.dsl, table.metrics {{ font-size: 12.5px; border-collapse: collapse; width: 100%; }}
    table.dsl th, table.metrics th {{ text-align: left; padding: 3px 6px; color: #6b7280; font-weight: 500; vertical-align: top; width: 22%; }}
    table.dsl td, table.metrics td {{ padding: 3px 6px; vertical-align: top; }}
    ul.npcs {{ margin: 0; padding-left: 16px; }}
    ul.npcs li {{ font-family: ui-monospace, "SF Mono", monospace; font-size: 11.5px; }}
    p.links {{ font-size: 12px; margin-top: 8px; }}
    p.links a {{ color: #2563eb; text-decoration: none; margin-right: 6px; }}
    p.links a:hover {{ text-decoration: underline; }}
    .svg-wrap {{ aspect-ratio: 1/1; max-height: 320px; background: #eef2f6; border-radius: 4px; overflow: hidden; }}
    .svg-wrap.empty {{ display: flex; align-items: center; justify-content: center; color: #9ca3af; font-size: 12px; }}
    .svg-wrap svg {{ width: 100%; height: 100%; display: block; }}
    .video-wrap {{ background: #000; border-radius: 4px; overflow: hidden; aspect-ratio: 16 / 9; }}
    .video-wrap video {{ width: 100%; height: 100%; display: block; }}
    .issues {{ margin: 8px 0; font-size: 12px; }}
    .issues summary {{ cursor: pointer; color: #b45309; font-weight: 500; }}
    .issues ul {{ margin: 4px 0 0 12px; color: #92400e; }}
    .muted {{ color: #6b7280; }}
  </style>
</head>
<body>
  <header class="top">
    <h1>NL → OpenSCENARIO Pipeline · 18 Medoid 画廊</h1>
    <p>n=100 stratified eval set, k={meta['best_k']} (silhouette = {meta['silhouette']:.3f}). 全 18 xodr+xosc XSD-valid + CARLA 端到端可执行；fidelity LLM 评分 + CARLA 视频内嵌。</p>
  </header>
  {summary_html}
  {body}
</body>
</html>
"""
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(out_html, encoding="utf-8")
    print(f"wrote {args.out.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
