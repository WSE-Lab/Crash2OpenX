#!/usr/bin/env python3
"""Refresh measured evidence for every frozen case without running CARLA."""
import json
import hashlib
import sys
from html import escape
from urllib.parse import quote
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.source_runtime_evidence import evaluate


def audit(base):
    frozen = json.loads((ROOT / "data/eval/baseline_42.json").read_text())["cases"]
    review_file = base / "audit/reviewed_selection.json"
    reviewed = {c["case_id"]: c for c in json.loads(review_file.read_text())["cases"]} if review_file.exists() else {}
    cases = []
    for item in frozen:
        cid = item["case_id"]
        attempts = []
        for path in sorted((base / "cases" / cid).glob("*/run/summary.json")):
            run, attempt = path.parent, path.parent.parent
            source_file = attempt / "reconstruction.json"
            if source_file.exists():
                source = json.loads(source_file.read_text())
                pairs = source["expected_ordered_collision_pairs"]
                mode = source["execution_mode"]
            elif (attempt / "scene_seed.json").exists():
                pairs = [["hero", "rear_car"]]
                mode = "pcla_autonomous_sut"
            else:
                continue
            try:
                measured = evaluate(run, cid[:3], pairs)
                (run / "measured_source_sequence.json").write_text(json.dumps(measured, indent=2))
                review = reviewed.get(cid, {})
                accepted = (review.get("reconstruction") == str(attempt.relative_to(base))
                            and measured["pass"] and review.get("status") == "reviewed_with_documented_approximations"
                            and hashlib.sha256((run / "carla_rgb.mp4").read_bytes()).hexdigest() == review.get("video_sha256")
                            and all(hashlib.sha256((attempt / name).read_bytes()).hexdigest() == digest
                                    for name, digest in review.get("input_sha256", {}).items()))
                attempts.append({"attempt": attempt.name, "mode": mode,
                                 "path": str(attempt.relative_to(base)),
                                 "has_rgb_video": (run / "carla_rgb.mp4").is_file(),
                                 "has_runtime_provenance": (run / "runtime_manifest.json").is_file(),
                                 "measured_sequence_pass": measured["pass"],
                                 "failed_checks": [k for k, v in measured["checks"].items() if not v],
                                 "metrics": measured.get("metrics", {}),
                                 "accepted": accepted,
                                 "review_status": "reviewed_with_documented_approximations" if accepted else "requires_source_and_visual_review"})
            except (KeyError, ValueError, OSError) as exc:
                attempts.append({"attempt": attempt.name, "mode": mode, "error": str(exc), "accepted": False})
        cases.append({"case_id": cid, "attempts": attempts,
                      "has_rgb_run": any(a.get("has_rgb_video") for a in attempts),
                      "has_measured_passing_run": any(a.get("has_rgb_video") and a.get("measured_sequence_pass") and a["mode"] == "scripted_physical_reconstruction" for a in attempts),
                      "has_pcla_rgb_run": any(a.get("has_rgb_video") and a["mode"] == "pcla_autonomous_sut" for a in attempts)})
    result = {"updated_at": datetime.now(timezone.utc).isoformat(), "case_count": len(cases),
              "rgb_cases": sum(c["has_rgb_run"] for c in cases),
              "measured_passing_cases": sum(c["has_measured_passing_run"] for c in cases),
              "pcla_rgb_cases": sum(c["has_pcla_rgb_run"] for c in cases),
              "accepted_cases": sum(any(a.get("accepted") for a in c["attempts"]) for c in cases),
              "note": "Reviewed counts refer to source behavior under documented reconstruction assumptions, not exact accident geometry or unassisted PDF conversion accuracy.",
              "cases": cases}
    destination = base / "audit/runtime_inventory.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, indent=2, ensure_ascii=False))
    write_index(base, result)
    return result


def write_index(base, result):
    cards = []
    selection_file = base / "audit/reviewed_selection.json"
    selections = {c["case_id"]: c for c in json.loads(selection_file.read_text())["cases"]} if selection_file.exists() else {}
    reviews = {c["id"]: c for c in json.loads((ROOT / "data/eval/baseline_42_source_review.json").read_text())["cases"]}
    for case in result["cases"]:
        cid = case["case_id"]
        available = [a for a in case["attempts"] if a.get("has_rgb_video") and a["mode"] == "scripted_physical_reconstruction"]
        if not available:
            continue
        chosen = max(available, key=lambda a: (a.get("accepted", False), a.get("measured_sequence_pass", False), (base / a["path"] / "run/carla_rgb.mp4").stat().st_mtime))
        selected_path = selections.get(cid, {}).get("reconstruction") or selections.get(cid, {}).get("candidate")
        chosen = next((a for a in available if a["path"] == selected_path), chosen)
        root = chosen["path"]
        def link(path, title):
            return f'<a href="{quote(path, safe="/")}">{escape(title)}</a>'
        mode = "PCLA 自主控制" if chosen["mode"] == "pcla_autonomous_sut" else "物理重建控制"
        status = "已审阅 · 存在已记录近似" if chosen.get("accepted") else ("动作检查通过 · 待完整审阅" if chosen["measured_sequence_pass"] else "仍需修正")
        failure = ", ".join(chosen.get("failed_checks", []))
        source = reviews[cid[:3]]
        links = [link(root + "/map.xodr", "OpenDRIVE"), link(root + "/scenario.xosc", "OpenSCENARIO 输入"),
                 link(root + "/run/scenario.runtime.xosc", "运行时 OpenSCENARIO"),
                 link(root + "/run/carla_rgb.mp4", "MP4"), link(root + "/run/demo.log", "CARLA 客户端日志"),
                 link(root + "/run/runner.log", "运行日志"),
                 link(root + "/run/measured_source_sequence.json", "动作检查与测量"),
                 link("cases/" + cid + "/source/report.pdf", "原始 PDF")]
        cards.append(f'''<article data-id="{escape(cid)}" data-pass="{str(chosen["measured_sequence_pass"]).lower()}">
<h2>{escape(cid)}</h2><p class="status">{status} · {mode} · {escape(chosen["attempt"])}</p>
<p>{escape(source.get("av", ""))}<br>{escape(source.get("other", ""))}</p>
<video controls preload="none" src="{quote(root + "/run/carla_rgb.mp4", safe="/")}"></video>
<nav>{" · ".join(links)}</nav><p class="failed">{escape(failure)}</p>
<details><summary>全部物理重建和 PCLA 运行记录</summary><ul>{"".join('<li>' + link(a['path'] + '/run/carla_rgb.mp4', a['attempt'] + ' / ' + a['mode']) + '</li>' for a in case['attempts'] if a.get('has_rgb_video'))}</ul></details></article>''')
    html = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>42 场景 CARLA 验证 — 进行中</title><style>
body{{font:16px/1.6 system-ui,sans-serif;max-width:1120px;margin:36px auto;padding:0 24px;background:#f7f8fa;color:#19232d}}
h1{{font-size:32px}}h2{{font-size:20px;overflow-wrap:anywhere}}article{{background:white;border:1px solid #d7dfe5;padding:24px;margin:24px 0}}
video{{width:100%;max-height:560px;background:#111}}a{{color:#14568c}}nav{{margin-top:14px}}.status{{font-weight:600}}.failed{{color:#a52b2b}}
input,select{{font:inherit;padding:8px 12px;margin:8px 12px 8px 0;max-width:100%}}.note{{background:#fff0d5;padding:16px}}small{{color:#5e6872}}
</style><h1>42 场景 CARLA 验证 · 进行中</h1>
<p>已有真实录像 <b>{result["rgb_cases"]}/42</b> · 动作检查通过 <b>{result["measured_passing_cases"]}/42</b> · 已审阅 <b>{result["accepted_cases"]}/42</b> · 已有 PCLA 自主运行 <b>{result["pcla_rgb_cases"]}/42</b></p>
<p class="note">本页是过程索引，尚未表示全部验收。物理重建与 PCLA 自主控制分开标记。车辆资产、传感器外形、未报告的速度/距离及代表性道路均有近似；生成道路缺少真实街景与可靠信号灯。MP4 来自 CARLA RGB 相机。</p>
<small>更新：{escape(result["updated_at"])} · 选择优先通过动作检查的运行；所有历史尝试仍保留。</small><p>
<input id="search" placeholder="搜索编号或名称" aria-label="搜索场景"><select id="filter" aria-label="按状态筛选"><option value="all">全部</option><option value="true">动作检查通过</option><option value="false">仍需修正</option></select></p>
{"".join(cards)}<script>
function filter(){{const q=document.querySelector('#search').value.toLowerCase(),v=document.querySelector('#filter').value;document.querySelectorAll('article').forEach(a=>a.hidden=!a.dataset.id.toLowerCase().includes(q)||(v!=='all'&&a.dataset.pass!==v));}}
document.querySelector('#search').addEventListener('input',filter);document.querySelector('#filter').addEventListener('change',filter);
</script></html>'''
    (base / "index.html").write_text(html, encoding="utf-8")


if __name__ == "__main__":
    result = audit(ROOT / "outputs/validated_42_20260917")
    print(json.dumps({k: v for k, v in result.items() if k != "cases"}, indent=2))
