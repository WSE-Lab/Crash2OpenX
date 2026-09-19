#!/usr/bin/env python3
"""Package reviewed reconstructions and separate genuine PCLA recordings."""
import argparse
import csv
import hashlib
import json
import shutil
import zipfile
from datetime import datetime, timezone
from html import escape
from pathlib import Path
from urllib.parse import quote

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "outputs/validated_42_20260917"


def digest(path):
    with path.open("rb") as stream:
        h = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--final", action="store_true", help="Require all 42 calibrated PCLA runs and matching visual reviews.")
    args = parser.parse_args()
    out = args.output.resolve()
    if out.exists():
        raise SystemExit("Output must be a new directory; existing deliveries are immutable.")
    reviews = json.loads((BASE / "audit/reviewed_selection.json").read_text())
    pcla = {c["case_id"]: c for c in json.loads((BASE / "audit/pcla_counterfactuals.json").read_text())["cases"]}
    video_audits = {c["case_id"]: c for c in json.loads((BASE / "audit/pcla_video_integrity.json").read_text())}
    pcla_reviews_path = BASE / "audit/pcla_reviewed_selection.json"
    pcla_reviews = json.loads(pcla_reviews_path.read_text()) if pcla_reviews_path.exists() else {"complete": False, "cases": []}
    pcla_selections = {c["case_id"]: c for c in pcla_reviews["cases"]}
    if args.final:
        frozen = {c["case_id"] for c in json.loads((ROOT / "data/eval/baseline_42.json").read_text())["cases"]}
        if not pcla_reviews["complete"] or set(pcla_selections) != frozen or {c["case_id"] for c in reviews["cases"]} != frozen:
            raise SystemExit("Final delivery requires exact frozen membership and 42 completed PCLA visual reviews.")
    out.mkdir(parents=True)
    rows, cards = [], []
    for review in reviews["cases"]:
        cid = review["case_id"]
        destination = out / "cases" / cid
        shutil.copytree(BASE / "cases" / cid / "source", destination / "source")
        selected = review.get("reconstruction") or review.get("candidate")
        candidates = [a for a in pcla[cid]["attempts"] if a["runtime_pass"]]
        autonomous = max(candidates, key=lambda a: (BASE / a["path"] / "run/carla_rgb.mp4").stat().st_mtime) if candidates else None
        if autonomous:
            video_audit = video_audits[cid]
            if (video_audit["attempt"] != autonomous["attempt"] or
                    not all(video_audit.get(key) for key in ("full_video_decode_pass", "encoded_timestamps_complete", "contact_timestamp_coverage_pass", "termination_timestamp_coverage_pass")) or
                    digest(BASE / autonomous["path"] / "run/carla_rgb.mp4") != video_audit["sha256"]):
                raise RuntimeError(f"Selected PCLA recording lacks matching video validation: {cid}/{autonomous['attempt']}")
        if args.final:
            selected_review = pcla_selections[cid]
            if (not review.get("reconstruction") or not autonomous or not autonomous.get("integration_pass") or
                    selected_review.get("visual_review") != "reviewed" or selected_review["path"] != autonomous["path"] or
                    selected_review["video_sha256"] != video_audits[cid]["sha256"] or
                    any(digest(BASE / autonomous["path"] / name) != expected for name, expected in selected_review["input_sha256"].items())):
                raise RuntimeError(f"Final delivery lacks reviewed, calibrated full-window PCLA evidence: {cid}")
        if autonomous is None:
            failures = list((BASE / "cases" / cid).glob("pcla_v*/run_failure.json"))
            if failures:
                failed = max(failures, key=lambda p: p.stat().st_mtime).parent
                shutil.copytree(failed, destination / "pcla_failed", ignore=shutil.ignore_patterns("*.jpg", "*.png", "*.pyc", "__pycache__"))
        row = {"case_id": cid, "reconstruction_review": review["status"],
               "reconstruction_attempt": Path(selected).name if selected else "missing",
               "pcla_attempt": autonomous["attempt"] if autonomous else "missing",
               "pcla_inference_device": autonomous["inference_device"] if autonomous else "not_run",
               "pcla_autonomous_outcome": autonomous["autonomous_outcome"] if autonomous else "missing_valid_run",
               "pcla_runtime_pass": bool(autonomous),
               "pcla_integration_pass": autonomous.get("integration_pass", False) if autonomous else False,
               "pcla_route_completion_percent": autonomous.get("route_completion_percent") if autonomous else None,
               "pcla_recorded_simulation_duration_s": autonomous.get("recorded_simulation_duration_s") if autonomous else None,
               "pcla_offroad_seconds": autonomous.get("ego_offroad_seconds") if autonomous else None,
               "pcla_actual_collision_pairs": "; ".join("/".join(pair) for pair in autonomous.get("physical_collision_pairs", [])) if autonomous else "",
               "pcla_source_sequence_pass": autonomous["source_sequence_pass"] if autonomous else "not_run"}
        modes = []
        for mode, source in (("reconstruction", selected), ("pcla", autonomous["path"] if autonomous else None)):
            if not source:
                modes.append(f"<p>{mode}：尚未完成，见 manifest.json。</p>")
                continue
            source = BASE / source
            target = destination / mode
            shutil.copytree(source, target, ignore=shutil.ignore_patterns("*.jpg", "*.png", "*.pyc", "__pycache__", "_failed_*"))
            for name in ("map.xodr", "scenario.xosc", "run/carla_rgb.mp4", "run/demo.log", "run/events.jsonl", "run/sim_trace_raw.jsonl", "run/remote_run.json"):
                if not (target / name).is_file():
                    raise RuntimeError(f"Missing required evidence: {target / name}")
            if mode == "reconstruction" and review.get("reconstruction"):
                for name, expected in review["input_sha256"].items():
                    assert digest(target / name) == expected
                assert digest(target / "run/carla_rgb.mp4") == review["video_sha256"]
            prefix = target.relative_to(out).as_posix()
            def link(name, label):
                return f'<a href="{quote(prefix + "/" + name, safe="/")}">{label}</a>'
            links = [link("map.xodr", "OpenDRIVE"), link("scenario.xosc", "OpenSCENARIO"),
                     link("run/scenario.runtime.xosc", "实际运行 XOSC"), link("run/carla_rgb.mp4", "下载 MP4"),
                     link("run/demo.log", "客户端日志"), link("run/runner.log", "运行日志"),
                     link("run/carla_server.log.gz", "CARLA 服务日志片段"), link("run/measured_source_sequence.json", "动作检查"),
                     link("reconstruction.json", "重建假设")]
            title = "物理重建" if mode == "reconstruction" else "PCLA 自主运行"
            modes.append(f'<details><summary>{title}</summary><video controls preload="none" src="{quote(prefix + "/run/carla_rgb.mp4", safe="/")}"></video><nav>{" · ".join(links)}</nav></details>')
        (destination / "selection.json").write_text(json.dumps({"review": review, "pcla": autonomous}, ensure_ascii=False, indent=2))
        rows.append(row)
        status = "已审阅，有明确近似" if review.get("reconstruction") else "物理重建待修正"
        source_link = quote((destination / "source/report.pdf").relative_to(out).as_posix(), safe="/")
        narrative = json.loads((destination / "source/source_review.json").read_text())
        outcome_label = {"map_surface_exit_failure": "出界失败（运行已结束并保留证据）", "physical_collision_observed": "观察到实际碰撞", "no_hero_collision_within_recorded_window": "本观察时段未发生 AV 碰撞"}.get(row["pcla_autonomous_outcome"], "待补跑")
        cards.append(f'<article data-id="{escape(cid)}"><h2>{escape(cid)}</h2><p>{status} · PCLA：{outcome_label} · <a href="{source_link}">原始 PDF</a></p><p>{escape(narrative.get("av", ""))}<br>{escape(narrative.get("other", ""))}</p>{"".join(modes)}</article>')
    summary = {"created_at": datetime.now(timezone.utc).isoformat(), "case_count": len(rows),
               "reviewed_reconstruction_cases": sum(bool(c.get("reconstruction")) for c in reviews["cases"]),
               "pcla_runtime_passing_cases": sum(r["pcla_runtime_pass"] for r in rows),
               "pcla_integration_passing_cases": sum(r["pcla_integration_pass"] for r in rows),
               "global_limits": reviews["global_limits"], "cases": rows}
    summary["complete"] = summary["reviewed_reconstruction_cases"] == 42 and summary["pcla_runtime_passing_cases"] == 42
    summary["completion_scope"] = "Reviewed source reconstructions and recorded valid autonomous experiments; does not mean the autonomous agent passed every driving test."
    summary["pcla_inference_devices"] = {device: sum(r["pcla_inference_device"] == device for r in rows) for device in ("cuda", "cpu")}
    summary["pcla_outcome_counts"] = {outcome: sum(r["pcla_autonomous_outcome"] == outcome for r in rows) for outcome in sorted({r["pcla_autonomous_outcome"] for r in rows})}
    summary["pcla_offroad_counter_cases"] = sum((r["pcla_offroad_seconds"] or 0) > 0 for r in rows)
    summary["evidence_collection_complete"] = summary.pop("complete")
    summary["delivery_validation_complete"] = args.final
    summary["remaining_work"] = [] if args.final else ["Complete and review the calibrated PCLA reruns before final delivery."]
    (out / "manifest.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    with (out / "manifest.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    for folder in ("audit", "infra"):
        (out / folder).mkdir()
    for name in ("reviewed_selection.json", "pcla_counterfactuals.json", "video_integrity.json", "pcla_video_integrity.json", "contact_geometry_review.json", "full_recording_physics_review.json", "source_inventory.json"):
        shutil.copy2(BASE / "audit" / name, out / "audit" / name)
    for image in {p for c in reviews["cases"] for p in c.get("visual_evidence", [])}:
        shutil.copy2(BASE / image, out / image)
    if pcla_reviews_path.exists():
        shutil.copy2(pcla_reviews_path, out / "audit/pcla_reviewed_selection.json")
        for relative in {p for c in pcla_reviews["cases"] for p in c.get("images", []) + [c["panel"]]}:
            shutil.copy2(BASE / relative, out / relative)
    for name in ("runner.sh", "runner_cpu.sh", "runner_cpu_bounded.sh", "runner_cpu_gnss.sh", "carla_launch_config.json", "pcla_runtime_environment.txt", "runtime_versions.json", "interfuser_checkpoint.sha256", "gpu_fault_20260917_0624.txt", "cpu_model_probe.json", "device_selection_provenance.json", "interfuser_device.patch", "final_server_status.txt", "final_runtime_tests.log", "GNSS_COORDINATE_FIX.md", "navigation_telemetry_provenance.json", "navigation_telemetry.patch", "gnss_runtime_tests.log", "INITIAL_POSE_FIX.md", "initial_pose_before_fix_test.log", "initial_pose_runtime_tests.log"):
        if (BASE / "infra" / name).exists():
            shutil.copy2(BASE / "infra" / name, out / "infra" / name)
    for name in ("initial_pose_deployment.json", "initial_pose_preservation.patch"):
        shutil.copy2(BASE / "infra" / name, out / "infra" / name)
    shutil.copy2(BASE / "README.md", out / "RUNTIME_AND_LIMITATIONS.md")
    shutil.copy2(ROOT / "tools/replay_source_delivery.py", out / "reproduce.py")
    (out / "README.md").write_text(f'''# 42 个事故场景的 CARLA / PCLA 运行证据

状态：{"42 个物理重建与 PCLA 接口运行完成核验" if args.final else "阶段交付，任务尚未全部完成"}。
物理重建已审阅 {summary["reviewed_reconstruction_cases"]}/42；PCLA 真实自主运行证据完整 {summary["pcla_runtime_passing_cases"]}/42。
本包选定的 PCLA 出界失败 {summary["pcla_outcome_counts"].get("map_surface_exit_failure", 0)} 例。自主驾驶碰撞与避碰结果按实际发生情况记录。

打开 `index.html` 可逐个播放录像并查看 PDF、XODR、XOSC 和日志。`manifest.csv` / `manifest.json` 是逐场景状态。
每例 `reconstruction/` 与 `pcla/` 完全分开，前者 AV 使用物理轨迹控制器，后者 AV 使用真正的 InterFuser 控制。自主驾驶可能改变或避免原事故，不要求人为制造碰撞。

PCLA 推理设备：CUDA {summary["pcla_inference_devices"]["cuda"]} 例、CPU {summary["pcla_inference_devices"]["cpu"]} 例。GPU 在长批次后报告 Xid 31 故障；CPU 运行保留同一权重、网络、预处理、传感器和控制器，仅显式选择推理设备。逐例设备记录见清单，不将这批结果用于统一硬件的速度比较。

坐标适配后的 PCLA 完整观察时段验证通过 {summary["pcla_integration_passing_cases"]}/42。此验证要求保留初始位姿、GNSS 与路线坐标一致、正常完成预定观察时段且未掉出地图；不代表延长导航路线全部完成或无碰撞驾驶。接口中的 GNSS 适配保留实际测量及其噪声，说明见 `infra/GNSS_COORDINATE_FIX.md`。

其中 {summary["pcla_offroad_counter_cases"]} 例记录了非零离路时间，包括路边初始位置、路口内行为及模型实际选择，均未从清单中抹去。不要将本包解释为 42 次自主驾驶任务全部通过。

PCLA 可能发生来源报告之外的碰撞，例如 263 的追尾后还发生了 NPC 之间的接触。实际碰撞对象与离路计数均列入清单，未删除这些结果。PCLA 的来源动作布尔检查不等于整段自主行为与 PDF 完全一致；逐例审阅备注见 `audit/pcla_reviewed_selection.json`。

PCLA 的碰撞与出界是自主测试结果，不等于驾驶任务通过。新运行在 AV 实测 z 低于 -0.3m、离开有限地图表面时结束，并记录 `map_surface_exit_failure`；这是失败结果，未被改写为无碰撞成功。正常录像保存完整观察窗口，出界失败录像保存到失败终止点。导航延长仅改变导航任务，不覆盖模型输出。发光调试标线实验未用于最终选定录像。

本批包含人工来源核查与逐例修正，不可表述为原始 PDF 模型未经干预即达到 42/42。地图、车辆、时间、距离及原文未给出的速度均有近似。原文红绿灯阶段通过参与者行为表达，未验证信号灯识别。精确镜面、传感器和自行车架碰撞网格未实现。235 的标准卡车在接触后仍有约 22.6 度倾斜，未侧翻。

`run/source_validation.json` 是运行时的初步记录；最终状态以 `selection.json`、`run/measured_source_sequence.json` 与审阅清单为准。服务端日志是有范围上限的实际截取；原始日志和所有历史尝试在工作目录及服务器保留。

仿真观察时长见清单的 `pcla_recorded_simulation_duration_s`，由实际轨迹时间戳计算。原始 `summary.json` 的 `duration_seconds` 是运行耗费的墙钟时间；CPU 推理时通常长于仿真时长。初始落地阶段的三维速度与行驶平面速度分别保存在轨迹中，不应把落地的垂直速度当作车辆行驶速度。

录像源于 CARLA RGB 相机。`run/video_frame_timestamps.jsonl` 对应实际编码帧；旧原始时间戳存在多余行时没有修改原文件，也没有合成缺失时间戳。`SHA256SUMS` 覆盖本包全部文件，原始输入和录像也保留各自来源哈希。

274、065、638、032 的旧物理重建录像各有最后一帧缺少时间戳；所有来源碰撞帧均有时间戳可以核对，原始录像未删帧。

服务器配置、环境依赖和从项目重跑的命令见 `RUNTIME_AND_LIMITATIONS.md`。本包可离线审阅；要执行场景，需要该文件所列的 CARLA、PCLA 权重和本项目运行入口，并非仅用通用 ScenarioRunner 即可执行的独立包。

若要执行本包选定的原始输入而不重新生成场景，可运行 `python reproduce.py --project /path/to/crash2openx --case 135 --mode reconstruction --output /path/to/new_run`。PCLA 模式使用 `--mode pcla`，并按运行说明设置 `CARLA_REMOTE_RUNNER` 为 `runner_cpu_gnss.sh`；物理重建使用 `runner.sh`。每次重跑目录必须不同。
''', encoding="utf-8")
    status = "已验证运行证据" if args.final else "阶段运行证据 · 未全部完成"
    (out / "index.html").write_text(f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>42 场景 {status}</title><style>body{{max-width:1100px;margin:36px auto;padding:0 22px;background:#f7f8fa;color:#172432;font:16px/1.7 system-ui}}article{{background:white;border:1px solid #dce1e6;margin:20px 0;padding:22px}}h2{{font-size:20px;overflow-wrap:anywhere}}video{{width:100%;max-height:560px;background:#111}}details{{margin:12px 0}}summary{{cursor:pointer;font-weight:600}}a{{color:#126198}}input{{font:inherit;padding:8px}}.notice{{background:#fff1d4;padding:16px}}</style><h1>42 场景 {status}</h1><p>物理重建已审阅 {summary["reviewed_reconstruction_cases"]}/42 · PCLA 运行证据完整 {summary["pcla_runtime_passing_cases"]}/42</p><p class="notice">手工核查修正的物理重建与 PCLA 自主实验分别展示。存在道路、资产和未报告数值的近似；详见 <a href="README.md">说明</a> 与 <a href="manifest.csv">逐例清单</a>。</p><input id="search" aria-label="搜索编号" placeholder="搜索编号或名称">{"".join(cards)}<script>document.querySelector('#search').addEventListener('input',e=>document.querySelectorAll('article').forEach(a=>a.hidden=!a.dataset.id.toLowerCase().includes(e.target.value.toLowerCase())));</script></html>''', encoding="utf-8")
    files = sorted(p for p in out.rglob("*") if p.is_file())
    (out / "SHA256SUMS").write_text("".join(f"{digest(p)}  {p.relative_to(out).as_posix()}\n" for p in files))
    archive = out.with_suffix(".zip")
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
        for path in sorted(p for p in out.rglob("*") if p.is_file()):
            zf.write(path, Path(out.name) / path.relative_to(out))
    with zipfile.ZipFile(archive) as zf:
        assert zf.testzip() is None
    archive.with_suffix(".zip.sha256").write_text(f"{digest(archive)}  {archive.name}\n")
    print(json.dumps({"output": str(out), "zip": str(archive), "bytes": archive.stat().st_size,
                      "delivery_validation_complete": summary["delivery_validation_complete"], "reconstructions_reviewed": summary["reviewed_reconstruction_cases"],
                      "pcla_runtime_pass": summary["pcla_runtime_passing_cases"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
