#!/usr/bin/env python3
"""Package explicitly reviewed, measured ADS tests into a portable showcase."""
import argparse
import csv
from html import escape
import json
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.run_fresh_framework_batch import digest, write


def package(base, output):
    reviews = json.loads((base/'visual_reviews.json').read_text())
    selected = [r for r in reviews if r.get('accepted')]
    if len({r['case_id'] for r in selected}) < 10:
        raise ValueError('Need at least ten distinct visually accepted cases')
    output.mkdir(parents=True, exist_ok=False)
    records, cards, clips = [], [], []
    for review in selected:
        source = base/review['case_id']/review['profile']
        quality = json.loads((source/'quality_review.json').read_text())
        if not quality['candidate_for_visual_review']:
            raise ValueError('Mechanical gate failed: '+review['case_id'])
        for name, expected in review['reviewed_sha256'].items():
            if digest(source/name) != expected:
                raise ValueError('Visually reviewed artifact changed: '+str(source/name))
        target = output/'cases'/review['case_id']
        target.mkdir(parents=True)
        for name in ['map.xodr', 'map.html', 'scenario.xosc', 'scene_seed.json', 'manifest.json',
                     'quality_review.json', 'clip_provenance.json', 'contact_sheet.jpg',
                     'map_and_response.png', 'map_overview.png', 'interaction.mp4']:
            if (source/name).is_file():
                shutil.copy2(source/name, target/name)
        originals = target/'source'
        originals.mkdir()
        for name, renamed in [('source.pdf', 'report.pdf'), ('source_source_text.txt', 'source_text.txt'),
                              ('source_scene_seed.json', 'scene_seed.json'), ('source_road_seed.json', 'road_seed.json'),
                              ('source_source_extraction.json', 'source_extraction.json'), ('source_road_generation.json', 'road_generation.json')]:
            shutil.copy2(source/name, originals/renamed)
        evidence = target/'evidence'
        evidence.mkdir()
        for name in ['carla_rgb.mp4', 'scenario.runtime.xosc', 'summary.json', 'sim_trace_raw.jsonl',
                     'frame_states.jsonl', 'events.jsonl', 'demo.log', 'runner.log', 'runtime_manifest.json',
                     'remote_run.json', 'behavior_check.json', 'run_scene_wrapper.log']:
            if (source/'run'/name).is_file():
                shutil.copy2(source/'run'/name, evidence/name)
        shutil.copytree(source/'run/runtime_code', evidence/'runtime_code')
        shutil.copytree(source/'run/ads_review', evidence/'ads_review')
        (evidence/'rgb_frames').mkdir()
        for name in ['camera.json', 'timestamps.jsonl']:
            shutil.copy2(source/'run/rgb_frames'/name, evidence/'rgb_frames'/name)
        native = quality['native_map']
        shutil.copy2(native['check_path'], evidence/'roadgraph_selfcheck.json')
        interactions = [i for i in quality['interactions'] if i['candidate_pass']]
        main = min(interactions, key=lambda i:i['min_clearance_m'])
        record = {'case_id': review['case_id'], 'profile': review['profile'], 'title': review['title'],
                  'test_value': review['test_value'], 'observed_outcome': review['outcome'],
                  'min_clearance_m': round(main['min_clearance_m'], 3),
                  'min_positive_ttc_s': round(main['min_positive_ttc_s'], 3) if main['min_positive_ttc_s'] is not None else '',
                  'physical_contact': main['physical_contact'],
                  'ads_lane_departure_samples': main['ads_lane_departure_samples'],
                  'episode_termination': quality['termination'], 'source_reconstruction_accepted': False,
                  'scene': str((target/'scenario.xosc').relative_to(output)),
                  'map': str((target/'map.xodr').relative_to(output)),
                  'clip': str((target/'interaction.mp4').relative_to(output)), 'limitations': review['limitations']}
        records.append(record)
        prefix = 'cases/'+review['case_id']+'/'
        cards.append(f'''<article id="case-{escape(review['case_id'][:3])}">
<h2>{escape(review['case_id'][:3])} · {escape(review['title'])}</h2>
<p>{escape(review['test_value'])}</p>
<img src="{prefix}contact_sheet.jpg" alt="四帧同步实测画面">
<p><a href="{prefix}interaction.mp4">播放原速连续视频</a></p>
<p><strong>实测：</strong>{escape(review['outcome'])}</p>
<p>动作后最多 6 秒的测量窗口：最小净空 {record['min_clearance_m']:.2f} m · 最小正 TTC {str(record['min_positive_ttc_s'])+' s' if record['min_positive_ttc_s'] != '' else '无有限值'}。接触结果按完整运行记录；若发生接触，视频片段包含首次接触。</p>
<details><summary>地图、轨迹与证据</summary><img src="{prefix}map_and_response.png" alt="生成地图与实测车辆轨迹">
<p><a href="{prefix}map_overview.png">完整地图</a> · <a href="{prefix}map.xodr">OpenDRIVE</a> · <a href="{prefix}scenario.xosc">OpenSCENARIO</a> · <a href="{prefix}source/report.pdf">事故来源</a> · <a href="{prefix}evidence/carla_rgb.mp4">完整原视频</a> · <a href="{prefix}evidence/demo.log">运行日志</a> · <a href="{prefix}quality_review.json">测量结果</a></p></details>
<p class="note">{escape(review['limitations'])}</p></article>''')
        clips.append(target/'interaction.mp4')
        (target/'README.md').write_text(f"# {review['title']}\n\n{review['test_value']}\n\n实测：{review['outcome']}\n\n局限：{review['limitations']}\n\n主入口为本目录 scenario.xosc，地图引用为 map.xodr。source/ 为原始事故材料，evidence/ 保存完整实跑视频、日志、轨迹和运行时代码。该例是事故启发的 ADS 测试变体，速度、触发窗口和间距的变化见 manifest.json；没有完成原始事故逐细节重建验收。\n")
    with (output/'case_index.csv').open('w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0]))
        writer.writeheader(); writer.writerows(records)
    write(output/'selection.json', {'distinct_cases':len(records), 'cases':records, 'visual_reviews':selected})
    shutil.copy2(base/'inventory_42.json', output/'inventory_42.json')
    shutil.copy2(base/'quality_index.json', output/'all_attempts_review.json')
    shutil.copy2(base/'selection_notes.md', output/'selection_notes.md')
    snapshot = output/'implementation_snapshot'
    for name in ['tools/expand_ads_batch.py', 'tools/ads_junction_timing.py',
                 'tools/ads_stress.py', 'tools/ads_trigger.py', 'tools/osc_blocks.py',
                 'tools/replay_scene_tools.py', 'tools/review_ads_expansion.py',
                 'tools/review_ads_stress.py', 'tools/build_ads_review_assets.py',
                 'tools/plot_ads_case.py', 'tools/package_ads_showcase.py',
                 'tools/validate_ads_showcase.py', 'tools/refresh_ads_map.py',
                 'runner/src/open_scenario.py', 'runner/runtime_overrides/npc_vehicle_control.py',
                 'tests/test_ads_route_generalization.py', 'tests/test_ads_approach_spacing.py',
                 'tests/test_ads_junction_timing.py', 'tests/test_ads_junction_waiting.py',
                 'schemas/scene_seed_schema_v2.md', 'pyproject.toml', 'uv.lock']:
        target = snapshot/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, target)
    shutil.copy2(ROOT/'outputs/ads_expansion_final_tests.log', output/'tests.log')
    html = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Crash2OpenX · {len(records)} 例 ADS 测试</title><style>
body{{font:16px/1.7 system-ui,sans-serif;background:#f3f5f6;color:#173041;margin:0}}main{{max-width:1080px;margin:auto;padding:42px 24px}}h1{{font-size:32px;line-height:1.3}}h2{{font-size:23px}}article{{background:white;padding:24px;margin:24px 0;border:1px solid #d8e0e4;border-radius:12px}}video,img{{display:block;width:100%;height:auto;border-radius:6px}}a{{color:#006d85}}.note{{font-size:14px;color:#5a6771}}summary{{cursor:pointer;color:#006d85}}nav a{{margin-right:16px}}</style>
<main><p>Crash2OpenX / 2026-09-19</p><h1>从 42 份事故材料中，形成 {len(records)} 例有明确交互的 ADS 测试</h1>
<p>原始事故 → 框架生成道路与场景种子 → 可记录参数变化的测试变体 → CARLA + InterFuser 实跑。</p>
<p>每例包含可移植的 XODR/XOSC、来源材料、真实视频、日志和测量结果。青色框为 ADS，橙色框为 NPC；图片下方可打开连续原速视频，也可用本地播放器播放。</p>
<p class="note">这里验收的是地图与测试刺激的可用性。ADS 碰撞、偏离车道或未完成全程属于观测结果；单次试验不代表统计失效率。道路标线和纹理在 CARLA 动态地图中显示有限，几何图以 XODR 和实测轨迹为依据。完整事故细节与信号合规尚未完成验收。</p>
<nav><a href="report.md">汇报说明</a><a href="case_index.csv">案例表</a><a href="showcase.mp4">全部案例短片</a><a href="REPRODUCE.md">复现方法</a><a href="selection_notes.md">42 例筛选记录</a></nav>
{''.join(cards)}</main></html>'''
    (output/'START_HERE.html').write_text(html)
    lines = ['# Crash2OpenX 场景测试进展', '',
             f'围绕上次讨论的 42 例事故场景转换任务，本轮从现有框架生成结果中选出 {len(records)} 个不同原始案例，形成可查看地图、运行测试并核对实测证据的交付集。', '',
             '本轮重点是测试交互时机：同向切入和前车减速按 ADS 的实际净空启动，路口来车结合生成路网的冲突位置设置出发窗口。每次改动均保存原始种子与实验参数，ADS 保持 InterFuser 自主控制。', '',
             '| 编号 | 场景 | 本轮观测 |', '|---|---|---|']
    lines += [f"| {r['case_id'][:3]} | {r['title']} | {r['observed_outcome']} |" for r in records]
    lines += ['', '这些结果展示的是事故材料转化为 ADS 测试刺激的能力。碰撞与车道偏离如实记录；未碰撞案例可以用于分析制动和避让表现。每例只有一次对应参数下的实跑，不能推导稳定失效率，也不能声称完整重建了原始事故。', '',
              '附件中每例都有地图、测试文件、原始事故 PDF、完整视频、日志与轨迹。建议下一轮按相同种子重复测试，检验结果是否稳定。']
    (output/'report.md').write_text('\n'.join(lines)+'\n')
    (output/'REPRODUCE.md').write_text('''# 复现\n\n主测试文件位于 cases/<case_id>/scenario.xosc，相对引用同目录 map.xodr。运行器为 CARLA 0.9.16 + 本项目 ScenarioRunner + PCLA 的 if_if（InterFuser），hero 始终为外部自主控制。\n\n请在 crash2openx 仓库中配置自己的 CARLA_REMOTE_* 后，使用 tools/carla_remote.py 的 run 子命令。每例 evidence/runtime_code 和 runtime_manifest.json 保留了实测版本，场景需支持 ADS 同时条件、真实车身净空、生成车道路由和 Act 停止条件。库存 ScenarioRunner 的条件锁存语义不能替代这些扩展。\n\n复现编译可使用 tools/expand_ads_batch.py，批量筛选用 tools/review_ads_expansion.py。原始种子在 source/，实验种子在 scene_seed.json。地图几何与对应原生 CARLA 检查见 evidence/roadgraph_selfcheck.json。\n\n直接重跑可能出现不同轨迹；本包只证明所附日志对应的单次试验。evidence/scenario.runtime.xosc 是运行时证据，可能含服务器绝对路径，请使用案例目录的 scenario.xosc 作为入口。\n''')
    with (output/'REPRODUCE.md').open('a') as handle:
        handle.write('''
示例（在已有依赖和远端运行环境的仓库中运行，路径替换为解压后的实际位置）：

```sh
CASE_DIR=/absolute/path/to/ads_showcase_10_20260919/cases/013_Zoox_February_19_2025
uv run python tools/carla_remote.py run --xodr "$CASE_DIR/map.xodr" --xosc "$CASE_DIR/scenario.xosc" --pcla-agent if_if --sut-actor hero --rgb-actor-role hero --max-seconds 600 --scene-seed "$CASE_DIR/scene_seed.json" --road-seed "$CASE_DIR/source/road_seed.json"
```

`implementation_snapshot/` 保存本轮编译、筛选及打包实现，需配合原仓库依赖；不包含 CARLA、模型权重或登录凭据。每例的 runtime_code 则保存该次运行实际使用的文件，早期和最终运行的停止条件版本可能不同。

`manifest.json` 的 source_scene 与 derived_scene 可直接比较参数变化。262 使用仍保持低速行驶的减速变体，避开当前 CARLA 车辆低速停住时的非物理速度尖峰，不能声称复现完全停车。038 的完整运行达到墙钟超时，仅验收所附交互窗口。

时序图的圆点表示片段起点，三角表示终点；底部净空/TTC 为动作后最多六秒的窗口指标。视频可延伸到之后的接触，接触由真实碰撞传感器证据确认。
''')
    concat = output/'clips.txt'
    concat.write_text(''.join("file '"+str(p.resolve()).replace("'", "'\\''")+"'\n" for p in clips))
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-f', 'concat', '-safe', '0', '-i', str(concat),
                    '-c', 'copy', '-movflags', '+faststart', str(output/'showcase.mp4')], check=True)
    concat.unlink()
    subprocess.run(['ffmpeg', '-v', 'error', '-i', str(output/'showcase.mp4'), '-f', 'null', '-'], check=True)
    write(output/'MANIFEST.json', {'files': {str(p.relative_to(output)): digest(p) for p in sorted(output.rglob('*')) if p.is_file()}})
    archive = output.with_suffix('.zip')
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as bundle:
        for p in sorted(output.rglob('*')):
            if p.is_file():
                bundle.write(p, str(Path(output.name)/p.relative_to(output)))
    with zipfile.ZipFile(archive) as bundle:
        if bundle.testzip():
            raise ValueError('ZIP CRC failure')
    archive.with_suffix('.zip.sha256').write_text(digest(archive)+'  '+archive.name+'\n')
    print(json.dumps({'directory':str(output), 'archive':str(archive), 'cases':len(records)}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('base', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    package(args.base.resolve(), args.output.resolve())
