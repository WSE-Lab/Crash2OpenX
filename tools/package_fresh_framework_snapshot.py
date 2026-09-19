#!/usr/bin/env python3
"""Freeze auditable partial results without implying source-accident acceptance."""
import argparse
import csv
import hashlib
import json
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree as ET


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def portable_scene(source, destination):
    """Relocate only LogicFile; keep the compiled original as separate evidence."""
    tree = ET.parse(source)
    nodes = tree.findall('./RoadNetwork/LogicFile')
    if len(nodes) != 1:
        raise ValueError('Expected exactly one RoadNetwork/LogicFile: ' + str(source))
    old = nodes[0].get('filepath')
    nodes[0].set('filepath', 'map.xodr')
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding='utf-8', xml_declaration=True)
    return {'operation': 'relocate_logicfile_only', 'original_filepath': old,
            'portable_filepath': 'map.xodr', 'compiled_sha256': digest(source),
            'portable_sha256': digest(destination), 'requires_carla_pcla_runtime': True}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('base', type=Path)
    ap.add_argument('output', type=Path)
    ap.add_argument('--review', type=Path,
                    help='Frozen trajectory review matching this audit and selected attempts')
    ap.add_argument('--supplement', type=Path, action='append', default=[],
                    help='Explicit review/report directories to preserve in the snapshot')
    args = ap.parse_args()
    base = args.base.resolve()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    audit = read(base/'audit.json')
    review = read(args.review) if args.review else None
    if review and review['audit_at'] != audit['updated_at']:
        raise ValueError('Review and audit cutoffs differ')
    reviews = {r['case_id']: r for r in review['cases']} if review else {}
    if review and set(reviews) != {c['case_id'] for c in audit['cases']}:
        raise ValueError('Review and audit case sets differ')
    files, rows = [], []
    for case in audit['cases']:
        attempts = case['attempts']
        passing = [a for a in attempts if a.get('runtime_evidence_pass')]
        def started(a):
            return read(base/'cases'/case['case_id']/a['attempt']/'framework_invocation.json').get('started_at', '')
        chosen = max(passing, key=started) if passing else None
        row = {'case_id': case['case_id'], 'generation_pass': any(a.get('generation_pass') for a in attempts),
               'runtime_evidence_pass': bool(chosen), 'selected_attempt': chosen['attempt'] if chosen else '',
               'source_fidelity_accepted': False, 'termination': '', 'collision_sensor_records': '',
               'offroad_seconds': '', 'route_completion': '', 'issues': '尚无完整运行证据',
               'package_map': case['case_id']+'/map.xodr' if chosen else '',
               'package_scenario': case['case_id']+'/scenario.xosc' if chosen else '',
               'package_video': case['case_id']+'/carla_rgb.mp4' if chosen else '',
               'package_demo_log': case['case_id']+'/demo.log' if chosen else ''}
        if chosen:
            run = Path(chosen['run'])
            result = read(run/'result.json')
            portable = out/'portable_scenes'/case['case_id']/'scenario.xosc'
            relocation = portable_scene(Path(result['xosc_path']), portable)
            relocation_file = portable.with_name('relocation.json')
            relocation_file.write_text(json.dumps(relocation, indent=2)+'\n')
            row.update(termination=chosen['termination'], collision_sensor_records=chosen['collision_count'],
                       offroad_seconds=chosen['offroad_seconds'], route_completion=chosen['route_completion'],
                       issues='; '.join(chosen['intent_issues']) or '完整原文语义仍待验收')
            sources = [(Path(result['xodr_path']), 'map.xodr'), (portable, 'scenario.xosc'),
                       (Path(result['xosc_path']), 'scenario.compiled.original.xosc'),
                       (relocation_file, 'relocation.json'),
                       (run/'map.xodr', 'runtime_map.xodr'),
                       (run.parents[1]/'source.pdf', 'source.pdf'),
                       (run.parents[1]/'framework_invocation.json', 'framework_invocation.json'),
                       (run.parents[1]/'resume_provenance.json', 'resume_provenance.json')]
            for name in ('carla_rgb.mp4', 'demo.log', 'runner.log', 'carla_server.log.gz', 'events.jsonl',
                         'sim_trace_raw.jsonl', 'scenario.runtime.xosc', 'road_seed.json', 'scene_seed.json',
                         'source_extraction.json', 'source_text.txt', 'road_generation.json', 'road_recompile_equivalence.json', 'result.json',
                         'scene_inference_attempts.json',
                         'summary.json', 'sim_feedback.json', 'remote_run.json', 'runtime_manifest.json',
                         'hero_pcla_route.xml', 'behavior_check.json', 'contact_sheet.jpg', 'path_plot.png',
                         'rgb_recorder.log', 'run_scene_wrapper.log', 'runner_entry.sh', 'metadata.json',
                         'server_log_capture.json', 'gpu_memory_before_run.csv',
                         'rgb_frames/timestamps.jsonl', 'rgb_frames/camera.json'):
                sources.append((run/name, name))
            for folder in ('framework_code', 'resume_code'):
                for path in (run.parents[1]/folder).rglob('*'):
                    if path.is_file() and path.suffix in ('.py', '.md', '.xsd'):
                        sources.append((path, str(path.relative_to(run.parents[1]))))
            for path in (run/'runtime_code').rglob('*'):
                if path.is_file() and path.suffix in ('.py', '.json', '.sh'):
                    sources.append((path, str(path.relative_to(run))))
            for folder in ('qa', 'modeling'):
                for path in (run/folder).rglob('*'):
                    if path.is_file() and path.suffix in ('.json', '.txt', '.png', '.jpg'):
                        sources.append((path, str(path.relative_to(run))))
            if result.get('roadgraph_cache_dir'):
                graph = Path(result['roadgraph_cache_dir'])
                for path in graph.rglob('*'):
                    if path.is_file() and (path.suffix in ('.json', '.csv', '.gz', '.py', '.sh') or path.name == '.input_sha256'):
                        sources.append((path, 'carla_roadgraph/'+str(path.relative_to(graph))))
            # Carry the chain back to the initial PDF invocation, including raw
            # progress and archived generation rounds, rather than only a path.
            ancestor = run.parents[1]
            seen = set()
            while ancestor not in seen:
                seen.add(ancestor)
                if not ancestor.is_relative_to(base/'cases'/case['case_id']):
                    raise ValueError('Generation ancestor escaped this fresh case: '+str(ancestor))
                for name in ('framework_invocation.json', 'resume_provenance.json', 'progress.jsonl'):
                    path = ancestor/name
                    if path.is_file():
                        sources.append((path, 'generation_ancestry/'+ancestor.name+'/'+name))
                for path in (ancestor/'rounds').rglob('*'):
                    if path.is_file() and path.suffix in ('.json', '.xodr', '.xosc'):
                        sources.append((path, 'generation_ancestry/'+ancestor.name+'/'+str(path.relative_to(ancestor))))
                for path in (ancestor/'pipeline').glob('*/*'):
                    if path.is_file() and (path.name in (
                            'result.json', 'road_generation.json', 'road_recompile_equivalence.json',
                            'road_seed.json', 'scene_seed.json', 'source_extraction.json', 'source_text.txt')
                            or path.suffix == '.xodr'):
                        sources.append((path, 'generation_ancestry/'+ancestor.name+'/'+str(path.relative_to(ancestor))))
                parent = read(ancestor/'framework_invocation.json').get('resumed_from')
                if not parent:
                    break
                ancestor = Path(parent).resolve()
            for source, relative in sources:
                if source.is_file():
                    files.append({'source_path': str(source), 'archive_path': case['case_id']+'/'+relative,
                                  'bytes': source.stat().st_size, 'sha256': digest(source)})
        if review:
            measured = reviews[case['case_id']]
            if measured['selected_attempt'] != row['selected_attempt']:
                raise ValueError('Review and archive select different attempts: '+case['case_id'])
            row.update(trajectory_category=measured['category_zh'],
                       review_reasons='; '.join(measured['reasons']),
                       native_geometry_verified=measured['native_geometry_verified'],
                       stage_observations_available=measured['stage_observations_available'],
                       events_without_start_observation='; '.join(measured['events_without_start_observation']),
                       actors_tilted_over_60_degrees_seconds=json.dumps(
                           measured['actors_tilted_over_60_degrees_seconds'], ensure_ascii=False),
                       generation_qa_mode=measured['generation_qa_mode'])
        rows.append(row)
    # Supplemental evaluations have their own provenance; they never replace
    # the runtime's original summary or claim source acceptance.
    review_files = [
        base/'multicontact_patch/113_phase_aware_review.json',
        base/'multicontact_patch/113_analytic_corridor_review.json',
        base/'parking_lane_patch/274_parking_motion_review.json',
        base/'parking_lane_patch/274_parking_contact_sheet.jpg',
    ]
    review_code = [Path(__file__).parent/name for name in
                   ('repeated_contact_review.py', 'xodr_corridor_review.py')]
    for source in review_files + review_code:
        if source.is_file():
            files.append({'source_path': str(source.resolve()), 'archive_path': 'supplemental_reviews/'+source.name,
                          'bytes': source.stat().st_size, 'sha256': digest(source)})
    for folder in args.supplement:
        if not folder.is_dir():
            raise ValueError('Missing supplement directory: '+str(folder))
        for source in sorted(folder.rglob('*')):
            if source.is_file() and source.suffix.lower() in ('.json', '.jsonl', '.csv', '.md', '.txt', '.html', '.pdf', '.jpg', '.png', '.mp4', '.log', '.stderr', '.stdout', '.py', '.sh', '.xosc', '.xodr', '.xml', '.gz'):
                files.append({'source_path': str(source.resolve()),
                              'archive_path': 'supplemental_reviews/'+folder.name+'/'+str(source.relative_to(folder)),
                              'bytes': source.stat().st_size, 'sha256': digest(source)})
    manifest = {'created_at': datetime.now(timezone.utc).isoformat(), 'audit_at': audit['updated_at'],
                'trajectory_review_sha256': digest(args.review) if args.review else None,
                'source_goal_cases': 42, 'included_runtime_cases': sum(r['runtime_evidence_pass'] for r in rows),
                'source_fidelity_accepted_cases': 0, 'final_delivery': False,
                'selection': 'Latest runtime-evidence-passing attempt per case; failed attempts remain in audit.json.',
                'cases': rows, 'files': files}
    (out/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    (out/'audit.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2)+'\n')
    with (out/'scene_status_42.csv').open('w', encoding='utf-8-sig', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    count = manifest['included_runtime_cases']
    (out/'README.txt').write_text(
        f'Crash2OpenX 轨迹表现测试阶段交付：{count}/42 套完整运行证据。\n'
        f'审计时间：{audit["updated_at"]}\n'
        '这是阶段快照，尚未完成42例原文语义与轨迹共同验收。\n'
        '每个编号内包含本批新生成 map.xodr、scenario.xosc、CARLA 实录 carla_rgb.mp4、运行日志及来源证据。\n'
        '地图来自原PDF经现有框架的新推理和编译；同构道路可能几何相同，编译记录与哈希保留。\n'
        '碰撞传感器记录条数不等于独立事故次数；路线100%不等于无越界或事故复现成功。\n'
        'SUT由原PCLA控制，所选SUT可能不是报告中的AV。参见42例CSV中的运行问题。\n'
        'scenario.xosc仅将LogicFile改为同目录map.xodr；原编译文件、真实运行文件及重定位哈希分别保留。运行仍需CARLA/PCLA。\n'
        'manifest的source_path保留原始本地来源；包内使用archive_path，可独立解压查看。\n')
    archive = out/f'trajectory_evidence_{count}_of_42.zip'
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=5) as z:
        for entry in files:
            source = Path(entry['source_path'])
            if digest(source) != entry['sha256']:
                raise RuntimeError('Artifact changed during snapshot: '+str(source))
            z.write(source, entry['archive_path'])
        for name in ('README.txt', 'manifest.json', 'audit.json', 'scene_status_42.csv'):
            z.write(out/name, name)
    with zipfile.ZipFile(archive) as z:
        bad = z.testzip()
        if bad:
            raise RuntimeError('Archive CRC failure: '+bad)
        file_count = len(z.infolist())
        for row in rows:
            if not row['runtime_evidence_pass']:
                continue
            prefix = row['case_id']+'/'
            scene = ET.fromstring(z.read(prefix+'scenario.xosc'))
            reference = scene.find('./RoadNetwork/LogicFile').get('filepath')
            if prefix+reference not in z.namelist():
                raise RuntimeError('Packaged scenario references missing map: '+prefix+reference)
    result = {'path': str(archive), 'sha256': digest(archive), 'bytes': archive.stat().st_size,
              'file_count': file_count, 'crc_verified': True, 'included_cases': count, 'final_delivery': False}
    (out/'archive_verification.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
