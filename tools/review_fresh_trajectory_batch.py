"""Summarize measured trajectory limitations without promoting runs to fidelity success."""
import argparse
import csv
import hashlib
import json
from collections import Counter
from datetime import datetime, timezone
from html import escape
from pathlib import Path

from tools.audit_fresh_framework_batch import execution_integrity
from tools.xodr_corridor_review import review_trace
from tools.review_runtime_action_progress import review_run as review_actions


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def native_geometry_review(result):
    directory = result.get('roadgraph_cache_dir')
    if not directory:
        return {'verified':False, 'reason':'No native CARLA roadgraph cache recorded'}
    graph = Path(directory)
    check_file, hash_file = graph/'roadgraph_selfcheck.json', graph/'.input_sha256'
    check = read(check_file) if check_file.is_file() else {}
    matches = hash_file.is_file() and hash_file.read_text().strip() == digest(Path(result['xodr_path']))
    verified = bool(matches and check.get('geometry_check_version') == 1
                    and check.get('geometry_consistency_pass') is True)
    return {'verified':verified, 'source_map_hash_matches':matches,
            'max_position_difference_m':check.get('geometry_max_distance_m'),
            'max_yaw_difference_degrees':check.get('geometry_max_yaw_degrees'),
            'check_path':str(check_file),
            'check_sha256':digest(check_file) if check_file.is_file() else None,
            'reason':'Native CARLA sampling agrees with generated XODR within recorded tolerance'
                     if verified else 'Design geometry only; native CARLA agreement is not verified for this map'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('base', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    base, out = args.base.resolve(), args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    audit = read(base/'audit.json')
    obligations = {r['case_id']: r for r in read(base/'source_review/reviewed_42_narratives.json')['cases']}
    labels = {
        'execution_failure': '运行不完整或参与者跌出地图',
        'outside_or_wrong_lane': '中心偏离设计车道或逆向行驶',
        'contact_requires_review': '发生接触，事故关系仍不满足或待核实',
        'lane_boundary_contacts': '存在车道边界接触，机动质量待验收',
        'limited_progress': '停车或停滞，预期机动未完成',
        'high_progress_observation': '高进度无接触观测，事故未重建',
        'insufficient_evidence': '缺少可核对运行证据',
    }
    cases = []
    for case in audit['cases']:
        cid = case['case_id']
        def started(attempt):
            return read(base/'cases'/cid/attempt['attempt']/'framework_invocation.json').get('started_at', '')
        attempts = sorted(case['attempts'], key=started)
        eligible = [a for a in attempts if a.get('runtime_evidence_pass')]
        if not eligible:
            raise ValueError('No complete evidence to review: '+cid)
        chosen = eligible[-1]
        run = Path(chosen['run'])
        summary = read(run/'summary.json')
        result = read(run/'result.json')
        native_geometry = native_geometry_review(result)
        recompiled = result.get('road_recompiled', False)
        trace = [json.loads(l) for l in (run/'sim_trace_raw.jsonl').read_text().splitlines()]
        corridor = review_trace(run/'map.xodr', trace)
        corridor.update(trace_sha256=digest(run/'sim_trace_raw.jsonl'), case_id=cid, attempt=chosen['attempt'])
        (out/(cid+'_corridor.json')).write_text(json.dumps(corridor, ensure_ascii=False, indent=2)+'\n')
        integrity = execution_integrity(summary, chosen.get('actor_minimum_z', {}))
        action_review = review_actions(run)
        (out/(cid+'_actions.json')).write_text(json.dumps(action_review, ensure_ascii=False, indent=2)+'\n')
        not_started = [e['event'] for e in action_review['planned_events']
                       if e['observation_status'] == 'start_not_observed']
        tilted = {actor: round(state['tilted_over_60_degrees_seconds'], 3)
                  for actor, state in action_review['actor_motion'].items()
                  if state.get('tilted_over_60_degrees_seconds', 0)}
        wrong = next((c.get('actual_value', 0) for c in summary.get('criteria', []) if c['name']=='WrongLaneTest'), 0)
        reasons = []
        if not action_review['has_stage_observations']:
            reasons.append('该版未保存可核对的动作阶段观察，不能据缺少阶段记录断言动作未执行。')
        elif not_started:
            reasons.append('运行阶段记录未观测到这些事件开始：'+', '.join(not_started)+'；需核对触发条件与实际初始关系。')
        if tilted:
            reasons.append('实测参与者横滚或俯仰绝对值超过60度的时长（秒）：'+json.dumps(tilted,ensure_ascii=False)+'；须与原文机动核对，不能以在地图上或路线完成判为正常轨迹。')
        if recompiled:
            reasons.append('本版从本批已验证RoadSeed重新编译道路；结构等价门禁通过，沿用原推理与模型QA，重新提取原生路网并实跑。没有重新调用VLM。')
        ticks = summary.get('total_ticks', 0)
        inv = summary.get('lane_invasion_count', 0)
        contacts = summary.get('collision_count', 0)
        progress = summary.get('final_route_completion', 0)
        if not integrity['execution_integrity_pass']:
            category = 'execution_failure'
            reasons.append(f"运行{ticks}帧后以{summary.get('termination_reason')}结束，未形成完整有效轨迹。")
        elif corridor['unresolved_samples'] or corridor['missing_actor_samples']:
            category = 'insufficient_evidence'
            reasons.append('部分车道投影或自车状态无法核实。')
        elif corridor['outside_samples'] or wrong:
            category = 'outside_or_wrong_lane'
        elif contacts:
            category = 'contact_requires_review'
        elif inv:
            category = 'lane_boundary_contacts'
        elif progress >= 90:
            category = 'high_progress_observation'
        else:
            category = 'limited_progress'
        if corridor['outside_samples']:
            reasons.append(f"解析XODR复核{corridor['outside_samples']}/{corridor['samples']}帧自车中心在设计驾驶车道范围外。")
        if not native_geometry['verified']:
            reasons.append('本版地图尚无匹配哈希的原生CARLA几何一致性通过记录；设计车道复核不能单独代表实际路面。')
        if summary.get('sensor_cleanup_errors'):
            reasons.append('运行结束后的传感器清理有告警，原始日志与告警另行保留；不据此改写已有轨迹或接触记录。')
        if wrong:
            reasons.append(f'WrongLaneTest记录{wrong}次，不能据路线进度判为轨迹合格。')
        if inv:
            reasons.append(f'车道边界接触{inv}次；需结合源报告判断是否属于所需机动。')
        if contacts:
            expected, actual = chosen.get('expected_collision'), chosen.get('actual_collision')
            if expected and actual and sorted(expected) != sorted(actual):
                reasons.append(f'接触参与者{actual}，预期为{expected}；事故对象不一致。')
            else:
                reasons.append('碰撞传感器已有接触，完整时序、接触部位及源报告约束尚未共同验收。')
        else:
            reasons.append('未记录碰撞接触，未复现源报告的事故。')
        if progress < 90:
            reasons.append(f'路线进度{progress:.1f}%，结束原因为{summary.get("termination_reason")}；只能说明已观察到部分行驶/停车。')
        else:
            reasons.append(f'路线进度{progress:.1f}%仅供观测，不代表任务或事故重建成功。')
        raw = summary.get('off_road_time', 0)
        conflict = bool(raw and corridor['samples'] and corridor['inside_samples']==corridor['samples']
                        and not corridor['unsupported'] and not corridor['missing_actor_samples'])
        if conflict:
            reasons.append(f'原始越界{raw}秒与完整中心投影复核不一致；原始值保留，未检查整车轮廓。')
        later_runs = [a for a in attempts if started(a)>started(chosen) and a.get('collision_count') is not None]
        latest_run = later_runs[-1] if later_runs else chosen
        if later_runs:
            failed = [k for k,v in latest_run.get('runtime_checks', {}).items() if not v]
            reasons.append('另有更新运行未通过证据检查：'+latest_run['attempt']+'；'+', '.join(failed)+'。')
        if chosen.get('terminal_braking_interventions'):
            reasons.append('运行触发道路末端制动保护，需计入轨迹解释。')
        generation_attempts = [a for a in attempts if not read(base/'cases'/cid/a['attempt']/'framework_invocation.json').get('resumed_from')]
        latest_generation = generation_attempts[-1]
        if not latest_generation.get('generation_pass'):
            reasons.append('最新重新推理尚未通过生成检查，当前使用较早的本批新生成产物。')
        path_names = {'xodr': result['xodr_path'], 'compiled_xosc': result['xosc_path'],
                      'mp4':str(run/'carla_rgb.mp4'), 'demo_log':str(run/'demo.log'),
                      'trace':str(run/'sim_trace_raw.jsonl'), 'source_pdf':str(run.parents[1]/'source.pdf')}
        record = dict(case_id=cid, selected_attempt=chosen['attempt'], latest_runtime_attempt=latest_run['attempt'],
                      source_obligations=obligations[cid]['source_obligations_zh'], category=category,
                      category_zh=labels[category], reasons=reasons,
                      generated=True, runtime_evidence_present=True, **integrity,
                      stage_observations_available=action_review['has_stage_observations'],
                      events_without_start_observation=not_started, actors_tilted_over_60_degrees_seconds=tilted,
                      road_recompiled_from_verified_seed=recompiled,
                      generation_qa_mode='本批种子重编译；模型QA沿用，等价/原生几何重新检查' if recompiled else '本批原PDF推理与QA；可含经哈希验证的运行续跑',
                      source_acceptance_status='尚未建立完整验收',
                      source_fidelity_accepted=False,
                      total_ticks=ticks, recorded_simulation_seconds=chosen.get('recorded_simulation_seconds'),
                      termination=summary.get('termination_reason'), contact_sensor_records=contacts,
                      lane_boundary_contacts=inv, wrong_lane_count=wrong, route_progress_percent=progress,
                      native_geometry_verified=native_geometry['verified'], native_geometry=native_geometry,
                      sensor_cleanup_status=summary.get('sensor_cleanup_status'),
                      sensor_cleanup_errors=summary.get('sensor_cleanup_errors', []),
                      raw_offroad_seconds=raw, center_outside_samples=corridor['outside_samples'],
                      center_review_samples=corridor['samples'], raw_projection_conflict=conflict,
                      files=path_names, sha256={k:digest(Path(p)) for k,p in path_names.items()},
                      original_intent_issues=chosen.get('intent_issues', []))
        cases.append(record)
    counts = Counter(r['category'] for r in cases)
    report = {'created_at':datetime.now(timezone.utc).isoformat(), 'audit_at':audit['updated_at'],
              'case_count':len(cases), 'artifact_cases':len(cases),
              'selected_execution_integrity_cases':sum(r['execution_integrity_pass'] for r in cases),
              'selected_native_geometry_verified_cases':sum(r['native_geometry_verified'] for r in cases),
              'selected_equivalent_seed_recompile_cases':sum(r['road_recompiled_from_verified_seed'] for r in cases),
              'selected_cases_with_stage_observations':sum(r['stage_observations_available'] for r in cases),
              'selected_cases_with_orientation_excursions':sum(bool(r['actors_tilted_over_60_degrees_seconds']) for r in cases),
              'full_source_accepted_cases':0, 'autonomous_task_success_count':None,
              'category_counts':dict(counts), 'category_labels':labels,
              'classification_policy':'Mutually exclusive priority: execution failure, missing projection evidence, outside/wrong lane, contact, lane boundary contacts, progress>=90%, limited progress. Descriptive screening, not source-fidelity acceptance.',
              'limitations':['Single selected run per case, not repeated trials.',
                            'Corridor review evaluates actor centre only, not vehicle footprint or lane legality.',
                            'Sensor records are not independent accident counts.',
                            'Runtime-evidence selection preserves latest failed-run disclosure; raw logs unchanged.',
                            'No documented full source acceptance; zero accepted does not prove all behaviors unusable.'],
              'code_sha256':{Path(__file__).name:digest(Path(__file__))}, 'cases':cases}
    (out/'trajectory_review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    fields = ['case_id','selected_attempt','latest_runtime_attempt','generation_qa_mode','source_obligations','category_zh','execution_integrity_pass','native_geometry_verified','sensor_cleanup_status','source_acceptance_status','total_ticks','recorded_simulation_seconds','termination','contact_sensor_records','lane_boundary_contacts','wrong_lane_count','route_progress_percent','raw_offroad_seconds','center_outside_samples','center_review_samples','raw_projection_conflict','reasons','mp4','xodr','compiled_xosc','demo_log']
    with (out/'trajectory_status_42.csv').open('w',encoding='utf-8-sig',newline='') as f:
        fields[3:3] = ['stage_observations_available', 'events_without_start_observation', 'actors_tilted_over_60_degrees_seconds']
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for r in cases:
            row={k:r.get(k,'') for k in fields};row.update({k:v for k,v in r['files'].items() if k in fields})
            row['reasons']=' '.join(r['reasons']);w.writerow(row)
    (out/'README.md').write_text('# 42 场景轨迹表现复核\n\n分类为描述性筛查，不是事故验收结论。每例保留真实运行版本、源报告要求、指标和原因。\n\n'+'\n'.join(f'- {labels[k]}：{v} 例' for k,v in counts.items())+'\n\n严格完整事故语义与轨迹共同验收：0/42。自主驾驶任务成功率：尚未建立。\n')
    print(json.dumps({k:v for k,v in report.items() if k!='cases'},ensure_ascii=False))


if __name__ == '__main__':
    main()
