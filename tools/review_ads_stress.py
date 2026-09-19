#!/usr/bin/env python3
"""Measure hazard onset and annotate actual CARLA RGB using recorded actor poses."""
import argparse
import bisect
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from runner.src.ads_geometry import pair_metrics


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def nearest(rows, times, timestamp, tolerance=.075):
    i = bisect.bisect_left(times, timestamp)
    candidates = rows[max(0, i-1):i+1]
    row = min(candidates, key=lambda r: abs(r['simulation_time']-timestamp)) if candidates else None
    return row if row and abs(row['simulation_time']-timestamp) <= tolerance else None


def scenario_info(path):
    root = ET.parse(path).getroot()
    ads = []
    for private in root.findall('.//Init/Actions/Private'):
        if private.find(".//Property[@value='external_control']") is not None:
            ads.append(private.get('entityRef'))
    if len(ads) != 1:
        raise ValueError('ADS annotation requires exactly one externally controlled SUT')
    parameter = root.find("./ParameterDeclarations/ParameterDeclaration[@name='C2XADSTriggers']")
    plans = json.loads(parameter.get('value')) if parameter is not None else {}
    if not plans:
        for group in root.findall('.//ManeuverGroup'):
            actor = group.find('./Actors/EntityRef')
            if actor is None or actor.get('entityRef') == ads[0]:
                continue
            for event in group.findall('.//Event'):
                name = event.get('name', '')
                if name.endswith(('_brake', '_cut', '_partial_intrusion', '_move', '_depart')):
                    plans[actor.get('entityRef')] = {'event': name, 'legacy_trigger': True}
    return ads[0], plans


def evaluate(rows, events, ads, plans):
    rows = sorted(rows, key=lambda r: r['simulation_time'])
    times = [r['simulation_time'] for r in rows]
    if not rows:
        raise ValueError('empty actor trace')
    first = times[0]
    observed = [e for e in events if e.get('event_type') == 'storyboard_transition']
    contacts = [e for e in events if e.get('event_type') == 'collision' and
                e.get('payload', {}).get('source') == 'carla_collision_sensor']
    result = {'ads_actor': ads, 'trace_start_time_s': first, 'trace_end_time_s': times[-1],
              'stage_observation_available': bool(observed), 'actors': {},
              'physical_ads_contact_count': sum(ads in e.get('payload', {}).get('actors', []) for e in contacts),
              'metric_notes': ['TTC assumes fixed headings and constant planar velocities; separate lanes may have no finite TTC.',
                  'Clearance/speed is a timing proxy, not perception latency or TTC.',
                  'Event START is observed from ScenarioRunner; action response is measured separately.',
                  'No physical collision is inferred from overlapping projected boxes alone.']}
    for actor, plan in plans.items():
        starts = [e for e in observed if e.get('payload', {}).get('element_type') == 'EVENT' and
                  e['payload'].get('element_name') == plan['event'] and e['payload'].get('transition') == 'START']
        timestamp = min(e['simulation_time'] for e in starts) if starts else None
        sample = nearest(rows, times, timestamp) if timestamp is not None else None
        metrics = []
        for row in rows:
            actors = row.get('actors', {})
            if ads in actors and actor in actors:
                metrics.append((row['simulation_time'], pair_metrics(actors[ads], actors[actor])))
        at_start = pair_metrics(sample['actors'][ads], sample['actors'][actor]) if sample and ads in sample['actors'] and actor in sample['actors'] else None
        braking = None
        already_braking = None
        if sample:
            already_braking = sample['actors'][ads].get('applied_control', {}).get('brake', 0) >= .1
            previous_time = timestamp
            for row in rows[bisect.bisect_left(times, timestamp):]:
                if row['simulation_time']-previous_time > .15:
                    break
                previous_time = row['simulation_time']
                if row.get('actors', {}).get(ads, {}).get('applied_control', {}).get('brake', 0) >= .1:
                    braking = row['simulation_time']-timestamp
                    break
        finite = [m['constant_velocity_ttc_s'] for _, m in metrics if m['constant_velocity_ttc_s'] is not None and m['constant_velocity_ttc_s'] > 0]
        window = [m for t, m in metrics if timestamp is not None and timestamp <= t <= timestamp+6]
        window_ttc = [m['constant_velocity_ttc_s'] for m in window
                      if m['constant_velocity_ttc_s'] is not None and m['constant_velocity_ttc_s'] > 0]
        eligible = []
        if 'distance_m' in plan:
            eligible = [t for t, m in metrics if plan['min_clearance_m'] < m['body_clearance_m'] < plan['distance_m']
                and m['ego_speed_mps'] > plan['min_ego_speed_mps']
                and (not plan['min_npc_speed_mps'] or m['npc_speed_mps'] > plan['min_npc_speed_mps'])
                and ('min_closing_speed_mps' not in plan or
                     m['ego_speed_mps']-m['npc_speed_mps'] > plan['min_closing_speed_mps'])]
        result['actors'][actor] = {'plan': plan,
            'start_status': 'observed' if timestamp is not None else ('not_observed' if observed else 'missing_instrumentation'),
            'start_time_s': timestamp, 'start_since_trace_s': timestamp-first if timestamp is not None else None,
            'at_start': at_start, 'already_braking_at_start': already_braking,
            'initial_body_clearance_m': metrics[0][1]['body_clearance_m'] if metrics else None,
            'first_brake_after_start_s': braking,
            'sampled_eligible_ticks': len(eligible) if 'distance_m' in plan else None,
            'first_sampled_eligible_time_s': eligible[0] if eligible else None,
            'six_seconds_after_start_min_clearance_m': min((m['body_clearance_m'] for m in window), default=None),
            'six_seconds_after_start_min_positive_ttc_s': min(window_ttc, default=None),
            'min_body_clearance_m': min((m['body_clearance_m'] for _, m in metrics), default=None),
            'min_positive_constant_velocity_ttc_s': min(finite, default=None),
            'physical_contact_with_ads': any(set(e.get('payload', {}).get('actors', [])) == {ads, actor} for e in contacts)}
        # Exclude startup, trace gaps and contact impulses from the motion bound.
        accelerations, lateral = [], []
        onset = None
        for previous, current in zip(rows, rows[1:]):
            t = current['simulation_time']
            dt = t-previous['simulation_time']
            if not 0 < dt <= .15 or t < first+.5 or actor not in current['actors'] or actor not in previous['actors']:
                continue
            if any(actor in e['payload'].get('actors', []) and abs(e['simulation_time']-t) < .25 for e in contacts):
                continue
            a, b = previous['actors'][actor], current['actors'][actor]
            ax, ay = (b['vx']-a['vx'])/dt, (b['vy']-a['vy'])/dt
            yaw = math.radians(b['yaw'])
            longitudinal = ax*math.cos(yaw)+ay*math.sin(yaw)
            lat = -ax*math.sin(yaw)+ay*math.cos(yaw)
            if timestamp is not None and timestamp <= t <= timestamp+6:
                accelerations.append(longitudinal)
                lateral.append(abs(lat))
                if onset is None:
                    if plan['event'].endswith('_brake') and b.get('applied_control', {}).get('brake', 0) >= .1:
                        onset = t
                    elif plan['event'].endswith(('_cut', '_partial_intrusion')) and sample:
                        heading = math.radians(sample['actors'][ads]['yaw'])
                        if abs(-b['vx']*math.sin(heading)+b['vy']*math.cos(heading)) > .3:
                            onset = t
        result['actors'][actor].update({
            'npc_motion_onset_time_s': onset,
            'event_to_motion_onset_s': onset-timestamp if onset is not None else None,
            'six_seconds_after_start_max_npc_deceleration_mps2': max(0.0, -min(accelerations)) if accelerations else None,
            'six_seconds_after_start_max_npc_lateral_acceleration_mps2': max(lateral, default=None)})
    return result


def transform(pose):
    pitch, yaw, roll = [math.radians(pose.get(k, 0)) for k in ('pitch', 'yaw', 'roll')]
    cp, sp, cy, sy, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw), math.cos(roll), math.sin(roll)
    return np.array([[cp*cy, cy*sp*sr-sy*cr, -cy*sp*cr-sy*sr, pose.get('x', 0)],
                     [cp*sy, sy*sp*sr+cy*cr, -sy*sp*cr+cy*sr, pose.get('y', 0)],
                     [sp, -cp*sr, cp*cr, pose.get('z', 0)], [0, 0, 0, 1]])


def font(size):
    for path in ['/System/Library/Fonts/Supplemental/Arial.ttf', '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default(size=size)


def annotate_frame(frame, row, camera, report, timestamp, title):
    draw = ImageDraw.Draw(frame)
    w, h = frame.size
    scale = w/1280
    large, small = font(round(27*scale)), font(round(21*scale))
    draw.rectangle((0, 0, w, 91*scale), fill='#101923')
    draw.text((22*scale, 12*scale), title, font=large, fill='white')
    draw.text((22*scale, 51*scale), f"ADS = {report['ads_actor']} | autonomous control | t = {timestamp-report['trace_start_time_s']:.2f} s", font=small, fill='#64e9ff')
    if row is None:
        draw.text((20, h-40), 'No synchronized actor sample - labels withheld', font=small, fill='white')
        return frame
    actors = row['actors']
    parent = actors.get(camera['actor_role'])
    if parent:
        inverse = np.linalg.inv(transform(parent) @ transform(camera['relative_pose']))
        focal = w/(2*math.tan(math.radians(float(camera['attributes']['fov']))/2))
        for name, actor in actors.items():
            if name != report['ads_actor'] and actor.get('actor_id') == actors.get(report['ads_actor'], {}).get('actor_id'):
                continue
            vertices = np.array([[*v, 1] for v in actor['bounding_box_world_vertices']])
            points = (inverse @ vertices.T).T
            if (points[:, 0] <= .25).any():
                continue
            xs, ys = w/2+focal*points[:, 1]/points[:, 0], h/2-focal*points[:, 2]/points[:, 0]
            left, right, top, bottom = min(xs), max(xs), min(ys), max(ys)
            if right < 0 or left > w or bottom < 0 or top > h:
                continue
            left, right, top, bottom = max(0, left), min(w-1, right), max(0, top), min(h-1, bottom)
            if left >= right or top >= bottom:
                continue
            is_ads = name == report['ads_actor']
            color = '#64e9ff' if is_ads else '#ffb657'
            draw.rectangle((left, top, right, bottom), outline=color, width=max(2, round(3*scale)))
            label = ('ADS / ' if is_ads else 'NPC / ')+name
            label_top = max(94*scale, top-28*scale)
            box = draw.textbbox((left, label_top), label, font=small)
            draw.rectangle((box[0]-3, box[1]-3, box[2]+3, box[3]+3), fill='#101923')
            draw.text((left, label_top), label, font=small, fill=color)
    panel_h = (39+32*len(report['actors']))*scale
    draw.rectangle((0, h-panel_h, w, h), fill='#101923')
    draw.text((20*scale, h-panel_h+7*scale), 'Measured body clearance | observed hazard event | constant-velocity TTC', font=small, fill='#d0d6dc')
    for i, (name, info) in enumerate(report['actors'].items()):
        if name not in actors or report['ads_actor'] not in actors:
            continue
        metrics = pair_metrics(actors[report['ads_actor']], actors[name])
        start = info['start_time_s']
        state = 'ACTION STARTED' if start is not None and timestamp >= start else 'WAITING'
        if info['start_status'] == 'missing_instrumentation':
            state = 'START UNKNOWN'
        ttc = metrics['constant_velocity_ttc_s']
        ttc_text = f'{ttc:.2f} s' if ttc is not None else 'no projected collision'
        line = f"{name}   gap {metrics['body_clearance_m']:.2f} m   closing {metrics['radial_closing_speed_mps']:.2f} m/s   {state}   TTC {ttc_text}"
        draw.text((20*scale, h-panel_h+(38+32*i)*scale), line, font=small, fill='#ffb657')
    return frame


def render(run, output, rows, report, title):
    video = run/'carla_rgb.mp4'
    camera = json.loads((run/'rgb_frames/camera.json').read_text())
    timestamps = read_rows(run/'rgb_frames/timestamps.jsonl')
    info = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
        '-show_entries', 'stream=width,height,avg_frame_rate,nb_frames', '-of', 'json', str(video)]))['streams'][0]
    w, h = info['width'], info['height']
    count = int(info['nb_frames'])
    indexed = {r['image_index']: r for r in timestamps if 0 <= r['image_index'] < count}
    if len(indexed) != count:
        raise ValueError('Each source video frame must have its real timestamp')
    times = [r['simulation_time'] for r in rows]
    decoder = subprocess.Popen(['ffmpeg', '-v', 'error', '-i', str(video), '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-'], stdout=subprocess.PIPE)
    encoder = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
        '-s', f'{w}x{h}', '-r', info['avg_frame_rate'], '-i', '-', '-an', '-c:v', 'libx264',
        '-preset', 'fast', '-crf', '20', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output/'ads_review.mp4')], stdin=subprocess.PIPE)
    saved = set()
    targets = [r['start_time_s'] for r in report['actors'].values() if r['start_time_s'] is not None]
    matched = 0
    try:
        for index in range(count):
            raw = decoder.stdout.read(w*h*3)
            if len(raw) != w*h*3:
                raise ValueError('Truncated RGB decode')
            timestamp = indexed[index]['simulation_time']
            row = nearest(rows, times, timestamp)
            matched += row is not None
            frame = annotate_frame(Image.frombytes('RGB', (w, h), raw), row, camera, report, timestamp, title)
            encoder.stdin.write(frame.tobytes())
            if index == 0 or any(abs(timestamp-t) < .08 and t not in saved for t in targets):
                frame.save(output/f'frame_{index:06d}.jpg', quality=92)
                saved.update(t for t in targets if abs(timestamp-t) < .08)
        encoder.stdin.close()
        if decoder.wait() or encoder.wait():
            raise RuntimeError('Video decode/encode failed')
    finally:
        for process in (decoder, encoder):
            if process.poll() is None:
                process.kill()
                process.wait()
    return {'frames': count, 'frames_with_pose_within_75ms': matched,
            'source_video_sha256': hashlib.sha256(video.read_bytes()).hexdigest(),
            'annotation': 'offline projection of measured CARLA body vertices; raw video unchanged'}


def review_run(run, xosc, output, *, video=False, title='ADS-relative pressure test'):
    output.mkdir(parents=True, exist_ok=False)
    rows = sorted(read_rows(run/'sim_trace_raw.jsonl'), key=lambda r:r['simulation_time'])
    ads, plans = scenario_info(xosc)
    actual_scenario = run/'scenario.runtime.xosc'
    if actual_scenario.is_file():
        actual_ads, _ = scenario_info(actual_scenario)
        if actual_ads != ads:
            raise ValueError('The runtime ADS identity differs from the supplied scenario')
    report = evaluate(rows, read_rows(run/'events.jsonl'), ads, plans)
    report['run'] = str(run.resolve())
    summary = json.loads((run/'summary.json').read_text())
    report['termination_reason'] = summary.get('termination_reason')
    if video:
        report['video'] = render(run, output, rows, report, title)
    (output/'review.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--xosc', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--title', default='ADS-relative pressure test')
    args = parser.parse_args()
    report = review_run(args.run, args.xosc, args.out, video=args.video, title=args.title)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
