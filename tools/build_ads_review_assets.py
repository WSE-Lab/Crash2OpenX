#!/usr/bin/env python3
"""Create timestamp-aligned excerpts and contact sheets from measured videos."""
import argparse
from fractions import Fraction
import json
from pathlib import Path
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.plot_ads_case import plot
from tools.review_ads_stress import read_rows
from tools.run_fresh_framework_batch import digest, write


def build(folder):
    quality = json.loads((folder/'quality_review.json').read_text())
    interactions = [i for i in quality['interactions'] if i['candidate_pass']]
    if not interactions:
        return
    onset = min(i['start_since_trace_s'] for i in interactions)
    run = folder/'run'
    video = run/'ads_review/ads_review.mp4'
    if not video.exists():
        raise ValueError('Synchronized ADS annotation missing: '+str(folder))
    timestamps = read_rows(run/'rgb_frames/timestamps.jsonl')
    first = quality['onset_review']['trace_start_time_s']
    probe = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries', 'stream=avg_frame_rate,nb_frames', '-of', 'json', str(video)]))['streams'][0]
    fps = float(Fraction(probe['avg_frame_rate']))
    def frame_at(t):
        return min(timestamps, key=lambda r: abs(r['simulation_time']-(first+t)))
    contacts = [p['first_time_s']-first for p in quality['actions']['contact_pairs']
                if quality['onset_review']['ads_actor'] in p['actors']]
    final_sample = max(onset+5, min(contacts)+.15) if contacts else onset+5
    targets = [frame_at(t) for t in (max(0,onset-1), onset+1, onset+3, final_sample)]
    output = folder/'visual_review'
    output.mkdir(exist_ok=True)
    condition = '+'.join(f"eq(n,{r['image_index']})" for r in targets)
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-i', str(video), '-vf', "select='"+condition+"'",
                    '-vsync', '0', str(output/'sample_%02d.jpg')], check=True)
    font = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', 18)
    sheet = Image.new('RGB', (1280, 776), '#101923')
    draw = ImageDraw.Draw(sheet)
    for i, item in enumerate(targets):
        frame = Image.open(output/f'sample_{i+1:02d}.jpg').resize((640, 360))
        x, y = (i%2)*640, (i//2)*388
        sheet.paste(frame, (x,y))
        draw.text((x+10,y+365), f"{quality['case_id'][:3]}  |  actual t={item['simulation_time']-first:.2f}s  |  frame {item['image_index']}", fill='white', font=font)
    sheet.save(folder/'contact_sheet.jpg', quality=93)
    end_time = max(onset+7, min(contacts)+1) if contacts else onset+7
    start, end = frame_at(max(0, onset-2)), frame_at(end_time)
    duration = (end['image_index']-start['image_index']+1)/fps
    clip = folder/'interaction.mp4'
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-ss', str(start['image_index']/fps), '-i', str(video),
                    '-t', str(duration), '-an', '-c:v', 'libx264', '-preset', 'fast', '-crf', '20',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(clip)], check=True)
    subprocess.run(['ffmpeg', '-v', 'error', '-i', str(clip), '-f', 'null', '-'], check=True)
    write(folder/'clip_provenance.json', {'source_video': 'run/ads_review/ads_review.mp4',
        'source_video_sha256': digest(video), 'raw_video_sha256': digest(run/'carla_rgb.mp4'),
        'clip_sha256': digest(clip), 'first_source_frame': start['image_index'],
        'last_source_frame': end['image_index'], 'source_fps': fps,
        'first_simulation_time_s': start['simulation_time'], 'last_simulation_time_s': end['simulation_time'],
        'playback': 'continuous original-speed excerpt', 'contact_sheet_samples': targets, 'full_decode_pass': True})
    plot(folder)
    print('ASSETS', quality['case_id'][:3], folder.name, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    build(parser.parse_args().folder)
