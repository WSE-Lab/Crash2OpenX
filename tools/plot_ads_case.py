#!/usr/bin/env python3
"""Plot generated map geometry and measured trajectories for a derived case."""
import argparse
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.review_ads_stress import read_rows
from tools.xodr_corridor_review import CorridorMap

COLORS = ['#087d9b', '#df672b', '#8059ad', '#587e37', '#c89527']


def map_background(ax, path):
    geometry = CorridorMap(path)
    for g in geometry.geometries:
        ss = np.linspace(0, g['length'], max(2, math.ceil(g['length'])+1))
        if g['curve']:
            x = np.array([g['curve'].X(s) for s in ss])
            y = np.array([g['curve'].Y(s) for s in ss])
            h = np.array([g['curve'].Theta(s) for s in ss])
        else:
            x, y, h = g['x']+ss*math.cos(g['h']), g['y']+ss*math.sin(g['h']), np.full_like(ss, g['h'])
        for lane in g['lanes']:
            edges = [np.column_stack([x-np.sin(h)*t, y+np.cos(h)*t]) for t in (lane['lo'], lane['hi'])]
            ax.add_patch(Polygon(np.vstack([edges[0], edges[1][::-1]]), facecolor='#e4e8eb', edgecolor='#afb8bf', linewidth=.55))
    ax.set_aspect('equal')
    ax.set_xlabel('OpenDRIVE x (m)')
    ax.set_ylabel('OpenDRIVE y (m)')
    ax.grid(alpha=.12)
    return geometry


def plot(folder):
    quality = json.loads((folder/'quality_review.json').read_text())
    rows = read_rows(folder/'run/sim_trace_raw.jsonl')
    onset = quality['onset_review']
    ads = onset['ads_actor']
    candidates = [i for i in quality['interactions'] if i['candidate_pass']] or quality['interactions']
    first = min((i['start_since_trace_s'] for i in candidates), default=5)
    start, end = max(0, first-2), first+8
    contacts = [p['first_time_s']-rows[0]['simulation_time'] for p in quality['actions']['contact_pairs']
                if ads in p['actors']]
    if contacts:
        end = max(end, min(contacts)+1)
    relative = np.array([r['simulation_time']-rows[0]['simulation_time'] for r in rows])
    window = [r for r, t in zip(rows, relative) if start <= t <= end]
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), gridspec_kw={'width_ratios': [1.08, 1]})
    fig.patch.set_facecolor('#fafafa')
    ax, speed_ax = axes
    map_background(ax, folder/'map.xodr')
    actors = [ads]+[n for n in rows[0]['actors'] if n != ads and rows[0]['actors'][n].get('actor_id') != rows[0]['actors'][ads].get('actor_id')]
    xs, ys = [], []
    for index, actor in enumerate(actors):
        states = [r['actors'][actor] for r in window if actor in r['actors']]
        x, y = [s['x'] for s in states], [-s['y'] for s in states]
        xs.extend(x); ys.extend(y)
        color = COLORS[index % len(COLORS)]
        ax.plot(x, y, color=color, linewidth=2.1, label=('ADS / ' if actor == ads else 'NPC / ')+actor)
        ax.scatter(x[0], y[0], color=color, s=30, marker='o')
        ax.scatter(x[-1], y[-1], color=color, s=35, marker='>')
        times = [r['simulation_time']-rows[0]['simulation_time'] for r in rows if actor in r['actors']]
        speeds = [math.hypot(r['actors'][actor]['vx'], r['actors'][actor]['vy']) for r in rows if actor in r['actors']]
        speed_ax.plot(times, speeds, color=color, linewidth=1.8, label=('ADS ' if actor == ads else 'NPC ')+actor)
    ax.set_xlim(min(xs)-12, max(xs)+12)
    ax.set_ylim(min(ys)-12, max(ys)+12)
    ax.set_title('Generated lanes + actual motion', fontsize=12, loc='left')
    ax.legend(fontsize=8, loc='best')
    speed_ax.set_xlim(0, min(relative[-1], end+2))
    speed_ax.set_xlabel('Time since first trace sample (s)')
    speed_ax.set_ylabel('Measured speed (m/s)')
    speed_ax.grid(alpha=.2)
    speed_ax.set_title('Autonomous response', fontsize=12, loc='left')
    speed_ax.legend(fontsize=8, loc='upper right')
    for i in candidates:
        speed_ax.axvline(i['start_since_trace_s'], color='#df672b', linestyle='--', alpha=.7)
    details = []
    for i in candidates:
        ttc = f"{i['min_positive_ttc_s']:.2f} s" if i['min_positive_ttc_s'] is not None else 'not finite'
        details.append(f"{i['actor']}: clearance {i['min_clearance_m']:.2f} m | TTC {ttc} | brake {i['ego_max_brake']:.2f}")
    fig.suptitle(quality['case_id'].replace('_', ' ')+'  /  '+folder.name, x=.06, ha='left', fontsize=15, weight='bold')
    fig.text(.06, .065, '\n'.join(details), fontsize=10)
    fig.text(.06, .025, 'Lanes: generated OpenDRIVE geometry. Traces: CARLA observations. Modified test parameters; no full accident-reconstruction claim.', fontsize=8, color='#58636d')
    fig.tight_layout(rect=[.02, .14, .98, .93])
    fig.savefig(folder/'map_and_response.png', dpi=145)
    plt.close(fig)
    # A complete map view remains available separately from the interaction zoom.
    fig, ax = plt.subplots(figsize=(8, 8))
    map_background(ax, folder/'map.xodr')
    ax.autoscale_view()
    ax.set_title(quality['case_id'].split('_')[0]+' — complete generated map')
    fig.tight_layout()
    fig.savefig(folder/'map_overview.png', dpi=140)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    plot(parser.parse_args().folder)
