#!/usr/bin/env python3
"""R1 v2 72-run development analysis, reusing R1-4 parsing helpers."""
from __future__ import annotations

import csv
import importlib.util
import json
import math
import os
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
from statistics import median

os.environ.setdefault('MPLCONFIGDIR', '/tmp/r1-v2-threshold72-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
font_manager.fontManager.addfont('/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc')
plt.rcParams['font.family'] = ['WenQuanYi Zen Hei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

ROOT = Path(__file__).resolve().parents[5]
OUT = Path(__file__).resolve().parent
BATCH = ROOT / 'logs/09_R1_switch_constraints/r1_v2_threshold_72/20261031_r1_v2_threshold72'
GOAL = (7.5, 0.0)
MAPS = ['r1_d09_gap_wide_v2.xml', 'r1_d09_gap_middle_v2.xml', 'r1_d09_gap_narrow_v2.xml',
        'r1_m02_space_wide_v2.xml', 'r1_m02_space_middle_v2.xml', 'r1_m02_space_narrow_v2.xml']
SCENE_TYPES = {'d09': MAPS[:3], 'm02': MAPS[3:]}
SETTINGS = ['A_REF', 'B_E03_X06', 'B_E04_X07', 'B_E06_X09']
EXPECTED = {
    'A_REF': (-0.05, -0.05, False),
    'B_E03_X06': (-0.03, -0.06, True),
    'B_E04_X07': (-0.04, -0.07, True),
    'B_E06_X09': (-0.06, -0.09, True),
}
COLORS = ['#4C72B0', '#DD8452', '#55A868', '#8172B2']
Z = 1.959963984540054

_helper_path = ROOT / 'docs/thesis_project/evidence/R1/M2_ABC_repeat_90/analyze_batch.py'
_spec = importlib.util.spec_from_file_location('r1_4_analysis_helpers', _helper_path)
_helpers = importlib.util.module_from_spec(_spec)
assert _spec and _spec.loader
_spec.loader.exec_module(_helpers)
# The reused R1-4 module sets its own preferred family during import; select
# the installed fallback after loading its shared parsing helpers.
plt.rcParams['font.family'] = ['WenQuanYi Zen Hei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def find_one(root: Path, filename: str) -> Path:
    return _helpers.find_one(root, filename)


def task_frames(path: Path) -> list[dict]:
    return _helpers.task_frames(path)


def wilson(k: int, n: int) -> tuple[float, float]:
    return _helpers.wilson(k, n)


def median_known(values: list[object]) -> float | None:
    known = [float(x) for x in values if isinstance(x, (int, float)) and math.isfinite(float(x))]
    return median(known) if known else None


def collect() -> tuple[dict, list[dict]]:
    plan = load_json(BATCH / 'run_plan.json')
    if len(plan['schedule']['items']) != 72:
        raise RuntimeError('frozen plan must contain all 72 items')
    rows = []
    for item in plan['schedule']['items']:
        root = BATCH / 'runs' / item['plan_item_id'] / 'result'
        metrics = load_json(find_one(root, 'metrics.json'))
        context = load_json(find_one(root, 'run_context.json'))
        terminal = load_json(find_one(root, 'terminal.json'))
        facts = load_json(find_one(root, 'process_facts.json'))
        frames = task_frames(find_one(root, 'runtime_record.jsonl'))
        setting = item['setting_id']
        E, X, hysteresis = EXPECTED[setting]
        evidence = ((context.get('runtime_source_validation') or {}).get('evidence') or {})
        actual_hysteresis = evidence.get('hysteresis')
        actual_hold = evidence.get('hold')
        if not isinstance(actual_hysteresis, bool):
            actual_hysteresis = str(actual_hysteresis).lower() == 'true'
        if not isinstance(actual_hold, bool):
            actual_hold = str(actual_hold).lower() == 'true'
        checks = {
            'setting_id': setting in EXPECTED,
            'runtime_group': evidence.get('group') == ('A' if setting == 'A_REF' else 'B'),
            'E': evidence.get('entry_threshold') is not None and abs(float(evidence['entry_threshold']) - E) < 1e-6,
            'effective_X': evidence.get('exit_threshold') is not None and abs(float(evidence['exit_threshold']) - X) < 1e-6,
            'H': evidence.get('recovery_min_hold_s') is not None and abs(float(evidence['recovery_min_hold_s'])) < 1e-6,
            'hysteresis': actual_hysteresis is hysteresis,
            'hold': actual_hold is False,
            'source': context.get('runtime_source_validation', {}).get('status') == 'confirmed',
            'scene': metrics.get('scene') == item['map'] and metrics.get('scene_id') == item['scene_id'],
            'record': metrics.get('record_validity') == 'VALID',
            'cleanup': facts.get('shutdown_complete') is True and facts.get('closeout_complete') is True,
        }
        if not all(checks.values()):
            raise RuntimeError(f"contract/record check failed for {item['plan_item_id']}: {checks}")
        terminal_result = metrics.get('navigation_terminal_result') or terminal.get('run_terminal_result') or 'UNKNOWN'
        safety = metrics.get('safety_task_completion_status')
        safety_label = 'TRUE' if safety is True else ('FALSE' if safety is False else 'UNKNOWN')
        # Use the first and last persisted RL-active poses: this batch's
        # metrics JSON does not expose target_distance_start_m consistently.
        start = frames[0]['distance_m']
        end = frames[-1]['distance_m']
        intervals = metrics.get('recovery_intervals') or []
        durations = [float(x['duration_s']) for x in intervals
                     if isinstance(x, dict) and isinstance(x.get('duration_s'), (int, float))]
        arrived = terminal_result == 'ARRIVED'
        stand_done = any(x.get('event') == 'STANDING_HOLD_OBSERVATION_COMPLETE'
                         for x in (facts.get('cleanup_events') or []))
        pose = frames[0]
        crossed_m02_gap = (any(5.30 <= f['x'] <= 5.60 and -1.67 <= f['y'] <= 0.42 for f in frames)
                           if item['map'].startswith('r1_m02_') else None)
        rows.append({
            'sequence': item['sequence'], 'round': item['round'], 'map': item['map'],
            'scene_id': item['scene_id'], 'setting_id': setting, 'runtime_group': item['group'],
            'plan_item_id': item['plan_item_id'], 'run_id': metrics.get('run_id'),
            'terminal_result': terminal_result, 'record_validity': metrics.get('record_validity'),
            'arrived': arrived, 'safe_completion': safety_label,
            'collision': terminal_result == 'COLLISION_TERMINATION',
            'fall': 'FALL' in terminal_result or metrics.get('motion_fall_result') is True,
            'sim_timeout': 'SIM_TIME_LIMIT' in terminal_result,
            'wall_timeout': 'WALL' in terminal_result and 'TIME' in terminal_result,
            'safety_abort': 'SAFETY_ABORT' in terminal_result or bool(facts.get('external_safety_events')),
            'standing_observation_complete': bool(stand_done) if arrived else 'NA',
            'arrival_time_s': metrics.get('elapsed_task_duration_s') if arrived else None,
            'task_duration_s': metrics.get('elapsed_task_duration_s'),
            'distance_start_m': start, 'distance_final_m': end,
            'distance_progress_m': float(start)-float(end) if isinstance(start, (int,float)) and isinstance(end, (int,float)) else None,
            'recovery_share': metrics.get('recovery_share_valid_motion_time'),
            'recovery_episode_count': len(intervals),
            'recovery_episode_median_s': median(durations) if durations else None,
            'recovery_episode_max_s': max(durations) if durations else None,
            'mode_transition_count': metrics.get('mode_transitions'),
            'short_reverse_switch_count': metrics.get('short_reverse_switch_count'),
            'foot_contact_count': metrics.get('foot_contact_count'),
            'foot_impact_count': metrics.get('foot_impact_count'),
            'safety_coverage_complete': (metrics.get('task_window_safety') or {}).get('coverage_complete_for_motion_interval'),
            'actual_initial_x_m': pose['x'], 'actual_initial_y_m': pose['y'], 'actual_initial_yaw_rad': pose['yaw'],
            'crossed_m02_gap_section': crossed_m02_gap,
            'initial_joint_state_status': (context.get('initial_state', {}).get('motion_start_measured_state', {}) or {}).get('status'),
            'actual_E': evidence.get('entry_threshold'), 'actual_X': evidence.get('exit_threshold'),
            'actual_H_s': evidence.get('recovery_min_hold_s'), 'runtime_contract_check': 'PASS',
        })
    return plan, rows


def summarize(rows: list[dict]) -> tuple[list[dict], list[dict]]:
    cells, settings = [], []
    for map_name in MAPS:
        for setting in SETTINGS:
            subset = [r for r in rows if r['map'] == map_name and r['setting_id'] == setting]
            arrivals = sum(r['arrived'] for r in subset)
            lo, hi = wilson(arrivals, len(subset))
            cells.append({
                'map': map_name.removesuffix('.xml'), 'setting_id': setting,
                'planned': 3, 'actual_valid': sum(r['record_validity'] == 'VALID' for r in subset),
                'arrived': arrivals, 'arrival_rate': arrivals / len(subset),
                'arrival_wilson_low': lo, 'arrival_wilson_high': hi,
                'safe_complete': sum(r['safe_completion'] == 'TRUE' for r in subset),
                'safe_complete_unknown': sum(r['safe_completion'] == 'UNKNOWN' for r in subset),
                'collisions': sum(r['collision'] for r in subset), 'falls': sum(r['fall'] for r in subset),
                'sim_timeouts': sum(r['sim_timeout'] for r in subset), 'wall_timeouts': sum(r['wall_timeout'] for r in subset),
                'safety_aborts': sum(r['safety_abort'] for r in subset),
                'arrival_time_n': sum(r['arrival_time_s'] is not None for r in subset),
                'arrival_time_median_s': median_known([r['arrival_time_s'] for r in subset]),
                'end_distance_median_m': median_known([r['distance_final_m'] for r in subset]),
                'sim_timeout_n': sum(r['sim_timeout'] for r in subset),
                'sim_timeout_progress_median': median_known([r['distance_progress_m'] for r in subset if r['sim_timeout']]),
                'recovery_share_median': median_known([r['recovery_share'] for r in subset]),
                'recovery_episode_median_s': median_known([r['recovery_episode_median_s'] for r in subset]),
                'mode_transitions_median': median_known([r['mode_transition_count'] for r in subset]),
                'short_reverse_median': median_known([r['short_reverse_switch_count'] for r in subset]),
                'foot_contacts': sum(int(r['foot_contact_count']) for r in subset if isinstance(r['foot_contact_count'], (int,float))),
                'foot_contact_unknown_runs': sum(not isinstance(r['foot_contact_count'], (int,float)) for r in subset),
                'foot_impacts': sum(int(r['foot_impact_count']) for r in subset if isinstance(r['foot_impact_count'], (int,float))),
                'foot_impact_unknown_runs': sum(not isinstance(r['foot_impact_count'], (int,float)) for r in subset),
            })
    for setting in SETTINGS:
        subset = [r for r in rows if r['setting_id'] == setting]
        arrivals = sum(r['arrived'] for r in subset)
        lo, hi = wilson(arrivals, len(subset))
        settings.append({
            'setting_id': setting, 'planned': 18, 'actual_valid': sum(r['record_validity'] == 'VALID' for r in subset),
            'arrived': arrivals, 'arrival_rate': arrivals / len(subset),
            'arrival_wilson_low': lo, 'arrival_wilson_high': hi,
            'safe_complete': sum(r['safe_completion'] == 'TRUE' for r in subset),
            'safe_complete_unknown': sum(r['safe_completion'] == 'UNKNOWN' for r in subset),
            'collisions': sum(r['collision'] for r in subset), 'falls': sum(r['fall'] for r in subset),
            'sim_timeouts': sum(r['sim_timeout'] for r in subset), 'wall_timeouts': sum(r['wall_timeout'] for r in subset),
            'safety_aborts': sum(r['safety_abort'] for r in subset),
            'arrival_time_n': sum(r['arrival_time_s'] is not None for r in subset),
            'arrival_time_median_s': median_known([r['arrival_time_s'] for r in subset]),
            'end_distance_median_m': median_known([r['distance_final_m'] for r in subset]),
            'recovery_share_median': median_known([r['recovery_share'] for r in subset]),
            'recovery_episode_median_s': median_known([r['recovery_episode_median_s'] for r in subset]),
            'mode_transitions_median': median_known([r['mode_transition_count'] for r in subset]),
            'short_reverse_median': median_known([r['short_reverse_switch_count'] for r in subset]),
            'foot_contacts': sum(int(r['foot_contact_count']) for r in subset if isinstance(r['foot_contact_count'], (int,float))),
            'foot_contact_unknown_runs': sum(not isinstance(r['foot_contact_count'], (int,float)) for r in subset),
            'foot_impacts': sum(int(r['foot_impact_count']) for r in subset if isinstance(r['foot_impact_count'], (int,float))),
            'foot_impact_unknown_runs': sum(not isinstance(r['foot_impact_count'], (int,float)) for r in subset),
        })
    return cells, settings


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_outcomes(cells: list[dict], rows: list[dict]) -> None:
    fig, axes = plt.subplots(1, 6, figsize=(23, 4.8), sharey=True)
    for ax, name in zip(axes, [m.removesuffix('.xml') for m in MAPS]):
        subset = [next(c for c in cells if c['map'] == name and c['setting_id'] == s) for s in SETTINGS]
        x = np.arange(4)
        arrived = np.array([c['arrived'] for c in subset], dtype=float)
        safe = np.array([c['safe_complete'] for c in subset], dtype=float)
        errors = np.array([[max(0, 3*c['arrival_wilson_low']-c['arrived']),
                            max(0, 3*c['arrival_wilson_high']-c['arrived'])] for c in subset]).T
        ax.bar(x-.18, arrived, width=.34, color='#3D76A8', label='到达', yerr=errors, capsize=2)
        ax.bar(x+.18, safe, width=.34, color='#65A878', label='确认安全完成')
        for i,c in enumerate(subset): ax.text(i, max(arrived[i],safe[i])+.12, f"{c['arrived']}/3", ha='center', fontsize=8)
        ax.set_title(name)
        ax.set_xticks(x, ['A_REF','E−.03\nX−.06','E−.04\nX−.07','E−.06\nX−.09'], rotation=0, fontsize=8)
        ax.set_ylim(0,4.0); ax.grid(axis='y',alpha=.2)
    axes[0].set_ylabel('次数（每格3次）')
    axes[0].legend(fontsize=8)
    fig.suptitle('R1 v2六张开发图：到达与确认安全完成（每格3次）')
    fig.tight_layout()
    fig.savefig(OUT/'地图到达与安全完成.png', dpi=180)
    plt.close(fig)

    categories=[('collision','碰撞','#C44E52'),('fall','跌倒','#8172B2'),
                ('sim_timeout','仿真限时','#DD8452'),('wall_timeout','墙钟保护','#937860'),
                ('safety_abort','安全中止','#64B5CD')]
    labels=[f"{m.removesuffix('.xml')}\n{s.replace('B_','').replace('_',' ')}" for m in MAPS for s in SETTINGS]
    x=np.arange(len(labels)); base=np.zeros(len(labels))
    fig,ax=plt.subplots(figsize=(17,5.5))
    for key,label,color in categories:
        vals=np.array([sum(bool(r[key]) for r in rows if r['map']==m and r['setting_id']==s) for m in MAPS for s in SETTINGS])
        ax.bar(x, vals, bottom=base, label=label, color=color, width=.76)
        for i,v in enumerate(vals):
            if v: ax.text(i,base[i]+v/2,str(int(v)),ha='center',va='center',color='white',fontsize=8)
        base+=vals
    ax.set_xticks(x,labels,fontsize=7); ax.set_ylabel('失败次数（每格计划3次）')
    ax.set_title('失败类型分布；合法导航失败均保留'); ax.grid(axis='y',alpha=.2)
    ax.legend(ncol=5,loc='upper center',bbox_to_anchor=(.5,-.13),frameon=False)
    fig.tight_layout(); fig.savefig(OUT/'失败类型分布.png',dpi=180,bbox_inches='tight'); plt.close(fig)


def plot_distance_time(rows: list[dict]) -> None:
    fig,axes=plt.subplots(6,2,figsize=(13,18))
    for i,name in enumerate(MAPS):
        at,ad=axes[i]
        for j,s in enumerate(SETTINGS):
            sub=[r for r in rows if r['map']==name and r['setting_id']==s]
            times=[r['arrival_time_s'] for r in sub if isinstance(r['arrival_time_s'],(int,float))]
            dists=[r['distance_final_m'] for r in sub if isinstance(r['distance_final_m'],(int,float))]
            at.scatter(np.full(len(times),j)+np.linspace(-.07,.07,max(1,len(times))),times,color=COLORS[j],s=24,label=s)
            ad.scatter(np.full(len(dists),j)+np.linspace(-.09,.09,max(1,len(dists))),dists,color=COLORS[j],s=24)
        at.set_title(f'{name.removesuffix(".xml")}：成功到达耗时'); at.set_ylabel('仿真秒'); at.set_xticks(range(4),SETTINGS,rotation=20,ha='right',fontsize=7); at.grid(alpha=.2)
        ad.set_title(f'{name.removesuffix(".xml")}：全部运行终点目标距离'); ad.axhline(.5,color='black',ls='--',lw=.8); ad.set_ylabel('米'); ad.set_xticks(range(4),SETTINGS,rotation=20,ha='right',fontsize=7); ad.grid(alpha=.2)
        if i==0: at.legend(fontsize=7)
    fig.suptitle('成功到达耗时（仅成功子集）与全部运行终点目标距离')
    fig.tight_layout(rect=(0,0,1,.99)); fig.savefig(OUT/'到达耗时与终点距离.png',dpi=180); plt.close(fig)


def plot_scene_type_comparison(rows: list[dict]) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    for ax, metric, title, ylabel in (
        (axes[0], 'arrived', '到达', '到达次数 / 9'),
        (axes[1], 'collision', '碰撞', '碰撞次数 / 9'),
    ):
        width = .34
        x = np.arange(len(SETTINGS))
        for offset, (kind, maps, color) in zip((-.18, .18), (('d09', SCENE_TYPES['d09'], '#4C72B0'),
                                                               ('m02', SCENE_TYPES['m02'], '#DD8452'))):
            vals = [sum(bool(r[metric]) for r in rows if r['map'] in maps and r['setting_id'] == s)
                    for s in SETTINGS]
            ax.bar(x + offset, vals, width, label=kind, color=color)
        ax.set_title(title); ax.set_ylabel(ylabel); ax.set_xticks(x, SETTINGS, rotation=18, ha='right', fontsize=8)
        ax.set_ylim(0, 9); ax.grid(axis='y', alpha=.2); ax.legend(title='场景类')
    fig.suptitle('按地图类型分层的结果（每类×设置计划9次）')
    fig.tight_layout(); fig.savefig(OUT/'场景类型比较.png', dpi=180); plt.close(fig)


def plot_representative_tracks(rows: list[dict]) -> list[dict]:
    chosen = []
    for kind, maps in SCENE_TYPES.items():
        sub = [r for r in rows if r['map'] in maps]
        for target in ('COLLISION_TERMINATION', 'ARRIVED'):
            candidate = next((r for r in sub if r['terminal_result'] == target), None)
            if candidate and candidate['plan_item_id'] not in {r['plan_item_id'] for r in chosen}:
                chosen.append(candidate)
    if not chosen:
        return []
    from matplotlib.patches import Circle, Polygon
    fig, axes = plt.subplots(len(chosen), 1, figsize=(11, 3.5*len(chosen)), squeeze=False)
    case_rows = []
    colors = dict(zip(SETTINGS, COLORS))
    for ax, row in zip(axes[:, 0], chosen):
        scene_xml = ROOT / 'unitree_mujoco/unitree_robots/go2' / row['map']
        tree = ET.parse(scene_xml)
        for geom in tree.findall('.//worldbody/geom'):
            if not (geom.get('name') or '').startswith('ppt_obstacle_'):
                continue
            pos = [float(v) for v in geom.get('pos', '0 0 0').split()]
            size = [float(v) for v in geom.get('size', '0 0 0').split()]
            if geom.get('type') == 'cylinder':
                ax.add_patch(Circle((pos[0], pos[1]), size[0], color='#555555', alpha=.6))
            elif geom.get('type') == 'box':
                yaw = float((geom.get('euler') or '0 0 0').split()[2])
                c, s = math.cos(yaw), math.sin(yaw)
                corners = [(-size[0], -size[1]), (size[0], -size[1]),
                           (size[0], size[1]), (-size[0], size[1])]
                points = [(pos[0]+u*c-v*s, pos[1]+u*s+v*c) for u,v in corners]
                ax.add_patch(Polygon(points, closed=True, color='#555555', alpha=.6))
        run_root = BATCH/'runs'/row['plan_item_id']/'result'
        frames = task_frames(find_one(run_root, 'runtime_record.jsonl'))
        if isinstance(row['task_duration_s'], (int, float)):
            frames = [f for f in frames if f['elapsed_s'] <= float(row['task_duration_s']) + 1e-6]
        xy = [(f.get('x'), f.get('y')) for f in frames
              if isinstance(f.get('x'), (int,float)) and isinstance(f.get('y'), (int,float))]
        if xy:
            ax.plot([p[0] for p in xy], [p[1] for p in xy], color=colors[row['setting_id']], lw=1.5)
            ax.scatter([xy[0][0]], [xy[0][1]], marker='o', color='#2ca02c', s=22, label='起点')
            ax.scatter([xy[-1][0]], [xy[-1][1]], marker='x', color='#c44e52', s=32, label='终点')
        ax.scatter([GOAL[0]], [GOAL[1]], marker='*', color='#e6a700', s=50, label='目标')
        ax.set_xlim(-.5, 8); ax.set_ylim(-3, 3); ax.set_aspect('equal', adjustable='box'); ax.grid(alpha=.18)
        ax.set_title(f"{row['map']} · {row['setting_id']} · {row['terminal_result']} · {row['plan_item_id']}", loc='left', fontsize=9)
        ax.set_xlabel('x / m'); ax.set_ylabel('y / m')
        case_rows.append({k:row[k] for k in ('plan_item_id','map','setting_id','run_id','terminal_result','task_duration_s','distance_final_m')})
    axes[0,0].legend(ncol=3, fontsize=8, loc='upper left')
    fig.suptitle('代表实际轨迹与地图障碍（只绘制导航任务窗）')
    fig.tight_layout(); fig.savefig(OUT/'代表轨迹与障碍.png', dpi=180); plt.close(fig)
    return case_rows


def select_case(rows: list[dict], predicate) -> dict | None:
    candidates=[r for r in rows if predicate(r)]
    return max(candidates,key=lambda r:float(r['task_duration_s'] or 0)) if candidates else None


def plot_timelines(rows: list[dict]) -> list[dict]:
    candidates=[select_case(rows,lambda r:r['collision']),select_case(rows,lambda r:r['sim_timeout']),select_case(rows,lambda r:r['arrived'])]
    chosen=[]
    for r in candidates:
        if r and r['plan_item_id'] not in {x['plan_item_id'] for x in chosen}: chosen.append(r)
    if not chosen:
        return []
    fig,axes=plt.subplots(len(chosen),1,figsize=(12,3.5*len(chosen)),squeeze=False)
    cases=[]
    for ax,row in zip(axes[:,0],chosen):
        root=BATCH/'runs'/row['plan_item_id']/'result'
        frames=task_frames(find_one(root,'runtime_record.jsonl'))
        if isinstance(row['task_duration_s'], (int, float)):
            frames=[f for f in frames if f['elapsed_s'] <= float(row['task_duration_s']) + 1e-6]
        t=[x['elapsed_s'] for x in frames]; dist=[x['distance_m'] for x in frames]
        ra=[float(x['ra']) if isinstance(x['ra'],(int,float)) else float('nan') for x in frames]
        modes=[str(x['mode']).upper() for x in frames]
        for j in range(len(frames)-1):
            if 'RECOVERY' in modes[j]: ax.axvspan(t[j],t[j+1],color='#F2C14E',alpha=.22,lw=0)
        ax.plot(t,dist,color='#3D76A8',lw=1.4,label='目标距离')
        ax.axhline(.5,color='#3D76A8',ls=':',lw=.8,label='到达半径')
        ax.set_ylabel('米'); ax.set_xlabel('RL任务窗仿真秒；黄色为Recovery')
        twin=ax.twinx(); twin.plot(t,ra,color='#C44E52',lw=1,label='RA')
        E,X,_=EXPECTED[row['setting_id']]
        twin.axhline(E,color='#C44E52',ls='--',lw=.8,label=f'E={E:.2f}')
        if row['setting_id']!='A_REF': twin.axhline(X,color='#8172B2',ls='--',lw=.8,label=f'X={X:.2f}')
        twin.set_ylabel('RA'); ax.set_title(f"{row['map']} {row['setting_id']} / {row['terminal_result']} / {row['plan_item_id']}",loc='left',fontsize=9); ax.grid(alpha=.18)
        h1,l1=ax.get_legend_handles_labels(); h2,l2=twin.get_legend_handles_labels(); ax.legend(h1+h2,l1+l2,loc='upper right',fontsize=7,ncol=3)
        cases.append({k:row[k] for k in ('plan_item_id','map','setting_id','run_id','terminal_result','task_duration_s','distance_final_m')})
    fig.suptitle('代表运行时间线：目标距离、RA与Recovery状态')
    fig.tight_layout(rect=(0,0,1,.98)); fig.savefig(OUT/'代表运行时间线.png',dpi=180); plt.close(fig)
    return cases


def pct(x: float | None) -> str:
    return 'UNKNOWN' if x is None else f'{100*x:.1f}%'


def num(x: float | None, digits: int = 2) -> str:
    return 'UNKNOWN' if x is None else f'{x:.{digits}f}'


def write_report(plan: dict, rows: list[dict], cells: list[dict], totals: list[dict], cases: list[dict]) -> None:
    size=sum(p.stat().st_size for p in BATCH.rglob('*') if p.is_file())
    terminal=Counter(r['terminal_result'] for r in rows)
    complete=sum(i['status']=='COMPLETE' for i in plan['schedule']['items'])
    safe_true=sum(r['safe_completion']=='TRUE' for r in rows)
    safe_unknown=sum(r['safe_completion']=='UNKNOWN' for r in rows)
    starts=plan['counters']['actual_scene_starts']
    replacements=plan['counters']['replacement_scene_starts']
    md=['# R1-5：五张M2地图的双阈值位置比较','','更新：2026-10-30。R1开发比较；不是独立验证。分析使用本批原始策略帧、context、terminal、metrics和进程记录。','',
        '## 执行情况','',
        f"- 计划/有效完成：120/{complete}；实际场景启动：{starts}；补做：{replacements}；导航期碰撞、跌倒、限时及安全中止均作为结果保留，不重跑。",
        f"- 冻结调度：6轮、每轮20项；调度种子{plan['schedule']['seed']}。启动终止计数：到达{terminal['ARRIVED']}、碰撞{terminal['COLLISION_TERMINATION']}、仿真限时{sum(r['sim_timeout'] for r in rows)}、墙钟保护{sum(r['wall_timeout'] for r in rows)}、跌倒{sum(r['fall'] for r in rows)}、安全中止{sum(r['safety_abort'] for r in rows)}。",
        f'- 安全完成TRUE：{safe_true}；UNKNOWN：{safe_unknown}。到达后2秒站立观察完成{sum(r["standing_observation_complete"] is True for r in rows)}次。',
        f"- 本批原始目录占用{size/1024**3:.2f} GiB；最后一次磁盘检查可用{plan.get('storage_checkpoints',[{}])[-1].get('filesystem_free_bytes',0)/1024**3:.1f} GiB。",
        '- 隐藏MuJoCo窗口、RViz关闭、无截图录像。每项运行后均要求诊断器确认关闭和清理完成。','',
        f"- 同一DISPLAY/Xauthority上下文的GLFW预检：{(plan.get('preflight') or {}).get('status','UNKNOWN')}；Git、MuJoCo和控制器/模型/配置版本仅记录一次，见`run_plan.json`（Git {plan.get('versions',{}).get('git_commit')}；MuJoCo {plan.get('versions',{}).get('runtime_mujoco_version')}）。", '',
        '地图来自M2 selected_v2，地图身份与XML绑定逐项核对：d01→m2v2_d01、d03→m2v2_d03、m01→m2v2_m01、d02→m2v2_d02、d08→m2v2_d08。m01沿用r5修订；d08保留selected_v2原身份，索引记录其来源为M1几何未改。d01/d03/m01与R1-4同版本。场景身份、参数运行证据、记录有效性和清理状态逐项检查。', '',
        '## 固定设置与比较解释','',
        '| 设置 | 运行机制 | E | 有效X | H | 与其他设置的比较含义 |','|---|---|---:|---:|---:|---|',
        '| A_REF | 单阈值A | −0.05 | −0.05 | 0 | 参考 |',
        '| B_E02_X05 | 双阈值B | −0.02 | −0.05 | 0 | 相对B_E05_X08整体移动阈值；相对A同时改变E和滞回 |',
        '| B_E05_X08 | 双阈值B | −0.05 | −0.08 | 0 | 与A同E，比较增加滞回的作用 |',
        '| B_E08_X11 | 双阈值B | −0.08 | −0.11 | 0 | 相对其他B组固定宽度0.03移动阈值位置 |','',
        '当前规则RA≥E进入Recovery；所以E=−0.02较晚进入，E=−0.08较早进入。三种双阈值均宽0.03。命令入口要求A/B也提供低于E的X和正H兼容值；A的X字段被省略、有效X=E，A/B的保持字段被省略、实际H=0。批次清单记录了这一入口语义，运行时逐次证据仍须匹配上述有效参数。','',
        '## 逐地图20格结果','',
        '每格计划6次；一次结果约占16.7个百分点。区间为到达率Wilson 95%区间。安全完成UNKNOWN独立计数。',
        '| 地图 | 设置 | 有效/6 | 到达 | 到达率（95%区间） | 安全完成 | UNKNOWN | 碰撞 | 跌倒 | 仿真限时 | 超时推进中位数m | 墙钟保护 | 安全中止 | 终点距离中位数m |',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for c in cells:
        md.append(f"| {c['map']} | {c['setting_id']} | {c['actual_valid']}/6 | {c['arrived']} | {pct(c['arrival_rate'])} ({100*c['arrival_wilson_low']:.1f}–{100*c['arrival_wilson_high']:.1f}%) | {c['safe_complete']} | {c['safe_complete_unknown']} | {c['collisions']} | {c['falls']} | {c['sim_timeouts']} | {num(c['sim_timeout_progress_median'])} | {c['wall_timeouts']} | {c['safety_aborts']} | {num(c['end_distance_median_m'])} |")
    md += ['', '## 每种设置30项汇总','',
           '| 设置 | 有效/30 | 到达 | 到达率（Wilson 95%） | 安全完成 | UNKNOWN | 碰撞 | 跌倒 | 仿真限时 | 墙钟保护 | 安全中止 | 到达耗时中位数（成功n） | 终点距离中位数m | Recovery占比中位数 | 单次驻留中位数s | 切换中位数 | 短反向中位数 | 足端接触/撞击已知计数 |','|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for t in totals:
        md.append(f"| {t['setting_id']} | {t['actual_valid']}/30 | {t['arrived']} | {pct(t['arrival_rate'])} ({100*t['arrival_wilson_low']:.1f}–{100*t['arrival_wilson_high']:.1f}%) | {t['safe_complete']} | {t['safe_complete_unknown']} | {t['collisions']} | {t['falls']} | {t['sim_timeouts']} | {t['wall_timeouts']} | {t['safety_aborts']} | {num(t['arrival_time_median_s'])}s (n={t['arrival_time_n']}) | {num(t['end_distance_median_m'])} | {num(100*t['recovery_share_median'] if t['recovery_share_median'] is not None else None,1)}% | {num(t['recovery_episode_median_s'])} | {num(t['mode_transitions_median'],0)} | {num(t['short_reverse_median'],0)} | {t['foot_contacts']}/{t['foot_impacts']} |")
    md += ['', '## 对比观察','',
           '先按地图看20格计数，不以30项总数掩盖地图差异。每张地图的四设置到达/安全完成/碰撞/仿真限时列在下方；终点距离、耗时、Recovery和切换行为见CSV及图。', '']
    for m in MAPS:
        cs=[c for c in cells if c['map']==m.removesuffix('.xml')]
        parts='；'.join(f"{c['setting_id']} {c['arrived']}/{c['safe_complete']}/{c['collisions']}/{c['sim_timeouts']}" for c in cs)
        md.append(f'- **{m.removesuffix(".xml")}：** 到达/安全完成/碰撞/仿真限时：{parts}。')
    md += ['',
           '### 结果解读', '',
           '同E对照中，B_E05_X08相对A_REF的到达为17/30对20/30，碰撞12对9，仿真限时各1次；到达数在d01、d03、d02较少，在m01、d08各多1次。此次没有显示滞回带来跨地图一致的任务收益。',
           '固定宽度三种B设置合计到达分别为20/30、17/30、22/30。较早进入的B_E08_X11在d01、d03、m01到达数较高，但d08为2/6并出现2次仿真限时；d02为3/6且有3次碰撞。按地图差异看阈值位置的表现依赖场景，22/30的合计不能支持普遍最优结论。',
           'A_REF与B_E02_X05合计均为20/30到达；B_E02_X05碰撞10次、没有仿真限时，A_REF碰撞9次、有1次限时。该整体对比同时改变进入阈值和滞回状态，不能解释为滞回单独作用。',
           'd02与d08的推进代价需要和终局类别一起看：4次仿真限时中，d02/A_REF推进1.68m、d02/B_E05_X08推进1.89m；d08/B_E08_X11两次推进3.14m和4.35m（中位数3.74m），限时时终点仍距目标3.19m和4.40m。d02共12次碰撞，五图中最多；但每格6次，不将地图碰撞率外推为一般地图风险。',
           '成功到达耗时只在到达子集中比较，各设置成功样本数不同。切换次数与Recovery占比作为行为描述，不单独代表效果。足端接触/撞击计数保留为独立字段，不并入环境碰撞。','',
           '仿真限时的距离变化以起始目标距离减终点目标距离报告，正值表示推进。', '',
           '图表：','',
           '- ![地图到达与安全完成](地图到达与安全完成.png)',
           '- ![失败类型分布](失败类型分布.png)',
           '- ![到达耗时与终点距离](到达耗时与终点距离.png)',
           '- ![代表运行时间线](代表运行时间线.png)', '',
           '## 代表运行','',
           '按数据可用性，选取一例碰撞、一例仿真限时和一例到达（若对应事件存在）。时间线中的距离由实际位姿计算，RA和模式来自策略帧；黄色背景为Recovery。时间上的接近只作描述，不据此推断因果。', '',
           '| 计划项 | 地图/设置 | 终止 | 任务耗时 | 终点目标距离 |','|---|---|---|---:|---:|']
    for c in cases: md.append(f"| {c['plan_item_id']} | {c['map']} / {c['setting_id']} | {c['terminal_result']} | {float(c['task_duration_s']):.2f}s | {float(c['distance_final_m']):.2f}m |")
    md += ['', '## 限制与数据','',
           '- 这是开发比较，五图按M2 selected_v2筛选而来，不是独立留出地图；每格6次且初始关节状态未严格重放。调度随机种子只打乱计划顺序，不保证物理初态相同。',
           '- 有效的控制证据来自运行上下文；A的单阈值退出由E导出。入口兼容参数与有效运行值分别记录，解释时以实际消费值为准。',
           '- 本报告不宣称某个设置普遍最优。负面结果、UNKNOWN及记录字段缺失按实际记录保留。','',
           f'- 原始批次：`logs/09_R1_switch_constraints/m2_threshold_position_120/{BATCH.name}/`（{size/1024**3:.2f} GiB）。',
           '- 逐次表：`逐次运行明细.csv`；20格表：`地图设置汇总.csv`；四设置汇总：`设置汇总.csv`。',
           '- 报告与图表：当前目录。','']
    (OUT/'分析报告.md').write_text('\n'.join(md),encoding='utf-8')


def write_r1_report(plan: dict, rows: list[dict], cells: list[dict], totals: list[dict], cases: list[dict]) -> None:
    size = sum(p.stat().st_size for p in BATCH.rglob('*') if p.is_file())
    terminal = Counter(r['terminal_result'] for r in rows)
    completed = sum(i['status'] == 'COMPLETE' for i in plan['schedule']['items'])
    md = [
        '# R1 v2六图×四设置×三次开发比较', '',
        '本批是按d09连续障碍与m02有限空间分层的开发筛查，不是独立验证。每格仅3次，初始关节状态未严格重放。', '',
        '## 执行与设置', '',
        f"- 完成记录：{completed}/72；实际场景启动{plan['counters']['actual_scene_starts']}次，非场景调用{plan['counters']['non_scene_invocations']}次，补做{plan['counters']['replacement_scene_starts']}次。终局：到达{terminal['ARRIVED']}、碰撞{terminal['COLLISION_TERMINATION']}、跌倒{terminal['FALL_TERMINATION']}、仿真限时{terminal['DIAGNOSTIC_SIM_TIME_LIMIT']}、墙钟保护{terminal['WALL_CLOCK_GUARD_INCOMPLETE']}、安全中止{terminal['SYSTEM_SAFETY_ABORT']}、其他/UNKNOWN {len(rows)-sum(terminal.values())}。",
        f"- 到达后站立观察完成{sum(r['standing_observation_complete'] is True for r in rows)}次；安全完成TRUE {sum(r['safe_completion']=='TRUE' for r in rows)}、FALSE {sum(r['safe_completion']=='FALSE' for r in rows)}、UNKNOWN {sum(r['safe_completion']=='UNKNOWN' for r in rows)}。",
        '- 统一目标(7.5,0)m、半径0.5m、仿真/墙钟上限20/120秒、到达后站立观察2仿真秒；A组E=X=−0.05，B三档按表，H=0。',
        f"- 3轮交错排程，种子{plan['schedule']['seed']}；本批占用{size/1024**3:.2f} GiB。隐藏GLFW、RViz关闭，无截图录像。",
        f"- 每项以实际context、控制参数、场景身份、VALID记录及shutdown/closeout字段逐项核对；同环境GLFW预检状态：{(plan.get('preflight') or {}).get('status','UNKNOWN')}。运行版本记录见`run_plan.json`。", '',
        '| 设置 | 组 | E | 有效X | 实际H | 说明 |', '|---|---|---:|---:|---:|---|',
        '| A_REF | A | −0.05 | −0.05 | 0 | A参考，单阈值 |',
        '| B_E03_X06 | B | −0.03 | −0.06 | 0 | 双阈值，宽度0.03 |',
        '| B_E04_X07 | B | −0.04 | −0.07 | 0 | 双阈值，宽度0.03 |',
        '| B_E06_X09 | B | −0.06 | −0.09 | 0 | 双阈值，宽度0.03 |',
        '', 'B相对A同时改变E和X，不能把整体差异单独归因于滞回；A的兼容CLI参数不代表启用保持，四组实际H均为0。', '',
        '## 24格结果', '',
        '| 地图 | 设置 | 有效/3 | 终局 | 安全完成 T/F/U | 碰撞 | 跌倒 | 仿真限时 | 墙钟保护 | 安全中止 | 到达耗时中位数s（成功n） | 终点距离中位数m | 推进中位数m |',
        '|---|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for c in cells:
        sub = [r for r in rows if r['map'].removesuffix('.xml') == c['map'] and r['setting_id'] == c['setting_id']]
        terminals = ', '.join(f'{k}:{n}' for k,n in sorted(Counter(r['terminal_result'] for r in sub).items())) or '无'
        md.append(f"| {c['map']} | {c['setting_id']} | {c['actual_valid']}/3 | {terminals} | {c['safe_complete']}/{sum(r['safe_completion']=='FALSE' for r in sub)}/{c['safe_complete_unknown']} | {c['collisions']} | {c['falls']} | {c['sim_timeouts']} | {c['wall_timeouts']} | {c['safety_aborts']} | {num(c['arrival_time_median_s'])} (n={c['arrival_time_n']}) | {num(c['end_distance_median_m'])} | {num(median_known([r['distance_progress_m'] for r in sub]))} |")
    md += ['', '## 两类场景分层比较', '',
           '每类每设置合计9项；下表并列报告到达、确认安全完成、碰撞、跌倒和限时。小样本仅看作开发线索。', '',
           '| 场景类 | 设置 | 到达/9 | 安全完成TRUE | 碰撞 | 跌倒 | 仿真限时 | 到达耗时中位数s（n） | 终点距离中位数m |',
           '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for kind, maps in SCENE_TYPES.items():
        for setting in SETTINGS:
            sub = [r for r in rows if r['map'] in maps and r['setting_id'] == setting]
            arrivals = [r['arrival_time_s'] for r in sub if r['arrival_time_s'] is not None]
            md.append(f"| {kind} | {setting} | {sum(r['arrived'] for r in sub)}/9 | {sum(r['safe_completion']=='TRUE' for r in sub)} | {sum(r['collision'] for r in sub)} | {sum(r['fall'] for r in sub)} | {sum(r['sim_timeout'] for r in sub)} | {num(median_known(arrivals))} (n={len(arrivals)}) | {num(median_known([r['distance_final_m'] for r in sub]))} |")
    b_m02 = [r for r in rows if r['map'] in SCENE_TYPES['m02'] and r['runtime_group'] == 'B']
    md += ['', '### m02研究区域路径检查', '',
           '以03/07两障碍实际轮廓之间的近似横截面x=5.30–5.60m、y=−1.67–0.42m统计任务窗位姿。B组三档进入该区域的运行数为：' +
           '；'.join(f"{s} {sum(r['crossed_m02_gap_section'] is True for r in b_m02 if r['setting_id']==s)}/9" for s in SETTINGS[1:]) +
           '。合计26/27；唯一未进入的是中档B_E04_X07，轨迹在约(x=5.25,y=0.33)m碰03后终止。该检查只说明中心位姿经过区域，不核对整机轮廓净空。','',
           '## 结果解读与候选','',
           'd09：A_REF与B_E03_X06均为8/9到达，B_E04_X07与B_E06_X09均为7/9。没有设置在d09上超过A；B_E03可作与A表现相近的对照，未显示收益。',
           'm02：A_REF与B_E03_X06均为1/9到达，B_E04_X07为4/9、B_E06_X09为3/9；相应碰撞数为8、8、5、6。E04值得保留为m02局部候选，E06可作次级对照；两者在d09均为7/9，不能称为跨场景改善。每格3次且各档同源，这些结果只作筛查线索。',
           '全部设置18次合并后到达为A_REF 9/18、B_E03_X06 9/18、B_E04_X07 11/18、B_E06_X09 10/18。合计差别主要来自m02，不能证明普遍收益；到达耗时只按成功样本比较且样本数不同。', '',
           '## 设置总览','',
           '| 设置 | 有效/18 | 到达 | 安全完成TRUE/FALSE/UNKNOWN | 碰撞 | 跌倒 | 仿真限时 | 墙钟保护 | 安全中止 | 到达耗时中位数s（成功n） | 终点距离中位数m | Recovery占比中位数 | 切换次数中位数 |','|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for t in totals:
        sub = [r for r in rows if r['setting_id'] == t['setting_id']]
        md.append(f"| {t['setting_id']} | {t['actual_valid']}/18 | {t['arrived']} | {t['safe_complete']}/{sum(r['safe_completion']=='FALSE' for r in sub)}/{t['safe_complete_unknown']} | {t['collisions']} | {t['falls']} | {t['sim_timeouts']} | {t['wall_timeouts']} | {t['safety_aborts']} | {num(t['arrival_time_median_s'])} (n={t['arrival_time_n']}) | {num(t['end_distance_median_m'])} | {num(t['recovery_share_median'])} | {num(t['mode_transitions_median'],0)} |")
    md += ['', '耗时只比较到达样本，样本数不同，不能据此宣称整体更快。Recovery占比和切换次数仅解释行为，不单独代表效果。六图同源，不视为六个独立环境。', '',
           '## 图表与代表轨迹', '',
           '- ![两类场景比较](场景类型比较.png)',
           '- ![六图到达与安全完成](地图到达与安全完成.png)',
           '- ![终局类型](失败类型分布.png)',
           '- ![耗时与终点距离](到达耗时与终点距离.png)',
           '- ![代表轨迹](代表轨迹与障碍.png)',
           '- ![代表运行时间线](代表运行时间线.png)', '',
           '代表轨迹按任务耗时截断，排除终局后站立和收尾段；轨迹用于展示位置，不等于安全或策略可达证明。', '',
           '## 数据入口与限制', '',
           f"- 逐次表：`逐次运行明细.csv`；地图×设置表：`地图设置汇总.csv`；设置汇总：`设置汇总.csv`。原始批次保存在`{BATCH.relative_to(ROOT)}/`。",
           '- 每格3次属于开发筛查；不能证明通过率、普遍最优或跨地图泛化。A与B同时改变E和X；调度种子只随机顺序，不保证物理初态重放。',
           '- 安全完成、位姿、Recovery及切换字段按现有记录口径保留；缺失指标为UNKNOWN，不据UNKNOWN形成安全结论。', '']
    (OUT/'分析报告.md').write_text('\n'.join(md), encoding='utf-8')


def main() -> None:
    plan,rows=collect()
    cells,totals=summarize(rows)
    write_csv(OUT/'逐次运行明细.csv',rows)
    write_csv(OUT/'地图设置汇总.csv',cells)
    write_csv(OUT/'设置汇总.csv',totals)
    plot_outcomes(cells,rows)
    plot_distance_time(rows)
    plot_scene_type_comparison(rows)
    cases=plot_timelines(rows)
    tracks=plot_representative_tracks(rows)
    (OUT/'代表案例.json').write_text(json.dumps(cases,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (OUT/'代表轨迹案例.json').write_text(json.dumps(tracks,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    write_r1_report(plan,rows,cells,totals,cases)
    print(f'Analyzed {len(rows)} valid runs; output={OUT}')


if __name__=='__main__':
    main()
