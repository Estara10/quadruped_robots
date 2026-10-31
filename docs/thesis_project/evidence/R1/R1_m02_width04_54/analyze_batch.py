#!/usr/bin/env python3
"""R1-8 54-run m02 hysteresis-width comparison, reusing R1-4 parsing helpers."""
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

os.environ.setdefault('MPLCONFIGDIR', '/tmp/r1-m02-width04-54-matplotlib')
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
BATCH = ROOT / 'logs/09_R1_switch_constraints/r1_m02_width04_54/20261031_r1_m02_width04_54'
GOAL = (7.5, 0.0)
MAPS = ['r1_m02_space_wide_v2.xml', 'r1_m02_space_middle_v2.xml', 'r1_m02_space_narrow_v2.xml']
SCENE_TYPES = {'m02': MAPS}
SETTINGS = ['B_W04_E01_X05','B_W04_E02_X06','B_W04_E03_X07',
            'B_W04_E04_X08','B_W04_E05_X09','B_W04_E06_X10']
EXPECTED = {
    'B_W04_E01_X05': (-0.01, -0.05, True),
    'B_W04_E02_X06': (-0.02, -0.06, True),
    'B_W04_E03_X07': (-0.03, -0.07, True),
    'B_W04_E04_X08': (-0.04, -0.08, True),
    'B_W04_E05_X09': (-0.05, -0.09, True),
    'B_W04_E06_X10': (-0.06, -0.10, True),
}
COLORS = ['#4C72B0', '#DD8452', '#55A868', '#8172B2', '#C44E52', '#64B5CD']
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
    if len(plan['schedule']['items']) != 54:
        raise RuntimeError('frozen plan must contain all 54 items')
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
        collision_events = (metrics.get('task_window_safety') or {}).get('collision_events') or []
        collision_obstacles = sorted({e.get('obstacle_geom_name') for e in collision_events
                                       if isinstance(e, dict) and e.get('obstacle_geom_name')})
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
            'collision_obstacles': ';'.join(collision_obstacles) if collision_obstacles else ('UNKNOWN' if terminal_result == 'COLLISION_TERMINATION' else ''),
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
            'setting_id': setting, 'planned': 9, 'actual_valid': sum(r['record_validity'] == 'VALID' for r in subset),
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


def allocated_bytes(root: Path) -> int:
    """Count allocated storage once per inode, avoiding hard-link double counts."""
    seen: set[tuple[int, int]] = set()
    total = 0
    for path in root.rglob('*'):
        if not path.is_file():
            continue
        st = path.stat()
        key = (st.st_dev, st.st_ino)
        if key not in seen:
            seen.add(key)
            total += getattr(st, 'st_blocks', 0) * 512 or st.st_size
    return total


def plot_outcomes(cells: list[dict], rows: list[dict]) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(19, 4.8), sharey=True)
    for ax, name in zip(axes, [m.removesuffix('.xml') for m in MAPS]):
        subset = [next(c for c in cells if c['map'] == name and c['setting_id'] == s) for s in SETTINGS]
        x = np.arange(6)
        arrived = np.array([c['arrived'] for c in subset], dtype=float)
        safe = np.array([c['safe_complete'] for c in subset], dtype=float)
        errors = np.array([[max(0, 3*c['arrival_wilson_low']-c['arrived']),
                            max(0, 3*c['arrival_wilson_high']-c['arrived'])] for c in subset]).T
        ax.bar(x-.18, arrived, width=.34, color='#3D76A8', label='到达', yerr=errors, capsize=2)
        ax.bar(x+.18, safe, width=.34, color='#65A878', label='确认安全完成')
        for i,c in enumerate(subset): ax.text(i, max(arrived[i],safe[i])+.12, f"{c['arrived']}/3", ha='center', fontsize=8)
        ax.set_title(name)
        ax.set_xticks(x, ['E−.01\nX−.05','E−.02\nX−.06','E−.03\nX−.07','E−.04\nX−.08','E−.05\nX−.09','E−.06\nX−.10'], rotation=0, fontsize=8)
        ax.set_ylim(0,4.0); ax.grid(axis='y',alpha=.2)
    axes[0].set_ylabel('次数（每格3次）')
    axes[0].legend(fontsize=8)
    fig.suptitle('m02 v2三档：六种宽度0.04双阈值设置的到达与确认安全完成（每格3次）')
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
    fig,axes=plt.subplots(len(MAPS),2,figsize=(12,10),squeeze=False)
    for i,name in enumerate(MAPS):
        at,ad=axes[i]
        for j,s in enumerate(SETTINGS):
            sub=[r for r in rows if r['map']==name and r['setting_id']==s]
            times=[r['arrival_time_s'] for r in sub if isinstance(r['arrival_time_s'],(int,float))]
            dists=[r['distance_final_m'] for r in sub if isinstance(r['distance_final_m'],(int,float))]
            at.scatter(np.full(len(times),j)+np.linspace(-.07,.07,max(1,len(times))),times,color=COLORS[j],s=24,label=s)
            ad.scatter(np.full(len(dists),j)+np.linspace(-.09,.09,max(1,len(dists))),dists,color=COLORS[j],s=24)
        at.set_title(f'{name.removesuffix(".xml")}：成功到达耗时'); at.set_ylabel('仿真秒'); at.set_xticks(range(6),SETTINGS,rotation=28,ha='right',fontsize=7); at.grid(alpha=.2)
        ad.set_title(f'{name.removesuffix(".xml")}：全部运行终点目标距离'); ad.axhline(.5,color='black',ls='--',lw=.8); ad.set_ylabel('米'); ad.set_xticks(range(6),SETTINGS,rotation=28,ha='right',fontsize=7); ad.grid(alpha=.2)
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


def prior_m02_rows() -> list[dict]:
    sources = [
        (ROOT / 'docs/thesis_project/evidence/R1/R1v2_threshold_72/逐次运行明细.csv',
         'R1-6宽度0.03（跨批次）', {'A_REF','B_E03_X06','B_E04_X07','B_E06_X09'}),
        (ROOT / 'docs/thesis_project/evidence/R1/R1_m02_threshold_refine_18/逐次运行明细.csv',
         'R1-7宽度0.03（跨批次）', {'B_E02_X05','B_E05_X08'}),
    ]
    result=[]
    for path,label,allowed in sources:
        with path.open(newline='',encoding='utf-8-sig') as f:
            result.extend({**r,'batch_source':label} for r in csv.DictReader(f)
                          if r.get('map') in MAPS and r.get('setting_id') in allowed)
    return result


def as_num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def as_bool(v):
    return v is True or str(v).lower() == 'true'


def write_candidate_comparison(rows: list[dict]) -> tuple[list[dict], list[dict]]:
    params={
        'A_REF':(-.05,-.05,0.0,None),
        'B_E02_X05':(-.02,-.05,0.0,.03), 'B_E03_X06':(-.03,-.06,0.0,.03),
        'B_E04_X07':(-.04,-.07,0.0,.03), 'B_E05_X08':(-.05,-.08,0.0,.03),
        'B_E06_X09':(-.06,-.09,0.0,.03),
        'B_W04_E01_X05':(-.01,-.05,0.0,.04), 'B_W04_E02_X06':(-.02,-.06,0.0,.04),
        'B_W04_E03_X07':(-.03,-.07,0.0,.04), 'B_W04_E04_X08':(-.04,-.08,0.0,.04),
        'B_W04_E05_X09':(-.05,-.09,0.0,.04), 'B_W04_E06_X10':(-.06,-.10,0.0,.04),
    }
    source_rows=prior_m02_rows()+[{**r,'batch_source':'R1-8宽度0.04（本批）'} for r in rows]
    grouped=[]
    for map_name in MAPS:
        for setting,(E,X,H,width) in params.items():
            subset=[r for r in source_rows if r['map']==map_name and r['setting_id']==setting]
            if not subset: continue
            times=[as_num(r.get('arrival_time_s')) for r in subset]
            times=[x for x in times if x is not None]
            progress=[as_num(r.get('distance_progress_m')) for r in subset]
            progress=[x for x in progress if x is not None]
            distances=[as_num(r.get('distance_final_m')) for r in subset]
            distances=[x for x in distances if x is not None]
            grouped.append({
                'map':map_name,'setting_id':setting,'batch_source':subset[0]['batch_source'],
                'threshold_width':width if width is not None else 'single_threshold_reference',
                'n':len(subset),'E':E,'X':X,'H_s':H,
                'arrived':sum(as_bool(r.get('arrived')) for r in subset),
                'safe_true':sum(r.get('safe_completion')=='TRUE' for r in subset),
                'safe_false':sum(r.get('safe_completion')=='FALSE' for r in subset),
                'safe_unknown':sum(r.get('safe_completion')=='UNKNOWN' for r in subset),
                'collisions':sum(as_bool(r.get('collision')) for r in subset),
                'falls':sum(as_bool(r.get('fall')) for r in subset),
                'sim_timeouts':sum(as_bool(r.get('sim_timeout')) for r in subset),
                'wall_timeouts':sum(as_bool(r.get('wall_timeout')) for r in subset),
                'safety_aborts':sum(as_bool(r.get('safety_abort')) for r in subset),
                'arrival_time_median_s':median(times) if times else None,
                'distance_progress_median':median(progress) if progress else None,
                'final_distance_median':median(distances) if distances else None,
                'gap_center_crossings':sum(as_bool(r.get('crossed_m02_gap_section')) for r in subset),
            })
    write_csv(OUT/'跨批次候选对照.csv',grouped)

    old_by_e={-.02:'B_E02_X05',-.03:'B_E03_X06',-.04:'B_E04_X07',
              -.05:'B_E05_X08',-.06:'B_E06_X09'}
    pairs=[]
    for map_name in MAPS:
        for new_setting,(E,X,H,width) in params.items():
            if width != .04: continue
            current=next(c for c in grouped if c['map']==map_name and c['setting_id']==new_setting)
            old_setting=old_by_e.get(E)
            old=next((c for c in grouped if c['map']==map_name and c['setting_id']==old_setting),None) if old_setting else None
            pairs.append({'map':map_name,'E':E,'old_setting':old_setting or 'NO_SAME_E_REFERENCE',
                'old_batch_source':old['batch_source'] if old else 'NONE',
                'old_n':old['n'] if old else 0,'old_arrived':old['arrived'] if old else None,
                'old_collisions':old['collisions'] if old else None,'old_safe_true':old['safe_true'] if old else None,
                'old_safe_false':old['safe_false'] if old else None,'old_safe_unknown':old['safe_unknown'] if old else None,
                'old_progress_median':old['distance_progress_median'] if old else None,
                'old_arrival_time_median_s':old['arrival_time_median_s'] if old else None,
                'new_setting':new_setting,'new_batch_source':'R1-8宽度0.04（本批）','new_n':current['n'],
                'new_arrived':current['arrived'],'new_collisions':current['collisions'],
                'new_safe_true':current['safe_true'],'new_safe_false':current['safe_false'],
                'new_safe_unknown':current['safe_unknown'],'new_progress_median':current['distance_progress_median'],
                'new_arrival_time_median_s':current['arrival_time_median_s']})
    write_csv(OUT/'同E宽度对照.csv',pairs)

    fig,axes=plt.subplots(1,3,figsize=(16,5),sharey=True)
    es=[-.01,-.02,-.03,-.04,-.05,-.06]
    for ax,map_name in zip(axes,MAPS):
        subset=[next(p for p in pairs if p['map']==map_name and abs(p['E']-e)<1e-8) for e in es]
        x=np.arange(len(es)); oldvals=[(p['old_arrived'] if p['old_arrived'] is not None else 0) for p in subset]
        newvals=[p['new_arrived'] for p in subset]
        oldbars=ax.bar(x-.19,oldvals,width=.36,color='#8C8C8C',label='宽度0.03（旧批次）')
        newbars=ax.bar(x+.19,newvals,width=.36,color='#3D76A8',label='宽度0.04（本批）')
        for j,p in enumerate(subset):
            ax.text(x[j]+.19,newvals[j]+.04,f"{newvals[j]}/3",ha='center',fontsize=7)
            if p['old_arrived'] is None:
                ax.text(x[j]-.19,.08,'无旧档',ha='center',rotation=90,fontsize=6)
            else:
                ax.text(x[j]-.19,oldvals[j]+.04,f"{oldvals[j]}/3",ha='center',fontsize=7)
        ax.set_xticks(x,[f'{e:.2f}' for e in es],fontsize=8); ax.set_title(map_name.removesuffix('.xml'))
        ax.set_ylim(0,3.7); ax.grid(axis='y',alpha=.2)
    axes[0].set_ylabel('到达次数（每格3次）')
    axes[0].legend(fontsize=7,loc='upper left')
    fig.suptitle('相同E下宽度0.03与0.04到达数对照（跨批次，E=−0.01无旧参考）')
    fig.tight_layout(); fig.savefig(OUT/'同E宽度对照.png',dpi=180); plt.close(fig)
    return grouped,pairs


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


def write_refine_report(plan: dict, rows: list[dict], cells: list[dict], totals: list[dict], comparison: list[dict], cases: list[dict], tracks: list[dict]) -> None:
    terminals=Counter(r['terminal_result'] for r in rows)
    contacts=Counter(name for r in rows for name in r['collision_obstacles'].split(';') if name)
    md=['# R1-7：m02三档两种双阈值补测（18项）','',
        '本批是R1开发阶段的跨批次补测。只运行m02 v2三张地图和两种B设置，不含同期A；旧72项A与三种B仅作跨批次参考，不能当作严格配对。每格3次，三张地图同源。','',
        '## 执行与参数','',
        f"- 完成：{len(rows)}/18 VALID；实际场景启动{plan['counters']['actual_scene_starts']}次、非场景调用{plan['counters']['non_scene_invocations']}次、补做{plan['counters']['replacement_scene_starts']}次。终局：到达{terminals['ARRIVED']}、碰撞{terminals['COLLISION_TERMINATION']}、跌倒{terminals['FALL_TERMINATION']}、仿真限时{terminals['DIAGNOSTIC_SIM_TIME_LIMIT']}、墙钟保护{terminals['WALL_CLOCK_GUARD_INCOMPLETE']}、安全中止{terminals['SYSTEM_SAFETY_ABORT']}、其他/UNKNOWN {len(rows)-sum(terminals.values())}。",
        f"- 安全完成TRUE/FALSE/UNKNOWN：{sum(r['safe_completion']=='TRUE' for r in rows)}/{sum(r['safe_completion']=='FALSE' for r in rows)}/{sum(r['safe_completion']=='UNKNOWN' for r in rows)}；到达后站立观察完成{sum(r['standing_observation_complete'] is True for r in rows)}次。",
        f"- 实际参数逐项匹配：B_E02_X05为E=−0.02/X=−0.05/H=0，B_E05_X08为E=−0.05/X=−0.08/H=0；均为B、保持关闭、双阈值宽度0.03。目标(7.5,0)m，半径0.5m，20/120秒，到达后站立观察2仿真秒。",
        f"- 三轮交错排程，种子{plan['schedule']['seed']}。隐藏MuJoCo、RViz关闭，无截图录像；同上下文GLFW预检{(plan.get('preflight') or {}).get('status','UNKNOWN')}。两个临时overlay已在运行结束后清理；原始记录保留。运行期间批次峰值约{max((x.get('batch_bytes',0) for x in plan.get('storage_checkpoints',[])),default=0)/1024**2:.1f}MiB，清理后约{sum(p.stat().st_size for p in BATCH.rglob('*') if p.is_file())/1024**2:.1f}MiB。",
        '- 版本、实际随机顺序、场景身份和每次运行清理证据见批次`run_plan.json`与`run_schedule.csv`。', '',
        '| 设置 | E | X | H | 用途 |','|---|---:|---:|---:|---|',
        '| B_E02_X05 | −0.02 | −0.05 | 0 | 本批新补测 |',
        '| B_E05_X08 | −0.05 | −0.08 | 0 | 本批新补测 |','',
        '## 本批六格结果','',
        '| 地图 | 设置 | 有效/3 | 到达 | 安全完成T/F/U | 碰撞 | 跌倒 | 仿真限时 | 墙钟保护 | 安全中止 | 到达耗时中位数s（成功n） | 终点距离中位数m | 推进中位数m | 03—07中心轨迹进入数/3 | 记录到的碰撞对象 |',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|']
    for c in cells:
        sub=[r for r in rows if r['map'].removesuffix('.xml')==c['map'] and r['setting_id']==c['setting_id']]
        obs=Counter(n for r in sub for n in r['collision_obstacles'].split(';') if n)
        obs_text='；'.join(f'{k}:{v}' for k,v in obs.items()) or ('UNKNOWN' if c['collisions'] else '无碰撞')
        atimes=[r['arrival_time_s'] for r in sub if r['arrival_time_s'] is not None]
        md.append(f"| {c['map']} | {c['setting_id']} | {c['actual_valid']}/3 | {c['arrived']}/3 | {c['safe_complete']}/{sum(r['safe_completion']=='FALSE' for r in sub)}/{c['safe_complete_unknown']} | {c['collisions']} | {c['falls']} | {c['sim_timeouts']} | {c['wall_timeouts']} | {c['safety_aborts']} | {num(median_known(atimes))} (n={len(atimes)}) | {num(c['end_distance_median_m'])} | {num(median_known([r['distance_progress_m'] for r in sub]))} | {sum(r['crossed_m02_gap_section'] is True for r in sub)}/3 | {obs_text} |")
    md += ['',f"本批碰撞事件记录到的对象：{dict(contacts) if contacts else 'UNKNOWN'}。该字段描述已记录接触对象，不补推未记录接触。",'',
           '## 与旧72项m02结果并列（跨批次）','',
           '下表汇总原72项中的A_REF与三种B，以及本批两种B；每一格均为3次。R1-6旧批次不计入本批18次，也不是本批同期A或严格配对。本批运行、模型和参数版本见本批`run_plan.json`；旧72项版本见[原批次清单](../../../../logs/09_R1_switch_constraints/r1_v2_threshold_72/20261031_r1_v2_threshold72/run_plan.json)。',
           '| 地图 | 设置 | 批次来源 | E/X/H | n | 到达 | 安全完成T/F/U | 碰撞 | 跌倒 | 仿真限时 | 墙钟保护 | 安全中止 | 到达耗时中位数s（成功n） | 终点距离中位数m | 推进中位数m | 03—07中心轨迹进入数 |',
           '|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for c in comparison:
        md.append(f"| {c['map'].removesuffix('.xml')} | {c['setting_id']} | {c['batch_source']} | {c['E']:.2f}/{c['X']:.2f}/{c['H_s']:.0f} | {c['n']} | {c['arrived']} | {c['safe_true']}/{c['safe_false']}/{c['safe_unknown']} | {c['collisions']} | {c['falls']} | {c['sim_timeouts']} | {c['wall_timeouts']} | {c['safety_aborts']} | {num(c['arrival_time_median_s'])} (n={c['arrived']}) | {num(c['final_distance_median'])} | {num(c['distance_progress_median'])} | {c['gap_center_crossings']}/{c['n']} |")
    md += ['',
           '## 判断与限制','',
           '逐档结果显示碰撞主要发生在障碍03；到达样本少，不能把成功子集的耗时解释为整体效率。推进量按每格全部有效运行计算，终点距离和终局需一起看。中心位姿进入03—07近似截面不等于整机通过净空。',
           '本批和旧72项只做跨批次开发线索：运行批次、样本次序、初态与随机扰动不同，且没有同期A。每格3次、三档同源，结果不足以估计通过率或判定一般优势。新两档若仅在某一档地图出现改善，也只构成该布局上的候选线索。',
           '双阈值与A的整体差异同时涉及E和X；两种新设置之间也同时移动阈值位置。H均为0，因此本批不评估保持时间。到达耗时仅对到达子集统计并注明样本数；Recovery与切换仅作行为描述。', '',
           '## 图表与案例','',
           '- ![跨批次六候选对照](跨批次候选对照.png)',
           '- ![本批到达与安全完成](地图到达与安全完成.png)',
           '- ![到达耗时与终点距离](到达耗时与终点距离.png)',
           '- ![代表轨迹与障碍](代表轨迹与障碍.png)',
           '- ![代表运行时间线](代表运行时间线.png)', '',
           '代表轨迹和时间线仅绘制导航任务窗；轨迹用于辅助核对研究区域，不等同于整机净空或策略可达保证。', '',
           '## 数据位置','',
           f"- 本批逐次明细：`逐次运行明细.csv`；六格汇总：`地图设置汇总.csv`；两设置汇总：`设置汇总.csv`；六种候选×三图跨批次对照：`跨批次候选对照.csv`。",
           f"- 原始运行记录：`logs/09_R1_switch_constraints/r1_m02_threshold_refine_18/{BATCH.name}/`。批次约{sum(p.stat().st_size for p in BATCH.rglob('*') if p.is_file())/1024**3:.2f}GiB；旧72项文件未改。", '']
    (OUT/'分析报告.md').write_text('\n'.join(md),encoding='utf-8')


def write_width04_report(plan: dict, rows: list[dict], cells: list[dict], totals: list[dict], grouped: list[dict], pairs: list[dict], cases: list[dict], tracks: list[dict]) -> None:
    terminals=Counter(r['terminal_result'] for r in rows)
    contacts=Counter(name for r in rows for name in r['collision_obstacles'].split(';') if name)
    raw_bytes=allocated_bytes(BATCH)
    md=['# R1-8：m02宽度0.04双阈值比较（54项）','',
        '本批为开发筛查，只运行m02 v2三档与六种宽度0.04的B设置；没有同期A。R1-6/7宽度0.03结果仅作跨批次参考，初始状态和运行次序未严格配对。每格3次，三张地图同源。','',
        '## 执行与参数','',
        f"- 完成：{len(rows)}/54 VALID；实际场景启动{plan['counters']['actual_scene_starts']}次、非场景调用{plan['counters']['non_scene_invocations']}次、补做{plan['counters']['replacement_scene_starts']}次。终局：到达{terminals['ARRIVED']}、碰撞{terminals['COLLISION_TERMINATION']}、跌倒{terminals['FALL_TERMINATION']}、仿真限时{terminals['DIAGNOSTIC_SIM_TIME_LIMIT']}、墙钟保护{terminals['WALL_CLOCK_GUARD_INCOMPLETE']}、安全中止{terminals['SYSTEM_SAFETY_ABORT']}、其他/UNKNOWN {len(rows)-sum(terminals.values())}。",
        f"- 安全完成TRUE/FALSE/UNKNOWN：{sum(r['safe_completion']=='TRUE' for r in rows)}/{sum(r['safe_completion']=='FALSE' for r in rows)}/{sum(r['safe_completion']=='UNKNOWN' for r in rows)}；到达后站立观察完成{sum(r['standing_observation_complete'] is True for r in rows)}次。",
        f"- 设置逐项匹配B组和E/X：E从−0.01至−0.06步长−0.01，X=E−0.04（最低X=−0.10）；全部实际H=0、保持关闭。目标(7.5,0)m、到达半径0.5m、仿真/墙钟上限20/120秒，到达后观察2仿真秒。",
        f"- 三轮交错排程，种子{plan['schedule']['seed']}。隐藏MuJoCo、RViz关闭，无截图录像；同上下文GLFW预检{(plan.get('preflight') or {}).get('status','UNKNOWN')}。六份临时overlay已在运行结束后清理。批次峰值约{max((x.get('batch_bytes',0) for x in plan.get('storage_checkpoints',[])),default=0)/1024**2:.1f}MiB，清理后运行记录约{raw_bytes/1024**3:.2f}GiB；分析和图表另存于报告目录。",
        '- 运行、模型、参数版本、冻结顺序和场景身份见本批`run_plan.json`与`run_schedule.csv`。旧R1-6/7版本记录链接见下方跨批次对照章节。','',
        '| 设置 | E | X | E−X | 实际H |','|---|---:|---:|---:|---:|']
    for s in SETTINGS:
        E,X,_=EXPECTED[s]
        md.append(f'| {s} | {E:.2f} | {X:.2f} | {E-X:.2f} | 0 |')
    md += ['', '## 18格地图×设置结果','',
           '| 地图 | 设置 | 有效/3 | 到达 | 安全完成T/F/U | 碰撞 | 跌倒 | 仿真限时 | 墙钟保护 | 安全中止 | 到达耗时中位数s（成功n） | 终点距离中位数m | 推进中位数m | 03—07中心轨迹进入数/3 | 碰撞对象 |',
           '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|']
    for c in cells:
        sub=[r for r in rows if r['map'].removesuffix('.xml')==c['map'] and r['setting_id']==c['setting_id']]
        obs=Counter(n for r in sub for n in r['collision_obstacles'].split(';') if n)
        obs_text='；'.join(f'{k}:{v}' for k,v in obs.items()) or ('UNKNOWN' if c['collisions'] else '无碰撞')
        at=[r['arrival_time_s'] for r in sub if r['arrival_time_s'] is not None]
        md.append(f"| {c['map']} | {c['setting_id']} | {c['actual_valid']}/3 | {c['arrived']}/3 | {c['safe_complete']}/{sum(r['safe_completion']=='FALSE' for r in sub)}/{c['safe_complete_unknown']} | {c['collisions']} | {c['falls']} | {c['sim_timeouts']} | {c['wall_timeouts']} | {c['safety_aborts']} | {num(median_known(at))} (n={len(at)}) | {num(c['end_distance_median_m'])} | {num(median_known([r['distance_progress_m'] for r in sub]))} | {sum(r['crossed_m02_gap_section'] is True for r in sub)}/3 | {obs_text} |")
    md += ['', '### 六设置汇总','',
           '| 设置 | 有效/9 | 到达 | 安全完成T/F/U | 碰撞 | 跌倒 | 仿真限时 | 墙钟保护 | 安全中止 | 到达耗时中位数s（成功n） | 终点距离中位数m | Recovery占比中位数 | 切换次数中位数 |',
           '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for t in totals:
        sub=[r for r in rows if r['setting_id']==t['setting_id']]
        md.append(f"| {t['setting_id']} | {t['actual_valid']}/9 | {t['arrived']} | {t['safe_complete']}/{sum(r['safe_completion']=='FALSE' for r in sub)}/{t['safe_complete_unknown']} | {t['collisions']} | {t['falls']} | {t['sim_timeouts']} | {t['wall_timeouts']} | {t['safety_aborts']} | {num(t['arrival_time_median_s'])} (n={t['arrival_time_n']}) | {num(t['end_distance_median_m'])} | {num(t['recovery_share_median'])} | {num(t['mode_transitions_median'],0)} |")
    md += ['',f"本批碰撞事件记录到的接触对象：{dict(contacts) if contacts else 'UNKNOWN'}。区域检查使用任务窗机器人中心位姿；中心经过不等同于整机轮廓净空。",'',
           '## 与宽度0.03同E设置并列（跨批次）','',
           'E=−0.02/−0.03/−0.04/−0.05/−0.06的宽度0.03结果来自R1-7或R1-6；E=−0.01没有同E旧档。原A_REF仅列作既有开发背景，不计入本批，不作同期对照。每个历史及新格各3次。下面的计数是描述性并列，不是配对推断或因果效应。',
           '[R1-6运行版本](../../../../logs/09_R1_switch_constraints/r1_v2_threshold_72/20261031_r1_v2_threshold72/run_plan.json)；[R1-7运行版本](../../../../logs/09_R1_switch_constraints/r1_m02_threshold_refine_18/20261031_r1_m02_threshold_refine18/run_plan.json)。',
           '| 地图 | E | 宽度0.03来源/设置 | 旧到达/3 | 旧碰撞 | 旧推进中位数m | 宽度0.04设置 | 新到达/3 | 新碰撞 | 新推进中位数m | 新到达耗时中位数s（成功n） |',
           '|---|---:|---|---:|---:|---:|---|---:|---:|---:|---:|']
    for p in pairs:
        md.append(f"| {p['map'].removesuffix('.xml')} | {p['E']:.2f} | {p['old_batch_source']} / {p['old_setting']} | {str(p['old_arrived'])+'/3' if p['old_arrived'] is not None else '无同E旧档'} | {p['old_collisions'] if p['old_collisions'] is not None else '—'} | {num(p['old_progress_median'])} | {p['new_setting']} | {p['new_arrived']}/3 | {p['new_collisions']} | {num(p['new_progress_median'])} | {num(p['new_arrival_time_median_s'])} (n={p['new_arrived']}) |")
    a_rows=[g for g in grouped if g['setting_id']=='A_REF']
    md += ['', 'A_REF旧72项背景（非同期、未计入本批）：'+'；'.join(f"{r['map'].removesuffix('.xml')}到达{r['arrived']}/3、碰撞{r['collisions']}" for r in a_rows)+ '。完整候选汇总表见`跨批次候选对照.csv`。', '',
           '## 走法与观察','',
           '用已有位姿记录统计中心位置是否进入x=5.30–5.60m、y=−1.67–0.42m的03—07近似截面；该统计只用于确认路径经过研究区域，不保证整机净空。碰撞对象来自原始任务窗安全记录。',
        '## 判断与解释边界','',
        '本批54次中6次到达、48次与障碍03碰撞，没有跌倒或超时。对宽度0.04设置本身，唯一相对集中的线索是窄档B_W04_E01_X05到达2/3；同设置在宽、中档均0/3。其他设置全批次到达为0/9或1/9。故目前没有跨布局的稳定任务改善；只把窄档E=−0.01/X=−0.05作为局部探索线索保留待审，不据此定为胜出参数。其余五种设置在本批没有显示足够改善证据，可暂不列优先候选。',
        '与宽度0.03的同E历史格相比，宽度0.04只有E=−0.02中档由0/3变1/3；E=−0.03宽档1/3降为0/3；E=−0.04宽档2/3降为0/3、另两档也未增加；E=−0.05窄档同为1/3；E=−0.06宽档由0/3变1/3但窄档由2/3变0/3。结合不同批次、初态和顺序，这些起伏不能归因于宽度变化。',
        '区域轨迹统计显示大多数运行的机器人中心进入03—07近似截面（各格2/3或3/3），但48次碰撞事件全记录为障碍03；中心位置通过截面不表示机体几何净空。布局仍被实际使用，但宽/中/窄本批都没有一致的避碰改善。',
        '',
           '对相同E，宽度0.04同时使X比宽度0.03低0.01；本批与旧批的初态、运行顺序和批次环境不同，因此差异只能作为开发线索。E=−0.01没有同E宽度0.03参考。每格3次、三档同源，不能估计稳健通过率或统计显著性。',
           '到达时间只对成功样本统计并注明n；不同成功子集不能证明整体更快。推进以每格全体有效运行的目标距离变化计算。Recovery占比和切换次数仅作行为说明，不单独作为效果判断。', '',
           '## 图表与数据','',
           '- ![同E宽度比较](同E宽度对照.png)',
           '- ![18格到达与安全完成](地图到达与安全完成.png)',
           '- ![终局类型](失败类型分布.png)',
           '- ![到达耗时与终点距离](到达耗时与终点距离.png)',
           '- ![代表轨迹](代表轨迹与障碍.png)',
           '- ![代表运行时间线](代表运行时间线.png)', '',
           '逐次、18格汇总、六设置汇总、36组批次×设置×地图宽度候选背景见`逐次运行明细.csv`、`地图设置汇总.csv`、`设置汇总.csv`、`跨批次候选对照.csv`及`同E宽度对照.csv`。轨迹只绘制导航任务窗，不包含到达后的站立和收尾段。',
           f"原始记录位于`logs/09_R1_switch_constraints/r1_m02_width04_54/{BATCH.name}/`；当前文件量约{raw_bytes/1024**3:.2f}GiB，旧批次与地图均未改。", '']
    (OUT/'分析报告.md').write_text('\n'.join(md),encoding='utf-8')


def main() -> None:
    plan,rows=collect()
    cells,totals=summarize(rows)
    write_csv(OUT/'逐次运行明细.csv',rows)
    write_csv(OUT/'地图设置汇总.csv',cells)
    write_csv(OUT/'设置汇总.csv',totals)
    plot_outcomes(cells,rows)
    plot_distance_time(rows)
    comparison,pairs=write_candidate_comparison(rows)
    cases=plot_timelines(rows)
    tracks=plot_representative_tracks(rows)
    (OUT/'代表案例.json').write_text(json.dumps(cases,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (OUT/'代表轨迹案例.json').write_text(json.dumps(tracks,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    write_width04_report(plan,rows,cells,totals,comparison,pairs,cases,tracks)
    print(f'Analyzed {len(rows)} valid runs; output={OUT}')


if __name__=='__main__':
    main()
