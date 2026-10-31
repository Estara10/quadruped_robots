#!/usr/bin/env python3
"""Task-specific R1-11 analysis using the production S1 run analyzer."""
from __future__ import annotations

import csv
import importlib.util
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median

os.environ.setdefault('MPLCONFIGDIR', '/tmp/r1-ab-two-candidates-120-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon

ROOT = Path(__file__).resolve().parents[5]
OUT = Path(__file__).resolve().parent
BATCH = ROOT / 'logs/09_R1_switch_constraints/ab_two_candidates_120/20261031_ab_two_candidates_120'
GOAL = (7.5, 0.0)
MAPS = ['r1_m02_space_wide_v2.xml', 'r1_m02_space_middle_v2.xml',
        'r1_m02_space_narrow_v2.xml', 'r1_d09_gap_narrow_v2.xml']
SCENE_TYPES = {'m02': MAPS[:3], 'd09': [MAPS[3]]}
SETTINGS = ['A_REF', 'B_W03_E04_X07', 'B_W05_E05_X10']
EXPECTED = {
    'A_REF': ('A', -0.05, -0.05, False),
    'B_W03_E04_X07': ('B', -0.04, -0.07, True),
    'B_W05_E05_X10': ('B', -0.05, -0.10, True),
}
COLORS = {'A_REF': '#4C72B0', 'B_W03_E04_X07': '#DD8452', 'B_W05_E05_X10': '#55A868'}
Z = 1.959963984540054
LEGAL_TERMINALS = {'ARRIVED', 'COLLISION_TERMINATION', 'FALL_TERMINATION',
                   'DIAGNOSTIC_SIM_TIME_LIMIT', 'WALL_CLOCK_GUARD_INCOMPLETE',
                   'SYSTEM_SAFETY_ABORT'}

sys.path.insert(0, str(ROOT / 'scripts'))
from s1_analyze_run import calculate  # noqa: E402
_helper_path = ROOT / 'docs/thesis_project/evidence/R1/M2_ABC_repeat_90/analyze_batch.py'
_spec = importlib.util.spec_from_file_location('r1_shared_analysis_helpers', _helper_path)
_helpers = importlib.util.module_from_spec(_spec)
assert _spec and _spec.loader
_spec.loader.exec_module(_helpers)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def find_one(root: Path, filename: str) -> Path:
    return _helpers.find_one(root, filename)


def task_frames(path: Path) -> list[dict]:
    return _helpers.task_frames(path)


def safe_label(value: object) -> str:
    if value is True:
        return 'TRUE'
    if value is False:
        return 'FALSE'
    if isinstance(value, str) and value.upper() == 'UNKNOWN':
        return 'UNKNOWN'
    return 'UNKNOWN'


def wilson(k: int, n: int) -> tuple[float | None, float | None]:
    if n <= 0:
        return None, None
    p = k / n
    den = 1 + Z * Z / n
    center = (p + Z * Z / (2 * n)) / den
    half = Z * math.sqrt(p * (1 - p) / n + Z * Z / (4 * n * n)) / den
    return center - half, center + half


def newcombe_difference(b_yes: int, b_n: int, a_yes: int, a_n: int) -> tuple[float | None, float | None, float | None]:
    """Independent two-proportion Newcombe-Wilson interval for B minus A."""
    if not b_n or not a_n:
        return None, None, None
    pb, pa = b_yes / b_n, a_yes / a_n
    blo, bhi = wilson(b_yes, b_n)
    alo, ahi = wilson(a_yes, a_n)
    diff = pb - pa
    low = diff - math.sqrt((pb - blo) ** 2 + (ahi - pa) ** 2)
    high = diff + math.sqrt((bhi - pb) ** 2 + (pa - alo) ** 2)
    return diff, max(-1.0, low), min(1.0, high)


def median_known(values: list[object]) -> float | None:
    finite = [float(v) for v in values if isinstance(v, (int, float)) and
              not isinstance(v, bool) and math.isfinite(float(v))]
    return median(finite) if finite else None


def num(value: float | None, digits: int = 2) -> str:
    return 'UNKNOWN' if value is None else f'{value:.{digits}f}'


def pct(value: float | None) -> str:
    return 'UNKNOWN' if value is None else f'{100*value:.1f}%'


def collect() -> tuple[dict, list[dict], list[dict]]:
    plan = load_json(BATCH / 'run_plan.json')
    if plan.get('schema') != 'r1-11-ab-two-candidates-120-v1' or len(plan['schedule']['items']) != 120:
        raise RuntimeError('frozen R1-11 plan must contain all 120 planned items')
    rows, sensitivity = [], []
    for item in plan['schedule']['items']:
        root = BATCH / 'runs' / item['plan_item_id'] / 'result'
        metrics_path = find_one(root, 'metrics.json')
        context_path = find_one(root, 'run_context.json')
        terminal_path = find_one(root, 'terminal.json')
        facts_path = find_one(root, 'process_facts.json')
        record_path = find_one(root, 'runtime_record.jsonl')
        published = load_json(metrics_path)
        context = load_json(context_path)
        terminal = load_json(terminal_path)
        facts = load_json(facts_path)
        raw_metrics = calculate(record_path, context_path, reverse_window_s=0.5,
                                decision_clock_override=False)
        unified = calculate(record_path, context_path, reverse_window_s=0.5,
                            decision_clock_override=True)
        frames = task_frames(record_path)
        group, entry, effective_exit, hysteresis = EXPECTED[item['setting_id']]
        evidence = ((context.get('runtime_source_validation') or {}).get('evidence') or {})
        actual_hysteresis = evidence.get('hysteresis')
        if not isinstance(actual_hysteresis, bool):
            actual_hysteresis = str(actual_hysteresis).lower() == 'true'
        actual_hold_enabled = evidence.get('hold')
        if not isinstance(actual_hold_enabled, bool):
            actual_hold_enabled = str(actual_hold_enabled).lower() == 'true'
        checks = {
            'runtime_group': evidence.get('group') == group,
            'E': evidence.get('entry_threshold') is not None and abs(float(evidence['entry_threshold']) - entry) < 1e-6,
            'effective_X': evidence.get('exit_threshold') is not None and abs(float(evidence['exit_threshold']) - effective_exit) < 1e-6,
            'H': evidence.get('recovery_min_hold_s') is not None and abs(float(evidence['recovery_min_hold_s'])) < 1e-6,
            'hysteresis': actual_hysteresis is hysteresis,
            'hold_disabled': actual_hold_enabled is False,
            'source_confirmed': (context.get('runtime_source_validation') or {}).get('status') == 'confirmed',
            'scene_identity': published.get('scene') == item['map'] and published.get('scene_id') == item['scene_id'],
            'record_valid': published.get('record_validity') == 'VALID',
            'cleanup': facts.get('shutdown_complete') is True and facts.get('closeout_complete') is True,
            'terminal_recognized': (terminal.get('run_terminal_result') in LEGAL_TERMINALS),
        }
        if not all(checks.values()):
            raise RuntimeError(f"contract/record check failed for {item['plan_item_id']}: {checks}")
        terminal_result = published.get('navigation_terminal_result') or terminal.get('run_terminal_result') or 'UNKNOWN'
        if terminal_result not in LEGAL_TERMINALS:
            raise RuntimeError(f"unrecognized navigation terminal for {item['plan_item_id']}: {terminal_result}")
        raw_safe = safe_label(raw_metrics.get('safety_task_completion_status'))
        unified_safe = safe_label(unified.get('safety_task_completion_status'))
        raw_elapsed = raw_metrics.get('elapsed_task_duration_s')
        unified_elapsed = unified.get('elapsed_task_duration_s')
        start_distance = unified.get('target_distance_start_m')
        final_distance = unified.get('target_distance_final_m')
        if not isinstance(start_distance, (int, float)):
            start_distance = published.get('target_distance_start_m')
        if not isinstance(final_distance, (int, float)):
            final_distance = published.get('target_distance_final_m')
        progress = (float(start_distance) - float(final_distance)
                    if isinstance(start_distance, (int, float)) and isinstance(final_distance, (int, float)) else None)
        collision_data = (unified.get('task_window_safety') or {})
        collision_events = collision_data.get('collision_events') or []
        collision_objects = sorted({e.get('obstacle_geom_name') for e in collision_events
                                    if isinstance(e, dict) and e.get('obstacle_geom_name')})
        safety_events = facts.get('external_safety_events') or []
        start_pose = frames[0]
        crossed_m02 = (any(5.30 <= f['x'] <= 5.60 and -1.67 <= f['y'] <= 0.42 for f in frames)
                       if item['map'].startswith('r1_m02_') else None)
        row = {
            'sequence': item['sequence'], 'round': item['round'], 'map': item['map'],
            'scene_id': item['scene_id'], 'scene_type': 'm02' if item['map'].startswith('r1_m02_') else 'd09',
            'setting_id': item['setting_id'], 'runtime_group': group,
            'plan_item_id': item['plan_item_id'], 'run_id': published.get('run_id'),
            'terminal_result': terminal_result, 'record_validity': published.get('record_validity'),
            'safe_completion': unified_safe, 'raw_recomputed_safe_completion': raw_safe,
            'published_safe_completion': safe_label(published.get('safety_task_completion_status')),
            'arrived': terminal_result == 'ARRIVED',
            'collision': terminal_result == 'COLLISION_TERMINATION',
            'fall': terminal_result == 'FALL_TERMINATION' or collision_data.get('fall') is True,
            'fall_status': ('TRUE' if terminal_result == 'FALL_TERMINATION' or collision_data.get('fall') is True
                            else 'FALSE' if collision_data.get('fall') is False else 'UNKNOWN'),
            'sim_timeout': terminal_result == 'DIAGNOSTIC_SIM_TIME_LIMIT',
            'wall_timeout': terminal_result == 'WALL_CLOCK_GUARD_INCOMPLETE',
            'safety_abort': terminal_result == 'SYSTEM_SAFETY_ABORT' or bool(safety_events),
            'other_terminal': terminal_result not in LEGAL_TERMINALS,
            'standing_observation_complete': any(e.get('event') == 'STANDING_HOLD_OBSERVATION_COMPLETE'
                                                 for e in (facts.get('cleanup_events') or [])),
            'arrival_time_raw_s': raw_elapsed if terminal_result == 'ARRIVED' else None,
            'arrival_time_unified_s': unified_elapsed if terminal_result == 'ARRIVED' else None,
            'task_duration_raw_s': raw_elapsed, 'task_duration_unified_s': unified_elapsed,
            'distance_start_m': start_distance, 'distance_final_m': final_distance,
            'distance_progress_m': progress,
            'recovery_share_unified': unified.get('recovery_share_valid_motion_time'),
            'mode_transition_count_unified': unified.get('mode_transitions'),
            'short_reverse_switch_count_unified': unified.get('short_reverse_switch_count'),
            'collision_obstacles': ';'.join(collision_objects) if collision_objects else
                ('UNKNOWN' if terminal_result == 'COLLISION_TERMINATION' else ''),
            'safety_coverage_unified': collision_data.get('coverage_complete_for_motion_interval'),
            'actual_initial_x_m': start_pose['x'], 'actual_initial_y_m': start_pose['y'],
            'actual_initial_yaw_rad': start_pose['yaw'],
            'initial_joint_state_status': (context.get('initial_state', {}).get('motion_start_measured_state', {}) or {}).get('status'),
            'crossed_m02_gap_section': crossed_m02,
            'decision_start_clock_raw_status': (raw_metrics.get('task_start_clock') or {}).get('status'),
            'decision_start_clock_unified_status': (unified.get('task_start_clock') or {}).get('status'),
            'decision_clock_reconstruction_applied': unified.get('timing_clock_override_applied') is True,
            'raw_invalid_clock_frames': raw_metrics.get('raw_invalid_clock_frames'),
            'timing_invalid_clock_frames_raw': raw_metrics.get('invalid_clock_frames'),
            'timing_invalid_clock_frames_unified': unified.get('invalid_clock_frames'),
            'actual_E': evidence.get('entry_threshold'), 'actual_X': evidence.get('exit_threshold'),
            'actual_H_s': evidence.get('recovery_min_hold_s'), 'runtime_contract_check': 'PASS',
        }
        rows.append(row)
        sensitivity.append({
            'sequence': item['sequence'], 'map': item['map'], 'setting_id': item['setting_id'],
            'run_id': published.get('run_id'), 'record_validity': published.get('record_validity'),
            'published_safety': row['published_safe_completion'], 'raw_recomputed_safety': raw_safe,
            'unified_decision_time_safety': unified_safe,
            'published_elapsed_s': published.get('elapsed_task_duration_s'),
            'raw_recomputed_elapsed_s': raw_elapsed, 'unified_elapsed_s': unified_elapsed,
            'timing_clock_override_applied': row['decision_clock_reconstruction_applied'],
            'decision_start_raw_status': row['decision_start_clock_raw_status'],
            'decision_start_unified_status': row['decision_start_clock_unified_status'],
            'raw_invalid_clock_frames': row['raw_invalid_clock_frames'],
            'invalid_clock_frames_raw_timing': row['timing_invalid_clock_frames_raw'],
            'invalid_clock_frames_unified_timing': row['timing_invalid_clock_frames_unified'],
            'safety_classification_changed': raw_safe != unified_safe,
            'elapsed_changed': (raw_elapsed != unified_elapsed),
        })
    return plan, rows, sensitivity


def known_rate(rows: list[dict], key: str) -> tuple[int, int, int]:
    values = [r[key] for r in rows]
    if key == 'safe_completion':
        yes = sum(v == 'TRUE' for v in values)
        no = sum(v == 'FALSE' for v in values)
        return yes, no, sum(v == 'UNKNOWN' for v in values)
    yes = sum(bool(v) for v in values)
    return yes, len(values) - yes, 0


def summarize(rows: list[dict]) -> tuple[list[dict], list[dict], list[dict], list[dict]]:
    cells, settings, strata, comparisons = [], [], [], []
    for map_name in MAPS:
        for setting in SETTINGS:
            subset = [r for r in rows if r['map'] == map_name and r['setting_id'] == setting]
            safe_t, safe_f, safe_u = known_rate(subset, 'safe_completion')
            arrived, _, _ = known_rate(subset, 'arrived')
            lo, hi = wilson(arrived, len(subset))
            collisions = sum(r['collision'] for r in subset)
            contacts = Counter(n for r in subset for n in r['collision_obstacles'].split(';') if n and n != 'UNKNOWN')
            cells.append({
                'map': map_name.removesuffix('.xml'), 'setting_id': setting,
                'planned': 10, 'valid': sum(r['record_validity'] == 'VALID' for r in subset),
                'safe_true': safe_t, 'safe_false': safe_f, 'safe_unknown': safe_u,
                'arrived': arrived, 'arrival_rate': arrived / len(subset) if subset else None,
                'arrival_wilson_low': lo, 'arrival_wilson_high': hi,
                'collisions': collisions, 'falls': sum(r['fall'] for r in subset),
                'fall_unknown': sum(r['fall_status'] == 'UNKNOWN' for r in subset),
                'sim_timeouts': sum(r['sim_timeout'] for r in subset),
                'wall_timeouts': sum(r['wall_timeout'] for r in subset),
                'safety_aborts': sum(r['safety_abort'] for r in subset),
                'other_terminals': sum(r['other_terminal'] for r in subset),
                'arrival_time_n': sum(r['arrival_time_unified_s'] is not None for r in subset),
                'arrival_time_raw_median_s': median_known([r['arrival_time_raw_s'] for r in subset]),
                'arrival_time_unified_median_s': median_known([r['arrival_time_unified_s'] for r in subset]),
                'end_distance_median': median_known([r['distance_final_m'] for r in subset]),
                'progress_median': median_known([r['distance_progress_m'] for r in subset]),
                'recovery_share_median': median_known([r['recovery_share_unified'] for r in subset]),
                'mode_transitions_median': median_known([r['mode_transition_count_unified'] for r in subset]),
                'crossed_m02_gap_center': sum(r['crossed_m02_gap_section'] is True for r in subset)
                    if map_name.startswith('r1_m02_') else None,
                'collision_objects': ';'.join(f'{k}:{v}' for k,v in sorted(contacts.items())) or
                    ('UNKNOWN' if collisions else 'none'),
            })
    for setting in SETTINGS:
        subset = [r for r in rows if r['setting_id'] == setting]
        safe_t, safe_f, safe_u = known_rate(subset, 'safe_completion')
        arrived, _, _ = known_rate(subset, 'arrived')
        lo, hi = wilson(arrived, len(subset))
        settings.append({
            'setting_id': setting, 'planned': 40,
            'valid': sum(r['record_validity'] == 'VALID' for r in subset),
            'safe_true': safe_t, 'safe_false': safe_f, 'safe_unknown': safe_u,
            'arrived': arrived, 'arrival_rate': arrived / len(subset) if subset else None,
            'arrival_wilson_low': lo, 'arrival_wilson_high': hi,
            'collisions': sum(r['collision'] for r in subset), 'falls': sum(r['fall'] for r in subset),
            'sim_timeouts': sum(r['sim_timeout'] for r in subset),
            'wall_timeouts': sum(r['wall_timeout'] for r in subset),
            'safety_aborts': sum(r['safety_abort'] for r in subset),
            'other_terminals': sum(r['other_terminal'] for r in subset),
            'arrival_time_n': sum(r['arrival_time_unified_s'] is not None for r in subset),
            'arrival_time_unified_median_s': median_known([r['arrival_time_unified_s'] for r in subset]),
            'distance_final_median': median_known([r['distance_final_m'] for r in subset]),
            'progress_median': median_known([r['distance_progress_m'] for r in subset]),
        })
    for stratum, map_names in [('m02_three_layouts', MAPS[:3]), ('d09_narrow_only', [MAPS[3]])]:
        for setting in SETTINGS:
            subset = [r for r in rows if r['map'] in map_names and r['setting_id'] == setting]
            safe_t, safe_f, safe_u = known_rate(subset, 'safe_completion')
            arrivals, _, _ = known_rate(subset, 'arrived')
            lo, hi = wilson(arrivals, len(subset))
            strata.append({
                'stratum': stratum, 'setting_id': setting, 'planned': len(subset),
                'valid': sum(r['record_validity'] == 'VALID' for r in subset),
                'safe_true': safe_t, 'safe_false': safe_f, 'safe_unknown': safe_u,
                'arrived': arrivals, 'arrival_rate': arrivals / len(subset) if subset else None,
                'arrival_wilson_low': lo, 'arrival_wilson_high': hi,
                'collisions': sum(r['collision'] for r in subset),
                'falls': sum(r['fall'] for r in subset),
                'sim_timeouts': sum(r['sim_timeout'] for r in subset),
                'wall_timeouts': sum(r['wall_timeout'] for r in subset),
                'safety_aborts': sum(r['safety_abort'] for r in subset),
                'other_terminals': sum(r['other_terminal'] for r in subset),
                'arrival_time_n': sum(r['arrival_time_unified_s'] is not None for r in subset),
                'arrival_time_median_s': median_known([r['arrival_time_unified_s'] for r in subset]),
                'progress_median': median_known([r['distance_progress_m'] for r in subset]),
                'end_distance_median': median_known([r['distance_final_m'] for r in subset]),
            })
        a = next(s for s in strata if s['stratum'] == stratum and s['setting_id'] == 'A_REF')
        for setting in SETTINGS[1:]:
            b = next(s for s in strata if s['stratum'] == stratum and s['setting_id'] == setting)
            for outcome, key in [('safe_completion', 'safe_true'), ('arrival', 'arrived'), ('collision', 'collisions')]:
                if outcome == 'safe_completion':
                    an, bn = a['safe_true'] + a['safe_false'], b['safe_true'] + b['safe_false']
                    ay, by = a['safe_true'], b['safe_true']
                else:
                    an, bn = a['planned'], b['planned']
                    ay, by = a[key], b[key]
                diff, low, high = newcombe_difference(by, bn, ay, an)
                comparisons.append({
                    'stratum': stratum, 'B_setting': setting, 'outcome': outcome,
                    'A_n_plan': a['planned'], 'A_n_known': an, 'A_events': ay,
                    'B_n_plan': b['planned'], 'B_n_known': bn, 'B_events': by,
                    'B_minus_A_rate_difference': diff,
                    'Newcombe_Wilson_95_low': low, 'Newcombe_Wilson_95_high': high,
                })
    return cells, settings, strata, comparisons


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def common_success(rows: list[dict]) -> list[dict]:
    out = []
    for map_name in MAPS:
        for setting in SETTINGS[1:]:
            diffs, pairs = [], 0
            for round_index in range(1, 11):
                a = next((r for r in rows if r['map'] == map_name and r['round'] == round_index and r['setting_id'] == 'A_REF'), None)
                b = next((r for r in rows if r['map'] == map_name and r['round'] == round_index and r['setting_id'] == setting), None)
                if a and b and a['arrived'] and b['arrived'] and isinstance(a['task_duration_unified_s'], (int,float)) and isinstance(b['task_duration_unified_s'], (int,float)):
                    pairs += 1
                    diffs.append(b['task_duration_unified_s'] - a['task_duration_unified_s'])
            out.append({'map': map_name.removesuffix('.xml'), 'B_setting': setting,
                        'same_map_round_both_arrived_n': pairs,
                        'median_B_minus_A_arrival_time_s': median(diffs) if diffs else None,
                        'note': 'same map and randomized round only; physical starting state not paired'})
    return out


def plot_outcomes(cells: list[dict]) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharey=True)
    for ax, map_name in zip(axes.flat, MAPS):
        sub = [next(c for c in cells if c['map'] == map_name.removesuffix('.xml') and c['setting_id'] == s) for s in SETTINGS]
        x = list(range(3))
        ax.bar([v-.18 for v in x], [c['arrived'] for c in sub], .35, color='#3D76A8', label='ARRIVED')
        ax.bar([v+.18 for v in x], [c['safe_true'] for c in sub], .35, color='#65A878', label='确认安全完成')
        for i,c in enumerate(sub):
            ax.text(i-.18, c['arrived']+.12, f"{c['arrived']}/10", ha='center', fontsize=8)
            ax.text(i+.18, c['safe_true']+.12, f"{c['safe_true']}/10", ha='center', fontsize=8)
        ax.set_title(map_name.removesuffix('.xml'))
        ax.set_xticks(x, ['A_REF\nE=X=−.05','B W03\n−.04/−.07','B W05\n−.05/−.10'], fontsize=8)
        ax.set_ylim(0, 11.5); ax.grid(axis='y', alpha=.2)
    axes[0,0].set_ylabel('次数（每格10次）'); axes[1,0].set_ylabel('次数（每格10次）')
    axes[0,0].legend(fontsize=8)
    fig.suptitle('四张开发图：A与两档B的到达和决策时钟重建后安全完成')
    fig.tight_layout(); fig.savefig(OUT/'地图到达与安全完成.png', dpi=180); plt.close(fig)

    fig, ax = plt.subplots(figsize=(14, 5.5))
    labels = [f"{c['map'].replace('r1_','')}\n{s.replace('B_W03_E04_X07','B03').replace('B_W05_E05_X10','B05').replace('A_REF','A')}"
              for c in cells for s in [c['setting_id']]]
    x = list(range(len(cells))); base = [0]*len(cells)
    for key,label,color in [('collisions','碰撞','#C44E52'),('falls','跌倒','#8172B2'),('sim_timeouts','仿真限时','#DD8452'),('wall_timeouts','墙钟保护','#937860'),('safety_aborts','安全中止','#64B5CD')]:
        vals=[c[key] for c in cells]
        ax.bar(x, vals, bottom=base, color=color, label=label, width=.78)
        for i,v in enumerate(vals):
            if v: ax.text(i,base[i]+v/2,str(v),ha='center',va='center',color='white',fontsize=7)
        base=[base[i]+vals[i] for i in range(len(base))]
    ax.set_xticks(x,labels,fontsize=7); ax.set_ylabel('次数 / 每格10次')
    ax.set_title('每格导航终局计数'); ax.grid(axis='y',alpha=.2); ax.legend(ncol=5,loc='upper center',bbox_to_anchor=(.5,-.16),frameon=False)
    fig.tight_layout(); fig.savefig(OUT/'终局分布.png',dpi=180,bbox_inches='tight'); plt.close(fig)


def plot_time_distance(rows: list[dict]) -> None:
    fig, axes = plt.subplots(4, 2, figsize=(13, 12), squeeze=False)
    for i, map_name in enumerate(MAPS):
        left,right=axes[i]
        for j,setting in enumerate(SETTINGS):
            sub=[r for r in rows if r['map']==map_name and r['setting_id']==setting]
            times=[r['arrival_time_unified_s'] for r in sub if isinstance(r['arrival_time_unified_s'],(int,float))]
            dists=[r['distance_final_m'] for r in sub if isinstance(r['distance_final_m'],(int,float))]
            left.scatter([j+(k-(len(times)-1)/2)*.012 for k in range(len(times))],times,color=COLORS[setting],s=22)
            right.scatter([j+(k-(len(dists)-1)/2)*.012 for k in range(len(dists))],dists,color=COLORS[setting],s=20)
        left.set_title(f'{map_name.removesuffix(".xml")}：到达耗时（统一时钟，成功子集）')
        right.set_title(f'{map_name.removesuffix(".xml")}：全部运行终点距离')
        left.set_ylabel('仿真秒'); right.set_ylabel('m'); right.axhline(.5,color='black',ls=':',lw=.8)
        for ax in (left,right):
            ax.set_xticks(range(3),['A_REF','B_W03','B_W05'],fontsize=8); ax.grid(alpha=.2)
    fig.suptitle('到达耗时、终点距离与推进见逐次表；成功子集耗时不代表整体效率')
    fig.tight_layout(rect=(0,0,1,.99)); fig.savefig(OUT/'耗时与终点距离.png',dpi=170); plt.close(fig)


def draw_obstacles(ax, map_name: str) -> None:
    tree=ET.parse(ROOT/'unitree_mujoco/unitree_robots/go2'/map_name)
    for geom in tree.findall('.//worldbody/geom'):
        name=geom.get('name') or ''
        if not name.startswith('ppt_obstacle_'): continue
        pos=[float(v) for v in geom.get('pos','0 0 0').split()]
        size=[float(v) for v in geom.get('size','0 0 0').split()]
        if geom.get('type')=='cylinder':
            ax.add_patch(Circle((pos[0],pos[1]),size[0],color='#555555',alpha=.56))
        elif geom.get('type')=='box':
            yaw=float((geom.get('euler') or '0 0 0').split()[2]); c,s=math.cos(yaw),math.sin(yaw)
            corners=[(-size[0],-size[1]),(size[0],-size[1]),(size[0],size[1]),(-size[0],size[1])]
            pts=[(pos[0]+u*c-v*s,pos[1]+u*s+v*c) for u,v in corners]
            ax.add_patch(Polygon(pts,closed=True,color='#555555',alpha=.56))
        ax.text(pos[0],pos[1],name.rsplit('_',1)[-1],ha='center',va='center',fontsize=7,color='white')


def plot_tracks(rows: list[dict]) -> list[dict]:
    chosen=[]
    for map_name in MAPS:
        for term in ('ARRIVED','COLLISION_TERMINATION'):
            sub=[r for r in rows if r['map']==map_name and r['terminal_result']==term]
            if sub:
                chosen.append(sorted(sub,key=lambda r:(r['round'],r['sequence']))[len(sub)//2])
    fig,axes=plt.subplots(4,2,figsize=(13,14),squeeze=False)
    cases=[]
    for i,map_name in enumerate(MAPS):
        for j,term in enumerate(('ARRIVED','COLLISION_TERMINATION')):
            ax=axes[i,j]; candidates=[r for r in chosen if r['map']==map_name and r['terminal_result']==term]
            if not candidates:
                ax.text(.5,.5,'本批无此终局',ha='center',va='center',transform=ax.transAxes)
                ax.set_title(f'{map_name} · {term}',loc='left',fontsize=8); continue
            row=candidates[0]; draw_obstacles(ax,map_name)
            root=BATCH/'runs'/row['plan_item_id']/'result'
            frames=task_frames(find_one(root,'runtime_record.jsonl'))
            duration=row['task_duration_unified_s']
            if isinstance(duration,(int,float)):
                frames=[f for f in frames if f['elapsed_s']<=float(duration)+1e-6]
            xy=[(f['x'],f['y']) for f in frames if isinstance(f.get('x'),(int,float)) and isinstance(f.get('y'),(int,float))]
            if xy:
                ax.plot([p[0] for p in xy],[p[1] for p in xy],color=COLORS[row['setting_id']],lw=1.5)
                ax.scatter([xy[0][0]],[xy[0][1]],marker='o',color='#2ca02c',s=22,label='起点')
                ax.scatter([xy[-1][0]],[xy[-1][1]],marker='x',color='#c44e52',s=30,label='终点')
            ax.scatter([GOAL[0]],[GOAL[1]],marker='*',color='#e6a700',s=50,label='目标')
            ax.set_xlim(-.5,8); ax.set_ylim(-3,3); ax.set_aspect('equal',adjustable='box'); ax.grid(alpha=.18)
            ax.set_title(f"{map_name} · {row['setting_id']} · {term} · 第{row['round']}轮",loc='left',fontsize=8)
            ax.set_xlabel('x / m'); ax.set_ylabel('y / m')
            cases.append({k:row[k] for k in ('plan_item_id','map','setting_id','run_id','terminal_result','task_duration_unified_s','distance_final_m','collision_obstacles')})
    axes[0,0].legend(fontsize=7,ncol=3,loc='upper left')
    fig.suptitle('各地图少量代表实际轨迹与障碍（只绘制导航任务窗）')
    fig.tight_layout(); fig.savefig(OUT/'代表轨迹与障碍.png',dpi=170); plt.close(fig)
    return cases


def write_report(plan: dict, rows: list[dict], sensitivity: list[dict], cells: list[dict],
                 totals: list[dict], strata: list[dict], comparisons: list[dict],
                 common: list[dict], cases: list[dict]) -> None:
    terminals=Counter(r['terminal_result'] for r in rows)
    valid=sum(r['record_validity']=='VALID' for r in rows)
    safe_counts=Counter(r['safe_completion'] for r in rows)
    raw_safe=Counter(r['raw_recomputed_safe_completion'] for r in rows)
    recon=sum(r['decision_clock_reconstruction_applied'] for r in rows)
    safety_changed=sum(r['safety_classification_changed'] for r in sensitivity)
    timing_changed=sum(r['elapsed_changed'] for r in sensitivity)
    collided=Counter(n for r in rows for n in r['collision_obstacles'].split(';') if n and n!='UNKNOWN')
    batch_size=sum(p.stat().st_size for p in BATCH.rglob('*') if p.is_file())
    strata_by={(x['stratum'],x['setting_id']):x for x in strata}
    md=[
        '# R1-11：A与两档B在四张开发图上的同期比较（120项）','',
        '本批比较四张已接入开发图上的A_REF、B_W03_E04_X07与B_W05_E05_X10。m02宽/中/窄三档同源，d09只有窄档；不视为四个独立场景，也不把120次混合数称作一般成功率。B与A共同E=-0.05的对照只在该同E设置下讨论滞回；E=-0.04/X=-0.07相对A同时改变进入与退出阈值。','',
        '## 执行情况与口径','',
        f"- 计划120项；完成{sum(i['status']=='COMPLETE' for i in plan['schedule']['items'])}项；有效记录{valid}；实际场景启动{plan['counters']['actual_scene_starts']}，非场景调用{plan['counters']['non_scene_invocations']}，补做{plan['counters']['replacement_scene_starts']}。终局：到达{terminals['ARRIVED']}、碰撞{terminals['COLLISION_TERMINATION']}、跌倒{terminals['FALL_TERMINATION']}、仿真限时{terminals['DIAGNOSTIC_SIM_TIME_LIMIT']}、墙钟保护{terminals['WALL_CLOCK_GUARD_INCOMPLETE']}、安全中止{terminals['SYSTEM_SAFETY_ABORT']}、其他/UNKNOWN {len(rows)-sum(terminals.values())}。",
        f"- 统一决策时钟口径安全完成T/F/U={safe_counts['TRUE']}/{safe_counts['FALSE']}/{safe_counts['UNKNOWN']}；默认/原始重算口径为T/F/U={raw_safe['TRUE']}/{raw_safe['FALSE']}/{raw_safe['UNKNOWN']}。重建应用{recon}项，安全分类变化{safety_changed}项，耗时字段变化{timing_changed}项。原始JSONL、context、terminal和metrics未修改；逐次原始/统一状态见`决策时钟敏感性.csv`。",
        f"- 运行实际参数由controller证据逐项匹配：A_REF E=X=-0.05、滞回关闭；B_W03_E04_X07 E/X=-0.04/-0.07；B_W05_E05_X10 E/X=-0.05/-0.10；全部H=0且保持关闭。目标(7.5,0)m、到达半径0.5m、20/120秒、到达后观察2仿真秒。",
        f"- 排程为10轮、每轮12个地图×设置组合各一次，种子{plan['schedule']['seed']}。每轮随机交错，不代表同轮A/B具有物理初态重放。隐藏MuJoCo、RViz关闭、无截图录像；同环境GLFW预检{(plan.get('preflight') or {}).get('status','UNKNOWN')}。批次当前目录约{batch_size/1024**2:.1f}MiB；三份临时overlay运行后清理。版本与每次关闭记录见`run_plan.json`、`run_schedule.csv`。", '',
        '## 12格地图×设置结果','',
        '| 地图 | 设置 | 有效/10 | 安全完成T/F/U | 到达/10（Wilson 95%） | 碰撞 | 跌倒/U | 仿真限时 | 墙钟保护 | 安全中止 | 成功耗时中位数（n） | 推进中位数m | 终点距离中位数m | 研究区进入/碰撞对象 |',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|',
    ]
    for c in cells:
        zone=(f"03—07中心近似区 {c['crossed_m02_gap_center']}/10" if c['crossed_m02_gap_center'] is not None else '轨迹图核对d09障碍01/02绕行')
        at=f"{num(c['arrival_time_unified_median_s'])}s (n={c['arrival_time_n']})"
        md.append(f"| {c['map']} | {c['setting_id']} | {c['valid']}/10 | {c['safe_true']}/{c['safe_false']}/{c['safe_unknown']} | {c['arrived']}/10 ({pct(c['arrival_rate'])}; {100*c['arrival_wilson_low']:.1f}–{100*c['arrival_wilson_high']:.1f}%) | {c['collisions']} | {c['falls']}/{c['fall_unknown']} | {c['sim_timeouts']} | {c['wall_timeouts']} | {c['safety_aborts']} | {at} | {num(c['progress_median'])} | {num(c['end_distance_median'])} | {zone}; {c['collision_objects']} |")
    md += ['', '## 同期率差：B减A','',
           '率差以百分点解释，区间使用独立二项比例的Newcombe-Wilson 95%区间；安全完成率的分母只含T/F，UNKNOWN单列，不作False处理。A与B运行次序交错，但没有严格物理初态配对。区间重叠或包含0不等于证明等效。',
           '| 场景层 | B设置 | 指标 | A分子/已知分母 | B分子/已知分母 | B−A率差百分点 | Newcombe-Wilson 95%区间百分点 |',
           '|---|---|---|---:|---:|---:|---:|']
    for c in comparisons:
        md.append(f"| {c['stratum']} | {c['B_setting']} | {c['outcome']} | {c['A_events']}/{c['A_n_known']} | {c['B_events']}/{c['B_n_known']} | {num(100*c['B_minus_A_rate_difference'],1)} | {num(100*c['Newcombe_Wilson_95_low'],1)} 至 {num(100*c['Newcombe_Wilson_95_high'],1)} |")
    md += ['', '## 分层汇总（不混合场景类）','',
           '| 场景层 | 设置 | 计划/有效 | 安全完成T/F/U | 到达 | 碰撞 | 跌倒 | 仿真限时 | 墙钟保护 | 安全中止 | 成功耗时中位数（n） | 推进中位数m | 终点距离中位数m |',
           '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for s in strata:
        md.append(f"| {s['stratum']} | {s['setting_id']} | {s['planned']}/{s['valid']} | {s['safe_true']}/{s['safe_false']}/{s['safe_unknown']} | {s['arrived']}/{s['planned']} | {s['collisions']} | {s['falls']} | {s['sim_timeouts']} | {s['wall_timeouts']} | {s['safety_aborts']} | {num(s['arrival_time_median_s'])}s (n={s['arrival_time_n']}) | {num(s['progress_median'])} | {num(s['end_distance_median'])} |")
    md += ['', 'm02每设置30项汇总的是同一模板三种变化；d09窄档每设置10项只代表这一张开发图。以上计数不能解释为对场景总体的独立抽样。','',
           '## 每设置40项记账汇总','',
           '此表用于完整报告本批40项/设置的终局记账，不作为四图混合成功率或一般场景性能估计。',
           '| 设置 | 有效/40 | 安全完成T/F/U | 到达 | 碰撞 | 跌倒 | 仿真限时 | 墙钟保护 | 安全中止 | 其他 | 到达耗时中位数（n） | 推进中位数m |',
           '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for t in totals:
        md.append(f"| {t['setting_id']} | {t['valid']}/40 | {t['safe_true']}/{t['safe_false']}/{t['safe_unknown']} | {t['arrived']}/40 | {t['collisions']} | {t['falls']} | {t['sim_timeouts']} | {t['wall_timeouts']} | {t['safety_aborts']} | {t['other_terminals']} | {num(t['arrival_time_unified_median_s'])}s (n={t['arrival_time_n']}) | {num(t['progress_median'])} |")
    md += ['', '## 成功耗时及共同成功案例','',
           '逐格到达耗时仅使用统一决策时钟后的成功样本并注明n；不同成功子集的耗时不解释为整体效率。下表列出同一地图、同一随机轮次中A与B都到达的运行；同轮不是相同物理初态，仅作有限的共同成功案例描述。',
           '| 地图 | B设置 | 同地图同轮共同到达n | B−A耗时中位差s |', '|---|---|---:|---:|']
    for c in common:
        md.append(f"| {c['map']} | {c['B_setting']} | {c['same_map_round_both_arrived_n']} | {num(c['median_B_minus_A_arrival_time_s'])} |")
    md += ['', '## 决策时钟重建敏感性','',
           '主要指标按生产分析器` s1_analyze_run.calculate(..., decision_clock_override=True)`重算。规则只在首个RL活跃周期发布时钟stale、但同运行/会话/周期的controller decision trace证明样本在决策时有效且一致时，恢复该周期的决策时钟。原始帧状态和原始记录不改；默认口径重算、统一口径重算以及发布metrics状态并列保存。',
           f"本批120项中统一重建实际应用{recon}项，安全分类变化{safety_changed}项，任务耗时字段变化{timing_changed}项。逐项变化见`决策时钟敏感性.csv`；如无有效同周期证明，保留UNKNOWN/原值，不向后移动任务起点。", '',
           '## 实际研究区域和轨迹','',
           f"所有导航碰撞记录对象汇总：{dict(sorted(collided.items())) if collided else 'UNKNOWN'}。m02研究区采用03—07附近位置的近似截面x=5.30–5.60m、y=−1.67–0.42m；中心进入不证明机器人整机通过净空。d09障碍01/02主要绕行侧需结合代表轨迹和逐次位姿判断，不将静态轨迹画成策略可达保证。少量成功/碰撞样例见下图和`代表案例.json`。", '',
           '- ![12格到达与安全完成](地图到达与安全完成.png)',
           '- ![12格终局分布](终局分布.png)',
           '- ![到达耗时与终点距离](耗时与终点距离.png)',
           '- ![代表轨迹](代表轨迹与障碍.png)', '',
           '| 代表运行 | 地图/设置 | 终局 | 统一耗时s | 终点距离m |', '|---|---|---|---:|---:|']
    for c in cases:
        md.append(f"| {c['plan_item_id']} | {c['map']} / {c['setting_id']} | {c['terminal_result']} | {num(c['task_duration_unified_s'])} | {num(c['distance_final_m'])} |")
    md += ['', '## 历史筛查背景（不进入本批分母）','',
           'R1-10中两档候选在m02三档的到达计数仅用于历史背景；该批无A/d09，不能代替本批同期A。详细旧批数据见[此前90项报告](../M02_CONFIRM_90/分析报告.md)。','',
           '## 结果解释与边界','',
           '阅读结果时以四张地图逐格和m02/d09分层为主；不根据120项混合平均值判断普遍成功率。E05/X10与A共享E，才可作为滞回整体作用的局部同期对照；E04/X07还改变E，因此是整体阈值配置对照。即使安全完成或碰撞率差方向有利，也需结合区间、推进、终点距离、耗时和局部地图退化，不以切换次数较少判优。',
           '三张m02图同源、d09只有一张，四图均为开发布局，不能代表独立场景分布。初始状态按每次实测记录，不宣称严格物理状态配对。足端接触/撞击、未覆盖或无法重建的安全指标保留UNKNOWN。合法导航终局均按原样计入，不以到达为目的重跑。','',
           f"原始运行目录：`{BATCH.relative_to(ROOT)}/`。逐次结果、12格、每设置40项、分层表、率差、共同成功耗时、决策时钟敏感性、终局和代表案例分别见`逐次运行明细.csv`、`地图设置汇总.csv`、`设置汇总.csv`、`场景分层汇总.csv`、`同期率差.csv`、`共同成功耗时.csv`、`决策时钟敏感性.csv`、`代表案例.json`。", '']
    (OUT/'分析报告.md').write_text('\n'.join(md), encoding='utf-8')


def main() -> None:
    plan, rows, sensitivity = collect()
    cells, totals, strata, comparisons = summarize(rows)
    common = common_success(rows)
    write_csv(OUT/'逐次运行明细.csv', rows)
    write_csv(OUT/'地图设置汇总.csv', cells)
    write_csv(OUT/'设置汇总.csv', totals)
    write_csv(OUT/'场景分层汇总.csv', strata)
    write_csv(OUT/'同期率差.csv', comparisons)
    write_csv(OUT/'共同成功耗时.csv', common)
    write_csv(OUT/'决策时钟敏感性.csv', sensitivity)
    plot_outcomes(cells)
    plot_time_distance(rows)
    cases = plot_tracks(rows)
    (OUT/'代表案例.json').write_text(json.dumps(cases, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    write_report(plan, rows, sensitivity, cells, totals, strata, comparisons, common, cases)
    print(f'Analyzed {len(rows)} production-valid runs; output={OUT}')


if __name__ == '__main__':
    main()
