#!/usr/bin/env python3
"""Dispatch the frozen 120-item R1-11 A/B development comparison."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import random
import shutil
import signal
import subprocess
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / 'scripts'))
import manual_ab_compare as manual  # noqa: E402
from abs_scene import resolve_scene  # noqa: E402
from s1_run_diagnostic import overlay_configuration_matches  # noqa: E402

EVID = Path(__file__).resolve().parent
BATCH_ID = '20261031_ab_two_candidates_120'
BATCH = ROOT / 'logs/09_R1_switch_constraints/ab_two_candidates_120' / BATCH_ID
PLAN = BATCH / 'run_plan.json'
MAPSET = ROOT / 'docs/thesis_project/designs/R1/generated_maps_v2'
LAYOUTS = MAPSET / 'layouts.json'
MAP_DIR = ROOT / 'unitree_mujoco/unitree_robots/go2'
SOURCE = ROOT / 'quadruped_ros2_control_humble/install/go2_description'
MAPS = ('r1_m02_space_wide_v2.xml', 'r1_m02_space_middle_v2.xml',
        'r1_m02_space_narrow_v2.xml', 'r1_d09_gap_narrow_v2.xml')
SETTINGS = {
    'A_REF': {'group': 'A', 'E': -0.05, 'X': -0.05, 'hysteresis': False},
    'B_W03_E04_X07': {'group': 'B', 'E': -0.04, 'X': -0.07, 'hysteresis': True},
    'B_W05_E05_X10': {'group': 'B', 'E': -0.05, 'X': -0.10, 'hysteresis': True},
}
SETTING_IDS = tuple(SETTINGS)
SEED = 2026103106
H_COMPATIBILITY_FALLBACK = 0.20
X_COMPATIBILITY_FALLBACK_DELTA = 0.03
GOAL = (7.5, 0.0)
ARRIVAL_RADIUS = 0.5
SIM_LIMIT = 20.0
WALL_CAP = 120.0
STANDING_SIM = 2.0
STANDING_WALL = 30.0
MAX_BYTES = 2 * 1024**3
MIN_FREE_BYTES = 15 * 1024**3
CLASSIFICATION = 'R1-11 four development layouts x A_REF and two B candidates x ten reps; development comparison, not independent validation'
MAX_SCENE_STARTS = 123
LEGAL_TERMINALS = {
    'ARRIVED', 'COLLISION_TERMINATION', 'FALL_TERMINATION',
    'DIAGNOSTIC_SIM_TIME_LIMIT', 'WALL_CLOCK_GUARD_INCOMPLETE',
    'SYSTEM_SAFETY_ABORT',
}


def now() -> str:
    return datetime.now().astimezone().isoformat(timespec='seconds')


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def atomic_json(path: Path, payload: dict) -> None:
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    os.replace(temp, path)


def json_or_empty(path: Path) -> dict:
    try:
        with path.open(encoding='utf-8') as f:
            value = json.load(f)
        return value if isinstance(value, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def scene_metadata() -> list[dict]:
    layouts = json_or_empty(LAYOUTS)
    if layouts.get('mapset_version') != 'r1-layout-proposals/2':
        raise RuntimeError('R1 v2 mapset version mismatch')
    by_name = {r['filename']: r for r in layouts.get('scenes', [])}
    result = []
    for filename in MAPS:
        entry = by_name.get(filename)
        spec = resolve_scene(filename)
        binding = (entry or {}).get('production_scene_binding') or {}
        if (not entry or entry.get('scene_id') != spec.scene_id or
                binding.get('scene_id') != spec.scene_id or
                entry.get('split') != 'development_layout_viewing'):
            raise RuntimeError(f'R1 v2 map identity or split mismatch: {filename}')
        xml = MAP_DIR / filename
        if not xml.is_file() or digest(xml) != spec.root_sha256:
            raise RuntimeError(f'R1 v2 XML path or catalog binding mismatch: {xml}')
        result.append({
            'filename': filename, 'scene_id': spec.scene_id,
            'xml_path': str(xml.resolve()), 'mapset_version': layouts['mapset_version'],
            'difficulty_label': entry['tier'], 'source_set': entry['template'],
            'source_filename': entry['source_v1'], 'layout_revision': entry.get('layout_revision'),
            'xml_sha256': spec.root_sha256, 'closure_sha256': spec.closure_sha256,
            'model_fingerprint': spec.model_fingerprint, 'obstacle_count': spec.obstacle_count,
        })
    return result


def versions_once() -> dict:
    git = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'],
                         check=False, text=True, stdout=subprocess.PIPE,
                         stderr=subprocess.DEVNULL, timeout=10).stdout.strip() or None
    install_config = SOURCE / 'share/go2_description/config'
    binary_paths = {
        'mujoco_binary': ROOT / 'unitree_mujoco/simulate/build2/unitree_mujoco',
        'controller_library': ROOT / 'quadruped_ros2_control_humble/install/rl_quadruped_controller/lib/rl_quadruped_controller/librl_quadruped_controller.so',
        'controller_source': ROOT / 'quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp',
        'agile_model': install_config / 'abs/policy.pt',
        'ra_model': install_config / 'abs/ra_value.pt',
        'recovery_model': install_config / 'rec/policy.pt',
        'agile_config': install_config / 'abs/config.yaml',
        'controller_config': install_config / 'robot_control.yaml',
    }
    return {'git_commit': git, 'runtime_mujoco_version': '3.3.3', 'recorded_at': now(),
            'artifacts': {name: {'path': str(p.resolve()), 'sha256': digest(p)} if p.is_file()
                          else {'path': str(p), 'missing': True}
                          for name, p in binary_paths.items()},
            'mapset_path': str(LAYOUTS.resolve()), 'mapset_version': 'r1-layout-proposals/2'}


def schedule(maps: list[dict]) -> list[dict]:
    rng = random.Random(SEED)
    metadata = {r['filename']: r for r in maps}
    items = []
    for round_index in range(1, 11):
        round_items = [(filename, setting_id) for filename in MAPS for setting_id in SETTING_IDS]
        rng.shuffle(round_items)
        for filename, setting_id in round_items:
            m = metadata[filename]
            seq = len(items) + 1
            items.append({
                'sequence': seq, 'plan_item_id': f'{seq:03d}_r{round_index}_{m["scene_id"]}_{setting_id}',
                'round': round_index, 'map': filename, 'scene_id': m['scene_id'],
                'setting_id': setting_id, 'group': SETTINGS[setting_id]['group'],
                'status': 'NOT_STARTED', 'attempts': [],
            })
    if len(items) != 120 or any(sum(i['round'] == r for i in items) != 12 for r in range(1, 11)):
        raise RuntimeError('schedule generation failed 120-item / 12-per-round check')
    if any(sum(i['map'] == m and i['setting_id'] == s for i in items) != 10
           for m in MAPS for s in SETTING_IDS):
        raise RuntimeError('schedule generation failed 10-per-map/setting check')
    if any(sum(i['setting_id'] == setting for i in items) != 40 for setting in SETTING_IDS):
        raise RuntimeError('schedule generation failed 40-per-setting check')
    return items


def create_plan() -> dict:
    maps = scene_metadata()
    payload = {
        'schema': 'r1-11-ab-two-candidates-120-v1', 'batch_id': BATCH_ID,
        'classification': CLASSIFICATION, 'created_at': now(),
        'purpose': '四张开发图上同期比较A_REF与两种保留B配置；仅开发比较，不作独立验证',
        'mapset': {'path': str(LAYOUTS.resolve()), 'version': 'r1-layout-proposals/2',
                   'layouts_sha256': digest(LAYOUTS),
                   'maps': maps},
        'versions': versions_once(),
        'contract': {
            'settings': {key: {**value, 'effective_X': value['X'], 'H_s': 0.0,
                               'hold_enabled': False}
                         for key, value in SETTINGS.items()},
            'compatibility_arguments': {
            'note': 'Existing CLI requires X<E and H>0 even for A/B; these fallback values are not consumed by A/B. Effective runtime values are checked from controller evidence.',
                'single_threshold_cli_X': 'E - 0.03 (A overlay omits exit_threshold; effective X=E)',
                'hold_s': H_COMPATIBILITY_FALLBACK,
            },
            'goal_world_m': list(GOAL), 'arrival_radius_m': ARRIVAL_RADIUS,
            'sim_limit_s': SIM_LIMIT, 'wall_cap_s': WALL_CAP,
            'standing_observation_sim_s': STANDING_SIM,
            'standing_observation_wall_cap_s': STANDING_WALL,
            'actual_initial_state_replay': False,
        },
        'rendering': {'backend': 'GLFW', 'mujoco_window': 'hidden', 'rviz': 'disabled',
                      'framebuffer_capture': False, 'video': False,
                      'note': 'Hidden GLFW uses the configured DISPLAY/OpenGL context.'},
        'schedule': {'seed': SEED, 'method': '10 rounds; each round shuffles all 12 map/setting pairs once',
                     'items': schedule(maps)},
        'budget': {'max_batch_bytes': MAX_BYTES, 'minimum_free_bytes': MIN_FREE_BYTES,
                   'maximum_actual_scene_starts': MAX_SCENE_STARTS, 'maximum_replacements_per_item': 1,
                   'maximum_total_replacements': 3},
        'counters': {'actual_scene_starts': 0, 'non_scene_invocations': 0,
                     'replacement_scene_starts': 0},
        'status': 'PREPARED',
    }
    atomic_json(PLAN, payload)
    write_schedule_csv(payload)
    write_progress(payload)
    return payload


def write_schedule_csv(plan: dict) -> None:
    with (BATCH / 'run_schedule.csv').open('w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(['sequence', 'round', 'map', 'scene_id', 'setting_id', 'runtime_group', 'status', 'run_id', 'terminal_result'])
        for item in plan['schedule']['items']:
            attempt = item['attempts'][-1] if item['attempts'] else {}
            w.writerow([item['sequence'], item['round'], item['map'], item['scene_id'], item['setting_id'], item['group'],
                        item['status'], attempt.get('run_id'), attempt.get('terminal_result')])


def write_progress(plan: dict) -> None:
    items = plan['schedule']['items']
    payload = {
        'updated_at': now(), 'status': plan.get('status'), 'planned': len(items),
        'finished': sum(i.get('status') in {'COMPLETE', 'RESULT_RECORDED'} for i in items),
        'not_started': sum(i.get('status') == 'NOT_STARTED' for i in items),
        'actual_scene_starts': plan['counters']['actual_scene_starts'],
        'non_scene_invocations': plan['counters']['non_scene_invocations'],
        'next_sequence': next((i['sequence'] for i in items if i.get('status') == 'NOT_STARTED'), None),
    }
    atomic_json(BATCH / 'progress.json', payload)
    write_schedule_csv(plan)


def prepare_overlays() -> dict[str, str]:
    overlays = {}
    for setting_id, setting in SETTINGS.items():
        group = setting['group']
        entry = setting['E']
        exit_value = setting['X'] if group == 'B' else entry - X_COMPATIBILITY_FALLBACK_DELTA
        target = BATCH / 'overlays' / setting_id
        if target.exists():
            raise RuntimeError(f'refusing to reuse pre-existing overlay: {target}')
        manual.prepare_overlay(SOURCE, target, group, entry, exit_value, H_COMPATIBILITY_FALLBACK)
        cfg = target / 'share/go2_description/config/abs/config.yaml'
        manual.set_overlay_goal(cfg, *GOAL)
        txt = cfg.read_text(encoding='utf-8')
        if not overlay_configuration_matches(txt, group, entry, exit_value,
                                             H_COMPATIBILITY_FALLBACK, *GOAL):
            raise RuntimeError(f'overlay config did not match requested {setting_id} contract')
        manual.verify_overlay_detached(SOURCE, target)
        overlays[setting_id] = str(target.resolve())
    return overlays


def run_preflight(args: argparse.Namespace) -> dict:
    env = os.environ.copy()
    env.update({'DISPLAY': args.display, 'XAUTHORITY': args.xauthority,
                'LIBGL_DRIVERS_PATH': '/usr/lib/x86_64-linux-gnu/dri',
                'MUJOCO_GL': 'glfw', 'PYTHONNOUSERSITE': '1', 'ABS_MUJOCO_NO_WINDOW': '1'})
    p = subprocess.run(['/usr/bin/python3', str(ROOT / 'scripts/m2_glfw_preflight.py'),
                        str(BATCH / 'glfw_preflight.json')], cwd=ROOT, env=env,
                       text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
    facts = json_or_empty(BATCH / 'glfw_preflight.json')
    facts['returncode'] = p.returncode
    facts['output_tail'] = p.stdout[-2000:]
    facts['same_execution_context_as_runs'] = (
        p.returncode == 0 and facts.get('status') == 'PASS' and
        (facts.get('environment') or {}).get('DISPLAY') == args.display and
        (facts.get('environment') or {}).get('XAUTHORITY') == args.xauthority)
    atomic_json(BATCH / 'glfw_preflight.json', facts)
    return facts


def config_for(item: dict) -> dict:
    setting = SETTINGS[item['setting_id']]
    return {'group': setting['group'], 'entry_threshold_E': setting['E'],
            'effective_exit_threshold': setting['X'],
            'hysteresis': setting['hysteresis'], 'hold_enabled': False,
            'hold_H_sim_s': 0.0}


def command_for(item: dict, overlay: str, result_root: Path, args: argparse.Namespace) -> list[str]:
    setting = SETTINGS[item['setting_id']]
    cli_exit = setting['X'] if setting['group'] == 'B' else setting['E'] - X_COMPATIBILITY_FALLBACK_DELTA
    return ['/usr/bin/python3', '-u', str(ROOT / 'scripts/manual_ab_compare.py'),
            '--groups', setting['group'], '--repeats', '1', '--scene', item['map'],
            '--goal-x', str(GOAL[0]), '--goal-y', str(GOAL[1]),
            '--entry-threshold', str(setting['E']), '--exit-threshold', str(cli_exit),
            '--hold-s', str(H_COMPATIBILITY_FALLBACK),
            '--sim-limit-s', str(SIM_LIMIT), '--wall-cap-s', str(WALL_CAP),
            '--standing-observation-sim-s', str(STANDING_SIM),
            '--result-root', str(result_root), '--prepared-overlay', overlay,
            '--display', args.display, '--xauthority', args.xauthority,
            '--plot-python', args.plot_python, '--classification', CLASSIFICATION,
            '--no-rviz', '--no-window', '--disable-framebuffer-capture']


def find_run_files(result_root: Path) -> dict[str, Path]:
    result = {}
    for key, filename in (('context', 'run_context.json'), ('metrics', 'metrics.json'),
                          ('terminal', 'terminal.json'), ('process_facts', 'process_facts.json'),
                          ('runtime_record', 'runtime_record.jsonl')):
        paths = list(result_root.rglob(filename))
        if len(paths) == 1:
            result[key] = paths[0]
    return result


def validate_result(item: dict, paths: dict[str, Path]) -> tuple[list[str], dict]:
    reasons = []
    if len(paths) < 5:
        return ['missing one or more context/metrics/terminal/facts/runtime records'], {}
    context = json_or_empty(paths['context'])
    metrics = json_or_empty(paths['metrics'])
    terminal = json_or_empty(paths['terminal'])
    facts = json_or_empty(paths['process_facts'])
    source = ((context.get('runtime_source_validation') or {}).get('evidence') or {})
    expected = config_for(item)
    for name, value in [('group', expected['group']), ('entry_threshold', expected['entry_threshold_E']),
                        ('exit_threshold', expected['effective_exit_threshold'])]:
        actual = source.get(name)
        if isinstance(value, float):
            try:
                if abs(float(actual) - value) > 1e-5: reasons.append(f'{name}_mismatch:{actual}')
            except (TypeError, ValueError): reasons.append(f'{name}_missing:{actual}')
        elif actual != value:
            reasons.append(f'{name}_mismatch:{actual}')
    if source.get('status') == 'not_confirmed' or (context.get('runtime_source_validation') or {}).get('status') != 'confirmed':
        reasons.append('runtime_source_validation_not_confirmed')
    expected_hold = expected['hold_H_sim_s']
    try:
        if abs(float(source.get('recovery_min_hold_s')) - expected_hold) > 1e-5:
            reasons.append(f'hold_mismatch:{source.get("recovery_min_hold_s")}')
    except (TypeError, ValueError): reasons.append('hold_missing')
    expected_bool = {'hysteresis': expected['hysteresis'], 'hold': expected['hold_enabled']}
    for field, value in expected_bool.items():
        actual = source.get(field)
        if isinstance(value, bool):
            actual_bool = str(actual).lower() == 'true' if not isinstance(actual, bool) else actual
            if actual_bool != value: reasons.append(f'{field}_mismatch:{actual}')
    if source.get('goal_world_m') != list(GOAL) and source.get('runtime_path_goal_world_m') != list(GOAL):
        reasons.append(f'goal_mismatch:{source.get("goal_world_m")}')
    if metrics.get('scene') != item['map'] or metrics.get('scene_id') != item['scene_id']:
        reasons.append(f'map_identity_mismatch:{metrics.get("scene")}/{metrics.get("scene_id")}')
    if metrics.get('record_validity') != 'VALID': reasons.append(f'record_validity:{metrics.get("record_validity")}')
    term = terminal.get('run_terminal_result')
    if term not in LEGAL_TERMINALS: reasons.append(f'unrecognized_terminal:{term}')
    if facts.get('shutdown_complete') is not True or facts.get('closeout_complete') is not True:
        reasons.append('shutdown_or_closeout_unconfirmed')
    if terminal.get('shutdown_complete') is not True: reasons.append('terminal_shutdown_unconfirmed')
    lifecycle = context.get('lifecycle') or {}
    scene_started = (lifecycle.get('mujoco_process_started_monotonic_ns') is not None or
                     facts.get('mujoco_process_created') is True or bool(facts.get('mujoco_pid')))
    out = {'run_id': metrics.get('run_id'), 'terminal_result': term,
           'navigation_terminal_result': metrics.get('navigation_terminal_result'),
           'record_validity': metrics.get('record_validity'),
           'source_evidence': source, 'scene_started_confirmed': scene_started,
           'shutdown_complete': facts.get('shutdown_complete'),
           'closeout_complete': facts.get('closeout_complete'),
           'metrics_path': str(paths['metrics']), 'runtime_record_path': str(paths['runtime_record'])}
    return reasons, out


def storage_snapshot() -> dict:
    batch_bytes = sum(p.stat().st_size for p in BATCH.rglob('*') if p.is_file())
    free = shutil.disk_usage(BATCH).free
    return {'at': now(), 'batch_bytes': batch_bytes, 'filesystem_free_bytes': free}


def stop_batch(plan: dict, status: str, reason: str) -> None:
    plan['status'] = status
    plan['stop_reason'] = reason
    plan['updated_at'] = now()
    atomic_json(PLAN, plan)
    write_progress(plan)


def run_item(plan: dict, item: dict, overlays: dict[str, str], args: argparse.Namespace) -> bool:
    storage = storage_snapshot()
    plan.setdefault('storage_checkpoints', []).append(storage)
    if plan['counters']['actual_scene_starts'] >= MAX_SCENE_STARTS:
        stop_batch(plan, 'STOPPED_SCENE_START_LIMIT', f'{MAX_SCENE_STARTS} actual scene starts reached before dispatch')
        return False
    if storage['batch_bytes'] >= MAX_BYTES or storage['filesystem_free_bytes'] < MIN_FREE_BYTES:
        stop_batch(plan, 'STOPPED_STORAGE', '2GiB batch budget or 15GiB free-space threshold reached before dispatch')
        return False
    result_root = BATCH / 'runs' / item['plan_item_id'] / 'result'
    if result_root.exists():
        stop_batch(plan, 'STOPPED_EXISTING_RESULT', f'refusing to overwrite {result_root}')
        return False
    result_root.parent.mkdir(parents=True, exist_ok=True)
    cmd = command_for(item, overlays[item['setting_id']], result_root, args)
    item['status'] = 'RUNNING'
    attempt = {'attempt_number': 1, 'started_at': now(), 'command': cmd,
               'result_root': str(result_root), 'status': 'RUNNING'}
    item['attempts'].append(attempt)
    plan['status'] = 'RUNNING'
    plan['current_item'] = item['sequence']
    plan['updated_at'] = now()
    atomic_json(PLAN, plan)
    console_path = result_root.parent / 'console.log'
    env = os.environ.copy()
    env.update({'DISPLAY': args.display, 'XAUTHORITY': args.xauthority,
                'LIBGL_DRIVERS_PATH': '/usr/lib/x86_64-linux-gnu/dri',
                'MUJOCO_GL': 'glfw', 'PYTHONNOUSERSITE': '1', 'ABS_MUJOCO_NO_WINDOW': '1'})
    print(f"\n[{item['sequence']}/120] round {item['round']} {item['map']} {item['setting_id']} starting", flush=True)
    with console_path.open('w', encoding='utf-8') as log:
        proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, bufsize=1,
                                start_new_session=True)
        try:
            assert proc.stdout is not None
            for line in proc.stdout:
                print(line, end='', flush=True)
                log.write(line); log.flush()
            code = proc.wait()
        except KeyboardInterrupt:
            attempt['interrupt_requested_at'] = now()
            plan['status'] = 'STOPPING_CURRENT_ITEM'
            atomic_json(PLAN, plan)
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                code = proc.wait(timeout=150)
            except subprocess.TimeoutExpired:
                try: os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError: pass
                try: code = proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    try: os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError: pass
                    code = proc.wait(timeout=5)
            raise
    paths = find_run_files(result_root)
    reasons, facts = validate_result(item, paths)
    attempt.update({'finished_at': now(), 'child_exit_code': code, **facts,
                    'failure_reasons': reasons,
                    'actual_scene_start': facts.get('scene_started_confirmed', False)})
    if facts.get('scene_started_confirmed'):
        plan['counters']['actual_scene_starts'] += 1
    else:
        plan['counters']['non_scene_invocations'] += 1
    if reasons:
        item['status'] = 'STOPPED_REVIEW'
        item['failure_reasons'] = reasons
        attempt['status'] = 'STOPPED_REVIEW'
        stop_batch(plan, 'STOPPED_REVIEW', f"sequence {item['sequence']}: {'; '.join(reasons)}")
        print(f"[{item['sequence']}/120] STOPPED_REVIEW: {'; '.join(reasons)}", flush=True)
        return False
    item['status'] = 'COMPLETE'
    item['run_id'] = facts['run_id']
    item['terminal_result'] = facts['terminal_result']
    attempt['status'] = 'COMPLETE'
    plan['updated_at'] = now()
    plan['current_item'] = None
    atomic_json(PLAN, plan)
    write_progress(plan)
    storage = storage_snapshot()
    plan.setdefault('storage_checkpoints', []).append(storage)
    atomic_json(PLAN, plan)
    if storage['batch_bytes'] > MAX_BYTES or storage['filesystem_free_bytes'] < MIN_FREE_BYTES:
        stop_batch(plan, 'STOPPED_STORAGE_AFTER_ITEM', 'storage limit crossed; no next item dispatched')
        return False
    print(f"[{item['sequence']}/120] {facts['terminal_result']} / record VALID / cleanup confirmed; "
          f"scene starts={plan['counters']['actual_scene_starts']}", flush=True)
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--display', default=os.environ.get('DISPLAY', ':0'))
    parser.add_argument('--xauthority', default=os.environ.get('XAUTHORITY') or
                        f'/run/user/{os.getuid()}/gdm/Xauthority')
    parser.add_argument('--plot-python', default='/home/lidio/anaconda3/bin/python3')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    BATCH.mkdir(parents=True, exist_ok=True)
    if not PLAN.exists():
        plan = create_plan()
    else:
        plan = json_or_empty(PLAN)
        if plan.get('schema') != 'r1-11-ab-two-candidates-120-v1':
            raise RuntimeError('existing batch plan schema mismatch')
        if plan.get('schedule', {}).get('seed') != SEED:
            raise RuntimeError('frozen schedule seed mismatch')
    overlay_paths = plan.get('overlays')
    if not overlay_paths:
        overlay_paths = prepare_overlays()
        plan['overlays'] = overlay_paths
        plan['status'] = 'PREPARED'
        atomic_json(PLAN, plan)
    if args.prepare_only:
        print(f"Prepared frozen plan: {PLAN}")
        return 0
    if not Path(args.plot_python).is_file():
        stop_batch(plan, 'STOPPED_PREFLIGHT', f'plot Python missing: {args.plot_python}')
        return 2
    preflight = run_preflight(args)
    plan['preflight'] = preflight
    if not preflight.get('same_execution_context_as_runs'):
        stop_batch(plan, 'STOPPED_PREFLIGHT', 'same-context hidden GLFW/OpenGL check failed; no scene started')
        return 2
    plan['preflight_passed_at'] = now()
    plan['status'] = 'RUNNING'
    atomic_json(PLAN, plan)
    for item in plan['schedule']['items']:
        if item.get('status') == 'COMPLETE':
            continue
        if item.get('status') != 'NOT_STARTED':
            stop_batch(plan, 'STOPPED_NONRESUMABLE_ITEM',
                       f"sequence {item['sequence']} state {item.get('status')} requires review")
            return 2
        if not run_item(plan, item, overlay_paths, args):
            return 2
    plan['status'] = 'COMPLETE'
    plan['completed_at'] = now()
    plan['current_item'] = None
    atomic_json(PLAN, plan)
    write_progress(plan)
    print('All 120 plan items completed.', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
