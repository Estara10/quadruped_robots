"""One-stage read-only analysis; writes only new derived tables in this directory."""
import bisect
import collections
import csv
import json
import math
from pathlib import Path
import re
import statistics
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[5]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'scripts'))
import generate_s6_08_scenes as geom

batches = {
    'M3_240': ROOT/'logs/08_formal_experiment/M3_ABCD_240/m3-abcd240-20261023',
    'AC_280': ROOT/'logs/09_R1_switch_constraints/ac_hold_280/20261101_r1_ac_hold_280',
    'AC_CONFIRM_120': ROOT/'logs/09_R1_switch_constraints/ac_hold_confirm_120/20261101_r1_ac_hold_confirm_120',
    'CH15_120': ROOT/'logs/09_R1_switch_constraints/ch15_ac_120/20261102_r1_ch15_ac_120',
    'X05_40': ROOT/'logs/09_R1_switch_constraints/ch15_x05_hold_40/20261102_r1_ch15_x05_hold_40',
}
run_rows = []
episodes = []
collisions = []
geometry = {}
timeseries = {}
for batch, folder in batches.items():
    files = sorted((folder/'runs').rglob('runtime_record.jsonl'))
    print(batch, 'records', len(files), flush=True)
    for number, record in enumerate(files, 1):
        directory = record.parent
        context = json.loads((directory/'run_context.json').read_text())
        metrics = json.loads((directory/'metrics.json').read_text())
        terminal = json.loads((directory/'terminal.json').read_text())
        settings = context['settings']
        config = context['runtime_config_observed']
        run_id = context['run_id']
        scene = settings['scene_filename']
        group = config['group']
        H = config['recovery_min_hold_s']
        if terminal['run_terminal_frame_ref'] is None or metrics['rl_task_metric_boundary']['start_cycle'] is None:
            run_rows.append(dict(batch=batch,run_id=run_id,map=scene,group=group,H_s=H,
                                 terminal=terminal['run_terminal_result'],analysis_status='UNKNOWN_TASK_BOUNDARY',
                                 record_path=str(record.relative_to(ROOT))))
            print('Boundary unknown, retaining without fabricated event analysis:', batch, run_id, flush=True)
            continue
        end_cycle = terminal['run_terminal_frame_ref']['rl_step']
        start_cycle = metrics['rl_task_metric_boundary']['start_cycle']
        start = metrics.get('task_start_sim_time_s')
        terminal_time = terminal['run_terminal_frame_ref']['sim_time_s']
        # This is a control-task-window analysis, not a revision of formal safety boundaries.
        end = terminal_time
        if scene not in geometry:
            xml = ROOT/'unitree_mujoco/unitree_robots/go2'/scene
            obs = []
            for n in ET.parse(xml).getroot().findall('.//worldbody/geom'):
                if not n.get('name', '').startswith('ppt_obstacle_'):
                    continue
                x, y, z = map(float, n.get('pos').split())
                size = list(map(float, n.get('size').split()))
                o = dict(name=n.get('name'), shape=n.get('type'), x=x, y=y)
                if o['shape'] == 'box':
                    o.update(half_x=size[0], half_y=size[1], yaw_rad=float(n.get('euler', '0 0 0').split()[-1]))
                else:
                    o.update(radius=size[0], yaw_rad=0.)
                obs.append(o)
            geometry[scene] = obs
        frames = []
        history = {}
        for line in record.open():
            row = json.loads(line)
            if row.get('kind') != 'frame':
                continue
            f = row['payload']
            for event in (f.get('collision_snapshot') or {}).get('collision_history', []):
                history[(event['start_physics_step'], event['obstacle_geom_name'])] = event
            cycle = f.get('policy_cycle_id', f.get('rl_step'))
            if f.get('session_id') != context['session_id'] or cycle is None or not start_cycle <= cycle <= end_cycle:
                continue
            frames.append({k:f.get(k) for k in ['policy_cycle_id','rl_step','sim_time_s','sim_clock_valid','sim_clock_segment_id','monotonic_ns','risk_evaluation_ns','world_pose','lin_vel','mode_before','mode_after','ra_value','entry_threshold','exit_threshold','hold_status','hold_elapsed_sim_s','hold_enter_sim_time_s','recovery_min_hold_s','transition_reason']})
        frames = sorted({f['rl_step']:f for f in frames}.values(), key=lambda f:f['rl_step'])
        if not frames:
            raise RuntimeError('No task frames: '+str(record))
        valid = [f for f in frames if f['sim_clock_valid'] and isinstance(f['sim_time_s'], (int,float))]
        ts = [f['sim_time_s'] for f in valid]
        assert all(b >= a for a,b in zip(ts, ts[1:])), record
        logs = (directory/'ros_launch.log').read_text(errors='replace')
        twist_logs = []
        for line in logs.splitlines():
            if '[RA-REC] 进入恢复' not in line:
                continue
            m = re.search(r'sim_time=([-+\deE.]+).*恢复速度=\[([^]]+)\]', line)
            if m:
                twist_logs.append((float(m[1]), [float(x) for x in m[2].split(',')]))
        run_eps = []
        active = None
        for index, f in enumerate(frames):
            if f['mode_after'] == 1 and active is None:
                active = dict(start_index=index, enter=f, frames=[], ended=False)
            if active is not None:
                if f['mode_after'] == 1:
                    active['frames'].append(f)
                else:
                    active.update(end_index=index, exit=f, ended=True)
                    run_eps.append(active)
                    active = None
        if active is not None:
            active.update(end_index=len(frames)-1, exit=frames[-1])
            run_eps.append(active)
        run_rec_time = run_blocked = run_posthigh = 0.
        run_block_cycles = bad_intervals = 0
        for epi, ep in enumerate(run_eps, 1):
            en, ex = ep['enter'], ep['exit']
            entry_known = en['mode_before'] == 0 and en['sim_clock_valid']
            et = en['sim_time_s'] if en['sim_clock_valid'] else None
            xt = ex['sim_time_s'] if ex['sim_clock_valid'] else None
            matched = [x for x in twist_logs if et is not None and abs(x[0]-et) < 1e-5]
            twist = matched[0][1] if len(matched) == 1 else None
            blocked = posthigh = rec_time = 0.
            blocked_cycles = ep_bad = near_expiry = 0
            for i in range(ep['start_index'], ep['end_index']+1):
                f = frames[i]
                is_blocked = bool(H > 0 and f['sim_clock_valid'] and f['mode_after']==1 and f['mode_before']==1 and f['hold_status']==2 and f['ra_value']<f['exit_threshold'])
                if is_blocked:
                    blocked_cycles += 1
                    if abs(H-f['hold_elapsed_sim_s']) < 1e-6:
                        near_expiry += 1
                if i==ep['end_index']:
                    continue
                nxt = frames[i+1]
                if not (f['sim_clock_valid'] and nxt['sim_clock_valid'] and nxt['rl_step'] == f['rl_step']+1 and f['sim_clock_segment_id'] == nxt['sim_clock_segment_id'] and nxt['sim_time_s'] > f['sim_time_s']):
                    ep_bad += 1
                    continue
                dt = nxt['sim_time_s'] - f['sim_time_s']
                rec_time += dt
                if is_blocked:
                    # Actual sampled control state persists until the next decision;
                    # do not replace it with ideal real-number expiry H.
                    blocked += dt
                if H > 0 and f['hold_status'] == 3 and f['ra_value'] >= f['exit_threshold']:
                    posthigh += dt
            poses = [f for f in ep['frames'] if f['sim_clock_valid'] and isinstance(f['world_pose'],list)]
            if ep['ended'] and isinstance(ex['world_pose'],list) and ex['sim_clock_valid']:
                poses.append(ex)
            progress = yaw_change = body_motion = path = speed_change = ''
            if len(poses) >= 2:
                a,b = poses[0]['world_pose'], poses[-1]['world_pose']
                progress = math.hypot(7.5-a[0],a[1])-math.hypot(7.5-b[0],b[1])
                yaw_change = math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
                body_motion = path = 0.
                for fa,fb in zip(poses,poses[1:]):
                    pa,pb = fa['world_pose'],fb['world_pose']
                    dx,dy = pb[0]-pa[0],pb[1]-pa[1]
                    body_motion += dx*math.cos(pa[2])+dy*math.sin(pa[2])
                    path += math.hypot(dx,dy)
                speed_change = math.hypot(*poses[-1]['lin_vel'][:2])-math.hypot(*poses[0]['lin_vel'][:2])
            e = dict(batch=batch,run_id=run_id,map=scene,group=group,H_s=H,episode=epi,
                     entry_cycle=en['rl_step'],entry_sim_s=et,entry_known=entry_known,
                     end_cycle=ex['rl_step'],end_sim_s=xt,censored=not ep['ended'],
                     observed_duration_s=xt-et if xt is not None and et is not None else '',
                     recovery_observed_s=rec_time,blocked_cycles=blocked_cycles,near_nominal_expiry_block_cycles=near_expiry,blocked_estimated_s=blocked,
                     post_H_high_RA_estimated_s=posthigh,invalid_or_missing_intervals=ep_bad,
                     twist_log_matches=len(matched),cached_twist_vx=twist[0] if twist else '',
                     cached_twist_vy=twist[1] if twist else '',cached_twist_wz=twist[2] if twist else '',
                     heading_change_rad=yaw_change,body_forward_net_m=body_motion,path_m=path,
                     goal_net_progress_m=progress,speed_xy_change_mps=speed_change,
                     terminal=terminal['run_terminal_result'])
            episodes.append(e)
            ep['summary'] = e
            run_rec_time += rec_time
            run_blocked += blocked
            run_posthigh += posthigh
            run_block_cycles += blocked_cycles
            bad_intervals += ep_bad
        edges_exit = [f for f in valid if f['mode_before']==1 and f['mode_after']==0]
        edges_enter = [f for f in valid if f['mode_before']==0 and f['mode_after']==1]
        events = [e for e in history.values() if isinstance(start,(int,float)) and start-1e-9 <= e['start_sim_time'] <= end+1e-9]
        if terminal['run_terminal_result']=='COLLISION_TERMINATION' and not events:
            # Preserve a missing authoritative task-event as unknown, never borrow closeout events.
            collisions.append(dict(batch=batch,run_id=run_id,map=scene,group=group,H_s=H,classification='UNKNOWN_EVENT_TIME',terminal=terminal['run_terminal_result']))
        for event in sorted(events,key=lambda e:e['start_sim_time']):
            t=event['start_sim_time'];idx=bisect.bisect_right(ts,t+1e-9)-1
            if idx<0:
                collisions.append(dict(batch=batch,run_id=run_id,map=scene,group=group,H_s=H,classification='UNKNOWN_NO_PRE_FRAME',terminal=terminal['run_terminal_result']))
                continue
            f=valid[idx];after=valid[idx+1] if idx+1<len(valid) else None
            exits=[x for x in edges_exit if x['sim_time_s']<=t+1e-9]
            enters=[x for x in edges_enter if x['sim_time_s']<=t+1e-9]
            last_exit=exits[-1] if exits else None
            last_enter=enters[-1] if enters else None
            reentered=bool(last_exit and last_enter and last_enter['sim_time_s']>last_exit['sim_time_s'])
            ambiguous=bool(after and after['mode_after']!=f['mode_after'] and f['sim_time_s']<t-1e-6 and after['sim_time_s']>t+1e-6)
            mode='RECOVERY' if f['mode_after']==1 else 'AGILE'
            classification=('UNKNOWN_MODE_BRACKET' if ambiguous else 'RECOVERY_INTERNAL' if mode=='RECOVERY' else 'POST_EXIT_AGILE' if last_exit else 'BEFORE_FIRST_RECOVERY')
            ep=next((e for e in run_eps if last_enter and e['enter']['rl_step']==last_enter['rl_step']),None)
            es=ep['summary'] if ep else None
            obstacle=next(o for o in geometry[scene] if o['name']==event['obstacle_geom_name'])
            window=[x for x in valid if max(t-1.,last_enter['sim_time_s'] if mode=='RECOVERY' and last_enter else t-1.)<=x['sim_time_s']<=t+1e-9]
            progress=clear_start=clear_end=heading=body_net=''
            if len(window)>=2:
                pa,pb=window[0]['world_pose'],window[-1]['world_pose']
                progress=math.hypot(7.5-pa[0],pa[1])-math.hypot(7.5-pb[0],pb[1])
                clear_start=geom.point_shape_distance(tuple(pa[:2]),obstacle)
                clear_end=geom.point_shape_distance(tuple(pb[:2]),obstacle)
                heading=math.atan2(math.sin(pb[2]-pa[2]),math.cos(pb[2]-pa[2]))
                body_net=0.
                for fa,fb in zip(window,window[1:]):
                    pa1,pb1=fa['world_pose'],fb['world_pose'];dx,dy=pb1[0]-pa1[0],pb1[1]-pa1[1]
                    body_net+=dx*math.cos(pa1[2])+dy*math.sin(pa1[2])
            c=dict(batch=batch,run_id=run_id,map=scene,group=group,H_s=H,classification=classification,
                   collision_sim_s=t,collision_step=event['start_physics_step'],obstacle=event['obstacle_geom_name'],robot_body=event['robot_body_name'],
                   last_mode=mode,last_RA=f['ra_value'],exit_threshold=f['exit_threshold'],
                   pre_frame_gap_s=t-f['sim_time_s'],post_frame_gap_s=after['sim_time_s']-t if after else '',
                   last_exit_sim_s=last_exit['sim_time_s'] if last_exit else '',exit_to_collision_s=t-last_exit['sim_time_s'] if last_exit else '',
                   reentered_after_last_exit=reentered,last_entry_sim_s=last_enter['sim_time_s'] if last_enter else '',
                   recovery_age_s=t-last_enter['sim_time_s'] if mode=='RECOVERY' and last_enter else '',
                   hold_status=f['hold_status'],hold_elapsed_s=f['hold_elapsed_sim_s'],
                   RA_exit_clear=f['ra_value']<f['exit_threshold'],
                   cached_twist_vx=es['cached_twist_vx'] if es and mode=='RECOVERY' else '',
                   cached_twist_vy=es['cached_twist_vy'] if es and mode=='RECOVERY' else '',
                   cached_twist_wz=es['cached_twist_wz'] if es and mode=='RECOVERY' else '',
                   observed_window_s=window[-1]['sim_time_s']-window[0]['sim_time_s'] if len(window)>1 else '',
                   goal_net_progress_m=progress,obstacle_centerpoint_clearance_start_m=clear_start,
                   obstacle_centerpoint_clearance_end_m=clear_end,heading_change_rad=heading,body_forward_net_m=body_net,
                   terminal=terminal['run_terminal_result'])
            collisions.append(c)
        run_rows.append(dict(batch=batch,run_id=run_id,map=scene,group=group,H_s=H,terminal=terminal['run_terminal_result'],analysis_status='OBSERVED_TASK_WINDOW',
                             frames=len(frames),valid_clock_frames=len(valid),episodes=len(run_eps),
                             censored_episodes=sum(not e['ended'] for e in run_eps),
                             episodes_with_blocking=sum(e['summary']['blocked_cycles']>0 for e in run_eps),
                             blocked_cycles=run_block_cycles,blocked_estimated_s=run_blocked,
                             post_H_high_RA_estimated_s=run_posthigh,recovery_observed_s=run_rec_time,
                             invalid_or_missing_intervals=bad_intervals,
                             gd_log_count=logs.count('[TWIST-GD]'),entry_log_count=len(twist_logs),
                             unbound_episode_twist=sum(e['summary']['twist_log_matches']!=1 for e in run_eps),
                             policy_interval_ms_median=metrics.get('policy_interval_ms_median'),
                             policy_interval_ms_p95=metrics.get('policy_interval_ms_p95'),
                             record_path=str(record.relative_to(ROOT))))
        # Compact data for at most six representative plots will be selected later.
        timeseries[run_id]=dict(frames=valid,episodes=[e['summary'] for e in run_eps],scene=scene,batch=batch,group=group,H_s=H)
        if number%40==0:
            print(batch,number,'done',flush=True)

groups=collections.defaultdict(list)
for r in run_rows:
    if r['analysis_status']=='OBSERVED_TASK_WINDOW':
        groups[(r['batch'],r['map'],r['group'],r['H_s'])].append(r)
holds=[]
for key,rs in sorted(groups.items()):
    batch,scene,group,H=key
    es=[e for e in episodes if (e['batch'],e['map'],e['group'],e['H_s'])==key]
    durations=[e['observed_duration_s'] for e in es if isinstance(e['observed_duration_s'],(float,int)) and not e['censored']]
    r=dict(batch=batch,map=scene,group=group,H_s=H,runs=len(rs),episodes=len(es),
           complete_episodes=sum(not e['censored'] for e in es),censored_episodes=sum(e['censored'] for e in es),
           episodes_with_blocking=sum(e['blocked_cycles']>0 for e in es),blocked_cycles=sum(e['blocked_cycles'] for e in es),
           blocked_estimated_s=sum(e['blocked_estimated_s'] for e in es),near_nominal_expiry_block_cycles=sum(e['near_nominal_expiry_block_cycles'] for e in es),post_H_high_RA_estimated_s=sum(e['post_H_high_RA_estimated_s'] for e in es),
           recovery_observed_s=sum(e['recovery_observed_s'] for e in es),invalid_or_missing_intervals=sum(e['invalid_or_missing_intervals'] for e in es),
           complete_residence_median_s=statistics.median(durations) if durations else '',
           unbound_episode_twist=sum(e['twist_log_matches']!=1 for e in es),
           episodes_observed_ge_0_10s=sum(isinstance(e['observed_duration_s'],(float,int)) and e['observed_duration_s']>=.1 for e in es))
    holds.append(r)
for name,rows in [('run_inventory.csv',run_rows),('hold_effect_by_scene.csv',holds),('recovery_episode_observations.csv',episodes),('collision_progress_cases.csv',collisions)]:
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with (OUT/name).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');w.writeheader();w.writerows(rows)
(OUT/'geometry_obstacles.json').write_text(json.dumps(geometry,ensure_ascii=False,indent=2)+'\n')

# Choose a small set by observable event categories, not by whether a future mode would win.
selected=[]
for label,predicate in [
    ('hold_active_collision',lambda r:r.get('last_mode')=='RECOVERY' and r.get('hold_status')==2 and r.get('RA_exit_clear') is True),
    ('expired_RA_high_collision',lambda r:r.get('last_mode')=='RECOVERY' and r.get('hold_status')==3 and r.get('RA_exit_clear') is False),
    ('long_cached_heading_change_collision',lambda r:r.get('last_mode')=='RECOVERY' and isinstance(r.get('recovery_age_s'),(int,float)) and r['recovery_age_s']>=.3 and isinstance(r.get('heading_change_rad'),(int,float)) and abs(r['heading_change_rad'])>=.2),
    ('recent_exit_collision',lambda r:r.get('classification')=='POST_EXIT_AGILE' and isinstance(r.get('exit_to_collision_s'),(int,float)) and r['exit_to_collision_s']<=.1),
]:
    matches=[r for r in collisions if predicate(r) and r['run_id'] not in selected]
    if matches:
        pick=sorted(matches,key=lambda r:(r['batch']!='M3_240',r['run_id']))[0]
        selected.append(pick['run_id']);timeseries[pick['run_id']]['selection']=label
longs=[e for e in episodes if e['terminal']=='DIAGNOSTIC_SIM_TIME_LIMIT' and isinstance(e['observed_duration_s'],(int,float)) and e['observed_duration_s']>=.5 and e['run_id'] not in selected]
if longs:
    pick=sorted(longs,key=lambda e:(e['goal_net_progress_m'] if isinstance(e['goal_net_progress_m'],(int,float)) else float('inf'),e['run_id']))[0]
    selected.append(pick['run_id']);timeseries[pick['run_id']]['selection']='long_recovery_low_progress_timeout'
goods=[e for e in episodes if e['terminal']=='ARRIVED' and isinstance(e['observed_duration_s'],(int,float)) and e['observed_duration_s']>=.5 and isinstance(e['goal_net_progress_m'],(int,float)) and e['goal_net_progress_m']>0 and e['run_id'] not in selected]
if goods:
    pick=sorted(goods,key=lambda e:(-e['observed_duration_s'],e['run_id']))[0]
    selected.append(pick['run_id']);timeseries[pick['run_id']]['selection']='long_cached_recovery_success_counterexample'
(OUT/'representative_series.json').write_text(json.dumps({k:timeseries[k] for k in selected},ensure_ascii=False)+'\n')
print('DONE',len(run_rows),'runs',len(episodes),'episodes',len(collisions),'collision rows',len(selected),'selected cases',flush=True)
