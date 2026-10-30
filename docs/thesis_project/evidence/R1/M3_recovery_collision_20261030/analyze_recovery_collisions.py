#!/usr/bin/env python3
"""One-purpose extension of the R1-1 M3 event-table analysis."""
import csv, json, math, re
from pathlib import Path
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[5]
EVID = ROOT / 'docs/thesis_project/evidence/R1'
OLD = EVID / 'M3_event_progress_20261030'
OUT = EVID / 'M3_recovery_collision_20261030'
BATCH = ROOT / 'logs/08_formal_experiment/M3_ABCD_240/m3-abcd240-20261023'
PLANS = BATCH / 'analysis/formal_summary/primary_plan_items.csv'
OUT.mkdir(parents=True, exist_ok=True)

old_rows = list(csv.DictReader((OLD / 'A组失败明细.csv').open(encoding='utf-8-sig')))
selected = {r['run_id']: {'group':'A','map':r['地图']} for r in old_rows
            if r['分类'] == 'Recovery内部碰撞'}
for r in csv.DictReader(PLANS.open(encoding='utf-8-sig')):
    if r['group'] != 'A' and r['collision'] == 'true':
        selected[r['plan_item_id']] = {'group':r['group'],'map':r['map']}

def get_record(run):
    paths = list((BATCH/'runs'/run).glob('attempt_*/result/*/runtime_record.jsonl'))
    if not paths: return None
    frames=[]
    for line in paths[0].open():
        x=json.loads(line)
        if x.get('kind')=='frame' and isinstance(x.get('payload'),dict): frames.append(x['payload'])
    summaries=list((BATCH/'runs'/run).glob('attempt_*/result/analysis/run_results.json'))
    task_start=None
    if summaries:
        with summaries[0].open() as sf: s=json.load(sf)
        s=s[0] if isinstance(s,list) else s
        task_start=(s.get('task_start_clock') or {}).get('sim_time_s')
    return frames,task_start

def obst_xy(map_name, obstacle):
    p=ROOT/'unitree_mujoco/unitree_robots/go2'/f'{map_name}.xml'
    if not p.exists(): return None
    try: tree=ET.parse(p)
    except Exception: return None
    for g in tree.iter('geom'):
        if g.get('name') == obstacle:
            vals=[float(v) for v in g.get('pos','0 0 0').split()]
            return vals[:2]
    return None

def mode(f): return f.get('policy_state_name') or ('RECOVERY' if f.get('mode_after')==1 else 'AGILE')

out=[]; case_data={}
for run, meta in sorted(selected.items(), key=lambda z:int(z[0].split('-')[1])):
    rec=get_record(run)
    if rec is None: continue
    frames,start=rec
    events=[]
    for f in frames:
        c=f.get('collision_snapshot') or {}
        for e in c.get('collision_history') or []:
            if e.get('obstacle_geom_name') and e.get('start_sim_time') is not None:
                events.append((float(e['start_sim_time']),e))
    if events:
        t,e=min(events,key=lambda z:z[0]); obstacle=e.get('obstacle_geom_name'); body=e.get('robot_body_name','')
    else:
        continue
    before=[f for f in frames if f.get('sim_clock_valid') and f.get('sim_time_s') is not None and f['sim_time_s']<=t+1e-7]
    if not before: continue
    hit=before[-1]; hit_t=float(hit['sim_time_s']); mode_hit=mode(hit)
    if mode_hit != 'RECOVERY': continue
    w=[f for f in before if f['sim_time_s']>=t-1.02]
    poses=[f.get('world_pose') for f in w]
    poses=[(f,p) for f,p in zip(w,poses) if isinstance(p,list) and len(p)>=3]
    dxbody=[]; path=[]
    for (fa,a),(fb,b) in zip(poses,poses[1:]):
        dt=float(fb['sim_time_s'])-float(fa['sim_time_s'])
        if dt<=0: continue
        yaw=(float(a[2])+float(b[2]))/2
        dx=float(b[0])-float(a[0]); dy=float(b[1])-float(a[1])
        longitudinal=dx*math.cos(yaw)+dy*math.sin(yaw)
        dxbody.append(longitudinal); path.append((dx,dy,dt))
    net=sum(dxbody); dist=sum(math.hypot(dx,dy) for dx,dy,dt in path)
    if not dxbody: direction='无法判断'
    elif net < -0.05: direction='可确认后退'
    elif net > 0.05: direction='可确认前进'
    else: direction='记录不足，无法判断'
    event_rel=(t-start) if start is not None else None
    entry=None; previous_mode=None
    for f in before:
        current_mode=mode(f)
        if current_mode=='RECOVERY' and previous_mode!='RECOVERY': entry=float(f['sim_time_s'])
        previous_mode=current_mode
    dwell=(t-entry) if entry is not None else None
    age=t-hit_t
    q=hit.get('world_pose') or [None,None,None]
    op=obst_xy(meta['map'],obstacle)
    rear='无法确认'
    if op and len(q)>=3 and q[0] is not None:
        dx=op[0]-float(q[0]);dy=op[1]-float(q[1]); yaw=float(q[2])
        localx=dx*math.cos(yaw)+dy*math.sin(yaw)
        rear='是' if localx < -0.10 else '否'
    ra=hit.get('ra_value')
    # Goal-distance proxy from the body-frame goal command, if present.
    def goal_dist(f):
        c=f.get('command')
        return math.hypot(float(c[0]),float(c[1])) if isinstance(c,list) and len(c)>=2 else None
    ds0=goal_dist(w[0]) if w else None; ds1=goal_dist(hit)
    trend='无法判断'
    delta_goal=None
    if ds0 is not None and ds1 is not None:
        dd=ds1-ds0; delta_goal=dd; trend='缩短' if dd < -0.08 else ('增加' if dd > 0.08 else '基本不变')
    # Report ray coverage values as body-front rays only; Recovery observation excludes rays.
    rays=hit.get('ray2d')
    rfront='有' if isinstance(rays,list) and len(rays)==11 else '无'
    cached_vx=None
    ros_logs=list((BATCH/'runs'/run).glob('attempt_*/result/*/ros_launch.log'))
    if ros_logs and start is not None:
        latest=[]
        for line in ros_logs[0].open(errors='ignore'):
            if '[RA-REC] 进入恢复' not in line: continue
            m=re.search(r'sim_time=([0-9.]+).*?恢复速度=\[([^]]+)\]',line)
            if m and float(m.group(1))<=t+0.03:
                vals=[float(v) for v in m.group(2).split(',')]
                if vals: latest.append((float(m.group(1)),vals[0]))
        if latest: cached_vx=latest[-1][1]
    row={'run_id':run,'地图':meta['map'],'组别':meta['group'],'碰撞时间_任务秒':round(event_rel,3) if event_rel is not None else '',
         '碰撞对象':obstacle,'机器人碰撞部位':body,'碰撞前模式':mode_hit,'碰撞前RA':ra,
         '本次Recovery持续至碰撞_s':round(dwell,3) if dwell is not None else '',
         '最近策略帧保持已过_s':hit.get('hold_elapsed_sim_s'), 'Recovery入口缓存twist_vx_mps':cached_vx,
         '最近策略帧至碰撞_s':round(age,3),
         '前约1秒实际机身前后位移_m':round(net,3),'前约1秒实际路径_m':round(dist,3),'运动分类':direction,
         '目标距离趋势':trend,'目标距离变化_m':round(delta_goal,3) if delta_goal is not None else '',
         '碰撞位置_xy':json.dumps([q[0],q[1]],ensure_ascii=False),
         '障碍当时位于后方':rear,'前向射线记录':rfront,
         '证据限制':'1秒窗口由实际world_pose及偏航角投影得到；碰撞为物理接触时间，策略帧约50Hz；障碍方位按地图几何中心与机器人偏航估算。'}
    out.append(row)
    case_data[run]=(w,hit,t,e,op,meta,row)

fields=list(out[0]) if out else []
with (OUT/'Recovery内部碰撞明细.csv').open('w',newline='',encoding='utf-8-sig') as f:
    wr=csv.DictWriter(f,fieldnames=fields);wr.writeheader();wr.writerows(out)

# Representative diagrams: one confirmed rearward case, one forward case, one unresolved/mixed case.
rear_back=next((r['run_id'] for r in out if r['运动分类']=='可确认后退' and r['障碍当时位于后方']=='是'),'M3-045')
forward=next((r['run_id'] for r in out if r['运动分类']=='可确认前进' and r['碰撞前模式']=='RECOVERY'),'M3-004')
back_side=next((r['run_id'] for r in out if r['运动分类']=='可确认后退' and r['障碍当时位于后方']=='否'),'M3-016')
for name,run in zip(('case1_rear_obstacle_backward','case2_forward_motion','case3_backward_obstacle_not_rear'),
                    (rear_back,forward,back_side)):
    if run not in case_data: continue
    w,hit,t,e,op,meta,row=case_data[run]
    fig,(ax,ar)=plt.subplots(2,1,figsize=(9,6),gridspec_kw={'height_ratios':[1.6,1]})
    ts=[float(f['sim_time_s'])-t for f in w]
    xs=[f['world_pose'][0] for f in w if isinstance(f.get('world_pose'),list)]
    ys=[f['world_pose'][1] for f in w if isinstance(f.get('world_pose'),list)]
    valid=[f for f in w if isinstance(f.get('world_pose'),list)]
    ax.plot(xs,ys,'o-',lw=1.5,label='Robot position (actual)')
    if valid:
        p=valid[-1]['world_pose']; ax.scatter([p[0]],[p[1]],marker='X',s=80,label='Last pose before contact')
        ax.arrow(p[0],p[1],0.25*math.cos(p[2]),0.25*math.sin(p[2]),head_width=.06,color='black',length_includes_head=True,label='Robot forward')
    if op: ax.scatter([op[0]],[op[1]],marker='s',s=90,label=f"Obstacle center {e.get('obstacle_geom_name')}")
    motion_label={'可确认后退':'confirmed backward','可确认前进':'confirmed forward','记录不足，无法判断':'indeterminate'}[row['运动分类']]
    ax.set_title(f"{run} / {meta['map']} / group {meta['group']}: {motion_label}, RA={float(row['碰撞前RA']):.3f}")
    ax.set_xlabel('World x (m)');ax.set_ylabel('World y (m)');ax.axis('equal');ax.grid(alpha=.25);ax.legend(fontsize=8)
    ra=[f.get('ra_value') for f in w]; ar.plot(ts,ra,color='#d97706',label='RA'); ar.axhline(-.05,color='gray',ls='--',lw=1,label='Entry threshold -0.05')
    ar.axvline(0,color='red',ls=':',label='Physical contact')
    modes=[1 if mode(f)=='RECOVERY' else 0 for f in w]
    ar.fill_between(ts,[-1.05]*len(ts),[-.85]*len(ts),where=[bool(x) for x in modes],step='mid',color='#8b5cf6',alpha=.55,label='Recovery state')
    ar.set_xlim(min(ts,default=-1),0.05);ar.set_xlabel('Time from physical collision (s)');ar.set_ylabel('RA');ar.grid(alpha=.25);ar.legend(fontsize=8,ncol=3)
    fig.tight_layout();fig.savefig(OUT/f'{name}.png',dpi=150);plt.close(fig)

print('events',len(out),'recovery',sum(r['碰撞前模式']=='RECOVERY' for r in out),'backward',sum(r['运动分类']=='可确认后退' for r in out),'forward',sum(r['运动分类']=='可确认前进' for r in out))
for r in out: print(r['run_id'],r['组别'],r['地图'],r['运动分类'],r['障碍当时位于后方'],r['前约1秒实际机身前后位移_m'],r['碰撞前RA'])
