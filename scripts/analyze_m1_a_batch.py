#!/usr/bin/env python3
"""Create M1 development A repeat tables from the frozen batch plan and child analyses."""
from __future__ import annotations
import argparse,csv,json
from pathlib import Path

FIELDS=[("supervisor_terminal_result","终局"),("record_validity","数据有效性"),
        ("target_distance_final_m","终点距离m"),("elapsed_task_duration_s","任务经过s"),
        ("valid_motion_duration_s","有效连续s"),("task_valid_time_coverage_fraction","时间覆盖"),
        ("path_length_m","有效路径m"),("mean_path_speed_mps","平均速度m/s"),
        ("policy_frequency_hz_overall","策略Hz"),("mode_transitions","模式切换"),
        ("short_reverse_switch_count","短反向切换"),("short_reverse_switch_coverage","短反向覆盖"),
        ("recovery_sim_duration_s","Recovery有效s"),("recovery_share_valid_motion_time","Recovery占比"),
        ("recovery_hold_audit_status","保持审计"),("task_window_safety.foot_contact_count","足端接触"),
        ("task_window_safety.foot_impact_count","足端撞击"),("task_window_safety.nonfoot_collision_failure","非足端碰撞"),
        ("motion_fall_result","任务期跌倒"),("declared_boundary_observation.status","越界采样状态"),
        ("standing_observation_status","站立观察"),("passive_controller_transition_confirmed","PASSIVE转换确认"),
        ("rl_step_gaps","周期缺口"),("invalid_clock_frames","无效时钟帧")]

def get(item,path):
    value=item
    for key in path.split("."):
        value=value.get(key) if isinstance(value,dict) else None
    return value

def main():
    p=argparse.ArgumentParser();p.add_argument("batch_root",type=Path);a=p.parse_args()
    root=a.batch_root.resolve();plan=json.loads((root/"m1_a_batch_plan.json").read_text(encoding="utf-8"))
    rows=[]
    for item in plan["runs"]:
        latest=item.get("attempts",[])[-1] if item.get("attempts") else {}
        child=Path(latest.get("child_root","")) if latest.get("child_root") else None
        metric={}
        if child:
            summary_path=child/"batch_summary.json"
            if summary_path.is_file():
                s=json.loads(summary_path.read_text(encoding="utf-8"));arr=s.get("groups",{}).get("A",{}).get("runs",[])
                if arr: metric=arr[0]
        row={"sequence":item["sequence"],"scene_id":item["scene_id"],"filename":item["filename"],
             "density":item["density"],"route_structure":item["route_structure"],"repeat":item["repeat"],
             "status":item["status"],"run_id":latest.get("run_id"),"child_root":str(child) if child else None,
             "metrics":metric}
        rows.append(row)
    out=root/"m1_results";out.mkdir(exist_ok=True)
    columns=["sequence","scene_id","filename","density","route_structure","repeat","status","run_id","child_root"]+[k for k,_ in FIELDS]
    with (out/"development_A_results.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=columns);w.writeheader()
        for row in rows:
            flat={k:row.get(k) for k in columns[:9]}
            for key,_ in FIELDS: flat[key]=get(row["metrics"],key)
            w.writerow(flat)
    lines=["# M1 开发集 A 组单次与重复结果（非正式流程核验）","",
           f"批次状态：{sum(r['status'] in ('VALID_TASK_RESULT','RETRIED_VALID_TASK_RESULT') for r in rows)}/90 条取得VALID记录。每图最多3次，轮次为全体首跑→全体第二跑→全体第三跑。",
           "到达/碰撞/跌倒/超时/安全中止均按真实任务终局保留；INVALID/未运行单列。各次运行均为固定流程诊断，不用于论文正式成功率、组间比较或参数优选。越界状态是策略决策周期位置采样，不证明周期之间的连续物理足迹。有效时钟缺口不插值，未知不当作0。","",
           "|Scene ID|类别/结构|第1次|第2次|第3次|", "|---|---|---|---|---|"]
    for scene in sorted({r["scene_id"] for r in rows}):
        rr=[r for r in rows if r["scene_id"]==scene]
        cells=[]
        for r in sorted(rr,key=lambda x:x["repeat"]):
            m=r["metrics"];terminal=m.get("supervisor_terminal_result") or m.get("navigation_terminal_result") or "UNKNOWN"
            d=m.get("target_distance_final_m");valid=m.get("record_validity","UNKNOWN")
            path=(Path(r["child_root"])/"A/rep_01"/str(r["run_id"])) if r.get("child_root") and r.get("run_id") else None
            link=f"[记录/指标]({path.as_posix()})" if path and path.exists() else "无运行记录"
            cells.append(f"{terminal}; {valid}; d={d if d is not None else 'UNKNOWN'}m; {link}")
        label=rr[0]["density"]+" / "+rr[0]["route_structure"]
        lines.append(f"|`{scene}`|{label}|"+"|".join(cells+(["未运行"]*(3-len(cells))))+"|")
    (out/"development_A_results.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
    (out/"indexed_runs.json").write_text(json.dumps(rows,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    print(f"wrote M1 result views: {out}; valid={sum(r['status'] in ('VALID_TASK_RESULT','RETRIED_VALID_TASK_RESULT') for r in rows)}/90")
if __name__=="__main__":main()
