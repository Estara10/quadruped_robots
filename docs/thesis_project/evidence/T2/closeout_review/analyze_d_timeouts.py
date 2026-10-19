#!/usr/bin/env python3
"""Offline, read-only closeout analysis for the four M2 D-group timeouts."""
from __future__ import annotations

import csv
import json
import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon

ROOT = Path(__file__).resolve().parents[5]
EVIDENCE = ROOT / "docs/thesis_project/evidence/T2"
RAW = ROOT / "logs/02_T2_minimum_hold/combined_check_48/20261019_1358_01_seed2026101901"
ANALYSIS = EVIDENCE / "combined_check_48/analysis"
OUT = EVIDENCE / "closeout_review"
TARGET = (7.5, 0.0)
E = -0.05
X = -0.10
H = 0.20


def read_rows():
    with (ANALYSIS / "per_run_results.csv").open(newline="") as f:
        return list(csv.DictReader(f))


def run_dir(row):
    p = Path(row["run_directory"]).resolve()
    if not p.exists():
        raise FileNotFoundError(p)
    return p


def read_run(row):
    p = run_dir(row)
    metrics = json.loads((p / "metrics.json").read_text())
    terminal = json.loads((p / "terminal.json").read_text())
    context = json.loads((p / "run_context.json").read_text())
    boundary = metrics.get("rl_task_metric_boundary", {})
    first_cycle = boundary.get("start_cycle")
    terminal_cycle = boundary.get("end_cycle")
    frames = []
    with (p / "runtime_record.jsonl").open() as f:
        for line in f:
            rec = json.loads(line)
            if rec.get("kind") != "frame":
                continue
            q = rec.get("payload", {})
            if q.get("rl_active") != 1 or q.get("rl_entered") != 1:
                continue
            cycle = int(q.get("policy_cycle_id", -1))
            # Explicitly exclude any post-terminal standing/cleanup tail.
            if first_cycle is not None and cycle < int(first_cycle):
                continue
            if terminal_cycle is not None and cycle > int(terminal_cycle):
                continue
            t = q.get("sim_time_s")
            if t is None or not q.get("sim_clock_valid", False):
                continue
            pose, vel, cmd = q.get("world_pose"), q.get("lin_vel"), q.get("command")
            if not pose or len(pose) < 3:
                continue
            frames.append({
                "t": float(t), "cycle": cycle,
                "x": float(pose[0]), "y": float(pose[1]), "yaw": float(pose[2]),
                "ra": float(q["ra_value"]) if q.get("ra_value") is not None else math.nan,
                "mode": int(q.get("mode_after", q.get("policy_state", 0))),
                "mode_before": int(q.get("mode_before", q.get("policy_state", 0))),
                "hold_enabled": bool(q.get("hold_enabled", False)),
                "hold_enter": q.get("hold_enter_sim_time_s"),
                "hold_elapsed": q.get("hold_elapsed_sim_s"),
                "exit_threshold": q.get("exit_threshold"),
                "risk_entered": int(q.get("risk_condition_entered", 0) or 0),
                "controller_active": q.get("controller_active"),
                "safety_faulted": q.get("safety_faulted"),
                "action_raw": q.get("action_raw"),
                "action_clipped": q.get("action_clipped"),
                "joint_target": q.get("joint_target_rad"),
                "body_vx": float(vel[0]) if vel and len(vel) >= 2 else math.nan,
                "body_vy": float(vel[1]) if vel and len(vel) >= 2 else math.nan,
                "cmd0": float(cmd[0]) if cmd and len(cmd) >= 3 else math.nan,
                "cmd1": float(cmd[1]) if cmd and len(cmd) >= 3 else math.nan,
                "cmd2": float(cmd[2]) if cmd and len(cmd) >= 3 else math.nan,
                "source": q.get("source"),
                "session": q.get("session_id"),
            })
    frames.sort(key=lambda z: (z["t"], z["cycle"]))
    return p, metrics, terminal, context, frames


def episodes_from_frames(frames):
    eps = []
    active = None
    for i, f in enumerate(frames):
        entered = f["mode_before"] == 0 and f["mode"] == 1
        exited = f["mode_before"] == 1 and f["mode"] == 0
        if entered:
            active = {"entry": f, "rows": [f], "exit": None}
            eps.append(active)
        elif active is not None:
            active["rows"].append(f)
        if exited and active is not None:
            active["exit"] = f
            active = None
    for ep in eps:
        rows = ep["rows"]
        vals = [r["hold_enter"] for r in rows if r["hold_enter"] is not None]
        ep["hold_entry_values"] = vals
        ep["hold_entry_stable"] = bool(vals) and max(vals) - min(vals) <= 1e-8
        # Do not round a just-below-H value upward: production audit follows
        # the observed effective-cycle boundary and the serialized value.
        due = [r for r in rows if r["hold_elapsed"] is not None and float(r["hold_elapsed"]) >= H]
        ep["first_due"] = due[0] if due else None
        ep["risk_reentries"] = sum(r["risk_entered"] == 1 for r in rows[1:])
    return eps


def parse_obstacles(scene):
    path = ROOT / f"unitree_mujoco/unitree_robots/go2/{scene}"
    root = ET.parse(path).getroot()
    out = []
    for g in root.iter("geom"):
        name = g.get("name", "")
        if not name.startswith("ppt_obstacle_"):
            continue
        typ = g.get("type", "")
        pos = [float(z) for z in g.get("pos", "0 0 0").split()]
        size = [float(z) for z in g.get("size", "").split()]
        yaw = 0.0
        if g.get("euler"):
            yaw = float(g.get("euler").split()[2])
        out.append((typ, pos, size, yaw, name))
    return out


def add_scene(ax, scene):
    for typ, pos, size, yaw, name in parse_obstacles(scene):
        if typ == "cylinder":
            ax.add_patch(Circle((pos[0], pos[1]), size[0], fc="#787878", ec="#333333", alpha=.65, lw=.7))
        elif typ == "box":
            sx, sy = size[:2]
            pts = [(-sx, -sy), (sx, -sy), (sx, sy), (-sx, sy)]
            c, s = math.cos(yaw), math.sin(yaw)
            xy = [(pos[0] + x*c-y*s, pos[1] + x*s+y*c) for x, y in pts]
            ax.add_patch(Polygon(xy, closed=True, fc="#787878", ec="#333333", alpha=.65, lw=.7))
        ax.text(pos[0], pos[1], name[-2:], ha="center", va="center", fontsize=6, color="white")


def finite_vector(a):
    return isinstance(a, list) and bool(a) and all(isinstance(v, (int, float)) and math.isfinite(v) for v in a)


def summarize(row, loaded):
    p, m, term, ctx, frames = loaded
    eps = episodes_from_frames(frames)
    ds = [math.hypot(f["x"]-TARGET[0], f["y"]-TARGET[1]) for f in frames]
    path = sum(math.hypot(b["x"]-a["x"], b["y"]-a["y"]) for a,b in zip(frames, frames[1:]))
    start_d = ds[0] if ds else math.nan
    end_d = ds[-1] if ds else math.nan
    d_by_time = [(f["t"], d) for f,d in zip(frames,ds)]
    progress_bins=[]
    if frames:
        t0=math.ceil(frames[0]["t"])
        t1=math.floor(frames[-1]["t"])
        for lo in range(t0,t1):
            a=min((z for z in d_by_time if z[0]>=lo),default=None,key=lambda z:z[0])
            b=min((z for z in d_by_time if z[0]>=lo+1),default=None,key=lambda z:z[0])
            if a and b: progress_bins.append(a[1]-b[1])
    actual_body_speeds=[math.hypot(f["body_vx"],f["body_vy"]) for f in frames if math.isfinite(f["body_vx"]) and math.isfinite(f["body_vy"])]
    commands=[f for f in frames if math.isfinite(f["cmd0"]) and math.isfinite(f["cmd1"]) and math.isfinite(f["cmd2"])]
    intervals=m.get("recovery_intervals", [])
    complete_durations=[float(i["duration_s"]) for i in intervals if not i.get("censored") and i.get("duration_s") is not None]
    censored_intervals=[i for i in intervals if i.get("censored")]
    complete_eps=[ep for ep in eps if ep["exit"] is not None]
    max_ep=max(complete_eps,key=lambda ep:ep["exit"]["t"]-ep["entry"]["t"]) if complete_eps else None
    max_ep_due=min((f for f in max_ep["rows"] if f["hold_elapsed"] is not None and float(f["hold_elapsed"])>=H),key=lambda f:f["t"],default=None) if max_ep else None
    censored=sum(ep["exit"] is None for ep in eps)
    due_high=sum(ep["first_due"] is not None and ep["first_due"]["ra"] >= float(ep["first_due"]["exit_threshold"]) for ep in eps)
    due_low=sum(ep["first_due"] is not None and ep["first_due"]["ra"] < float(ep["first_due"]["exit_threshold"]) for ep in eps)
    due_low_then_exit=sum(ep["first_due"] is not None and ep["first_due"]["ra"] < float(ep["first_due"]["exit_threshold"]) and ep["exit"] is not None and ep["exit"]["cycle"]==ep["first_due"]["cycle"] for ep in eps)
    entry_reset=sum(not ep["hold_entry_stable"] for ep in eps)
    finite_actions=sum(finite_vector(f["action_raw"]) and finite_vector(f["action_clipped"]) and finite_vector(f["joint_target"]) for f in frames)
    controller_off=sum(f["controller_active"] not in (1, True) for f in frames)
    return {
        "sequence":row["sequence"],"map":row["map"],"repeat":row["repeat"],"group":row["group"],"run_id":row["run_id"],
        "record_validity":row["record_validity"],"terminal":row["supervisor_terminal"],
        "collision_terminal":row["collision_terminal"],"nonfoot_collision":row["nonfoot_collision"],
        "E_observed":row["actual_E"],"X_effective":row["actual_X_effective"],"H_sim_s":row["actual_H_sim_s"],
        "elapsed_sim_s":row["task_elapsed_sim_s"],"final_goal_distance_m":row["distance_to_goal_final_m"],
        "start_goal_distance_m":round(start_d,6) if math.isfinite(start_d) else "",
        "net_goal_distance_reduction_m":round(start_d-end_d,6) if math.isfinite(start_d) else "",
        "path_length_valid_segments_m":row["path_length_valid_segments_m"],
        "mean_valid_world_position_speed_mps":row["mean_speed_valid_segments_mps"],
        "mean_logged_body_xy_speed_mps":round(sum(actual_body_speeds)/len(actual_body_speeds),6) if actual_body_speeds else "",
        "mean_logged_target_command_0":round(sum(f["cmd0"] for f in commands)/len(commands),6) if commands else "",
        "mean_logged_target_command_1":round(sum(f["cmd1"] for f in commands)/len(commands),6) if commands else "",
        "mean_logged_target_command_2":round(sum(f["cmd2"] for f in commands)/len(commands),6) if commands else "",
        "1s_goal_distance_progress_median_m":round(sorted(progress_bins)[len(progress_bins)//2],6) if progress_bins else "",
        "1s_goal_distance_progress_nonpositive_bins":sum(v<=0 for v in progress_bins),
        "1s_goal_distance_progress_bins":len(progress_bins),
        "mode_edges":row["mode_switches_all_cycles"],"Agile_to_Recovery":row["A_to_Recovery"],"Recovery_to_Agile":row["Recovery_to_A"],
        "Recovery_valid_time_share":row["recovery_share_valid_time"],"Recovery_episodes_from_frames":len(eps),
        "Recovery_exit_count_observed":row["Recovery_exit_count_observed"],
        "Recovery_reentry_within_0_5s":row["Recovery_reentry_within_0_5s"],
        "Recovery_reentry_observable_denominator":row["Recovery_reentry_observable_denominator"],
        "Recovery_reentry_rate_observable_only":row["Recovery_reentry_rate_observable_only"],
        "Recovery_reentry_window_censored":row["Recovery_reentry_window_censored"],
        "Recovery_episodes_censored":len(censored_intervals),
        "Recovery_duration_median_complete_s":round(sorted(complete_durations)[len(complete_durations)//2],6) if complete_durations else "",
        "Recovery_duration_max_complete_s":round(max(complete_durations),6) if complete_durations else "",
        "max_complete_episode_entry_cycle":max_ep["entry"]["cycle"] if max_ep else "",
        "max_complete_episode_exit_cycle":max_ep["exit"]["cycle"] if max_ep else "",
        "max_complete_episode_RA_at_first_H_due":round(max_ep_due["ra"],6) if max_ep_due else "",
        "max_complete_episode_X_at_first_H_due":round(float(max_ep_due["exit_threshold"]),6) if max_ep_due else "",
        "censored_episode_observed_lower_bound_s":round(frames[-1]["t"]-float(censored_intervals[-1]["start_sim_time_s"]),6) if censored_intervals and frames else "",
        "first_H_due_RA_ge_exit_threshold_count":due_high,"first_H_due_RA_lt_exit_threshold_count":due_low,
        "RA_lt_exit_threshold_first_due_exited_same_cycle_count":due_low_then_exit,
        "hold_entry_timestamp_restarts_in_episode_count":entry_reset,
        "risk_reentry_inside_Recovery_count":sum(ep["risk_reentries"] for ep in eps),
        "episodes_with_H_audit_PASS":sum(ep.get("hold_rule") == "PASS" for ep in json.loads((p/"metrics.json").read_text())["recovery_hold_audit"]["episodes"]),
        "episodes_audit_incomplete":row["hold_episodes_incomplete"],"hold_violations":row["hold_violations"],
        "task_cycle_gaps":row["task_cycle_gaps"],"task_invalid_clock_cycles":row["task_invalid_clock_cycles"],
        "RA_min_task":row["ra_min_task"],"RA_max_task":row["ra_max_task"],
        "policy_frequency_hz":row["policy_frequency_hz"],
        "collision_coverage_complete":row["collision_coverage_complete"],"fall_result":row["fall"],
        "task_terminal_policy_state":(term.get("run_terminal_frame_ref") or {}).get("policy_state"),
        "all_actions_targets_finite_count":finite_actions,"active_controller_missing_frames":controller_off,
        "capture_frame_count":row["analyzed_frame_count"] if "analyzed_frame_count" in row else len(frames),
        "command_consumption_evidence":row["command_consumption_evidence"],
        "process_shutdown_complete":row["process_shutdown_complete"],"closeout_complete":row["closeout_complete"],
        "run_directory_canonical":str(p),
    }


def plot_map(scene, rows, loaded_by_id, outfile):
    colors={"A":"#4c78a8","B":"#f58518","C":"#54a24b","D":"#e45756"}
    fig, axes=plt.subplots(2,3,figsize=(17,10),constrained_layout=True)
    axxy,axd,axra,axmode,axspeed,axcmd=axes.flat
    add_scene(axxy,scene)
    axxy.scatter([0],[0],marker="o",c="black",s=25,label="run start")
    axxy.scatter([TARGET[0]],[TARGET[1]],marker="*",c="gold",edgecolor="black",s=150,label="goal (7.5,0)")
    mode_labels=[]
    mode_end=0.0
    for lane, row in enumerate(rows):
        _,m,_,_,frames=loaded_by_id[row["run_id"]]
        if not frames: continue
        col=colors[row["group"]]; ls="-" if row["repeat"]=="1" else "--"
        ts=[f["t"]-frames[0]["t"] for f in frames]
        lab=f'{row["group"]} r{row["repeat"]}'
        axxy.plot([f["x"] for f in frames],[f["y"] for f in frames],color=col,ls=ls,lw=1.5,label=lab,alpha=.9)
        ds=[math.hypot(f["x"]-TARGET[0],f["y"]-TARGET[1]) for f in frames]
        axd.plot(ts,ds,color=col,ls=ls,lw=1.2,label=lab)
        axra.plot(ts,[f["ra"] for f in frames],color=col,ls=ls,lw=1.0,label=lab)
        modes=[f["mode"] for f in frames]
        mode_labels.append(lab)
        mode_end=max(mode_end,ts[-1])
        start=0
        while start < len(modes)-1:
            state=modes[start]
            stop=start+1
            while stop < len(modes) and modes[stop] == state:
                stop += 1
            width=ts[min(stop,len(ts)-1)]-ts[start]
            if width > 0:
                axmode.broken_barh([(ts[start],width)],(lane-.35,.7),facecolors="#f58518" if state else "#4c78a8",edgecolors="none")
            start=stop
        bxy=[math.hypot(f["body_vx"],f["body_vy"]) if math.isfinite(f["body_vx"]) and math.isfinite(f["body_vy"]) else math.nan for f in frames]
        axspeed.plot(ts,bxy,color=col,ls=ls,lw=1.0,label=lab)
        axcmd.plot(ts,[f["cmd0"] for f in frames],color=col,ls=ls,lw=1.0,label=f"{lab} cmd[0]")
        axcmd.plot(ts,[f["cmd1"] for f in frames],color=col,ls=ls,lw=.8,alpha=.6,label=f"{lab} cmd[1]")
    axxy.set(title=f"{scene}: world XY trajectory and obstacle geoms",xlabel="world x (m)",ylabel="world y (m)",xlim=(-.5,8),ylim=(-3,3),aspect="equal")
    axxy.legend(ncol=2,fontsize=7)
    axd.set(title="Distance to goal",xlabel="task sim time from first active frame (s)",ylabel="distance (m)")
    axra.set(title="Recorded RA and thresholds",xlabel="task sim time from first active frame (s)",ylabel="RA")
    axra.axhline(E,color="black",ls=":",lw=1,label="E=-0.05")
    axra.axhline(X,color="gray",ls="--",lw=1,label="X=-0.10")
    axra.legend(fontsize=7,ncol=2)
    axmode.set(title="Mode by run (blue=Agile, orange=Recovery)",xlabel="task sim time from first active frame (s)",ylabel="group / repeat",yticks=range(len(rows)),yticklabels=mode_labels,ylim=(-.5,len(rows)-.5))
    axspeed.set(title="Logged body-frame XY speed",xlabel="task sim time from first active frame (s)",ylabel="|Body XY lin_vel| (m/s)")
    axcmd.set(title="Logged target-command components (not velocity)",xlabel="task sim time from first active frame (s)",ylabel="command[0], command[1] (target input)")
    fig.suptitle(f"T2 closeout offline review — {scene}; solid=repeat 1, dashed=repeat 2\nNo speed-command field; command is target input; body XY speed is not world XY speed",fontsize=13)
    fig.savefig(outfile,dpi=160)
    plt.close(fig)


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    rows=read_rows()
    selected=[r for r in rows if r["map"] in ("m06.xml","d07.xml") and r["group"] in ("A","B","C","D")]
    loaded={r["run_id"]:read_run(r) for r in selected}
    summaries=[summarize(r,loaded[r["run_id"]]) for r in selected if r["group"]=="D"]
    headers=list(summaries[0])
    with (OUT/"d_timeout_summary.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=headers);w.writeheader();w.writerows(summaries)
    with (OUT/"m06_d07_all_group_context.csv").open("w",newline="") as f:
        allrows=[summarize(r,loaded[r["run_id"]]) for r in selected]
        w=csv.DictWriter(f,fieldnames=list(allrows[0]));w.writeheader();w.writerows(allrows)
    for scene in ("m06.xml","d07.xml"):
        srows=[r for r in selected if r["map"]==scene]
        plot_map(scene,srows,loaded,OUT/(scene.replace(".xml","_comparison.png")))
    print(f"wrote {len(summaries)} D timeout rows; {len(selected)} contextual runs; figures and CSV in {OUT}")
    for s in summaries:
        print(s["map"],s["repeat"],s["run_id"],s["terminal"],s["net_goal_distance_reduction_m"],s["path_length_valid_segments_m"],s["Recovery_valid_time_share"],s["mode_edges"],s["Recovery_duration_max_complete_s"],s["first_H_due_RA_ge_exit_threshold_count"],s["hold_entry_timestamp_restarts_in_episode_count"],s["hold_violations"])


if __name__ == "__main__":
    main()
