#!/usr/bin/env python3
"""Round-robin, resumable three-repeat A diagnostic over M1 development maps.

Held-out scenes are never selected. Each invocation delegates exactly one run
to the existing production manual_ab_compare/s1_run_diagnostic chain.
"""
from __future__ import annotations
import argparse, hashlib, json, os, subprocess, sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"scripts"))
import manual_ab_compare as manual
from s1_run_diagnostic import overlay_configuration_matches
LAYOUT=ROOT/"docs/thesis_project/designs/M1/library_v1/layouts.json"
OUT=ROOT/"docs/thesis_project/evidence/M1"
DEFAULT_ROOT=ROOT/"logs/manual_m1_a/library_v1"

def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def write(path,obj): path.write_text(json.dumps(obj,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")

def make_plan(doc):
    dev=[s for s in doc["scenes"] if s["split"]=="development"]
    dev.sort(key=lambda s:(list(doc["density_bands_exclusive"]).index(s["density"]),s["route_structure"],s["generation_seed"]))
    order=[(s,r) for r in (1,2,3) for s in dev]
    return {"schema":"m1-a-dev-batch-v1","classification":"nonformal M1 development-map A-group repeatability workflow; not a formal experiment",
            "library_version":doc["library_version"],"layout_sha256":digest(LAYOUT),"scene_count":30,"run_budget":90,
            "fixed_conditions":{"group":"A","entry_threshold_E":-0.05,"goal_world_m":[7.5,0.0],"arrival_radius_m":0.5,
                "sim_limit_s":20.0,"wall_cap_s":120.0,"standing_observation_sim_s":5.0,"standing_wall_cap_s":30.0,
                "scene_field_bounds_m":doc["field_bounds_m"],"switching_behavior":"A; no hysteresis; no minimum hold",
                "startup_gain_transition_s_control_runtime":0.30,"startup_recovery_observation_enabled":False},
            "schedule":"round robin: all 30 dev scenes once, then second, then third",
            "runs":[{"sequence":i+1,"scene_id":s["scene_id"],"filename":s["filename"],"density":s["density"],
                     "route_structure":s["route_structure"],"generation_seed":s["generation_seed"],"repeat":r,
                     "status":"NOT_STARTED","retry_count":0,"attempts":[]} for i,(s,r) in enumerate(order)]}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--batch-root",type=Path,default=DEFAULT_ROOT)
    p.add_argument("--resume",action="store_true")
    p.add_argument("--preflight-only",action="store_true",help="create/validate the frozen batch plan and detached overlay without launching a scene")
    p.add_argument("--retry-failed-once",action="store_true",help="after a recorded infrastructure failure has been repaired, retry the current item once")
    p.add_argument("--display",default=os.environ.get("DISPLAY",":0"))
    p.add_argument("--xauthority",default=os.environ.get("XAUTHORITY",str(Path(os.environ.get("XDG_RUNTIME_DIR",f"/run/user/{os.getuid()}"))/"gdm/Xauthority")))
    p.add_argument("--plot-python",default=os.environ.get("ABS_PLOT_PYTHON",sys.executable))
    a=p.parse_args(); a.batch_root=a.batch_root.resolve()
    doc=json.loads(LAYOUT.read_text(encoding="utf-8"))
    if a.batch_root.exists():
        plan_path=a.batch_root/"m1_a_batch_plan.json"
        if not a.resume or not plan_path.is_file(): raise SystemExit(f"refusing to overwrite existing batch root: {a.batch_root}")
        plan=json.loads(plan_path.read_text(encoding="utf-8"))
        if plan.get("layout_sha256")!=digest(LAYOUT): raise SystemExit("layout changed since batch plan; refusing resume")
    else:
        if a.resume: raise SystemExit("--resume requested but batch plan does not exist")
        a.batch_root.mkdir(parents=True)
        plan=make_plan(doc);write(a.batch_root/"m1_a_batch_plan.json",plan)
    source=ROOT/"quadruped_ros2_control_humble/install/go2_description"
    overlay=a.batch_root/"shared_overlay/A"
    if not overlay.exists():
        manual.prepare_overlay(source,overlay,"A",-0.05,-0.08,0.30)
        manual.set_overlay_goal(overlay/"share/go2_description/config/abs/config.yaml",7.5,0.0)
    detached=manual.verify_overlay_detached(source,overlay)
    config=(overlay/"share/go2_description/config/abs/config.yaml").read_text(encoding="utf-8")
    if not overlay_configuration_matches(config,"A",-0.05,-0.08,0.30,7.5,0.0):
        raise SystemExit("shared M1 A overlay does not match production config contract")
    plan["shared_overlay"]={"path":str(overlay),"config_goal_world_m":[7.5,0.0],
                            "entry_threshold_E":-0.05,"independent_files":detached,
                            "configuration":"abs.switching.group=A; no exit override/hysteresis/hold"}
    write(a.batch_root/"m1_a_batch_plan.json",plan)
    if a.preflight_only:
        print(f"M1 preflight ready: 60-map catalog and production reader were previously checked; detached A overlay validates group=A, E=-0.05, goal=(7.5,0). No strategy scene was launched.\nPlan: {a.batch_root/'m1_a_batch_plan.json'}")
        return 0
    plan_path=a.batch_root/"m1_a_batch_plan.json"
    current=next((x for x in plan["runs"] if x["status"] not in ("VALID_TASK_RESULT","RETRIED_VALID_TASK_RESULT","SKIPPED_ALREADY_VALID")),None)
    if current is None:
        print("M1 development A schedule is complete.");return 0
    if current["status"]=="INFRASTRUCTURE_FAILURE_STOPPED":
        if not a.retry_failed_once or current["retry_count"]>=1:
            print(f"Stopped at infrastructure failure {current['scene_id']} repeat {current['repeat']}; repair then use --resume --retry-failed-once.")
            return 1
        current["retry_count"]+=1
    elif a.retry_failed_once:
        raise SystemExit("--retry-failed-once applies only to the current recorded infrastructure failure")
    elif current["status"] in ("OPERATOR_ABORT_STOPPED","RUNNING"):
        raise SystemExit(f"run state is {current['status']}; inspect original output and recover explicitly; no automatic restart")
    print(f"Running sequence {current['sequence']}/90: {current['scene_id']} repeat {current['repeat']} (A, E=-0.05, goal 7.5m)",flush=True)
    child=a.batch_root/current["scene_id"]/f"rep_{current['repeat']:02d}_attempt_{current['retry_count']+1:02d}"
    child.parent.mkdir(parents=True,exist_ok=True)
    current["status"]="RUNNING"; current["active_output_root"]=str(child); write(plan_path,plan)
    cmd=["/usr/bin/python3","-u",str(ROOT/"scripts/manual_ab_compare.py"),"--scene",current["filename"],
         "--goal-x","7.5","--goal-y","0","--sim-limit-s","20","--wall-cap-s","120",
         "--groups","A","--repeats","1","--entry-threshold","-0.05","--classification",plan["classification"],
         "--display",a.display,"--xauthority",a.xauthority,"--plot-python",a.plot_python,"--result-root",str(child)]
    cmd.extend(["--prepared-overlay",str(overlay)])
    log=child.parent/(child.name+"_console.log")
    with log.open("w",encoding="utf-8") as f:
        proc=subprocess.Popen(cmd,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        try:
            for line in proc.stdout:
                print(line,end="",flush=True);f.write(line);f.flush()
            rc=proc.wait()
        except KeyboardInterrupt:
            if proc.poll() is None: proc.send_signal(2)
            proc.wait(timeout=60);current["status"]="OPERATOR_ABORT_STOPPED";write(plan_path,plan);return 130
    childplan=child/"batch_plan.json"
    child_status={}
    if childplan.is_file():
        cp=json.loads(childplan.read_text(encoding="utf-8"))
        child_status=next((x for x in cp.get("run_statuses",[]) if x.get("group")=="A"),{})
    attempt={"attempt":current["retry_count"]+1,"child_root":str(child),"exit_code":rc,
             "child_status":child_status.get("status"),"run_id":child_status.get("run_id"),
             "result":child_status.get("result"),"record_validity":child_status.get("record_validity"),
             "console_log":str(log)}
    current["attempts"].append(attempt);current.pop("active_output_root",None)
    if rc==0 and child_status.get("record_validity")=="VALID":
        current["status"]="VALID_TASK_RESULT" if current["retry_count"]==0 else "RETRIED_VALID_TASK_RESULT"
    else:
        current["status"]="INFRASTRUCTURE_FAILURE_STOPPED"
        current["stop_reason"]="runner/record/source/cleanup failure; original artifacts retained; no later run started"
    write(plan_path,plan)
    print(f"M1 batch checkpoint: {current['status']} · {current['scene_id']} repeat {current['repeat']}; next launch only after this run's cleanup was confirmed.")
    if current["status"] in ("VALID_TASK_RESULT","RETRIED_VALID_TASK_RESULT"):
        # Each child run has already closed and verified its own processes.
        # Re-enter through the persisted plan so interruption/failure is resumable.
        argv=[sys.executable,"-u",str(Path(__file__).resolve()),"--batch-root",str(a.batch_root),"--resume",
              "--display",a.display,"--xauthority",a.xauthority,"--plot-python",a.plot_python]
        os.execv(sys.executable,argv)
    return 1

if __name__=="__main__":raise SystemExit(main())
