#!/usr/bin/env python3
"""Run the authorized 12-item R1 geometry-versus-predicted-ray comparison."""

import json
import os
import random
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import manual_ab_compare as manual  # noqa: E402

OUTPUT = ROOT / "logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12"
MODEL = ROOT / "archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt"
PREDICTOR_PYTHON = Path("/home/lidio/anaconda3/envs/abs/bin/python")
SCENES = ("scene_flat.xml", "challenge27.xml")
SOURCES = ("geometric", "prediction")
LEGAL_TERMINALS = {"ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION",
                   "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE"}


def write_plan(plan):
    path = OUTPUT / "run_plan.json"
    path.write_text(json.dumps(plan, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def build_schedule():
    items = [(scene, source) for scene in SCENES for source in SOURCES]
    rng = random.Random(2026110203)
    rng.shuffle(items)
    schedule = []
    for round_index in range(3):
        order = items[round_index:] + items[:round_index]
        if round_index % 2:
            order.reverse()
        schedule.extend({"index": len(schedule) + 1, "round": round_index + 1,
                         "scene": scene, "ray_source": source, "status": "NOT_STARTED"}
                        for scene, source in order)
    return schedule


def tail_stop_log(path):
    if not path.is_file():
        return ""
    return "\n".join(path.read_text(encoding="utf-8", errors="replace").splitlines()[-40:])


def run_one(item, overlay):
    attempts = item.setdefault("attempts", [])
    attempt_number = len(attempts) + 1
    stem = f"{item['index']:02d}_{item['scene'].removesuffix('.xml')}_{item['ray_source']}"
    run_root = OUTPUT / "runs" / f"{stem}_attempt_{attempt_number:02d}"
    while run_root.exists():
        attempt_number += 1
        run_root = OUTPUT / "runs" / f"{stem}_attempt_{attempt_number:02d}"
    command = [sys.executable, "-u", str(ROOT / "scripts/manual_ab_compare.py"),
               "--scene", item["scene"], "--goal-x", "7.5", "--goal-y", "0",
               "--sim-limit-s", "20", "--wall-cap-s", "120",
               "--standing-observation-sim-s", "2", "--groups", "A", "--repeats", "1",
               "--entry-threshold", "-0.05", "--exit-threshold", "-0.08", "--hold-s", "0.1",
               "--prepared-overlay", str(overlay), "--result-root", str(run_root),
               "--classification", "R1 Periodic A geometry versus MuJoCo depth predicted rays; 12-item batch",
               "--ray-source", item["ray_source"], "--ray-model", str(MODEL),
               "--ray-python", str(PREDICTOR_PYTHON), "--render-backend", "egl", "--no-rviz",
               "--disable-framebuffer-capture", "--plot-python", "/usr/bin/python3"]
    if item["ray_source"] == "prediction":
        command.extend(["--ray-device", "cuda"])
    log_path = run_root.parent / f"{run_root.name}_console.log"
    if log_path.exists():
        log_path = run_root.parent / f"{run_root.name}_prelaunch_retry_console.log"
    print(f"\n[{item['index']}/12] {item['scene']} / {item['ray_source']}", flush=True)
    run_root.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log:
        child = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, bufsize=1)
        try:
            for line in child.stdout:
                print(line, end="", flush=True)
                log.write(line)
                log.flush()
            returncode = child.wait()
        except KeyboardInterrupt:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=60)
                except subprocess.TimeoutExpired:
                    child.send_signal(signal.SIGTERM)
                    child.wait(timeout=10)
            raise
    summary_path = run_root / "batch_summary.json"
    if not summary_path.is_file():
        attempts.append({"attempt": attempt_number, "status": "ENGINEERING_FAILURE_NO_SUMMARY",
                         "returncode": returncode, "console_log": str(log_path),
                         "reason": tail_stop_log(log_path)})
        item["status"] = "STARTUP_FAILURE_NO_POLICY_CYCLE"
        raise RuntimeError(f"manual runner returned {returncode} without summary; log={log_path}\n{tail_stop_log(log_path)}")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    runs = summary.get("groups", {}).get("A", {}).get("runs", [])
    if len(runs) != 1:
        raise RuntimeError(f"expected one run summary, got {len(runs)}: {summary_path}")
    result = runs[0]
    run_dir = run_root / "A" / "rep_01" / result["run_id"]
    context = json.loads((run_dir / "run_context.json").read_text(encoding="utf-8"))
    facts = json.loads((run_dir / "process_facts.json").read_text(encoding="utf-8"))
    terminal = result.get("terminal_result")
    if not facts.get("shutdown_complete") or not facts.get("closeout_complete"):
        raise RuntimeError(f"run cleanup unconfirmed: {run_dir}")
    if terminal == "STARTUP_FAILURE":
        attempts.append({"attempt": attempt_number, "status": "STARTUP_FAILURE_NO_POLICY_CYCLE",
                         "returncode": returncode, "run_id": result["run_id"],
                         "run_directory": str(run_dir), "console_log": str(log_path),
                         "cleanup_confirmed": True,
                         "strategy_cycles": result.get("strategy_cycles", 0)})
        item.update({"status": "STARTUP_FAILURE_NO_POLICY_CYCLE",
                     "latest_attempt_directory": str(run_dir)})
        raise RuntimeError(f"startup failed before policy execution; preserving attempt {run_dir}\n{tail_stop_log(log_path)}")
    if terminal == "INVALID_RUNTIME_SOURCE":
        attempts.append({"attempt": attempt_number, "status": "INVALID_RUNTIME_SOURCE_PRETASK_STOP",
                         "returncode": returncode, "run_id": result["run_id"],
                         "run_directory": str(run_dir), "console_log": str(log_path),
                         "cleanup_confirmed": True, "strategy_cycles": result.get("strategy_cycles")})
        item["status"] = "NOT_STARTED"
        item["latest_attempt_directory"] = str(run_dir)
        raise RuntimeError(f"runtime source gate rejected this attempt before valid task execution; preserving {run_dir}")
    if returncode:
        raise RuntimeError(f"manual runner returned {returncode}; log={log_path}\n{tail_stop_log(log_path)}")
    if terminal == "SYSTEM_SAFETY_ABORT":
        raise RuntimeError(f"system safety abort; preserving run and pausing: {run_dir}")
    if terminal not in LEGAL_TERMINALS:
        raise RuntimeError(f"unrecognized or infrastructure terminal {terminal!r}; pausing at {run_dir}")
    if context.get("settings", {}).get("ray_source") != item["ray_source"]:
        raise RuntimeError(f"actual ray-source setting differs from schedule: {run_dir}")
    ros_text = (run_dir / "ros_launch.log").read_text(encoding="utf-8", errors="replace")
    if "[TWIST-UPDATE-CONFIG] mode=Periodic period_s=0.060" not in ros_text:
        raise RuntimeError(f"Periodic P=0.06 configuration was not confirmed: {run_dir}")
    if item["ray_source"] == "prediction":
        pred_text = (run_dir / "ray_predictor.log").read_text(encoding="utf-8", errors="replace")
        if "[RayPredictorFrame]" not in pred_text or not context.get("preflight", {}).get("ray_prediction_source", {}).get("status") == "FRESH_FRAME":
            raise RuntimeError(f"predicted ray frame was not confirmed: {run_dir}")
    attempts.append({"attempt": attempt_number, "status": "NAVIGATION_TERMINAL",
                     "returncode": returncode, "run_id": result["run_id"],
                     "run_directory": str(run_dir), "console_log": str(log_path),
                     "cleanup_confirmed": True, "terminal": terminal,
                     "record_validity": result.get("record_validity")})
    item.update({"status": "FINISHED", "run_id": result["run_id"], "terminal": terminal,
                 "record_validity": result.get("record_validity"), "run_directory": str(run_dir),
                 "ray_source": item["ray_source"], "shutdown_complete": facts["shutdown_complete"],
                 "closeout_complete": facts["closeout_complete"]})


def main():
    resumed = OUTPUT.exists()
    if resumed:
        previous_plan_path = OUTPUT / "run_plan.json"
        if not previous_plan_path.is_file():
            raise SystemExit(f"refusing to overwrite unrelated existing batch output: {OUTPUT}")
        plan = json.loads(previous_plan_path.read_text(encoding="utf-8"))
        schedule = plan.get("schedule", [])
        if len(schedule) != 12 or plan.get("seed") != 2026110203:
            raise SystemExit("existing plan does not match the frozen 12-item schedule")
        if any(item.get("status") not in {"FINISHED", "NOT_STARTED", "STARTUP_FAILURE_NO_POLICY_CYCLE"}
               for item in schedule):
            raise SystemExit("existing plan has an unresolved RUNNING or infrastructure status")
        if any(item.get("status") == "FINISHED" and not item.get("run_id") for item in schedule):
            raise SystemExit("finished schedule item lacks its recorded run id")
        for item in schedule:
            if item.get("status") == "STARTUP_FAILURE_NO_POLICY_CYCLE":
                item.setdefault("attempts", []).append({
                    "attempt": 1, "status": "STARTUP_FAILURE_NO_POLICY_CYCLE",
                    "run_id": item.get("run_id"), "terminal": item.get("terminal"),
                    "reason": item.get("reason"), "run_directory": item.get("run_directory"),
                    "cleanup_confirmed": item.get("cleanup_confirmed")})
                break
    else:
        schedule = build_schedule()
        plan = {"batch": "R1_GO2_RAY_PRED_12", "status": "RUNNING",
                "seed": 2026110203, "schedule": schedule,
                "scenes": {"scene_flat.xml": {"role": "open", "obstacles": 0},
                           "challenge27.xml": {"role": "registered bypassable obstacle map", "obstacles": 7}},
                "conditions": {"group": "A", "E": -0.05, "effective_X": -0.05, "H_sim_s": 0.0,
                               "hysteresis": False, "twist_update": "Periodic", "P_sim_s": 0.06,
                               "goal_world_m": [7.5, 0.0], "arrival_radius_m": 0.5,
                               "sim_limit_s": 20.0, "wall_cap_s": 120.0,
                               "standing_observation_sim_s": 2.0,
                               "rendering": "hidden surfaceless EGL pbuffer, RViz off, no capture"},
                "ray_model": {"path": str(MODEL), "origin": "archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611",
                              "training_restriction": "historical fine-tuned checkpoint; not trained or updated for this batch"},
                "prelaunch_attempts": []}
    if not MODEL.is_file() or not PREDICTOR_PYTHON.is_file():
        raise SystemExit("explicit model or predictor Python environment is unavailable")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    plan["status"] = "RUNNING"
    plan["conditions"]["rendering"] = "hidden surfaceless EGL pbuffer, RViz off, no capture"
    write_plan(plan)
    install = ROOT / "quadruped_ros2_control_humble/install/go2_description"
    overlay = Path(tempfile.mkdtemp(prefix="r1_go2_ray_pred_12_")) / "periodic_A"
    try:
        manual.prepare_overlay(install, overlay, "A", -0.05, -0.05, 0.1)
        config = overlay / "share/go2_description/config/abs/config.yaml"
        manual.set_overlay_goal(config, 7.5, 0.0)
        text = config.read_text(encoding="utf-8")
        if "  twist_update:" in text:
            raise RuntimeError("base config already has twist_update; refusing ambiguous overlay edit")
        marker = "  # Recovery-aware straight-line tracking:"
        if text.count(marker) != 1:
            raise RuntimeError("could not locate unique ABS config insertion point")
        text = text.replace(marker, '  twist_update:\n    mode: "Periodic"\n\n' + marker)
        config.write_text(text, encoding="utf-8")
        (OUTPUT / "effective_config.yaml").write_text(text, encoding="utf-8")
        for item in schedule:
            if item.get("status") == "FINISHED":
                continue
            item["status"] = "RUNNING"
            write_plan(plan)
            run_one(item, overlay)
            write_plan(plan)
        plan["status"] = "COMPLETE"
    except Exception as exc:
        plan["status"] = "PAUSED"
        plan["stop_reason"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        plan["schedule"] = schedule
        write_plan(plan)
        shutil.rmtree(overlay.parent, ignore_errors=True)
    print(f"Completed {len(schedule)} planned items: {OUTPUT}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
