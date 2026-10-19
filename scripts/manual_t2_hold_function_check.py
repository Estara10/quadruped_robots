#!/usr/bin/env python3
"""T2 A/C minimum-hold function check on m02.xml (4 hidden-GLFW runs).

Fixed order: A (no hold), C H=0.10, C H=0.30, C H=0.50 simulation seconds.
The production chain is manual_ab_compare.py -> s1_run_diagnostic.py.
A/C overlays contain no exit_threshold; the CLI X=-0.10 and A's CLI H=0.30
are validator fallbacks only and are not written into the A/C configuration.
"""
from __future__ import annotations

import argparse
import csv
import fcntl
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import uuid
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import manual_ab_compare as manual  # noqa: E402
import manual_t1_m2_smoke_18 as smoke  # noqa: E402

PARENT = ROOT / "logs/t2/hold_function_check"
SOURCE = ROOT / "quadruped_ros2_control_humble/install/go2_description"
PLAN_FILE = "t2_hold_function_plan.json"
SCENE = "m02.xml"
GROUP_ORDER = (("A", None), ("C", 0.10), ("C", 0.30), ("C", 0.50))
E = -0.05
CLI_EXIT_FALLBACK = -0.10
A_CLI_HOLD_FALLBACK = 0.30
GOAL = (7.5, 0.0)
SIM_LIMIT_S = 20.0
WALL_CAP_S = 120.0
STANDING_SIM_S = 2.0
STANDING_WALL_CAP_S = 30.0
CLASSIFICATION = "T2 保持时间功能试跑，非正式实验"
LEGITIMATE_TERMINALS = {"ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION",
                        "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE"}
BASE_SCENE_STARTS = 4
MAX_SCENE_STARTS = 5
MAX_ENGINEERING_REPAIRS = 1

now = smoke.now
sha = smoke.sha
atomic_json = smoke.atomic_json
load_json = smoke.load_json
related_processes = smoke.related_processes
write_status = smoke.write_status
Tee = smoke.Tee


def item_id(group: str, hold: float | None) -> str:
    return "A_no_hold" if group == "A" else f"C_H{hold:.2f}"


def create_plan(root: Path) -> dict:
    contract = smoke.load_contract()
    base = next(row for row in contract if row["map"] == SCENE)
    items = []
    for sequence, (group, hold) in enumerate(GROUP_ORDER, start=1):
        items.append({
            "sequence": sequence, "plan_item_id": f"{sequence:02d}_{item_id(group, hold)}",
            "group": group, "hold_s": hold,
            "hold_consumed": group == "C",
            "cli_hold_fallback_s": hold if group == "C" else A_CLI_HOLD_FALLBACK,
            "cli_exit_fallback": CLI_EXIT_FALLBACK,
            "scene": SCENE, "scene_id": base["scene_id"],
            "difficulty_label": base["difficulty_label"],
            "source_set": base["source_set"], "source_filename": base["source_filename"],
            "layout_revision": base.get("layout_revision"),
            "field_bounds_m": base["field_bounds_m"], "xml_sha256": base["xml_sha256"],
            "goal_world_m": list(GOAL), "entry_threshold": E,
            "sim_limit_s": SIM_LIMIT_S, "wall_cap_s": WALL_CAP_S,
            "standing_observation_sim_s": STANDING_SIM_S,
            "standing_observation_wall_cap_s": STANDING_WALL_CAP_S,
            "status": "NOT_STARTED", "attempts": []})
    plan = {
        "schema": "t2-hold-function-check-v1", "batch_id": root.name,
        "classification": CLASSIFICATION,
        "authority": "docs/thesis_project/T2_HOLD_FUNCTION_CHECK_EXECUTOR_PROMPT.md",
        "created_at": now(),
        "versions": smoke.record_versions(),
        "contract": {"groups_order": [{"group": g, "hold_s": h} for g, h in GROUP_ORDER],
                     "entry_threshold_E": E, "single_threshold_exit": E,
                     "cli_exit_threshold_fallback": CLI_EXIT_FALLBACK,
                     "a_cli_hold_fallback_s": A_CLI_HOLD_FALLBACK,
                     "fallback_note": "manual_ab_compare/s1 require X<E and H>0 as CLI validation; A/C overlays do not contain exit_threshold or (A) recovery_min_hold_s",
                     "goal_world_m": list(GOAL), "arrival_radius_m": 0.5,
                     "sim_limit_s": SIM_LIMIT_S, "wall_cap_s": WALL_CAP_S,
                     "standing_observation_sim_s": STANDING_SIM_S,
                     "standing_wall_cap_s": STANDING_WALL_CAP_S},
        "rendering": {"backend": "GLFW", "requested_mode": "hidden",
                      "env": "ABS_MUJOCO_NO_WINDOW=1", "rviz": "start_rviz:=false",
                      "framebuffer_capture": False},
        "budget": {"base_scene_starts": BASE_SCENE_STARTS,
                   "max_scene_starts": MAX_SCENE_STARTS,
                   "max_engineering_repairs": MAX_ENGINEERING_REPAIRS},
        "counters": {"confirmed_scene_starts": 0, "engineering_repairs": 0,
                     "orchestration_corrections": 0},
        "items": items}
    atomic_json(root / PLAN_FILE, plan)
    return plan


def prepare_overlay(root: Path, item: dict) -> Path:
    overlay = root / "overlays" / item_id(item["group"], item["hold_s"])
    hold_arg = item["hold_s"] if item["group"] == "C" else A_CLI_HOLD_FALLBACK
    if not (overlay / "share/go2_description/config/abs/config.yaml").is_file():
        overlay.parent.mkdir(parents=True, exist_ok=True)
        manual.prepare_overlay(SOURCE, overlay, item["group"], E, CLI_EXIT_FALLBACK, hold_arg)
    manual.set_overlay_goal(overlay / "share/go2_description/config/abs/config.yaml", GOAL[0], GOAL[1])
    text = (overlay / "share/go2_description/config/abs/config.yaml").read_text(encoding="utf-8")
    if not manual.overlay_configuration_matches(text, item["group"], E, CLI_EXIT_FALLBACK,
                                                hold_arg, GOAL[0], GOAL[1]):
        raise RuntimeError(f"overlay validation failed for {item['plan_item_id']}")
    block = text[text.index("  switching:"):]
    block = block[:block.index("\n  ", block.index("\n    "))]
    if "exit_threshold" in block:
        raise RuntimeError("A/C overlay unexpectedly contains exit_threshold")
    if item["group"] == "A" and "recovery_min_hold_s" in block:
        raise RuntimeError("A overlay unexpectedly contains recovery_min_hold_s")
    return overlay


def build_command(item: dict, overlay: Path, result_root: Path, args: argparse.Namespace) -> list[str]:
    hold_arg = item["hold_s"] if item["group"] == "C" else A_CLI_HOLD_FALLBACK
    return ["/usr/bin/python3", "-u", str(ROOT / "scripts/manual_ab_compare.py"),
            "--groups", item["group"], "--repeats", "1", "--scene", SCENE,
            "--goal-x", f"{GOAL[0]:g}", "--goal-y", f"{GOAL[1]:g}",
            "--entry-threshold", f"{E:g}", "--exit-threshold", f"{CLI_EXIT_FALLBACK:g}",
            "--hold-s", f"{hold_arg:g}",
            "--sim-limit-s", f"{SIM_LIMIT_S:g}", "--wall-cap-s", f"{WALL_CAP_S:g}",
            "--standing-observation-sim-s", f"{STANDING_SIM_S:g}",
            "--result-root", str(result_root), "--prepared-overlay", str(overlay),
            "--display", args.display, "--xauthority", args.xauthority,
            "--plot-python", args.plot_python,
            "--classification", CLASSIFICATION,
            "--no-window", "--disable-framebuffer-capture"]


def classify(item: dict, found: dict, code: int | None) -> tuple[str, list[str], str | None]:
    metrics = found["metrics"]
    context = found["context"]
    facts = found["process_facts"]
    source = ((context.get("runtime_source_validation") or {}).get("evidence") or {})
    supervisor = metrics.get("supervisor_terminal_result")
    reasons = []
    if (found["run_directory"] is None and not found["mujoco_started"]
            and not metrics and not found["terminal"]):
        return "ORCHESTRATION_FAILURE", [f"runner_exit={code}"], "orchestration"
    if code != 0:
        reasons.append(f"runner_exit={code}")
    if supervisor == "SYSTEM_SAFETY_ABORT":
        return "STOPPED_FAILURE", reasons + ["system_safety_abort"], "safety"
    if metrics.get("record_validity") != "VALID":
        reasons.append(f"record_validity={metrics.get('record_validity')}")
    if (context.get("runtime_source_validation") or {}).get("status") != "confirmed":
        reasons.append("runtime_source_not_confirmed")
    if source.get("group") != item["group"]:
        reasons.append(f"actual_group={source.get('group')}")
    try:
        if abs(float(source.get("entry_threshold")) - E) > 1e-5:
            reasons.append("entry_threshold_mismatch")
        if abs(float(source.get("exit_threshold")) - E) > 1e-5:
            reasons.append("a_c_exit_threshold_not_equal_to_E")
    except (TypeError, ValueError):
        reasons.append("consumed_threshold_missing")
    actual_hold = str(source.get("hold")).lower() in ("true", "1")
    if item["group"] == "A":
        if actual_hold:
            reasons.append("A_hold_enabled")
        if abs(float(source.get("recovery_min_hold_s") or 0.0)) > 1e-9:
            reasons.append("A_nonzero_hold")
    else:
        if not actual_hold:
            reasons.append("C_hold_not_enabled")
        try:
            if abs(float(source.get("recovery_min_hold_s")) - item["hold_s"]) > 1e-5:
                reasons.append("C_hold_value_mismatch")
        except (TypeError, ValueError):
            reasons.append("C_hold_value_missing")
    if metrics.get("s6_preparation_readiness", {}).get("first_active_policy_frame_exactly_persisted") is not True:
        reasons.append("first_active_policy_frame_not_exactly_persisted")
    if facts.get("shutdown_complete") is not True or facts.get("closeout_complete") is not True:
        reasons.append("shutdown_or_closeout_unconfirmed")
    if not found["runtime_record"] or not found["terminal"]:
        reasons.append("raw_runtime_or_terminal_missing")
    if supervisor not in LEGITIMATE_TERMINALS:
        reasons.append(f"terminal={supervisor}")
    if reasons:
        stop = "cleanup" if any("shutdown_or_closeout" in r for r in reasons) else (
            "record" if any(("record_validity" in r or "raw_runtime" in r or "source" in r or
                             "threshold" in r or "hold" in r) for r in reasons) else "infrastructure")
        return "STOPPED_FAILURE", reasons, stop
    return "VALID_TASK_RESULT", [], None


def write_progress(root: Path, plan: dict) -> None:
    rows = []
    for item in plan["items"]:
        last = item["attempts"][-1] if item["attempts"] else {}
        hold = "off" if item["group"] == "A" else f"{item['hold_s']:.2f}"
        rows.append([item["sequence"], item["group"], hold, item["scene"], item.get("status"),
                     last.get("run_id"), last.get("terminal_result"),
                     last.get("record_validity"), last.get("hold_audit_status"),
                     "; ".join(item.get("failure_reasons") or [])])
    with (root / "attempt_ledger.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sequence", "group", "H_s", "scene", "item_status", "run_id",
                         "terminal_result", "record_validity", "hold_audit_status", "failure_reasons"])
        writer.writerows(rows)
    atomic_json(root / "progress.json", {
        "updated_at": now(), "planned": len(plan["items"]),
        "completed": sum(i.get("status") == "VALID_TASK_RESULT" for i in plan["items"]),
        "confirmed_scene_starts": plan["counters"]["confirmed_scene_starts"],
        "items": [{"sequence": i["sequence"], "group": i["group"], "H_s": i["hold_s"],
                   "status": i.get("status")} for i in plan["items"]]})


def read_run(root: Path, result_root: Path, group: str) -> dict:
    batch = load_json(result_root / "batch_plan.json")
    status = next((row for row in batch.get("run_statuses", [])
                   if row.get("group") == group), {})
    run_id = status.get("run_id")
    run_dir = Path(status["output_directory"]) if status.get("output_directory") else None
    if run_dir is not None and not run_dir.is_absolute():
        run_dir = (result_root / run_dir).resolve()
    found = {"batch_plan": batch, "status": status, "run_id": run_id,
             "run_directory": run_dir, "metrics": {}, "context": {}, "terminal": {},
             "process_facts": {}}
    if run_dir is None or not run_dir.is_dir():
        candidates = sorted(result_root.glob(f"{group}/rep_01/*"))
        run_dir = candidates[0] if candidates else None
        found["run_directory"] = run_dir
    for name, key in (("metrics.json", "metrics"), ("run_context.json", "context"),
                      ("terminal.json", "terminal"), ("process_facts.json", "process_facts")):
        if run_dir is not None:
            found[key] = load_json(run_dir / name)
    mujoco_log = (run_dir / "mujoco.log") if run_dir is not None else None
    found["mujoco_started"] = bool(mujoco_log and mujoco_log.is_file()
                                   and "MuJoCo version" in mujoco_log.read_text(encoding="utf-8", errors="replace"))
    found["runtime_record"] = bool(run_dir and (run_dir / "runtime_record.jsonl").is_file())
    return found


def attempt_from_found(item: dict, result_root: Path, found: dict, code: int = 0) -> dict:
    metrics = found["metrics"]
    context = found["context"]
    facts = found["process_facts"]
    source = ((context.get("runtime_source_validation") or {}).get("evidence") or {})
    cycle0 = (context.get("initial_state") or {}).get("first_active_policy_cycle") or {}
    audit = metrics.get("recovery_hold_audit") or {}
    return {
        "attempt_number": 1, "status": "VALID_TASK_RESULT", "started_at": now(),
        "finished_at": now(), "result_root": str(result_root), "classification": CLASSIFICATION,
        "render_mode": "hidden GLFW", "rviz": "disabled", "child_exit_code": code,
        "run_id": found["run_id"],
        "run_directory": str(found["run_directory"]) if found["run_directory"] else None,
        "terminal_result": metrics.get("supervisor_terminal_result"),
        "record_validity": metrics.get("record_validity"),
        "runtime_source_validation": (context.get("runtime_source_validation") or {}).get("status"),
        "actual_group": source.get("group"), "actual_entry_threshold": source.get("entry_threshold"),
        "actual_exit_threshold": source.get("exit_threshold"), "actual_hold": source.get("hold"),
        "actual_recovery_min_hold_s": source.get("recovery_min_hold_s"),
        "hold_audit_status": audit.get("status"),
        "hold_episode_count": len(audit.get("episodes") or []),
        "hold_violation_count": len(audit.get("violations") or []),
        "process_shutdown_complete": facts.get("shutdown_complete"),
        "closeout_complete": facts.get("closeout_complete"), "mujoco_started": found["mujoco_started"],
        "muJoCo_window_log": ((context.get("preflight") or {}).get("mujoco_window_observation") or {}),
        "initial_state": {"status": cycle0.get("status"), "sim_time_s": cycle0.get("sim_time_s"),
                          "world_position_xy_m": cycle0.get("world_position_xy_m"),
                          "world_yaw_rad": cycle0.get("world_yaw_rad")},
        "failure_reasons": []}


def repair_false_orchestration(root: Path, plan: dict) -> None:
    """Recover a valid run that an older read bug mislabeled as no-scene."""
    for item in plan["items"]:
        if item.get("status") != "ORCHESTRATION_FAILURE" or item.get("attempts"):
            continue
        result_root = root / "runs" / item["plan_item_id"] / "result"
        found = read_run(root, result_root, item["group"])
        if (found["run_id"] and found["run_directory"] and found["metrics"]
                and found["terminal"] and found["mujoco_started"]):
            item["attempts"] = [attempt_from_found(item, result_root, found)]
            item["status"] = "VALID_TASK_RESULT"
            item["failure_reasons"] = []
            item["finished_at"] = now()
            item["classification_repair"] = {
                "previous_status": "ORCHESTRATION_FAILURE",
                "reason": "older read_run looked only for group B; the group A/C run was valid and is recovered without rerunning",
                "run_id": found["run_id"], "at": now()}
            plan["counters"]["confirmed_scene_starts"] += 1
            if plan["counters"].get("orchestration_corrections", 0) > 0:
                plan["counters"]["orchestration_corrections"] -= 1
            plan.setdefault("classification_repairs", []).append({
                "sequence": item["sequence"], "plan_item_id": item["plan_item_id"],
                "run_id": found["run_id"], "scene_rerun": False, "at": now()})


def run_item(root: Path, plan: dict, item: dict, args: argparse.Namespace) -> dict:
    if len(item["attempts"]) >= 2:
        raise RuntimeError("per-item scene-start cap exhausted")
    if plan["counters"]["confirmed_scene_starts"] >= MAX_SCENE_STARTS:
        raise RuntimeError("total scene-start cap exhausted")
    overlay = prepare_overlay(root, item)
    item_root = root / "runs" / item["plan_item_id"]
    attempt_number = len(item["attempts"]) + 1
    child = item_root if attempt_number == 1 else item_root / f"attempt_{attempt_number:02d}"
    child.mkdir(parents=True, exist_ok=True)
    result_root = child / "result"
    if result_root.exists():
        raise RuntimeError(f"result root already exists: {result_root}")
    attempt = {"attempt_number": attempt_number, "status": "RUNNING", "started_at": now(),
               "result_root": str(result_root), "classification": CLASSIFICATION,
               "render_mode": "hidden GLFW", "rviz": "disabled",
               "command": build_command(item, overlay, result_root, args)}
    item["attempts"].append(attempt)
    item["status"] = "RUNNING"
    atomic_json(root / PLAN_FILE, plan)
    console = item_root / "console.log" if attempt_number == 1 else child / "console.log"
    code = None
    with console.open("w", encoding="utf-8") as stream:
        proc = subprocess.Popen(build_command(item, overlay, result_root, args), cwd=ROOT,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        try:
            assert proc.stdout is not None
            for line in proc.stdout:
                print(line, end="", flush=True)
                stream.write(line)
                stream.flush()
            code = proc.wait()
        except KeyboardInterrupt:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=150)
            except subprocess.TimeoutExpired as error:
                raise RuntimeError("child cleanup unconfirmed after Ctrl+C") from error
            raise
    found = read_run(root, result_root, item["group"])
    status, reasons, stop_class = classify(item, found, code)
    if status == "ORCHESTRATION_FAILURE":
        plan["counters"]["orchestration_corrections"] += 1
        item["attempts"].pop()
        item["status"] = "ORCHESTRATION_FAILURE"
        item["failure_reasons"] = reasons
        atomic_json(root / PLAN_FILE, plan)
        write_progress(root, plan)
        return {"status": status, "stop_class": stop_class, "reasons": reasons}
    metrics, context, facts = found["metrics"], found["context"], found["process_facts"]
    source = ((context.get("runtime_source_validation") or {}).get("evidence") or {})
    cycle0 = (context.get("initial_state") or {}).get("first_active_policy_cycle") or {}
    hold_audit = metrics.get("recovery_hold_audit") or {}
    attempt.update({
        "finished_at": now(), "child_exit_code": code, "run_id": found["run_id"],
        "run_directory": str(found["run_directory"]) if found["run_directory"] else None,
        "terminal_result": metrics.get("supervisor_terminal_result"),
        "record_validity": metrics.get("record_validity"),
        "runtime_source_validation": (context.get("runtime_source_validation") or {}).get("status"),
        "actual_group": source.get("group"), "actual_entry_threshold": source.get("entry_threshold"),
        "actual_exit_threshold": source.get("exit_threshold"), "actual_hold": source.get("hold"),
        "actual_recovery_min_hold_s": source.get("recovery_min_hold_s"),
        "hold_audit_status": hold_audit.get("status"),
        "hold_episode_count": len(hold_audit.get("episodes") or []),
        "hold_violation_count": len(hold_audit.get("violations") or []),
        "process_shutdown_complete": facts.get("shutdown_complete"),
        "closeout_complete": facts.get("closeout_complete"),
        "mujoco_started": found["mujoco_started"],
        "muJoCo_window_log": ((context.get("preflight") or {}).get("mujoco_window_observation") or {}),
        "initial_state": {"status": cycle0.get("status"), "sim_time_s": cycle0.get("sim_time_s"),
                          "world_position_xy_m": cycle0.get("world_position_xy_m"),
                          "world_yaw_rad": cycle0.get("world_yaw_rad")},
        "status": status, "failure_reasons": reasons})
    if found["mujoco_started"]:
        plan["counters"]["confirmed_scene_starts"] += 1
    item["status"] = status
    item["finished_at"] = attempt["finished_at"]
    if status != "VALID_TASK_RESULT":
        item["failure_reasons"] = reasons
    item["stop_class"] = stop_class
    atomic_json(root / PLAN_FILE, plan)
    write_progress(root, plan)
    print(f"[{item['sequence']}/4] {item['group']} H={item['hold_s']} -> "
          f"{attempt['terminal_result']} ({status}); hold_audit={attempt['hold_audit_status']}; "
          f"record={attempt['record_validity']}; cleanup="
          f"{attempt['process_shutdown_complete']}/{attempt['closeout_complete']}", flush=True)
    return {"status": status, "stop_class": stop_class, "reasons": reasons}


def pending_items(plan: dict, retry_sequence: int | None, retry_reason: str | None) -> list[dict]:
    if retry_sequence is not None:
        matches = [item for item in plan["items"] if item["sequence"] == retry_sequence]
        if len(matches) != 1:
            raise RuntimeError("retry sequence does not identify exactly one item")
        item = matches[0]
        if item.get("status") not in {"ENGINEERING_FAILURE", "STOPPED_FAILURE"}:
            raise RuntimeError("retry requires a previously stopped engineering failure")
        if len(item["attempts"]) >= 2:
            raise RuntimeError("per-item scene-start cap exhausted")
        if not retry_reason:
            raise RuntimeError("retry requires an explicit repair reason")
        item["attempts"][-1]["status"] = "ENGINEERING_FAILURE_REPAIRED"
        item["attempts"][-1]["repair"] = {"reason": retry_reason, "authorized_at": now()}
        plan["counters"]["engineering_repairs"] += 1
        if plan["counters"]["engineering_repairs"] > MAX_ENGINEERING_REPAIRS:
            raise RuntimeError("engineering repair budget exhausted")
        item["status"] = "NOT_STARTED"
        return [item]
    done = {item["sequence"] for item in plan["items"] if item.get("status") == "VALID_TASK_RESULT"}
    pending = [item for item in plan["items"] if item["sequence"] not in done]
    stopped = [item for item in pending
               if item.get("status") in {"ENGINEERING_FAILURE", "STOPPED_FAILURE"}]
    if stopped:
        raise RuntimeError("batch contains a stopped item; explicit retry authorization is required: "
                           + ", ".join(str(item["sequence"]) for item in stopped))
    if [item["sequence"] for item in pending] != sorted(item["sequence"] for item in pending):
        raise RuntimeError("resume would reorder the frozen sequence")
    return pending


def _run_locked(root: Path, args: argparse.Namespace) -> int:
    plan_path = root / PLAN_FILE
    plan = load_json(plan_path) or create_plan(root)
    plan["classification"] = CLASSIFICATION
    repair_false_orchestration(root, plan)
    atomic_json(plan_path, plan)
    pending = pending_items(plan, args.retry_sequence, args.retry_reason)
    if not pending:
        write_status(root, "NO_PENDING_ITEMS")
        return 0
    existing = related_processes()
    if existing:
        plan["preflight_process_inventory_before_scene"] = existing
        atomic_json(plan_path, plan)
        raise RuntimeError("related project process already running; refusing first scene")
    facts = smoke.preflight(root, args)
    plan["preflight"] = facts
    if not facts.get("same_execution_context_as_scene_runs"):
        plan["status"] = "STOPPED_PREFLIGHT"
        atomic_json(plan_path, plan)
        write_status(root, "STOPPED_PREFLIGHT", preflight=facts)
        raise RuntimeError("same-context hidden GLFW preflight failed; no scene started")
    for item in plan["items"]:
        prepare_overlay(root, item)
    plan["status"] = "RUNNING"
    plan["runner_pid"] = os.getpid()
    atomic_json(plan_path, plan)
    (root / "runner.pid").write_text(f"{os.getpid()}\n", encoding="ascii")
    write_status(root, "RUNNING", preflight_status="PASS", renderer=facts.get("renderer"),
                 planned=len(plan["items"]), pending=len(pending))
    for item in pending:
        if (root / "STOP_AFTER_CURRENT_RUN").exists():
            plan["status"] = "STOPPED_USER_REQUEST"
            plan["next_sequence"] = item["sequence"]
            atomic_json(plan_path, plan)
            write_status(root, "STOPPED_USER_REQUEST", next_sequence=item["sequence"])
            return 0
        if related_processes():
            raise RuntimeError("identified project process remained after prior cleanup")
        atomic_json(plan_path, plan)
        print(f"开始第 {item['sequence']}/4 项：{item['group']} "
              f"H={item['hold_s'] if item['hold_s'] is not None else 'off'}", flush=True)
        outcome = run_item(root, plan, item, args)
        if outcome["status"] != "VALID_TASK_RESULT":
            plan["status"] = "STOPPED"
            plan["stop_sequence"] = item["sequence"]
            plan["stop_reason"] = "; ".join(outcome["reasons"])
            plan["stop_class"] = outcome["stop_class"]
            atomic_json(plan_path, plan)
            write_progress(root, plan)
            print(f"STOP at {item['plan_item_id']}: {plan['stop_reason']}", file=sys.stderr, flush=True)
            return 1
    plan["status"] = "RUNS_COMPLETE_PENDING_ANALYSIS"
    atomic_json(plan_path, plan)
    try:
        result = subprocess.run([args.plot_python, str(ROOT / "scripts/analyze_t2_hold_function_check.py"),
                                 "--plan", str(plan_path), "--output-dir", str(root)],
                                cwd=ROOT, capture_output=True, text=True, timeout=300)
        (root / "analysis_stdout.log").write_text(result.stdout + result.stderr, encoding="utf-8")
        if result.returncode != 0:
            raise RuntimeError(f"analysis failed with exit {result.returncode}: {result.stderr[-2000:]}")
        plan["analysis"] = {"status": "COMPLETE", "log": str(root / "analysis_stdout.log")}
    except Exception as error:
        plan["status"] = "ANALYSIS_FAILED_RAW_DATA_PRESERVED"
        plan["analysis_error"] = f"{type(error).__name__}: {error}"
        atomic_json(plan_path, plan)
        raise
    plan["status"] = "COMPLETE"
    plan["finished_at"] = now()
    atomic_json(plan_path, plan)
    write_progress(root, plan)
    write_status(root, "COMPLETE", completed=4,
                 confirmed_scene_starts=plan["counters"]["confirmed_scene_starts"])
    return 0


def run_batch(root: Path, args: argparse.Namespace) -> int:
    lock_stream = (root / "runner.lock").open("a+")
    try:
        fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lock_stream.close()
        raise RuntimeError("another T2 runner owns the batch lock") from error
    try:
        return _run_locked(root, args)
    except KeyboardInterrupt:
        write_status(root, "STOPPED_OPERATOR", reason="keyboard interrupt; child cleanup requested")
        raise
    except Exception as error:
        write_status(root, "FAILED", error=f"{type(error).__name__}: {error}")
        raise
    finally:
        fcntl.flock(lock_stream.fileno(), fcntl.LOCK_UN)
        lock_stream.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-id", default=None)
    parser.add_argument("--root", type=Path, default=None)
    parser.add_argument("--retry-sequence", type=int, default=None)
    parser.add_argument("--retry-reason", default=None)
    parser.add_argument("--plot-python", default="/home/lidio/anaconda3/bin/python3")
    parser.add_argument("--display", default=":0")
    parser.add_argument("--xauthority", default="/run/user/1000/gdm/Xauthority")
    args = parser.parse_args()
    if args.root is not None and args.batch_id is not None:
        parser.error("use either --root or --batch-id")
    if not Path(args.plot_python).is_file():
        parser.error(f"plot python not found: {args.plot_python}")
    if args.root is not None:
        root = args.root.resolve()
    elif args.batch_id:
        root = (PARENT / args.batch_id).resolve()
    else:
        root = PARENT / (datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6])
    if root.exists() and not (root / PLAN_FILE).is_file() and any(root.iterdir()):
        parser.error(f"existing non-empty directory without a plan: {root}")
    root.mkdir(parents=True, exist_ok=True)
    log_stream = (root / "runner.log").open("a", encoding="utf-8", buffering=1)
    sys.stdout = Tee(sys.__stdout__, log_stream)
    sys.stderr = Tee(sys.__stderr__, log_stream)
    print(f"批次目录：{root}", flush=True)
    return run_batch(root, args)


if __name__ == "__main__":
    sys.exit(main())
