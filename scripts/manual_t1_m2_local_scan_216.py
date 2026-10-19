#!/usr/bin/env python3
"""T1 new-M2 18-map x 6 exit-threshold local scan (216 runs, hidden GLFW).

Frozen schedule: two rounds of 108 items. Each round has 18 map blocks with six
adjacent thresholds; round 1 uses the six cyclic shifts of one random base
permutation, round 2 reverses each map's round-1 order. The production chain is
manual_ab_compare.py -> s1_run_diagnostic.py, exactly as in the 18-map smoke run.
"""
from __future__ import annotations

import argparse
import csv
import fcntl
import json
import math
import os
import random
import shutil
import signal
import subprocess
import sys
import uuid
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import manual_ab_compare as manual  # noqa: E402
import manual_t1_m2_smoke_18 as smoke  # noqa: E402
from abs_scene import resolve_scene  # noqa: E402

PARENT = ROOT / "logs/t1/m2_local_scan_216"
LAYOUTS = ROOT / "docs/thesis_project/designs/M2/selected_v2/layouts.json"
SCENE_INDEX = ROOT / "docs/thesis_project/designs/M2/selected_v2/scene_index.csv"
SOURCE = ROOT / "quadruped_ros2_control_humble/install/go2_description"
PLAN_FILE = "m2_local_scan_216_plan.json"
MAP_ORDER = smoke.MAP_ORDER
XS = (-0.07, -0.08, -0.09, -0.10, -0.11, -0.12)
REPEATS = 2
GROUP = "B"
E = -0.05
GOAL = (7.5, 0.0)
SIM_LIMIT_S = 20.0
WALL_CAP_S = 120.0
STANDING_SIM_S = 2.0
STANDING_WALL_CAP_S = 30.0
ARRIVAL_RADIUS_M = 0.5
SCHEDULE_SEED = 2026101801
CLASSIFICATION = "T1 新版M2 18图六档退出阈值局部扫描，非正式实验"
LEGITIMATE_TERMINALS = {"ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION",
                        "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE"}
BASE_SCENE_STARTS = 216
MAX_SCENE_STARTS = 220
MAX_STARTS_PER_ITEM = 2
MAX_ENGINEERING_REPAIRS = 4
MAX_ORCHESTRATION_CORRECTIONS = 2
MAX_BATCH_BYTES = 4 * 1024**3
MIN_FREE_BYTES = 15 * 1024**3

now = smoke.now
sha = smoke.sha
atomic_json = smoke.atomic_json
load_json = smoke.load_json
related_processes = smoke.related_processes
write_status = smoke.write_status
Tee = smoke.Tee


def x_key(x: float) -> str:
    return f"{x:.2f}".replace("-", "m").replace(".", "")


def x_slug(x: float) -> str:
    return f"x{int(round(abs(x) * 100)):02d}"


def make_schedule() -> tuple[list[dict], dict]:
    base = list(XS)
    rng = random.Random(SCHEDULE_SEED)
    rng.shuffle(base)
    shifts = [base[i:] + base[:i] for i in range(6)]
    map_order_1 = list(MAP_ORDER)
    rng.shuffle(map_order_1)
    shift_ids = [index % 6 for index in range(len(MAP_ORDER))]
    rng.shuffle(shift_ids)
    round1_orders = {}
    blocks_1 = []
    for map_name, shift_id in zip(map_order_1, shift_ids):
        order = list(shifts[shift_id])
        round1_orders[map_name] = order
        blocks_1.append({"map": map_name, "shift_id": shift_id, "order": order})
    map_order_2 = list(MAP_ORDER)
    rng.shuffle(map_order_2)
    blocks_2 = [{"map": map_name, "shift_id": None,
                 "order": list(reversed(round1_orders[map_name]))} for map_name in map_order_2]
    schedule_meta = {"seed": SCHEDULE_SEED, "base_permutation": base,
                     "cyclic_shifts": shifts, "round1_map_order": map_order_1,
                     "round1_shift_ids": shift_ids, "round2_map_order": map_order_2}
    blocks = []
    sequence = 0
    for round_index, block_list in ((1, blocks_1), (2, blocks_2)):
        for block_index, block in enumerate(block_list, start=1):
            for position, x in enumerate(block["order"], start=1):
                sequence += 1
                blocks.append({"sequence": sequence, "round": round_index,
                               "repeat": round_index, "block_index": block_index,
                               "position": position, "map": block["map"],
                               "exit_threshold_X": x, "shift_id": block["shift_id"]})
    return blocks, schedule_meta


def verify_schedule(items: list[dict], schedule_meta: dict) -> dict:
    if len(items) != 216:
        raise ValueError("schedule does not contain 216 items")
    keys = [(item["map"], item["repeat"], item["exit_threshold_X"]) for item in items]
    if len(set(keys)) != 216:
        raise ValueError("schedule keys are not unique")
    if len({item["sequence"] for item in items}) != 216:
        raise ValueError("schedule sequences are not unique")
    per_x = Counter(item["exit_threshold_X"] for item in items)
    if set(per_x.values()) != {36}:
        raise ValueError(f"threshold counts are not 36 each: {per_x}")
    cells = Counter((item["map"], item["exit_threshold_X"]) for item in items)
    if len(cells) != 108 or set(cells.values()) != {2}:
        raise ValueError("expected 108 map x threshold cells with 2 repeats each")
    positions = Counter((item["exit_threshold_X"], item["position"]) for item in items)
    if len(positions) != 36 or set(positions.values()) != {6}:
        raise ValueError(f"position balance failed: {positions}")
    round1 = [item for item in items if item["round"] == 1]
    round2 = [item for item in items if item["round"] == 2]
    if len(round1) != 108 or len(round2) != 108:
        raise ValueError("round sizes are not 108/108")
    r1_positions = Counter((item["exit_threshold_X"], item["position"]) for item in round1)
    if set(r1_positions.values()) != {3}:
        raise ValueError("round 1 position balance is not 3 per threshold/position")
    r1_order = defaultdict(list)
    for item in sorted(round1, key=lambda row: (row["block_index"], row["position"])):
        r1_order[item["map"]].append(item["exit_threshold_X"])
    for item in round2:
        expected = list(reversed(r1_order[item["map"]]))
        actual = [row["exit_threshold_X"] for row in round2
                  if row["map"] == item["map"] and row["position"] <= len(expected)]
        if actual != expected:
            raise ValueError(f"round 2 does not reverse round 1 for {item['map']}")
    if len({tuple(order) for order in r1_order.values()}) < 2:
        raise ValueError("round 1 orders are degenerate")
    return {"items": 216, "per_threshold": dict(per_x), "cells": len(cells),
            "position_counts": {f"{k[0]:.2f}/{k[1]}": v for k, v in sorted(positions.items())},
            "round1_position_counts": {f"{k[0]:.2f}/{k[1]}": v for k, v in sorted(r1_positions.items())},
            "round2_reverses_round1": True, "base_permutation": schedule_meta["base_permutation"],
            "shift_assignment_counts": dict(Counter(schedule_meta["round1_shift_ids"]))}


def load_contract() -> dict[str, dict]:
    snapshot = smoke.load_contract()
    return {row["map"]: row for row in snapshot}


def create_plan(root: Path) -> dict:
    contract = load_contract()
    blocks, schedule_meta = make_schedule()
    verification = verify_schedule(blocks, schedule_meta)
    items = []
    for block in blocks:
        base = contract[block["map"]]
        items.append({
            "sequence": block["sequence"], "plan_item_id": f"{block['sequence']:03d}_r{block['round']}_"
                f"{base['scene_id']}_{x_slug(block['exit_threshold_X'])}",
            "round": block["round"], "repeat": block["repeat"],
            "block_index": block["block_index"], "position": block["position"],
            "map": block["map"], "scene_id": base["scene_id"],
            "difficulty_label": base["difficulty_label"],
            "source_set": base["source_set"], "source_filename": base["source_filename"],
            "previous_m2_filename": base.get("previous_m2_filename"),
            "layout_revision": base.get("layout_revision"),
            "field_bounds_m": base["field_bounds_m"], "xml_sha256": base["xml_sha256"],
            "group": GROUP, "entry_threshold": E,
            "exit_threshold": block["exit_threshold_X"], "hold": False,
            "goal_world_m": list(GOAL), "sim_limit_s": SIM_LIMIT_S, "wall_cap_s": WALL_CAP_S,
            "standing_observation_sim_s": STANDING_SIM_S,
            "standing_observation_wall_cap_s": STANDING_WALL_CAP_S,
            "status": "NOT_STARTED", "attempts": []})
    plan = {
        "schema": "t1-m2-local-scan-216-v1", "batch_id": root.name,
        "classification": CLASSIFICATION,
        "authority": "docs/thesis_project/T1_M2_LOCAL_SCAN_216_EXECUTOR_PROMPT.md",
        "frozen_method": "docs/thesis_project/evidence/T1/m2_local_scan_216/FROZEN_METHOD_216.md",
        "created_at": now(),
        "mapset": {"path": str(LAYOUTS.resolve()), "version": "m2-selected-development/2.1",
                   "sha256": sha(LAYOUTS), "scene_index_sha256": sha(SCENE_INDEX)},
        "versions": smoke.record_versions(),
        "contract": {"group": GROUP, "entry_threshold_E": E,
                     "exit_threshold_X": list(XS), "hysteresis": True, "hold_enabled": False,
                     "goal_world_m": list(GOAL), "arrival_radius_m": ARRIVAL_RADIUS_M,
                     "sim_limit_s": SIM_LIMIT_S, "wall_cap_s": WALL_CAP_S,
                     "arrival_standing_observation_sim_s": STANDING_SIM_S,
                     "arrival_standing_wall_cap_s": STANDING_WALL_CAP_S},
        "schedule": {"seed": SCHEDULE_SEED, "meta": schedule_meta,
                     "verification": verification},
        "rendering": {"backend": "GLFW", "requested_mode": "hidden",
                      "env": "ABS_MUJOCO_NO_WINDOW=1", "rviz": "start_rviz:=false",
                      "framebuffer_capture": False,
                      "note": "hidden GLFW still requires the approved X11/OpenGL context; not true headless"},
        "budget": {"base_scene_starts": BASE_SCENE_STARTS, "max_scene_starts": MAX_SCENE_STARTS,
                   "max_starts_per_item": MAX_STARTS_PER_ITEM,
                   "max_engineering_repairs": MAX_ENGINEERING_REPAIRS,
                   "max_orchestration_corrections": MAX_ORCHESTRATION_CORRECTIONS,
                   "max_batch_bytes": MAX_BATCH_BYTES, "min_free_bytes": MIN_FREE_BYTES},
        "counters": {"confirmed_scene_starts": 0, "engineering_repairs": 0,
                     "orchestration_corrections": 0},
        "items": items}
    atomic_json(root / PLAN_FILE, plan)
    return plan


def prepare_overlay(root: Path, x: float) -> Path:
    overlay = root / "overlays" / f"X_{x:.2f}"
    if not (overlay / "share/go2_description/config/abs/config.yaml").is_file():
        overlay.parent.mkdir(parents=True, exist_ok=True)
        manual.prepare_overlay(SOURCE, overlay, GROUP, E, x, 0.30)
    manual.set_overlay_goal(overlay / "share/go2_description/config/abs/config.yaml", GOAL[0], GOAL[1])
    config = overlay / "share/go2_description/config/abs/config.yaml"
    if not manual.overlay_configuration_matches(config.read_text(encoding="utf-8"), GROUP, E,
                                               x, 0.30, GOAL[0], GOAL[1]):
        raise RuntimeError(f"prepared B overlay for X={x:.2f} failed production configuration validation")
    return overlay


def build_command(item: dict, overlay: Path, result_root: Path, args: argparse.Namespace) -> list[str]:
    x = item["exit_threshold"]
    return ["/usr/bin/python3", "-u", str(ROOT / "scripts/manual_ab_compare.py"),
            "--groups", "B", "--repeats", "1", "--scene", item["map"],
            "--goal-x", f"{GOAL[0]:g}", "--goal-y", f"{GOAL[1]:g}",
            "--entry-threshold", f"{E:g}", "--exit-threshold", f"{x:g}",
            "--sim-limit-s", f"{SIM_LIMIT_S:g}", "--wall-cap-s", f"{WALL_CAP_S:g}",
            "--standing-observation-sim-s", f"{STANDING_SIM_S:g}",
            "--result-root", str(result_root), "--prepared-overlay", str(overlay),
            "--display", args.display, "--xauthority", args.xauthority,
            "--plot-python", args.plot_python,
            "--classification", CLASSIFICATION,
            "--no-window", "--disable-framebuffer-capture"]


def classify(item: dict, found: dict, child_code: int | None) -> tuple[str, list[str], str | None]:
    metrics = found["metrics"]
    context = found["context"]
    facts = found["process_facts"]
    source = ((context.get("runtime_source_validation") or {}).get("evidence") or {})
    supervisor = metrics.get("supervisor_terminal_result")
    reasons = []
    if (found["run_directory"] is None and not found["mujoco_started"]
            and not metrics and not found["terminal"]):
        return "ORCHESTRATION_FAILURE", reasons + [f"runner_exit={child_code}"], "orchestration"
    if child_code != 0:
        reasons.append(f"runner_exit={child_code}")
    if not found["mujoco_started"] and not supervisor:
        return "ENGINEERING_FAILURE", reasons + ["no_scene_or_terminal_evidence"], "infrastructure"
    if supervisor == "SYSTEM_SAFETY_ABORT":
        return "STOPPED_FAILURE", reasons + ["system_safety_abort"], "safety"
    if metrics.get("record_validity") != "VALID":
        reasons.append(f"record_validity={metrics.get('record_validity')}")
    if (context.get("runtime_source_validation") or {}).get("status") != "confirmed":
        reasons.append("runtime_source_not_confirmed")
    if source.get("group") != GROUP:
        reasons.append(f"actual_group={source.get('group')}")
    try:
        if abs(float(source.get("entry_threshold")) - E) > 1e-5:
            reasons.append("entry_threshold_mismatch")
        if abs(float(source.get("exit_threshold")) - item["exit_threshold"]) > 1e-5:
            reasons.append("exit_threshold_mismatch")
    except (TypeError, ValueError):
        reasons.append("consumed_threshold_missing")
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
                             "threshold" in r) for r in reasons) else "infrastructure")
        return "STOPPED_FAILURE", reasons, stop
    return "VALID_TASK_RESULT", [], None


def write_progress(root: Path, plan: dict) -> None:
    rows = []
    for item in plan["items"]:
        last = item["attempts"][-1] if item["attempts"] else {}
        rows.append([item["sequence"], item["round"], item["map"], item["difficulty_label"],
                     f"{item['exit_threshold']:.2f}", item.get("status"), last.get("run_id"),
                     last.get("terminal_result"), last.get("record_validity"),
                     "; ".join(item.get("failure_reasons") or [])])
    with (root / "attempt_ledger.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sequence", "round", "map", "difficulty", "X", "item_status",
                         "run_id", "terminal_result", "record_validity", "failure_reasons"])
        writer.writerows(rows)
    atomic_json(root / "progress.json", {
        "updated_at": now(), "planned": len(plan["items"]),
        "completed": sum(item.get("status") == "VALID_TASK_RESULT" for item in plan["items"]),
        "confirmed_scene_starts": plan["counters"]["confirmed_scene_starts"],
        "items": [{"sequence": item["sequence"], "map": item["map"],
                   "X": item["exit_threshold"], "status": item.get("status")}
                  for item in plan["items"]]})


def run_item(root: Path, plan: dict, item: dict, args: argparse.Namespace) -> dict:
    attempt_number = len(item["attempts"]) + 1
    if attempt_number > MAX_STARTS_PER_ITEM:
        raise RuntimeError("per-item scene-start cap exhausted")
    if plan["counters"]["confirmed_scene_starts"] >= MAX_SCENE_STARTS:
        raise RuntimeError("total scene-start cap exhausted")
    overlay = prepare_overlay(root, item["exit_threshold"])
    item_root = root / "runs" / item["plan_item_id"]
    child = item_root if attempt_number == 1 else item_root / f"attempt_{attempt_number:02d}"
    child.mkdir(parents=True, exist_ok=True)
    result_root = child / "result"
    if result_root.exists():
        raise RuntimeError(f"result root already exists; refusing overwrite: {result_root}")
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
    found = smoke.read_run(root, result_root)
    status, reasons, stop_class = classify(item, found, code)
    if status == "ORCHESTRATION_FAILURE":
        plan.setdefault("orchestration_events", []).append({
            "sequence": item["sequence"], "map": item["map"], "X": item["exit_threshold"],
            "at": now(), "child_exit_code": code, "reasons": reasons,
            "classification": "pure orchestration error; no scene created; not counted as a scene start"})
        plan["counters"]["orchestration_corrections"] += 1
        if plan["counters"]["orchestration_corrections"] > MAX_ORCHESTRATION_CORRECTIONS:
            raise RuntimeError("orchestration correction budget exhausted")
        item["attempts"].pop()
        archive_index = len([e for e in plan["orchestration_events"]
                             if e.get("sequence") == item["sequence"]])
        if result_root.exists():
            result_root.rename(child / f"orchestration_result_{archive_index:02d}")
        if console.exists():
            console.rename(child / f"orchestration_console_{archive_index:02d}.log")
        item["status"] = "ORCHESTRATION_FAILURE"
        item["failure_reasons"] = reasons
        atomic_json(root / PLAN_FILE, plan)
        write_progress(root, plan)
        print(f"[{item['sequence']}/216] {item['map']} X={item['exit_threshold']:.2f} -> "
              f"ORCHESTRATION_FAILURE; no scene created", file=sys.stderr, flush=True)
        return {"attempt": None, "status": status, "stop_class": stop_class, "reasons": reasons}
    metrics, context, facts = found["metrics"], found["context"], found["process_facts"]
    source = ((context.get("runtime_source_validation") or {}).get("evidence") or {})
    cycle0 = (context.get("initial_state") or {}).get("first_active_policy_cycle") or {}
    attempt.update({
        "finished_at": now(), "child_exit_code": code,
        "run_id": found["run_id"],
        "run_directory": str(found["run_directory"]) if found["run_directory"] else None,
        "terminal_result": metrics.get("supervisor_terminal_result"),
        "record_validity": metrics.get("record_validity"),
        "runtime_source_validation": (context.get("runtime_source_validation") or {}).get("status"),
        "actual_group": source.get("group"),
        "actual_entry_threshold": source.get("entry_threshold"),
        "actual_exit_threshold": source.get("exit_threshold"),
        "actual_hold": source.get("hold"),
        "process_shutdown_complete": facts.get("shutdown_complete"),
        "closeout_complete": facts.get("closeout_complete"),
        "mujoco_started": found["mujoco_started"],
        "muJoCo_window_log": ((context.get("preflight") or {}).get("mujoco_window_observation") or {}),
        "initial_state": {"status": cycle0.get("status"), "sim_time_s": cycle0.get("sim_time_s"),
                          "world_position_xy_m": cycle0.get("world_position_xy_m"),
                          "world_yaw_rad": cycle0.get("world_yaw_rad"),
                          "same_cycle_runtime_frame_persisted_by_supervisor":
                              cycle0.get("same_cycle_runtime_frame_persisted_by_supervisor")},
        "status": status, "failure_reasons": reasons})
    if found["mujoco_started"]:
        plan["counters"]["confirmed_scene_starts"] += 1
    item["status"] = status
    if status != "VALID_TASK_RESULT":
        item["failure_reasons"] = reasons
    item["stop_class"] = stop_class
    item["finished_at"] = attempt["finished_at"]
    atomic_json(root / PLAN_FILE, plan)
    write_progress(root, plan)
    print(f"[{item['sequence']}/216] {item['map']} X={item['exit_threshold']:.2f} -> "
          f"{attempt['terminal_result']} ({status}); record={attempt['record_validity']}; "
          f"cleanup={attempt['process_shutdown_complete']}/{attempt['closeout_complete']}", flush=True)
    return {"attempt": attempt, "status": status, "stop_class": stop_class, "reasons": reasons}


def pending_items(root: Path, plan: dict, retry_sequence: int | None,
                  retry_reason: str | None) -> list[dict]:
    for item in plan["items"]:
        if item.get("status") != "ENGINEERING_FAILURE" or not item.get("attempts"):
            continue
        last = item["attempts"][-1]
        if last.get("run_id") or last.get("mujoco_started") or last.get("run_directory"):
            continue
        plan.setdefault("orchestration_events", []).append({
            "sequence": item["sequence"], "map": item["map"], "X": item["exit_threshold"],
            "at": now(), "child_exit_code": last.get("child_exit_code"),
            "reasons": last.get("failure_reasons"), "reclassified_from": "ENGINEERING_FAILURE",
            "classification": "pre-scene orchestration error; no scene created; not a scene start"})
        item["attempts"].pop()
        child = root / "runs" / item["plan_item_id"]
        failed_result = Path(last["result_root"]) if last.get("result_root") else child / "result"
        archive_index = len([e for e in plan["orchestration_events"]
                             if e.get("sequence") == item["sequence"]]) - 1
        if failed_result.exists():
            failed_result.rename(failed_result.parent / f"orchestration_result_{archive_index:02d}")
        console = child / "console.log"
        if console.exists():
            console.rename(child / f"orchestration_console_{archive_index:02d}.log")
        item["status"] = "NOT_STARTED"
        item["failure_reasons"] = []
        plan["counters"]["orchestration_corrections"] += 1
    if retry_sequence is not None:
        matches = [item for item in plan["items"] if item["sequence"] == retry_sequence]
        if len(matches) != 1:
            raise RuntimeError("retry sequence does not identify exactly one item")
        item = matches[0]
        if item.get("status") not in {"ENGINEERING_FAILURE", "STOPPED_FAILURE"}:
            raise RuntimeError("retry requires a previously stopped engineering failure")
        if len(item["attempts"]) >= MAX_STARTS_PER_ITEM:
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


def storage_guard(root: Path, plan: dict) -> None:
    usage = sum(path.stat().st_size for path in root.rglob("*") if path.is_file())
    free = shutil.disk_usage(root).free
    plan.setdefault("storage_checkpoints", []).append(
        {"at": now(), "allocated_bytes": usage, "filesystem_free_bytes": free})
    if usage > MAX_BATCH_BYTES or free < MIN_FREE_BYTES:
        raise RuntimeError(f"storage guard tripped: batch={usage} free={free}")


def run_batch(root: Path, args: argparse.Namespace) -> int:
    lock_stream = (root / "runner.lock").open("a+")
    try:
        fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lock_stream.close()
        raise RuntimeError("another 216-scan runner owns the batch lock") from error
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


def _run_locked(root: Path, args: argparse.Namespace) -> int:
    plan_path = root / PLAN_FILE
    plan = load_json(plan_path) or create_plan(root)
    plan["classification"] = CLASSIFICATION
    pending = pending_items(root, plan, args.retry_sequence, args.retry_reason)
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
    for x in XS:
        prepare_overlay(root, x)
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
        storage_guard(root, plan)
        atomic_json(plan_path, plan)
        print(f"开始第 {item['sequence']}/216 项：{item['map']} X={item['exit_threshold']:.2f} "
              f"(round {item['round']} block {item['block_index']} pos {item['position']})", flush=True)
        outcome = run_item(root, plan, item, args)
        if outcome["status"] != "VALID_TASK_RESULT":
            plan["status"] = "STOPPED"
            plan["stop_sequence"] = item["sequence"]
            plan["stop_reason"] = "; ".join(outcome["reasons"])
            plan["stop_class"] = outcome["stop_class"]
            atomic_json(plan_path, plan)
            write_progress(root, plan)
            print(f"STOP at {item['map']} X={item['exit_threshold']:.2f}: "
                  f"{plan['stop_reason']}", file=sys.stderr, flush=True)
            return 1
        if item["sequence"] % 8 == 0:
            storage_guard(root, plan)
            atomic_json(plan_path, plan)
            print(f"进度 {item['sequence']}/216；完成 "
                  f"{plan['counters']['confirmed_scene_starts']} 次场景", flush=True)
    plan["status"] = "RUNS_COMPLETE_PENDING_ANALYSIS"
    atomic_json(plan_path, plan)
    try:
        result = subprocess.run([args.plot_python, str(ROOT / "scripts/analyze_t1_m2_local_scan_216.py"),
                                 "--plan", str(plan_path), "--output-dir", str(root)],
                                cwd=ROOT, capture_output=True, text=True, timeout=900)
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
    write_status(root, "COMPLETE",
                 completed=sum(i.get("status") == "VALID_TASK_RESULT" for i in plan["items"]),
                 confirmed_scene_starts=plan["counters"]["confirmed_scene_starts"])
    return 0


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
