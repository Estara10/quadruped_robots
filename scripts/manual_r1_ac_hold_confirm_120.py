#!/usr/bin/env python3
"""R1 three-map A/C hold-time confirmation (120 hidden-GLFW runs)."""
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
import manual_t2_hold_function_check as t2  # noqa: E402

PARENT = ROOT / "logs/09_R1_switch_constraints/ac_hold_confirm_120"
REPORT_PARENT = ROOT / "docs/thesis_project/evidence/R1/AC_HOLD_CONFIRM_120"
SOURCE = ROOT / "quadruped_ros2_control_humble/install/go2_description"
PLAN_FILE = "run_plan.json"
LEVELS = (("A", None), ("C", 0.02), ("C", 0.10), ("C", 0.12))
MAPS = ("challenge15.xml", "challenge27.xml", "r1_m02_space_middle_v2.xml")
REPEATS = 10
E = -0.05
CLI_EXIT_FALLBACK = -0.10
A_CLI_HOLD_FALLBACK = 0.30
GOAL = (7.5, 0.0)
SIM_LIMIT_S = 20.0
WALL_CAP_S = 120.0
STANDING_SIM_S = 2.0
STANDING_WALL_CAP_S = 30.0
SCHEDULE_SEED = 2026110102
CLASSIFICATION = "R1三张困难开发图A/C保持时间确认，非独立验证"
LEGITIMATE_TERMINALS = {"ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION",
                        "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE"}
BASE_SCENE_STARTS = 120
MAX_SCENE_STARTS = 123
MAX_STARTS_PER_ITEM = 2
MAX_ENGINEERING_REPAIRS = 3
MAX_ORCHESTRATION_CORRECTIONS = 2
MAX_BATCH_BYTES = 2 * 1024**3
MIN_FREE_BYTES = 15 * 1024**3

now = smoke.now
sha = smoke.sha
atomic_json = smoke.atomic_json
load_json = smoke.load_json
related_processes = smoke.related_processes
write_status = smoke.write_status
Tee = smoke.Tee
read_run = t2.read_run
attempt_from_found = t2.attempt_from_found


def level_id(group: str, hold: float | None) -> str:
    return "A" if group == "A" else f"C_H{hold:.2f}"


def make_schedule() -> tuple[list[dict], dict]:
    rng = random.Random(SCHEDULE_SEED)
    base_levels = list(LEVELS)
    rng.shuffle(base_levels)
    shifts = [base_levels[i:] + base_levels[:i] for i in range(len(LEVELS))]
    items = []
    sequence = 0
    round_map_orders = []
    round_positions = {}
    for repeat in range(1, REPEATS + 1):
        map_order = list(MAPS)
        rng.shuffle(map_order)
        round_map_orders.append(map_order)
        for block_index, map_name in enumerate(map_order, start=1):
            offset = (repeat - 1 + MAPS.index(map_name)) % len(LEVELS)
            order = shifts[offset]
            for position, (group, hold) in enumerate(order, start=1):
                sequence += 1
                items.append({"sequence": sequence, "repeat": repeat, "block_index": block_index,
                              "position": position, "map": map_name, "group": group,
                              "hold_s": hold, "shift": offset})
                round_positions[f"{repeat}/{map_name}/{level_id(group, hold)}"] = position
    meta = {"seed": SCHEDULE_SEED, "base_levels": [level_id(g, h) for g, h in base_levels],
            "shifts": [[level_id(g, h) for g, h in shift] for shift in shifts],
            "round_map_orders": round_map_orders, "within_map_positions": round_positions}
    return items, meta


def verify_schedule(items: list[dict], meta: dict) -> dict:
    if len(items) != 120:
        raise ValueError("schedule does not contain 120 items")
    keys = [(i["map"], i["repeat"], level_id(i["group"], i["hold_s"])) for i in items]
    if len(set(keys)) != 120:
        raise ValueError("schedule keys are not unique")
    per_level = Counter(level_id(i["group"], i["hold_s"]) for i in items)
    if set(per_level.values()) != {30}:
        raise ValueError(f"level counts are not 30 each: {per_level}")
    cells = Counter((i["map"], level_id(i["group"], i["hold_s"])) for i in items)
    if len(cells) != 12 or set(cells.values()) != {10}:
        raise ValueError("expected 12 map x setting cells with 10 repeats")
    rounds = Counter(i["repeat"] for i in items)
    if set(rounds) != set(range(1, 11)) or set(rounds.values()) != {12}:
        raise ValueError("expected ten rounds of 12 items")
    for repeat in range(1, 11):
        round_items = [i for i in items if i["repeat"] == repeat]
        if {(i["map"], level_id(i["group"], i["hold_s"])) for i in round_items} != {
                (m, level_id(g, h)) for m in MAPS for g, h in LEVELS}:
            raise ValueError(f"round {repeat} does not contain each map-setting combination once")
    positions = Counter((level_id(i["group"], i["hold_s"]), i["position"]) for i in items)
    return {"items": 120, "per_level": dict(per_level), "cells": len(cells),
            "rounds": {str(r): rounds[r] for r in range(1, 11)},
            "position_counts": {f"{k[0]}/{k[1]}": v for k, v in sorted(positions.items())},
            "base_levels": meta["base_levels"], "shifts": meta["shifts"]}


def create_plan(root: Path) -> dict:
    from abs_scene import resolve_scene
    m3_index = ROOT / "docs/thesis_project/designs/M3/scene_index.csv"
    m3_rows = {}
    with m3_index.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            m3_rows[row["filename"]] = row
    r1_maps = load_json(ROOT / "docs/thesis_project/designs/R1/generated_maps_v2/layouts.json")
    r1_by_name = {row["filename"]: row for row in r1_maps.get("scenes", [])}
    contract = {}
    for map_name in MAPS:
        spec = resolve_scene(map_name)
        if map_name == "r1_m02_space_middle_v2.xml":
            record = r1_by_name[map_name]
            contract[map_name] = {"scene_id": spec.scene_id, "difficulty_label": "middle",
                "source_set": "R1 v2", "source_filename": record["source_v1"],
                "layout_revision": record["layout_revision"],
                "xml_sha256": record["production_scene_binding"]["root_sha256"],
                "field_bounds_m": record["field_bounds_m"]}
        else:
            row = m3_rows[map_name]
            contract[map_name] = {"scene_id": spec.scene_id, "difficulty_label": row["source_difficulty"],
                "source_set": "M3", "source_filename": row["source_filename"],
                "layout_revision": "M3 registered current", "xml_sha256": sha(ROOT / "unitree_mujoco/unitree_robots/go2" / map_name),
                "field_bounds_m": None}
    blocks, meta = make_schedule()
    items = []
    for block in blocks:
        base = contract[block["map"]]
        items.append({
            "sequence": block["sequence"], "plan_item_id": f"{block['sequence']:03d}_r{block['repeat']}_"
                f"{base['scene_id']}_{level_id(block['group'], block['hold_s'])}",
            "repeat": block["repeat"], "block_index": block["block_index"],
            "position": block["position"], "map": block["map"],
            "scene_id": base["scene_id"], "difficulty_label": base["difficulty_label"],
            "source_set": base["source_set"], "source_filename": base["source_filename"],
            "layout_revision": base.get("layout_revision"),
            "field_bounds_m": base["field_bounds_m"], "xml_sha256": base["xml_sha256"],
            "group": block["group"], "hold_s": block["hold_s"],
            "hold_consumed": block["group"] == "C",
            "cli_hold_fallback_s": block["hold_s"] if block["group"] == "C" else A_CLI_HOLD_FALLBACK,
            "cli_exit_fallback": CLI_EXIT_FALLBACK,
            "scene": block["map"], "goal_world_m": list(GOAL), "entry_threshold": E,
            "sim_limit_s": SIM_LIMIT_S, "wall_cap_s": WALL_CAP_S,
            "standing_observation_sim_s": STANDING_SIM_S,
            "standing_observation_wall_cap_s": STANDING_WALL_CAP_S,
            "status": "NOT_STARTED", "attempts": []})
    verification = verify_schedule(items, meta)
    plan = {
        "schema": "r1-ac-hold-confirm-120-v1", "batch_id": root.name,
        "classification": CLASSIFICATION,
        "authority": "user-confirmed R1 three-map A/C hold confirmation batch",
        "created_at": now(), "versions": {**smoke.record_versions(),
            "batch_scripts": {"runner_sha256": sha(ROOT / "scripts/manual_r1_ac_hold_confirm_120.py"),
                              "analyzer_sha256": sha(ROOT / "scripts/analyze_r1_ac_hold_confirm_120.py")}},
        "mapset": {"maps": list(MAPS), "M3_index": str(m3_index),
                   "R1_layouts": str(ROOT / "docs/thesis_project/designs/R1/generated_maps_v2/layouts.json")},
        "contract": {"levels": [{"level": level_id(g, h), "group": g, "hold_s": h}
                                for g, h in LEVELS],
                     "entry_threshold_E": E, "single_threshold_exit": E,
                     "cli_exit_threshold_fallback": CLI_EXIT_FALLBACK,
                     "a_cli_hold_fallback_s": A_CLI_HOLD_FALLBACK,
                     "goal_world_m": list(GOAL), "arrival_radius_m": 0.5,
                     "sim_limit_s": SIM_LIMIT_S, "wall_cap_s": WALL_CAP_S,
                     "standing_observation_sim_s": STANDING_SIM_S,
                     "standing_wall_cap_s": STANDING_WALL_CAP_S},
        "schedule": {"seed": SCHEDULE_SEED, "meta": meta, "verification": verification},
        "rendering": {"backend": "GLFW", "requested_mode": "hidden",
                      "env": "ABS_MUJOCO_NO_WINDOW=1", "rviz": "start_rviz:=false",
                      "framebuffer_capture": False},
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


def prepare_overlay(root: Path, item: dict) -> Path:
    overlay = root / "overlays" / level_id(item["group"], item["hold_s"])
    hold_arg = item["hold_s"] if item["group"] == "C" else A_CLI_HOLD_FALLBACK
    if not (overlay / "share/go2_description/config/abs/config.yaml").is_file():
        overlay.parent.mkdir(parents=True, exist_ok=True)
        manual.prepare_overlay(SOURCE, overlay, item["group"], E, CLI_EXIT_FALLBACK, hold_arg)
    manual.set_overlay_goal(overlay / "share/go2_description/config/abs/config.yaml", GOAL[0], GOAL[1])
    text = (overlay / "share/go2_description/config/abs/config.yaml").read_text(encoding="utf-8")
    if not manual.overlay_configuration_matches(text, item["group"], E, CLI_EXIT_FALLBACK,
                                                hold_arg, GOAL[0], GOAL[1]):
        raise RuntimeError(f"overlay validation failed for {item['plan_item_id']}")
    lines = text.splitlines()
    start = next(i for i, line in enumerate(lines) if line.strip() == "switching:")
    block = [lines[start]]
    for line in lines[start + 1:]:
        if line.startswith("  ") and not line.startswith("    "):
            break
        block.append(line)
    block_text = "\n".join(block)
    if "exit_threshold" in block_text:
        raise RuntimeError("A/C overlay unexpectedly contains exit_threshold")
    if item["group"] == "A" and "recovery_min_hold_s" in block_text:
        raise RuntimeError("A overlay unexpectedly contains recovery_min_hold_s")
    return overlay


def build_command(item: dict, overlay: Path, result_root: Path, args: argparse.Namespace) -> list[str]:
    hold_arg = item["hold_s"] if item["group"] == "C" else A_CLI_HOLD_FALLBACK
    return ["/usr/bin/python3", "-u", str(ROOT / "scripts/manual_ab_compare.py"),
            "--groups", item["group"], "--repeats", "1", "--scene", item["scene"],
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
    status, reasons, stop_class = t2.classify(item, found, code)
    source = ((found["context"].get("runtime_source_validation") or {}).get("evidence") or {})
    if source.get("hysteresis") != "false":
        reasons.append(f"hysteresis_not_disabled:{source.get('hysteresis')}")
        return "STOPPED_FAILURE", reasons, "record"
    return status, reasons, stop_class


def write_progress(root: Path, plan: dict) -> None:
    rows = []
    for item in plan["items"]:
        last = item["attempts"][-1] if item["attempts"] else {}
        rows.append([item["sequence"], item["repeat"], item["map"], item["difficulty_label"],
                     item["group"], item["hold_s"], item.get("status"), last.get("run_id"),
                     last.get("terminal_result"), last.get("record_validity"),
                     last.get("hold_audit_status"), "; ".join(item.get("failure_reasons") or [])])
    with (root / "attempt_ledger.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sequence", "repeat", "map", "difficulty", "group", "H_s",
                         "item_status", "run_id", "terminal_result", "record_validity",
                         "hold_audit_status", "failure_reasons"])
        writer.writerows(rows)
    atomic_json(root / "progress.json", {
        "updated_at": now(), "planned": len(plan["items"]),
        "completed": sum(i.get("status") == "VALID_TASK_RESULT" for i in plan["items"]),
        "confirmed_scene_starts": plan["counters"]["confirmed_scene_starts"],
        "items": [{"sequence": i["sequence"], "map": i["map"],
                   "level": level_id(i["group"], i["hold_s"]), "status": i.get("status")}
                  for i in plan["items"]]})


def run_item(root: Path, plan: dict, item: dict, args: argparse.Namespace) -> dict:
    attempt_number = len(item["attempts"]) + 1
    if attempt_number > MAX_STARTS_PER_ITEM:
        raise RuntimeError("per-item scene-start cap exhausted")
    if plan["counters"]["confirmed_scene_starts"] >= MAX_SCENE_STARTS:
        raise RuntimeError("total scene-start cap exhausted")
    overlay = prepare_overlay(root, item)
    item_root = root / "runs" / item["plan_item_id"]
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
    if status == "VALID_TASK_RESULT" and item["group"] == "C":
        audit = found["metrics"].get("recovery_hold_audit") or {}
        if audit.get("status") == "fail":
            status = "STOPPED_FAILURE"
            reasons = ["hold_audit_fail"] + [v.get("reason", str(v)) for v in audit.get("violations") or []][:5]
            stop_class = "hold"
    if status == "ORCHESTRATION_FAILURE":
        plan.setdefault("orchestration_events", []).append({
            "sequence": item["sequence"], "level": level_id(item["group"], item["hold_s"]),
            "at": now(), "child_exit_code": code, "reasons": reasons,
            "classification": "pure orchestration error; no scene created"})
        plan["counters"]["orchestration_corrections"] += 1
        if plan["counters"]["orchestration_corrections"] > MAX_ORCHESTRATION_CORRECTIONS:
            raise RuntimeError("orchestration correction budget exhausted")
        item["attempts"].pop()
        item["status"] = "ORCHESTRATION_FAILURE"
        item["failure_reasons"] = reasons
        atomic_json(root / PLAN_FILE, plan)
        write_progress(root, plan)
        return {"status": status, "stop_class": stop_class, "reasons": reasons}
    attempt = attempt_from_found(item, result_root, found, code or 0)
    attempt["attempt_number"] = attempt_number
    attempt["status"] = status
    attempt["failure_reasons"] = reasons
    item["attempts"][-1] = attempt
    if found["mujoco_started"]:
        plan["counters"]["confirmed_scene_starts"] += 1
    item["status"] = status
    item["finished_at"] = attempt["finished_at"]
    item["failure_reasons"] = [] if status == "VALID_TASK_RESULT" else reasons
    item["stop_class"] = stop_class
    atomic_json(root / PLAN_FILE, plan)
    write_progress(root, plan)
    print(f"[{item['sequence']}/120] {item['map']} {level_id(item['group'], item['hold_s'])} -> "
          f"{attempt['terminal_result']} ({status}); hold_audit={attempt.get('hold_audit_status')}; "
          f"record={attempt.get('record_validity')}; cleanup="
          f"{attempt.get('process_shutdown_complete')}/{attempt.get('closeout_complete')}", flush=True)
    return {"status": status, "stop_class": stop_class, "reasons": reasons}


def pending_items(root: Path, plan: dict, retry_sequence: int | None,
                  retry_reason: str | None) -> list[dict]:
    for item in plan["items"]:
        if item.get("status") != "ENGINEERING_FAILURE" or not item.get("attempts"):
            continue
        last = item["attempts"][-1]
        if last.get("run_id") or last.get("mujoco_started") or last.get("run_directory"):
            continue
        plan.setdefault("orchestration_events", []).append({
            "sequence": item["sequence"], "level": level_id(item["group"], item["hold_s"]),
            "at": now(), "child_exit_code": last.get("child_exit_code"),
            "reasons": last.get("failure_reasons"), "reclassified_from": "ENGINEERING_FAILURE"})
        item["attempts"].pop()
        item["status"] = "NOT_STARTED"
        item["failure_reasons"] = []
        plan["counters"]["orchestration_corrections"] += 1
    safety_aborts = [item for item in plan["items"]
                     if item.get("status") == "STOPPED_FAILURE"
                     and item.get("attempts")
                     and item["attempts"][-1].get("terminal_result") == "SYSTEM_SAFETY_ABORT"]
    if safety_aborts:
        raise RuntimeError("batch contains a preserved system safety abort; explicit continuation is required")
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
        # Continue through the remaining frozen sequence after the retry item.
        done = {row["sequence"] for row in plan["items"]
                if row.get("status") == "VALID_TASK_RESULT"}
        pending = [row for row in plan["items"] if row["sequence"] not in done]
        stopped = [row for row in pending
                   if row.get("status") in {"ENGINEERING_FAILURE", "STOPPED_FAILURE"}]
        if stopped:
            raise RuntimeError("batch contains another stopped item; retry one item at a time")
        return pending
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
        storage_guard(root, plan)
        atomic_json(plan_path, plan)
        print(f"开始第 {item['sequence']}/120 项：{item['map']} "
              f"{level_id(item['group'], item['hold_s'])} (round {item['repeat']})", flush=True)
        outcome = run_item(root, plan, item, args)
        if outcome["status"] != "VALID_TASK_RESULT":
            plan["status"] = "STOPPED"
            plan["stop_sequence"] = item["sequence"]
            plan["stop_reason"] = "; ".join(outcome["reasons"])
            plan["stop_class"] = outcome["stop_class"]
            atomic_json(plan_path, plan)
            write_progress(root, plan)
            try:
                report_root = REPORT_PARENT / root.name
                report_root.mkdir(parents=True, exist_ok=True)
                result = subprocess.run([args.plot_python, str(ROOT / "scripts/analyze_r1_ac_hold_confirm_120.py"),
                                         "--plan", str(plan_path), "--output-dir", str(report_root)],
                                        cwd=ROOT, capture_output=True, text=True, timeout=1800)
                (root / "analysis_stdout.log").write_text(result.stdout + result.stderr, encoding="utf-8")
                if result.returncode != 0:
                    raise RuntimeError(f"analysis failed with exit {result.returncode}: {result.stderr[-2000:]}")
                plan["analysis"] = {"status": "PARTIAL_COMPLETE", "log": str(root / "analysis_stdout.log")}
                atomic_json(plan_path, plan)
            except Exception as error:
                plan["analysis_error"] = f"{type(error).__name__}: {error}"
                atomic_json(plan_path, plan)
            print(f"STOP at {item['plan_item_id']}: {plan['stop_reason']}", file=sys.stderr, flush=True)
            return 1
        if item["sequence"] % 8 == 0:
            storage_guard(root, plan)
            atomic_json(plan_path, plan)
            print(f"进度 {item['sequence']}/120；场景 "
                  f"{plan['counters']['confirmed_scene_starts']}", flush=True)
    plan["status"] = "RUNS_COMPLETE_PENDING_ANALYSIS"
    atomic_json(plan_path, plan)
    try:
        report_root = REPORT_PARENT / root.name
        report_root.mkdir(parents=True, exist_ok=True)
        result = subprocess.run([args.plot_python, str(ROOT / "scripts/analyze_r1_ac_hold_confirm_120.py"),
                                 "--plan", str(plan_path), "--output-dir", str(report_root)],
                                cwd=ROOT, capture_output=True, text=True, timeout=1800)
        (root / "analysis_stdout.log").write_text(result.stdout + result.stderr, encoding="utf-8")
        if result.returncode != 0:
            raise RuntimeError(f"analysis failed with exit {result.returncode}: {result.stderr[-2000:]}")
        plan["analysis"] = {"status": "COMPLETE", "log": str(root / "analysis_stdout.log")}
    except Exception as error:
        plan["status"] = "ANALYSIS_FAILED_RAW_DATA_PRESERVED"
        plan["analysis_error"] = f"{type(error).__name__}: {error}"
        atomic_json(plan_path, plan)
        raise
    has_preserved_safety_abort = any(
        item.get("status") == "STOPPED_FAILURE" and item.get("attempts")
        and item["attempts"][-1].get("terminal_result") == "SYSTEM_SAFETY_ABORT"
        for item in plan["items"])
    plan["status"] = "COMPLETE_WITH_SYSTEM_SAFETY_ABORT" if has_preserved_safety_abort else "COMPLETE"
    plan["finished_at"] = now()
    atomic_json(plan_path, plan)
    write_progress(root, plan)
    write_status(root, plan["status"],
                 completed=sum(i.get("status") == "VALID_TASK_RESULT" for i in plan["items"]),
                 system_safety_aborts=sum(
                     1 for i in plan["items"] if i.get("status") == "STOPPED_FAILURE"
                     and i.get("attempts") and i["attempts"][-1].get("terminal_result") == "SYSTEM_SAFETY_ABORT"),
                 confirmed_scene_starts=plan["counters"]["confirmed_scene_starts"],
                 attempted=len([i for i in plan["items"] if i.get("attempts")]),
                 pending_not_dispatched=sum(not i.get("attempts") for i in plan["items"]),
                 finished_at=plan["finished_at"])
    return 0


def run_batch(root: Path, args: argparse.Namespace) -> int:
    lock_stream = (root / "runner.lock").open("a+")
    try:
        fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lock_stream.close()
        raise RuntimeError("another R1 AC hold batch runner owns the batch lock") from error
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
        pid_path = root / "runner.pid"
        if pid_path.is_file() and pid_path.read_text(encoding="ascii").strip() == str(os.getpid()):
            pid_path.unlink()
        status = load_json(root / "runner_status.json")
        if status.get("runner_pid") == os.getpid():
            status["runner_pid"] = None
            atomic_json(root / "runner_status.json", status)


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
