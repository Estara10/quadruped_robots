#!/usr/bin/env python3
"""Frozen 6-map x 8-exit-threshold x 3-repeat M2 development batch."""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import os
import random
import shutil
import signal
import subprocess
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import manual_ab_compare as manual  # noqa: E402
import manual_t1_expanded_400 as legacy  # noqa: E402
from abs_scene import resolve_scene  # noqa: E402
from s1_run_diagnostic import overlay_configuration_matches  # noqa: E402

INDEX = ROOT / "docs/thesis_project/designs/M2/candidate_v1/scene_index.csv"
LAYOUTS = ROOT / "docs/thesis_project/designs/M2/candidate_v1/layouts.json"
SOURCE = ROOT / "quadruped_ros2_control_humble/install/go2_description"
MAP_DIR = ROOT / "unitree_mujoco/unitree_robots/go2"
PARENT = ROOT / "logs/t1/m2_exit_round2"
MAPS = tuple(f"m2dev{i:02d}.xml" for i in range(1, 7))
XS = (-0.06, -0.07, -0.08, -0.09, -0.10, -0.11, -0.12, -0.13)
E = -0.05
SCHEDULE_SEED = 2026101602
MAX_BATCH_BYTES = 4 * 1024**3
MIN_FREE_BYTES = 15 * 1024**3
PLAN_NAME = "m2_round2_plan.json"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_json(path: Path, value: object) -> None:
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    os.replace(temp, path)


def load_catalog() -> list[dict]:
    with INDEX.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    layouts = json.loads(LAYOUTS.read_text(encoding="utf-8"))
    by_name = {entry["filename"]: entry for entry in layouts["scenes"]}
    result = []
    for name in MAPS:
        row = next((item for item in rows if item["filename"] == name), None)
        layout = by_name.get(name)
        if row is None or layout is None or layout.get("split") != "development_candidate":
            raise RuntimeError(f"missing or non-development M2 map: {name}")
        scene = resolve_scene(name)
        if scene.scene_id != row["scene_id"] or scene.scene_id != layout["scene_id"]:
            raise RuntimeError(f"scene identity mismatch for {name}")
        if len(scene.scene_id.encode("ascii")) >= 32:
            raise RuntimeError(f"scene ID does not fit production identity field: {name}")
        xml = MAP_DIR / name
        if not xml.is_file():
            raise RuntimeError(f"map XML missing: {xml}")
        result.append({"filename": name, "scene_id": scene.scene_id,
                       "density": layout.get("density", "UNKNOWN"),
                       "route_structure": row["route_structure_label_zh"],
                       "split": layout["split"], "map_seed": int(row["generation_seed"]),
                       "xml_sha256": sha(xml)})
    return result


def make_schedule(catalog: list[dict]) -> tuple[list[dict], list[str]]:
    rng = random.Random(SCHEDULE_SEED)
    runs, screenshots = [], []
    sequence = 0
    for repeat in range(1, 4):
        map_order = list(catalog)
        rng.shuffle(map_order)
        for map_index, scene in enumerate(map_order):
            levels = list(XS)
            rng.shuffle(levels)
            for x in levels:
                sequence += 1
                screenshot = repeat == 1 and map_index == 0
                item_id = f"{sequence:03d}_r{repeat}_{scene['scene_id']}_x{abs(round(x*100)):02d}"
                if screenshot:
                    screenshots.append(item_id)
                runs.append({"sequence": sequence, "plan_item_id": item_id,
                    "map": scene["filename"], "scene_id": scene["scene_id"],
                    "density": scene["density"], "route_structure": scene["route_structure"],
                    "repeat": repeat, "group": "B", "entry_threshold_E": E,
                    "exit_threshold_X": x, "effective_exit_threshold": x,
                    "hysteresis": True, "hold_enabled": False,
                    "screenshot_preselected": screenshot, "status": "NOT_STARTED", "attempts": []})
    return runs, screenshots


def validate_schedule(runs: list[dict], catalog: list[dict]) -> None:
    if len(runs) != 144 or len({run["plan_item_id"] for run in runs}) != 144:
        raise RuntimeError("schedule must contain 144 unique runs")
    if {run["map"] for run in runs} != set(MAPS) or any(run["group"] != "B" for run in runs):
        raise RuntimeError("schedule map/group mismatch")
    for scene in MAPS:
        for x in XS:
            cell = [run for run in runs if run["map"] == scene and
                    math.isclose(run["exit_threshold_X"], x, abs_tol=1e-12)]
            if len(cell) != 3 or {run["repeat"] for run in cell} != {1, 2, 3}:
                raise RuntimeError(f"incomplete map-threshold cell: {scene} {x}")
    regenerated, selected = make_schedule(catalog)
    fields = ("plan_item_id", "map", "scene_id", "repeat", "group", "exit_threshold_X")
    if [[run[field] for field in fields] for run in runs] != [[run[field] for field in fields] for run in regenerated]:
        raise RuntimeError("schedule not reproducible from frozen seed")
    if len(selected) != 8:
        raise RuntimeError("expected eight preselected screenshot runs")


def attempt_accounting(plan: dict) -> dict:
    """Count saved attempt records and distinct wrapper failures; ignore duplicate summaries."""
    attempts = [attempt for item in plan.get("runs", [])
                for attempt in item.get("attempts", [])]
    scene_started = sum(bool(attempt.get("run_id") or attempt.get("scene_started") is True)
                        for attempt in attempts)
    item_no_scene = sum(not (attempt.get("run_id") or attempt.get("scene_started") is True)
                        for attempt in attempts)
    pre_scene = plan.get("pre_scene_start_attempts", [])
    resume_failures = plan.get("resume_preflight_failures", [])
    wrapper_no_scene = len(pre_scene) + len(resume_failures)
    valid_items = sum(item.get("status") == "VALID_TASK_RESULT" and
                      item.get("record_validity") == "VALID" for item in plan.get("runs", []))
    pending = sum(item.get("status") != "VALID_TASK_RESULT" or
                  item.get("record_validity") != "VALID" for item in plan.get("runs", []))
    return {
        "plan_attempt_records": len(attempts),
        "scene_started_attempts": scene_started,
        "item_attempts_without_confirmed_scene_start": item_no_scene,
        "distinct_pre_scene_wrapper_failures": wrapper_no_scene,
        "total_attempts_used": len(attempts) + wrapper_no_scene,
        "valid_plan_items": valid_items,
        "pending_plan_items": pending,
        "duplicate_summary_sources_excluded": ["engineering_attempt_history", "engineering_retry_history"],
    }


def _attempt_scene_state(attempt: dict) -> str:
    """Return CONFIRMED_STARTED, CONFIRMED_NOT_STARTED, or UNKNOWN from launch evidence."""
    explicit = attempt.get("mujoco_scene_start_status") or attempt.get("scene_start_status")
    if explicit in {"CONFIRMED_STARTED", "CONFIRMED_NOT_STARTED", "UNKNOWN"}:
        return explicit
    if attempt.get("scene_started") is True:
        return "CONFIRMED_STARTED"
    run_dir = Path(attempt["run_directory"]) if attempt.get("run_directory") else None
    if run_dir and attempt.get("run_id"):
        context_path = run_dir / "run_context.json"
        facts_path = run_dir / "process_facts.json"
        try:
            context = json.loads(context_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            context = {}
        lifecycle = context.get("lifecycle") or {}
        if lifecycle.get("mujoco_process_started_monotonic_ns") is not None:
            return "CONFIRMED_STARTED"
        try:
            facts = json.loads(facts_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            facts = {}
        if facts.get("mujoco_process_created") is True or facts.get("mujoco_pid"):
            return "CONFIRMED_STARTED"
        if facts.get("mujoco_process_created") is False and facts.get("shutdown_complete") is True:
            return "CONFIRMED_NOT_STARTED"
    if attempt.get("scene_started") is False and attempt.get("child_process_spawned") is False:
        return "CONFIRMED_NOT_STARTED"
    if attempt.get("run_id") or attempt.get("child_process_spawned") is True:
        return "UNKNOWN"
    if attempt.get("scene_started") is False:
        return "CONFIRMED_NOT_STARTED"
    return "UNKNOWN"


def scene_budget_accounting(plan: dict) -> dict:
    """Count MuJoCo scene starts separately from launcher/orchestration calls."""
    attempts = [attempt for item in plan.get("runs", [])
                for attempt in item.get("attempts", [])]
    states = [_attempt_scene_state(attempt) for attempt in attempts]
    orchestration = []
    for event in plan.get("pre_scene_start_attempts", []):
        if event.get("scene_launched") is False or event.get("scene_started") is False:
            orchestration.append(event)
        else:
            orchestration.append({**event, "scene_status": "UNKNOWN"})
    for event in plan.get("resume_preflight_failures", []):
        if event.get("scene_started") is False and event.get("child_process_spawned") is False:
            orchestration.append(event)
        else:
            orchestration.append({**event, "scene_status": "UNKNOWN"})
    started = states.count("CONFIRMED_STARTED")
    unknown = states.count("UNKNOWN") + sum(event.get("scene_status") == "UNKNOWN" for event in orchestration)
    no_scene = states.count("CONFIRMED_NOT_STARTED") + sum(
        not (event.get("scene_status") == "UNKNOWN") for event in orchestration)
    completed = sum(item.get("status") == "VALID_TASK_RESULT" and
                    item.get("record_validity") == "VALID" for item in plan.get("runs", []))
    pending_sequences = [item.get("sequence") for item in plan.get("runs", [])
                         if item.get("status") != "VALID_TASK_RESULT" or item.get("record_validity") != "VALID"]
    return {"confirmed_mujoco_scene_starts": started,
        "scene_starts_unknown_conservatively_reserved": unknown,
        "confirmed_no_scene_orchestration_failures": no_scene,
        "scene_budget_consumed_or_reserved": started + unknown,
        "plan_attempt_records": len(attempts), "completed_valid_plan_items": completed,
        "pending_sequences": pending_sequences, "pending_plan_items": len(pending_sequences),
        "duplicate_summary_sources_excluded": ["engineering_attempt_history", "engineering_retry_history"]}


def validate_scene_budget(plan: dict, *, additional_scene_starts: int = 1) -> dict:
    accounting = scene_budget_accounting(plan)
    cap = int((plan.get("scene_start_budget_policy") or {}).get("max_actual_scene_starts", 0))
    projected = accounting["scene_budget_consumed_or_reserved"] + additional_scene_starts
    if cap <= 0 or projected > cap:
        raise RuntimeError(f"actual MuJoCo scene-start budget exceeded: consumed/reserved="
            f"{accounting['scene_budget_consumed_or_reserved']}, additional={additional_scene_starts}, cap={cap}")
    return {**accounting, "actual_scene_start_cap": cap,
            "projected_scene_starts_after_additional": projected,
            "remaining_scene_start_capacity": cap-accounting["scene_budget_consumed_or_reserved"]}


def classify_scene_start_evidence(result_root: Path, run_directory: Path | None,
                                  batch_status: dict) -> tuple[str, dict]:
    """Resolve scene creation from the diagnostic run's producer-side lifecycle facts."""
    searched = []
    candidates = [run_directory] if run_directory else []
    if result_root.exists():
        candidates.extend(path.parent for path in result_root.rglob("run_context.json"))
    unique = []
    for candidate in candidates:
        if candidate and candidate not in unique:
            unique.append(candidate)
    for directory in unique:
        context_path = directory / "run_context.json"
        facts_path = directory / "process_facts.json"
        searched.append(str(directory))
        try:
            context = json.loads(context_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            context = {}
        lifecycle = context.get("lifecycle") or {}
        if lifecycle.get("mujoco_process_started_monotonic_ns") is not None:
            return "CONFIRMED_STARTED", {"evidence": "run_context.lifecycle.mujoco_process_started_monotonic_ns",
                "run_directory": str(directory), "searched_directories": searched}
        try:
            facts = json.loads(facts_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            facts = {}
        if facts.get("mujoco_process_created") is True or facts.get("mujoco_pid"):
            return "CONFIRMED_STARTED", {"evidence": "process_facts MuJoCo PID/created flag",
                "run_directory": str(directory), "searched_directories": searched}
        if context and facts.get("shutdown_complete") is True:
            return "CONFIRMED_NOT_STARTED", {"evidence": "closed run has no MuJoCo process-start lifecycle event or PID",
                "run_directory": str(directory), "searched_directories": searched}
    if batch_status.get("mujoco_scene_start_status") == "CONFIRMED_NOT_STARTED":
        return "CONFIRMED_NOT_STARTED", {"evidence": "batch status explicit no-scene fact",
            "searched_directories": searched}
    return "UNKNOWN", {"evidence": "no authoritative producer-side MuJoCo start/no-start record",
        "run_id": batch_status.get("run_id"), "status": batch_status.get("status"),
        "searched_directories": searched}


def effective_attempt_limit(plan: dict) -> int:
    amended = plan.get("effective_attempt_limits") or {}
    original = plan.get("attempt_limits") or {}
    return int(amended.get("attempts_total_max", original.get("attempts_total_max", 0)))


def validate_attempt_budget(plan: dict, *, additional_attempts: int = 1) -> dict:
    accounting = attempt_accounting(plan)
    cap = effective_attempt_limit(plan)
    projected = accounting["total_attempts_used"] + additional_attempts
    if cap <= 0 or projected > cap:
        raise RuntimeError(f"attempt budget exceeded: used={accounting['total_attempts_used']}, "
                           f"additional={additional_attempts}, cap={cap}")
    return {**accounting, "effective_attempt_limit": cap,
            "projected_total_after_additional": projected,
            "remaining_capacity_before_additional": cap-accounting["total_attempts_used"]}


def validate_final_139_resume(plan: dict, root: Path, catalog: list[dict], *, allow_fixture_root: bool = False) -> dict:
    """Accept only the valid frozen prefix and a no-scene failure at the next item."""
    if (plan.get("batch_id") != "20261016_174309_9" or root.name != "20261016_174309_9" or
            (not allow_fixture_root and root != (PARENT / "20261016_174309_9").resolve())):
        raise RuntimeError("final resume is restricted to the original M2 batch")
    stop_sequence = plan.get("stop_sequence")
    if (plan.get("status") != "STOPPED" or not isinstance(stop_sequence, int) or
            not 6 <= stop_sequence <= 144):
        raise RuntimeError("final resume requires a stopped pending item in the frozen 006–144 suffix")
    if (plan.get("planned_count") != 144 or plan.get("group") != "B" or
            plan.get("entry_threshold_E") != E or plan.get("exit_thresholds_X") != list(XS) or
            plan.get("hold_enabled") is not False or plan.get("goal_world_m") != [7.5, 0.0] or
            plan.get("arrival_radius_m") != 0.5 or plan.get("sim_limit_s") != 20 or
            plan.get("wall_cap_s") != 120 or plan.get("standing_observation_sim_s") not in (2, 5) or
            plan.get("standing_wall_cap_s") != 30):
        raise RuntimeError("frozen M2 experiment conditions changed")
    limits = plan.get("attempt_limits", {})
    if limits.get("attempts_total_max") != 147 or limits.get("attempts_per_item_max") != 2:
        raise RuntimeError("original 147 limit or per-item limit was rewritten")
    if len(plan.get("runs", [])) != 144 or plan.get("maps") != catalog:
        raise RuntimeError("frozen 144-row schedule or M2 map catalog changed")
    validate_schedule(plan["runs"], catalog)

    for item in plan["runs"][:4]:
        if (item.get("status") != "VALID_TASK_RESULT" or
                item.get("record_validity") != "VALID" or len(item.get("attempts", [])) != 1 or
                not item["attempts"][0].get("run_id")):
            raise RuntimeError("one of the first four valid attempts is missing or changed")
    fifth = plan["runs"][4]
    if (fifth.get("plan_item_id") != "005_r1_m2d06_x11" or
            fifth.get("status") != "VALID_TASK_RESULT" or
            fifth.get("record_validity") != "VALID" or len(fifth.get("attempts", [])) != 2 or
            fifth["attempts"][0].get("status") != "STOPPED_FAILURE" or
            fifth["attempts"][1].get("status") != "VALID_TASK_RESULT" or
            not fifth["attempts"][0].get("run_id") or not fifth["attempts"][1].get("run_id")):
        raise RuntimeError("item 005 failure history or valid attempt_02 is not intact")
    original_dir = Path(fifth["attempts"][0]["run_directory"])
    if not all((original_dir / name).is_file() for name in
               ("run_context.json", "terminal.json", "metrics.json", "process_facts.json")):
        raise RuntimeError("original item 005 failure data is incomplete")

    item6 = plan["runs"][5]
    if (item6.get("sequence") != 6 or item6.get("plan_item_id") != "006_r1_m2d06_x12" or
            len(item6.get("attempts", [])) < 1 or
            item6["attempts"][0].get("status") != "ENGINEERING_FAILURE_REPAIRED" or
            item6["attempts"][0].get("scene_started") is not False or
            item6["attempts"][0].get("run_id") is not None):
        raise RuntimeError("item 006 original no-scene engineering history is not intact")
    if (root / "runs" / item6["plan_item_id"] / "attempt_02").exists():
        if stop_sequence == 6 and len(item6.get("attempts", [])) == 1:
            raise RuntimeError("item 006 attempt_02 already exists without a corresponding attempt record")

    prefix = plan["runs"][:stop_sequence - 1]
    pending = plan["runs"][stop_sequence - 1:]
    if any(item.get("status") != "VALID_TASK_RESULT" or item.get("record_validity") != "VALID"
           for item in prefix):
        raise RuntimeError("a prior plan item is not a preserved valid result")
    current = plan["runs"][stop_sequence - 1]
    if current.get("status") != "STOPPED_FAILURE":
        raise RuntimeError("stopped sequence does not identify the pending failed item")
    if current.get("attempts"):
        if any(_attempt_scene_state(attempt) != "CONFIRMED_NOT_STARTED" for attempt in current["attempts"]):
            raise RuntimeError("current item has a scene start or unknown attempt; no retry is authorized")
        for attempt in current["attempts"]:
            if attempt.get("status") not in {"ENGINEERING_FAILURE_REPAIRED", "STOPPED_FAILURE"}:
                raise RuntimeError("current item contains a non-orchestration attempt state")
    elif not any(event.get("sequence") == stop_sequence and
                 (event.get("scene_started") is False or event.get("scene_launched") is False)
                 for event in plan.get("pre_scene_start_attempts", [])):
        raise RuntimeError("current item has no attempt or confirmed pre-scene failure record")
    if any(item.get("status") != "NOT_STARTED" or item.get("attempts") for item in pending[1:]):
        raise RuntimeError("later plan items are not untouched in frozen order")
    if [item.get("sequence") for item in pending] != list(range(stop_sequence, 145)):
        raise RuntimeError("remaining plan is not the exact frozen suffix")

    accounting = scene_budget_accounting(plan)
    if accounting["scene_starts_unknown_conservatively_reserved"] != 0:
        raise RuntimeError("an earlier scene-start state is unknown; resolve/reserve it before continuing")
    if (accounting["completed_valid_plan_items"] != stop_sequence - 1 or
            accounting["pending_sequences"] != list(range(stop_sequence, 145))):
        raise RuntimeError("valid prefix or exact pending suffix changed")
    policy = plan.get("scene_start_budget_policy") or {}
    if policy.get("max_actual_scene_starts") not in (None, 145):
        raise RuntimeError("actual scene-start cap differs from this authorization")
    historical_amendment = plan.get("authorization_amendments", [])
    if not any(entry.get("previous_total_attempt_limit") == 147 and
               entry.get("revised_effective_total_attempt_limit") == 148 for entry in historical_amendment):
        raise RuntimeError("historical 147-to-148 amendment was not preserved")
    budget_plan = copy.deepcopy(plan)
    budget_plan["scene_start_budget_policy"] = {"max_actual_scene_starts": 145}
    budget = validate_scene_budget(budget_plan, additional_scene_starts=len(pending))
    if budget["projected_scene_starts_after_additional"] != budget["actual_scene_start_cap"]:
        raise RuntimeError("frozen pending suffix does not reconcile with the authorized scene-start cap")
    prior_policy = plan.get("scene_start_budget_policy") or {}
    baseline_failures = prior_policy.get("confirmed_no_scene_failures_at_authorization",
                                         accounting["confirmed_no_scene_orchestration_failures"])
    new_no_scene_failures = accounting["confirmed_no_scene_orchestration_failures"] - baseline_failures
    if new_no_scene_failures < 0 or new_no_scene_failures > 2:
        raise RuntimeError("bounded allowance for new pre-scene orchestration failures is exhausted")
    if new_no_scene_failures:
        cutoff = prior_policy.get("recorded_at", "")
        signatures = []
        for event in [*plan.get("pre_scene_start_attempts", []),
                      *plan.get("resume_preflight_failures", [])]:
            if event.get("occurred_at", "") >= cutoff and event.get("error"):
                signatures.append(str(event["error"]))
        for row in plan.get("runs", []):
            for attempt in row.get("attempts", []):
                if (attempt.get("started_at", "") >= cutoff and
                        _attempt_scene_state(attempt) == "CONFIRMED_NOT_STARTED"):
                    if attempt.get("failure_signature"):
                        signatures.append(str(attempt["failure_signature"]))
                    else:
                        signatures.extend(str(error) for error in attempt.get("failure_reasons", []))
        if len(signatures) != len(set(signatures)):
            raise RuntimeError("the same pre-scene orchestration failure has repeated; refusing re-entry")
    return {"accounting": accounting, "pending_sequences": [x["sequence"] for x in pending],
            "pending_count": len(pending), "actual_scene_start_limit": 145,
            "remaining_actual_scene_starts": len(pending), "stop_sequence": stop_sequence,
            "new_confirmed_no_scene_failures": new_no_scene_failures, "budget": budget,
            "original_005_failure_dir": str(original_dir)}


def validate_item73_repair_and_continue(plan: dict, root: Path, catalog: list[dict], *,
                                        allow_fixture_root: bool = False) -> dict:
    """Allow one isolated retry of 073, then only the untouched 074–144 suffix."""
    if (plan.get("batch_id") != "20261016_174309_9" or root.name != "20261016_174309_9" or
            (not allow_fixture_root and root != (PARENT / "20261016_174309_9").resolve())):
        raise RuntimeError("item-073 repair is restricted to the original M2 batch")
    if (plan.get("status") != "STOPPED" or plan.get("stop_sequence") != 73 or
            plan.get("planned_count") != 144 or plan.get("group") != "B" or
            plan.get("entry_threshold_E") != E or plan.get("exit_thresholds_X") != list(XS) or
            plan.get("hold_enabled") is not False or plan.get("goal_world_m") != [7.5, 0.0] or
            plan.get("arrival_radius_m") != 0.5 or plan.get("sim_limit_s") != 20 or
            plan.get("wall_cap_s") != 120 or plan.get("standing_observation_sim_s") not in (2, 5) or
            plan.get("standing_wall_cap_s") != 30):
        raise RuntimeError("item-073 repair requires the original frozen conditions stopped at 073")
    if len(plan.get("runs", [])) != 144 or plan.get("maps") != catalog:
        raise RuntimeError("frozen schedule or M2 map catalog changed")
    validate_schedule(plan["runs"], catalog)
    if any(item.get("status") != "VALID_TASK_RESULT" or item.get("record_validity") != "VALID"
           for item in plan["runs"][:72]):
        raise RuntimeError("the first 72 valid results are not intact")
    item73 = plan["runs"][72]
    if (item73.get("sequence") != 73 or item73.get("plan_item_id") != "073_r2_m2d01_x12" or
            item73.get("status") != "STOPPED_FAILURE" or len(item73.get("attempts", [])) != 1):
        raise RuntimeError("item 073 is not eligible for exactly one isolated attempt_02")
    original = item73["attempts"][0]
    if (original.get("attempt_number") != 1 or original.get("status") != "STOPPED_FAILURE" or
            not original.get("run_id") or original.get("record_validity") != "INVALID" or
            original.get("terminal_result") != "PREPARATION_FAILURE"):
        raise RuntimeError("original item-073 INVALID preparation failure is not preserved")
    original_dir = Path(original.get("run_directory", ""))
    if not all((original_dir / name).is_file() for name in
               ("run_context.json", "terminal.json", "metrics.json", "process_facts.json")):
        raise RuntimeError("original item-073 raw evidence is incomplete")
    context = json.loads((original_dir / "run_context.json").read_text(encoding="utf-8"))
    prep = ((context.get("initial_state") or {}).get("pre_rl_standing_preparation") or {})
    first = prep.get("first_rejected_sample") or {}
    last = prep.get("last_rejected_sample") or {}
    if not (first.get("physics_diagnostic", {}).get("snapshot_status") == "LIVE" and
            first.get("physics_coverage_complete") == 0 and
            last.get("snapshot_status") == "STALE"):
        raise RuntimeError("original item-073 does not contain the authorized LIVE-coverage-before-STALE evidence")
    if (root / "runs" / item73["plan_item_id"] / "attempt_02").exists():
        raise RuntimeError("item-073 attempt_02 already exists; no second retry is authorized")
    if any(item.get("status") != "NOT_STARTED" or item.get("attempts")
           for item in plan["runs"][73:]):
        raise RuntimeError("items 074–144 are not untouched")
    if [item.get("sequence") for item in plan["runs"][72:]] != list(range(73, 145)):
        raise RuntimeError("pending plan suffix is not exactly 073–144")
    accounting = scene_budget_accounting(plan)
    if (accounting["completed_valid_plan_items"] != 72 or
            accounting["confirmed_mujoco_scene_starts"] != 74 or
            accounting["scene_starts_unknown_conservatively_reserved"] != 0 or
            accounting["pending_sequences"] != list(range(73, 145))):
        raise RuntimeError("scene ledger does not match 72 valid + original 073 failure + untouched 074–144")
    old_policy = plan.get("scene_start_budget_policy") or {}
    if old_policy.get("max_actual_scene_starts") != 145:
        raise RuntimeError("historical 145-start policy is missing or was rewritten")
    budget = copy.deepcopy(plan)
    budget["scene_start_budget_policy"] = {"max_actual_scene_starts": 146}
    budget_facts = validate_scene_budget(budget, additional_scene_starts=72)
    if budget_facts["projected_scene_starts_after_additional"] != 146:
        raise RuntimeError("073 retry and 074–144 do not reconcile with the authorized 146-start cap")
    try:
        validate_scene_budget(budget, additional_scene_starts=73)
    except RuntimeError:
        pass
    else:
        raise RuntimeError("scene budget incorrectly permits a 147th MuJoCo start")
    return {"accounting": accounting, "pending_sequences": list(range(73, 145)),
            "pending_count": 72, "actual_scene_start_limit": 146,
            "budget": budget_facts, "original_item73_run_id": original["run_id"],
            "original_item73_directory": str(original_dir)}


def record_pre_scene_failure(plan: dict, item: dict, error: str) -> dict:
    """Record a distinct orchestration failure without inventing a process or run."""
    event = {"event_id": f"pre_scene_{item['sequence']:03d}_{len(plan.get('pre_scene_start_attempts', [])) + 1:02d}",
        "sequence": item["sequence"], "plan_item_id": item["plan_item_id"],
        "occurred_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "stage": "before_confirmed_scene_start", "scene_started": False,
        "run_id": None, "pid": None, "error": error,
        "failure_signature": hashlib.sha256(error.encode("utf-8")).hexdigest()}
    plan.setdefault("pre_scene_start_attempts", []).append(event)
    return event


def overlay_set(root: Path) -> dict:
    overlays = {}
    for x in XS:
        target = root / "overlays" / f"B_{x:.2f}"
        manual.prepare_overlay(SOURCE, target, "B", E, x, 0.30)
        config = target / "share/go2_description/config/abs/config.yaml"
        manual.set_overlay_goal(config, 7.5, 0.0)
        if not manual.verify_overlay_detached(SOURCE, target):
            raise RuntimeError(f"overlay is not detached: {target}")
        text = config.read_text(encoding="utf-8")
        if not overlay_configuration_matches(text, "B", E, x, 0.30, 7.5, 0.0):
            raise RuntimeError(f"production parser rejected {target}")
        if "recovery_min_hold_s:" in text:
            raise RuntimeError(f"B overlay unexpectedly enables hold: {target}")
        overlays[f"{x:.2f}"] = {"path": str(target.resolve()), "group": "B",
            "entry_threshold_E": E, "exit_threshold_X": x, "hold_enabled": False,
            "config_sha256": sha(config),
            "detached_files": ["config.yaml", "policy.pt", "ra_value.pt", "rec/policy.pt"]}
    return overlays


def code_freeze() -> dict:
    # The production stage helper embeds a fresh audit timestamp on every call;
    # keep stable identity/hash fields but exclude that non-semantic timestamp.
    stage = dict(legacy.coarse.ray_repair_code_stage({}))
    stage.pop("created_at", None)
    library = Path(stage["controller_library_path"])
    mujoco = Path(stage["mujoco_executable_path"])
    model_paths = [SOURCE / "share/go2_description/config/abs/policy.pt",
                   SOURCE / "share/go2_description/config/abs/ra_value.pt",
                   SOURCE / "share/go2_description/config/rec/policy.pt"]
    if not all(path.is_file() for path in model_paths):
        raise RuntimeError("one or more deployed model artifacts are missing")
    tracked = subprocess.run(["git", "diff", "--name-only"], cwd=ROOT, capture_output=True, text=True)
    status = subprocess.run(["git", "status", "--short"], cwd=ROOT, capture_output=True, text=True)
    runtime_ids = ROOT / "docs/thesis_project/designs/M2/candidate_v1/runtime_model_identities.txt"
    return {"controller_code_stage": stage, "controller_library_sha256": sha(library),
        "mujoco_executable_sha256": sha(mujoco),
        "models": {str(path.relative_to(ROOT)): sha(path) for path in model_paths},
        "m2_runtime_model_identities_sha256": sha(runtime_ids),
        "source_code": {rel: sha(ROOT / rel) for rel in
            ("scripts/manual_ab_compare.py", "scripts/s1_run_diagnostic.py",
             "scripts/s1_analyze_run.py", "scripts/manual_t1_m2_round2.py",
             "scripts/score_t1_m2_round2.py", "scripts/m2_glfw_preflight.py")},
        "m2_scene_index_sha256": sha(INDEX), "m2_layouts_sha256": sha(LAYOUTS),
        "git_base": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
            capture_output=True, text=True, check=True).stdout.strip(),
        "tracked_diff_paths_at_freeze": tracked.stdout.splitlines(),
        "worktree_status_at_freeze": status.stdout.splitlines()}


def storage_reference() -> dict:
    old = ROOT / "logs/t1/b_exit_validation_400/20261016_131943_26"
    paths = []
    plan_file = old / "expanded_400_plan.json"
    if plan_file.is_file():
        plan = json.loads(plan_file.read_text(encoding="utf-8"))
        for item in plan.get("runs", []):
            for attempt in item.get("attempts", []):
                if attempt.get("record_validity") == "VALID" and attempt.get("run_directory"):
                    path = Path(attempt["run_directory"])
                    if path.is_dir():
                        paths.append(legacy.allocated_bytes(path))
    if not paths:
        raise RuntimeError("no prior valid raw-run size samples for storage estimate")
    ordered = sorted(paths)
    return {"reference_batch": str(old), "n_valid_run_samples": len(ordered),
        "median_allocated_bytes": int(ordered[len(ordered)//2]),
        "p90_allocated_bytes": int(ordered[int(.9*(len(ordered)-1))])}


def storage_guard(root: Path, remaining: int, reference: dict) -> dict:
    allocated = legacy.allocated_bytes(root)
    free = shutil.disk_usage(root).free
    projected = allocated + remaining * reference["p90_allocated_bytes"] + 8 * 20 * 1024**2
    facts = {"checked_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "batch_allocated_bytes": allocated, "filesystem_free_bytes": free,
        "remaining_planned_runs": remaining, "reference_p90_bytes": reference["p90_allocated_bytes"],
        "projected_final_batch_bytes": projected, "batch_budget_bytes": MAX_BATCH_BYTES,
        "minimum_filesystem_free_bytes": MIN_FREE_BYTES}
    if free < MIN_FREE_BYTES or projected > MAX_BATCH_BYTES:
        raise RuntimeError(f"disk guard failed: free={free}, projected={projected}")
    return facts


def verify_preparation_recovery_freeze(plan: dict, catalog: list[dict], *,
                                       allow_physics_coverage_diagnostic: bool = False) -> dict:
    """Allow documented diagnostic-only revisions while preserving run inputs."""
    baseline = plan.get("code_freeze")
    current = code_freeze()
    if not isinstance(baseline, dict):
        raise RuntimeError("original code freeze is missing")
    coverage_manifest = plan.get("physics_coverage_diagnostic_revision")
    coverage_delta_valid = False
    if allow_physics_coverage_diagnostic:
        sources = {
            "unitree_mujoco/simulate/src/obstacle_collision_authority.h":
                ROOT / "unitree_mujoco/simulate/src/obstacle_collision_authority.h",
            "unitree_mujoco/simulate/src/physics_coverage_diagnostic.h":
                ROOT / "unitree_mujoco/simulate/src/physics_coverage_diagnostic.h",
        }
        coverage_delta_valid = (
            isinstance(coverage_manifest, dict) and
            coverage_manifest.get("baseline_mujoco_executable_sha256") == baseline.get("mujoco_executable_sha256") and
            coverage_manifest.get("current_mujoco_executable_sha256") == current.get("mujoco_executable_sha256") and
            coverage_manifest.get("safety_predicate_changed") is False and
            coverage_manifest.get("scope") == "first physical coverage-loss diagnostic only" and
            coverage_manifest.get("source_sha256") == {key: sha(path) for key, path in sources.items()}
        )
        if not coverage_delta_valid:
            raise RuntimeError("physics coverage diagnostic revision manifest does not match the built producer")
    immutable = ("controller_library_sha256", "models",
                 "m2_runtime_model_identities_sha256", "m2_scene_index_sha256",
                 "m2_layouts_sha256", "git_base")
    for key in immutable:
        if baseline.get(key) != current.get(key):
            raise RuntimeError(f"frozen runtime identity changed: {key}")
    if (baseline.get("mujoco_executable_sha256") != current.get("mujoco_executable_sha256") and
            not coverage_delta_valid):
        raise RuntimeError("MuJoCo executable changed without the exact authorized diagnostic-only manifest")
    allowed_sources = {"scripts/s1_run_diagnostic.py", "scripts/manual_t1_m2_round2.py",
                       "scripts/score_t1_m2_round2.py"}
    old_sources, new_sources = baseline.get("source_code", {}), current.get("source_code", {})
    if set(old_sources) != set(new_sources):
        raise RuntimeError("frozen source inventory changed")
    for key in old_sources:
        if key not in allowed_sources and old_sources[key] != new_sources[key]:
            raise RuntimeError(f"unapproved source change since freeze: {key}")

    old_stage = copy.deepcopy(baseline.get("controller_code_stage", {}))
    new_stage = copy.deepcopy(current.get("controller_code_stage", {}))
    old_stage.pop("stage_id", None)
    new_stage.pop("stage_id", None)
    old_stage_sources = old_stage.pop("source_sha256", {})
    new_stage_sources = new_stage.pop("source_sha256", {})
    old_stage_mujoco = old_stage.pop("mujoco_executable_sha256", None)
    new_stage_mujoco = new_stage.pop("mujoco_executable_sha256", None)
    if coverage_delta_valid and (old_stage_mujoco != baseline.get("mujoco_executable_sha256") or
                                 new_stage_mujoco != current.get("mujoco_executable_sha256")):
        raise RuntimeError("build-stage MuJoCo hashes do not match the diagnostic build manifest")
    if old_stage != new_stage:
        raise RuntimeError("controller build-stage identity changed beyond the diagnostic source allowance")
    stage_allowed_sources = {"scripts/s1_run_diagnostic.py"}
    if coverage_delta_valid:
        stage_allowed_sources.update({
            "unitree_mujoco/simulate/src/obstacle_collision_authority.h",
            "unitree_mujoco/simulate/src/physics_coverage_diagnostic.h"})
    for key in set(old_stage_sources) | set(new_stage_sources):
        if key not in stage_allowed_sources and old_stage_sources.get(key) != new_stage_sources.get(key):
            raise RuntimeError(f"unapproved controller-stage source change: {key}")
    if plan.get("maps") != catalog:
        raise RuntimeError("frozen M2 map catalog/layout identity changed")
    for x in XS:
        overlay = Path(plan["overlays"][f"{x:.2f}"]["path"])
        config = overlay / "share/go2_description/config/abs/config.yaml"
        text = config.read_text(encoding="utf-8")
        if not manual.verify_overlay_detached(SOURCE, overlay):
            raise RuntimeError(f"existing overlay is no longer detached: {overlay}")
        if not overlay_configuration_matches(text, "B", E, x, 0.30, 7.5, 0.0):
            raise RuntimeError(f"existing frozen overlay no longer matches B/X={x}")
        if "recovery_min_hold_s:" in text:
            raise RuntimeError(f"hold unexpectedly enabled in existing B overlay X={x}")
    return {"original_code_freeze_preserved": True,
            "continuation_code_freeze": current,
            "physics_coverage_diagnostic_revision": coverage_manifest if coverage_delta_valid else None,
            "allowed_source_deltas": sorted(allowed_sources),
            "allowed_delta_reasons": {
                "scripts/s1_run_diagnostic.py": "diagnostic fields and exact rejection reasons only; no safety predicate changed",
                "scripts/manual_t1_m2_round2.py": "item-005-only attempt_02 recovery orchestration and continuity checks",
                "scripts/score_t1_m2_round2.py": "analysis-only completeness fix; requires three valid scores per map×threshold cell and uniformly rescores all original/new valid rows",
            },
            "immutable_runtime_inputs_match": True,
            "controller_behavior_changed": False,
            "map_or_model_changed": False,
            "overlay_configs_revalidated": len(XS)}


def current_related_processes() -> list[dict]:
    """Find only identifiable project simulator/controller/runner processes."""
    matches = []
    root_text = str(ROOT)
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            exe = os.readlink(entry / "exe")
            cmdline = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", "replace").strip()
        except OSError:
            continue
        base = Path(exe).name
        related_exe = base in {"unitree_mujoco", "ros2_control_node", "rviz2"}
        related_cmd = ((base == "ros2" and "launch rl_quadruped_controller" in cmdline) or
                       (base.startswith("python") and any(name in cmdline for name in
                        ("scripts/s1_run_diagnostic.py", "scripts/manual_ab_compare.py"))))
        if related_exe or related_cmd:
            matches.append({"pid": int(entry.name), "exe": exe, "cmdline": cmdline[:1200],
                            "exe_under_project": exe.startswith(root_text)})
    return matches


def allocate_attempt_directory(item_root: Path, attempt_number: int, *,
                                create_missing_retry_root: bool = False) -> Path:
    """Create a fresh first-attempt directory or a distinct numbered retry."""
    if attempt_number == 1:
        item_root.mkdir(parents=True, exist_ok=False)
        return item_root
    if not item_root.is_dir():
        if not create_missing_retry_root or item_root.exists():
            raise RuntimeError(f"retry item root is missing: {item_root}")
        item_root.mkdir(parents=False, exist_ok=False)
    child = item_root / f"attempt_{attempt_number:02d}"
    child.mkdir(parents=False, exist_ok=False)
    return child


def run_one(root: Path, plan: dict, item: dict, args: argparse.Namespace,
            *, authorized_item5_retry: bool = False,
            authorized_engineering_retry: bool = False,
            authorized_final_139_resume: bool = False,
            authorized_item73_repair: bool = False,
            popen_factory=None) -> None:
    retry_item = (authorized_item5_retry and item.get("sequence") == 5 and
                  item.get("plan_item_id") == "005_r1_m2d06_x11" and
                  item.get("status") == "STOPPED_FAILURE" and len(item.get("attempts", [])) == 1 and
                  item["attempts"][0].get("attempt_number") == 1)
    repaired_engineering_item = (authorized_engineering_retry and
                   item.get("status") in {"RETRY_AUTHORIZED_ENGINEERING", "STOPPED_FAILURE"} and
                   len(item.get("attempts", [])) == 1 and
                   item["attempts"][0].get("status") == "ENGINEERING_FAILURE_REPAIRED" and
                   item["attempts"][0].get("run_id") is None)
    fresh_item = item.get("status") == "NOT_STARTED" and not item.get("attempts")
    confirmed_no_scene_reentry = (authorized_final_139_resume and
        item.get("sequence", 0) >= 6 and item.get("status") == "STOPPED_FAILURE" and
        bool(item.get("attempts")) and
        all(_attempt_scene_state(attempt) == "CONFIRMED_NOT_STARTED" and
            attempt.get("status") in {"STOPPED_FAILURE", "ENGINEERING_FAILURE_REPAIRED"}
            for attempt in item["attempts"]))
    stopped_before_attempt = (authorized_final_139_resume and
        item.get("sequence") == plan.get("stop_sequence") and
        item.get("status") == "STOPPED_FAILURE" and not item.get("attempts") and
        any(event.get("sequence") == item.get("sequence") and
            (event.get("scene_started") is False or event.get("scene_launched") is False)
            for event in plan.get("pre_scene_start_attempts", [])))
    final_resume_item = (authorized_final_139_resume and item.get("sequence", 0) >= 6 and
                         (fresh_item or repaired_engineering_item or confirmed_no_scene_reentry or
                          stopped_before_attempt))
    item73_retry = (authorized_item73_repair and item.get("sequence") == 73 and
                    item.get("plan_item_id") == "073_r2_m2d01_x12" and
                    item.get("status") == "STOPPED_FAILURE" and len(item.get("attempts", [])) == 1 and
                    item["attempts"][0].get("run_id") and
                    item["attempts"][0].get("record_validity") == "INVALID")
    item73_continue_item = (authorized_item73_repair and item.get("sequence", 0) >= 74 and fresh_item)
    if not fresh_item and not retry_item and not repaired_engineering_item and not confirmed_no_scene_reentry and not stopped_before_attempt:
        if not item73_retry:
            raise RuntimeError(f"refusing duplicate or nonfresh run: {item['plan_item_id']}")
    if authorized_final_139_resume and not final_resume_item:
        raise RuntimeError(f"final 139 resume refuses non-authorized item: {item['plan_item_id']}")
    if authorized_item73_repair and not (item73_retry or item73_continue_item):
        raise RuntimeError(f"item-073 repair mode refuses non-authorized item: {item['plan_item_id']}")
    validate_scene_budget(plan, additional_scene_starts=1)
    expected_freeze = plan.get("continuation_code_freeze", plan["code_freeze"])
    if expected_freeze != code_freeze():
        raise RuntimeError("runtime/controller/model/map inputs changed after freeze")
    x = float(item["exit_threshold_X"])
    overlay = Path(plan["overlays"][f"{x:.2f}"]["path"])
    config = overlay / "share/go2_description/config/abs/config.yaml"
    if not overlay_configuration_matches(config.read_text(encoding="utf-8"), "B", E, x, 0.30, 7.5, 0.0):
        raise RuntimeError(f"frozen production config mismatch: {item['plan_item_id']}")
    upcoming_not_started = sum(row["status"] == "NOT_STARTED" for row in plan["runs"])
    guard = storage_guard(root, upcoming_not_started +
                          (1 if (retry_item or authorized_engineering_retry or item73_retry) else 0),
                          plan["storage_reference"])
    plan.setdefault("storage_checks", []).append(guard)
    item_root = root / "runs" / item["plan_item_id"]
    attempt_number = len(item.get("attempts", [])) + 1
    child = allocate_attempt_directory(item_root, attempt_number,
        create_missing_retry_root=bool(authorized_final_139_resume and repaired_engineering_item))
    result_root = child / "result"
    attempt = {"attempt_number": attempt_number, "status": "RUNNING",
        "started_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "result_root": str(result_root), "screenshot_preselected": item["screenshot_preselected"],
        "result_root_isolated_from_prior_attempts": attempt_number > 1,
        "scene_started": False, "scene_start_status": "UNKNOWN",
        "child_process_spawned": False, "child_pid": None,
        "scene_start_budget_before_launch": validate_scene_budget(plan, additional_scene_starts=1)}
    item["attempts"].append(attempt)
    item["status"] = "RUNNING"
    atomic_json(root / PLAN_NAME, plan)
    command = ["/usr/bin/python3", "-u", str(ROOT / "scripts/manual_ab_compare.py"),
        "--groups", "B", "--repeats", "1", "--scene", item["map"],
        "--goal-x", "7.5", "--goal-y", "0", "--sim-limit-s", "20", "--wall-cap-s", "120",
        "--standing-observation-sim-s", str(plan["standing_observation_sim_s"]),
        "--entry-threshold", "-0.05", "--exit-threshold", format(x, ".17g"), "--hold-s", "0.30",
        "--classification", "T1 M2 second-round B exit-threshold development; nonformal",
        "--display", args.display, "--xauthority", args.xauthority, "--plot-python", args.plot_python,
        "--result-root", str(result_root), "--prepared-overlay", str(overlay)]
    if not item["screenshot_preselected"]:
        command.append("--disable-framebuffer-capture")
    attempt["command"] = command
    log_path = child / "console.log"
    exit_code = None
    with log_path.open("w", encoding="utf-8") as log:
        factory = popen_factory or subprocess.Popen
        try:
            proc = factory(command, cwd=ROOT, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1)
        except Exception as error:
            attempt.update({"child_process_spawned": False,
                "scene_started": False, "scene_start_status": "CONFIRMED_NOT_STARTED",
                "scene_start_evidence": "orchestrator Popen raised before creating the diagnostic wrapper process",
                "failure_signature": hashlib.sha256(str(error).encode("utf-8")).hexdigest()})
            atomic_json(root / PLAN_NAME, plan)
            raise
        attempt.update({"child_process_spawned": True, "child_pid": proc.pid,
                       "child_spawned_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                       "scene_start_status": "UNKNOWN"})
        atomic_json(root / PLAN_NAME, plan)
        try:
            assert proc.stdout is not None
            for line in proc.stdout:
                print(line, end="", flush=True)
                log.write(line); log.flush()
            exit_code = proc.wait()
        except KeyboardInterrupt:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=90)
            except subprocess.TimeoutExpired as exc:
                raise RuntimeError("child cleanup not confirmed after interrupt") from exc
            raise
    batch_path = result_root / "batch_plan.json"
    batch = json.loads(batch_path.read_text(encoding="utf-8")) if batch_path.is_file() else {}
    status = next((entry for entry in batch.get("run_statuses", []) if entry.get("group") == "B"), {})
    run_id = status.get("run_id")
    run_dir = Path(status["output_directory"]).resolve() if status.get("output_directory") else None
    scene_state, scene_evidence = classify_scene_start_evidence(result_root, run_dir, status)
    attempt["scene_start_status"] = scene_state
    attempt["scene_started"] = scene_state == "CONFIRMED_STARTED"
    attempt["scene_start_evidence"] = scene_evidence
    metrics, context, facts = {}, {}, {}
    if run_dir:
        for name, target in (("metrics.json", metrics), ("run_context.json", context), ("process_facts.json", facts)):
            path = run_dir / name
            if path.is_file():
                try:
                    target.update(json.loads(path.read_text(encoding="utf-8")))
                except (OSError, json.JSONDecodeError):
                    pass
    source_validation = context.get("runtime_source_validation") or {}
    evidence = source_validation.get("evidence") or {}
    supervisor = metrics.get("supervisor_terminal_result") or status.get("result") or context.get("terminal_result")
    readiness = metrics.get("s6_preparation_readiness") or {}
    attempt.update({"finished_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "child_exit_code": exit_code, "run_id": run_id, "run_directory": str(run_dir) if run_dir else None,
        "metrics_path": str(run_dir / "metrics.json") if run_dir else None,
        "terminal_result": supervisor, "record_validity": metrics.get("record_validity"),
        "source_validation": source_validation.get("status"), "actual_group": evidence.get("group"),
        "actual_entry_threshold": evidence.get("entry_threshold"),
        "actual_exit_threshold": evidence.get("exit_threshold"),
        "process_shutdown_complete": facts.get("shutdown_complete", metrics.get("process_shutdown_complete")),
        "closeout_complete": facts.get("closeout_complete", metrics.get("closeout_complete")),
        "cycle0_frame_saved": readiness.get("first_active_policy_frame_exactly_persisted"),
        "failure_or_unknown_reasons": []})
    item.update({"run_id": run_id, "terminal_result": supervisor,
        "record_validity": metrics.get("record_validity"), "run_directory": str(run_dir) if run_dir else None})
    failures = []
    if exit_code != 0: failures.append(f"runner_exit={exit_code}")
    if metrics.get("record_validity") != "VALID": failures.append(f"record_validity={metrics.get('record_validity')}")
    if source_validation.get("status") != "confirmed": failures.append("runtime_source_not_confirmed")
    if evidence.get("group") != "B": failures.append(f"actual_group={evidence.get('group')}")
    try:
        if abs(float(evidence.get("entry_threshold")) - E) > 1e-5 or abs(float(evidence.get("exit_threshold")) - x) > 1e-5:
            failures.append("consumed_threshold_mismatch")
    except (TypeError, ValueError):
        failures.append("consumed_threshold_missing")
    if attempt["process_shutdown_complete"] is not True or attempt["closeout_complete"] is not True:
        failures.append("shutdown_or_closeout_unconfirmed")
    if readiness.get("first_active_policy_frame_exactly_persisted") is not True:
        failures.append("cycle0_or_first_active_frame_not_exactly_persisted")
    if supervisor == "SYSTEM_SAFETY_ABORT":
        failures.append("system_safety_abort_requires_review_before_continuation")
    if not run_dir or not (run_dir / "runtime_record.jsonl").is_file() or not (run_dir / "terminal.json").is_file():
        failures.append("raw_runtime_or_terminal_missing")
    attempt["failure_reasons"] = failures
    attempt["status"] = "STOPPED_FAILURE" if failures else "VALID_TASK_RESULT"
    item["status"] = "STOPPED_FAILURE" if failures else "VALID_TASK_RESULT"
    if failures:
        item["failure_reasons"] = failures
    else:
        item.pop("failure_reasons", None)
    atomic_json(root / PLAN_NAME, plan)
    if failures:
        raise RuntimeError(f"{item['plan_item_id']} stopped: {'; '.join(failures)}")
    print(f"M2 run {item['sequence']}/144: {item['map']} B X={x:.2f} -> {supervisor}; source, record, closeout confirmed.", flush=True)


def write_rollups(root: Path, plan: dict) -> None:
    rows = legacy.all_metrics(root, plan)
    for row in rows:
        row["task_ra_min"] = None
        row["task_ra_max"] = None
        run_dir = Path(row["run_directory"]) if row.get("run_directory") else None
        metrics_path = run_dir / "metrics.json" if run_dir else None
        record_path = run_dir / "runtime_record.jsonl" if run_dir else None
        if metrics_path and record_path and metrics_path.is_file() and record_path.is_file():
            metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
            boundary = metrics.get("motion_metric_time_boundary") or {}
            start, end = boundary.get("start_cycle"), boundary.get("end_cycle")
            session = metrics.get("session_id")
            values = []
            if isinstance(start, int) and isinstance(end, int) and isinstance(session, int):
                with record_path.open(encoding="utf-8") as frames:
                    for line in frames:
                        try:
                            entry = json.loads(line)
                        except json.JSONDecodeError:
                            continue
                        payload = entry.get("payload") or {}
                        cycle = payload.get("rl_step")
                        value = payload.get("ra_value")
                        if (entry.get("kind") == "frame" and entry.get("status") == "LIVE" and
                                payload.get("session_id") == session and payload.get("rl_active") == 1 and
                                isinstance(cycle, int) and start <= cycle <= end and
                                isinstance(value, (int, float)) and math.isfinite(value)):
                            values.append(float(value))
            if values:
                row["task_ra_min"] = min(values)
                row["task_ra_max"] = max(values)
    result_fields = list(legacy.CSV_FIELDS) + ["task_ra_min", "task_ra_max"]
    with (root / "m2_run_results_144.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=result_fields, extrasaction="ignore")
        writer.writeheader(); writer.writerows(rows)
    attempts = []
    for item in plan["runs"]:
        for attempt in item.get("attempts", []):
            attempts.append({"sequence": item["sequence"], "plan_item_id": item["plan_item_id"],
                "map": item["map"], "scene_id": item["scene_id"], "repeat": item["repeat"],
                "group": "B", "entry_threshold_E": E, "exit_threshold_X": item["exit_threshold_X"], **attempt})
    fields = sorted({key for row in attempts for key in row}) or ["sequence", "plan_item_id"]
    with (root / "m2_attempt_results.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader(); writer.writerows(attempts)
    accounting = attempt_accounting(plan)
    scene_accounting = scene_budget_accounting(plan)
    summary = {"status": plan["status"], "planned": 144, "attempts": len(attempts),
        "attempt_accounting": accounting,
        "scene_budget_accounting": scene_accounting,
        "scene_start_budget_policy": plan.get("scene_start_budget_policy"),
        "effective_attempt_limit": effective_attempt_limit(plan),
        "original_attempt_limit": (plan.get("attempt_limits") or {}).get("attempts_total_max"),
        "valid_task_records": sum(row["status"] == "VALID_TASK_RESULT" and row["record_validity"] == "VALID" for row in rows),
        "completed_valid": sum(row["status"] == "VALID_TASK_RESULT" and row["record_validity"] == "VALID" for row in rows),
        "not_started": sum(row["status"] == "NOT_STARTED" for row in rows),
        "terminal_valid": dict(Counter(row["terminal_result"] for row in rows if row["status"] == "VALID_TASK_RESULT" and row["record_validity"] == "VALID")),
        "terminal_all_rows": dict(Counter(row["terminal_result"] or "NOT_RUN" for row in rows)),
        "batch_allocated_bytes": legacy.allocated_bytes(root),
        "filesystem_free_bytes": shutil.disk_usage(root).free}
    atomic_json(root / "m2_partial_summary.json", summary)


def main(argv=None, *, test_hooks=None) -> int:
    test_hooks = test_hooks or {}
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--batch-root", type=Path)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--authorized-item-005-recovery-and-continue", action="store_true",
                        help="explicitly apply the user-authorized single retry for original M2 item 005, then continue its suffix")
    parser.add_argument("--authorized-repaired-engineering-item-and-continue", action="store_true",
                        help="explicitly retry only the frozen next item after one recorded and repaired pre-launch orchestration failure")
    parser.add_argument("--authorized-final-139-resume", action="store_true",
                        help="apply the explicit 147-to-148 amendment and run only original items 006–144 once each")
    parser.add_argument("--authorized-item-073-repair-and-completion", action="store_true",
                        help="one isolated retry of item 073, then untouched items 074–144; hard cap 146 actual scene starts")
    parser.add_argument("--display", default=os.environ.get("DISPLAY", ":0"))
    parser.add_argument("--xauthority", default=os.environ.get("XAUTHORITY", "/run/user/1000/gdm/Xauthority"))
    parser.add_argument("--plot-python", default="/home/lidio/anaconda3/bin/python3")
    args = parser.parse_args(argv)
    catalog = load_catalog()
    if args.batch_root:
        if not args.resume:
            parser.error("existing batch requires --resume")
        root = args.batch_root.resolve()
        plan = json.loads((root / PLAN_NAME).read_text(encoding="utf-8"))
        recovery_mode = args.authorized_item_005_recovery_and_continue
        engineering_mode = args.authorized_repaired_engineering_item_and_continue
        final_resume_mode = args.authorized_final_139_resume
        item73_repair_mode = args.authorized_item_073_repair_and_completion
        if sum((recovery_mode, engineering_mode, final_resume_mode, item73_repair_mode)) > 1:
            raise RuntimeError("multiple explicit recovery modes are not allowed")
        preflight_name = ("glfw_preflight_recovery.json" if recovery_mode else
                          "glfw_preflight_scene_budget_resume.json" if final_resume_mode else
                          "glfw_preflight_item73_repair.json" if item73_repair_mode else
                          "glfw_preflight_engineering_retry.json" if engineering_mode else
                          "glfw_preflight.json")
        preflight_path = root / preflight_name
        if not preflight_path.is_file():
            raise RuntimeError("same-context GLFW preflight record is missing")
        preflight = json.loads(preflight_path.read_text(encoding="utf-8"))
        expected_env = {"DISPLAY": args.display, "XAUTHORITY": args.xauthority, "MUJOCO_GL": "glfw"}
        if preflight.get("status") != "PASS" or any((preflight.get("environment") or {}).get(k) != v for k, v in expected_env.items()):
            raise RuntimeError("GLFW preflight failed or did not use the batch display environment")
        if recovery_mode or engineering_mode or final_resume_mode or item73_repair_mode:
            inventory_path = root / ("recovery_process_inventory.json" if recovery_mode else
                                     "scene_budget_resume_process_inventory.json" if final_resume_mode else
                                     "item73_repair_process_inventory.json" if item73_repair_mode else
                                     "engineering_retry_process_inventory.json")
            inventory = {"checked_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                         "scope": "exact identified M2/MuJoCo/ROS/controller/recorder/runner project processes",
                         "matches": current_related_processes()}
            atomic_json(inventory_path, inventory)
        else:
            inventory_path = root / "preflight_process_inventory.json"
            if not inventory_path.is_file():
                raise RuntimeError("task-scoped preflight process inventory is missing")
            inventory = json.loads(inventory_path.read_text(encoding="utf-8"))
        if inventory.get("matches"):
            raise RuntimeError("preflight found existing related project processes; refusing launch")
        preflight_key = ("glfw_preflight_recovery" if recovery_mode else
                         "glfw_preflight_scene_budget_resume" if final_resume_mode else
                         "glfw_preflight_item73_repair" if item73_repair_mode else "glfw_preflight")
        plan[preflight_key] = {"record": str(preflight_path), "status": preflight["status"],
            "environment": preflight.get("environment"), "vendor": preflight.get("vendor"),
            "renderer": preflight.get("renderer"), "version": preflight.get("version"),
            "same_parent_context_as_batch": True,
            "process_inventory": str(inventory_path), "related_processes_before_launch": inventory["matches"]}
        atomic_json(root / PLAN_NAME, plan)
        validate_schedule(plan["runs"], catalog)
        if args.authorized_item_005_recovery_and_continue:
            if root != (PARENT / "20261016_174309_9").resolve() or plan.get("batch_id") != "20261016_174309_9":
                raise RuntimeError("authorized recovery is restricted to the original 20261016_174309_9 batch")
            if plan.get("status") != "STOPPED" or plan.get("stop_sequence") != 5:
                raise RuntimeError("recovery requires the original batch stopped at plan item 005")
            if (plan.get("planned_count") != 144 or plan.get("group") != "B" or
                    plan.get("entry_threshold_E") != E or plan.get("exit_thresholds_X") != list(XS) or
                    plan.get("hold_enabled") is not False or plan.get("goal_world_m") != [7.5, 0.0] or
                    plan.get("arrival_radius_m") != 0.5 or plan.get("sim_limit_s") != 20 or
                    plan.get("wall_cap_s") != 120 or plan.get("standing_observation_sim_s") not in (2, 5) or
                    plan.get("standing_wall_cap_s") != 30):
                raise RuntimeError("original frozen experiment conditions do not match the authorized matrix")
            if plan.get("attempt_limits", {}).get("attempts_total_max") != 147 or plan.get("attempt_limits", {}).get("attempts_per_item_max") != 2:
                raise RuntimeError("original attempt limits changed")
            if len(plan["runs"]) != 144 or plan["maps"] != catalog:
                raise RuntimeError("original 144-row map catalog or schedule changed")
            if any(item["status"] != "VALID_TASK_RESULT" or len(item.get("attempts", [])) != 1 or
                   item.get("record_validity") != "VALID" for item in plan["runs"][:4]):
                raise RuntimeError("the first four valid records are not intact")
            fifth = plan["runs"][4]
            if (fifth["plan_item_id"] != "005_r1_m2d06_x11" or fifth["status"] != "STOPPED_FAILURE" or
                    len(fifth.get("attempts", [])) != 1 or fifth["attempts"][0].get("attempt_number") != 1):
                raise RuntimeError("original item 005 is not eligible for its one authorized attempt_02")
            original_dir = Path(fifth["attempts"][0]["run_directory"])
            if not all((original_dir / name).is_file() for name in
                       ("run_context.json", "terminal.json", "metrics.json", "process_facts.json")):
                raise RuntimeError("original failed item 005 evidence is incomplete; no overwrite/repair allowed")
            suffix = plan["runs"][5:]
            if any(item["status"] != "NOT_STARTED" or item.get("attempts") for item in suffix):
                raise RuntimeError("items 006–144 are not cleanly unstarted")
            if 1 + sum(len(item.get("attempts", [])) for item in plan["runs"]) != 6:
                raise RuntimeError("historical attempt tally changed; expected 5 scene attempts + 1 wrapper attempt")
            freeze_delta = verify_preparation_recovery_freeze(plan, catalog)
            if inventory["matches"]:
                raise RuntimeError("current recovery preflight found existing related project processes")
            plan["preparation_failure_recovery"] = {
                "authorized_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                "pre_attempt_accounting": {"scene_attempts": 5, "wrapper_no_scene_attempts": 1,
                    "total_attempts_used": 6, "remaining_total_attempt_capacity": 141},
                "attempt_02_only_for_plan_sequence": 5,
                "original_attempt_directory_preserved": str(original_dir),
                "resume_gate": "all immutable plan/scene/model/controller/config inputs matched; only preparation diagnostic fields and explicit recovery orchestration changed",
                "code_freeze_delta": freeze_delta,
                "glfw_preflight_record": str(preflight_path),
                "fresh_process_inventory": str(inventory_path),
            }
            plan["continuation_code_freeze"] = freeze_delta["continuation_code_freeze"]
            pending = [fifth, *suffix]
        elif final_resume_mode:
            resume_facts = validate_final_139_resume(
                plan, root, catalog, allow_fixture_root=bool(test_hooks.get("allow_fixture_root")))
            freeze_delta = verify_preparation_recovery_freeze(plan, catalog)
            if inventory["matches"]:
                raise RuntimeError("final-resume preflight found existing related project processes")
            policy = plan.get("scene_start_budget_policy")
            if policy:
                if (policy.get("source") != "docs/thesis_project/T1_M2_SCENE_BUDGET_AND_RESUME_PROMPT.md" or
                        policy.get("max_actual_scene_starts") != 145 or
                        policy.get("authorized_plan_sequences") != list(range(6, 145))):
                    raise RuntimeError("saved scene-start authorization conflicts with this task")
            else:
                policy = {"source": "docs/thesis_project/T1_M2_SCENE_BUDGET_AND_RESUME_PROMPT.md",
                    "recorded_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                    "max_actual_scene_starts": 145,
                    "confirmed_scene_starts_at_authorization": resume_facts["accounting"]["confirmed_mujoco_scene_starts"],
                    "confirmed_no_scene_failures_at_authorization": resume_facts["accounting"]["confirmed_no_scene_orchestration_failures"],
                    "remaining_authorized_scene_starts": len(resume_facts["pending_sequences"]),
                    "authorized_plan_sequences": list(range(6, 145)),
                    "one_scene_start_per_plan_item": True, "additional_scene_retries": 0,
                    "max_new_confirmed_pre_scene_orchestration_failures": 2,
                    "same_pre_scene_failure_reentry_limit": 1,
                    "pre_scene_orchestration_failures_consume_scene_budget": False,
                    "unknown_scene_start_state_consumes_reserved_capacity": True,
                    "historical_total_call_limits_preserved": [147, 148],
                    "replaces_historical_total_call_cap_for_this_scope": True}
                plan.setdefault("scene_start_budget_policy_history", []).append(policy)
            plan["scene_start_budget_policy"] = policy
            plan["scene_budget_accounting_at_resume"] = resume_facts["accounting"]
            plan["scene_budget_authorization"] = {
                "pending_sequences": resume_facts["pending_sequences"],
                "pending_count": resume_facts["pending_count"],
                "actual_scene_start_limit": 145,
                "remaining_actual_scene_starts": len(resume_facts["pending_sequences"]),
                "code_freeze_delta": freeze_delta,
                "glfw_preflight_record": str(preflight_path),
                "fresh_process_inventory": str(inventory_path)}
            plan["continuation_code_freeze"] = freeze_delta["continuation_code_freeze"]
            pending = [plan["runs"][sequence - 1] for sequence in resume_facts["pending_sequences"]]
        elif item73_repair_mode:
            if root != (PARENT / "20261016_174309_9").resolve() or plan.get("batch_id") != "20261016_174309_9":
                raise RuntimeError("item-073 repair is restricted to original batch 20261016_174309_9")
            resume_facts = validate_item73_repair_and_continue(
                plan, root, catalog, allow_fixture_root=bool(test_hooks.get("allow_fixture_root")))
            if inventory["matches"]:
                raise RuntimeError("item-073 repair preflight found existing related project processes")
            coverage_paths = {
                "unitree_mujoco/simulate/src/obstacle_collision_authority.h":
                    ROOT / "unitree_mujoco/simulate/src/obstacle_collision_authority.h",
                "unitree_mujoco/simulate/src/physics_coverage_diagnostic.h":
                    ROOT / "unitree_mujoco/simulate/src/physics_coverage_diagnostic.h",
            }
            plan["physics_coverage_diagnostic_revision"] = {
                "scope": "first physical coverage-loss diagnostic only",
                "baseline_mujoco_executable_sha256": plan["code_freeze"]["mujoco_executable_sha256"],
                "current_mujoco_executable_sha256": code_freeze()["mujoco_executable_sha256"],
                "source_sha256": {key: sha(path) for key, path in coverage_paths.items()},
                "safety_predicate_changed": False,
                "predicate": "first step==1 and sim_time==timestep; subsequent step==previous+1 and sim_time delta==timestep, tolerance 1e-9",
            }
            freeze_delta = verify_preparation_recovery_freeze(
                plan, catalog, allow_physics_coverage_diagnostic=True)
            old_policy = copy.deepcopy(plan.get("scene_start_budget_policy") or {})
            if old_policy.get("max_actual_scene_starts") != 145:
                raise RuntimeError("the historical 145 actual scene-start policy is missing or changed")
            plan.setdefault("scene_start_budget_policy_history", []).append(old_policy)
            policy = {**old_policy,
                "source": "docs/thesis_project/T1_M2_ITEM73_REPAIR_AND_COMPLETION_PROMPT.md",
                "recorded_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                "max_actual_scene_starts": 146,
                "confirmed_scene_starts_at_authorization": resume_facts["accounting"]["confirmed_mujoco_scene_starts"],
                "remaining_authorized_scene_starts": 72,
                "authorized_plan_sequences": list(range(73, 145)),
                "one_scene_start_for_item073_attempt_02_then_items074_144_once": True,
                "additional_scene_retries": 0,
                "unknown_scene_start_state_consumes_reserved_capacity": True,
                "old_145_limit_preserved_in_history": True,
                "pre_scene_failure_policy": "stop immediately; no additional re-entry authorized"}
            plan["scene_start_budget_policy"] = policy
            used = attempt_accounting(plan)["total_attempts_used"]
            amended_attempt_limit = used + 72
            plan.setdefault("effective_attempt_limit_history", []).append({
                "recorded_at": policy["recorded_at"],
                "previous_effective_total_attempt_limit": effective_attempt_limit(plan),
                "revised_effective_total_attempt_limit": amended_attempt_limit,
                "reason": "exactly 72 authorized scene attempts remain; no additional retry or relaunch is authorized"})
            plan["effective_attempt_limits"] = {**(plan.get("effective_attempt_limits") or {}),
                "attempts_total_max": amended_attempt_limit, "attempts_per_item_max": 2,
                "authorized_scene_starts_remaining": 72,
                "amendment_source": "docs/thesis_project/T1_M2_ITEM73_REPAIR_AND_COMPLETION_PROMPT.md"}
            plan["scene_budget_accounting_at_resume"] = resume_facts["accounting"]
            plan["scene_budget_authorization"] = {"pending_sequences": resume_facts["pending_sequences"],
                "pending_count": 72, "actual_scene_start_limit": 146,
                "remaining_actual_scene_starts": 72,
                "projected_scene_starts_at_completion": resume_facts["budget"]["projected_scene_starts_after_additional"],
                "original_item073_failure_run_id": resume_facts["original_item73_run_id"],
                "code_freeze_delta": freeze_delta,
                "glfw_preflight_record": str(preflight_path),
                "fresh_process_inventory": str(inventory_path)}
            plan["continuation_code_freeze"] = freeze_delta["continuation_code_freeze"]
            pending = [plan["runs"][sequence - 1] for sequence in resume_facts["pending_sequences"]]
        else:
            if plan.get("status") not in {"FROZEN", "RUNNING", "STOPPED_OPERATOR"}:
                raise RuntimeError(f"batch not resumable: {plan.get('status')}")
            if plan.get("code_freeze") != code_freeze():
                raise RuntimeError("frozen code/runtime inputs changed")
            if any(item["status"] not in {"NOT_STARTED", "VALID_TASK_RESULT"} for item in plan["runs"]):
                raise RuntimeError("batch has unresolved failure; automatic continuation forbidden")
            pending = [item for item in plan["runs"] if item["status"] == "NOT_STARTED"]
    elif (args.authorized_item_005_recovery_and_continue or args.authorized_repaired_engineering_item_and_continue or
          args.authorized_final_139_resume or args.authorized_item_073_repair_and_completion):
        parser.error("explicit recovery requires --batch-root and --resume")
    else:
        if args.resume:
            parser.error("--resume requires --batch-root")
        PARENT.mkdir(parents=True, exist_ok=True)
        root = PARENT / (datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + str(os.getpid()))
        root.mkdir(parents=False)
        (root / "runs").mkdir(); (root / "overlays").mkdir()
        catalog = load_catalog()
        runs, screenshots = make_schedule(catalog)
        validate_schedule(runs, catalog)
        overlays = overlay_set(root)
        code = code_freeze()
        reference = storage_reference()
        plan = {"schema": "t1-m2-exit-round2-v1", "status": "FROZEN",
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "batch_id": root.name, "classification": "M2 development parameter batch; nonformal",
            "maps": catalog, "runs": runs, "planned_count": 144, "group": "B",
            "entry_threshold_E": E, "exit_thresholds_X": list(XS), "hold_enabled": False,
            "goal_world_m": [7.5, 0.0], "arrival_radius_m": 0.5, "sim_limit_s": 20,
            "wall_cap_s": 120, "standing_observation_sim_s": 2, "standing_wall_cap_s": 30,
            "schedule_seed": SCHEDULE_SEED,
            "schedule_rule": "3 rounds; shuffle 6 maps each round and 8 X values within each map-round",
            "screenshot_preselected_item_ids": screenshots, "screenshot_runs_max": 8,
            "screenshot_images_max": 16, "video": False,
            "storage_budget_bytes": MAX_BATCH_BYTES, "minimum_free_bytes": MIN_FREE_BYTES,
            "storage_reference": reference, "code_freeze": code, "overlays": overlays,
            "scoring": {"weights": {"arrival": 0.4, "no_terminal_collision": 0.3,
                "efficiency": 0.2, "stability": 0.1}, "time_cap_s": 20,
                "short_reverse_reference_per_min": 257.8,
                "formula": "100*(.40*arrival + .30*no_terminal_collision + .20*efficiency + .10/(1+f/257.8))",
                "fixed_before_M2_runs": True, "bootstrap_seed": 2026101603, "bootstrap_replicates": 10000},
            "attempt_limits": {"planned": 144, "engineering_attempts_extra": 3,
                "attempts_total_max": 147, "attempts_per_item_max": 2},
            "preflight_process_inventory": {"checked_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                "scope": "only this task's identified project processes"}}
        atomic_json(root / PLAN_NAME, plan)
        if args.prepare_only:
            print(f"Prepared frozen M2 144 plan: {root}; no simulation started.")
            return 0
        pending = plan["runs"]
    plan["status"] = "RUNNING"
    atomic_json(root / PLAN_NAME, plan)
    write_rollups(root, plan)
    for index, item in enumerate(pending, 1):
        attempt_count_before = len(item.get("attempts", []))
        try:
            run_one(root, plan, item, args,
                    authorized_item5_retry=bool(args.authorized_item_005_recovery_and_continue and item["sequence"] == 5),
                    authorized_engineering_retry=bool((args.authorized_repaired_engineering_item_and_continue or args.authorized_final_139_resume) and item["sequence"] == 6),
                    authorized_final_139_resume=bool(args.authorized_final_139_resume),
                    authorized_item73_repair=bool(args.authorized_item_073_repair_and_completion),
                    popen_factory=test_hooks.get("popen_factory"))
            if args.authorized_item_073_repair_and_completion and item["sequence"] == 73:
                plan["item073_repair_result"] = {"attempt_02_valid": True,
                    "original_failed_run_id": "bd6502665c9a40ac832c7d971e19214f",
                    "completed_at": datetime.now().astimezone().isoformat(timespec="seconds")}
                plan.pop("stop_sequence", None)
                plan.pop("stop_reason", None)
                plan["status"] = "RUNNING"
                atomic_json(root / PLAN_NAME, plan)
        except KeyboardInterrupt:
            plan["status"] = "STOPPED_OPERATOR"
            atomic_json(root / PLAN_NAME, plan)
            write_rollups(root, plan)
            return 130
        except Exception as error:
            if item.get("attempts") and item["attempts"][-1].get("status") == "RUNNING":
                item["attempts"][-1].update({"status": "STOPPED_FAILURE", "failure_reasons": [str(error)],
                    "finished_at": datetime.now().astimezone().isoformat(timespec="seconds")})
                if _attempt_scene_state(item["attempts"][-1]) == "CONFIRMED_NOT_STARTED":
                    item["attempts"][-1]["failure_signature"] = hashlib.sha256(
                        str(error).encode("utf-8")).hexdigest()
                item["status"] = "STOPPED_FAILURE"
                item["failure_reasons"] = [str(error)]
            elif len(item.get("attempts", [])) == attempt_count_before:
                record_pre_scene_failure(plan, item, str(error))
                item["status"] = "STOPPED_FAILURE"
                item["failure_reasons"] = [str(error)]
            plan["status"] = "STOPPED"
            plan["stop_sequence"] = item["sequence"]
            plan["stop_reason"] = str(error)
            plan["attempt_accounting_at_stop"] = attempt_accounting(plan)
            plan["scene_budget_accounting_at_stop"] = scene_budget_accounting(plan)
            atomic_json(root / PLAN_NAME, plan)
            write_rollups(root, plan)
            print(f"STOP at {item['plan_item_id']}: {error}", file=sys.stderr, flush=True)
            return 1
        write_rollups(root, plan)
        if item["sequence"] % 8 == 0:
            usage = legacy.allocated_bytes(root)
            free = shutil.disk_usage(root).free
            plan.setdefault("eight_run_checkpoints", []).append({
                "sequence": item["sequence"], "allocated_bytes": usage,
                "filesystem_free_bytes": free, "checked_at": datetime.now().astimezone().isoformat(timespec="seconds")})
            atomic_json(root / PLAN_NAME, plan)
            print(f"进度 {item['sequence']}/144；批次占用 {usage/1024**3:.2f} GiB，可用 {free/1024**3:.1f} GiB。", flush=True)
            if usage > MAX_BATCH_BYTES or free < MIN_FREE_BYTES:
                plan["status"] = "STOPPED_DISK_GUARD"
                plan["stop_sequence"] = item["sequence"]
                plan["stop_reason"] = "8-run disk guard exceeded"
                atomic_json(root / PLAN_NAME, plan)
                return 2
        if item["sequence"] % 48 == 0:
            plan.setdefault("round_checkpoints", []).append({"sequence": item["sequence"],
                "round": item["sequence"] // 48, "completed_at": datetime.now().astimezone().isoformat(timespec="seconds")})
            atomic_json(root / PLAN_NAME, plan)
    plan["status"] = "RUNS_COMPLETE_PENDING_ANALYSIS"
    plan["finished_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
    atomic_json(root / PLAN_NAME, plan)
    write_rollups(root, plan)
    print(f"All 144 planned runs completed: {root}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
