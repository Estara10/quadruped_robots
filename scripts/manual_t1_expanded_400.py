#!/usr/bin/env python3
"""Frozen 10-map x 8-threshold x 5-repeat T1 development batch.

This runner delegates every actual run to manual_ab_compare.py and the
production diagnostic/analyzer chain. Plan state is compact and atomically
replaced after each item; run payloads and metrics stay in their own folders.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import random
import shutil
import signal
import statistics
import subprocess
import sys
import time
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import manual_ab_compare as manual  # noqa: E402
import manual_s6_12_coarse as coarse  # noqa: E402
from abs_scene import resolve_scene  # noqa: E402
from s1_run_diagnostic import overlay_configuration_matches  # noqa: E402

INDEX = ROOT / "docs/thesis_project/designs/M1/library_v1/scene_index.csv"
SOURCE = ROOT / "quadruped_ros2_control_humble/install/go2_description"
PARENT = ROOT / "logs/t1/b_exit_validation_400"
SEED = 20261016
BOOTSTRAP_SEED = 20261017
E = -0.05
X_LEVELS = (-0.05, -0.07, -0.09, -0.11, -0.13, -0.15, -0.17, -0.19)
MAP_NAMES = ("dev01.xml", "dev03.xml", "dev05.xml", "dev09.xml", "dev11.xml",
             "dev15.xml", "dev17.xml", "dev23.xml", "dev27.xml", "dev29.xml")
MAX_BATCH_BYTES = 10 * 1024**3
MIN_FREE_BYTES = 15 * 1024**3
MAX_ENGINEERING_RETRIES = 5


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_json(path: Path, value: object) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    os.replace(tmp, path)


def allocated_bytes(path: Path) -> int:
    total = 0
    if path.is_file():
        return path.stat().st_blocks * 512
    for base, dirs, files in os.walk(path):
        for name in files:
            try:
                total += (Path(base) / name).lstat().st_blocks * 512
            except FileNotFoundError:
                continue
    return total


def scene_catalog() -> list[dict]:
    with INDEX.open(newline="", encoding="utf-8") as f:
        by_name = {r["filename"]: r for r in csv.DictReader(f)}
    result = []
    for filename in MAP_NAMES:
        row = by_name.get(filename)
        if not row or row.get("split") != "development":
            raise RuntimeError(f"required development map missing/misclassified: {filename}")
        spec = resolve_scene(filename)
        if spec.scene_id != row.get("scene_id"):
            raise RuntimeError(f"scene registry identity mismatch: {filename}")
        if (not math.isclose(float(row["goal_x"]), 7.5, abs_tol=1e-9) or
                not math.isclose(float(row["goal_y"]), 0.0, abs_tol=1e-9) or
                float(row["sim_limit_s"]) != 20 or float(row["wall_cap_s"]) != 120):
            raise RuntimeError(f"frozen map goal/limits mismatch: {filename}")
        result.append({"filename": filename, "scene_id": spec.scene_id,
                       "density": row["density"], "route_structure": row["route_structure"],
                       "split": row["split"], "map_seed": row["seed"]})
    return result


def make_schedule(catalog: list[dict]) -> tuple[list[dict], list[str]]:
    rng = random.Random(SEED)
    runs, selected_screenshots = [], []
    seq = 0
    for repeat in range(1, 6):
        map_order = list(catalog)
        rng.shuffle(map_order)
        for map_index, m in enumerate(map_order):
            levels = list(X_LEVELS)
            rng.shuffle(levels)
            for x in levels:
                seq += 1
                group = "A" if x == -0.05 else "B"
                item_id = f"{seq:03d}_r{repeat:02d}_{m['scene_id']}_x{abs(round(x*100)):02d}"
                screenshot = repeat == 1 and map_index == 0
                if screenshot:
                    selected_screenshots.append(item_id)
                runs.append({"sequence": seq, "plan_item_id": item_id,
                             "map": m["filename"], "scene_id": m["scene_id"],
                             "density": m["density"], "route_structure": m["route_structure"],
                             "repeat": repeat, "group": group,
                             "entry_threshold_E": E, "exit_threshold_X": x,
                             "effective_exit_threshold": E if group == "A" else x,
                             "hysteresis": group == "B", "hold_enabled": False,
                             "screenshot_preselected": screenshot,
                             "status": "NOT_STARTED", "attempts": []})
    return runs, selected_screenshots


def validate_schedule(runs: list[dict], catalog: list[dict]) -> None:
    if len(runs) != 400 or len({r["plan_item_id"] for r in runs}) != 400:
        raise RuntimeError("expanded plan must contain 400 unique rows")
    if sum(r["group"] == "A" for r in runs) != 50 or sum(r["group"] == "B" for r in runs) != 350:
        raise RuntimeError("wrong A/B allocation")
    expected_maps = {m["filename"] for m in catalog}
    if {r["map"] for r in runs} != expected_maps:
        raise RuntimeError("plan includes unapproved or missing maps")
    for repeat in range(1, 6):
        block = [r for r in runs if r["repeat"] == repeat]
        if len(block) != 80:
            raise RuntimeError(f"repeat {repeat} must contain 80 rows")
        for m in expected_maps:
            rows = [r for r in block if r["map"] == m]
            if len(rows) != 8 or {r["exit_threshold_X"] for r in rows} != set(X_LEVELS):
                raise RuntimeError(f"incomplete map/threshold block: repeat={repeat}, map={m}")
    regenerated, selected = make_schedule(catalog)
    keys = ("plan_item_id", "map", "scene_id", "repeat", "group", "exit_threshold_X")
    if [[r[k] for k in keys] for r in runs] != [[r[k] for k in keys] for r in regenerated]:
        raise RuntimeError("schedule differs from frozen seed 20261016")
    if selected != [r["plan_item_id"] for r in runs if r["screenshot_preselected"]] or len(selected) != 8:
        raise RuntimeError("screenshot selection must be the 8 settings of first map in round 1")


def code_snapshot(plan: dict) -> dict:
    stage = coarse.ray_repair_code_stage(plan)
    library = Path(stage["controller_library_path"])
    mujoco = Path(stage["mujoco_executable_path"])
    model_paths = [SOURCE / "share/go2_description/config/abs/policy.pt",
                   SOURCE / "share/go2_description/config/abs/ra_value.pt",
                   SOURCE / "share/go2_description/config/rec/policy.pt"]
    if any(not p.is_file() for p in model_paths):
        raise RuntimeError("installed Agile/RA/Recovery artifact missing")
    diff = subprocess.run(["git", "diff", "--stat"], cwd=ROOT, capture_output=True, text=True)
    names = subprocess.run(["git", "diff", "--name-only"], cwd=ROOT, capture_output=True, text=True)
    batch_code = {
        "scripts/manual_t1_expanded_400.py": sha(ROOT / "scripts/manual_t1_expanded_400.py"),
        "scripts/manual_ab_compare.py": sha(ROOT / "scripts/manual_ab_compare.py"),
        "scripts/s1_run_diagnostic.py": sha(ROOT / "scripts/s1_run_diagnostic.py"),
        "scripts/s1_analyze_run.py": sha(ROOT / "scripts/s1_analyze_run.py"),
    }
    return {"controller_code_stage": stage, "batch_code_sha256": batch_code,
            "controller_library_sha256": sha(library), "mujoco_executable_sha256": sha(mujoco),
            "analysis_entry_sha256": sha(ROOT / "scripts/s1_analyze_run.py"),
            "manual_run_entry_sha256": sha(ROOT / "scripts/manual_ab_compare.py"),
            "model_artifacts": {str(p.relative_to(ROOT)): sha(p) for p in model_paths},
            "git_base": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                       capture_output=True, text=True, check=True).stdout.strip(),
            "tracked_diff_stat_at_freeze": diff.stdout.strip(),
            "tracked_diff_paths_at_freeze": names.stdout.splitlines(),
            "scene_index_sha256": sha(INDEX)}


def assert_frozen_code_unchanged(plan: dict) -> None:
    frozen = plan.get("code_snapshot") or {}
    expected_stage = frozen.get("controller_code_stage") or {}
    current_stage = coarse.ray_repair_code_stage({"code_commit": frozen.get("git_base")})
    for key in ("stage_id", "controller_library_sha256", "mujoco_executable_sha256"):
        if current_stage.get(key) != expected_stage.get(key):
            raise RuntimeError(f"frozen controller/MuJoCo code changed: {key}")
    for relpath, expected in (frozen.get("batch_code_sha256") or {}).items():
        path = ROOT / relpath
        if not path.is_file() or sha(path) != expected:
            raise RuntimeError(f"frozen run/analyzer code changed: {relpath}")
    for relpath, expected in (frozen.get("model_artifacts") or {}).items():
        path = ROOT / relpath
        if not path.is_file() or sha(path) != expected:
            raise RuntimeError(f"frozen model artifact changed: {relpath}")
    if frozen.get("scene_index_sha256") != sha(INDEX):
        raise RuntimeError("frozen scene index changed")


def prepare_overlays(root: Path) -> dict:
    overlays = {}
    for x in X_LEVELS:
        group = "A" if x == -0.05 else "B"
        key = "A_ref" if group == "A" else f"B_{x:.2f}"
        target = root / "overlays" / key
        manual.prepare_overlay(SOURCE, target, group, E, -0.07 if group == "A" else x, 0.30)
        cfg = target / "share/go2_description/config/abs/config.yaml"
        manual.set_overlay_goal(cfg, 7.5, 0.0)
        if group == "B":
            coarse.set_b_threshold(cfg, x)
        text = cfg.read_text(encoding="utf-8")
        if not manual.verify_overlay_detached(SOURCE, target):
            raise RuntimeError(f"overlay is not detached: {key}")
        if not overlay_configuration_matches(text, group, E, -0.07 if group == "A" else x,
                                            0.30, 7.5, 0.0):
            raise RuntimeError(f"production config parser rejected {key}")
        overlays[key] = {"path": str(target.resolve()), "group": group,
                         "entry_threshold_E": E,
                         "exit_setting": x,
                         "effective_exit_threshold": E if group == "A" else x,
                         "hold_enabled": False, "config_sha256": sha(cfg),
                         "detached_files": ["config.yaml", "policy.pt", "ra_value.pt", "rec/policy.pt"]}
    return overlays


def run_size_reference() -> dict:
    old = ROOT / "logs/s6_12/b_exit_coarse/20261015_192706_3"
    plan_path = old / "s6_12_coarse_plan.json"
    sizes = []
    if plan_path.is_file():
        plan = json.loads(plan_path.read_text(encoding="utf-8"))
        for item in plan.get("runs", []):
            for attempt in item.get("attempts", []):
                raw_path = attempt.get("run_directory")
                path = Path(raw_path) if raw_path else None
                if path is not None and path.is_dir() and attempt.get("record_validity") == "VALID":
                    sizes.append(allocated_bytes(path))
    if not sizes:
        raise RuntimeError("cannot estimate storage from prior valid production runs")
    return {"reference_batch": str(old), "valid_run_samples": len(sizes),
            "median_allocated_bytes_per_run": int(statistics.median(sizes)),
            "p90_allocated_bytes_per_run": int(sorted(sizes)[int(.9*(len(sizes)-1))])}


def storage_guard(root: Path, remaining: int, reference: dict) -> dict:
    usage = allocated_bytes(root)
    free = shutil.disk_usage(root).free
    # Use the prior-batch p90 per-run allocation, with a 20 MiB cap estimate for
    # each of the eight preselected two-framebuffer screenshot runs.
    projected = usage + remaining * reference["p90_allocated_bytes_per_run"] + 8 * 20 * 1024**2
    facts = {"checked_at": datetime.now().astimezone().isoformat(timespec="seconds"),
             "batch_allocated_bytes": usage, "filesystem_free_bytes": free,
             "remaining_planned_runs": remaining,
             "reference_median_run_bytes": reference["median_allocated_bytes_per_run"],
             "reference_p90_run_bytes": reference["p90_allocated_bytes_per_run"],
             "projected_final_batch_bytes": projected,
             "batch_budget_bytes": MAX_BATCH_BYTES, "minimum_free_bytes": MIN_FREE_BYTES}
    if free < MIN_FREE_BYTES:
        raise RuntimeError(f"disk protection: free space {free} below 15 GiB")
    if projected > MAX_BATCH_BYTES:
        raise RuntimeError(f"disk protection: projected batch {projected} exceeds 10 GiB")
    return facts


def all_metrics(root: Path, plan: dict) -> list[dict]:
    output = []
    for item in plan["runs"]:
        row = {k: item.get(k) for k in ("sequence", "plan_item_id", "map", "scene_id", "density",
              "route_structure", "repeat", "group", "entry_threshold_E", "exit_threshold_X",
              "effective_exit_threshold", "status", "screenshot_preselected")}
        row["attempts"] = len(item.get("attempts", []))
        latest = (item.get("attempts") or [{}])[-1]
        row["run_id"] = latest.get("run_id")
        row["terminal_result"] = latest.get("terminal_result")
        row["record_validity"] = latest.get("record_validity")
        row["run_directory"] = latest.get("run_directory")
        metrics_path = Path(latest.get("metrics_path", "")) if latest.get("metrics_path") else None
        metrics = {}
        if metrics_path and metrics_path.is_file():
            try: metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError): pass
        safety = metrics.get("task_window_safety") or {}
        row.update({
            "target_distance_final_m": metrics.get("target_distance_final_m"),
            "elapsed_task_duration_s": metrics.get("elapsed_task_duration_s"),
            "valid_motion_duration_s": metrics.get("valid_motion_duration_s"),
            "task_valid_time_coverage_fraction": metrics.get("task_valid_time_coverage_fraction"),
            "path_length_m": metrics.get("path_length_m"),
            "mean_path_speed_mps": metrics.get("mean_path_speed_mps"),
            "policy_frequency_hz": metrics.get("policy_frequency_hz_overall"),
            "mode_transitions": metrics.get("mode_transitions"),
            "short_reverse_switch_count": metrics.get("short_reverse_switch_count"),
            "short_reverse_switch_coverage": metrics.get("short_reverse_switch_coverage"),
            "short_reverse_per_valid_min": (metrics.get("short_reverse_switch_count") /
                (metrics.get("valid_motion_duration_s") / 60.0)
                if metrics.get("short_reverse_switch_coverage") == "complete" and
                isinstance(metrics.get("short_reverse_switch_count"), (int,float)) and
                isinstance(metrics.get("valid_motion_duration_s"), (int,float)) and metrics["valid_motion_duration_s"] > 0 else None),
            "recovery_duration_s": metrics.get("recovery_sim_duration_s"),
            "recovery_share_valid_motion_time": metrics.get("recovery_share_valid_motion_time"),
            "motion_collision_result": safety.get("collision", "UNKNOWN"),
            "motion_fall_result": metrics.get("motion_fall_result", "UNKNOWN"),
            "motion_safety_coverage_complete": safety.get("coverage_complete_for_motion_interval"),
            "foot_contact_count": safety.get("foot_contact_count"),
            "foot_impact_count": safety.get("foot_impact_count"),
            "nonfoot_collision_failure": safety.get("nonfoot_collision_failure"),
            "rl_step_gaps": metrics.get("rl_step_gaps"),
            "invalid_clock_frames": metrics.get("invalid_clock_frames"),
            "standing_observation_status": metrics.get("standing_observation_status"),
            "process_shutdown_complete": latest.get("process_shutdown_complete"),
            "closeout_complete": latest.get("closeout_complete"),
        })
        output.append(row)
    return output


CSV_FIELDS = ["sequence", "plan_item_id", "map", "scene_id", "density", "route_structure", "repeat",
 "group", "entry_threshold_E", "exit_threshold_X", "effective_exit_threshold", "status", "attempts",
 "run_id", "terminal_result", "record_validity", "run_directory", "screenshot_preselected",
 "target_distance_final_m", "elapsed_task_duration_s", "valid_motion_duration_s",
 "task_valid_time_coverage_fraction", "path_length_m", "mean_path_speed_mps", "policy_frequency_hz",
 "mode_transitions", "short_reverse_switch_count", "short_reverse_switch_coverage", "short_reverse_per_valid_min",
 "recovery_duration_s", "recovery_share_valid_motion_time", "motion_collision_result", "motion_fall_result",
 "motion_safety_coverage_complete", "foot_contact_count", "foot_impact_count", "nonfoot_collision_failure",
 "rl_step_gaps", "invalid_clock_frames", "standing_observation_status", "process_shutdown_complete", "closeout_complete"]


def write_rollup(root: Path, plan: dict) -> dict:
    rows = all_metrics(root, plan)
    with (root / "expanded_run_results.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction="ignore"); w.writeheader(); w.writerows(rows)
    attempts = []
    for item in plan["runs"]:
        for attempt in item.get("attempts", []):
            attempts.append({"sequence": item["sequence"], "plan_item_id": item["plan_item_id"],
                             "map": item["map"], "scene_id": item["scene_id"], "repeat": item["repeat"],
                             "group": item["group"], "entry_threshold_E": E,
                             "exit_threshold_X": item["exit_threshold_X"], **attempt})
    fields = sorted({k for a in attempts for k in a}) or ["sequence", "plan_item_id"]
    with (root / "expanded_attempt_results.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore"); w.writeheader(); w.writerows(attempts)
    valid = [r for r in rows if r["status"] == "VALID_TASK_RESULT" and r["record_validity"] == "VALID"]
    summary = {"status": plan["status"], "planned": 400,
               "started_plan_items": sum(bool(r["attempts"]) for r in rows),
               "attempts": len(attempts),
               "valid_latest_results": len(valid),
               "invalid_or_stopped_plan_items": sum(bool(r["attempts"]) and r["status"] != "VALID_TASK_RESULT" for r in rows),
               "not_started": sum(r["status"] == "NOT_STARTED" for r in rows),
               "terminal_latest": dict(Counter(r["terminal_result"] for r in valid)),
               "terminal_all_attempts": dict(Counter(a.get("terminal_result") for a in attempts)),
               "attempt_validity": dict(Counter(a.get("record_validity") for a in attempts)),
               "by_exit_threshold": {}, "by_map_threshold": {},
               "note": "Development-only; unknown safety results are not treated as false; no held-out data."}
    for x in X_LEVELS:
        bucket = [r for r in valid if math.isclose(float(r["exit_threshold_X"]), x, abs_tol=1e-9)]
        summary["by_exit_threshold"][f"{x:.2f}"] = {
            "planned": 50, "valid": len(bucket), "terminal": dict(Counter(r["terminal_result"] for r in bucket)),
            "collision_TRUE_FALSE_UNKNOWN": [sum(r["motion_collision_result"] is True for r in bucket),
              sum(r["motion_collision_result"] is False for r in bucket),
              sum(r["motion_collision_result"] not in (True,False) for r in bucket)],
            "fall_TRUE_FALSE_UNKNOWN": [sum(r["motion_fall_result"] is True for r in bucket),
              sum(r["motion_fall_result"] is False for r in bucket),
              sum(r["motion_fall_result"] not in (True,False) for r in bucket)],
            "arrived_elapsed_median_s": statistics.median([r["elapsed_task_duration_s"] for r in bucket
                if r["terminal_result"] == "ARRIVED" and isinstance(r["elapsed_task_duration_s"], (int,float))])
                if any(r["terminal_result"] == "ARRIVED" and isinstance(r["elapsed_task_duration_s"], (int,float)) for r in bucket) else None,
            "short_reverse_per_valid_min_n": sum(isinstance(r["short_reverse_per_valid_min"], (int,float)) for r in bucket),
            "short_reverse_per_valid_min_median": statistics.median([r["short_reverse_per_valid_min"] for r in bucket if isinstance(r["short_reverse_per_valid_min"], (int,float))]) if any(isinstance(r["short_reverse_per_valid_min"], (int,float)) for r in bucket) else None,
            "recovery_share_median": statistics.median([r["recovery_share_valid_motion_time"] for r in bucket if isinstance(r["recovery_share_valid_motion_time"], (int,float))]) if any(isinstance(r["recovery_share_valid_motion_time"], (int,float)) for r in bucket) else None}
    for m in plan["maps"]:
        for x in X_LEVELS:
            bucket=[r for r in valid if r["map"]==m["filename"] and math.isclose(float(r["exit_threshold_X"]),x,abs_tol=1e-9)]
            summary["by_map_threshold"][f"{m['filename']}|{x:.2f}"]={
                "planned":5,"valid":len(bucket),"terminal":dict(Counter(r["terminal_result"] for r in bucket)),
                "arrived":sum(r["terminal_result"]=="ARRIVED" for r in bucket),
                "collision_true_false_unknown":[sum(r["motion_collision_result"] is True for r in bucket),sum(r["motion_collision_result"] is False for r in bucket),sum(r["motion_collision_result"] not in (True,False) for r in bucket)],
                "fall_true_false_unknown":[sum(r["motion_fall_result"] is True for r in bucket),sum(r["motion_fall_result"] is False for r in bucket),sum(r["motion_fall_result"] not in (True,False) for r in bucket)],
                "arrival_elapsed_median_s":statistics.median([r["elapsed_task_duration_s"] for r in bucket if r["terminal_result"]=="ARRIVED" and isinstance(r["elapsed_task_duration_s"],(int,float))]) if any(r["terminal_result"]=="ARRIVED" and isinstance(r["elapsed_task_duration_s"],(int,float)) for r in bucket) else None,
                "short_reverse_rate_median":statistics.median([r["short_reverse_per_valid_min"] for r in bucket if isinstance(r["short_reverse_per_valid_min"],(int,float))]) if any(isinstance(r["short_reverse_per_valid_min"],(int,float)) for r in bucket) else None,
                "recovery_share_median":statistics.median([r["recovery_share_valid_motion_time"] for r in bucket if isinstance(r["recovery_share_valid_motion_time"],(int,float))]) if any(isinstance(r["recovery_share_valid_motion_time"],(int,float)) for r in bucket) else None}
    atomic_json(root / "expanded_partial_summary.json", summary)
    return summary


def _num(value):
    try:
        x=float(value)
        return x if math.isfinite(x) else None
    except (TypeError,ValueError): return None


def _bool_or_unknown(value):
    if value is True or value == "True": return True
    if value is False or value == "False": return False
    return None


def final_tables_and_cluster_ci(root: Path, rows: list[dict]) -> dict:
    """Write 80 map-arm rows, 8 arm rows, and paired map-cluster bootstrap CIs."""
    maps=MAP_NAMES
    map_rows=[]
    for map_name in maps:
        for x in X_LEVELS:
            planned=[r for r in rows if r["map"]==map_name and abs(float(r["exit_threshold_X"])-x)<1e-8]
            valid=[r for r in planned if r["status"]=="VALID_TASK_RESULT" and r["record_validity"]=="VALID"]
            terminals=Counter(r["terminal_result"] for r in valid)
            collision_true=sum(_bool_or_unknown(r["motion_collision_result"]) is True for r in valid)
            collision_false=sum(_bool_or_unknown(r["motion_collision_result"]) is False for r in valid)
            fall_true=sum(_bool_or_unknown(r["motion_fall_result"]) is True for r in valid)
            fall_false=sum(_bool_or_unknown(r["motion_fall_result"]) is False for r in valid)
            arr=[r for r in valid if r["terminal_result"]=="ARRIVED"]
            rev=[_num(r["short_reverse_per_valid_min"]) for r in valid];rev=[v for v in rev if v is not None]
            rec=[_num(r["recovery_share_valid_motion_time"]) for r in valid];rec=[v for v in rec if v is not None]
            times=[_num(r["elapsed_task_duration_s"]) for r in arr];times=[v for v in times if v is not None]
            map_rows.append({"map":map_name,"scene_id":planned[0]["scene_id"] if planned else "",
                "density":planned[0]["density"] if planned else "","route_structure":planned[0]["route_structure"] if planned else "",
                "exit_threshold_X":f"{x:.2f}","planned_n":5,"attempt_n":sum(int(r["attempts"]) for r in planned),
                "valid_task_n":len(valid),"invalid_or_stopped_n":sum(bool(r["attempts"]) and r["status"]!="VALID_TASK_RESULT" for r in planned),
                "not_started_n":sum(not r["attempts"] for r in planned),
                "arrived_n":terminals["ARRIVED"],"collision_terminal_n":terminals["COLLISION_TERMINATION"],
                "fall_terminal_n":terminals["FALL_TERMINATION"],"sim_limit_n":terminals["DIAGNOSTIC_SIM_TIME_LIMIT"],
                "collision_true_n":collision_true,"collision_false_n":collision_false,
                "collision_unknown_n":len(valid)-collision_true-collision_false,
                "fall_true_n":fall_true,"fall_false_n":fall_false,"fall_unknown_n":len(valid)-fall_true-fall_false,
                "arrival_fraction_of_valid":terminals["ARRIVED"]/len(valid) if valid else None,
                "arrived_elapsed_mean_s":statistics.mean(times) if times else None,
                "short_reverse_per_valid_min_n":len(rev),"short_reverse_per_valid_min_mean":statistics.mean(rev) if rev else None,
                "recovery_share_n":len(rec),"recovery_share_mean":statistics.mean(rec) if rec else None})
    fpath=root/"map_threshold_summary_80.csv"
    with fpath.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=list(map_rows[0]));w.writeheader();w.writerows(map_rows)
    threshold_rows=[]
    for x in X_LEVELS:
        rr=[r for r in rows if abs(float(r["exit_threshold_X"])-x)<1e-8]
        v=[r for r in rr if r["status"]=="VALID_TASK_RESULT" and r["record_validity"]=="VALID"]
        terminal=Counter(r["terminal_result"] for r in v)
        known_collision=[r for r in v if _bool_or_unknown(r["motion_collision_result"]) is not None]
        known_fall=[r for r in v if _bool_or_unknown(r["motion_fall_result"]) is not None]
        arrivals=[r for r in v if r["terminal_result"]=="ARRIVED"]
        arrival_times=[_num(r["elapsed_task_duration_s"]) for r in arrivals];arrival_times=[z for z in arrival_times if z is not None]
        rev=[_num(r["short_reverse_per_valid_min"]) for r in v];rev=[z for z in rev if z is not None]
        shares=[_num(r["recovery_share_valid_motion_time"]) for r in v];shares=[z for z in shares if z is not None]
        threshold_rows.append({"exit_threshold_X":f"{x:.2f}","group":"A_reference" if x==-.05 else "B",
            "planned_n":50,"attempt_n":sum(int(r["attempts"]) for r in rr),"valid_task_n":len(v),
            "invalid_or_stopped_n":sum(bool(r["attempts"]) and r["status"]!="VALID_TASK_RESULT" for r in rr),
            "not_started_n":sum(not r["attempts"] for r in rr),"arrived_n":terminal["ARRIVED"],
            "collision_terminal_n":terminal["COLLISION_TERMINATION"],"fall_terminal_n":terminal["FALL_TERMINATION"],
            "sim_limit_n":terminal["DIAGNOSTIC_SIM_TIME_LIMIT"],"other_terminal_n":len(v)-sum(terminal[t] for t in ("ARRIVED","COLLISION_TERMINATION","FALL_TERMINATION","DIAGNOSTIC_SIM_TIME_LIMIT")),
            "collision_true_n":sum(_bool_or_unknown(r["motion_collision_result"]) is True for r in v),"collision_false_n":sum(_bool_or_unknown(r["motion_collision_result"]) is False for r in v),
            "collision_unknown_n":len(v)-len(known_collision),"fall_true_n":sum(_bool_or_unknown(r["motion_fall_result"]) is True for r in v),
            "fall_false_n":sum(_bool_or_unknown(r["motion_fall_result"]) is False for r in v),"fall_unknown_n":len(v)-len(known_fall),
            "arrival_fraction_valid_results":terminal["ARRIVED"]/len(v) if v else None,
            "arrival_time_mean_s_arrived_only":statistics.mean(arrival_times) if arrival_times else None,
            "short_reverse_rate_n_complete":len(rev),"short_reverse_per_valid_min_mean":statistics.mean(rev) if rev else None,
            "recovery_share_n":len(shares),"recovery_share_mean":statistics.mean(shares) if shares else None})
    with (root/"threshold_summary_8.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=list(threshold_rows[0]));w.writeheader();w.writerows(threshold_rows)

    # Map-cluster bootstrap; use identical sampled map-index vectors for all X and
    # all paired B-A differences, preserving within-map repeat dependence.
    rng=random.Random(BOOTSTRAP_SEED); draws=[[rng.randrange(len(maps)) for _ in maps] for _ in range(10000)]
    metrics=("arrival_fraction_of_valid","collision_true_fraction_known","fall_true_fraction_known",
             "short_reverse_per_valid_min_mean","recovery_share_mean","arrived_elapsed_mean_s_arrived_only")
    per_map={}
    for m in maps:
        for x in X_LEVELS:
            rr=[r for r in rows if r["map"]==m and abs(float(r["exit_threshold_X"])-x)<1e-8 and r["status"]=="VALID_TASK_RESULT" and r["record_validity"]=="VALID"]
            arr=sum(r["terminal_result"]=="ARRIVED" for r in rr)
            collision=[_bool_or_unknown(r["motion_collision_result"]) for r in rr if _bool_or_unknown(r["motion_collision_result"]) is not None]
            fall=[_bool_or_unknown(r["motion_fall_result"]) for r in rr if _bool_or_unknown(r["motion_fall_result"]) is not None]
            rev=[_num(r["short_reverse_per_valid_min"]) for r in rr];rev=[z for z in rev if z is not None]
            shares=[_num(r["recovery_share_valid_motion_time"]) for r in rr];shares=[z for z in shares if z is not None]
            at=[_num(r["elapsed_task_duration_s"]) for r in rr if r["terminal_result"]=="ARRIVED"];at=[z for z in at if z is not None]
            per_map[(m,x)]={"arrival_fraction_of_valid":arr/len(rr) if rr else None,
                "collision_true_fraction_known":sum(v is True for v in collision)/len(collision) if collision else None,
                "fall_true_fraction_known":sum(v is True for v in fall)/len(fall) if fall else None,
                "short_reverse_per_valid_min_mean":statistics.mean(rev) if rev else None,
                "recovery_share_mean":statistics.mean(shares) if shares else None,
                "arrived_elapsed_mean_s_arrived_only":statistics.mean(at) if at else None}
    ci_rows=[]
    def ci(values):
        values=sorted(v for v in values if v is not None and math.isfinite(v))
        if not values:return (None,None)
        return values[int(.025*(len(values)-1))],values[int(.975*(len(values)-1))]
    for x in X_LEVELS:
        for metric in metrics:
            key=metric
            available=[m for m in maps if per_map[(m,x)][key] is not None]
            vals=[per_map[(m,x)][key] for m in available]
            point=statistics.mean(vals) if vals else None
            boot=[]
            for draw in draws:
                sample=[per_map[(maps[i],x)][key] for i in draw if per_map[(maps[i],x)][key] is not None]
                if sample:boot.append(statistics.mean(sample))
            lo,hi=ci(boot)
            ci_rows.append({"exit_threshold_X":f"{x:.2f}","contrast":"within_threshold","metric":metric,
                "map_clusters_available":len(available),"bootstrap_seed":BOOTSTRAP_SEED,"bootstrap_replicates":len(boot),
                "estimate":point,"ci95_low":lo,"ci95_high":hi})
            if x != -.05:
                paired=[m for m in maps if per_map[(m,x)][key] is not None and per_map[(m,-.05)][key] is not None]
                diffs=[per_map[(m,x)][key]-per_map[(m,-.05)][key] for m in paired]
                dpoint=statistics.mean(diffs) if diffs else None
                dboot=[]
                for draw in draws:
                    sample=[]
                    for i in draw:
                        m=maps[i]
                        if per_map[(m,x)][key] is not None and per_map[(m,-.05)][key] is not None:
                            sample.append(per_map[(m,x)][key]-per_map[(m,-.05)][key])
                    if sample:dboot.append(statistics.mean(sample))
                dlo,dhi=ci(dboot)
                ci_rows.append({"exit_threshold_X":f"{x:.2f}","contrast":"paired_B_minus_A","metric":metric,
                    "map_clusters_available":len(paired),"bootstrap_seed":BOOTSTRAP_SEED,"bootstrap_replicates":len(dboot),
                    "estimate":dpoint,"ci95_low":dlo,"ci95_high":dhi})
    with (root/"map_cluster_bootstrap_ci.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=list(ci_rows[0]));w.writeheader();w.writerows(ci_rows)
    return {"map_threshold_rows":len(map_rows),"threshold_rows":len(threshold_rows),"bootstrap_rows":len(ci_rows),
            "bootstrap_maps":maps,"bootstrap_resampling":"10 map clusters sampled with replacement; 10,000 draws; shared map-index draws across thresholds and B-A contrasts",
            "bootstrap_seed":BOOTSTRAP_SEED,"unknown_handling":"unknown collision/fall excluded from known-outcome proportion and separately counted; never treated as false"}


def run_one(root: Path, plan: dict, item: dict, args: argparse.Namespace) -> None:
    assert_frozen_code_unchanged(plan)
    previous_attempts = item.get("attempts", [])
    retry = item["status"] == "RETRY_AUTHORIZED_ENGINEERING"
    if retry:
        if len(previous_attempts) != 1 or previous_attempts[0].get("status") != "ENGINEERING_FAILURE_REPAIRED":
            raise RuntimeError("engineering retry must have exactly one explicitly repaired prior attempt")
    elif item["status"] != "NOT_STARTED" or previous_attempts:
        raise RuntimeError(f"refusing to rerun or skip non-fresh item {item['plan_item_id']}")
    overlay_key = "A_ref" if item["group"] == "A" else f"B_{item['exit_threshold_X']:.2f}"
    overlay = Path(plan["overlays"][overlay_key]["path"])
    expected_x = E if item["group"] == "A" else item["exit_threshold_X"]
    config = overlay / "share/go2_description/config/abs/config.yaml"
    if item["group"] == "B": coarse.set_b_threshold(config, expected_x)
    if not overlay_configuration_matches(config.read_text(encoding="utf-8"), item["group"], E,
                -0.07 if item["group"] == "A" else expected_x, 0.30, 7.5, 0.0):
        raise RuntimeError(f"overlay no longer matches frozen configuration for {item['plan_item_id']}")
    guard = storage_guard(root, sum(r["status"] == "NOT_STARTED" for r in plan["runs"]), plan["storage_reference"])
    plan.setdefault("storage_checks", []).append(guard)
    attempt_number = len(previous_attempts) + 1
    item_root = root / "runs" / item["plan_item_id"]
    child = item_root if attempt_number == 1 else item_root / f"attempt_{attempt_number:02d}"
    child.mkdir(parents=True, exist_ok=False)
    result_root = child / "result"
    attempt = {"attempt_number":attempt_number,"status":"RUNNING","started_at":datetime.now().astimezone().isoformat(timespec="seconds"),
               "result_root":str(result_root),"screenshot_preselected":item["screenshot_preselected"]}
    item.setdefault("attempts", []).append(attempt); item["status"]="RUNNING"; atomic_json(root/"expanded_400_plan.json",plan)
    command=["/usr/bin/python3","-u",str(ROOT/"scripts/manual_ab_compare.py"),
      "--groups",item["group"],"--repeats","1","--scene",item["map"],
      "--goal-x","7.5","--goal-y","0","--sim-limit-s","20","--wall-cap-s","120",
      "--entry-threshold","-0.05","--exit-threshold",f"{-0.07 if item['group']=='A' else expected_x:.17g}",
      "--hold-s","0.30","--classification","T1 expanded 400 development parameter validation; nonformal",
      "--display",args.display,"--xauthority",args.xauthority,"--plot-python",args.plot_python,
      "--result-root",str(result_root),"--prepared-overlay",str(overlay)]
    if not item["screenshot_preselected"]: command.append("--disable-framebuffer-capture")
    log=child/"console.log"; code=None
    with log.open("w",encoding="utf-8") as f:
        proc=subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        try:
            assert proc.stdout is not None
            for line in proc.stdout:
                print(line,end="",flush=True); f.write(line); f.flush()
            code=proc.wait()
        except KeyboardInterrupt:
            if proc.poll() is None: proc.send_signal(signal.SIGINT)
            try: proc.wait(timeout=90)
            except subprocess.TimeoutExpired: raise RuntimeError("run child cleanup unconfirmed after Ctrl+C")
            raise
    bp_path=result_root/"batch_plan.json"; bp=json.loads(bp_path.read_text(encoding="utf-8")) if bp_path.is_file() else {}
    status=next((x for x in bp.get("run_statuses",[]) if x.get("group")==item["group"]),{})
    run_id=status.get("run_id")
    run_dir=Path(status.get("output_directory","")) if status.get("output_directory") else None
    if run_dir and not run_dir.is_absolute(): run_dir=(result_root/run_dir).resolve()
    metrics={}; context={}; terminal={}; facts={}
    for name,dest in (("metrics.json",metrics),("run_context.json",context),("terminal.json",terminal),("process_facts.json",facts)):
        if run_dir and (run_dir/name).is_file():
            try: dest.update(json.loads((run_dir/name).read_text(encoding="utf-8")))
            except (OSError,json.JSONDecodeError): pass
    source=(context.get("runtime_source_validation") or {}).get("evidence") or {}
    supervisor=metrics.get("supervisor_terminal_result") or status.get("result") or context.get("terminal_result")
    attempt.update({"finished_at":datetime.now().astimezone().isoformat(timespec="seconds"),"child_exit_code":code,
      "run_id":run_id,"run_directory":str(run_dir) if run_dir else None,"metrics_path":str(run_dir/"metrics.json") if run_dir else None,
      "terminal_result":supervisor,"record_validity":metrics.get("record_validity"),
      "controller_code_stage":plan["code_snapshot"]["controller_code_stage"]["stage_id"],
      "runtime_source_validation":(context.get("runtime_source_validation") or {}).get("status"),
      "actual_group":source.get("group"),"actual_entry_threshold":source.get("entry_threshold"),"actual_exit_threshold":source.get("exit_threshold"),
      "process_shutdown_complete":facts.get("shutdown_complete",metrics.get("process_shutdown_complete")),
      "closeout_complete":facts.get("closeout_complete",metrics.get("closeout_complete")),
      "first_active_policy_frame_exactly_persisted":(metrics.get("s6_preparation_readiness") or {}).get("first_active_policy_frame_exactly_persisted"),
      "task_window_safety":{"collision":(metrics.get("task_window_safety") or {}).get("collision"),"fall":metrics.get("motion_fall_result"),"coverage":(metrics.get("task_window_safety") or {}).get("coverage_reason")}})
    item.update({"run_id":run_id,"terminal_result":supervisor,"record_validity":metrics.get("record_validity"),
                 "run_directory":str(run_dir) if run_dir else None})
    fatal=[]
    if code != 0: fatal.append(f"runner_exit={code}")
    if metrics.get("record_validity") != "VALID": fatal.append(f"record_validity={metrics.get('record_validity')}")
    if (context.get("runtime_source_validation") or {}).get("status") != "confirmed": fatal.append("runtime_source_not_confirmed")
    if source.get("group") != item["group"]: fatal.append(f"actual_group={source.get('group')}")
    try:
        if abs(float(source.get("entry_threshold"))-E)>1e-5 or abs(float(source.get("exit_threshold"))-expected_x)>1e-5:
            fatal.append("consumed_threshold_mismatch")
    except (TypeError,ValueError): fatal.append("consumed_threshold_missing")
    if attempt["process_shutdown_complete"] is not True or attempt["closeout_complete"] is not True:
        fatal.append("shutdown_or_closeout_unconfirmed")
    if attempt["first_active_policy_frame_exactly_persisted"] is not True:
        fatal.append("cycle0_or_first_active_frame_not_exactly_persisted")
    if supervisor == "SYSTEM_SAFETY_ABORT":
        fatal.append("system_safety_abort_requires_review_before_continuation")
    if not run_dir or not (run_dir/"runtime_record.jsonl").is_file() or not (run_dir/"terminal.json").is_file():
        fatal.append("raw_runtime_or_terminal_missing")
    item["finished_at"]=attempt["finished_at"]
    if fatal:
        attempt["status"] = "STOPPED_FAILURE"
        attempt["failure_reasons"] = fatal
    else:
        item["failure_reasons"] = []
        item["engineering_retry_used"] = attempt_number > 1
    item["status"]="STOPPED_FAILURE" if fatal else "VALID_TASK_RESULT"
    item["failure_reasons"]=fatal
    atomic_json(root/"expanded_400_plan.json",plan)
    if fatal:
        raise RuntimeError(f"{item['plan_item_id']} stopped: {'; '.join(fatal)}; no next item launched")
    print(f"T1 expanded {item['sequence']}/400: {item['map']} {item['group']} X={expected_x:.2f} -> {supervisor}; record/source/cleanup checked.",flush=True)


def finalize(root: Path, plan: dict, plot_python: str) -> None:
    summary=write_rollup(root,plan)
    rows=list(csv.DictReader((root/"expanded_run_results.csv").open(encoding="utf-8")))
    final_stats=final_tables_and_cluster_ci(root,rows)
    summary.update(final_stats)
    atomic_json(root/"expanded_partial_summary.json",summary)
    # Predeclared timeline selection: one ARRIVED sample nearest its threshold
    # median time, plus the first non-arrival in frozen order per threshold.
    timelines=[]
    for x in X_LEVELS:
        group=[r for r in rows if r["status"]=="VALID_TASK_RESULT" and abs(float(r["exit_threshold_X"])-x)<1e-8]
        arrived=[r for r in group if r["terminal_result"]=="ARRIVED" and _num(r["elapsed_task_duration_s"]) is not None]
        if arrived:
            med=statistics.median(_num(r["elapsed_task_duration_s"]) for r in arrived)
            pick=min(arrived,key=lambda r:abs(_num(r["elapsed_task_duration_s"])-med))
            timelines.append({"threshold":x,"role":"ARRIVED time nearest within-arm median","plan_item_id":pick["plan_item_id"],"run_directory":pick["run_directory"]})
        fail=next((r for r in group if r["terminal_result"]!="ARRIVED"),None)
        if fail:
            timelines.append({"threshold":x,"role":"first non-ARRIVED in frozen order","plan_item_id":fail["plan_item_id"],"run_directory":fail["run_directory"]})
    atomic_json(root/"representative_timelines.json",{"selection_rule":"per threshold, ARRIVED elapsed nearest arm median plus first non-ARRIVED in frozen order; never best-case selection","items":timelines})
    plot_script=r'''import csv,json,sys,math
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(sys.argv[1]); rows=list(csv.DictReader((p/'expanded_run_results.csv').open())); x=[-.05,-.07,-.09,-.11,-.13,-.15,-.17,-.19]; labs=[f'{v:.2f}' for v in x]
valid=[r for r in rows if r['status']=='VALID_TASK_RESULT']
terms=sorted(set(r['terminal_result'] for r in valid if r['terminal_result']))
fig,ax=plt.subplots(figsize=(10,5)); bottom=np.zeros(8)
for term in terms:
 vals=np.array([sum(r['terminal_result']==term and abs(float(r['exit_threshold_X'])-v)<1e-7 for r in valid) for v in x]); ax.bar(labs,vals,bottom=bottom,label=term); bottom+=vals
ax.set(title=f'T1 400 development runs: terminal composition (valid records n={len(valid)})',xlabel='Exit threshold; -0.05 is A reference',ylabel='runs');ax.legend(fontsize=8);fig.tight_layout();fig.savefig(p/'terminal_composition.png',dpi=140);plt.close(fig)
def barline(field,title,ylabel,predicate=lambda r:True,fn=None):
 if fn is None:
  fn=lambda r:float(r[field]) if r[field] not in ('',None) else math.nan
 vals=[];ns=[]
 for v in x:
  q=[r for r in valid if abs(float(r['exit_threshold_X'])-v)<1e-7 and predicate(r)]; z=[fn(r) for r in q];z=[a for a in z if math.isfinite(a)];vals.append(np.mean(z) if z else np.nan);ns.append(len(z))
 fig,ax=plt.subplots(figsize=(9,4));ax.plot(labs,vals,'o-');ax.grid(alpha=.3);ax.set(title=f'{title}; n={ns}',xlabel='Exit threshold; -0.05 is A reference',ylabel=ylabel);fig.tight_layout();fig.savefig(p/(field+'.png'),dpi=140);plt.close(fig)
barline('short_reverse_per_valid_min','Short reverse switches per valid task minute','count/min')
barline('recovery_share_valid_motion_time','Recovery share of valid motion time','fraction')
barline('elapsed_task_duration_s','Arrival time (ARRIVED only)','seconds',lambda r:r['terminal_result']=='ARRIVED')
maps=['dev01.xml','dev03.xml','dev05.xml','dev09.xml','dev11.xml','dev15.xml','dev17.xml','dev23.xml','dev27.xml','dev29.xml']; mat=np.zeros((10,8))
for i,m in enumerate(maps):
 for j,v in enumerate(x): mat[i,j]=sum(r['map']==m and r['terminal_result']=='ARRIVED' and abs(float(r['exit_threshold_X'])-v)<1e-7 for r in valid)/5
fig,ax=plt.subplots(figsize=(10,5));im=ax.imshow(mat,vmin=0,vmax=1,aspect='auto',cmap='viridis');ax.set_xticks(range(8),labs);ax.set_yticks(range(10),maps);ax.set(title='ARRIVED fraction by map and exit threshold (five planned repeats)');fig.colorbar(im,ax=ax,label='ARRIVED / 5');fig.tight_layout();fig.savefig(p/'map_threshold_arrival_fraction.png',dpi=140);plt.close(fig)
for field,name,title in [('motion_collision_result','collision_outcome_by_threshold.png','Task-window collision truth'),('motion_fall_result','fall_outcome_by_threshold.png','Task-window fall truth')]:
 fig,ax=plt.subplots(figsize=(9,4)); bottoms=np.zeros(8)
 for cat in ('True','False','UNKNOWN'):
  vals=np.array([sum(r[field]==cat and abs(float(r['exit_threshold_X'])-v)<1e-7 for r in valid) for v in x]);ax.bar(labs,vals,bottom=bottoms,label=cat);bottoms+=vals
 ax.set(title=title+' (UNKNOWN retained)',xlabel='Exit threshold',ylabel='runs');ax.legend();fig.tight_layout();fig.savefig(p/name,dpi=140);plt.close(fig)
'''
    env=os.environ.copy();env["MPLCONFIGDIR"]="/tmp/t1_400_mpl";Path(env["MPLCONFIGDIR"]).mkdir(parents=True,exist_ok=True);env["PYTHONNOUSERSITE"]="1"
    proc=subprocess.run([plot_python,"-c",plot_script,str(root)],cwd=ROOT,env=env,capture_output=True,text=True)
    if proc.returncode: raise RuntimeError(f"summary plot generation failed: {proc.stderr[-2000:]}")
    plan["status"]="COMPLETE";plan["completed_at"]=datetime.now().astimezone().isoformat(timespec="seconds")
    summary=write_rollup(root,plan)
    rows=list(csv.DictReader((root/"expanded_run_results.csv").open(encoding="utf-8")))
    final_stats=final_tables_and_cluster_ci(root,rows);summary.update(final_stats)
    atomic_json(root/"expanded_partial_summary.json",summary)
    atomic_json(root/"expanded_400_plan.json",plan)
    atomic_json(root/"expanded_final_summary.json",summary)


def main() -> int:
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--prepare-only",action="store_true")
    ap.add_argument("--refresh-frozen-code-snapshot",action="store_true",
                    help="refresh code hashes only before the first attempt of a FROZEN plan")
    ap.add_argument("--authorize-repaired-engineering-retry",action="store_true",
                    help="authorize only the specifically repaired first-attempt CLI argument failure")
    ap.add_argument("--batch-root",type=Path)
    ap.add_argument("--resume",action="store_true")
    ap.add_argument("--display",default=os.environ.get("DISPLAY",":0"))
    ap.add_argument("--xauthority",default=os.environ.get("XAUTHORITY","/run/user/1000/gdm/Xauthority"))
    ap.add_argument("--plot-python",default="/home/lidio/anaconda3/bin/python3")
    args=ap.parse_args()
    if not Path(args.plot_python).is_file(): ap.error("plot Python is missing")
    plot_env=os.environ.copy();plot_env["MPLCONFIGDIR"]="/tmp/t1_400_mpl_preflight";Path(plot_env["MPLCONFIGDIR"]).mkdir(parents=True,exist_ok=True);plot_env["PYTHONNOUSERSITE"]="1"
    plot_check=subprocess.run([args.plot_python,"-c","import numpy, matplotlib; import matplotlib.pyplot"],cwd=ROOT,env=plot_env,capture_output=True,text=True)
    if plot_check.returncode: raise RuntimeError(f"plotting stack unavailable before launch: {plot_check.stderr[-1200:]}")
    catalog=scene_catalog()
    if args.batch_root:
        if not args.resume: ap.error("existing batch requires --resume")
        root=args.batch_root.resolve();plan=json.loads((root/"expanded_400_plan.json").read_text(encoding="utf-8"))
        if args.refresh_frozen_code_snapshot:
            if plan.get("status") != "FROZEN" or any(r.get("attempts") or r.get("status") != "NOT_STARTED" for r in plan["runs"]):
                raise RuntimeError("code snapshot can refresh only before any attempt in a FROZEN plan")
            snapshot=code_snapshot({"code_commit":plan["code_snapshot"]["git_base"]})
            plan["code_snapshot"]=snapshot
            atomic_json(root/"expanded_400_plan.json",plan)
            print(f"Refreshed pre-run frozen code snapshot: {snapshot['controller_code_stage']['stage_id']}",flush=True)
            return 0
        if args.authorize_repaired_engineering_retry:
            if plan.get("status") != "STOPPED":
                raise RuntimeError("engineering retry authorization requires a stopped batch")
            failed = [r for r in plan["runs"] if r.get("status") == "STOPPED_FAILURE"]
            if len(failed) != 1:
                raise RuntimeError("expected exactly one stopped plan item")
            item = failed[0]
            attempts = item.get("attempts", [])
            if len(attempts) != 1 or attempts[0].get("child_exit_code") != 2 or attempts[0].get("run_id"):
                raise RuntimeError("prior attempt was not a pre-launch CLI failure")
            console = Path(attempts[0].get("result_root", "" )).parent / "console.log"
            if not console.is_file() or "unrecognized arguments: --startup-gain-trace" not in console.read_text(encoding="utf-8", errors="replace"):
                raise RuntimeError("prior failure does not match the one explicitly repaired CLI issue")
            if len(plan.get("engineering_retry_history", [])) >= MAX_ENGINEERING_RETRIES:
                raise RuntimeError("engineering retry allowance exhausted")
            attempts[0]["status"] = "ENGINEERING_FAILURE_REPAIRED"
            attempts[0]["repair"] = {"issue":"outer batch passed --startup-gain-trace to manual_ab_compare.py, which already enables it internally",
                                      "fix":"removed unsupported duplicate CLI argument; no simulation/controller behavior changed",
                                      "repaired_at":datetime.now().astimezone().isoformat(timespec="seconds")}
            item["status"] = "RETRY_AUTHORIZED_ENGINEERING"
            plan.setdefault("engineering_retry_history", []).append({"plan_item_id":item["plan_item_id"],
                "attempt_number":1,"reason":"pre-launch CLI parser rejection; no MuJoCo/controller process launched",
                "repair":"remove redundant unsupported argument; retry same plan item once"})
            plan["code_snapshot"] = code_snapshot({"code_commit":plan["code_snapshot"]["git_base"]})
            plan["status"] = "FROZEN"
            atomic_json(root/"expanded_400_plan.json",plan)
            print(f"Authorized one repaired engineering retry for {item['plan_item_id']}; original attempt retained.",flush=True)
            return 0
        if plan.get("scene_index_sha256")!=sha(INDEX): raise RuntimeError("M1 scene index changed")
        assert_frozen_code_unchanged(plan)
        if [[r[k] for k in ("plan_item_id","map","scene_id","repeat","group","exit_threshold_X")] for r in plan["runs"]] != [[r[k] for k in ("plan_item_id","map","scene_id","repeat","group","exit_threshold_X")] for r in make_schedule(catalog)[0]]: raise RuntimeError("resume schedule identity mismatch")
        if any(r["status"] not in ("NOT_STARTED","VALID_TASK_RESULT","RETRY_AUTHORIZED_ENGINEERING") for r in plan["runs"]): raise RuntimeError("batch contains a stopped/incomplete row; no automatic retry/continuation")
        if any(r["status"]=="NOT_STARTED" for r in plan["runs"]):
            idx=min(i for i,r in enumerate(plan["runs"]) if r["status"] in ("NOT_STARTED","RETRY_AUTHORIZED_ENGINEERING"))
            if any(r["status"] not in ("NOT_STARTED","RETRY_AUTHORIZED_ENGINEERING") for r in plan["runs"][idx:]): raise RuntimeError("resume would skip/duplicate an item")
        elif any(r["status"]=="RETRY_AUTHORIZED_ENGINEERING" for r in plan["runs"]):
            idx=min(i for i,r in enumerate(plan["runs"]) if r["status"]=="RETRY_AUTHORIZED_ENGINEERING")
            if any(r["status"] not in ("NOT_STARTED","RETRY_AUTHORIZED_ENGINEERING") for r in plan["runs"][idx:]): raise RuntimeError("retry would skip/duplicate an item")
    else:
        if args.resume: ap.error("--resume needs --batch-root")
        PARENT.mkdir(parents=True,exist_ok=True)
        root=PARENT/(datetime.now().strftime("%Y%m%d_%H%M%S")+"_"+str(os.getpid()))
        root.mkdir(parents=False,exist_ok=False);(root/"runs").mkdir();(root/"summary").mkdir()
        runs,screens=make_schedule(catalog);validate_schedule(runs,catalog)
        code=code_snapshot({"code_commit":subprocess.run(["git","rev-parse","HEAD"],cwd=ROOT,capture_output=True,text=True,check=True).stdout.strip()})
        overlays=prepare_overlays(root)
        plan={"schema":"t1-expanded-400-v1","status":"FROZEN","created_at":datetime.now().astimezone().isoformat(timespec="seconds"),
              "batch_id":root.name,"classification":"development-set parameter validation; nonformal; not paper results",
              "library_version":"m1-mapset/1.0","scene_index_sha256":sha(INDEX),"maps":catalog,"runs":runs,
              "planned_count":400,"A_count":50,"B_count":350,"thresholds":list(X_LEVELS),"entry_threshold_E":E,
              "hold_enabled":False,"goal_world_m":[7.5,0.0],"arrival_radius_m":0.5,"sim_limit_s":20,"wall_cap_s":120,
              "standing_observation_sim_s":5,"standing_wall_cap_s":30,"startup_gain_transition_control_s":0.30,
              "schedule_seed":SEED,"schedule_rule":"five rounds; shuffled map order per round; eight shuffled thresholds within each map-round block",
              "screenshot_selection_plan_item_ids":screens,"screenshot_count_max":8,"recording_policy":"raw JSONL/context/terminal/process facts/metrics/timeline retained; no video; screenshots only for selected first-map round-1 threshold block",
              "storage_budget_bytes":MAX_BATCH_BYTES,"minimum_filesystem_free_bytes":MIN_FREE_BYTES,
              "storage_reference":run_size_reference(),"code_snapshot":code,"overlays":overlays,
              "engineering_retry_limit":MAX_ENGINEERING_RETRIES,"per_item_max_attempts":2,
              "recovery_policy":"only explicit repaired engineering failures; never retry valid terminal outcomes; stop on first unresolved failure",
              "preflight_process_inventory":{"checked_before_batch_creation":True,"matches":[],"method":"exact process command-line scan; no task-owned MuJoCo/controller/RViz/runner/recorder process was present"},
              "run_command_template":"rtk proxy env DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority MUJOCO_GL=glfw /usr/bin/python3 scripts/manual_t1_expanded_400.py --batch-root <batch> --resume"}
        atomic_json(root/"expanded_400_plan.json",plan)
        validate_schedule(plan["runs"],catalog)
        if args.prepare_only:
            print(f"Prepared frozen 400-item plan: {root}; overlays detached; production parser accepted all 8 settings; no simulation started.",flush=True)
            return 0
    if plan["status"]=="COMPLETE": print("400-item batch already complete; no new launch.");return 0
    plan["status"]="RUNNING";atomic_json(root/"expanded_400_plan.json",plan)
    rows=plan["runs"]
    pending=[r for r in rows if r["status"] in ("NOT_STARTED","RETRY_AUTHORIZED_ENGINEERING")]
    for ix,item in enumerate(pending,1):
        remaining=sum(r["status"]=="NOT_STARTED" for r in rows)
        guard=storage_guard(root,remaining,plan["storage_reference"])
        plan.setdefault("storage_checks",[]).append(guard)
        try: run_one(root,plan,item,args)
        except KeyboardInterrupt:
            plan["status"]="STOPPED_OPERATOR";atomic_json(root/"expanded_400_plan.json",plan);write_rollup(root,plan);raise
        except Exception as e:
            plan["status"]="STOPPED";plan["stop_sequence"]=item["sequence"];plan["stop_reason"]=str(e)
            atomic_json(root/"expanded_400_plan.json",plan);write_rollup(root,plan);print(f"STOP at {item['plan_item_id']}: {e}",file=sys.stderr);return 1
        if item["sequence"]%8==0:
            write_rollup(root,plan)
            print(f"进度 {item['sequence']}/400：{item['map']} / X={item['exit_threshold_X']:.2f}；批次已分配{allocated_bytes(root)/1024**3:.2f}GiB。",flush=True)
        if item["sequence"]%80==0:
            atomic_json(root/f"round_progress_{item['sequence']:03d}.json",write_rollup(root,plan))
    finalize(root,plan,args.plot_python)
    print(f"T1 expanded 400 COMPLETE: {root}",flush=True)
    return 0


if __name__=="__main__": raise SystemExit(main())
