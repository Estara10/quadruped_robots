#!/usr/bin/env python3
"""Analysis for the T1 new-M2 18-map x 6-threshold 216-run local scan.

Reads the frozen plan and each item's last valid attempt; never reruns a scene.
Produces per-run, 108-cell, layer and overall tables, map-cluster bootstrap
differences versus X=-0.07, collision context, candidate segments and charts.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import statistics
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
XS = (-0.07, -0.08, -0.09, -0.10, -0.11, -0.12)
REF_X = -0.07
LAYERS = (("easy", 3), ("middle", 6), ("difficult", 9))
BOOTSTRAP_SEED = 2026101802
BOOTSTRAP_N = 10000
F_REF = 257.8
TIME_CAP = 20.0
TOLERANCE_PP = 5.0
SWITCH_IMPROVEMENT = 0.20

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#6250d6"]
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
MUTED = "#52514e"
GRID = "#dddcd6"


def load_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def fnum(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def mean(values):
    values = [v for v in values if v is not None]
    return statistics.mean(values) if values else None


def pct(value):
    return f"{value * 100:.1f}%" if value is not None else "UNKNOWN"


def interval(values):
    values = sorted(values)
    if not values:
        return None, None
    return values[int(0.025 * (len(values) - 1))], values[int(0.975 * (len(values) - 1))]


def write_csv(path: Path, rows: list[dict], encoding="utf-8") -> None:
    if not rows:
        path.write_text("", encoding=encoding)
        return
    fieldnames = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding=encoding) as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def state_name(payload: dict) -> str:
    return str(payload.get("policy_state_name") or "").upper()


def read_frames(run_dir: Path) -> list[dict]:
    path = run_dir / "runtime_record.jsonl"
    if not path.is_file():
        return []
    frames = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if row.get("kind") == "frame" and isinstance(row.get("payload"), dict):
            payload = row["payload"]
            if fnum(payload.get("sim_time_s")) is not None:
                frames.append(payload)
    return sorted(frames, key=lambda p: float(p["sim_time_s"]))


def exit_reentry(frames: list[dict], window_s: float = 0.5) -> dict:
    active = [f for f in frames if f.get("rl_entered") == 1 and f.get("rl_active") == 1]
    exits = []
    for previous, current in zip(active, active[1:]):
        if state_name(previous) == "RECOVERY" and state_name(current) == "AGILE":
            exits.append(current)
    reentries = nonreentries = censored = unknown = 0
    for exit_frame in exits:
        start = float(exit_frame["sim_time_s"])
        following = [f for f in active if float(f["sim_time_s"]) > start]
        if not following:
            censored += 1
            continue
        window = [f for f in following if float(f["sim_time_s"]) <= start + window_s]
        if any(state_name(f) == "RECOVERY" for f in window):
            reentries += 1
            continue
        if float(following[-1]["sim_time_s"]) < start + window_s:
            censored += 1
            continue
        gaps = [float(b["sim_time_s"]) - float(a["sim_time_s"])
                for a, b in zip(window, window[1:])]
        if gaps and max(gaps) > 0.1:
            unknown += 1
            continue
        nonreentries += 1
    eligible = reentries + nonreentries
    return {"exit_count": len(exits), "reentries_within_0_5s": reentries,
            "nonreentries_observed": nonreentries, "censored_exits": censored,
            "unknown_window_exits": unknown,
            "reentry_ratio": (reentries / eligible) if eligible else None}


def collision_context(run_dir: Path, metrics: dict, frames: list[dict]) -> dict | None:
    safety = metrics.get("task_window_safety") or {}
    events = safety.get("collision_events") or []
    if not events:
        return None
    event = events[0]
    start = fnum(event.get("start_sim_time"))
    if start is None:
        return None
    active = [f for f in frames if f.get("rl_entered") == 1 and f.get("rl_active") == 1]
    before = [f for f in active if float(f["sim_time_s"]) <= start]
    last = before[-1] if before else None
    exits = []
    for previous, current in zip(active, active[1:]):
        if (state_name(previous) == "RECOVERY" and state_name(current) == "AGILE"
                and float(current["sim_time_s"]) <= start):
            exits.append(current)
    last_exit = exits[-1] if exits else None
    recovery_frames = sum(1 for f in before if state_name(f) == "RECOVERY")
    return {
        "obstacle": event.get("obstacle_geom_name"),
        "robot_body": event.get("robot_body_name"),
        "start_sim_s": start,
        "end_sim_s": event.get("end_sim_time"),
        "start_physics_step": event.get("start_physics_step"),
        "end_physics_step": event.get("end_physics_step"),
        "last_policy_frame_sim_s": fnum(last.get("sim_time_s")) if last else None,
        "last_policy_frame_state": state_name(last) if last else None,
        "last_policy_frame_ra": fnum(last.get("ra_value")) if last else None,
        "last_policy_frame_ra_ge_E": (fnum(last.get("ra_value")) >= -0.05) if last and fnum(last.get("ra_value")) is not None else None,
        "recovery_frames_before_collision": recovery_frames,
        "no_recovery_before_collision": recovery_frames == 0,
        "last_recovery_to_agile_sim_s": fnum(last_exit.get("sim_time_s")) if last_exit else None,
        "last_recovery_to_agile_ra": fnum(last_exit.get("ra_value")) if last_exit else None,
        "time_since_last_recovery_exit_s": (start - float(last_exit["sim_time_s"])) if last_exit else None,
        "policy_to_collision_gap_s": (start - float(last["sim_time_s"])) if last else None,
    }


def score_run(row: dict) -> dict:
    if row["status"] != "VALID_TASK_RESULT" or row["record_validity"] != "VALID":
        return {"score_status": "UNKNOWN", "score_missing_reasons": "not_valid_task_record",
                "total_score_100": None}
    collision = row.get("nonfoot_collision")
    fall = row.get("fall")
    frequency = row.get("short_reverse_per_valid_min")
    elapsed = row.get("task_duration_s")
    reasons = []
    if collision not in (True, False):
        reasons.append("nonfoot_collision_UNKNOWN")
    if fall not in (True, False):
        reasons.append("fall_UNKNOWN")
    if elapsed is None:
        reasons.append("task_duration_missing")
    if row.get("short_reverse_switch_coverage") != "complete" or frequency is None:
        reasons.append("short_reverse_incomplete")
    if reasons:
        return {"score_status": "UNKNOWN", "score_missing_reasons": ";".join(reasons),
                "total_score_100": None}
    arrived = row["terminal"] == "ARRIVED"
    arrival = 1.0 if arrived else 0.0
    safe = 1.0 if (collision is False and fall is False) else 0.0
    efficiency = max(0.0, 1.0 - elapsed / TIME_CAP) if arrived else 0.0
    stability = 1.0 / (1.0 + frequency / F_REF)
    total = 100.0 * (0.4 * arrival + 0.3 * safe + 0.2 * efficiency + 0.1 * stability)
    return {"score_status": "COMPLETE", "score_missing_reasons": "",
            "arrival_score_0_1": arrival, "no_nonfoot_collision_or_fall_score_0_1": safe,
            "efficiency_score_0_1": efficiency, "stability_score_0_1": stability,
            "total_score_100": total}


def extract_run(item: dict, attempt: dict) -> dict:
    run_dir = Path(attempt["run_directory"]) if attempt.get("run_directory") else None
    metrics = load_json(run_dir / "metrics.json") if run_dir else {}
    context = load_json(run_dir / "run_context.json") if run_dir else {}
    safety = metrics.get("task_window_safety") or {}
    source = ((context.get("runtime_source_validation") or {}).get("evidence") or {})
    lifecycle = (context.get("lifecycle") or {}).get("standing_observation") or {}
    cycle0 = (context.get("initial_state") or {}).get("first_active_policy_cycle") or {}
    frames = read_frames(run_dir) if run_dir else []
    duration = fnum(metrics.get("valid_motion_duration_s"))
    count = fnum(metrics.get("short_reverse_switch_count"))
    coverage = metrics.get("short_reverse_switch_coverage")
    frequency = (count / (duration / 60.0)) if (duration and duration > 0 and count is not None
                                                and coverage == "complete") else None
    row = {
        "sequence": item["sequence"], "plan_item_id": item["plan_item_id"],
        "round": item["round"], "block_index": item["block_index"], "position": item["position"],
        "map": item["map"], "scene_id": item["scene_id"],
        "difficulty": item["difficulty_label"], "source_set": item["source_set"],
        "source_map": item["source_filename"], "layout_revision": item.get("layout_revision"),
        "exit_threshold_X": item["exit_threshold"], "run_id": attempt.get("run_id"),
        "attempt": attempt.get("attempt_number"), "terminal": attempt.get("terminal_result"),
        "record_validity": attempt.get("record_validity"),
        "config_consumption": (context.get("runtime_source_validation") or {}).get("status"),
        "actual_group": source.get("group"), "actual_entry_threshold": source.get("entry_threshold"),
        "actual_exit_threshold": source.get("exit_threshold"), "actual_hold": source.get("hold"),
        "arrival_distance_m": fnum(metrics.get("target_distance_final_m")),
        "task_duration_s": fnum(metrics.get("elapsed_task_duration_s")),
        "path_length_m": fnum(metrics.get("path_length_m")),
        "policy_frequency_hz": fnum(metrics.get("policy_frequency_hz_overall")),
        "cycle_gaps": metrics.get("motion_policy_cycle_gaps"),
        "invalid_clock_frames": metrics.get("invalid_clock_frames"),
        "recovery_entries": fnum(metrics.get("agile_to_recovery")),
        "recovery_exits": fnum(metrics.get("recovery_to_agile")),
        "recovery_share": fnum(metrics.get("recovery_share_valid_motion_time")),
        "total_switches": fnum(metrics.get("mode_transitions")),
        "short_reversals": count,
        "short_reverse_switch_coverage": coverage,
        "short_reverse_per_valid_min": frequency,
        "valid_task_time_min": (duration / 60.0) if duration else None,
        "foot_impact_count": fnum(metrics.get("foot_impact_count")),
        "nonfoot_collision": safety.get("nonfoot_collision_failure"),
        "collision": safety.get("collision"), "fall": safety.get("fall"),
        "safety_coverage_complete": safety.get("coverage_complete_for_motion_interval"),
        "safety_coverage_reason": safety.get("coverage_reason"),
        "outside_physics_steps": fnum((metrics.get("declared_boundary_physics_observation") or {}).get("outside_physics_step_delta")),
        "bounds_status": (metrics.get("declared_boundary_physics_observation") or {}).get("status"),
        "standing_observation": metrics.get("standing_observation_status"),
        "standing_requested_s": lifecycle.get("requested_sim_time_s"),
        "standing_observed_s": lifecycle.get("observed_elapsed_sim_s"),
        "shutdown_complete": attempt.get("process_shutdown_complete"),
        "closeout_complete": attempt.get("closeout_complete"),
        "mujoco_started": attempt.get("mujoco_started"),
        "window_status": (attempt.get("muJoCo_window_log") or {}).get("status"),
        "window_line": (attempt.get("muJoCo_window_log") or {}).get("log_line"),
        "initial_xy": json.dumps(cycle0.get("world_position_xy_m")),
        "initial_yaw": cycle0.get("world_yaw_rad"),
        "initial_sim_time_s": cycle0.get("sim_time_s"),
        "status": item.get("status"),
        "failure_reasons": "; ".join(item.get("failure_reasons") or []),
    }
    row.update(exit_reentry(frames))
    collision = collision_context(run_dir, metrics, frames) if run_dir else None
    row["collision_context_status"] = "CAPTURED" if collision else "NOT_APPLICABLE_OR_UNKNOWN"
    row.update({f"collision_{k}": v for k, v in (collision or {}).items()})
    row.update(score_run(row))
    return row


def aggregate_cell(rows: list[dict]) -> dict:
    valid = [r for r in rows if r["status"] == "VALID_TASK_RESULT" and r["record_validity"] == "VALID"]
    terminals = Counter(r["terminal"] for r in valid)
    nonfoot_known = [r["nonfoot_collision"] for r in valid if r["nonfoot_collision"] in (True, False)]
    nonfoot_unknown = sum(1 for r in valid if r["nonfoot_collision"] not in (True, False))
    fall_known = [r["fall"] for r in valid if r["fall"] in (True, False)]
    fall_unknown = sum(1 for r in valid if r["fall"] not in (True, False))
    return {
        "planned": len(rows), "valid": len(valid),
        "ARRIVED": terminals.get("ARRIVED", 0),
        "COLLISION_TERMINATION": terminals.get("COLLISION_TERMINATION", 0),
        "FALL_TERMINATION": terminals.get("FALL_TERMINATION", 0),
        "DIAGNOSTIC_SIM_TIME_LIMIT": terminals.get("DIAGNOSTIC_SIM_TIME_LIMIT", 0),
        "WALL_CLOCK_GUARD_INCOMPLETE": terminals.get("WALL_CLOCK_GUARD_INCOMPLETE", 0),
        "arrival_rate": (terminals.get("ARRIVED", 0) / len(valid)) if valid else None,
        "nonfoot_collision_count": sum(1 for v in nonfoot_known if v),
        "nonfoot_collision_unknown": nonfoot_unknown,
        "collision_rate_known": (sum(1 for v in nonfoot_known if v) / len(nonfoot_known)) if nonfoot_known else None,
        "collision_rate_worst": ((sum(1 for v in nonfoot_known if v) + nonfoot_unknown) / len(valid)) if valid else None,
        "collision_rate_best": (sum(1 for v in nonfoot_known if v) / len(valid)) if valid else None,
        "fall_count": sum(1 for v in fall_known if v), "fall_unknown": fall_unknown,
        "foot_impact_total": sum(r["foot_impact_count"] for r in valid if r["foot_impact_count"] is not None),
        "foot_impact_unknown": sum(1 for r in valid if r["foot_impact_count"] is None),
        "recovery_share_mean": mean(r["recovery_share"] for r in valid),
        "recovery_entries_mean": mean(r["recovery_entries"] for r in valid),
        "recovery_exits_mean": mean(r["recovery_exits"] for r in valid),
        "short_reverse_count_mean": mean(r["short_reversals"] for r in valid),
        "total_switches_mean": mean(r["total_switches"] for r in valid),
        "short_reverse_per_valid_min_mean": mean(r["short_reverse_per_valid_min"] for r in valid),
        "short_reverse_known": sum(1 for r in valid if r["short_reverse_per_valid_min"] is not None),
        "arrival_time_mean_s": mean(r["task_duration_s"] for r in valid if r["terminal"] == "ARRIVED"),
        "arrival_time_known": sum(1 for r in valid if r["terminal"] == "ARRIVED" and r["task_duration_s"] is not None),
        "score_mean_100": mean(r["total_score_100"] for r in valid),
        "score_known": sum(1 for r in valid if r["total_score_100"] is not None),
        "reentry_ratio_mean": mean(r["reentry_ratio"] for r in valid),
        "exit_count_total": sum(r["exit_count"] or 0 for r in valid),
        "exit_reentry_known_runs": sum(1 for r in valid if r["reentry_ratio"] is not None),
        "safety_coverage_complete": sum(1 for r in valid if r["safety_coverage_complete"] is True),
        "standing_complete": sum(1 for r in valid if r["standing_observation"] == "COMPLETE"),
        "cleanup_complete": sum(1 for r in valid if r["shutdown_complete"] is True and r["closeout_complete"] is True),
    }


def map_metric(rows: list[dict], key: str) -> float | None:
    values = [r[key] for r in rows if r.get(key) is not None]
    return mean(values)


def build_map_table(rows: list[dict]) -> list[dict]:
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row["map"], row["difficulty"], row["exit_threshold_X"])].append(row)
    out = []
    for (map_name, difficulty, x), cell_rows in sorted(
            grouped.items(), key=lambda kv: (kv[0][2], kv[0][0])):
        cell = aggregate_cell(cell_rows)
        out.append({"map": map_name, "difficulty": difficulty, "X": x, **cell})
    return out


def build_threshold_summary(rows: list[dict], layer: str | None = None) -> list[dict]:
    maps = sorted({r["map"] for r in rows
                   if layer is None or r["difficulty"] == layer})
    out = []
    for x in XS:
        cell_by_map = {}
        for map_name in maps:
            cell_rows = [r for r in rows if r["map"] == map_name and r["exit_threshold_X"] == x]
            cell_by_map[map_name] = aggregate_cell(cell_rows)
        def avg(key):
            return mean(cell.get(key) for cell in cell_by_map.values())
        out.append({
            "layer": layer or "all", "X": x, "maps": len(maps),
            "planned": sum(c["planned"] for c in cell_by_map.values()),
            "valid": sum(c["valid"] for c in cell_by_map.values()),
            "ARRIVED": sum(c["ARRIVED"] for c in cell_by_map.values()),
            "COLLISION_TERMINATION": sum(c["COLLISION_TERMINATION"] for c in cell_by_map.values()),
            "FALL_TERMINATION": sum(c["FALL_TERMINATION"] for c in cell_by_map.values()),
            "DIAGNOSTIC_SIM_TIME_LIMIT": sum(c["DIAGNOSTIC_SIM_TIME_LIMIT"] for c in cell_by_map.values()),
            "WALL_CLOCK_GUARD_INCOMPLETE": sum(c["WALL_CLOCK_GUARD_INCOMPLETE"] for c in cell_by_map.values()),
            "arrival_rate": avg("arrival_rate"),
            "collision_rate_known": avg("collision_rate_known"),
            "collision_rate_best": avg("collision_rate_best"),
            "collision_rate_worst": avg("collision_rate_worst"),
            "fall_count": sum(c["fall_count"] for c in cell_by_map.values()),
            "fall_unknown": sum(c["fall_unknown"] for c in cell_by_map.values()),
            "foot_impact_total": sum(c["foot_impact_total"] for c in cell_by_map.values()),
            "recovery_share_mean": avg("recovery_share_mean"),
            "recovery_entries_mean": avg("recovery_entries_mean"),
            "recovery_exits_mean": avg("recovery_exits_mean"),
            "short_reverse_per_valid_min_mean": avg("short_reverse_per_valid_min_mean"),
            "short_reverse_count_mean": avg("short_reverse_count_mean"),
            "total_switches_mean": avg("total_switches_mean"),
            "arrival_time_mean_s": avg("arrival_time_mean_s"),
            "score_mean_100": avg("score_mean_100"),
            "reentry_ratio_mean": avg("reentry_ratio_mean"),
            "exit_count_total": sum(c["exit_count_total"] for c in cell_by_map.values()),
            "exit_reentry_known_runs": sum(c["exit_reentry_known_runs"] for c in cell_by_map.values()),
            "safety_coverage_complete": sum(c["safety_coverage_complete"] for c in cell_by_map.values()),
            "standing_complete": sum(c["standing_complete"] for c in cell_by_map.values()),
            "cleanup_complete": sum(c["cleanup_complete"] for c in cell_by_map.values()),
        })
    return out


def bootstrap_differences(rows: list[dict], map_rows: list[dict]) -> dict:
    """Stratified map-cluster bootstrap of threshold - reference differences."""
    cell = {(row["map"], row["X"]): row for row in map_rows}
    strata = {layer: sorted({row["map"] for row in map_rows if row["difficulty"] == layer})
              for layer, _ in LAYERS}
    metrics = ("arrival_rate", "collision_rate_worst", "collision_rate_best",
               "short_reverse_per_valid_min_mean", "recovery_share_mean", "score_mean_100")
    map_names = sorted({row["map"] for row in map_rows})
    maps = {(x, key): {map_name: (cell.get((map_name, x)) or {}).get(key)
                       for map_name in map_names}
            for x in XS for key in metrics}
    rng = random.Random(BOOTSTRAP_SEED)
    samples = {x: {key: [] for key in metrics} for x in XS if x != REF_X}
    for _ in range(BOOTSTRAP_N):
        sampled = []
        for layer, _ in LAYERS:
            pool = strata[layer]
            sampled.extend(rng.choices(pool, k=len(pool)))
        for x in XS:
            if x == REF_X:
                continue
            for key in metrics:
                pairs = [(maps[(x, key)].get(m), maps[(REF_X, key)].get(m)) for m in sampled]
                pairs = [(a, b) for a, b in pairs if a is not None and b is not None]
                if pairs:
                    samples[x][key].append(mean([a for a, _ in pairs]) - mean([b for _, b in pairs]))
    out = {}
    by_x = {row["X"]: row for row in build_threshold_summary(rows)}
    ref_summary = by_x[REF_X]
    for x in XS:
        if x == REF_X:
            continue
        summary = by_x[x]
        item = {"X": x}
        for key in metrics:
            point = None
            if summary.get(key) is not None and ref_summary.get(key) is not None:
                point = summary[key] - ref_summary[key]
            low, high = interval(samples[x][key])
            item[f"{key}_difference"] = point
            item[f"{key}_ci95_low"] = low
            item[f"{key}_ci95_high"] = high
        out[x] = item
    return out


def paired_arrival_time(rows: list[dict]) -> list[dict]:
    by_key = {(r["map"], r["round"], r["exit_threshold_X"]): r for r in rows}
    out = []
    for map_name in sorted({r["map"] for r in rows}):
        for x in XS:
            if x == REF_X:
                continue
            diffs = []
            pairs = 0
            for repeat in (1, 2):
                a = by_key.get((map_name, repeat, x))
                b = by_key.get((map_name, repeat, REF_X))
                if (a and b and a["terminal"] == "ARRIVED" and b["terminal"] == "ARRIVED"
                        and a["task_duration_s"] is not None and b["task_duration_s"] is not None):
                    diffs.append(a["task_duration_s"] - b["task_duration_s"])
                    pairs += 1
            out.append({"map": map_name, "X": x, "paired_arrivals": pairs,
                        "mean_time_difference_s": mean(diffs),
                        "interpretation": "same map and repeat; negative means X is faster"})
    return out


def map_deltas(rows: list[dict]) -> list[dict]:
    out = []
    ref = {s["X"]: s for s in build_threshold_summary(rows)}[REF_X]
    for x in XS:
        if x == REF_X:
            continue
        summary = {s["X"]: s for s in build_threshold_summary(rows)}[x]
        for map_name in sorted({r["map"] for r in rows}):
            a = aggregate_cell([r for r in rows if r["map"] == map_name and r["exit_threshold_X"] == x])
            b = aggregate_cell([r for r in rows if r["map"] == map_name and r["exit_threshold_X"] == REF_X])
            def diff(key):
                if a.get(key) is None or b.get(key) is None:
                    return None
                return a[key] - b[key]
            rel = None
            if (a.get("short_reverse_per_valid_min_mean") is not None
                    and b.get("short_reverse_per_valid_min_mean")):
                rel = (a["short_reverse_per_valid_min_mean"]
                       - b["short_reverse_per_valid_min_mean"]) / b["short_reverse_per_valid_min_mean"]
            out.append({"map": map_name, "difficulty": next(r["difficulty"] for r in rows if r["map"] == map_name),
                        "X": x, "arrival_rate_difference": diff("arrival_rate"),
                        "collision_rate_difference": diff("collision_rate_known"),
                        "recovery_share_difference": diff("recovery_share_mean"),
                        "short_reverse_relative_difference": rel,
                        "arrival_time_difference_s": (
                            (a["arrival_time_mean_s"] - b["arrival_time_mean_s"])
                            if a.get("arrival_time_mean_s") is not None
                            and b.get("arrival_time_mean_s") is not None else None)})
    return out


def candidate_segments(summary_rows: list[dict], deltas: dict) -> dict:
    by_x = {row["X"]: row for row in summary_rows}
    ref = by_x[REF_X]
    assessments = []
    for x in XS:
        row = by_x[x]
        arrival_pp = (row["arrival_rate"] - ref["arrival_rate"]) * 100.0
        collision_known_pp = ((row["collision_rate_known"] - ref["collision_rate_known"]) * 100.0
                              if row["collision_rate_known"] is not None
                              and ref["collision_rate_known"] is not None else None)
        collision_worst_pp = ((row["collision_rate_worst"] - ref["collision_rate_worst"]) * 100.0
                              if row["collision_rate_worst"] is not None
                              and ref["collision_rate_worst"] is not None else None)
        ref_freq = ref["short_reverse_per_valid_min_mean"]
        freq = row["short_reverse_per_valid_min_mean"]
        relative = ((freq - ref_freq) / ref_freq) if ref_freq and freq is not None else None
        task_best = arrival_pp >= -TOLERANCE_PP and (collision_known_pp is not None and collision_known_pp <= TOLERANCE_PP)
        task_worst = arrival_pp >= -TOLERANCE_PP and (collision_worst_pp is not None and collision_worst_pp <= TOLERANCE_PP)
        task_ok = task_best and task_worst
        switch_ok = relative is not None and relative <= -SWITCH_IMPROVEMENT
        fall_free = (row["fall_count"] == 0 and row["fall_unknown"] == 0)
        evidence_insufficient = (task_best != task_worst) or row["valid"] != 36
        eligible = bool(task_ok and switch_ok and fall_free and not evidence_insufficient)
        assessments.append({
            "X": x, "arrival_rate": row["arrival_rate"], "arrival_pp_vs_ref": arrival_pp,
            "collision_rate_known": row["collision_rate_known"],
            "collision_known_pp_vs_ref": collision_known_pp,
            "collision_rate_worst": row["collision_rate_worst"],
            "collision_worst_pp_vs_ref": collision_worst_pp,
            "short_reverse_per_valid_min": freq, "short_reverse_relative_vs_ref": relative,
            "fall_count": row["fall_count"], "fall_unknown": row["fall_unknown"],
            "recovery_share_mean": row["recovery_share_mean"], "valid": row["valid"],
            "task_ok_best_case": task_best, "task_ok_worst_case": task_worst,
            "task_ok": task_ok, "switch_ok": switch_ok, "fall_free": fall_free,
            "evidence_insufficient": evidence_insufficient, "eligible": eligible,
            "difference_ci95": deltas.get(x, {})})
    segments = []
    start = None
    for index, assessment in enumerate(assessments):
        if assessment["eligible"] and start is None:
            start = index
        if (not assessment["eligible"] or index == len(assessments) - 1) and start is not None:
            end = index - 1 if not assessment["eligible"] else index
            if end - start + 1 >= 2:
                members = assessments[start:end + 1]
                middle = members[(len(members) - 1) // 2]
                if len(members) % 2 == 0:
                    left = members[len(members) // 2 - 1]
                    right = members[len(members) // 2]
                    representative = right if right["X"] > left["X"] else left
                else:
                    representative = middle
                segments.append({"X_values": [m["X"] for m in members],
                                 "length": len(members), "representative_X": representative["X"],
                                 "representative_rule": "middle discrete value; even length prefers narrower hysteresis (X closer to E)",
                                 "members": members})
            start = None
    if not segments:
        eligible = [a for a in assessments if a["task_ok"] and a["fall_free"]]
        eligible.sort(key=lambda a: (-a["arrival_rate"], a["collision_rate_known"] or 9,
                                     a["short_reverse_per_valid_min"] or 9,
                                     a["recovery_share_mean"] or 9))
        fallback = eligible[0] if eligible else None
        return {"segments": [], "supported": False,
                "message": "不支持明确稳定区间：没有连续至少两个相邻档同时满足任务容忍度与短反向下降20%。",
                "fallback_engineering_representative": fallback,
                "fallback_note": "仅按任务表现优先、切换其次、占用/效率辅助提出，未通过区间规则。",
                "assessments": assessments}
    return {"segments": segments, "supported": True,
            "message": "存在连续相邻候选段；仅代表已采样离散网格，不声称未测数值均鲁棒。",
            "assessments": assessments}


def recovery_residence(plan: dict, out: Path) -> None:
    rows = []
    difficulty = {item["map"]: item["difficulty_label"] for item in plan["items"]}
    for layer in (None, "easy", "middle", "difficult"):
        for x in XS:
            durations = []
            censored = 0
            runs = 0
            for item in plan["items"]:
                if abs(float(item["exit_threshold"]) - x) > 1e-9:
                    continue
                if layer is not None and difficulty[item["map"]] != layer:
                    continue
                if not item.get("attempts"):
                    continue
                rd = Path(item["attempts"][-1]["run_directory"])
                metrics = load_json(rd / "metrics.json")
                intervals = metrics.get("recovery_intervals") or []
                if intervals:
                    runs += 1
                for interval in intervals:
                    duration = fnum(interval.get("duration_s"))
                    if duration is None:
                        continue
                    if interval.get("censored"):
                        censored += 1
                    else:
                        durations.append(duration)
            ordered = sorted(durations)
            rows.append({
                "layer": layer or "all", "X": x, "runs_with_recovery": runs,
                "uncensored_intervals": len(durations), "censored_intervals": censored,
                "mean_duration_s": mean(durations),
                "median_duration_s": statistics.median(durations) if durations else None,
                "p90_duration_s": ordered[int(0.9 * (len(ordered) - 1))] if ordered else None,
                "min_duration_s": min(durations) if durations else None,
                "max_duration_s": max(durations) if durations else None,
            })
    write_csv(out / "recovery_residence.csv", rows)


def plots(root: Path, summary_rows, layer_rows, map_table, candidate) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    chart_dir = root / "charts"
    chart_dir.mkdir(exist_ok=True)

    def style(ax):
        ax.set_facecolor(SURFACE)
        ax.grid(color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.tick_params(colors=MUTED, labelsize=8)

    xs = [row["X"] for row in summary_rows]
    figure, axes = plt.subplots(1, 2, figsize=(10.5, 4.2), facecolor=SURFACE)
    for ax, key, title, ylabel in (
            (axes[0], "arrival_rate", "Arrival proportion", "proportion"),
            (axes[1], "collision_rate_known", "Non-foot obstacle collision proportion", "proportion")):
        ax.plot(xs, [row[key] * 100 if row[key] is not None else math.nan for row in summary_rows],
                marker="o", color=SERIES[0], linewidth=2, markersize=5, label="all 18 maps")
        for layer, color in zip(("easy", "middle", "difficult"), SERIES[1:4]):
            rows = [r for r in layer_rows if r["layer"] == layer]
            ax.plot([r["X"] for r in rows],
                    [r[key] * 100 if r[key] is not None else math.nan for r in rows],
                    marker="o", color=color, linewidth=1.5, markersize=4, label=layer)
        style(ax)
        ax.set_title(title, fontsize=10, loc="left")
        ax.set_ylabel("%", fontsize=8)
        ax.set_xlabel("X", fontsize=8)
        ax.invert_xaxis()
    axes[0].legend(frameon=False, fontsize=7)
    figure.tight_layout()
    figure.savefig(chart_dir / "arrival_collision_vs_X.png", dpi=160, facecolor=SURFACE)
    plt.close(figure)

    figure, axes = plt.subplots(1, 2, figsize=(10.5, 4.2), facecolor=SURFACE)
    for ax, key, title in (
            (axes[0], "short_reverse_per_valid_min_mean", "Short reverse switches per valid minute"),
            (axes[1], "recovery_share_mean", "Recovery share of valid task time")):
        ax.plot(xs, [row[key] if row[key] is not None else math.nan for row in summary_rows],
                marker="o", color=SERIES[0], linewidth=2, markersize=5, label="all 18 maps")
        for layer, color in zip(("easy", "middle", "difficult"), SERIES[1:4]):
            rows = [r for r in layer_rows if r["layer"] == layer]
            ax.plot([r["X"] for r in rows],
                    [r[key] if r[key] is not None else math.nan for r in rows],
                    marker="o", color=color, linewidth=1.5, markersize=4, label=layer)
        style(ax)
        ax.set_title(title, fontsize=10, loc="left")
        ax.set_xlabel("X", fontsize=8)
        ax.invert_xaxis()
    axes[0].legend(frameon=False, fontsize=7)
    figure.tight_layout()
    figure.savefig(chart_dir / "switching_recovery_vs_X.png", dpi=160, facecolor=SURFACE)
    plt.close(figure)

    figure, axes = plt.subplots(1, 2, figsize=(10.5, 4.2), facecolor=SURFACE)
    for ax, key, title in ((axes[0], "DIAGNOSTIC_SIM_TIME_LIMIT", "Sim-time-limit count"),
                           (axes[1], "WALL_CLOCK_GUARD_INCOMPLETE", "Wall-clock guard count")):
        ax.bar([str(x) for x in xs], [row[key] for row in summary_rows], color=SERIES[0],
               edgecolor=SURFACE, linewidth=1.2)
        style(ax)
        ax.set_title(title, fontsize=10, loc="left")
    figure.tight_layout()
    figure.savefig(chart_dir / "timeouts_vs_X.png", dpi=160, facecolor=SURFACE)
    plt.close(figure)

    maps = sorted({row["map"] for row in map_table})
    for key, filename, title in (
            ("arrival_rate", "heatmap_arrival.png", "Arrival proportion by map and X"),
            ("collision_rate_known", "heatmap_collision.png", "Non-foot collision proportion by map and X")):
        values = []
        for map_name in maps:
            values.append([next((row[key] for row in map_table
                                 if row["map"] == map_name and row["X"] == x), None)
                           for x in xs])
        figure, ax = plt.subplots(figsize=(7.0, 7.5), facecolor=SURFACE)
        masked = [[math.nan if v is None else v * 100 for v in row] for row in values]
        image = ax.imshow(masked, aspect="auto", cmap="Blues", vmin=0, vmax=100)
        ax.set_xticks(range(len(xs)))
        ax.set_xticklabels([f"{x:.2f}" for x in xs])
        ax.set_yticks(range(len(maps)))
        ax.set_yticklabels(maps)
        ax.set_title(title, fontsize=10, loc="left")
        figure.colorbar(image, ax=ax, fraction=0.046, pad=0.04)
        figure.tight_layout()
        figure.savefig(chart_dir / filename, dpi=160, facecolor=SURFACE)
        plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    plan = load_json(args.plan)
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    collisions = []
    for item in plan["items"]:
        if not item.get("attempts"):
            continue
        attempt = item["attempts"][-1]
        row = extract_run(item, attempt)
        rows.append(row)
        if row.get("collision_start_sim_s") is not None:
            collisions.append({k: v for k, v in row.items() if k.startswith("collision_")
                               or k in ("map", "difficulty", "exit_threshold_X", "run_id")})
    if len(rows) != 216:
        raise ValueError(f"expected 216 run rows, found {len(rows)}")
    write_csv(out / "per_run_results.csv", rows)
    map_table = build_map_table(rows)
    write_csv(out / "map_threshold_108.csv", map_table)
    summary = build_threshold_summary(rows)
    write_csv(out / "threshold_summary_6.csv", summary)
    layer_rows = []
    for layer, _ in LAYERS:
        layer_rows.extend(build_threshold_summary(rows, layer))
    write_csv(out / "layer_summary_18.csv", layer_rows)
    recovery_residence(plan, out)
    deltas = bootstrap_differences(rows, map_table)
    delta_rows = []
    for x in XS:
        row = {"X": x}
        if x == REF_X:
            row.update({"reference": True, "note": "reference runs of this batch"})
        else:
            row.update(deltas[x])
        delta_rows.append(row)
    write_csv(out / "threshold_deltas_vs_ref.csv", delta_rows)
    write_csv(out / "map_deltas_vs_ref.csv", map_deltas(rows))
    write_csv(out / "paired_arrival_time_diff.csv", paired_arrival_time(rows))
    write_csv(out / "collision_context.csv", collisions)
    candidate = candidate_segments(summary, deltas)
    (out / "candidate_segments.json").write_text(
        json.dumps(candidate, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    plots(out, summary, layer_rows, map_table, candidate)

    lines = ["# T1 新版M2六档退出阈值216项局部扫描：自动汇总", ""]
    lines.append(f"- 计划项：{len(plan['items'])}；有效记录：{len(rows)}")
    lines.append(f"- 固定：B / E=-0.05 / X=-0.07至-0.12 / 保持关闭；目标 (7.5,0)，半径 0.5 m")
    lines.append(f"- 时限：20仿真秒/120墙钟秒；到达后站立2仿真秒；隐藏GLFW、RViz关闭")
    lines.append("")
    lines.append("## 六档总览（18图地图等权）")
    lines.append("| X | 有效 | 到达 | 非足端碰撞 | 跌倒 | 仿真时限 | 墙钟 | 到达率 | 碰撞率(已知) | 短反向/有效分钟 | Recovery占比 | 辅助分 |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for row in summary:
        def fmt(value, digits=2):
            return f"{value:.{digits}f}" if value is not None else "UNKNOWN"
        lines.append("| " + " | ".join(str(v) for v in (
            f"{row['X']:.2f}", row["valid"], row["ARRIVED"], row["COLLISION_TERMINATION"],
            row["FALL_TERMINATION"], row["DIAGNOSTIC_SIM_TIME_LIMIT"],
            row["WALL_CLOCK_GUARD_INCOMPLETE"], fmt(row["arrival_rate"], 3),
            fmt(row["collision_rate_known"], 3), fmt(row["short_reverse_per_valid_min_mean"], 1),
            fmt(row["recovery_share_mean"], 3), fmt(row["score_mean_100"], 1))) + " |")
    lines.append("")
    lines.append("## 候选段")
    lines.append(candidate["message"])
    if candidate["segments"]:
        for segment in candidate["segments"]:
            lines.append(f"- X={segment['X_values']}；代表值 X={segment['representative_X']:.2f}")
    elif candidate.get("fallback_engineering_representative"):
        fallback = candidate["fallback_engineering_representative"]
        lines.append(f"- 工程备选：X={fallback['X']:.2f}（未通过区间规则）")
    lines.append("")
    lines.append("## 解释边界")
    lines.append("单次/两次重复是开发数据；不估计总体成功率，不证明统计非劣效或全局最优。")
    lines.append("UNKNOWN不补成False；碰撞/跌倒/足端/越界与UNKNOWN独立保留。")
    (out / "RESULTS_SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"analysis written to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
