#!/usr/bin/env python3
"""Analysis for the R1 three-map A/C hold-time confirmation (120 runs)."""
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
E = -0.05
LEVELS = (("A", None), ("C", 0.02), ("C", 0.10), ("C", 0.12))
LEVEL_KEYS = ["A", "C_H0.02", "C_H0.10", "C_H0.12"]
LEVEL_NUM = {key: index for index, key in enumerate(LEVEL_KEYS)}
REF_KEY = "A"
LAYERS = (("difficult", 2), ("middle", 1))
BOOTSTRAP_SEED = 2026110102
BOOTSTRAP_N = 10000
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
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


def interval(values):
    values = sorted(values)
    if not values:
        return None, None
    return values[int(0.025 * (len(values) - 1))], values[int(0.975 * (len(values) - 1))]


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def level_key(group: str, hold: float | None) -> str:
    return "A" if group == "A" else f"C_H{hold:.2f}"


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


def state(payload: dict) -> int | None:
    value = payload.get("policy_state")
    return int(value) if isinstance(value, (int, float)) else None


def exit_reentry(frames: list[dict], window_s: float = 0.5) -> dict:
    active = [f for f in frames if f.get("rl_entered") == 1 and f.get("rl_active") == 1]
    exits = [current for previous, current in zip(active, active[1:])
             if state(previous) == 1 and state(current) == 0]
    reentries = nonreentries = censored = unknown = 0
    for exit_frame in exits:
        start = float(exit_frame["sim_time_s"])
        following = [f for f in active if float(f["sim_time_s"]) > start]
        if not following:
            censored += 1
            continue
        window = [f for f in following if float(f["sim_time_s"]) <= start + window_s]
        if any(state(f) == 1 for f in window):
            reentries += 1
        elif float(following[-1]["sim_time_s"]) < start + window_s:
            censored += 1
        else:
            gaps = [float(b["sim_time_s"]) - float(a["sim_time_s"]) for a, b in zip(window, window[1:])]
            if gaps and max(gaps) > 0.1:
                unknown += 1
            else:
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
    exits = [current for previous, current in zip(active, active[1:])
             if state(previous) == 1 and state(current) == 0
             and float(current["sim_time_s"]) <= start]
    last_exit = exits[-1] if exits else None
    recovery_frames = sum(1 for f in before if state(f) == 1)
    return {
        "obstacle": event.get("obstacle_geom_name"),
        "robot_body": event.get("robot_body_name"),
        "start_sim_s": start,
        "end_sim_s": event.get("end_sim_time"),
        "start_physics_step": event.get("start_physics_step"),
        "end_physics_step": event.get("end_physics_step"),
        "last_policy_frame_sim_s": fnum(last.get("sim_time_s")) if last else None,
        "last_policy_frame_state": "RECOVERY" if last and state(last) == 1 else ("AGILE" if last else None),
        "last_policy_frame_ra": fnum(last.get("ra_value")) if last else None,
        "last_policy_frame_ra_ge_E": (fnum(last.get("ra_value")) >= E) if last and fnum(last.get("ra_value")) is not None else None,
        "recovery_frames_before_collision": recovery_frames,
        "no_recovery_before_collision": recovery_frames == 0,
        "last_recovery_to_agile_sim_s": fnum(last_exit.get("sim_time_s")) if last_exit else None,
        "last_recovery_to_agile_ra": fnum(last_exit.get("ra_value")) if last_exit else None,
        "time_since_last_recovery_exit_s": (start - float(last_exit["sim_time_s"])) if last_exit else None,
        "policy_to_collision_gap_s": (start - float(last["sim_time_s"])) if last else None,
    }


def hold_episodes(item: dict, attempt: dict) -> tuple[list[dict], list[dict], str]:
    run_dir = Path(attempt["run_directory"])
    metrics = load_json(run_dir / "metrics.json")
    audit = metrics.get("recovery_hold_audit") or {}
    frames = read_frames(run_dir)
    by_cycle = {int(f["rl_step"]): f for f in frames if isinstance(f.get("rl_step"), (int, float))}
    episodes = []
    for index, episode in enumerate(audit.get("episodes") or [], start=1):
        enter_cycle = episode.get("enter_cycle")
        exit_cycle = episode.get("exit_cycle")
        t_enter = fnum(episode.get("t_enter_sim_s"))
        H = fnum(episode.get("H_s"))
        exit_frame = by_cycle.get(int(exit_cycle)) if isinstance(exit_cycle, (int, float)) else None
        episode_frames = []
        for frame in frames:
            if t_enter is None or float(frame["sim_time_s"]) < t_enter:
                continue
            if enter_cycle is not None and int(frame.get("rl_step", -1)) < int(enter_cycle):
                continue
            if exit_cycle is not None:
                if int(frame.get("rl_step", -1)) <= int(exit_cycle):
                    episode_frames.append(frame)
                else:
                    break
            else:
                # An unclosed audit episode ends at the first non-Recovery
                # frame; post-terminal/closeout frames must not be folded in.
                if state(frame) == 1:
                    episode_frames.append(frame)
                else:
                    break
        later = episode_frames
        first_clear = next((f for f in later
                            if fnum(f.get("ra_value")) is not None
                            and float(f["ra_value"]) < E
                            and (state(f) == 1 or (exit_cycle is not None
                                 and int(f.get("rl_step", -1)) == int(exit_cycle)))), None)
        expiry = t_enter + H if t_enter is not None and H is not None else None
        first_eligible = None
        for frame in later:
            if (state(frame) == 1 and expiry is not None
                    and float(frame["sim_time_s"]) >= expiry
                    and frame.get("hold_status") == 3
                    and fnum(frame.get("ra_value")) is not None
                    and float(frame["ra_value"]) < E):
                first_eligible = frame
                break
        residence = (float(exit_frame["sim_time_s"]) - t_enter) if exit_frame and t_enter is not None else None
        if exit_frame is not None:
            reason = "exited"
            if residence is not None and H is not None and residence < H - 1e-9:
                reason = "exited_before_H"
            elif fnum(exit_frame.get("ra_value")) is not None and float(exit_frame["ra_value"]) >= E:
                reason = "exited_with_RA_not_below_E"
            elif first_eligible is not None and int(exit_frame.get("rl_step", -1)) > int(first_eligible.get("rl_step", -1)):
                reason = "exited_later_than_first_eligible_cycle"
        else:
            last = later[-1] if later else None
            if last is None:
                reason = "no_frames_after_entry"
            elif state(last) == 1:
                if expiry is not None and float(last["sim_time_s"]) < expiry:
                    reason = "censored_before_H_at_task_terminal"
                elif first_eligible is None:
                    reason = "risk_not_below_E_at_task_terminal"
                else:
                    reason = "did_not_exit_after_H_and_RA_below_E"
            else:
                reason = "no_exit_observed_unknown"
        expiry_frame = next((f for f in later if expiry is not None
                             and float(f["sim_time_s"]) >= expiry), None)
        ra_at_expiry = fnum(expiry_frame.get("ra_value")) if expiry_frame else None
        risk_high_at_expiry = ra_at_expiry is not None and ra_at_expiry >= E
        continued_high = bool(risk_high_at_expiry and (
            exit_cycle is None or int(expiry_frame.get("rl_step", -1)) < int(exit_cycle)))
        entry_time_changed = any(
            fnum(f.get("hold_enter_sim_time_s")) is not None and t_enter is not None
            and abs(float(f["hold_enter_sim_time_s"]) - t_enter) > 1e-9 for f in later)
        seen_low = False
        risk_re_rise = False
        for frame in later:
            ra = fnum(frame.get("ra_value"))
            if ra is None:
                continue
            if ra < E:
                seen_low = True
            elif seen_low:
                risk_re_rise = True
        episodes.append({
            "run_item": item["plan_item_id"], "run_id": attempt.get("run_id"),
            "level": level_key(item["group"], item["hold_s"]),
            "map": item["map"], "repeat": item["repeat"],
            "H_s": H, "episode_index": index, "enter_cycle": enter_cycle,
            "enter_sim_s": t_enter, "H_expiry_sim_s": expiry,
            "first_risk_clear_sim_s": fnum(first_clear.get("sim_time_s")) if first_clear else None,
            "first_eligible_after_H_sim_s": fnum(first_eligible.get("sim_time_s")) if first_eligible else None,
            "exit_cycle": exit_cycle, "exit_sim_s": fnum(exit_frame.get("sim_time_s")) if exit_frame else None,
            "observed_residence_s": residence,
            "residence_minus_H_s": (residence - H) if residence is not None and H is not None else None,
            "exit_ra": fnum(exit_frame.get("ra_value")) if exit_frame else None,
            "censored": bool(episode.get("censored")),
            "clock_coverage": episode.get("clock_coverage"),
            "hold_rule": episode.get("hold_rule"),
            "exit_reason": reason,
            "ra_at_expiry": ra_at_expiry,
            "risk_high_at_expiry": risk_high_at_expiry,
            "continued_recovery_after_expiry_high_risk": continued_high,
            "entry_time_changed_within_episode": entry_time_changed,
            "risk_re_rise_within_episode": risk_re_rise,
            "reentered_after_exit": None,
        })
    for episode in episodes:
        if episode["exit_sim_s"] is None:
            continue
        window = [f for f in frames if episode["exit_sim_s"] < float(f["sim_time_s"])
                  <= episode["exit_sim_s"] + 0.5]
        episode["reentered_after_exit"] = any(state(f) == 1 for f in window)
    return episodes, list(audit.get("violations") or []), audit.get("status")


def extract_row(item: dict, attempt: dict) -> tuple[dict, list[dict], dict | None]:
    run_dir = Path(attempt["run_directory"])
    metrics = load_json(run_dir / "metrics.json")
    context = load_json(run_dir / "run_context.json")
    safety = metrics.get("task_window_safety") or {}
    source = ((context.get("runtime_source_validation") or {}).get("evidence") or {})
    lifecycle = (context.get("lifecycle") or {}).get("standing_observation") or {}
    cycle0 = (context.get("initial_state") or {}).get("first_active_policy_cycle") or {}
    audit = metrics.get("recovery_hold_audit") or {}
    frames = read_frames(run_dir)
    duration = fnum(metrics.get("valid_motion_duration_s"))
    count = fnum(metrics.get("short_reverse_switch_count"))
    coverage = metrics.get("short_reverse_switch_coverage")
    frequency = (count / (duration / 60.0)) if (duration and duration > 0 and count is not None
                                                and coverage == "complete") else None
    reentry = exit_reentry(frames)
    collision = collision_context(run_dir, metrics, frames)
    row = {
        "sequence": item["sequence"], "plan_item_id": item["plan_item_id"],
        "repeat": item["repeat"], "block_index": item["block_index"], "position": item["position"],
        "map": item["map"], "scene_id": item["scene_id"], "difficulty": item["difficulty_label"],
        "source_set": item["source_set"], "source_map": item["source_filename"],
        "layout_revision": item.get("layout_revision"),
        "level": level_key(item["group"], item["hold_s"]),
        "level_num": LEVEL_NUM[level_key(item["group"], item["hold_s"])],
        "group": item["group"], "H_s": item["hold_s"], "run_id": attempt.get("run_id"),
        "attempt": attempt.get("attempt_number"), "terminal": attempt.get("terminal_result"),
        "record_validity": attempt.get("record_validity"),
        "config_source": (context.get("runtime_source_validation") or {}).get("status"),
        "actual_group": source.get("group"), "actual_entry": source.get("entry_threshold"),
        "actual_exit": source.get("exit_threshold"), "actual_hold": source.get("hold"),
        "actual_recovery_min_hold_s": source.get("recovery_min_hold_s"),
        "hold_audit_status": audit.get("status"),
        "hold_episode_count": len(audit.get("episodes") or []),
        "hold_violation_count": len(audit.get("violations") or []),
        "arrival_distance_m": fnum(metrics.get("target_distance_final_m")),
        "task_duration_s": fnum(metrics.get("elapsed_task_duration_s")),
        "path_length_m": fnum(metrics.get("path_length_m")),
        "safe_completion": metrics.get("safety_task_completion_status", "UNKNOWN"),
        "policy_frequency_hz": fnum(metrics.get("policy_frequency_hz_overall")),
        "cycle_gaps": metrics.get("motion_policy_cycle_gaps"),
        "invalid_clock_frames": metrics.get("invalid_clock_frames"),
        "recovery_entries": fnum(metrics.get("agile_to_recovery")),
        "recovery_exits": fnum(metrics.get("recovery_to_agile")),
        "recovery_share": fnum(metrics.get("recovery_share_valid_motion_time")),
        "total_switches": fnum(metrics.get("mode_transitions")),
        "short_reversals": count, "short_reverse_switch_coverage": coverage,
        "short_reverse_per_valid_min": frequency,
        "foot_impact_count": fnum(metrics.get("foot_impact_count")),
        "nonfoot_collision": safety.get("nonfoot_collision_failure"),
        "collision": safety.get("collision"), "fall": safety.get("fall"),
        "safety_coverage": safety.get("coverage_complete_for_motion_interval"),
        "outside_physics_steps": fnum((metrics.get("declared_boundary_physics_observation") or {}).get("outside_physics_step_delta")),
        "bounds_status": (metrics.get("declared_boundary_physics_observation") or {}).get("status"),
        "standing_observation": metrics.get("standing_observation_status"),
        "standing_observed_s": lifecycle.get("observed_elapsed_sim_s"),
        "shutdown_complete": attempt.get("process_shutdown_complete"),
        "closeout_complete": attempt.get("closeout_complete"),
        "mujoco_started": attempt.get("mujoco_started"),
        "window_status": (attempt.get("muJoCo_window_log") or {}).get("status"),
        "window_line": (attempt.get("muJoCo_window_log") or {}).get("log_line"),
        "initial_sim_time_s": cycle0.get("sim_time_s"),
        "initial_xy": json.dumps(cycle0.get("world_position_xy_m")),
        "initial_yaw": cycle0.get("world_yaw_rad"),
        "status": item.get("status"), "failure_reasons": "; ".join(item.get("failure_reasons") or []),
        **{f"reentry_{k}": v for k, v in reentry.items()},
    }
    return row, frames, collision


def aggregate_cell(rows: list[dict]) -> dict:
    valid = [r for r in rows if r["record_validity"] == "VALID"]
    terminals = Counter(r["terminal"] for r in valid)
    nonfoot_known = [r["nonfoot_collision"] for r in valid if r["nonfoot_collision"] in (True, False)]
    nonfoot_unknown = sum(1 for r in valid if r["nonfoot_collision"] not in (True, False))
    fall_known = [r["fall"] for r in valid if r["fall"] in (True, False)]
    fall_unknown = sum(1 for r in valid if r["fall"] not in (True, False))
    return {
        "planned": len(rows), "valid": len(valid),
        "safe_completion_true": sum(1 for r in valid if r["safe_completion"] is True),
        "safe_completion_false": sum(1 for r in valid if r["safe_completion"] is False),
        "safe_completion_unknown": sum(1 for r in valid if r["safe_completion"] not in (True, False)),
        "ARRIVED": terminals.get("ARRIVED", 0),
        "COLLISION_TERMINATION": terminals.get("COLLISION_TERMINATION", 0),
        "FALL_TERMINATION": terminals.get("FALL_TERMINATION", 0),
        "SYSTEM_SAFETY_ABORT": terminals.get("SYSTEM_SAFETY_ABORT", 0),
        "DIAGNOSTIC_SIM_TIME_LIMIT": terminals.get("DIAGNOSTIC_SIM_TIME_LIMIT", 0),
        "WALL_CLOCK_GUARD_INCOMPLETE": terminals.get("WALL_CLOCK_GUARD_INCOMPLETE", 0),
        "arrival_rate": (terminals.get("ARRIVED", 0) / len(valid)) if valid else None,
        "collision_count": sum(1 for v in nonfoot_known if v),
        "collision_unknown": nonfoot_unknown,
        "collision_rate_known": (sum(1 for v in nonfoot_known if v) / len(nonfoot_known)) if nonfoot_known else None,
        "collision_rate_best": (sum(1 for v in nonfoot_known if v) / len(valid)) if valid else None,
        "collision_rate_worst": ((sum(1 for v in nonfoot_known if v) + nonfoot_unknown) / len(valid)) if valid else None,
        "fall_count": sum(1 for v in fall_known if v), "fall_unknown": fall_unknown,
        "foot_impact_total": sum(r["foot_impact_count"] for r in valid if r["foot_impact_count"] is not None),
        "recovery_share_mean": mean(r["recovery_share"] for r in valid),
        "recovery_entries_mean": mean(r["recovery_entries"] for r in valid),
        "recovery_exits_mean": mean(r["recovery_exits"] for r in valid),
        "total_switches_mean": mean(r["total_switches"] for r in valid),
        "short_reverse_per_valid_min_mean": mean(r["short_reverse_per_valid_min"] for r in valid),
        "short_reverse_count_mean": mean(r["short_reversals"] for r in valid),
        "arrival_time_mean_s": mean(r["task_duration_s"] for r in valid if r["terminal"] == "ARRIVED"),
        "arrival_distance_mean_m": mean(r["arrival_distance_m"] for r in valid),
        "task_progress_mean_m": mean(r["path_length_m"] for r in valid),
        "reentry_ratio_mean": mean(r["reentry_reentry_ratio"] for r in valid),
        "exit_count_total": sum(r["reentry_exit_count"] or 0 for r in valid),
        "reentry_eligible_total": sum((r["reentry_reentries_within_0_5s"] or 0)
                                      + (r["reentry_nonreentries_observed"] or 0) for r in valid),
        "reentry_censored_total": sum(r["reentry_censored_exits"] or 0 for r in valid),
        "hold_audit_pass": sum(1 for r in valid if r["hold_audit_status"] == "pass"),
        "hold_audit_incomplete": sum(1 for r in valid if r["hold_audit_status"] == "incomplete"),
        "hold_audit_fail": sum(1 for r in valid if r["hold_audit_status"] == "fail"),
        "hold_audit_not_observed": sum(1 for r in valid if r["hold_audit_status"] == "not_observed"),
        "hold_episodes": sum(r["hold_episode_count"] or 0 for r in valid),
        "safety_coverage": sum(1 for r in valid if r["safety_coverage"] is True),
        "standing_complete": sum(1 for r in valid if r["standing_observation"] == "COMPLETE"),
        "cleanup_complete": sum(1 for r in valid if r["shutdown_complete"] is True and r["closeout_complete"] is True),
    }


def build_map_table(rows: list[dict], planned_items: list[dict] | None = None) -> list[dict]:
    grouped = defaultdict(list)
    map_difficulty = {r["map"]: r["difficulty"] for r in rows}
    for item in planned_items or []:
        map_difficulty.setdefault(item["map"], item["difficulty_label"])
    for row in rows:
        grouped[(row["map"], row["difficulty"], row["level"])].append(row)
    out = []
    for map_name in sorted(map_difficulty):
        for level in LEVEL_KEYS:
            cell_rows = grouped[(map_name, map_difficulty[map_name], level)]
            out.append({"map": map_name, "difficulty": map_difficulty[map_name], "level": level,
                        "observed": len(cell_rows), "remaining": max(0, 10 - len(cell_rows)),
                        **{**aggregate_cell(cell_rows), "planned": 10}})
    return out


def build_level_summary(rows: list[dict], layer: str | None = None, planned_maps: list[str] | None = None) -> list[dict]:
    maps = sorted(set(planned_maps or {r["map"] for r in rows if layer is None or r["difficulty"] == layer}))
    if layer is not None:
        maps = [m for m in maps if any(r["map"] == m and r["difficulty"] == layer for r in rows)]
    out = []
    for level in LEVEL_KEYS:
        cells = {m: aggregate_cell([r for r in rows if r["map"] == m and r["level"] == level])
                 for m in maps}
        def avg(key):
            return mean(c.get(key) for c in cells.values())
        out.append({
            "layer": layer or "all", "level": level, "maps": len(maps),
            "planned": sum(c["planned"] for c in cells.values()),
            "valid": sum(c["valid"] for c in cells.values()),
            "ARRIVED": sum(c["ARRIVED"] for c in cells.values()),
            "safe_completion_true": sum(c["safe_completion_true"] for c in cells.values()),
            "safe_completion_false": sum(c["safe_completion_false"] for c in cells.values()),
            "safe_completion_unknown": sum(c["safe_completion_unknown"] for c in cells.values()),
            "COLLISION_TERMINATION": sum(c["COLLISION_TERMINATION"] for c in cells.values()),
            "FALL_TERMINATION": sum(c["FALL_TERMINATION"] for c in cells.values()),
            "SYSTEM_SAFETY_ABORT": sum(c["SYSTEM_SAFETY_ABORT"] for c in cells.values()),
            "DIAGNOSTIC_SIM_TIME_LIMIT": sum(c["DIAGNOSTIC_SIM_TIME_LIMIT"] for c in cells.values()),
            "WALL_CLOCK_GUARD_INCOMPLETE": sum(c["WALL_CLOCK_GUARD_INCOMPLETE"] for c in cells.values()),
            "other_terminal_count": sum(c["valid"] - c["ARRIVED"] - c["COLLISION_TERMINATION"]
                                         - c["FALL_TERMINATION"] - c["DIAGNOSTIC_SIM_TIME_LIMIT"]
                                         - c["WALL_CLOCK_GUARD_INCOMPLETE"] for c in cells.values()),
            "arrival_rate": avg("arrival_rate"),
            "collision_rate_known": avg("collision_rate_known"),
            "collision_rate_best": avg("collision_rate_best"),
            "collision_rate_worst": avg("collision_rate_worst"),
            "fall_count": sum(c["fall_count"] for c in cells.values()),
            "collision_count": sum(c["collision_count"] for c in cells.values()),
            "fall_unknown": sum(c["fall_unknown"] for c in cells.values()),
            "foot_impact_total": sum(c["foot_impact_total"] for c in cells.values()),
            "recovery_share_mean": avg("recovery_share_mean"),
            "recovery_entries_mean": avg("recovery_entries_mean"),
            "recovery_exits_mean": avg("recovery_exits_mean"),
            "total_switches_mean": avg("total_switches_mean"),
            "short_reverse_per_valid_min_mean": avg("short_reverse_per_valid_min_mean"),
            "short_reverse_count_mean": avg("short_reverse_count_mean"),
            "arrival_time_mean_s": avg("arrival_time_mean_s"),
            "arrival_distance_mean_m": avg("arrival_distance_mean_m"),
            "task_progress_mean_m": avg("task_progress_mean_m"),
            "reentry_ratio_mean": avg("reentry_ratio_mean"),
            "exit_count_total": sum(c["exit_count_total"] for c in cells.values()),
            "reentry_eligible_total": sum(c["reentry_eligible_total"] for c in cells.values()),
            "reentry_censored_total": sum(c["reentry_censored_total"] for c in cells.values()),
            "hold_audit_pass": sum(c["hold_audit_pass"] for c in cells.values()),
            "hold_audit_incomplete": sum(c["hold_audit_incomplete"] for c in cells.values()),
            "hold_audit_fail": sum(c["hold_audit_fail"] for c in cells.values()),
            "hold_audit_not_observed": sum(c["hold_audit_not_observed"] for c in cells.values()),
            "hold_episodes": sum(c["hold_episodes"] for c in cells.values()),
            "safety_coverage": sum(c["safety_coverage"] for c in cells.values()),
            "standing_complete": sum(c["standing_complete"] for c in cells.values()),
            "cleanup_complete": sum(c["cleanup_complete"] for c in cells.values()),
        })
    return out


def bootstrap(rows: list[dict], map_rows: list[dict]) -> dict:
    def wilson(x: int, n: int, z: float = 1.959963984540054) -> tuple[float, float]:
        p = x / n
        den = 1 + z * z / n
        mid = (p + z * z / (2 * n)) / den
        half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
        return mid - half, mid + half
    cell = {(row["map"], row["level"]): row for row in map_rows}
    maps = sorted({row["map"] for row in rows})
    out = {}
    for level in LEVEL_KEYS[1:]:
        diffs = []
        xc = nc = xa = na = 0
        for map_name in maps:
            c = cell.get((map_name, level))
            a = cell.get((map_name, REF_KEY))
            if not c or not a or not c["valid"] or not a["valid"]:
                continue
            diffs.append(c["arrival_rate"] - a["arrival_rate"])
            xc += c["ARRIVED"]
            nc += c["valid"]
            xa += a["ARRIVED"]
            na += a["valid"]
        if not diffs or not nc or not na:
            out[level] = {"level": level, "arrival_rate_difference": None,
                          "arrival_rate_difference_ci95_low": None,
                          "arrival_rate_difference_ci95_high": None,
                          "map_equal_mean_difference": None,
                          "selected_map_bootstrap_ci95_low": None,
                          "selected_map_bootstrap_ci95_high": None,
                          "A_arrived": xa, "A_valid": na, "C_arrived": xc, "C_valid": nc}
            continue
        pc, pa = xc / nc, xa / na
        lc, uc = wilson(xc, nc)
        la, ua = wilson(xa, na)
        delta = pc - pa
        low = delta - math.sqrt((pc - lc) ** 2 + (ua - pa) ** 2)
        high = delta + math.sqrt((uc - pc) ** 2 + (pa - la) ** 2)
        rng = random.Random(BOOTSTRAP_SEED + LEVEL_NUM[level])
        boot = [mean(rng.choices(diffs, k=len(diffs))) for _ in range(BOOTSTRAP_N)]
        map_low, map_high = interval(boot)
        out[level] = {"level": level, "arrival_rate_difference": delta,
                      "arrival_rate_difference_ci95_low": low,
                      "arrival_rate_difference_ci95_high": high,
                      "map_equal_mean_difference": mean(diffs),
                      "selected_map_bootstrap_ci95_low": map_low,
                      "selected_map_bootstrap_ci95_high": map_high,
                      "A_arrived": xa, "A_valid": na, "C_arrived": xc, "C_valid": nc}
    return out

    cell = {(row["map"], row["level"]): row for row in map_rows}
    strata = {layer: sorted({row["map"] for row in map_rows if row["difficulty"] == layer})
              for layer, _ in LAYERS}
    metrics = ("arrival_rate", "collision_rate_worst", "collision_rate_best",
               "short_reverse_per_valid_min_mean", "recovery_share_mean")
    map_names = sorted({row["map"] for row in map_rows})
    values = {(level, key): {m: (cell.get((m, level)) or {}).get(key) for m in map_names}
              for level in LEVEL_KEYS for key in metrics}
    rng = random.Random(BOOTSTRAP_SEED)
    samples = {level: {key: [] for key in metrics} for level in LEVEL_KEYS if level != REF_KEY}
    for _ in range(BOOTSTRAP_N):
        sampled = []
        for layer, _ in LAYERS:
            sampled.extend(rng.choices(strata[layer], k=len(strata[layer])))
        for level in LEVEL_KEYS:
            if level == REF_KEY:
                continue
            for key in metrics:
                # Scenario endpoints for a DIFFERENCE use opposite bounds.
                ref_key = ("collision_rate_best" if key == "collision_rate_worst" else
                           "collision_rate_worst" if key == "collision_rate_best" else key)
                pairs = [(values[(level, key)].get(m), values[(REF_KEY, ref_key)].get(m)) for m in sampled]
                pairs = [(a, b) for a, b in pairs if a is not None and b is not None]
                if pairs:
                    samples[level][key].append(mean([a for a, _ in pairs]) - mean([b for _, b in pairs]))
    by_level = {row["level"]: row for row in build_level_summary(rows)}
    ref = by_level[REF_KEY]
    out = {}
    for level in LEVEL_KEYS:
        if level == REF_KEY:
            continue
        item = {"level": level}
        for key in metrics:
            ref_key = ("collision_rate_best" if key == "collision_rate_worst" else
                       "collision_rate_worst" if key == "collision_rate_best" else key)
            point = (by_level[level][key] - ref[ref_key]) if by_level[level].get(key) is not None and ref.get(ref_key) is not None else None
            low, high = interval(samples[level][key])
            item[f"{key}_difference"] = point
            item[f"{key}_ci95_low"] = low
            item[f"{key}_ci95_high"] = high
        out[level] = item
    return out


def map_deltas(rows: list[dict]) -> list[dict]:
    out = []
    for level in LEVEL_KEYS:
        if level == REF_KEY:
            continue
        for map_name in sorted({r["map"] for r in rows}):
            a = aggregate_cell([r for r in rows if r["map"] == map_name and r["level"] == level])
            b = aggregate_cell([r for r in rows if r["map"] == map_name and r["level"] == REF_KEY])
            rel = None
            if a["short_reverse_per_valid_min_mean"] is not None and b["short_reverse_per_valid_min_mean"]:
                rel = (a["short_reverse_per_valid_min_mean"] - b["short_reverse_per_valid_min_mean"]) / b["short_reverse_per_valid_min_mean"]
            out.append({"level": level, "map": map_name,
                        "difficulty": next(r["difficulty"] for r in rows if r["map"] == map_name),
                        "arrival_rate_difference": (a["arrival_rate"] - b["arrival_rate"]) if a["arrival_rate"] is not None and b["arrival_rate"] is not None else None,
                        "collision_rate_difference": (a["collision_rate_known"] - b["collision_rate_known"]) if a["collision_rate_known"] is not None and b["collision_rate_known"] is not None else None,
                        "recovery_share_difference": (a["recovery_share_mean"] - b["recovery_share_mean"]) if a["recovery_share_mean"] is not None and b["recovery_share_mean"] is not None else None,
                        "short_reverse_relative_difference": rel,
                        "arrival_time_difference_s": (a["arrival_time_mean_s"] - b["arrival_time_mean_s"]) if a["arrival_time_mean_s"] is not None and b["arrival_time_mean_s"] is not None else None})
    return out


def paired_arrival_time(rows: list[dict]) -> list[dict]:
    by_key = {(r["map"], r["repeat"], r["level"]): r for r in rows}
    out = []
    for map_name in sorted({r["map"] for r in rows}):
        for level in LEVEL_KEYS:
            if level == REF_KEY:
                continue
            diffs = []
            pairs = 0
            for repeat in range(1, 9):
                a = by_key.get((map_name, repeat, level))
                b = by_key.get((map_name, repeat, REF_KEY))
                if (a and b and a["terminal"] == "ARRIVED" and b["terminal"] == "ARRIVED"
                        and a["task_duration_s"] is not None and b["task_duration_s"] is not None):
                    diffs.append(a["task_duration_s"] - b["task_duration_s"])
                    pairs += 1
            out.append({"map": map_name, "level": level, "paired_arrivals": pairs,
                        "mean_time_difference_s": mean(diffs),
                        "interpretation": "same map and repeat; negative means this level is faster"})
    return out


def recovery_residence(plan: dict, out: Path) -> list[dict]:
    rows = []
    difficulty = {item["map"]: item["difficulty_label"] for item in plan["items"]}
    for layer in (None, "historical_nonarrival", "middle", "difficult"):
        for level in LEVEL_KEYS:
            durations, censored, runs = [], 0, 0
            for item in plan["items"]:
                if level_key(item["group"], item["hold_s"]) != level:
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
            rows.append({"layer": layer or "all", "level": level, "runs_with_recovery": runs,
                         "uncensored_intervals": len(durations), "censored_intervals": censored,
                         "mean_duration_s": mean(durations),
                         "median_duration_s": statistics.median(durations) if durations else None,
                         "p90_duration_s": ordered[int(0.9 * (len(ordered) - 1))] if ordered else None,
                         "max_duration_s": max(durations) if durations else None})
    write_csv(out / "recovery_residence.csv", rows)
    return rows


def hold_summary(episodes: list[dict], rows: list[dict]) -> list[dict]:
    out = []
    for level in LEVEL_KEYS:
        level_episodes = [e for e in episodes if e["level"] == level]
        level_runs = [r for r in rows if r["level"] == level]
        out.append({
            "level": level, "runs": len(level_runs),
            "runs_with_recovery": sum(1 for r in level_runs if (r["recovery_entries"] or 0) > 0),
            "runs_no_recovery": sum(1 for r in level_runs if (r["recovery_entries"] or 0) == 0),
            "episodes": len(level_episodes),
            "exits_observed": sum(1 for e in level_episodes if e["exit_sim_s"] is not None),
            "censored": sum(1 for e in level_episodes if e["censored"]),
            "clock_incomplete": sum(1 for e in level_episodes if e["clock_coverage"] != "complete"),
            "early_exits_before_H": sum(1 for e in level_episodes if e["exit_reason"] == "exited_before_H"),
            "risk_clear_before_H_expiry": sum(1 for e in level_episodes
                if e["first_risk_clear_sim_s"] is not None and e["H_expiry_sim_s"] is not None
                and e["first_risk_clear_sim_s"] < e["H_expiry_sim_s"]),
            "risk_clear_before_H_then_exit": sum(1 for e in level_episodes
                if e["first_risk_clear_sim_s"] is not None and e["H_expiry_sim_s"] is not None
                and e["first_risk_clear_sim_s"] < e["H_expiry_sim_s"] and e["exit_sim_s"] is not None),
            "expired_low_risk_no_prompt_exit": sum(1 for e in level_episodes if e["exit_reason"] in
                                                   ("did_not_exit_after_H_and_RA_below_E", "exited_later_than_first_eligible_cycle")),
            "expired_high_risk_continued": sum(1 for e in level_episodes if e["continued_recovery_after_expiry_high_risk"]),
            "risk_re_rise_episodes": sum(1 for e in level_episodes if e["risk_re_rise_within_episode"]),
            "entry_time_changed_episodes": sum(1 for e in level_episodes if e["entry_time_changed_within_episode"]),
            "reentry_within_0_5s": sum(1 for e in level_episodes if e["reentered_after_exit"] is True),
        })
    return out


def plots(out: Path, summary_rows: list[dict], layer_rows: list[dict],
          map_table: list[dict], episodes: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    chart_dir = out / "charts"
    chart_dir.mkdir(exist_ok=True)

    def style(ax):
        ax.set_facecolor(SURFACE)
        ax.grid(color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.tick_params(colors=MUTED, labelsize=8)

    labels = [row["level"].replace("C_H", "C H=") for row in summary_rows]
    xs = list(range(len(labels)))
    figure, axes = plt.subplots(1, 2, figsize=(10.5, 4.2), facecolor=SURFACE)
    for ax, key, title in ((axes[0], "arrival_rate", "Arrival proportion"),
                           (axes[1], "collision_rate_known", "Non-foot obstacle collision proportion")):
        ax.plot(xs, [row[key] * 100 if row[key] is not None else math.nan for row in summary_rows],
                marker="o", color=SERIES[0], linewidth=2, markersize=5, label="three-map equal mean")
        for layer, color in zip(("historical_nonarrival", "middle", "difficult"), SERIES[1:4]):
            rows = [r for r in layer_rows if r["layer"] == layer]
            if not rows:
                continue
            ax.plot(xs, [r[key] * 100 if r[key] is not None else math.nan for r in rows],
                    marker="o", color=color, linewidth=1.5, markersize=4, label=layer)
        style(ax)
        ax.set_xticks(xs)
        ax.set_xticklabels(labels, rotation=15, ha="right")
        ax.set_ylabel("%", fontsize=8)
        ax.set_title(title, fontsize=10, loc="left")
    axes[0].legend(frameon=False, fontsize=7)
    figure.tight_layout()
    figure.savefig(chart_dir / "arrival_collision_vs_level.png", dpi=160, facecolor=SURFACE)
    plt.close(figure)

    figure, axes = plt.subplots(1, 2, figsize=(10.5, 4.2), facecolor=SURFACE)
    for ax, key, title in ((axes[0], "short_reverse_per_valid_min_mean", "Short reverse switches per valid minute"),
                           (axes[1], "recovery_share_mean", "Recovery share of valid task time")):
        ax.plot(xs, [row[key] if row[key] is not None else math.nan for row in summary_rows],
                marker="o", color=SERIES[0], linewidth=2, markersize=5, label="three-map equal mean")
        for layer, color in zip(("historical_nonarrival", "middle", "difficult"), SERIES[1:4]):
            rows = [r for r in layer_rows if r["layer"] == layer]
            if not rows:
                continue
            ax.plot(xs, [r[key] if r[key] is not None else math.nan for r in rows],
                    marker="o", color=color, linewidth=1.5, markersize=4, label=layer)
        style(ax)
        ax.set_xticks(xs)
        ax.set_xticklabels(labels, rotation=15, ha="right")
        ax.set_title(title, fontsize=10, loc="left")
    figure.tight_layout()
    figure.savefig(chart_dir / "switching_recovery_vs_level.png", dpi=160, facecolor=SURFACE)
    plt.close(figure)

    figure, ax = plt.subplots(figsize=(8.0, 4.2), facecolor=SURFACE)
    ax.bar(xs, [row["DIAGNOSTIC_SIM_TIME_LIMIT"] + row["WALL_CLOCK_GUARD_INCOMPLETE"] for row in summary_rows],
           color=SERIES[0], edgecolor=SURFACE, linewidth=1.2)
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=15, ha="right")
    style(ax)
    ax.set_title("Timeout / wall-clock guard count", fontsize=10, loc="left")
    figure.tight_layout()
    figure.savefig(chart_dir / "timeouts_vs_level.png", dpi=160, facecolor=SURFACE)
    plt.close(figure)

    if episodes:
        figure, ax = plt.subplots(figsize=(7.5, 4.2), facecolor=SURFACE)
        for hold, color in zip((0.02, 0.10, 0.12), SERIES):
            rows = [e for e in episodes if e["H_s"] == hold]
            ax.scatter([hold] * len(rows), [e["observed_residence_s"] for e in rows],
                       color=color, s=18, alpha=0.7, label=f"H={hold:.2f}")
        ax.plot([0.0, 0.13], [0.0, 0.13], color=MUTED, linewidth=1.0,
                linestyle=(0, (3, 3)), label="residence = H")
        style(ax)
        ax.set_xlabel("configured H (sim s)", fontsize=8)
        ax.set_ylabel("observed residence (sim s)", fontsize=8)
        ax.set_title("Recovery residence versus configured hold", fontsize=10, loc="left")
        ax.legend(frameon=False, fontsize=7)
        figure.tight_layout()
        figure.savefig(chart_dir / "hold_residence.png", dpi=160, facecolor=SURFACE)
        plt.close(figure)

    for key, filename, title in (("arrival_rate", "heatmap_arrival.png", "Arrival proportion by map and level"),
                                 ("collision_rate_known", "heatmap_collision.png", "Non-foot collision proportion by map and level")):
        maps = sorted({row["map"] for row in map_table})
        values = [[next((row[key] for row in map_table if row["map"] == m and row["level"] == level), None)
                   for level in LEVEL_KEYS] for m in maps]
        figure, ax = plt.subplots(figsize=(7.0, 7.5), facecolor=SURFACE)
        image = ax.imshow([[math.nan if v is None else v * 100 for v in row] for row in values],
                          aspect="auto", cmap="Blues", vmin=0, vmax=100)
        ax.set_xticks(range(len(LEVEL_KEYS)))
        ax.set_xticklabels([l.replace("C_H", "H=") for l in LEVEL_KEYS], rotation=15, ha="right")
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
    rows, episodes, collision_rows = [], [], []
    for item in plan["items"]:
        if not item.get("attempts"):
            continue
        attempt = item["attempts"][-1]
        row, frames, collision = extract_row(item, attempt)
        rows.append(row)
        if item["group"] == "C":
            item_episodes, _, _ = hold_episodes(item, attempt)
            episodes.extend(item_episodes)
        if collision:
            entry = {k: v for k, v in row.items() if k in ("sequence", "map", "difficulty", "level", "run_id")}
            entry.update({f"collision_{k}": v for k, v in collision.items()})
            collision_rows.append(entry)
    if not rows:
        raise ValueError("no attempted run rows to analyze")
    write_csv(out / "per_run_results.csv", rows)
    write_csv(out / "actual_schedule.csv", [{k: item.get(k) for k in ("sequence", "repeat", "block_index", "position", "map", "group", "hold_s", "plan_item_id", "status")} for item in plan["items"]])
    starts = []
    for item in plan["items"]:
        for attempt in item.get("attempts", []):
            starts.append({"sequence": item["sequence"], "plan_item_id": item["plan_item_id"],
                           "map": item["map"], "level": level_key(item["group"], item["hold_s"]),
                           "attempt_number": attempt.get("attempt_number"),
                           "run_id": attempt.get("run_id"), "status": attempt.get("status"),
                           "terminal_result": attempt.get("terminal_result"),
                           "run_directory": attempt.get("run_directory"),
                           "started_at": attempt.get("started_at"),
                           "finished_at": attempt.get("finished_at"),
                           "mujoco_started": attempt.get("mujoco_started"),
                           "process_shutdown_complete": attempt.get("process_shutdown_complete"),
                           "closeout_complete": attempt.get("closeout_complete")})
    write_csv(out / "actual_scene_starts.csv", starts)
    map_table = build_map_table(rows, plan["items"])
    write_csv(out / "map_setting_12.csv", map_table)
    summary = build_level_summary(rows, planned_maps=plan["mapset"]["maps"])
    write_csv(out / "setting_summary_4.csv", summary)
    layer_rows = []
    for layer, _ in LAYERS:
        if any(r["difficulty"] == layer for r in rows):
            layer_rows.extend(build_level_summary(rows, layer))
    write_csv(out / "source_stratum_summary.csv", layer_rows)
    deltas = bootstrap(rows, map_table)
    delta_rows = []
    for level in LEVEL_KEYS:
        if level == REF_KEY:
            delta_rows.append({"level": level, "reference": True, "note": "A reference of this batch"})
        else:
            delta_rows.append(deltas[level])
    write_csv(out / "setting_deltas_vs_A.csv", delta_rows)
    write_csv(out / "map_deltas_vs_A.csv", map_deltas(rows))
    write_csv(out / "paired_arrival_time_diff.csv", paired_arrival_time(rows))
    write_csv(out / "collision_context.csv", collision_rows)
    write_csv(out / "hold_episodes.csv", episodes)
    hold_rows = hold_summary(episodes, rows)
    write_csv(out / "hold_audit_summary.csv", hold_rows)
    residence_rows = recovery_residence(plan, out)
    plots(out, summary, layer_rows, map_table, episodes)

    lines = ["# R1：A参考下C组保持时间比较（120项）", ""]
    ordinary_results = sum(item.get("status") == "VALID_TASK_RESULT" for item in plan["items"])
    safety_aborts = [row for row in rows if row["terminal"] == "SYSTEM_SAFETY_ABORT"]
    not_started = sum(not item.get("attempts") for item in plan["items"])
    lines.append(f"- 冻结计划项：{len(plan['items'])}；已有运行记录：{len(rows)}；普通有效任务终局：{ordinary_results}；系统安全中止：{len(safety_aborts)}；未启动计划项：{not_started}；确认场景启动：{plan['counters']['confirmed_scene_starts']}；工程补做：{plan['counters'].get('engineering_repairs', 0)}")
    if plan.get("status") == "COMPLETE_WITH_SYSTEM_SAFETY_ABORT":
        lines.append("- 冻结排程已处理完毕，但包含保留的系统安全中止项；该项仍作为单独终局计入记录和设置分母，不视为普通任务结果。")
    elif plan.get("status") != "COMPLETE":
        lines.append(f"- 批次未完成：状态={plan.get('status')}；停止项={plan.get('stop_sequence')}；停止原因={plan.get('stop_reason')}。以下仅汇总已启动项，不构成完整比较或选参结论。")
    for row in safety_aborts:
        lines.append(f"- 第{row['sequence']:03d}项（{row['map']} / {row['level']}）保留为`SYSTEM_SAFETY_ABORT`，记录有效性={row['record_validity']}；安全状态见逐次记录，不作为普通有效任务终局，也未重跑或替代。")
        if False:
            lines.append("- 第029项的`[ABS-CONTRACT]`日志记录策略周期37的读取序列从11944变为11946，`attempts=1`、耗时1,172,959 ns，状态为读取预算耗尽且复制期间序列变化。实现已有最多16次重读和250,000 ns总预算；本次单次复制已超过预算，因此一致性检查按原安全规则拒绝该帧并请求PASSIVE。未发现应修复的读取逻辑错误；没有放宽安全检查或过期阈值，也未改控制器代码。")
    lines.append("- A/C单阈值 E=−0.05，有效X=E，滞回关闭；A H=0且保持关闭；C H=0.02/0.10/0.12仿真秒，滞回关闭。目标(7.5,0)m、到达半径0.5m、仿真/墙钟上限20/120秒，到达后站立观察2秒。")
    lines.append("- 三张图是有意选择的困难开发图，不是独立留出集；每格10次，初态实测保存但没有严格物理状态配对。")
    lines.append("")
    lines.append("## 四设置汇总")
    lines.append("")
    lines.append("| 设置 | 记录有效/计划 | 安全完成T/F/U | 到达 | 碰撞终止 | 安全窗碰撞 | 跌倒终止 | 安全窗跌倒 | 安全中止 | 仿真限时 | 墙钟保护 | 其他终局 | 到达率 | 平均终点距离(m) | 平均路径长度(m) | 到达耗时均值(n) | Recovery占比 |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for row in summary:
        def fmt(v, d=3):
            return f"{v:.{d}f}" if v is not None else "UNKNOWN"
        lines.append("| " + " | ".join(str(v) for v in (
            row["level"].replace("C_H", "C H="), f"{row['valid']}/{row['planned']}",
            f"{row['safe_completion_true']}/{row['safe_completion_false']}/{row['safe_completion_unknown']}",
            row["ARRIVED"],
            row["COLLISION_TERMINATION"], row["collision_count"], row["FALL_TERMINATION"], row["fall_count"],
            row["SYSTEM_SAFETY_ABORT"], row["DIAGNOSTIC_SIM_TIME_LIMIT"], row["WALL_CLOCK_GUARD_INCOMPLETE"], row["other_terminal_count"],
            fmt(row["arrival_rate"]), fmt(row.get("arrival_distance_mean_m")),
            fmt(row.get("task_progress_mean_m")),
            f"{fmt(row['arrival_time_mean_s'])} (n={sum(r['terminal']=='ARRIVED' and r['level']==row['level'] for r in rows)})",
            fmt(row["recovery_share_mean"]))) + " |")
    lines.append("")
    lines.append("## 同期到达率差：C减A")
    lines.append("")
    lines.append("Newcombe–Wilson 95%区间基于当前可比地图中的记录有效A/C样本；三图等权差及地图重抽样95%区间只用同时观察到A和该C档的地图。地图数仅3，重抽样区间很粗略；不因区间重叠认定等效，也不因点估计较高认定优越。")
    lines.append("")
    lines.append("| 设置 | C到达/有效 | A到达/有效 | 率差(百分点) | Newcombe–Wilson 95%区间 | 三图等权差 | 三图重抽样95%区间 |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|")
    for level in LEVEL_KEYS[1:]:
        d = deltas[level]
        def pp(v): return "UNKNOWN" if v is None else f"{100*v:.1f}"
        ci = "UNKNOWN" if d["arrival_rate_difference_ci95_low"] is None else f"{pp(d['arrival_rate_difference_ci95_low'])} 至 {pp(d['arrival_rate_difference_ci95_high'])}"
        map_ci = "UNKNOWN" if d["selected_map_bootstrap_ci95_low"] is None else f"{pp(d['selected_map_bootstrap_ci95_low'])} 至 {pp(d['selected_map_bootstrap_ci95_high'])}"
        lines.append(f"| {level.replace('C_H','C H=')} | {d['C_arrived']}/{d['C_valid']} | {d['A_arrived']}/{d['A_valid']} | {pp(d['arrival_rate_difference'])} | {ci} | {pp(d['map_equal_mean_difference'])} | {map_ci} |")
    lines.append("")
    lines.append("## 每张地图结果")
    lines.append("")
    lines.append("三图×四设置表固定为12格；`observed`和`remaining`标出已完成与未运行次数。逐次终局、到达耗时、推进和保持审计见`per_run_results.csv`与`hold_audit_summary.csv`。单次到达耗时只对成功样本计算，不同成功子集不能据此推断总体效率。")
    lines.append("")
    h002_episodes = [e for e in episodes if e["H_s"] == 0.02]
    h002_clear_before_expiry = [e for e in h002_episodes
        if e["first_risk_clear_sim_s"] is not None and e["H_expiry_sim_s"] is not None
        and e["first_risk_clear_sim_s"] < e["H_expiry_sim_s"]]
    h002_residence = next((r for r in residence_rows if r["level"] == "C_H0.02" and r["layer"] == "all"), {})
    lines.append(f"H=0.02保持审计：{len(h002_episodes)}个Recovery episode中，{len(h002_clear_before_expiry)}个在H到期前RA已低于E且随后退出；到期前退出为{sum(e['exit_reason']=='exited_before_H' for e in h002_episodes)}个。已观测恢复驻留中位数{h002_residence.get('median_duration_s'):.3f}秒、均值{h002_residence.get('mean_duration_s'):.3f}秒（配置H=0.020秒）；这些记录表明短保持实际挡住了一部分提前退出，主要约为一个控制周期量级。")
    lines.append("保持审计仅在Recovery episode出现时能检验实际保持语义；A的未观察不代表审计失败。实际驻留通常长于H，因为H到期后仍须满足RA退出条件。")
    lines.append("")
    lines.append("合法失败和UNKNOWN按原记录保留；记录有效性与安全完成状态分开记录；记录VALID不等于安全完成。切换与Recovery占比仅用于解释，不单独判定C优于A。结果只描述这三张困难开发图，不估计一般场景成功率，也不自动挑选H或启动D。")
    lines.append("")
    lines.append("- 到达/碰撞和切换/Recovery图见`charts/arrival_collision_vs_level.png`、`charts/switching_recovery_vs_level.png`。")
    (out / "RESULTS_SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"analysis written to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
