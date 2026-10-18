#!/usr/bin/env python3
"""Analysis for the T2 stage-3 18-map x 5-level A/C hold-time scan (180 runs)."""
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
LEVELS = (("A", None), ("C", 0.10), ("C", 0.20), ("C", 0.30), ("C", 0.50))
LEVEL_KEYS = ["A", "C_H0.10", "C_H0.20", "C_H0.30", "C_H0.50"]
LEVEL_NUM = {key: index for index, key in enumerate(LEVEL_KEYS)}
REF_KEY = "A"
LAYERS = (("easy", 3), ("middle", 6), ("difficult", 9))
BOOTSTRAP_SEED = 2026101804
BOOTSTRAP_N = 10000
F_REF = 257.8
TIME_CAP = 20.0
TOLERANCE_PP = 5.0
SWITCH_IMPROVEMENT = 0.20
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


def score_run(row: dict) -> dict:
    if row["status"] != "VALID_TASK_RESULT" or row["record_validity"] != "VALID":
        return {"score_status": "UNKNOWN", "total_score_100": None}
    collision = row.get("nonfoot_collision")
    fall = row.get("fall")
    frequency = row.get("short_reverse_per_valid_min")
    elapsed = row.get("task_duration_s")
    if collision not in (True, False) or fall not in (True, False) or elapsed is None:
        return {"score_status": "UNKNOWN", "total_score_100": None}
    if row.get("short_reverse_switch_coverage") != "complete" or frequency is None:
        return {"score_status": "UNKNOWN", "total_score_100": None}
    arrived = row["terminal"] == "ARRIVED"
    total = 100.0 * (0.4 * (1.0 if arrived else 0.0)
                     + 0.3 * (1.0 if (collision is False and fall is False) else 0.0)
                     + 0.2 * (max(0.0, 1.0 - elapsed / TIME_CAP) if arrived else 0.0)
                     + 0.1 / (1.0 + frequency / F_REF))
    return {"score_status": "COMPLETE", "total_score_100": total}


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
    row.update(score_run(row))
    return row, frames, collision


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
        "score_mean_100": mean(r["total_score_100"] for r in valid),
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


def build_map_table(rows: list[dict]) -> list[dict]:
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row["map"], row["difficulty"], row["level"])].append(row)
    out = []
    for (map_name, difficulty, level), cell_rows in sorted(
            grouped.items(), key=lambda kv: (LEVEL_NUM[kv[0][2]], kv[0][0])):
        out.append({"map": map_name, "difficulty": difficulty, "level": level,
                    **aggregate_cell(cell_rows)})
    return out


def build_level_summary(rows: list[dict], layer: str | None = None) -> list[dict]:
    maps = sorted({r["map"] for r in rows if layer is None or r["difficulty"] == layer})
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
            "COLLISION_TERMINATION": sum(c["COLLISION_TERMINATION"] for c in cells.values()),
            "FALL_TERMINATION": sum(c["FALL_TERMINATION"] for c in cells.values()),
            "DIAGNOSTIC_SIM_TIME_LIMIT": sum(c["DIAGNOSTIC_SIM_TIME_LIMIT"] for c in cells.values()),
            "WALL_CLOCK_GUARD_INCOMPLETE": sum(c["WALL_CLOCK_GUARD_INCOMPLETE"] for c in cells.values()),
            "arrival_rate": avg("arrival_rate"),
            "collision_rate_known": avg("collision_rate_known"),
            "collision_rate_best": avg("collision_rate_best"),
            "collision_rate_worst": avg("collision_rate_worst"),
            "fall_count": sum(c["fall_count"] for c in cells.values()),
            "fall_unknown": sum(c["fall_unknown"] for c in cells.values()),
            "foot_impact_total": sum(c["foot_impact_total"] for c in cells.values()),
            "recovery_share_mean": avg("recovery_share_mean"),
            "recovery_entries_mean": avg("recovery_entries_mean"),
            "recovery_exits_mean": avg("recovery_exits_mean"),
            "total_switches_mean": avg("total_switches_mean"),
            "short_reverse_per_valid_min_mean": avg("short_reverse_per_valid_min_mean"),
            "short_reverse_count_mean": avg("short_reverse_count_mean"),
            "arrival_time_mean_s": avg("arrival_time_mean_s"),
            "score_mean_100": avg("score_mean_100"),
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
    cell = {(row["map"], row["level"]): row for row in map_rows}
    strata = {layer: sorted({row["map"] for row in map_rows if row["difficulty"] == layer})
              for layer, _ in LAYERS}
    metrics = ("arrival_rate", "collision_rate_worst", "collision_rate_best",
               "short_reverse_per_valid_min_mean", "recovery_share_mean", "score_mean_100")
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
                pairs = [(values[(level, key)].get(m), values[(REF_KEY, key)].get(m)) for m in sampled]
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
            point = (by_level[level][key] - ref[key]) if by_level[level].get(key) is not None and ref.get(key) is not None else None
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
            for repeat in (1, 2):
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
    for layer in (None, "easy", "middle", "difficult"):
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
            "expired_low_risk_no_prompt_exit": sum(1 for e in level_episodes if e["exit_reason"] in
                                                   ("did_not_exit_after_H_and_RA_below_E", "exited_later_than_first_eligible_cycle")),
            "expired_high_risk_continued": sum(1 for e in level_episodes if e["continued_recovery_after_expiry_high_risk"]),
            "risk_re_rise_episodes": sum(1 for e in level_episodes if e["risk_re_rise_within_episode"]),
            "entry_time_changed_episodes": sum(1 for e in level_episodes if e["entry_time_changed_within_episode"]),
            "reentry_within_0_5s": sum(1 for e in level_episodes if e["reentered_after_exit"] is True),
        })
    return out


def candidate_levels(summary_rows: list[dict], deltas: dict) -> dict:
    by_level = {row["level"]: row for row in summary_rows}
    ref = by_level[REF_KEY]
    assessments = []
    for level in LEVEL_KEYS:
        if level == REF_KEY:
            continue
        row = by_level[level]
        arrival_pp = (row["arrival_rate"] - ref["arrival_rate"]) * 100.0 if row["arrival_rate"] is not None and ref["arrival_rate"] is not None else None
        known_pp = ((row["collision_rate_known"] - ref["collision_rate_known"]) * 100.0
                    if row["collision_rate_known"] is not None and ref["collision_rate_known"] is not None else None)
        worst_pp = ((row["collision_rate_worst"] - ref["collision_rate_worst"]) * 100.0
                    if row["collision_rate_worst"] is not None and ref["collision_rate_worst"] is not None else None)
        ref_freq = ref["short_reverse_per_valid_min_mean"]
        freq = row["short_reverse_per_valid_min_mean"]
        relative = ((freq - ref_freq) / ref_freq) if ref_freq and freq is not None else None
        task_best = arrival_pp is not None and arrival_pp >= -TOLERANCE_PP and known_pp is not None and known_pp <= TOLERANCE_PP
        task_worst = arrival_pp is not None and arrival_pp >= -TOLERANCE_PP and worst_pp is not None and worst_pp <= TOLERANCE_PP
        task_ok = task_best and task_worst
        switch_ok = relative is not None and relative <= -SWITCH_IMPROVEMENT
        fall_free = row["fall_count"] == 0 and row["fall_unknown"] == 0
        evidence_insufficient = (task_best != task_worst) or row["valid"] != 36
        eligible = bool(task_ok and switch_ok and fall_free and not evidence_insufficient)
        assessments.append({
            "level": level, "H_s": float(level.split("H")[1]) if "H" in level else None,
            "arrival_rate": row["arrival_rate"], "arrival_pp_vs_A": arrival_pp,
            "collision_rate_known": row["collision_rate_known"],
            "collision_known_pp_vs_A": known_pp,
            "collision_rate_worst": row["collision_rate_worst"],
            "collision_worst_pp_vs_A": worst_pp,
            "short_reverse_per_valid_min": freq, "short_reverse_relative_vs_A": relative,
            "recovery_share": row["recovery_share_mean"],
            "fall_count": row["fall_count"], "fall_unknown": row["fall_unknown"],
            "task_ok": task_ok, "switch_ok": switch_ok, "fall_free": fall_free,
            "evidence_insufficient": evidence_insufficient, "eligible": eligible,
            "difference_ci95": deltas.get(level, {})})
    eligible = [a for a in assessments if a["eligible"]]
    eligible.sort(key=lambda a: a["H_s"] if a["H_s"] is not None else 9)
    if eligible:
        recommended = eligible[0]
        return {"supported": True, "recommended_level": recommended["level"],
                "recommended_H_s": recommended["H_s"],
                "message": "存在满足任务/安全/切换规则的已测H；按最小有效H优先提出代表值。",
                "eligible_levels": [a["level"] for a in eligible],
                "assessments": assessments}
    fallback = [a for a in assessments if a["task_ok"] and a["fall_free"]]
    fallback.sort(key=lambda a: (-(a["arrival_rate"] or 0), a["collision_rate_known"] or 9,
                                 a["short_reverse_per_valid_min"] or 9))
    return {"supported": False, "recommended_level": None, "recommended_H_s": None,
            "message": "没有C档同时满足任务容忍度、无跌倒和短反向下降20%；不自动扩展网格。",
            "fallback_engineering": fallback[0] if fallback else None,
            "assessments": assessments}


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
                marker="o", color=SERIES[0], linewidth=2, markersize=5, label="all 18 maps")
        for layer, color in zip(("easy", "middle", "difficult"), SERIES[1:4]):
            rows = [r for r in layer_rows if r["layer"] == layer]
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
                marker="o", color=SERIES[0], linewidth=2, markersize=5, label="all 18 maps")
        for layer, color in zip(("easy", "middle", "difficult"), SERIES[1:4]):
            rows = [r for r in layer_rows if r["layer"] == layer]
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
        for hold, color in zip((0.10, 0.20, 0.30, 0.50), SERIES[:4]):
            rows = [e for e in episodes if e["H_s"] == hold]
            ax.scatter([hold] * len(rows), [e["observed_residence_s"] for e in rows],
                       color=color, s=18, alpha=0.7, label=f"H={hold:.2f}")
        ax.plot([0.08, 0.55], [0.08, 0.55], color=MUTED, linewidth=1.0,
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
    if len(rows) != 180:
        raise ValueError(f"expected 180 run rows, found {len(rows)}")
    write_csv(out / "per_run_results.csv", rows)
    map_table = build_map_table(rows)
    write_csv(out / "map_level_90.csv", map_table)
    summary = build_level_summary(rows)
    write_csv(out / "level_summary_5.csv", summary)
    layer_rows = []
    for layer, _ in LAYERS:
        layer_rows.extend(build_level_summary(rows, layer))
    write_csv(out / "layer_summary_15.csv", layer_rows)
    deltas = bootstrap(rows, map_table)
    delta_rows = []
    for level in LEVEL_KEYS:
        if level == REF_KEY:
            delta_rows.append({"level": level, "reference": True, "note": "A reference of this batch"})
        else:
            delta_rows.append(deltas[level])
    write_csv(out / "level_deltas_vs_A.csv", delta_rows)
    write_csv(out / "map_deltas_vs_A.csv", map_deltas(rows))
    write_csv(out / "paired_arrival_time_diff.csv", paired_arrival_time(rows))
    write_csv(out / "collision_context.csv", collision_rows)
    write_csv(out / "hold_episodes.csv", episodes)
    hold_rows = hold_summary(episodes, rows)
    write_csv(out / "hold_audit_summary.csv", hold_rows)
    recovery_residence(plan, out)
    candidate = candidate_levels(summary, deltas)
    (out / "candidate_H.json").write_text(json.dumps(candidate, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    plots(out, summary, layer_rows, map_table, episodes)

    lines = ["# T2 180项A/C保持时间扫描：自动汇总", ""]
    lines.append(f"- 计划/有效：{len(plan['items'])}/{len(rows)}")
    lines.append("- 固定：A/C单阈值 E=-0.05；A无保持；C H=0.10/0.20/0.30/0.50仿真秒；目标(7.5,0)、20仿真秒/120墙钟秒、2秒站立")
    lines.append("")
    lines.append("| 档位 | 有效 | 到达 | 非足端碰撞 | 跌倒 | 时限/墙钟 | 到达率 | 碰撞率(已知) | 短反向/分钟 | Recovery占比 | 辅助分 |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for row in summary:
        def fmt(v, d=3):
            return f"{v:.{d}f}" if v is not None else "UNKNOWN"
        lines.append("| " + " | ".join(str(v) for v in (
            row["level"].replace("C_H", "C H="), row["valid"], row["ARRIVED"],
            row["COLLISION_TERMINATION"], row["FALL_TERMINATION"],
            f"{row['DIAGNOSTIC_SIM_TIME_LIMIT']}/{row['WALL_CLOCK_GUARD_INCOMPLETE']}",
            fmt(row["arrival_rate"]), fmt(row["collision_rate_known"]),
            fmt(row["short_reverse_per_valid_min_mean"], 1), fmt(row["recovery_share_mean"]),
            fmt(row["score_mean_100"], 1))) + " |")
    lines.append("")
    lines.append("## 候选H")
    lines.append(candidate["message"])
    if candidate["supported"]:
        lines.append(f"- 推荐代表值：{candidate['recommended_level']}（H={candidate['recommended_H_s']:.2f}s）；合格档：{candidate['eligible_levels']}")
    elif candidate.get("fallback_engineering"):
        lines.append(f"- 工程备选：{candidate['fallback_engineering']['level']}（未通过完整候选规则）")
    lines.append("")
    lines.append("单次/两次重复是开发数据；不估计可靠成功率、不证明全局最优或一般环境分布。")
    (out / "RESULTS_SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"analysis written to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
