#!/usr/bin/env python3
"""Analyze the frozen T2 6-map x A/B/C/D x 2-repeat development check."""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

GROUPS = ("A", "B", "C", "D")
MAPS = ("e03.xml", "m02.xml", "m06.xml", "d02.xml", "d05.xml", "d07.xml")
GROUP_MODE = {"A": 1, "B": 2, "C": 3, "D": 4}
CANDIDATE = {"A": "paper_faithful_switch", "B": "hysteresis_switch",
             "C": "minimum_hold_switch", "D": "combined_switch"}
COLORS = {"A": "#3569a5", "B": "#e48b34", "C": "#42966c", "D": "#a85572"}


def load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def finite(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def mean(values):
    xs = [finite(x) for x in values]
    xs = [x for x in xs if x is not None]
    return statistics.mean(xs) if xs else None


def median(values):
    xs = [finite(x) for x in values]
    xs = [x for x in xs if x is not None]
    return statistics.median(xs) if xs else None


def encode(value):
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return value


def write_csv(path: Path, rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows([{k: encode(v) for k, v in row.items()} for row in rows])


def read_frames(run_dir: Path) -> list[dict]:
    rows = []
    path = run_dir / "runtime_record.jsonl"
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if record.get("kind") == "frame" and isinstance(record.get("payload"), dict):
                payload = record["payload"]
                payload["_run_id"] = record.get("run_id")
                rows.append(payload)
    return rows


def frames_in_task(frames: list[dict], metrics: dict, context: dict) -> tuple[list[dict], list[str]]:
    boundary = metrics.get("rl_task_metric_boundary") or {}
    start, end = boundary.get("start_cycle"), boundary.get("end_cycle")
    session = context.get("session_id")
    issues = []
    selected = [f for f in frames if f.get("rl_entered") == 1 and f.get("rl_active") == 1 and
                isinstance(f.get("rl_step"), int) and isinstance(start, int) and isinstance(end, int) and
                start <= f["rl_step"] <= end and f.get("session_id") == session]
    selected.sort(key=lambda f: f["rl_step"])
    if not selected:
        return [], ["no policy frames in declared task boundary"]
    cycles = [f["rl_step"] for f in selected]
    if cycles[0] != start:
        issues.append("first task cycle missing")
    if cycles[-1] != end:
        issues.append("terminal task cycle missing")
    if len(cycles) != len(set(cycles)):
        issues.append("duplicate cycle IDs")
    if any(b != a + 1 for a, b in zip(cycles, cycles[1:])):
        issues.append("policy cycle gaps in task window")
    if any(f.get("session_id") != session for f in selected):
        issues.append("session mismatch")
    return selected, issues


def expected_parameters(group: str):
    return (-0.05, -0.10 if group in ("B", "D") else -0.05,
            0.20 if group in ("C", "D") else 0.0,
            group in ("B", "D"), group in ("C", "D"))


def verify_actual_consumption(item: dict, metrics: dict, context: dict,
                              task_frames: list[dict], plan: dict) -> list[str]:
    problems = []
    group = item["group"]
    E, X, H, hysteresis, hold = expected_parameters(group)
    settings = context.get("settings", {})
    expected = {"group": group, "candidate": CANDIDATE[group], "entry_threshold": E,
                "exit_threshold": X, "recovery_min_hold_s": H,
                "hysteresis": "true" if hysteresis else "false",
                "hold": "true" if hold else "false", "goal_world_m": [7.5, 0.0]}
    for key, value in expected.items():
        actual = settings.get(key)
        if isinstance(value, float):
            if finite(actual) is None or abs(float(actual) - value) > 1e-6:
                problems.append(f"settings.{key}={actual!r}, expected {value}")
        elif actual != value:
            problems.append(f"settings.{key}={actual!r}, expected {value!r}")
    overlay = Path(plan["overlays"][group]["config"])
    config = overlay.read_text(encoding="utf-8")
    switching = config.split("  switching:", 1)[1].split("\n  # Recovery", 1)[0]
    if ("exit_threshold:" in switching) != hysteresis:
        problems.append("overlay exit_threshold field presence mismatch")
    if ("recovery_min_hold_s:" in switching) != hold:
        problems.append("overlay recovery_min_hold_s field presence mismatch")
    mode = GROUP_MODE[group]
    for frame in task_frames:
        if frame.get("switching_mode") != mode:
            problems.append(f"cycle {frame.get('rl_step')} switching_mode mismatch")
            break
        if frame.get("hold_enabled") is not hold:
            problems.append(f"cycle {frame.get('rl_step')} hold_enabled mismatch")
            break
        for key, value in (("entry_threshold", E), ("exit_threshold", X),
                           ("recovery_min_hold_s", H)):
            actual = finite(frame.get(key))
            if actual is None or abs(actual - value) > 1e-5:
                problems.append(f"cycle {frame.get('rl_step')} {key} mismatch")
                break
        else:
            continue
        break
    if metrics.get("record_validity") != "VALID":
        problems.append(f"record_validity={metrics.get('record_validity')}")
    readiness = metrics.get("s6_preparation_readiness") or {}
    if readiness.get("first_active_policy_frame_exactly_persisted") is not True:
        problems.append("cycle0/companion not confirmed")
    initial = context.get("initial_state") or {}
    cycle0 = initial.get("first_active_policy_cycle") or {}
    if (cycle0.get("status") != "CAPTURED" or cycle0.get("policy_cycle_id") != 0 or
            cycle0.get("first_persisted_active_cycle_id") != 0 or
            cycle0.get("same_cycle_runtime_frame_persisted_by_supervisor") is not True or
            cycle0.get("session_id") != context.get("session_id")):
        problems.append("initial-state companion is not bound to persisted cycle0/session")
    if metrics.get("process_shutdown_complete") is not True or metrics.get("closeout_complete") is not True:
        problems.append("shutdown/closeout incomplete")
    if metrics.get("policy_cycle_history_complete_at_final_closeout_drain") is not True:
        problems.append("final policy-cycle drain incomplete")
    audit = metrics.get("recovery_hold_audit") or {}
    if group in ("C", "D") and audit.get("status") == "fail":
        problems.append("production hold audit reports FAIL")
    if audit.get("violations"):
        problems.append(f"production hold audit has {len(audit['violations'])} violation(s)")
    return problems


def hold_episode_details(item: dict, metrics: dict, frames: list[dict]) -> list[dict]:
    group = item["group"]
    if group not in ("C", "D"):
        return []
    _, threshold, H, _, _ = expected_parameters(group)
    by_cycle = {int(f["rl_step"]): f for f in frames if isinstance(f.get("rl_step"), int)}
    audit = metrics.get("recovery_hold_audit") or {}
    output = []
    for index, ep in enumerate(audit.get("episodes") or [], 1):
        enter = ep.get("enter_cycle")
        exit_cycle = ep.get("exit_cycle")
        t_enter = finite(ep.get("t_enter_sim_s"))
        entry = by_cycle.get(int(enter)) if isinstance(enter, (int, float)) else None
        seg = entry.get("sim_clock_segment_id") if entry else None
        end_cycle = int(exit_cycle) if isinstance(exit_cycle, (int, float)) else (
            max(by_cycle) if by_cycle else int(enter or -1))
        rows = [by_cycle[c] for c in sorted(by_cycle) if isinstance(enter, (int, float)) and
                int(enter) <= c <= end_cycle]
        continuity = bool(rows and rows[0].get("rl_step") == enter)
        continuity = continuity and all(b.get("rl_step") == a.get("rl_step") + 1
                                       for a, b in zip(rows, rows[1:]))
        clock_complete = continuity and all(f.get("sim_clock_valid") is True and
            f.get("sim_clock_segment_id") == seg and finite(f.get("sim_time_s")) is not None
            for f in rows)
        entries = {finite(f.get("hold_enter_sim_time_s")) for f in rows if finite(f.get("hold_enter_sim_time_s")) is not None}
        entry_time_stable = len(entries) == 1 and t_enter is not None and abs(next(iter(entries)) - t_enter) < 1e-6
        expiry = t_enter + H if t_enter is not None else None
        expiry_rows = [f for f in rows if expiry is not None and finite(f.get("sim_time_s")) is not None
                       and float(f["sim_time_s"]) >= expiry and f.get("hold_status") == 3]
        at_expiry = expiry_rows[0] if expiry_rows else None
        risk_high_at_expiry = bool(at_expiry and finite(at_expiry.get("ra_value")) is not None and
                                   float(at_expiry["ra_value"]) >= threshold)
        first_eligible = next((f for f in expiry_rows if finite(f.get("ra_value")) is not None and
                               float(f["ra_value"]) < threshold and f.get("mode_after") == 0), None)
        exit_frame = by_cycle.get(int(exit_cycle)) if isinstance(exit_cycle, (int, float)) else None
        exited_on_first_eligible = bool(exit_frame and first_eligible and
                                        exit_frame.get("rl_step") == first_eligible.get("rl_step"))
        risk_clear_seen = False
        risk_rerise = False
        for frame in rows:
            ra = finite(frame.get("ra_value"))
            if ra is None:
                continue
            if ra < threshold:
                risk_clear_seen = True
            elif risk_clear_seen:
                risk_rerise = True
        output.append({
            "sequence": item["sequence"], "plan_item_id": item["plan_item_id"],
            "map": item["map"], "group": group, "repeat": item["repeat"],
            "run_id": metrics.get("run_id"), "episode_index": index,
            "entry_cycle": enter, "entry_sim_time_s": t_enter,
            "H_s": ep.get("H_s"), "expiry_sim_time_s": expiry,
            "exit_cycle": exit_cycle, "observed_residence_s": ep.get("observed_exit_elapsed_s"),
            "censored": ep.get("censored"), "clock_coverage": ep.get("clock_coverage"),
            "production_hold_rule": ep.get("hold_rule"),
            "task_cycle_contiguous": continuity, "same_clock_segment_complete": clock_complete,
            "entry_time_stable_through_episode": entry_time_stable,
            "first_observed_expired_cycle": (at_expiry.get("rl_step") if at_expiry else None),
            "RA_at_first_expired_cycle": finite(at_expiry.get("ra_value")) if at_expiry else None,
            "risk_still_high_at_expiry": risk_high_at_expiry,
            "risk_re_rise_inside_episode": risk_rerise,
            "exit_on_first_expired_eligible_cycle": exited_on_first_eligible if first_eligible else None,
            "did_not_exit_after_expiry_and_RA_below_threshold": bool(
                first_eligible and exit_frame and exit_frame.get("rl_step") > first_eligible.get("rl_step")),
            "threshold_used": threshold,
        })
    return output


def reentry_summary(frames: list[dict]) -> dict:
    active = [f for f in frames if f.get("rl_entered") == 1 and f.get("rl_active") == 1]
    exits = [b for a, b in zip(active, active[1:]) if a.get("policy_state") == 1 and b.get("policy_state") == 0]
    reentry = observed_none = censored = unknown = 0
    for exit_frame in exits:
        t0 = finite(exit_frame.get("sim_time_s"))
        step0 = exit_frame.get("rl_step")
        seg0 = exit_frame.get("sim_clock_segment_id")
        if t0 is None:
            unknown += 1; continue
        later = [f for f in active if isinstance(step0, int) and f.get("rl_step", -1) > step0]
        next_half = [f for f in later if finite(f.get("sim_time_s")) is not None and
                     float(f["sim_time_s"]) <= t0 + 0.5]
        if any(f.get("policy_state") == 1 for f in next_half):
            reentry += 1; continue
        end = later[-1] if later else None
        if (end is None or finite(end.get("sim_time_s")) is None or
                float(end["sim_time_s"]) < t0 + 0.5):
            censored += 1; continue
        interval = [exit_frame] + next_half
        if (any(f.get("sim_clock_valid") is not True or f.get("sim_clock_segment_id") != seg0 for f in interval) or
                any(b.get("rl_step") != a.get("rl_step", -2) + 1 for a, b in zip(interval, interval[1:]))):
            unknown += 1
        else:
            observed_none += 1
    den = reentry + observed_none
    return {"exit_count": len(exits), "reentries_within_0_5s": reentry,
            "observed_no_reentry": observed_none, "censored": censored, "unknown": unknown,
            "observable_denominator": den, "reentry_rate_observed": reentry / den if den else None}


def extract_run(plan_item: dict, plan: dict) -> tuple[dict, list[dict], list[dict]]:
    actual_attempts = [a for a in plan_item.get("attempts", []) if a.get("run_directory") and a.get("run_id")]
    if len(actual_attempts) != 1:
        raise RuntimeError(f"{plan_item['plan_item_id']}: expected one actual run artifact, got {len(actual_attempts)}")
    attempt = actual_attempts[0]
    run_dir = Path(attempt["run_directory"])
    metrics = load(run_dir / "metrics.json")
    context = load(run_dir / "run_context.json")
    frames_all = read_frames(run_dir)
    frames, identity_issues = frames_in_task(frames_all, metrics, context)
    consumption_issues = verify_actual_consumption(plan_item, metrics, context, frames, plan)
    payloads = [f for f in frames if f.get("kind") is None]
    ras = [finite(f.get("ra_value")) for f in payloads]
    ras = [x for x in ras if x is not None]
    state_counts = Counter(f.get("policy_state_name") or f.get("policy_state") for f in payloads)
    action_counts = Counter({"Agile": sum(f.get("action_source") == 1 for f in payloads),
                             "Recovery": sum(f.get("action_source") == 2 for f in payloads),
                             "None": sum(f.get("action_source") == 3 for f in payloads)})
    safety = metrics.get("task_window_safety") or {}
    collision_coverage = metrics.get("collision_coverage") or {}
    risk = metrics.get("risk_to_recovery_response") or {}
    hold = metrics.get("recovery_hold_audit") or {}
    lifecycle = context.get("lifecycle") or {}
    readiness = metrics.get("s6_preparation_readiness") or {}
    boundary = metrics.get("rl_task_metric_boundary") or {}
    transition_count = metrics.get("mode_transitions")
    recovery_intervals = metrics.get("recovery_intervals") or []
    residence = [finite(x.get("duration_s")) for x in recovery_intervals if x.get("mode") == "RECOVERY"]
    residence = [x for x in residence if x is not None and x >= 0]
    reentry = reentry_summary(frames)
    terminal_capture = lifecycle.get("terminal_frame_capture") or {}
    terminal_ref = lifecycle.get("terminal_frame_ref")
    row = {
        "sequence": plan_item["sequence"], "plan_item_id": plan_item["plan_item_id"],
        "round": plan_item["round"], "map": plan_item["map"], "scene_id": plan_item["scene_id"],
        "group": plan_item["group"], "repeat": plan_item["repeat"], "run_id": metrics.get("run_id"),
        "run_directory": str(run_dir), "record_validity": metrics.get("record_validity"),
        "reset_state_status": (context.get("initial_state") or {}).get("reset_state", {}).get("status"),
        "reset_state_source": (context.get("initial_state") or {}).get("reset_state", {}).get("source"),
        "reset_base_pose_world_m": (context.get("initial_state") or {}).get("reset_state", {}).get("base_pose_world_m"),
        "reset_qpos0": (context.get("initial_state") or {}).get("reset_state", {}).get("qpos0"),
        "reset_joint_velocity_status": (context.get("initial_state") or {}).get("reset_state", {}).get("reset_joint_velocity"),
        "cycle0_state_status": (context.get("initial_state") or {}).get("first_active_policy_cycle", {}).get("status"),
        "cycle0_frame_identity": (context.get("initial_state") or {}).get("first_active_policy_cycle", {}).get("first_persisted_active_frame_identity"),
        "cycle0_session_id": (context.get("initial_state") or {}).get("first_active_policy_cycle", {}).get("session_id"),
        "cycle0_policy_cycle_id": (context.get("initial_state") or {}).get("first_active_policy_cycle", {}).get("policy_cycle_id"),
        "cycle0_sim_time_s": (context.get("initial_state") or {}).get("first_active_policy_cycle", {}).get("sim_time_s"),
        "cycle0_sim_time_valid": (context.get("initial_state") or {}).get("first_active_policy_cycle", {}).get("sim_time_valid"),
        "cycle0_world_position_xy_m": (context.get("initial_state") or {}).get("first_active_policy_cycle", {}).get("world_position_xy_m"),
        "cycle0_world_yaw_rad": (context.get("initial_state") or {}).get("first_active_policy_cycle", {}).get("world_yaw_rad"),
        "cycle0_imu_quaternion": (context.get("initial_state") or {}).get("first_active_policy_cycle", {}).get("imu_quaternion"),
        "cycle0_joint_order": (context.get("initial_state") or {}).get("first_active_policy_cycle", {}).get("joint_order"),
        "cycle0_joint_position_rad": (context.get("initial_state") or {}).get("first_active_policy_cycle", {}).get("joint_position_rad"),
        "cycle0_joint_velocity_rad_s": (context.get("initial_state") or {}).get("first_active_policy_cycle", {}).get("joint_velocity_rad_s"),
        "supervisor_terminal": metrics.get("supervisor_terminal_result"),
        "navigation_terminal": metrics.get("navigation_terminal_result"),
        "goal_x_m": context.get("settings", {}).get("goal_world_m", [None])[0],
        "arrival_radius_m": context.get("settings", {}).get("arrival_threshold_m"),
        "distance_to_goal_final_m": metrics.get("target_distance_final_m"),
        "task_elapsed_sim_s": boundary.get("elapsed_sim_s"),
        "valid_continuous_sim_s": boundary.get("valid_continuous_sim_s"),
        "task_time_coverage_fraction": metrics.get("task_valid_time_coverage_fraction"),
        "path_length_valid_segments_m": boundary.get("valid_continuous_path_length_m"),
        "mean_speed_valid_segments_mps": boundary.get("mean_valid_continuous_speed_mps"),
        "peak_interval_speed_mps": metrics.get("peak_interval_path_speed_mps"),
        "mode_switches_all_cycles": transition_count,
        "mode_switches_with_valid_sim_time": metrics.get("transitions_with_valid_sim_time"),
        "A_to_Recovery": metrics.get("agile_to_recovery"), "Recovery_to_A": metrics.get("recovery_to_agile"),
        "short_reverse_count_observed": metrics.get("short_reverse_switch_count"),
        "short_reverse_coverage": metrics.get("short_reverse_switch_coverage"),
        "short_reverse_per_valid_min_if_complete": (
            metrics.get("short_reverse_switch_count") / (metrics.get("valid_motion_duration_s") / 60)
            if metrics.get("short_reverse_switch_coverage") == "complete" and
            finite(metrics.get("valid_motion_duration_s")) not in (None, 0) else None),
        "Recovery_exit_count_observed": reentry.get("exit_count"),
        "Recovery_reentry_within_0_5s": reentry.get("reentries_within_0_5s"),
        "Recovery_exit_no_reentry_observed_0_5s": reentry.get("observed_no_reentry"),
        "Recovery_reentry_window_censored": reentry.get("censored"),
        "Recovery_reentry_window_unknown": reentry.get("unknown"),
        "Recovery_reentry_observable_denominator": reentry.get("observable_denominator"),
        "Recovery_reentry_rate_observable_only": reentry.get("reentry_rate_observed"),
        "recovery_cycles": metrics.get("recovery_cycles"), "agile_cycles": metrics.get("agile_cycles"),
        "recovery_valid_time_s": metrics.get("recovery_sim_duration_s"),
        "recovery_share_valid_time": metrics.get("recovery_share_valid_motion_time"),
        "recovery_residence_median_s": median(residence), "recovery_residence_p90_s": percentile(residence, .90),
        "recovery_episode_intervals": len(recovery_intervals),
        "risk_to_mode_paired_count": risk.get("paired_Agile_to_Recovery_count"),
        "risk_to_mode_decision_median_ns": risk.get("median_ns"),
        "risk_reentered_while_recovery_count": risk.get("risk_reentered_while_already_Recovery_count"),
        "ra_min_task": min(ras) if ras else None, "ra_max_task": max(ras) if ras else None,
        "policy_state_cycle_counts": dict(state_counts), "action_source_cycle_counts": dict(action_counts),
        "collision_terminal": metrics.get("navigation_terminal_result") == "COLLISION_TERMINATION",
        "nonfoot_collision": safety.get("nonfoot_collision_failure"),
        "collision_events_count": len(safety.get("collision_events") or []),
        "collision_coverage_complete": safety.get("coverage_complete_for_motion_interval"),
        "collision_coverage_reason": safety.get("coverage_reason"),
        "fall": safety.get("fall"), "fall_result_basis": safety.get("fall_result_basis"),
        "foot_contact_count": safety.get("foot_contact_count"), "foot_impact_count": safety.get("foot_impact_count"),
        "foot_contact_duration_s": safety.get("foot_contact_duration_s"),
        "foot_impact_duration_s": safety.get("foot_impact_duration_s"),
        "declared_boundary_status": (metrics.get("declared_boundary_observation") or {}).get("status"),
        "task_cycle_gaps": metrics.get("motion_policy_cycle_gaps"),
        "task_invalid_clock_cycles": metrics.get("motion_invalid_clock_cycles"),
        "full_capture_invalid_clock_frames": metrics.get("invalid_clock_frames"),
        "policy_frequency_hz": metrics.get("policy_frequency_hz_overall"),
        "policy_interval_median_ms": metrics.get("policy_interval_ms_median"),
        "policy_interval_p95_ms": metrics.get("policy_interval_ms_p95"),
        "hold_audit_status": hold.get("status"), "hold_episodes": len(hold.get("episodes") or []),
        "hold_episodes_complete": sum(ep.get("censored") is False and ep.get("clock_coverage") == "complete" for ep in hold.get("episodes", [])),
        "hold_episodes_incomplete": sum(ep.get("censored") is True or ep.get("clock_coverage") != "complete" for ep in hold.get("episodes", [])),
        "hold_violations": len(hold.get("violations") or []),
        "hold_entry_ready_cycle0": readiness.get("first_active_policy_frame_exactly_persisted"),
        "recorder_ready_before_RL": readiness.get("recorder_ready_before_rl"),
        "standing_observation_status": metrics.get("standing_observation_status"),
        "standing_observation_complete": (lifecycle.get("standing_observation") or {}).get("complete"),
        "PASSIVE_transition_confirmed": metrics.get("passive_controller_transition_confirmed"),
        "process_shutdown_complete": metrics.get("process_shutdown_complete"),
        "closeout_complete": metrics.get("closeout_complete"),
        "cycle_history_drained": metrics.get("policy_cycle_history_complete_at_final_closeout_drain"),
        "terminal_frame_capture": terminal_capture,
        "terminal_frame_ref": terminal_ref,
        "command_consumption_evidence": metrics.get("command_consumption"),
        "actual_group": context.get("settings", {}).get("group"),
        "actual_candidate": context.get("settings", {}).get("candidate"),
        "actual_E": context.get("settings", {}).get("entry_threshold"),
        "actual_X_effective": context.get("settings", {}).get("exit_threshold"),
        "actual_H_sim_s": context.get("settings", {}).get("recovery_min_hold_s"),
        "actual_hysteresis": context.get("settings", {}).get("hysteresis"),
        "actual_hold": context.get("settings", {}).get("hold"),
        "actual_goal_world_m": context.get("settings", {}).get("goal_world_m"),
        "runtime_frame_version": (payloads[0].get("runtime_frame_version") if payloads else None),
        "analysis_identity_issues": identity_issues,
        "consumption_issues": consumption_issues,
    }
    hold_rows = hold_episode_details(plan_item, metrics, frames)
    risk_rows = risk.get("events") or []
    risk_rows = [{"sequence": plan_item["sequence"], "map": plan_item["map"], "group": plan_item["group"],
                  "repeat": plan_item["repeat"], "run_id": metrics.get("run_id"), **event} for event in risk_rows]
    return row, hold_rows, risk_rows


def percentile(values: list[float], q: float):
    xs = sorted(x for x in (finite(v) for v in values) if x is not None)
    if not xs:
        return None
    return xs[min(len(xs) - 1, max(0, math.ceil(q * len(xs)) - 1))]


def group_summaries(rows: list[dict]) -> list[dict]:
    output = []
    for group in GROUPS:
        subset = [r for r in rows if r["group"] == group]
        row = {"group": group, "planned": 12, "valid_records": sum(r["record_validity"] == "VALID" for r in subset),
               "ARRIVED": sum(r["navigation_terminal"] == "ARRIVED" for r in subset),
               "COLLISION_TERMINATION": sum(r["navigation_terminal"] == "COLLISION_TERMINATION" for r in subset),
               "FALL_TERMINATION": sum(r["navigation_terminal"] == "FALL_TERMINATION" for r in subset),
               "DIAGNOSTIC_SIM_TIME_LIMIT": sum(r["navigation_terminal"] == "DIAGNOSTIC_SIM_TIME_LIMIT" for r in subset),
               "WALL_CLOCK_GUARD_INCOMPLETE": sum(r["navigation_terminal"] == "WALL_CLOCK_GUARD_INCOMPLETE" for r in subset),
               "other_terminals": sum(r["navigation_terminal"] not in ("ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION", "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE") for r in subset)}
        for key in ("distance_to_goal_final_m", "task_elapsed_sim_s", "valid_continuous_sim_s",
                    "task_time_coverage_fraction", "path_length_valid_segments_m", "mean_speed_valid_segments_mps",
                    "peak_interval_speed_mps", "mode_switches_all_cycles", "mode_switches_with_valid_sim_time",
                    "short_reverse_count_observed", "short_reverse_per_valid_min_if_complete", "recovery_share_valid_time",
                    "recovery_residence_median_s", "policy_frequency_hz", "foot_contact_count", "foot_impact_count"):
            row["map_equal_mean_" + key] = map_equal_mean(subset, key)
        row["nonfoot_collision_true_false_unknown"] = dict(Counter(
            "UNKNOWN" if r["nonfoot_collision"] not in (True, False) else str(r["nonfoot_collision"]) for r in subset))
        row["collision_terminal_count"] = row["COLLISION_TERMINATION"]
        row["fall_true_false_unknown"] = dict(Counter(
            "UNKNOWN" if r["fall"] not in (True, False) else str(r["fall"]) for r in subset))
        row["short_reverse_complete_runs"] = sum(r["short_reverse_coverage"] == "complete" for r in subset)
        row["hold_audit_statuses"] = dict(Counter(str(r["hold_audit_status"]) for r in subset))
        row["hold_episodes"] = sum(r["hold_episodes"] or 0 for r in subset)
        row["hold_complete_episodes"] = sum(r["hold_episodes_complete"] or 0 for r in subset)
        row["hold_incomplete_episodes"] = sum(r["hold_episodes_incomplete"] or 0 for r in subset)
        row["hold_violations"] = sum(r["hold_violations"] or 0 for r in subset)
        for key in ("Recovery_exit_count_observed", "Recovery_reentry_within_0_5s",
                    "Recovery_exit_no_reentry_observed_0_5s", "Recovery_reentry_window_censored",
                    "Recovery_reentry_window_unknown", "Recovery_reentry_observable_denominator"):
            row[key] = sum(r[key] or 0 for r in subset)
        den = row["Recovery_reentry_observable_denominator"]
        row["Recovery_reentry_rate_observable_only"] = row["Recovery_reentry_within_0_5s"] / den if den else None
        row["standing_observation_complete"] = sum(r["standing_observation_complete"] is True for r in subset)
        row["shutdown_closeout_complete"] = sum(r["process_shutdown_complete"] is True and r["closeout_complete"] is True for r in subset)
        output.append(row)
    return output


def map_equal_mean(rows: list[dict], key: str):
    by_map = defaultdict(list)
    for row in rows:
        value = finite(row.get(key))
        if value is not None:
            by_map[row["map"]].append(value)
    return mean([mean(values) for values in by_map.values()])


def map_group_table(rows: list[dict]) -> list[dict]:
    out = []
    for scene in MAPS:
        for group in GROUPS:
            cell = [r for r in rows if r["map"] == scene and r["group"] == group]
            out.append({"map": scene, "group": group, "planned": 2,
                "valid": sum(r["record_validity"] == "VALID" for r in cell),
                "run_ids": [r["run_id"] for r in sorted(cell, key=lambda r: r["repeat"])],
                "terminals_by_repeat": [r["navigation_terminal"] for r in sorted(cell, key=lambda r: r["repeat"])],
                "arrived": sum(r["navigation_terminal"] == "ARRIVED" for r in cell),
                "task_time_mean_s": mean([r["task_elapsed_sim_s"] for r in cell]),
                "successful_arrival_time_mean_s": mean([r["task_elapsed_sim_s"] for r in cell if r["navigation_terminal"] == "ARRIVED"]),
                "distance_final_mean_m": mean([r["distance_to_goal_final_m"] for r in cell]),
                "valid_time_mean_s": mean([r["valid_continuous_sim_s"] for r in cell]),
                "valid_path_mean_m": mean([r["path_length_valid_segments_m"] for r in cell]),
                "valid_speed_mean_mps": mean([r["mean_speed_valid_segments_mps"] for r in cell]),
                "all_cycle_switches_mean": mean([r["mode_switches_all_cycles"] for r in cell]),
                "valid_time_switches_mean": mean([r["mode_switches_with_valid_sim_time"] for r in cell]),
                "short_reverse_counts": [r["short_reverse_count_observed"] for r in sorted(cell, key=lambda r: r["repeat"])],
                "short_reverse_coverage": [r["short_reverse_coverage"] for r in sorted(cell, key=lambda r: r["repeat"])],
                "recovery_share_mean": mean([r["recovery_share_valid_time"] for r in cell]),
                "nonfoot_collision_values": [r["nonfoot_collision"] for r in sorted(cell, key=lambda r: r["repeat"])],
                "fall_values": [r["fall"] for r in sorted(cell, key=lambda r: r["repeat"])],
                "foot_contact_total": sum(r["foot_contact_count"] or 0 for r in cell),
                "foot_impact_total": sum(r["foot_impact_count"] or 0 for r in cell),
                "hold_audit_statuses": [r["hold_audit_status"] for r in sorted(cell, key=lambda r: r["repeat"])],
                "hold_episodes_total": sum(r["hold_episodes"] or 0 for r in cell),
                "hold_violations_total": sum(r["hold_violations"] or 0 for r in cell),
                "Recovery_exit_count_total": sum(r["Recovery_exit_count_observed"] or 0 for r in cell),
                "Recovery_reentry_within_0_5s_total": sum(r["Recovery_reentry_within_0_5s"] or 0 for r in cell),
                "Recovery_reentry_observable_denominator": sum(r["Recovery_reentry_observable_denominator"] or 0 for r in cell),
                "Recovery_reentry_window_censored": sum(r["Recovery_reentry_window_censored"] or 0 for r in cell),
                "standing_complete": sum(r["standing_observation_complete"] is True for r in cell),
                "shutdown_closeout_complete": sum(r["process_shutdown_complete"] is True and r["closeout_complete"] is True for r in cell)})
    return out


def rate_bounds(rows: list[dict], key: str, maps: list[str]):
    lower, upper, known = [], [], []
    for scene in maps:
        cell = [r for r in rows if r["map"] == scene]
        n = len(cell)
        yes = sum(r.get(key) is True for r in cell)
        unk = sum(r.get(key) not in (True, False) for r in cell)
        lower.append(yes / n if n else None)
        upper.append((yes + unk) / n if n else None)
        known.append(yes / (n - unk) if n - unk else None)
    return mean(lower), mean(upper), mean(known)


def contrasts(rows: list[dict]) -> tuple[list[dict], list[dict]]:
    pairs = (("C-A", "C", "A"), ("D-B", "D", "B"), ("D-C", "D", "C"), ("B-A", "B", "A"))
    metrics = ("arrived", "nonfoot_collision", "task_elapsed_sim_s", "valid_continuous_sim_s",
               "path_length_valid_segments_m", "mode_switches_all_cycles",
               "mode_switches_with_valid_sim_time", "recovery_share_valid_time",
               "short_reverse_per_valid_min_if_complete")
    output, paired_times = [], []
    for label, candidate, reference in pairs:
        item = {"contrast": label, "candidate": candidate, "reference": reference}
        for key in metrics:
            if key == "arrived":
                a = map_equal_mean([dict(r, arrived=float(r["navigation_terminal"] == "ARRIVED")) for r in rows if r["group"] == candidate], "arrived")
                b = map_equal_mean([dict(r, arrived=float(r["navigation_terminal"] == "ARRIVED")) for r in rows if r["group"] == reference], "arrived")
                item["arrival_rate_map_equal_difference"] = a - b if a is not None and b is not None else None
                continue
            if key == "nonfoot_collision":
                cand_lo, cand_hi, cand_known = rate_bounds([r for r in rows if r["group"] == candidate], key, list(MAPS))
                ref_lo, ref_hi, ref_known = rate_bounds([r for r in rows if r["group"] == reference], key, list(MAPS))
                item.update({"collision_diff_candidate_worst_upper_minus_reference_lower": cand_hi - ref_lo if cand_hi is not None and ref_lo is not None else None,
                    "collision_diff_candidate_best_lower_minus_reference_upper": cand_lo - ref_hi if cand_lo is not None and ref_hi is not None else None,
                    "collision_diff_known_condition_only": cand_known - ref_known if cand_known is not None and ref_known is not None else None,
                    "candidate_collision_bounds_map_equal":[cand_lo,cand_hi],"reference_collision_bounds_map_equal":[ref_lo,ref_hi]})
                continue
            if key == "task_elapsed_sim_s":
                cand = map_equal_mean([r for r in rows if r["group"] == candidate and r["navigation_terminal"] == "ARRIVED"],key)
                ref = map_equal_mean([r for r in rows if r["group"] == reference and r["navigation_terminal"] == "ARRIVED"],key)
                item["successful_arrival_time_map_equal_candidate_minus_reference_s"] = cand-ref if cand is not None and ref is not None else None
                continue
            cand = map_equal_mean([r for r in rows if r["group"] == candidate], key)
            ref = map_equal_mean([r for r in rows if r["group"] == reference], key)
            item[f"{key}_map_equal_candidate_minus_reference"] = cand - ref if cand is not None and ref is not None else None
        by = {(r["map"],r["repeat"]):r for r in rows}
        diffs=[]; pairs_n=0
        for scene in MAPS:
            for repeat in (1,2):
                a=by[(scene,repeat)] if (scene,repeat) in by else None
                cand=next((r for r in rows if r["map"]==scene and r["repeat"]==repeat and r["group"]==candidate),None)
                ref=next((r for r in rows if r["map"]==scene and r["repeat"]==repeat and r["group"]==reference),None)
                if cand and ref and cand["navigation_terminal"]==ref["navigation_terminal"]=="ARRIVED":
                    diff=float(cand["task_elapsed_sim_s"])-float(ref["task_elapsed_sim_s"])
                    diffs.append(diff); pairs_n+=1
                    paired_times.append({"contrast":label,"map":scene,"repeat":repeat,
                        "candidate_run_id":cand["run_id"],"reference_run_id":ref["run_id"],
                        "candidate_arrival_s":cand["task_elapsed_sim_s"],"reference_arrival_s":ref["task_elapsed_sim_s"],
                        "candidate_minus_reference_s":diff,
                        "pairing_scope":"same scheduled map/repeat; not exact initial-state replay"})
        item["same_map_repeat_common_arrival_pairs"] = pairs_n
        item["common_arrival_time_difference_mean_s"] = mean(diffs)
        item["common_arrival_time_difference_median_s"] = median(diffs)
        item["common_arrival_time_difference_min_s"] = min(diffs) if diffs else None
        item["common_arrival_time_difference_max_s"] = max(diffs) if diffs else None
        output.append(item)
    return output, paired_times


def plot_outputs(out: Path, rows: list[dict], hold_rows: list[dict], plan: dict):
    charts = out / "charts"; charts.mkdir(parents=True, exist_ok=True)
    groups = list(GROUPS); positions = np.arange(len(groups))
    terminals = ("ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION", "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE")
    fig, ax = plt.subplots(figsize=(9,5)); bottom=np.zeros(4); palette=("#42966c","#b64a45","#7d4e8b","#d4a72c","#6b7280")
    for terminal,color in zip(terminals,palette):
        counts=np.array([sum(r["group"]==g and r["navigation_terminal"]==terminal for r in rows) for g in groups])
        ax.bar(groups,counts,bottom=bottom,label=terminal,color=color); bottom+=counts
    ax.set_ylabel("Runs (n=12 per group)"); ax.set_title("T2 development check terminal outcomes"); ax.legend(fontsize=8); ax.grid(axis="y",alpha=.2)
    fig.tight_layout(); fig.savefig(charts/"terminal_outcomes.png",dpi=160); plt.close(fig)

    fig, axes=plt.subplots(1,2,figsize=(11,4.5))
    keys=["mode_switches_all_cycles","short_reverse_per_valid_min_if_complete"]
    labels=["All observed mode edges per run","Short reverse / effective minute\n(complete-coverage runs only)"]
    for ax,key,label in zip(axes,keys,labels):
        vals=[mean([r.get(key) for r in rows if r["group"]==g]) for g in groups]
        ax.bar(groups,[v or 0 for v in vals],color=[COLORS[g] for g in groups]); ax.set_title(label); ax.set_ylabel("Map-equal mean")
        ax.grid(axis="y",alpha=.2)
        if key.startswith("short_reverse"):
            ax.text(.02,.98,"n complete="+", ".join(str(sum(r['group']==g and r['short_reverse_coverage']=='complete' for r in rows)) for g in groups),transform=ax.transAxes,va="top",fontsize=8)
    fig.tight_layout(); fig.savefig(charts/"switching_metrics.png",dpi=160); plt.close(fig)

    fig, axes=plt.subplots(1,2,figsize=(11,4.5))
    share=[map_equal_mean([r for r in rows if r["group"]==g],"recovery_share_valid_time") for g in groups]
    timeouts=[sum(r["group"]==g and r["navigation_terminal"]=="DIAGNOSTIC_SIM_TIME_LIMIT" for r in rows) for g in groups]
    axes[0].bar(groups,[v or 0 for v in share],color=[COLORS[g] for g in groups]); axes[0].set_title("Recovery share of valid task time"); axes[0].set_ylabel("Map-equal fraction")
    axes[1].bar(groups,timeouts,color=[COLORS[g] for g in groups]); axes[1].set_title("Simulation-time-limit terminals"); axes[1].set_ylabel("Runs")
    for ax in axes: ax.grid(axis="y",alpha=.2)
    fig.tight_layout(); fig.savefig(charts/"recovery_occupancy_and_timeouts.png",dpi=160); plt.close(fig)

    fig,ax=plt.subplots(figsize=(9,5)); mat=np.full((6,4),np.nan); texts=[]
    for yi,scene in enumerate(MAPS):
        for xi,group in enumerate(GROUPS):
            cell=[r for r in rows if r["map"]==scene and r["group"]==group]
            arrivals=sum(r["navigation_terminal"]=="ARRIVED" for r in cell)
            mat[yi,xi]=arrivals/2
            terms="/".join(r["navigation_terminal"].replace("DIAGNOSTIC_SIM_TIME_LIMIT","TIME_LIMIT").replace("COLLISION_TERMINATION","COLLISION").replace("ARRIVED","ARR") for r in sorted(cell,key=lambda r:r["repeat"]))
            texts.append((yi,xi,terms))
    im=ax.imshow(mat,vmin=0,vmax=1,cmap="RdYlGn",aspect="auto"); ax.set_xticks(range(4),GROUPS); ax.set_yticks(range(6),[m.replace('.xml','') for m in MAPS]); ax.set_title("Map × group terminal display\ncell color = ARRIVED fraction; text = repeat 1 / 2 terminal")
    for y,x,t in texts: ax.text(x,y,t,ha="center",va="center",fontsize=7,color="black")
    fig.colorbar(im,ax=ax,label="ARRIVED fraction (2 runs)"); fig.tight_layout(); fig.savefig(charts/"map_group_terminals.png",dpi=170); plt.close(fig)

    fig,ax=plt.subplots(figsize=(8,4.5)); cd=["C","D"]
    for i,g in enumerate(cd):
        subset=[r for r in rows if r["group"]==g]
        complete=sum(r["hold_episodes_complete"] or 0 for r in subset)
        incomplete=sum(r["hold_episodes_incomplete"] or 0 for r in subset)
        ax.bar(g,complete,color=COLORS[g],label="complete episodes" if i==0 else None)
        ax.bar(g,incomplete,bottom=complete,color="#c9cdd3",label="incomplete/censored" if i==0 else None)
    ax.set_title("C/D Recovery hold episode coverage"); ax.set_ylabel("Episodes"); ax.legend(); ax.grid(axis="y",alpha=.2)
    fig.tight_layout(); fig.savefig(charts/"hold_episode_coverage.png",dpi=160); plt.close(fig)

    # Four observed first-round e03 runs; illustrative traces only, not selected by outcome.
    fig,axes=plt.subplots(4,1,figsize=(11,11),sharex=True)
    for ax,g in zip(axes,GROUPS):
        item=next(x for x in plan["schedule"] if x["round"]==1 and x["map"]=="e03.xml" and x["group"]==g)
        attempt=next(a for a in item["attempts"] if a.get("run_directory") and a.get("run_id"))
        run_dir=Path(attempt["run_directory"]); metrics=load(run_dir/"metrics.json"); context=load(run_dir/"run_context.json")
        fs,_=frames_in_task(read_frames(run_dir),metrics,context)
        ts=[finite(f.get("sim_time_s")) for f in fs]; ra=[finite(f.get("ra_value")) for f in fs]
        ax.plot(ts,ra,color=COLORS[g],lw=1,label="RA"); ax.axhline(-.05,color="#333",ls="--",lw=.8,label="E=-0.05")
        if g in ("B","D"): ax.axhline(-.10,color="#a85572",ls=":",lw=.8,label="X=-0.10")
        states=[1 if f.get("policy_state_name")=="RECOVERY" else 0 for f in fs]
        twin=ax.twinx(); twin.step(ts,states,where="post",color="#222",alpha=.35,lw=.8); twin.set_yticks([0,1],labels=["Agile","Recovery"]); twin.set_ylim(-.2,1.2)
        terminal=next(x for x in plan["schedule"] if x["round"]==1 and x["map"]=="e03.xml" and x["group"]==g)
        att=next(a for a in terminal["attempts"] if a.get("run_id"))
        ax.set_ylabel(f"{g} RA"); ax.grid(alpha=.18); ax.legend(loc="upper right",fontsize=7,ncol=3)
        ax.set_title(f"Round 1 e03 {g}: terminal {att.get('terminal','UNKNOWN')} (policy frames)",fontsize=9)
    axes[-1].set_xlabel("Simulation time (s)"); fig.tight_layout(); fig.savefig(charts/"example_timeline_e03_round1.png",dpi=170); plt.close(fig)


def main() -> int:
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument("--plan",type=Path,required=True); parser.add_argument("--output-dir",type=Path,required=True); args=parser.parse_args()
    plan=load(args.plan); out=args.output_dir.resolve(); out.mkdir(parents=True,exist_ok=True)
    if plan.get("status") not in {"RUNS_COMPLETE_PENDING_ANALYSIS","ANALYSIS_FAILED_RAW_DATA_PRESERVED","ANALYSIS_COMPLETE_PENDING_DIRECTOR_REVIEW","COMPLETE"}:
        raise SystemExit(f"batch is not ready for analysis: {plan.get('status')}")
    if len(plan.get("schedule",[]))!=48:
        raise SystemExit("expected frozen 48-item matrix")
    rows=[]; hold_rows=[]; risk_rows=[]; problems=[]
    for item in plan["schedule"]:
        row,episodes,risks=extract_run(item,plan); rows.append(row); hold_rows.extend(episodes); risk_rows.extend(risks)
        if row["consumption_issues"] or row["analysis_identity_issues"]:
            problems.append({"sequence":row["sequence"],"run_id":row["run_id"],
                "consumption_issues":row["consumption_issues"],"identity_issues":row["analysis_identity_issues"]})
    if len(rows)!=48 or len({r["run_id"] for r in rows})!=48:
        raise SystemExit("48 unique run records not available")
    write_csv(out/"per_run_results.csv",rows)
    map_rows=map_group_table(rows); write_csv(out/"map_group_24.csv",map_rows)
    summaries=group_summaries(rows); write_csv(out/"group_summary.csv",summaries)
    write_csv(out/"hold_episode_audit.csv",hold_rows)
    write_csv(out/"risk_to_recovery_events.csv",risk_rows)
    contrast_rows,paired=contrasts(rows); write_csv(out/"factor_contrasts.csv",contrast_rows); write_csv(out/"paired_arrival_time_comparisons.csv",paired)
    # Count process-level safety and status facts without converting UNKNOWN to false.
    status={'runs':len(rows),'valid':sum(r['record_validity']=='VALID' for r in rows),
        'terminals':dict(Counter(str(r['navigation_terminal']) for r in rows)),
        'groups':{g:{'n':sum(r['group']==g for r in rows),'arrived':sum(r['group']==g and r['navigation_terminal']=='ARRIVED' for r in rows),
            'collision_termination':sum(r['group']==g and r['navigation_terminal']=='COLLISION_TERMINATION' for r in rows),
            'simulation_time_limit':sum(r['group']==g and r['navigation_terminal']=='DIAGNOSTIC_SIM_TIME_LIMIT' for r in rows),
            'nonfoot_collision':dict(Counter('UNKNOWN' if r['nonfoot_collision'] not in (True,False) else str(r['nonfoot_collision']) for r in rows if r['group']==g)),
            'fall':dict(Counter('UNKNOWN' if r['fall'] not in (True,False) else str(r['fall']) for r in rows if r['group']==g)),
            'hold_audit':dict(Counter(str(r['hold_audit_status']) for r in rows if r['group']==g)),
            'hold_episodes':sum(r['hold_episodes'] or 0 for r in rows if r['group']==g),
            'hold_episodes_complete':sum(r['hold_episodes_complete'] or 0 for r in rows if r['group']==g),
            'hold_episodes_incomplete':sum(r['hold_episodes_incomplete'] or 0 for r in rows if r['group']==g),
            'hold_violations':sum(r['hold_violations'] or 0 for r in rows if r['group']==g)} for g in GROUPS},
        'run_integrity_issue_count':len(problems),'run_integrity_issues':problems,
        'all_shutdown_closeout_confirmed':all(r['process_shutdown_complete'] is True and r['closeout_complete'] is True for r in rows),
        'all_cycle_history_drained':all(r['cycle_history_drained'] is True for r in rows),
        'all_cycle0_exactly_persisted':all(r['hold_entry_ready_cycle0'] is True for r in rows),
        'pre_scene_orchestration_failures':plan.get('pre_scene_failures',[])}
    (out/"analysis_summary.json").write_text(json.dumps(status,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    plot_outputs(out,rows,hold_rows,plan)
    print(json.dumps({'runs':len(rows),'integrity_issues':len(problems),'terminals':status['terminals'],
        'groups':status['groups'],'analysis_dir':str(out)},ensure_ascii=False,indent=2))
    return 1 if problems else 0


if __name__=="__main__":
    raise SystemExit(main())
