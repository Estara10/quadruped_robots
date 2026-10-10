#!/usr/bin/env python3
"""Compute the small S1-04 run table and timelines from saved per-run records."""
from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path
from typing import Any

from run_record import load_record, summarize_record
from abs_collision import CollisionStatus


def quantile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    lo, hi = math.floor(pos), math.ceil(pos)
    return ordered[lo] if lo == hi else ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)


def motion_safety(frames: list[dict], start_s: float | None,
                  end_s: float | None,
                  terminal_snapshot: dict | None = None) -> dict[str, Any]:
    """Classify collision and fall evidence only inside inclusive motion bounds."""
    snapshots = [p.get("collision_snapshot") for p in frames]
    if isinstance(terminal_snapshot, dict):
        snapshots.append(terminal_snapshot)
    source_complete = bool(frames) and start_s is not None and end_s is not None
    source_complete = source_complete and all(
        isinstance(item, dict) and item.get("status") == CollisionStatus.LIVE.value and
        item.get("version") in (4, 5) and item.get("physics_coverage_complete") is True and
        item.get("collision_history_overflow") is False and
        item.get("fall_history_overflow") is False and
        item.get("event_latches_complete") is True
        for item in snapshots)
    snapshot_times = [item.get("sim_time") for item in snapshots
                      if isinstance(item, dict) and isinstance(item.get("sim_time"), (int, float))]
    source_complete = source_complete and len(snapshot_times) == len(snapshots)
    source_complete = source_complete and min(snapshot_times, default=math.inf) <= start_s
    source_complete = source_complete and max(snapshot_times, default=-math.inf) >= end_s
    history: dict[int, dict] = {}
    for item in snapshots:
        if not isinstance(item, dict):
            continue
        for event in item.get("collision_history", []):
            if isinstance(event, dict):
                key = event.get("start_physics_step")
                if isinstance(key, int) and (key not in history or
                        event.get("end_physics_step", 0) > history[key].get("end_physics_step", 0)):
                    history[key] = event
    motion_events = [event for event in history.values()
                     if start_s is not None and end_s is not None and
                     event.get("end_sim_time", -math.inf) >= start_s and
                     event.get("start_sim_time", math.inf) <= end_s]
    def range_overlaps(item: dict, prefix: str) -> bool:
        count = item.get(f"{prefix}_steps", 0)
        first = item.get(f"first_{prefix}_sim_time", 0.0)
        last = item.get(f"last_{prefix}_sim_time", 0.0)
        return (isinstance(count, int) and count > 0 and
                isinstance(first, (int, float)) and isinstance(last, (int, float)) and
                start_s is not None and end_s is not None and first <= end_s and last >= start_s)

    unknown_contact_overlap = any(isinstance(item, dict) and range_overlaps(item, "unknown_contact")
                                   for item in snapshots)
    invalid_posture_overlap = any(isinstance(item, dict) and range_overlaps(item, "invalid_posture")
                                  for item in snapshots)
    collision = True if motion_events else (
        False if source_complete and not unknown_contact_overlap else "UNKNOWN")
    v5_snapshots = [item for item in snapshots if isinstance(item, dict) and item.get("version") == 5]
    v5_coverage = (source_complete and len(v5_snapshots) == len(snapshots) and bool(v5_snapshots) and
                   not unknown_contact_overlap and
                   all(item.get("foot_identity_valid") is True and
                       item.get("foot_contact_history_overflow") is False and
                       item.get("foot_impact_history_overflow") is False for item in v5_snapshots))

    def unique_events(field: str) -> list[dict]:
        found: dict[tuple, dict] = {}
        for item in v5_snapshots:
            for event in item.get(field, []):
                if not isinstance(event, dict):
                    continue
                key = (event.get("foot_geom_id"), event.get("obstacle_geom_id"),
                       event.get("start_physics_step"))
                previous = found.get(key)
                if previous is None or event.get("end_physics_step", 0) > previous.get("end_physics_step", 0):
                    found[key] = event
        return list(found.values())

    def overlaps_motion(event: dict) -> bool:
        return (start_s is not None and end_s is not None and
                event.get("end_sim_time", -math.inf) >= start_s and
                event.get("start_sim_time", math.inf) <= end_s)

    foot_contacts = [event for event in unique_events("foot_contact_history") if overlaps_motion(event)]
    foot_impacts = [event for event in unique_events("foot_impact_history") if overlaps_motion(event)]
    foot_unknown = any(
        isinstance(item, dict) and item.get("foot_impact_unknown_steps", 0) > 0 and
        item.get("foot_impact_unknown_first_sim_time", math.inf) <= (end_s if end_s is not None else -math.inf) and
        item.get("foot_impact_unknown_last_sim_time", -math.inf) >= (start_s if start_s is not None else math.inf)
        for item in v5_snapshots)
    foot_contact_result: Any = len(foot_contacts) if v5_coverage else "UNKNOWN"
    foot_impact_result: Any = len(foot_impacts) if v5_coverage and not foot_unknown else "UNKNOWN"
    nonfoot_failure: Any = (True if motion_events else False) if v5_coverage else (
        "UNKNOWN" if motion_events or not source_complete else False)
    foot_contact_duration: Any = "UNKNOWN"
    foot_impact_duration: Any = "UNKNOWN"
    if v5_coverage:
        before = [item for item in v5_snapshots if item.get("sim_time", math.inf) <= start_s]
        through = [item for item in v5_snapshots if item.get("sim_time", -math.inf) <= end_s]
        after_start = max(before, key=lambda item: item.get("sim_time", -math.inf), default=None)
        through_end = max(through, key=lambda item: item.get("sim_time", -math.inf), default=None)
        if after_start is not None and through_end is not None:
            contact_boundary_crossed = any(
                event.get("start_sim_time", math.inf) < start_s or
                event.get("end_sim_time", -math.inf) > end_s for event in foot_contacts)
            impact_boundary_crossed = any(
                event.get("start_sim_time", math.inf) < start_s or
                event.get("end_sim_time", -math.inf) > end_s for event in foot_impacts)
            foot_contact_duration = ("UNKNOWN: event crosses motion boundary"
                if contact_boundary_crossed else max(0.0, through_end["foot_contact_duration_s"] - after_start["foot_contact_duration_s"]))
            foot_impact_duration = ("UNKNOWN: event crosses motion boundary"
                if impact_boundary_crossed else max(0.0, through_end["foot_impact_duration_s"] - after_start["foot_impact_duration_s"]))
    fall_history: dict[int, dict] = {}
    for item in snapshots:
        if not isinstance(item, dict):
            continue
        for event in item.get("fall_history", []):
            if not isinstance(event, dict):
                continue
            key = event.get("start_physics_step")
            if isinstance(key, int) and (key not in fall_history or
                    event.get("end_physics_step", 0) > fall_history[key].get("end_physics_step", 0) or
                    event.get("confirmed") is True):
                fall_history[key] = event
    motion_falls = [event for event in fall_history.values()
                    if start_s is not None and end_s is not None and
                    event.get("end_sim_time", -math.inf) >= start_s and
                    event.get("start_sim_time", math.inf) <= end_s]
    confirmed_motion_falls = [event for event in motion_falls if event.get("confirmed") is True]
    unresolved_motion_fall = any(event.get("confirmed") is not True for event in motion_falls)
    fall = True if confirmed_motion_falls else (
        False if source_complete and not invalid_posture_overlap and not unresolved_motion_fall else "UNKNOWN")
    return {
        "motion_interval_sim_s_inclusive": [start_s, end_s],
        "collision": collision,
        "collision_events": sorted(motion_events, key=lambda item: item.get("start_sim_time", 0.0)),
        "nonfoot_collision_failure": nonfoot_failure,
        "foot_contact_count": foot_contact_result,
        "foot_contact_duration_s": foot_contact_duration,
        "foot_contact_events": foot_contacts,
        "foot_contact_duration_capture_s": max(
            (item.get("foot_contact_duration_s", 0.0) for item in v5_snapshots), default=None),
        "foot_impact_count": foot_impact_result,
        "foot_impact_duration_s": foot_impact_duration,
        "foot_impact_events": foot_impacts,
        "foot_impact_duration_capture_s": max(
            (item.get("foot_impact_duration_s", 0.0) for item in v5_snapshots), default=None),
        "foot_impact_unknown_overlaps_motion": foot_unknown,
        "abs_reference_collision": (True if nonfoot_failure is True or
                                    (isinstance(foot_impact_result, int) and foot_impact_result > 0)
                                    else False if nonfoot_failure is False and foot_impact_result == 0
                                    else "UNKNOWN"),
        "fall": fall,
        "fall_events": sorted(motion_falls, key=lambda item: item.get("start_sim_time", 0.0)),
        "fall_event": (confirmed_motion_falls[-1] if confirmed_motion_falls else None),
        "coverage_complete_for_motion_interval": bool(source_complete),
        "coverage_reason": None if source_complete else
            "requires valid v4 snapshots, contiguous physics publisher, bounded motion interval, and no relevant unclassified contact, invalid posture, or history overflow",
        "unclassified_contact_overlaps_motion": unknown_contact_overlap,
        "invalid_posture_overlaps_motion": invalid_posture_overlap,
        "unconfirmed_fall_candidate_overlaps_motion": unresolved_motion_fall,
        "max_observed_contacts_per_policy_snapshot": {
            "foot_floor": max((item.get("foot_ground_contacts", 0) for item in snapshots if isinstance(item, dict)), default=None),
            "nonfoot_floor": max((item.get("nonfoot_ground_contacts", 0) for item in snapshots if isinstance(item, dict)), default=None),
            "self": max((item.get("self_contacts", 0) for item in snapshots if isinstance(item, dict)), default=None),
        },
    }


def calculate(record_path: Path, context_path: Path, reverse_window_s: float = 0.5) -> dict[str, Any]:
    data = load_record(str(record_path))
    base = summarize_record(str(record_path))
    context = json.loads(context_path.read_text(encoding="utf-8"))
    settings = context.get("settings", {})
    context_scene_name = (settings.get("scene_filename") or
                          Path(settings.get("scene", "")).name or "UNKNOWN")
    record_scene = data.meta.get("scene_binding")
    expected_scene = (settings.get("scene_filename"), settings.get("scene_id"),
                      settings.get("scene_obstacle_count"))
    if record_scene is not None and expected_scene[1] is not None and (
            record_scene.get("filename"), record_scene.get("scene_id"),
            record_scene.get("obstacle_count")) != expected_scene:
        return {"run_id": context.get("run_id"), "scene": expected_scene[0] or context_scene_name,
                "scene_id": expected_scene[1], "record_validity": "INVALID",
                "record_validity_reasons": ["run_context_record_scene_binding_mismatch"],
                "terminal_result": "INVALID_SCENE_BINDING", "frame_count": len(data.frames)}
    closeout_errors = context.get("lifecycle", {}).get("closeout_errors", [])
    frames = [line["payload"] for line in data.frames if line.get("status") == "LIVE" and isinstance(line.get("payload"), dict)]
    frames.sort(key=lambda p: (p["session_id"], p["rl_step"]))
    if closeout_errors:
        return {
            "run_id": context.get("run_id"), "record_validity": "INVALID",
            "record_validity_reason_count": len(closeout_errors),
            "record_validity_reasons": [f"closeout_error:{item.get('stage', 'unknown')}" for item in closeout_errors],
            "terminal_result": "INCOMPLETE_CLOSEOUT",
            "closeout_errors": closeout_errors,
            "frame_count": len(frames),
            "all_derived_metrics": "UNKNOWN: run closeout reported an artifact or cleanup error",
            "external_safety_events": (data.terminal or {}).get("external_safety_events", []),
            "cleanup_events": (data.terminal or {}).get("cleanup_events", []),
        }
    if base.get("record_validity") != "VALID":
        sessions = sorted({p.get("session_id") for p in frames})
        context_result = context.get("result_contract", {}).get("classification", "UNKNOWN")
        record_result = (data.terminal or {}).get("run_terminal_result")
        final_result = ("STARTUP_FAILURE" if context_result == "STARTUP_FAILURE" and record_result == "STARTUP_FAILURE"
                        else "INCOMPLETE_UNTRUSTED_RECORD")
        return {
            "run_id": context.get("run_id"), "record_validity": "INVALID",
            "record_validity_reason_count": len(base.get("record_validity_reasons") or []),
            "record_validity_reasons": sorted(set(base.get("record_validity_reasons") or [])),
            "terminal_result": final_result,
            "run_terminal_result_context": context_result,
            "run_terminal_result_record": record_result,
            "session_ids_observed": sessions, "frame_count": len(frames),
            "all_derived_metrics": "UNKNOWN: record fails continuity/schema validation; no aggregation across sessions",
            "collision_coverage": base.get("collision"),
            "external_safety_events": (data.terminal or {}).get("external_safety_events", []),
            "cleanup_events": (data.terminal or {}).get("cleanup_events", []),
            "command_consumption": "UNKNOWN",
        }
    gaps = 0
    periods_s: list[float] = []
    frequency_hz: list[float] = []
    transitions: list[dict[str, Any]] = []
    usable_pairs: list[tuple[dict, dict, float]] = []
    mode_changes_with_valid_time = 0
    invalid_clock_frames = 0
    for p in frames:
        if p.get("sim_clock_valid") is not True or not isinstance(p.get("sim_time_s"), (int, float)):
            invalid_clock_frames += 1
    for a, b in zip(frames, frames[1:]):
        same_session = a["session_id"] == b["session_id"]
        delta_step = b["rl_step"] - a["rl_step"] if same_session else None
        if not same_session or delta_step != 1:
            gaps += max(1, (delta_step - 1) if delta_step is not None and delta_step > 1 else 1)
            continue
        dt_wall = (b["monotonic_ns"] - a["monotonic_ns"]) / 1e9
        if dt_wall > 0:
            periods_s.append(dt_wall)
            frequency_hz.append(1.0 / dt_wall)
        av = a.get("sim_clock_valid") is True and isinstance(a.get("sim_time_s"), (int, float))
        bv = b.get("sim_clock_valid") is True and isinstance(b.get("sim_time_s"), (int, float))
        if av and bv and b["sim_time_s"] >= a["sim_time_s"]:
            dt = b["sim_time_s"] - a["sim_time_s"]
            if dt > 0:
                usable_pairs.append((a, b, dt))

    # The frame records the actual decision-cycle mode_before/mode_after edge.
    # Use it directly so a transition on the first motion cycle is not lost.
    for p in frames:
        before, after = p.get("mode_before"), p.get("mode_after")
        if before in (0, 1) and after in (0, 1) and before != after:
            when = p.get("sim_time_s") if p.get("sim_clock_valid") is True else None
            if when is not None:
                mode_changes_with_valid_time += 1
            transitions.append({"session_id": p["session_id"], "rl_step": p["rl_step"],
                                "from": before, "to": after, "sim_time_s": when,
                                "period_gap_before": None})
    transitions.sort(key=lambda t: (t["session_id"], t["rl_step"]))

    lifecycle = context.get("lifecycle", {})
    movement_start = lifecycle.get("motion_start_observed_sim_time_s")
    terminal_ref = lifecycle.get("terminal_frame_ref")
    movement_end = lifecycle.get("movement_end_sim_time_s")
    if movement_end is None and isinstance(terminal_ref, dict):
        movement_end = terminal_ref.get("sim_time_s")
    terminal_frame_payload_complete: bool | None = None
    if isinstance(terminal_ref, dict):
        terminal_frame_payload_complete = False
        capture_status = lifecycle.get("terminal_frame_capture", {})
        matched_terminal_frames = [p for p in frames
                                  if p.get("session_id") == terminal_ref.get("session_id")
                                  and p.get("rl_step") == terminal_ref.get("rl_step")
                                  and (terminal_ref.get("source_sequence") is None
                                       or p.get("source_sequence") == terminal_ref.get("source_sequence"))
                                  and (terminal_ref.get("monotonic_ns") is None
                                       or p.get("monotonic_ns") == terminal_ref.get("monotonic_ns"))]
        if matched_terminal_frames:
            payload = matched_terminal_frames[0]
            required_terminal_fields = ("ra_value", "mode_before", "mode_after", "action_source",
                                       "action_raw", "action_clipped", "joint_target_rad", "world_pose",
                                       "monotonic_ns", "sim_time_s", "sim_clock_sequence",
                                       "sim_clock_status", "sim_clock_valid")
            terminal_frame_payload_complete = (
                all(field in payload for field in required_terminal_fields)
                and payload.get("ra_value") is not None
                and payload.get("action_raw") is not None
                and payload.get("action_clipped") is not None
                and payload.get("joint_target_rad") is not None
                and payload.get("world_pose") is not None
                and capture_status.get("complete") is not False
                and terminal_ref.get("same_snapshot_persisted") is not False
            )
    terminal_frame_failure = isinstance(terminal_ref, dict) and not terminal_frame_payload_complete
    terminal_frame_reason = "terminal_frame_payload_missing_or_incomplete" if terminal_frame_failure else None
    # The observed threshold-crossing frame is the operational start boundary.
    # Terminal-frame identity (not cleanup time) is the inclusive end boundary.
    movement_frames = [] if terminal_frame_failure else [p for p in frames
                       if movement_start is not None and movement_end is not None
                       and p.get("sim_clock_valid") is True
                       and isinstance(p.get("sim_time_s"), (int, float))
                       and movement_start <= p["sim_time_s"] <= movement_end
                       and (not isinstance(terminal_ref, dict)
                            or (p.get("session_id"), p.get("rl_step")) <=
                               (terminal_ref.get("session_id", p.get("session_id")), terminal_ref.get("rl_step", p.get("rl_step"))))]
    movement_keys = {(p.get("session_id"), p.get("rl_step")) for p in movement_frames}
    valid_times = [p["sim_time_s"] for p in frames if p.get("sim_clock_valid") is True and isinstance(p.get("sim_time_s"), (int, float))]
    goal = context.get("settings", {}).get("goal_world_m", [None, None])
    goal_distance_start = None
    goal_distance_final = None
    trajectory_length = 0.0
    valid_motion_duration = 0.0
    recovery_duration = 0.0
    motion_pairs: list[tuple[dict, dict, float]] = []
    for a, b, dt in usable_pairs:
        if (a.get("session_id"), a.get("rl_step")) not in movement_keys or (b.get("session_id"), b.get("rl_step")) not in movement_keys:
            continue
        motion_pairs.append((a, b, dt))
        ax, ay = a["world_pose"][:2]
        bx, by = b["world_pose"][:2]
        distance = math.hypot(bx - ax, by - ay)
        trajectory_length += distance
        valid_motion_duration += dt
        if a.get("mode_after") == 1:
            recovery_duration += dt
    if movement_frames and goal and goal[0] is not None:
        valid_pose = movement_frames
        if valid_pose:
            first, last = valid_pose[0], valid_pose[-1]
            goal_distance_start = math.hypot(goal[0] - first["world_pose"][0], goal[1] - first["world_pose"][1])
            goal_distance_final = math.hypot(goal[0] - last["world_pose"][0], goal[1] - last["world_pose"][1])

    motion_transitions = [t for t in transitions if (t.get("session_id"), t.get("rl_step")) in movement_keys]
    terminal_collision_snapshot = ((data.terminal or {}).get("collision_coverage") or {}).get("last_snapshot")
    safety = motion_safety(frames, movement_start, movement_end, terminal_collision_snapshot)
    declared_terminal = context.get("result_contract", {}).get("classification", "UNKNOWN")
    terminal_safety_conflict = (
        (declared_terminal == "COLLISION_TERMINATION" and safety["collision"] is not True) or
        (declared_terminal == "FALL_TERMINATION" and safety["fall"] is not True) or
        (declared_terminal in {"ARRIVED", "DIAGNOSTIC_SIM_TIME_LIMIT"} and
         (safety["collision"] is True or safety["fall"] is True))
    )
    agile_recovery = sum(1 for t in motion_transitions if t["from"] == 0 and t["to"] == 1)
    recovery_agile = sum(1 for t in motion_transitions if t["from"] == 1 and t["to"] == 0)
    reverse_pairs = 0
    for left, right in zip(motion_transitions, motion_transitions[1:]):
        if (left["from"], left["to"]) == (0, 1) and (right["from"], right["to"]) == (1, 0):
            if left["sim_time_s"] is not None and right["sim_time_s"] is not None and 0 <= right["sim_time_s"] - left["sim_time_s"] <= reverse_window_s:
                reverse_pairs += 1
        elif (left["from"], left["to"]) == (1, 0) and (right["from"], right["to"]) == (0, 1):
            if left["sim_time_s"] is not None and right["sim_time_s"] is not None and 0 <= right["sim_time_s"] - left["sim_time_s"] <= reverse_window_s:
                reverse_pairs += 1

    intervals: list[dict[str, Any]] = []
    if movement_frames:
        start = movement_frames[0]
        state = start.get("mode_after")
        prior = start
        for item in movement_frames[1:]:
            continuous = (item["session_id"] == prior["session_id"] and item["rl_step"] == prior["rl_step"] + 1
                          and item.get("sim_clock_valid") is True and prior.get("sim_clock_valid") is True
                          and isinstance(item.get("sim_time_s"), (int, float)) and isinstance(prior.get("sim_time_s"), (int, float))
                          and item["sim_time_s"] >= prior["sim_time_s"])
            if not continuous:
                if state == 1:
                    intervals.append({"mode": "RECOVERY", "start_sim_time_s": start.get("sim_time_s") if start.get("sim_clock_valid") else None,
                                      "end_sim_time_s": None, "duration_s": None, "censored": True,
                                      "reason": "cycle gap, invalid clock, session change, or time regression"})
                start, state = item, item.get("mode_after")
            elif item.get("mode_after") != state:
                if state == 1:
                    start_t = start.get("sim_time_s") if start.get("sim_clock_valid") else None
                    # The first Agile decision cycle is the end boundary, not the
                    # last preceding Recovery sample.
                    end_t = item.get("sim_time_s") if item.get("sim_clock_valid") else None
                    intervals.append({"mode": "RECOVERY", "start_sim_time_s": start_t,
                                      "end_sim_time_s": end_t,
                                      "duration_s": end_t - start_t if start_t is not None and end_t is not None and end_t >= start_t else None,
                                      "censored": False, "exit_cycle": item.get("rl_step")})
                start, state = item, item.get("mode_after")
            prior = item
        if state == 1:
            intervals.append({"mode": "RECOVERY", "start_sim_time_s": start.get("sim_time_s") if start.get("sim_clock_valid") else None,
                              "end_sim_time_s": None, "duration_s": None, "censored": True})

    if (data.terminal or {}).get("termination_reason") == "SAFETY_FAULT" and context.get("result_contract", {}).get("classification") not in {"SYSTEM_SAFETY_ABORT", "STARTUP_FAILURE"}:
        final_result = "INVALID_TERMINAL_CONFLICT"
    elif (data.terminal or {}).get("run_terminal_result") not in (None, context.get("result_contract", {}).get("classification")):
        final_result = "INVALID_TERMINAL_CONFLICT"
    elif terminal_safety_conflict:
        final_result = "INVALID_TERMINAL_SAFETY_CONFLICT"
    elif terminal_frame_failure:
        final_result = "INCOMPLETE_TERMINAL_FRAME_CAPTURE"
    else:
        final_result = context.get("result_contract", {}).get("classification", "UNKNOWN")
    validity_reasons = list(base.get("record_validity_reasons") or [])
    if terminal_frame_failure and terminal_frame_reason not in validity_reasons:
        validity_reasons.append(terminal_frame_reason)
    return {
        "run_id": context.get("run_id"),
        "scene": context_scene_name,
        "scene_id": settings.get("scene_id", "UNKNOWN"),
        "record_validity": "INVALID" if terminal_frame_failure else base.get("record_validity"),
        "record_validity_reasons": validity_reasons,
        "terminal_result": final_result,
        "navigation_terminal_result": final_result,
        "run_terminal_result_context": context.get("result_contract", {}).get("classification", "UNKNOWN"),
        "run_terminal_result_record": (data.terminal or {}).get("run_terminal_result"),
        "cleanup_events": (data.terminal or {}).get("cleanup_events", []),
        "motion_metric_time_boundary": {"start_sim_time_s": movement_start, "end_sim_time_s": movement_end,
                                         "start_cycle": (movement_frames[0].get("rl_step") if movement_frames else None),
                                         "end_cycle": terminal_ref.get("rl_step") if isinstance(terminal_ref, dict) else None,
                                         "terminal_frame_payload_complete": terminal_frame_payload_complete},
        "session_id": context.get("session_id"), "frame_count": len(frames),
        "rl_step_gaps": gaps, "invalid_clock_frames": invalid_clock_frames,
        "clock_valid_fraction": ((len(frames) - invalid_clock_frames) / len(frames)) if frames else None,
        "first_valid_sim_time_s": min(valid_times) if valid_times else None,
        "last_valid_sim_time_s": max(valid_times) if valid_times else None,
        "policy_interval_ms_median": statistics.median(periods_s) * 1000 if periods_s else None,
        "policy_interval_ms_p95": quantile(periods_s, 0.95) * 1000 if periods_s else None,
        "policy_interval_ms_min": min(periods_s) * 1000 if periods_s else None,
        "policy_interval_ms_max": max(periods_s) * 1000 if periods_s else None,
        "policy_frequency_hz_overall": ((len(periods_s) / sum(periods_s)) if periods_s and sum(periods_s) > 0 else None),
        "mode_transitions": len(motion_transitions) if movement_frames else None,
        "agile_to_recovery": agile_recovery if movement_frames else None,
        "recovery_to_agile": recovery_agile if movement_frames else None,
        "transitions_with_valid_sim_time": mode_changes_with_valid_time if movement_frames else None,
        "transitions": motion_transitions, "recovery_intervals": intervals,
        "recovery_sim_duration_s": recovery_duration if valid_motion_duration else None,
        "valid_motion_duration_s": valid_motion_duration if valid_motion_duration else None,
        "recovery_share_valid_motion_time": recovery_duration / valid_motion_duration if valid_motion_duration else None,
        "movement_start_sim_time_s": movement_start,
        "target_distance_start_m": goal_distance_start, "target_distance_final_m": goal_distance_final,
        "path_length_m": trajectory_length if valid_motion_duration else None,
        "mean_path_speed_mps": trajectory_length / valid_motion_duration if valid_motion_duration else None,
        "short_reverse_switch_count": reverse_pairs if movement_frames else None,
        "short_reverse_window_s": reverse_window_s,
        "period_statistics_scope": "all captured LIVE frames including preparation and cleanup tail, if present",
        "motion_metrics_scope": "valid continuous cycles from observed 0.10m motion-start cycle through terminal frame; cleanup tail excluded",
        "collision_coverage": base.get("collision"),
        "motion_safety": safety,
        "nonfoot_collision_failure": safety["nonfoot_collision_failure"],
        "foot_contact_count": safety["foot_contact_count"],
        "foot_contact_duration_s": safety["foot_contact_duration_s"],
        "foot_contact_duration_capture_s": safety["foot_contact_duration_capture_s"],
        "foot_impact_count": safety["foot_impact_count"],
        "foot_impact_duration_s": safety["foot_impact_duration_s"],
        "foot_impact_duration_capture_s": safety["foot_impact_duration_capture_s"],
        "arrived_with_foot_contact": (
            (safety["foot_contact_count"] > 0 if isinstance(safety["foot_contact_count"], int)
             else "UNKNOWN") if final_result == "ARRIVED" else False),
        "arrived_with_foot_impact": (
            (safety["foot_impact_count"] > 0 if isinstance(safety["foot_impact_count"], int)
             else "UNKNOWN") if final_result == "ARRIVED" else False),
        "abs_reference_collision": safety["abs_reference_collision"],
        "external_safety_events": (data.terminal or {}).get("external_safety_events", []),
        "command_consumption": "UNKNOWN",
    }


def make_timeline(record_path: Path, context_path: Path, output_path: Path,
                  task_label: str = "S1-04") -> None:
    import os
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/s1-04-matplotlib")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    data = load_record(str(record_path))
    context = json.loads(context_path.read_text(encoding="utf-8"))
    base = summarize_record(str(record_path))
    rows = [line["payload"] for line in data.frames if line.get("status") == "LIVE" and isinstance(line.get("payload"), dict)]
    rows.sort(key=lambda p: (p["session_id"], p["rl_step"]))
    fig, ax = plt.subplots(figsize=(11, 5.5), constrained_layout=True)
    lifecycle = context.get("lifecycle", {})
    terminal_ref = lifecycle.get("terminal_frame_ref")
    motion_start = lifecycle.get("motion_start_observed_sim_time_s")
    motion_end = lifecycle.get("movement_end_sim_time_s")
    if motion_end is None and isinstance(terminal_ref, dict):
        motion_end = terminal_ref.get("sim_time_s")
    valid = [p for p in rows if p.get("sim_clock_valid") is True and isinstance(p.get("sim_time_s"), (int, float))
             and motion_start is not None and motion_end is not None
             and motion_start <= p["sim_time_s"] <= motion_end
             and (not isinstance(terminal_ref, dict) or
                  (p.get("session_id"), p.get("rl_step")) <=
                  (terminal_ref.get("session_id", p.get("session_id")), terminal_ref.get("rl_step", p.get("rl_step"))))]
    if base.get("record_validity") != "VALID":
        ax.text(0.5, 0.57, "Timeline not computed", ha="center", va="center", fontsize=14)
        ax.text(0.5, 0.43, "Run record is INVALID; see metrics.json for the specific validation reason.",
                ha="center", va="center", wrap=True)
        ax.set_axis_off()
    elif valid:
        times = [p["sim_time_s"] for p in valid]
        ras = [p["ra_value"] for p in valid]
        ax.plot(times, ras, color="#2455a4", lw=1.0, label="RA")
        ax.plot(times, [p["entry_threshold"] for p in valid], color="#a54535", ls="--", lw=0.9, label="entry threshold")
        ax.plot(times, [p["exit_threshold"] for p in valid], color="#d18b28", ls=":", lw=1.0, label="exit threshold")
        ax.set_xlabel("MuJoCo simulation time (s)")
        ax.set_ylabel("RA (unitless)")
        recovery_runs: list[tuple[float, float]] = []
        begin = None
        prev = None
        for p in valid:
            if p["mode_after"] == 1 and begin is None:
                begin = p["sim_time_s"]
            if begin is not None and prev is not None and p["mode_after"] != 1:
                recovery_runs.append((begin, prev))
                begin = None
            prev = p["sim_time_s"]
        if begin is not None and prev is not None:
            recovery_runs.append((begin, prev))
        for left, right in recovery_runs:
            ax.axvspan(left, right, color="#db8e24", alpha=0.17)
        goal = context.get("settings", {}).get("goal_world_m", [None, None])
        if goal and goal[0] is not None:
            dists = [math.hypot(goal[0] - p["world_pose"][0], goal[1] - p["world_pose"][1]) for p in valid]
            ax2 = ax.twinx()
            ax2.plot(times, dists, color="#27804b", lw=0.9, alpha=0.85, label="goal distance")
            ax2.set_ylabel("Goal distance (m)")
            ax2.legend(loc="upper right")
        result = context.get("result_contract", {}).get("classification", "UNKNOWN")
        ax.axvline(motion_end, color="#222222", lw=1.2, ls="-.", label=f"motion terminal: {result}")
        seen_markers: set[tuple] = set()
        for payload in valid:
            collision = payload.get("collision_snapshot")
            if not isinstance(collision, dict) or collision.get("version") != 5:
                continue
            for key, color, marker, label, level in (
                    ("foot_contact_history", "#228833", "o", "foot contact", 0.05),
                    ("foot_impact_history", "#cc6677", "X", "foot impact", 0.12),
                    ("collision_history", "#aa3377", "s", "non-foot collision", 0.19)):
                for event in collision.get(key, []):
                    time_s = event.get("start_sim_time")
                    if not isinstance(time_s, (int, float)) or not motion_start <= time_s <= motion_end:
                        continue
                    identity = (key, event.get("start_physics_step"), event.get("foot_geom_id"),
                                event.get("robot_geom_id"), event.get("obstacle_geom_id"))
                    if identity in seen_markers:
                        continue
                    seen_markers.add(identity)
                    y_span = max(ras) - min(ras) if ras else 1.0
                    y_value = min(ras) + level * max(y_span, 0.1)
                    ax.scatter([time_s], [y_value], color=color, marker=marker, s=34,
                               zorder=5, label=label if label not in ax.get_legend_handles_labels()[1] else None)
        ax.legend(loc="upper left")
        scene = context.get("settings", {})
        scene_label = scene.get("scene_filename") or scene.get("scene_id") or "scene UNKNOWN"
        ax.set_title(f"{task_label} | {scene_label} | run {context.get('run_id')} (valid sim-time samples only)")
        ax.grid(alpha=0.2)
        ax.text(0.01, 0.02, f"invalid clock frames in full capture: {sum(p.get('sim_clock_valid') is not True for p in rows)}; mode band = Recovery",
                transform=ax.transAxes, fontsize=8)
    else:
        ax.text(0.5, 0.5, "No valid sim-time samples; timeline unavailable", ha="center", va="center")
        ax.set_axis_off()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--reverse-window-s", type=float, default=0.5)
    parser.add_argument("--task-label", default="S1-04",
                        help="label used in each timeline title (default: S1-04)")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for run_dir in sorted(p for p in args.runs_root.iterdir() if p.is_dir()):
        record = run_dir / "runtime_record.jsonl"
        context = run_dir / "run_context.json"
        if record.exists() and context.exists():
            summary = calculate(record, context, args.reverse_window_s)
            run_context = json.loads(context.read_text(encoding="utf-8"))
            settings = run_context.get("settings", {})
            summary["scene"] = settings.get("scene_filename") or Path(settings.get("scene", "")).name or "UNKNOWN"
            summary["scene_id"] = settings.get("scene_id", "UNKNOWN")
            (run_dir / "metrics.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            make_timeline(record, context, run_dir / "timeline.png", args.task_label)
            rows.append(summary)
    (args.output_dir / "run_results.json").write_text(json.dumps(rows, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    cols = ["run_id", "scene", "scene_id", "record_validity", "navigation_terminal_result",
            "nonfoot_collision_failure", "foot_contact_count", "foot_contact_duration_s",
            "foot_impact_count", "foot_impact_duration_s", "arrived_with_foot_contact",
            "arrived_with_foot_impact", "abs_reference_collision", "frame_count", "rl_step_gaps",
            "invalid_clock_frames", "policy_frequency_hz_overall", "mode_transitions",
            "recovery_share_valid_motion_time", "target_distance_final_m", "mean_path_speed_mps"]
    lines = ["| " + " | ".join(cols) + " |", "|" + "|".join(["---"] * len(cols)) + "|"]
    for row in rows:
        vals = [row.get(c) for c in cols]
        lines.append("| " + " | ".join("UNKNOWN" if v is None else (f"{v:.4f}" if isinstance(v, float) else str(v)) for v in vals) + " |")
    (args.output_dir / "run_results.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"runs={len(rows)} table={args.output_dir / 'run_results.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
