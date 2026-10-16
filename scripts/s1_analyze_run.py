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


def summarize_policy_cadence(frames: list[dict[str, Any]]) -> dict[str, Any]:
    """Estimate policy cadence from cycle identity, not reader poll count.

    Adjacent-frame period quantiles include only persisted consecutive policy
    cycles. Overall frequency counts cycle IDs across the observed monotonic
    span, so a skipped reader sample is not treated as if only one policy cycle
    elapsed. Missing cycles remain explicit and never become fabricated frames.
    """
    adjacent_periods_s: list[float] = []
    observed_cycle_span = 0
    observed_wall_span_s = 0.0
    missing_cycles = 0
    for left, right in zip(frames, frames[1:]):
        if left.get("session_id") != right.get("session_id"):
            continue
        left_step, right_step = left.get("rl_step"), right.get("rl_step")
        if not isinstance(left_step, int) or not isinstance(right_step, int):
            continue
        delta_step = right_step - left_step
        dt_wall = (right.get("monotonic_ns", 0) - left.get("monotonic_ns", 0)) / 1e9
        if delta_step <= 0 or dt_wall <= 0:
            continue
        observed_cycle_span += delta_step
        observed_wall_span_s += dt_wall
        missing_cycles += max(0, delta_step - 1)
        if delta_step == 1:
            adjacent_periods_s.append(dt_wall)
    return {
        "policy_frequency_hz_overall": (
            observed_cycle_span / observed_wall_span_s if observed_wall_span_s > 0 else None),
        "observed_policy_cycle_span": observed_cycle_span,
        "observed_wall_span_s": observed_wall_span_s if observed_wall_span_s > 0 else None,
        "missing_policy_cycles_between_saved_frames": missing_cycles,
        "adjacent_consecutive_cycle_period_s": adjacent_periods_s,
        "period_quantile_scope": "saved adjacent frames with rl_step delta exactly 1",
        "frequency_scope": "policy cycle-ID span divided by monotonic span between saved frames",
        "coverage": "complete" if frames and missing_cycles == 0 else "partial_or_unknown",
    }


def mode_edge_coverage(policy_frames: list[dict[str, Any]], identity_complete: bool) -> str:
    """A saved-edge count is complete only if the active cycle sequence is contiguous."""
    contiguous = bool(policy_frames) and all(
        left.get("session_id") == right.get("session_id") and
        right.get("rl_step") == left.get("rl_step", -2) + 1
        for left, right in zip(policy_frames, policy_frames[1:]))
    return "complete" if identity_complete and contiguous else "partial_or_unknown"


def audit_hold_episodes(motion_frames: list[dict]) -> dict[str, Any]:
    """Audit C/D hold against saved decision-cycle frames without interpolation."""
    episodes: list[dict[str, Any]] = []
    violations: list[dict[str, Any]] = []
    for index, entry in enumerate(motion_frames):
        if (entry.get("switching_mode") not in (3, 4) or entry.get("mode_before") != 0 or
                entry.get("mode_after") != 1):
            continue
        hold_s = entry.get("recovery_min_hold_s")
        t_enter = entry.get("hold_enter_sim_time_s")
        episode = {"session_id": entry.get("session_id"), "enter_cycle": entry.get("rl_step"),
                   "H_s": hold_s, "t_enter_sim_s": t_enter, "exit_cycle": None,
                   "observed_exit_elapsed_s": None, "censored": True,
                   "clock_coverage": "complete", "hold_rule": "UNKNOWN"}
        if (entry.get("hold_enabled") is not True or not isinstance(hold_s, (int, float)) or hold_s <= 0 or
                not isinstance(t_enter, (int, float)) or entry.get("sim_clock_valid") is not True or
                entry.get("hold_status") != 2):
            episode["clock_coverage"] = "incomplete"
            violations.append({"cycle": entry.get("rl_step"), "reason": "invalid_or_missing_hold_entry_payload"})
        entry_segment = entry.get("sim_clock_segment_id")
        prior = entry
        for current in motion_frames[index + 1:]:
            if current.get("session_id") != entry.get("session_id"):
                episode["clock_coverage"] = "incomplete"
                break
            if current.get("rl_step", 0) <= prior.get("rl_step", 0):
                continue
            if current.get("rl_step") != prior.get("rl_step", 0) + 1:
                episode["clock_coverage"] = "incomplete"
            if current.get("switching_mode") != entry.get("switching_mode"):
                episode["clock_coverage"] = "incomplete"
            if current.get("sim_clock_valid") is not True or current.get("sim_clock_segment_id") != entry_segment:
                episode["clock_coverage"] = "incomplete"
            else:
                now_s = current.get("sim_time_s")
                elapsed_reported = current.get("hold_elapsed_sim_s")
                elapsed = now_s - t_enter if isinstance(now_s, (int, float)) and isinstance(t_enter, (int, float)) else None
                hold_expired = current.get("hold_status") == 3
                if elapsed is None or elapsed < 0 or (elapsed_reported is not None and abs(elapsed_reported - elapsed) > 1e-5):
                    episode["clock_coverage"] = "incomplete"
                if elapsed is not None and hold_expired != (elapsed >= hold_s):
                    episode["clock_coverage"] = "incomplete"
                    violations.append({"cycle": current.get("rl_step"),
                                       "reason": "hold_status_elapsed_mismatch"})
                if current.get("mode_before") == 1 and current.get("mode_after") == 0:
                    episode["exit_cycle"] = current.get("rl_step")
                    episode["observed_exit_elapsed_s"] = elapsed
                    episode["censored"] = False
                    exit_threshold = (current.get("exit_threshold", math.inf)
                                      if entry.get("switching_mode") == 4
                                      else current.get("entry_threshold", math.inf))
                    # The producer classifies expiry with the same double
                    # subtraction recorded in hold_elapsed_sim_s. Trust that
                    # decision-cycle status here; an analyzer epsilon can
                    # otherwise call a still-active 0.2999999999998 s hold
                    # expired before the controller does.
                    allowed = (elapsed is not None and hold_expired and
                               isinstance(current.get("ra_value"), (int, float)) and
                               current["ra_value"] < exit_threshold and
                               current.get("hold_status") == 3)
                    episode["hold_rule"] = "PASS" if allowed else "FAIL"
                    if not allowed:
                        exit_name = "X" if entry.get("switching_mode") == 4 else "E"
                        violations.append({"cycle": current.get("rl_step"),
                                           "reason": f"exit_before_H_or_RA_not_below_{exit_name}"})
                    break
                if current.get("mode_after") == 1 and elapsed is not None and not hold_expired:
                    if current.get("hold_status") != 2:
                        violations.append({"cycle": current.get("rl_step"), "reason": "hold_not_active_before_expiry"})
                        episode["hold_rule"] = "FAIL"
                if (entry.get("switching_mode") == 4 and elapsed is not None and hold_expired and
                        current.get("ra_value") < current.get("exit_threshold", -math.inf)
                        and current.get("mode_after") == 1):
                    violations.append({"cycle": current.get("rl_step"),
                                       "reason": "D_did_not_exit_when_H_expired_and_RA_below_X"})
                    episode["hold_rule"] = "FAIL"
                if current.get("mode_after") == 0:
                    break
            prior = current
        episodes.append(episode)
    if not episodes:
        status = "not_observed"
    elif violations:
        status = "fail"
    elif any(item["censored"] or item["clock_coverage"] != "complete" for item in episodes):
        status = "incomplete"
    else:
        status = "pass"
    return {"status": status, "episodes": episodes, "violations": violations,
            "scope": "motion window decision cycles only; no interpolation; terminal truncation is censored"}


def motion_safety(frames: list[dict], start_s: float | None,
                  end_s: float | None,
                  terminal_snapshot: dict | None = None,
                  boundary_snapshots: list[dict] | None = None,
                  start_boundary_snapshot: dict | None = None,
                  end_boundary_snapshot: dict | None = None) -> dict[str, Any]:
    """Classify safety events using valid sim-time or an identity-bound physics-step window.

    A policy frame can have an invalid sim clock while the independently sampled,
    same-run collision snapshot remains LIVE. Such snapshots may establish a
    positive event's task-window membership by physics-step identity. They do not
    make elapsed-time coverage complete and cannot support a no-event conclusion.
    Callers must pass only frames within the safety interval; adjacent persisted
    boundary snapshots may be supplied separately. Post-terminal capture frames
    belong to the full-capture summary, not to a completed task-window assessment.
    """
    snapshots = [p.get("collision_snapshot") for p in frames]
    snapshots.extend(boundary_snapshots or [])
    if isinstance(terminal_snapshot, dict):
        snapshots.append(terminal_snapshot)
    capture_ids = {item.get("capture_id") for item in snapshots if isinstance(item, dict)}
    capture_ids.discard(None)
    expected_capture_id = next(iter(capture_ids)) if len(capture_ids) == 1 else None

    def usable_boundary(item: dict | None) -> dict | None:
        if (not isinstance(item, dict) or item.get("status") != CollisionStatus.LIVE.value or
                item.get("physics_coverage_complete") is not True or
                not isinstance(item.get("physics_step"), int) or
                not isinstance(item.get("sim_time"), (int, float)) or
                not math.isfinite(float(item.get("sim_time"))) or
                item.get("event_latches_complete") is not True or
                item.get("collision_history_overflow") is not False or
                item.get("fall_history_overflow") is not False or
                expected_capture_id is None or item.get("capture_id") != expected_capture_id):
            return None
        return item

    start_physics = usable_boundary(start_boundary_snapshot)
    end_physics = usable_boundary(end_boundary_snapshot)
    start_step = start_physics.get("physics_step") if start_physics else None
    end_step = end_physics.get("physics_step") if end_physics else None
    step_window_valid = (isinstance(start_step, int) and isinstance(end_step, int) and
                         end_step >= start_step)
    coverage_failures: list[str] = []
    if not frames or start_s is None or end_s is None or end_s < start_s:
        coverage_failures.append("task_window_frames_or_time_bounds_missing")
    if not snapshots:
        coverage_failures.append("task_window_physical_snapshots_missing")
    if expected_capture_id is None:
        coverage_failures.append("task_window_capture_identity_missing_or_conflicting")
    elif any(not isinstance(item, dict) or item.get("capture_id") != expected_capture_id
             for item in snapshots):
        coverage_failures.append("task_window_capture_identity_conflict")
    if any(not isinstance(item, dict) or
           item.get("status") != CollisionStatus.LIVE.value for item in snapshots):
        coverage_failures.append("task_window_contains_non_live_physical_snapshot")
    if any(not isinstance(item, dict) or item.get("version") not in (4, 5)
           for item in snapshots):
        coverage_failures.append("task_window_snapshot_version_missing_or_unsupported")
    if any(not isinstance(item, dict) or item.get("physics_coverage_complete") is not True
           for item in snapshots):
        coverage_failures.append("task_window_physics_publisher_coverage_incomplete")
    if any(not isinstance(item, dict) or item.get("collision_history_overflow") is not False
           for item in snapshots):
        coverage_failures.append("task_window_collision_history_overflow_or_unknown")
    if any(not isinstance(item, dict) or item.get("fall_history_overflow") is not False
           for item in snapshots):
        coverage_failures.append("task_window_fall_history_overflow_or_unknown")
    if any(not isinstance(item, dict) or item.get("event_latches_complete") is not True
           for item in snapshots):
        coverage_failures.append("task_window_event_latches_incomplete")
    source_complete = not coverage_failures
    snapshot_times = [item.get("sim_time") for item in snapshots
                      if isinstance(item, dict) and isinstance(item.get("sim_time"), (int, float))]
    if len(snapshot_times) != len(snapshots):
        coverage_failures.append("task_window_snapshot_time_missing")
    if start_s is not None and min(snapshot_times, default=math.inf) > start_s:
        coverage_failures.append("task_window_start_not_bracketed_by_physical_snapshot")
    if end_s is not None and max(snapshot_times, default=-math.inf) < end_s:
        coverage_failures.append("task_window_terminal_not_bracketed_by_physical_snapshot")
    source_complete = not coverage_failures
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
    if start_s is not None and end_s is not None:
        motion_events = [event for event in history.values()
                         if event.get("end_sim_time", -math.inf) >= start_s and
                         event.get("start_sim_time", math.inf) <= end_s]
        event_window_basis = "valid_sim_time"
    elif step_window_valid:
        motion_events = [event for event in history.values()
                         if isinstance(event.get("start_physics_step"), int) and
                         isinstance(event.get("end_physics_step"), int) and
                         event.get("end_physics_step") >= start_step and
                         event.get("start_physics_step") <= end_step and
                         (end_s is None or event.get("start_sim_time", math.inf) <= end_s)]
        event_window_basis = "identity_bound_physics_steps; elapsed_time_and_negative_claim_remain_unknown"
    else:
        motion_events = []
        event_window_basis = "unavailable"
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
    # A cumulative fall event can change lifecycle state without advancing
    # end_physics_step (notably open -> closed after posture recovers). Merge
    # by event identity and the publishing snapshot's physical order, scoped
    # to what was observable by the task's terminal boundary. Do not use a
    # later cleanup snapshot to rewrite the event state at task end.
    fall_history_updates: dict[int, tuple[dict, tuple, dict]] = {}
    fall_snapshot_identity_valid = bool(snapshots) and expected_capture_id is not None
    by_sequence: dict[int, dict] = {}
    for item in snapshots:
        if not isinstance(item, dict):
            fall_snapshot_identity_valid = False
            continue
        if (item.get("status") != CollisionStatus.LIVE.value or
                item.get("capture_id") != expected_capture_id):
            fall_snapshot_identity_valid = False
            continue
        sequence = item.get("sequence")
        if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence <= 0:
            fall_snapshot_identity_valid = False
            continue
        previous_same_sequence = by_sequence.get(sequence)
        if previous_same_sequence is not None:
            if previous_same_sequence != item:
                fall_snapshot_identity_valid = False
            continue
        by_sequence[sequence] = item
    unique_physical_snapshots = [by_sequence[key] for key in sorted(by_sequence)]
    for previous, current in zip(unique_physical_snapshots, unique_physical_snapshots[1:]):
        if (current.get("sequence", -1) <= previous.get("sequence", -1) or
                current.get("physics_step", -1) <= previous.get("physics_step", -1) or
                not isinstance(current.get("sim_time"), (int, float)) or
                not isinstance(previous.get("sim_time"), (int, float)) or
                current.get("sim_time", -math.inf) <= previous.get("sim_time", -math.inf)):
            fall_snapshot_identity_valid = False

    ordered_snapshots = []
    for index, item in enumerate(unique_physical_snapshots):
        sample_time = item.get("sim_time")
        sample_step = item.get("physics_step")
        # Keep a same-run terminal-boundary snapshot even when it was published
        # just after the terminal policy frame. Its publication time is not the
        # event time: an explicit confirmed_sim_time/physics_step may establish
        # an in-window event. The evaluator below clips lifecycle facts to the
        # task boundary instead of discarding the whole snapshot.
        if not isinstance(sample_time, (int, float)) or not math.isfinite(float(sample_time)):
            continue
        order = (float(sample_time) if isinstance(sample_time, (int, float)) else -math.inf,
                 item.get("sequence", -1) if isinstance(item.get("sequence"), int) else -1,
                 sample_step if isinstance(sample_step, int) else -1, index)
        ordered_snapshots.append((order, item))
    for order, item in sorted(ordered_snapshots, key=lambda pair: pair[0]):
        for event in item.get("fall_history", []):
            if not isinstance(event, dict):
                continue
            key = event.get("start_physics_step")
            if not isinstance(key, int):
                continue
            previous = fall_history_updates.get(key)
            if previous is not None and previous[0].get("confirmed") is True:
                # Confirmation is a latched historical fact. Later state
                # cannot clear it, even if a malformed/old snapshot says so.
                merged = dict(event)
                merged["confirmed"] = True
                for field in ("confirmed_physics_step", "confirmed_sim_time"):
                    if previous[0].get(field) not in (None, 0, 0.0):
                        merged[field] = previous[0][field]
                event = merged
            if previous is None or order > previous[1]:
                fall_history_updates[key] = (dict(event), order, item)
    fall_history = {key: value[0] for key, value in fall_history_updates.items()}
    # Reconstruct event status as of the task boundary. A later snapshot may
    # reveal an earlier confirmation (positive evidence), but a later `closed`
    # bit has no closure timestamp and cannot prove that the event closed by
    # the task boundary. Use a close only when a closed version was already
    # published no later than that boundary.
    motion_falls: list[dict] = []
    confirmed_motion_falls: list[dict] = []
    active_motion_falls: list[dict] = []
    closed_unconfirmed_motion_falls: list[dict] = []
    fall_event_timing_conflicts: list[dict] = []
    for key, (latest_event, _latest_order, _latest_snapshot) in fall_history_updates.items():
        versions = [(order, item, event)
                    for order, item in ordered_snapshots
                    for event in item.get("fall_history", [])
                    if isinstance(event, dict) and event.get("start_physics_step") == key]
        if not versions:
            continue
        start_step_value = latest_event.get("start_physics_step")
        start_time_value = latest_event.get("start_sim_time")
        if (not isinstance(start_step_value, int) or isinstance(start_step_value, bool) or
                not isinstance(start_time_value, (int, float)) or
                not math.isfinite(float(start_time_value))):
            fall_event_timing_conflicts.append({"start_physics_step": key, "reason": "invalid_event_start"})
            continue
        event_starts = {(event.get("start_physics_step"), event.get("start_sim_time"))
                        for _o, _i, event in versions}
        if len(event_starts) != 1:
            fall_event_timing_conflicts.append({"start_physics_step": key,
                "reason": "event_start_identity_changed", "values": sorted(map(str, event_starts))})
            continue
        # The event must intersect the task window. `end_sim_time` is the last
        # abnormal sample, not a close timestamp; a still-open candidate whose
        # last abnormal sample precedes task start is nevertheless active at
        # task start unless a close was observed before that boundary.
        before_start_closed = any(
            event.get("closed") is True and start_s is not None and end_s is not None and
            isinstance(event.get("end_sim_time"), (int, float)) and
            event.get("end_sim_time") < start_s and
            isinstance(item.get("sim_time"), (int, float)) and item.get("sim_time") <= start_s and
            (not step_window_valid or item.get("physics_step", math.inf) <= start_step)
            for _order, item, event in versions)
        if before_start_closed:
            continue
        intersects = (start_s is not None and end_s is not None and
                      start_time_value <= end_s and
                      (latest_event.get("end_sim_time", -math.inf) >= start_s or
                       any(event.get("closed") is not True for _o, _i, event in versions)))
        if not intersects and step_window_valid:
            intersects = (isinstance(latest_event.get("end_physics_step"), int) and
                          latest_event.get("start_physics_step") <= end_step and
                          (latest_event.get("end_physics_step") >= start_step or
                           any(event.get("closed") is not True for _o, _i, event in versions)))
        if not intersects:
            continue
        event_for_window = dict(latest_event)
        event_for_window["status_at_task_terminal"] = "UNKNOWN"
        confirmation_versions = [event for _o, _i, event in versions if event.get("confirmed") is True]
        if confirmation_versions:
            confirmations = {(event.get("confirmed_physics_step"), event.get("confirmed_sim_time"))
                             for event in confirmation_versions}
            if len(confirmations) != 1:
                fall_event_timing_conflicts.append({"start_physics_step": key,
                    "reason": "confirmation_identity_changed", "values": sorted(map(str, confirmations))})
            else:
                confirmed_step, confirmed_time = next(iter(confirmations))
                time_valid = (isinstance(confirmed_time, (int, float)) and
                              math.isfinite(float(confirmed_time)) and end_s is not None and
                              start_time_value <= confirmed_time <= end_s)
                step_valid = (isinstance(confirmed_step, int) and not isinstance(confirmed_step, bool) and
                              confirmed_step >= start_step_value and
                              (not step_window_valid or confirmed_step <= end_step))
                time_after = (isinstance(confirmed_time, (int, float)) and end_s is not None and confirmed_time > end_s)
                step_after = (isinstance(confirmed_step, int) and step_window_valid and confirmed_step > end_step)
                confirmation_event_end = latest_event.get("end_physics_step")
                confirmation_after_event_end = (isinstance(confirmation_event_end, int) and
                                                isinstance(confirmed_step, int) and
                                                confirmed_step > confirmation_event_end)
                if step_window_valid and time_after != step_after:
                    fall_event_timing_conflicts.append({"start_physics_step": key,
                        "reason": "confirmation_time_step_terminal_boundary_disagrees",
                        "confirmed_physics_step": confirmed_step, "confirmed_sim_time": confirmed_time})
                elif confirmation_after_event_end:
                    fall_event_timing_conflicts.append({"start_physics_step": key,
                        "reason": "confirmation_after_event_physics_end",
                        "confirmed_physics_step": confirmed_step, "event_end_physics_step": confirmation_event_end})
                elif time_valid and step_valid:
                    event_for_window["confirmed"] = True
                    event_for_window["closed"] = False
                    event_for_window["status_at_task_terminal"] = "CONFIRMED_BY_TASK_TERMINAL"
                    event_for_window["confirmation_within_task_window"] = (
                        start_s is not None and confirmed_time >= start_s)
                    confirmed_motion_falls.append(event_for_window)
                elif time_after and (not step_window_valid or step_after):
                    event_for_window["confirmed"] = False
                    event_for_window["closed"] = False
                    event_for_window["status_at_task_terminal"] = "CONFIRMATION_AFTER_TASK_TERMINAL"
                    active_motion_falls.append(event_for_window)
                else:
                    fall_event_timing_conflicts.append({"start_physics_step": key,
                        "reason": "confirmation_time_step_outside_or_conflicting",
                        "confirmed_physics_step": confirmed_step, "confirmed_sim_time": confirmed_time})
        else:
            event_for_window["confirmed"] = False
            # Close is task-visible only if that closed version itself was
            # published within the task interval/boundary.
            closed_by_terminal = any(
                event.get("closed") is True and isinstance(item.get("sim_time"), (int, float)) and
                end_s is not None and item.get("sim_time") <= end_s and
                (not step_window_valid or item.get("physics_step", math.inf) <= end_step)
                for _o, item, event in versions)
            if closed_by_terminal:
                event_for_window["closed"] = True
                event_for_window["status_at_task_terminal"] = "CLOSED_UNCONFIRMED_IN_TASK"
                closed_unconfirmed_motion_falls.append(event_for_window)
            else:
                event_for_window["closed"] = False
                event_for_window["status_at_task_terminal"] = "ACTIVE_OR_CLOSE_TIME_UNKNOWN_AT_TERMINAL"
                active_motion_falls.append(event_for_window)
        motion_falls.append(event_for_window)
    fall_lifecycle_assessments = []
    for event in sorted(motion_falls, key=lambda item: item.get("start_sim_time", 0.0)):
        key = event.get("start_physics_step")
        update = fall_history_updates.get(key)
        sample = update[2] if update is not None else {}
        lifecycle_status = event.get("status_at_task_terminal", "UNKNOWN")
        fall_lifecycle_assessments.append({
            "event_start_physics_step": key,
            "status_at_task_terminal": lifecycle_status,
            "confirmed": event.get("confirmed"),
            "closed": event.get("closed"),
            "event_start_sim_time_s": event.get("start_sim_time"),
            "event_last_abnormal_sim_time_s": event.get("end_sim_time"),
            "event_last_abnormal_after_task_terminal": (
                end_s is not None and isinstance(event.get("end_sim_time"), (int, float)) and
                event.get("end_sim_time") > end_s),
            "confirmation_sim_time_s": event.get("confirmed_sim_time"),
            "lifecycle_snapshot_physics_step": sample.get("physics_step"),
            "lifecycle_snapshot_sim_time_s": sample.get("sim_time"),
            "lifecycle_snapshot_sequence": sample.get("sequence"),
        })
    fall_source_complete = source_complete and fall_snapshot_identity_valid
    if fall_event_timing_conflicts:
        fall = "UNKNOWN"
        fall_result_basis = "fall_event_confirmation_time_step_conflict"
    elif not fall_source_complete:
        fall = "UNKNOWN"
        fall_result_basis = "fall_history_source_incomplete_or_identity_conflict"
    elif invalid_posture_overlap:
        fall = "UNKNOWN"
        fall_result_basis = "invalid_posture_overlaps_motion"
    elif confirmed_motion_falls:
        fall = True
        fall_result_basis = "confirmed_fall_event_time_and_physics_step_within_motion_window"
    elif (fall_source_complete and not invalid_posture_overlap and not active_motion_falls):
        fall = False
        fall_result_basis = ("closed_unconfirmed_posture_candidates; no candidate met confirmation rule"
                             if closed_unconfirmed_motion_falls else
                             "complete_fall_history_no_confirmed_or_active_candidate")
    else:
        fall = "UNKNOWN"
        fall_result_basis = (
            "invalid_posture_overlaps_motion" if invalid_posture_overlap else
            "candidate_active_at_terminal_boundary" if active_motion_falls else
            "fall_history_source_incomplete_or_identity_conflict")
    before_motion_times = [item["sim_time"] for item in snapshots if isinstance(item, dict) and
                           isinstance(item.get("sim_time"), (int, float)) and start_s is not None and
                           item["sim_time"] <= start_s]
    through_terminal_times = [item["sim_time"] for item in snapshots if isinstance(item, dict) and
                              isinstance(item.get("sim_time"), (int, float)) and end_s is not None and
                              item["sim_time"] >= end_s]
    coverage_evidence = {
        "sample_count_including_terminal_snapshot": len(snapshots),
        "versions": sorted({item.get("version") for item in snapshots if isinstance(item, dict)}),
        "sim_time_min_s": min(snapshot_times, default=None),
        "sim_time_max_s": max(snapshot_times, default=None),
        "motion_start_s": start_s, "motion_end_s": end_s,
        "event_window_basis": event_window_basis,
        "start_physics_step_boundary": start_step,
        "end_physics_step_boundary": end_step,
        "physics_step_window_identity_bound": step_window_valid,
        "has_snapshot_at_or_before_motion_start": bool(before_motion_times),
        "nearest_snapshot_before_or_at_start_s": max(before_motion_times, default=None),
        "has_snapshot_at_or_after_terminal": bool(through_terminal_times),
        "nearest_snapshot_at_or_after_terminal_s": min(through_terminal_times, default=None),
        "all_physics_publishers_contiguous": bool(snapshots) and all(
            isinstance(item, dict) and item.get("physics_coverage_complete") is True for item in snapshots),
        "collision_history_overflow_free": bool(snapshots) and all(
            isinstance(item, dict) and item.get("collision_history_overflow") is False for item in snapshots),
        "fall_history_overflow_free": bool(snapshots) and all(
            isinstance(item, dict) and item.get("fall_history_overflow") is False for item in snapshots),
        "unclassified_contact_overlaps_motion": unknown_contact_overlap,
        "invalid_posture_overlaps_motion": invalid_posture_overlap,
    }
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
        "fall_result_basis": fall_result_basis,
        "fall_snapshot_identity_valid": fall_snapshot_identity_valid,
        "fall_lifecycle_assessments": fall_lifecycle_assessments,
        "fall_event_timing_conflicts": fall_event_timing_conflicts,
        "fall_history_events_at_terminal": sorted(motion_falls, key=lambda item: item.get("start_sim_time", 0.0)),
        "fall_events": sorted(motion_falls, key=lambda item: item.get("start_sim_time", 0.0)),
        "fall_event": (confirmed_motion_falls[-1] if confirmed_motion_falls else None),
        "coverage_complete_for_motion_interval": bool(source_complete),
        "coverage_reason": None if source_complete else ";".join(dict.fromkeys(coverage_failures)),
        "coverage_evidence": coverage_evidence,
        "unclassified_contact_overlaps_motion": unknown_contact_overlap,
        "invalid_posture_overlaps_motion": invalid_posture_overlap,
        "unconfirmed_fall_candidate_overlaps_motion": bool(active_motion_falls),
        "closed_unconfirmed_fall_candidate_count": len(closed_unconfirmed_motion_falls),
        "active_fall_candidate_at_terminal_count": len(active_motion_falls),
        "max_observed_contacts_per_policy_snapshot": {
            "foot_floor": max((item.get("foot_ground_contacts", 0) for item in snapshots if isinstance(item, dict)), default=None),
            "nonfoot_floor": max((item.get("nonfoot_ground_contacts", 0) for item in snapshots if isinstance(item, dict)), default=None),
            "self": max((item.get("self_contacts", 0) for item in snapshots if isinstance(item, dict)), default=None),
        },
    }


def risk_edge_counts(policy_frames: list[dict[str, Any]]) -> dict[str, int]:
    """Count the tri-state RiskEdge enum without Python truthiness coercion.

    Contract values are 0=false, 1=true, 2=unknown. Unknown is not evidence of
    an event and malformed values are surfaced separately for audit callers.
    """
    counts = {"true": 0, "false": 0, "unknown": 0, "invalid": 0}
    for frame in policy_frames:
        value = frame.get("risk_condition_entered")
        if type(value) is not int or value not in (0, 1, 2):
            counts["invalid"] += 1
        elif value == 1:
            counts["true"] += 1
        elif value == 0:
            counts["false"] += 1
        else:
            counts["unknown"] += 1
    return counts


def physical_base_bounds_window(pre_task: dict | None, terminal: dict | None,
                                task_start_s: float | None, task_end_s: float | None) -> dict:
    """Assess base_link XY against the frozen rectangle using PhysicsLoop latches.

    The cumulative outside-step counter observes between-reader events. A zero
    counter delta over continuous, identity-matched physical coverage supports a
    bounded base-point conclusion. It does not prove the robot's full footprint
    stayed inside the rectangle.
    """
    required = ("producer_session_id", "physics_step", "physics_coverage_complete",
                "bounds_valid", "outside_physics_steps", "xmin", "xmax", "ymin", "ymax",
                "scenario_id", "capture_id", "runtime_model_fingerprint")
    for label, item in (("pre_task", pre_task), ("terminal", terminal)):
        if not isinstance(item, dict) or item.get("status") != "LIVE":
            return {"status": "UNKNOWN", "reason": f"{label}_base_bounds_snapshot_not_live"}
        missing = [key for key in required if key not in item]
        if missing:
            return {"status": "UNKNOWN", "reason": f"{label}_base_bounds_fields_missing",
                    "missing": missing}
        if item.get("bounds_valid") != 1 or item.get("physics_coverage_complete") != 1:
            return {"status": "UNKNOWN", "reason": f"{label}_base_bounds_coverage_invalid"}
    identity = ("producer_session_id", "capture_id", "scenario_id", "runtime_model_fingerprint",
                "xmin", "xmax", "ymin", "ymax")
    if any(pre_task.get(key) != terminal.get(key) for key in identity):
        return {"status": "UNKNOWN", "reason": "base_bounds_source_or_rectangle_changed"}
    if terminal["physics_step"] < pre_task["physics_step"]:
        return {"status": "UNKNOWN", "reason": "base_bounds_physics_step_regressed"}
    delta = terminal["outside_physics_steps"] - pre_task["outside_physics_steps"]
    if delta < 0:
        return {"status": "UNKNOWN", "reason": "base_bounds_outside_counter_regressed"}
    evidence = {"pre_task_physics_step": pre_task["physics_step"],
                "terminal_physics_step": terminal["physics_step"],
                "outside_physics_steps_before_task": pre_task["outside_physics_steps"],
                "outside_physics_steps_at_terminal": terminal["outside_physics_steps"],
                "outside_physics_step_delta": delta,
                "task_interval_sim_s": [task_start_s, task_end_s],
                "declared_bounds_m": {"x": [pre_task["xmin"], pre_task["xmax"]],
                                       "y": [pre_task["ymin"], pre_task["ymax"]]},
                "scope": "base_link XY point at every continuously published PhysicsLoop step; not robot-footprint containment"}
    if delta == 0:
        return {"status": "BASE_LINK_IN_BOUNDS_PHYSICS_STEPS_OBSERVED", **evidence}
    if task_start_s is None or task_end_s is None or task_end_s < task_start_s:
        return {"status": "UNKNOWN", "reason": "valid_physics_time_window_unavailable",
                "outside_physics_step_delta": delta, **evidence}
    last_outside = terminal.get("last_outside_sim_time")
    first_outside = terminal.get("first_outside_sim_time")
    if (isinstance(last_outside, (int, float)) and math.isfinite(float(last_outside)) and
            task_start_s <= float(last_outside) <= task_end_s):
        return {"status": "BASE_LINK_OUT_OF_BOUNDS_PHYSICS_STEP_OBSERVED",
                "first_outside_sim_time_s": first_outside,
                "last_outside_sim_time_s": last_outside, **evidence}
    if (isinstance(last_outside, (int, float)) and math.isfinite(float(last_outside)) and
            float(last_outside) < task_start_s):
        return {"status": "NO_BASE_LINK_OUTSIDE_STEP_AFTER_TASK_START_OBSERVED",
                "last_outside_sim_time_s": last_outside, **evidence}
    return {"status": "UNKNOWN", "reason": "outside_step_delta_not_localized_to_task_window",
            "first_outside_sim_time_s": first_outside,
            "last_outside_sim_time_s": last_outside, **evidence}


def _closeout_only_task_window(data: Any, context: dict, base: dict,
                               live_frames: list[dict[str, Any]]) -> dict[str, Any] | None:
    """Validate a complete task boundary when only final closeout history failed.

    This does not make the full record valid. It permits task-window metrics only
    when the persisted terminal-boundary counters and cycle payloads prove the
    task window independently complete, while the final producer tail is
    explicitly accounted as closeout-only and missing.
    """
    reasons = sorted(set(base.get("record_validity_reasons") or []))
    if reasons != ["policy_cycle_history_incomplete"]:
        return None
    if context.get("lifecycle", {}).get("closeout_errors"):
        return None
    terminal = data.terminal if isinstance(data.terminal, dict) else {}
    history = terminal.get("policy_cycle_history")
    if not isinstance(history, dict) or history.get("accounting_version") != 2:
        return None
    boundary = history.get("task_termination_boundary")
    final = history.get("final_closeout_drain")
    if not isinstance(boundary, dict) or not isinstance(final, dict):
        return None
    if (boundary.get("status") != "complete" or
            boundary.get("terminal_frame_payload_persisted_same_snapshot") is not True or
            boundary.get("missing_active_policy_cycle_count") != 0 or
            boundary.get("missing_active_policy_cycle_ranges") != [] or
            boundary.get("consumer_unread_loss_count_at_task_terminal") != 0):
        return None
    terminal_ref = boundary.get("terminal_frame_ref")
    context_ref = context.get("lifecycle", {}).get("terminal_frame_ref")
    facts_ref = terminal.get("run_terminal_frame_ref")
    if not isinstance(terminal_ref, dict) or not isinstance(context_ref, dict) or not isinstance(facts_ref, dict):
        return None
    identity_keys = ("session_id", "rl_step", "source_sequence", "monotonic_ns")
    if any(not all(ref.get(key) == terminal_ref.get(key) for key in identity_keys)
           for ref in (context_ref, facts_ref)):
        return None
    session_id, end_cycle = terminal_ref.get("session_id"), terminal_ref.get("rl_step")
    if (type(session_id) is not int or type(end_cycle) is not int or end_cycle < 0 or
            boundary.get("first_active_policy_cycle") != 0 or
            boundary.get("last_active_policy_cycle") != end_cycle):
        return None
    task_cursor = boundary.get("consumer_cursor_at_task_terminal")
    task_producer = boundary.get("producer_count_at_task_terminal")
    if (task_cursor != end_cycle + 1 or type(task_producer) is not int or
            task_producer < task_cursor):
        return None
    final_cursor, final_producer = final.get("consumer_cursor"), final.get("producer_count")
    if (final.get("complete") is not False or type(final_cursor) is not int or
            type(final_producer) is not int or final_cursor < task_cursor or
            final_producer <= final_cursor or final.get("consumer_unread_loss_count") != 0 or
            history.get("status") != "OK" or history.get("error") is not None or
            history.get("identity_content_conflict_count") != 0 or
            history.get("consumer_unread_loss_count") != 0):
        return None
    # A prior implementation stopped the recorder before its final drain. Only
    # this known lifecycle error is admissible, and only with a proven complete
    # terminal boundary and no task-window unread loss.
    if history.get("merge_error") not in (
            None, "capture was stopped before the observed policy frame was persisted"):
        return None
    if terminal.get("run_terminal_result") != context.get("result_contract", {}).get("classification"):
        return None
    if terminal.get("shutdown_complete") is not True:
        return None

    by_cycle: dict[int, dict[str, Any]] = {}
    for payload in live_frames:
        if payload.get("session_id") != session_id:
            return None
        cycle = payload.get("rl_step")
        if type(cycle) is not int:
            return None
        if 0 <= cycle <= end_cycle:
            if cycle in by_cycle:
                return None
            by_cycle[cycle] = payload
    expected = list(range(end_cycle + 1))
    if sorted(by_cycle) != expected:
        return None
    task_frames = [by_cycle[cycle] for cycle in expected]
    required = ("source_sequence", "monotonic_ns", "sim_time_s", "sim_clock_valid",
                "world_pose", "ra_value", "mode_before", "mode_after", "action_source",
                "action_raw", "action_clipped", "joint_target_rad")
    previous_monotonic = previous_sim = None
    for payload in task_frames:
        if (payload.get("rl_entered") != 1 or payload.get("rl_active") != 1 or
                payload.get("sim_clock_valid") is not True or
                any(key not in payload or payload.get(key) is None for key in required)):
            return None
        sim_time, monotonic = payload.get("sim_time_s"), payload.get("monotonic_ns")
        if (not isinstance(sim_time, (int, float)) or not math.isfinite(float(sim_time)) or
                type(monotonic) is not int or
                (previous_sim is not None and sim_time <= previous_sim) or
                (previous_monotonic is not None and monotonic <= previous_monotonic)):
            return None
        previous_sim, previous_monotonic = sim_time, monotonic
    last = task_frames[-1]
    if any(last.get(payload_key) != terminal_ref.get(ref_key) for payload_key, ref_key in (
            ("session_id", "session_id"), ("rl_step", "rl_step"),
            ("source_sequence", "source_sequence"), ("monotonic_ns", "monotonic_ns"))):
        return None
    closeout_missing = final_producer - final_cursor
    if closeout_missing <= 0:
        return None
    return {
        "status": "task_window_complete_closeout_incomplete",
        "task_frames": task_frames,
        "task_cycle_start": 0,
        "task_cycle_end": end_cycle,
        "task_frame_count": len(task_frames),
        "persisted_live_frame_count": len(live_frames),
        "closeout_producer_entries_unpersisted": closeout_missing,
        "full_record_validity": "INVALID",
        "reason": "complete task boundary; final producer history has unpersisted closeout entries",
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
    persisted_live_frame_count = len(frames)
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
    task_closeout_recovery = None
    if base.get("record_validity") != "VALID":
        task_closeout_recovery = _closeout_only_task_window(data, context, base, frames)
        if task_closeout_recovery is not None:
            frames = task_closeout_recovery["task_frames"]
        else:
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
                "task_window_metrics_status": "UNKNOWN: task boundary failed closeout-only recovery checks",
                "all_derived_metrics": "UNKNOWN: record fails continuity/schema validation; no aggregation across sessions",
                "collision_coverage": base.get("collision"),
                "external_safety_events": (data.terminal or {}).get("external_safety_events", []),
                "cleanup_events": (data.terminal or {}).get("cleanup_events", []),
                "command_consumption": "UNKNOWN",
            }
    gaps = 0
    cadence = summarize_policy_cadence(frames)
    periods_s: list[float] = []
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
    motion_start_event = next((event for event in context.get("events", [])
                               if event.get("phase") == "motion" and event.get("event") == "motion_start"), None)
    motion_start_session = motion_start_event.get("session_id") if isinstance(motion_start_event, dict) else None
    motion_start_cycle = motion_start_event.get("rl_step") if isinstance(motion_start_event, dict) else None
    terminal_session = terminal_ref.get("session_id") if isinstance(terminal_ref, dict) else None
    terminal_cycle = terminal_ref.get("rl_step") if isinstance(terminal_ref, dict) else None
    if (isinstance(motion_start_session, int) and isinstance(motion_start_cycle, int) and
            terminal_session == motion_start_session and isinstance(terminal_cycle, int) and
            terminal_cycle >= motion_start_cycle):
        # Use policy-cycle identity for event/count boundaries so transitions
        # on invalid-clock cycles remain counted. Timed metrics still use only
        # valid, contiguous sim-time pairs below.
        motion_cycle_frames = [p for p in frames
                               if p.get("session_id") == motion_start_session and
                               isinstance(p.get("rl_step"), int) and
                               motion_start_cycle <= p["rl_step"] <= terminal_cycle]
        motion_boundary_identity_complete = True
    else:
        motion_cycle_frames = movement_frames
        motion_boundary_identity_complete = False
    movement_keys = {(p.get("session_id"), p.get("rl_step")) for p in motion_cycle_frames}
    # Preserve the historical >=0.10m displacement metric boundary while also
    # reporting an earlier policy-active epoch for future formal definitions.
    active_event = lifecycle.get("policy_active_start", {})
    active_session = active_event.get("session_id") if isinstance(active_event, dict) else None
    active_cycle = active_event.get("rl_step") if isinstance(active_event, dict) else None
    if not isinstance(active_session, int) or not isinstance(active_cycle, int):
        active_frame = next((p for p in frames if p.get("rl_entered") == 1 and p.get("rl_active") == 1), None)
        if active_frame:
            active_session, active_cycle = active_frame.get("session_id"), active_frame.get("rl_step")
    policy_cycle_frames = []
    policy_boundary_identity_complete = False
    if (terminal_frame_payload_complete is True and
            isinstance(active_session, int) and isinstance(active_cycle, int) and
            terminal_session == active_session and isinstance(terminal_cycle, int) and terminal_cycle >= active_cycle):
        policy_cycle_frames = [p for p in frames if p.get("session_id") == active_session and
                               isinstance(p.get("rl_step"), int) and active_cycle <= p["rl_step"] <= terminal_cycle]
        policy_boundary_identity_complete = True
    policy_keys = {(p.get("session_id"), p.get("rl_step")) for p in policy_cycle_frames}
    policy_pairs = [(a, b, dt) for a, b, dt in usable_pairs
                    if (a.get("session_id"), a.get("rl_step")) in policy_keys and
                    (b.get("session_id"), b.get("rl_step")) in policy_keys]
    policy_active_valid_duration = sum(dt for _, _, dt in policy_pairs)
    policy_active_path = sum(math.hypot(b["world_pose"][0] - a["world_pose"][0],
                                        b["world_pose"][1] - a["world_pose"][1])
                             for a, b, _ in policy_pairs)
    goal = context.get("settings", {}).get("goal_world_m", [None, None])
    task_peak_interval_path_speed = max(
        (math.hypot(b["world_pose"][0] - a["world_pose"][0],
                    b["world_pose"][1] - a["world_pose"][1]) / dt
         for a, b, dt in policy_pairs if dt > 0), default=None)
    policy_active_elapsed = None
    active_start_frame = policy_cycle_frames[0] if policy_cycle_frames else None
    terminal_policy_frame = next((p for p in policy_cycle_frames
                                  if isinstance(terminal_ref, dict) and
                                  p.get("session_id") == terminal_ref.get("session_id") and
                                  p.get("rl_step") == terminal_ref.get("rl_step")), None)
    if (active_start_frame and terminal_policy_frame and terminal_frame_payload_complete is True and
            active_start_frame.get("sim_clock_valid") is True and terminal_policy_frame.get("sim_clock_valid") is True and
            isinstance(active_start_frame.get("sim_time_s"), (int, float)) and
            isinstance(terminal_policy_frame.get("sim_time_s"), (int, float)) and
            terminal_policy_frame["sim_time_s"] >= active_start_frame["sim_time_s"]):
        policy_active_elapsed = terminal_policy_frame["sim_time_s"] - active_start_frame["sim_time_s"]
    task_goal_distance_start = None
    task_goal_distance_final = None
    if policy_cycle_frames and goal and goal[0] is not None:
        task_goal_distance_start = math.hypot(goal[0] - policy_cycle_frames[0]["world_pose"][0],
                                              goal[1] - policy_cycle_frames[0]["world_pose"][1])
        task_goal_distance_final = math.hypot(goal[0] - policy_cycle_frames[-1]["world_pose"][0],
                                              goal[1] - policy_cycle_frames[-1]["world_pose"][1])
    declared_bounds = settings.get("declared_field_bounds_m")
    boundary_observation = {"status": "NOT_APPLICABLE", "scope": "no declared scene boundary in run context"}
    if isinstance(declared_bounds, dict) and policy_cycle_frames:
        try:
            xmin, xmax = map(float, declared_bounds["x"])
            ymin, ymax = map(float, declared_bounds["y"])
            out_frames = [p for p in policy_cycle_frames
                          if not (xmin <= float(p["world_pose"][0]) <= xmax and
                                  ymin <= float(p["world_pose"][1]) <= ymax)]
            expected_cycles = (terminal_cycle - active_cycle + 1
                               if isinstance(terminal_cycle, int) and isinstance(active_cycle, int) else None)
            cycle_coverage_complete = (expected_cycles is not None and
                                       len(policy_cycle_frames) == expected_cycles and
                                       all(b.get("rl_step") == a.get("rl_step") + 1
                                           for a,b in zip(policy_cycle_frames,policy_cycle_frames[1:])))
            valid_oob_span = 0.0
            for a,b,dt in policy_pairs:
                a_out = not (xmin <= float(a["world_pose"][0]) <= xmax and ymin <= float(a["world_pose"][1]) <= ymax)
                b_out = not (xmin <= float(b["world_pose"][0]) <= xmax and ymin <= float(b["world_pose"][1]) <= ymax)
                if a_out and b_out:
                    valid_oob_span += dt
            status = "OUT_OF_BOUNDS_SAMPLED" if out_frames else (
                "SAMPLED_IN_BOUNDS" if cycle_coverage_complete and
                all(p.get("sim_clock_valid") is True for p in policy_cycle_frames) else "UNKNOWN_INCOMPLETE_COVERAGE")
            boundary_observation = {
                "status": status, "declared_bounds_m": {"x": [xmin,xmax], "y": [ymin,ymax]},
                "sampled_policy_frame_count": len(policy_cycle_frames),
                "expected_policy_cycle_count": expected_cycles,
                "cycle_coverage_complete": cycle_coverage_complete,
                "out_of_bounds_frame_count": len(out_frames),
                "out_of_bounds_frames": [{"session_id":p.get("session_id"),"rl_step":p.get("rl_step"),
                                            "sim_time_s":p.get("sim_time_s") if p.get("sim_clock_valid") is True else None,
                                            "world_xy_m":p.get("world_pose",[])[:2]} for p in out_frames],
                "minimum_observed_contiguous_outside_duration_s": valid_oob_span if valid_oob_span > 0 else None,
                "duration_scope": "only adjacent valid policy frames both sampled outside; no interpolation",
                "interpretation": "sampled positions only; no physical trajectory between policy cycles, so in-bounds samples do not prove continuous footprint containment",
            }
        except (KeyError, TypeError, ValueError, IndexError):
            boundary_observation = {"status":"UNKNOWN_INVALID_BOUNDARY_OR_POSE_DATA",
                                    "scope":"declared boundary exists but sampled XY/bounds are unusable"}
    policy_active_transitions = [p for p in policy_cycle_frames
                                 if p.get("mode_before") in (0, 1) and p.get("mode_after") in (0, 1)
                                 and p["mode_before"] != p["mode_after"]]
    valid_times = [p["sim_time_s"] for p in frames if p.get("sim_clock_valid") is True and isinstance(p.get("sim_time_s"), (int, float))]
    goal_distance_start = None
    goal_distance_final = None
    trajectory_length = 0.0
    valid_motion_duration = 0.0
    recovery_duration = 0.0
    peak_interval_path_speed = None
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
        interval_speed = distance / dt
        peak_interval_path_speed = (interval_speed if peak_interval_path_speed is None else
                                    max(peak_interval_path_speed, interval_speed))
        if a.get("mode_after") == 1:
            recovery_duration += dt
    if movement_frames and goal and goal[0] is not None:
        valid_pose = movement_frames
        if valid_pose:
            first, last = valid_pose[0], valid_pose[-1]
            goal_distance_start = math.hypot(goal[0] - first["world_pose"][0], goal[1] - first["world_pose"][1])
            goal_distance_final = math.hypot(goal[0] - last["world_pose"][0], goal[1] - last["world_pose"][1])
    elapsed_motion_duration = (
        movement_end - movement_start
        if (motion_boundary_identity_complete and isinstance(movement_start, (int, float)) and
            isinstance(movement_end, (int, float)) and math.isfinite(movement_start) and
            math.isfinite(movement_end) and movement_end >= movement_start and
            terminal_frame_payload_complete is True)
        else None)

    motion_transitions = [t for t in transitions if (t.get("session_id"), t.get("rl_step")) in movement_keys]
    task_transitions = [p for p in policy_active_transitions
                        if p.get("mode_before") in (0, 1) and p.get("mode_after") in (0, 1)
                        and p.get("mode_before") != p.get("mode_after")]
    recovery_hold_audit = audit_hold_episodes(policy_cycle_frames)
    risk_edges = [p for p in policy_cycle_frames if p.get("risk_condition_entered") == 1]
    risk_to_recovery: list[dict[str, Any]] = []
    risk_reentered_in_recovery = 0
    risk_response_unavailable = 0
    for p in risk_edges:
        if p.get("mode_before") == 1:
            risk_reentered_in_recovery += 1
            continue
        risk_ns = p.get("risk_condition_entered_ns")
        change_ns = p.get("mode_change_ns")
        if (p.get("mode_before") == 0 and p.get("mode_after") == 1 and
                p.get("policy_mode_changed") is True and
                isinstance(risk_ns, int) and isinstance(change_ns, int) and change_ns >= risk_ns):
            risk_to_recovery.append({"session_id": p.get("session_id"),
                                     "rl_step": p.get("rl_step"),
                                     "sim_time_s": p.get("sim_time_s") if p.get("sim_clock_valid") is True else None,
                                     "response_ns": change_ns - risk_ns})
        else:
            risk_response_unavailable += 1
    risk_response_ns = [item["response_ns"] for item in risk_to_recovery]
    task_start_event = lifecycle.get("task_start") or lifecycle.get("policy_active_start") or {}
    task_start_time = task_start_event.get("sim_time_s") if task_start_event.get("sim_clock_valid") else None
    pre_task_boundary = context.get("initial_state", {}).get("pre_task_safety_boundary", {})
    task_boundary_snapshot = pre_task_boundary.get("snapshot") if pre_task_boundary.get("status") == "LIVE" else None
    task_start_frame = next((p for p in frames
                             if p.get("session_id") == task_start_event.get("session_id") and
                             p.get("rl_step") == task_start_event.get("rl_step")), None)
    task_start_physics_snapshot = (
        task_start_frame.get("collision_snapshot") if isinstance(task_start_frame, dict) else None)
    if not isinstance(task_start_physics_snapshot, dict):
        task_start_physics_snapshot = task_boundary_snapshot
    terminal_policy_payload = next((p for p in frames
                                    if isinstance(terminal_ref, dict) and
                                    p.get("session_id") == terminal_ref.get("session_id") and
                                    p.get("rl_step") == terminal_ref.get("rl_step") and
                                    (terminal_ref.get("source_sequence") is None or
                                     p.get("source_sequence") == terminal_ref.get("source_sequence")) and
                                    (terminal_ref.get("monotonic_ns") is None or
                                     p.get("monotonic_ns") == terminal_ref.get("monotonic_ns"))), None)
    terminal_policy_physics_snapshot = (
        terminal_policy_payload.get("collision_snapshot")
        if isinstance(terminal_policy_payload, dict) else None)
    # Safety evidence for motion is bounded by persisted policy-cycle identity,
    # just like the task metrics. `frames` also contains post-terminal standing
    # and cleanup records; their attached physical snapshots may be STALE and
    # must not contaminate a completed task interval. Keep the full-capture
    # coverage separately in `capture_collision_coverage` below.
    safety_boundary_snapshots = ([task_boundary_snapshot]
                                 if isinstance(task_boundary_snapshot, dict) else [])
    safety = motion_safety(
        policy_cycle_frames, movement_start, movement_end,
        terminal_policy_physics_snapshot, safety_boundary_snapshots,
        start_boundary_snapshot=task_start_physics_snapshot,
        end_boundary_snapshot=terminal_policy_physics_snapshot)
    pre_task_bounds = pre_task_boundary.get("base_bounds_snapshot")
    terminal_bounds = (terminal_policy_payload.get("base_bounds_snapshot")
                       if isinstance(terminal_policy_payload, dict) else None)
    active_bounds = (task_start_frame.get("base_bounds_snapshot")
                     if isinstance(task_start_frame, dict) else None)
    physical_boundary_observation = physical_base_bounds_window(
        pre_task_bounds, terminal_bounds,
        (active_bounds.get("sim_time") if isinstance(active_bounds, dict) and
         active_bounds.get("status") == "LIVE" else None),
        (terminal_bounds.get("sim_time") if isinstance(terminal_bounds, dict) and
         terminal_bounds.get("status") == "LIVE" else None))
    task_safety = motion_safety(
        policy_cycle_frames, task_start_time, movement_end,
        terminal_policy_physics_snapshot, safety_boundary_snapshots,
        start_boundary_snapshot=task_start_physics_snapshot,
        end_boundary_snapshot=(terminal_policy_physics_snapshot
                               if isinstance(terminal_policy_physics_snapshot, dict)
                               else None))
    task_recovery_duration = sum(dt for a, b, dt in policy_pairs if a.get("mode_after") == 1)
    task_elapsed_coverage_fraction = (
        policy_active_valid_duration / policy_active_elapsed
        if isinstance(policy_active_elapsed, (int, float)) and policy_active_elapsed > 0 else None)
    declared_terminal = context.get("result_contract", {}).get("classification", "UNKNOWN")
    terminal_safety_conflict = (
        (declared_terminal == "COLLISION_TERMINATION" and task_safety["collision"] is not True) or
        (declared_terminal == "FALL_TERMINATION" and task_safety["fall"] is not True) or
        (declared_terminal in {"ARRIVED", "DIAGNOSTIC_SIM_TIME_LIMIT"} and
         (task_safety["collision"] is True or task_safety["fall"] is True))
    )
    agile_recovery = sum(1 for p in task_transitions if p["mode_before"] == 0 and p["mode_after"] == 1)
    recovery_agile = sum(1 for p in task_transitions if p["mode_before"] == 1 and p["mode_after"] == 0)
    agile_cycles = sum(1 for p in policy_cycle_frames if p.get("mode_after") == 0)
    recovery_cycles = sum(1 for p in policy_cycle_frames if p.get("mode_after") == 1)
    motion_transitions_with_valid_time = sum(1 for p in task_transitions if p.get("sim_clock_valid") is True and isinstance(p.get("sim_time_s"), (int, float)))
    motion_invalid_clock_cycles = sum(1 for p in policy_cycle_frames
                                      if p.get("sim_clock_valid") is not True)
    motion_policy_cycle_gaps = sum(1 for a, b in zip(policy_cycle_frames, policy_cycle_frames[1:])
                                   if a.get("session_id") != b.get("session_id") or
                                   b.get("rl_step") != a.get("rl_step", -2) + 1)
    observed_mode_edge_coverage = mode_edge_coverage(policy_cycle_frames,
                                                    policy_boundary_identity_complete)
    reverse_pairs = 0
    for left, right in zip(task_transitions, task_transitions[1:]):
        left_time = left.get("sim_time_s") if left.get("sim_clock_valid") else None
        right_time = right.get("sim_time_s") if right.get("sim_clock_valid") else None
        if (left["mode_before"], left["mode_after"]) == (0, 1) and (right["mode_before"], right["mode_after"]) == (1, 0):
            if left_time is not None and right_time is not None and 0 <= right_time - left_time <= reverse_window_s:
                reverse_pairs += 1
        elif (left["mode_before"], left["mode_after"]) == (1, 0) and (right["mode_before"], right["mode_after"]) == (0, 1):
            if left_time is not None and right_time is not None and 0 <= right_time - left_time <= reverse_window_s:
                reverse_pairs += 1

    intervals: list[dict[str, Any]] = []
    if policy_cycle_frames:
        start = policy_cycle_frames[0]
        state = start.get("mode_after")
        prior = start
        for item in policy_cycle_frames[1:]:
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
        "task_window_metrics_status": (task_closeout_recovery["status"]
                                        if task_closeout_recovery is not None else
                                        ("complete" if base.get("record_validity") == "VALID" else "UNKNOWN")),
        "task_window_integrity": ({key: value for key, value in task_closeout_recovery.items()
                                   if key != "task_frames"}
                                  if task_closeout_recovery is not None else None),
        "persisted_live_frame_count_total": persisted_live_frame_count,
        "analyzed_frame_count": len(frames),
        "policy_cycle_history": base.get("policy_cycle_history"),
        "policy_cycle_history_accounting_version": (
            (base.get("policy_cycle_history") or {}).get("accounting_version")
            if isinstance(base.get("policy_cycle_history"), dict) else None),
        "policy_cycle_history_complete_at_task_termination": (
            ((base.get("policy_cycle_history") or {}).get("task_termination_boundary") or {}).get("status") == "complete"
            if isinstance((base.get("policy_cycle_history") or {}).get("task_termination_boundary"), dict) else None),
        "policy_cycle_history_complete_at_final_closeout_drain": (
            ((base.get("policy_cycle_history") or {}).get("final_closeout_drain") or {}).get("complete")
            if isinstance((base.get("policy_cycle_history") or {}).get("final_closeout_drain"), dict) else
            ((base.get("policy_cycle_history") or {}).get("complete_at_last_drain")
             if isinstance(base.get("policy_cycle_history"), dict) else None)),
        "policy_cycle_producer_slot_reuse_count": (
            (base.get("policy_cycle_history") or {}).get("producer_slot_reuse_count",
                (base.get("policy_cycle_history") or {}).get("producer_overwritten_count"))
            if isinstance(base.get("policy_cycle_history"), dict) else None),
        "policy_cycle_consumer_unread_loss_count": (
            (base.get("policy_cycle_history") or {}).get("consumer_unread_loss_count")
            if isinstance(base.get("policy_cycle_history"), dict) else None),
        "terminal_result": final_result,
        "navigation_terminal_result": final_result,
        "run_terminal_result_context": context.get("result_contract", {}).get("classification", "UNKNOWN"),
        "run_terminal_result_record": (data.terminal or {}).get("run_terminal_result"),
        "cleanup_events": (data.terminal or {}).get("cleanup_events", []),
        "motion_metric_time_boundary": {"start_sim_time_s": movement_start, "end_sim_time_s": movement_end,
                                         "start_cycle": (motion_start_cycle if motion_boundary_identity_complete else
                                                         (movement_frames[0].get("rl_step") if movement_frames else None)),
                                         "end_cycle": terminal_ref.get("rl_step") if isinstance(terminal_ref, dict) else None,
                                         "terminal_frame_payload_complete": terminal_frame_payload_complete,
                                         "cycle_identity_complete": motion_boundary_identity_complete},
        "rl_task_metric_boundary": {
            "definition": "first persisted frame with rl_entered=1 and rl_active=1 through terminal frame, inclusive",
            "start_cycle": active_cycle if policy_boundary_identity_complete else None,
            "end_cycle": terminal_cycle if policy_boundary_identity_complete else None,
            "cycle_identity_complete": policy_boundary_identity_complete,
            "elapsed_sim_s": policy_active_elapsed,
            "valid_continuous_sim_s": policy_active_valid_duration if policy_active_valid_duration else None,
            "valid_continuous_path_length_m": policy_active_path if policy_active_valid_duration else None,
            "mean_valid_continuous_speed_mps": (policy_active_path / policy_active_valid_duration
                                                 if policy_active_valid_duration else None),
            "mode_transition_count_all_cycles": len(policy_active_transitions) if policy_cycle_frames else None,
            "mode_transitions_with_valid_sim_time": sum(1 for p in policy_active_transitions
                                                          if p.get("sim_clock_valid") is True and
                                                          isinstance(p.get("sim_time_s"), (int, float)))
            if policy_cycle_frames else None,
            "valid_time_coverage_fraction_of_elapsed": task_elapsed_coverage_fraction,
            "recovery_valid_continuous_sim_s": task_recovery_duration if policy_active_valid_duration else None,
            "recovery_share_of_valid_continuous_time": (task_recovery_duration / policy_active_valid_duration
                                                           if policy_active_valid_duration else None),
            "task_window_mode_edges_all_cycles": len(policy_active_transitions) if policy_cycle_frames else None,
            "frequency_of_policy_activity_cycles_excluded_before_displacement_start": (
                max(0, len(policy_cycle_frames) - len(motion_cycle_frames))
                if policy_boundary_identity_complete and motion_boundary_identity_complete else None),
        },
        "session_id": context.get("session_id"), "frame_count": len(frames),
        "rl_step_gaps": gaps, "invalid_clock_frames": invalid_clock_frames,
        "clock_valid_fraction": ((len(frames) - invalid_clock_frames) / len(frames)) if frames else None,
        "first_valid_sim_time_s": min(valid_times) if valid_times else None,
        "last_valid_sim_time_s": max(valid_times) if valid_times else None,
        "policy_interval_ms_median": statistics.median(periods_s) * 1000 if periods_s else None,
        "policy_interval_ms_p95": quantile(periods_s, 0.95) * 1000 if periods_s else None,
        "policy_interval_ms_min": min(periods_s) * 1000 if periods_s else None,
        "policy_interval_ms_max": max(periods_s) * 1000 if periods_s else None,
        "policy_frequency_hz_overall": cadence["policy_frequency_hz_overall"],
        "policy_frequency_definition": cadence["frequency_scope"],
        "policy_frequency_observed_cycle_span": cadence["observed_policy_cycle_span"],
        "policy_frequency_monotonic_span_s": cadence["observed_wall_span_s"],
        "policy_frequency_coverage": cadence["coverage"],
        "policy_period_statistics_scope": cadence["period_quantile_scope"],
        "mode_transitions": len(task_transitions) if policy_cycle_frames else None,
        "mode_transitions_observed_in_persisted_frames": len(task_transitions) if policy_cycle_frames else None,
        "mode_transition_count_coverage": observed_mode_edge_coverage,
        "mode_transition_count_complete": observed_mode_edge_coverage == "complete",
        "agile_to_recovery": agile_recovery if policy_cycle_frames else None,
        "recovery_to_agile": recovery_agile if policy_cycle_frames else None,
        "transitions_with_valid_sim_time": motion_transitions_with_valid_time if policy_cycle_frames else None,
        "agile_cycles": agile_cycles if policy_cycle_frames else None,
        "recovery_cycles": recovery_cycles if policy_cycle_frames else None,
        "motion_invalid_clock_cycles": motion_invalid_clock_cycles if policy_cycle_frames else None,
        "motion_policy_cycle_gaps": motion_policy_cycle_gaps if policy_cycle_frames else None,
        "transitions": task_transitions, "legacy_displacement_window_transitions": motion_transitions,
        "recovery_intervals": intervals,
        "risk_to_recovery_response": {
            "definition": "risk_condition_entered to paired same-cycle Agile->Recovery policy_mode_changed; monotonic delta is internal decision-path time only, not complete avoidance response, command write, action application, or MuJoCo consumption",
            "risk_edge_count": risk_edge_counts(policy_cycle_frames)["true"] if policy_cycle_frames else None,
            "risk_edge_enum_counts": risk_edge_counts(policy_cycle_frames) if policy_cycle_frames else None,
            "paired_Agile_to_Recovery_count": len(risk_to_recovery) if policy_cycle_frames else None,
            "risk_reentered_while_already_Recovery_count": risk_reentered_in_recovery if policy_cycle_frames else None,
            "unavailable_or_unpaired_count": risk_response_unavailable if policy_cycle_frames else None,
            "min_ns": min(risk_response_ns) if risk_response_ns else None,
            "median_ns": statistics.median(risk_response_ns) if risk_response_ns else None,
            "max_ns": max(risk_response_ns) if risk_response_ns else None,
            "events": risk_to_recovery,
        },
        "recovery_hold_audit": recovery_hold_audit,
        "recovery_sim_duration_s": task_recovery_duration if policy_active_valid_duration else None,
        "valid_motion_duration_s": policy_active_valid_duration if policy_active_valid_duration else None,
        "elapsed_motion_duration_s": policy_active_elapsed,
        "elapsed_task_duration_s": policy_active_elapsed,
        "task_valid_time_coverage_fraction": task_elapsed_coverage_fraction,
        "recovery_share_valid_motion_time": task_recovery_duration / policy_active_valid_duration if policy_active_valid_duration else None,
        "task_start_sim_time_s": task_start_time,
        "legacy_displacement_start_sim_time_s": movement_start,
        "target_distance_start_m": task_goal_distance_start, "target_distance_final_m": task_goal_distance_final,
        "declared_boundary_observation": boundary_observation,
        "declared_boundary_physics_observation": physical_boundary_observation,
        "path_length_m": policy_active_path if policy_active_valid_duration else None,
        "mean_path_speed_mps": policy_active_path / policy_active_valid_duration if policy_active_valid_duration else None,
        "peak_interval_path_speed_mps": task_peak_interval_path_speed,
        "legacy_displacement_window_metrics": {
            "start_sim_time_s": movement_start, "elapsed_s": elapsed_motion_duration,
            "valid_continuous_s": valid_motion_duration if valid_motion_duration else None,
            "path_length_m": trajectory_length if valid_motion_duration else None,
            "mean_speed_mps": trajectory_length / valid_motion_duration if valid_motion_duration else None,
            "mode_transition_count": len(motion_transitions) if motion_cycle_frames else None,
        },
        "short_reverse_switch_count": reverse_pairs if policy_cycle_frames else None,
        "short_reverse_switch_coverage": ("complete" if policy_cycle_frames and
                                           motion_invalid_clock_cycles == 0 and
                                           motion_policy_cycle_gaps == 0 and
                                           motion_transitions_with_valid_time == len(task_transitions)
                                           else "partial_or_unknown" if policy_cycle_frames else "UNKNOWN"),
        "short_reverse_window_s": reverse_window_s,
        "period_statistics_scope": ("validated task window only; full record has an incomplete closeout tail"
                                     if task_closeout_recovery is not None else
                                     "all captured LIVE frames including preparation and cleanup tail, if present"),
        "motion_metrics_scope": "primary task window: first rl_entered=1 && rl_active=1 frame through terminal cycle inclusive; cleanup tail excluded",
        "legacy_displacement_window_scope": "historical auxiliary window: first observed >=0.10m displacement through terminal frame; preserved separately and not used as the primary task boundary",
        "capture_collision_coverage": base.get("collision"),
        "collision_coverage": {
            "scope": "primary task interval from first active policy cycle through terminal inclusive",
            "result": task_safety["collision"],
            "complete_for_motion_interval": task_safety["coverage_complete_for_motion_interval"],
            "reason": task_safety["coverage_reason"],
            "evidence": task_safety["coverage_evidence"],
        },
        "capture_fall_events": (data.terminal or {}).get("fall_events"),
        "motion_fall_result": task_safety["fall"],
        "motion_safety": task_safety,
        "task_window_safety": task_safety,
        "legacy_displacement_window_safety": safety,
        "nonfoot_collision_failure": task_safety["nonfoot_collision_failure"],
        "foot_contact_count": task_safety["foot_contact_count"],
        "foot_contact_duration_s": task_safety["foot_contact_duration_s"],
        "foot_contact_duration_capture_s": task_safety["foot_contact_duration_capture_s"],
        "foot_impact_count": task_safety["foot_impact_count"],
        "foot_impact_duration_s": task_safety["foot_impact_duration_s"],
        "foot_impact_duration_capture_s": task_safety["foot_impact_duration_capture_s"],
        "arrived_with_foot_contact": (
            (task_safety["foot_contact_count"] > 0 if isinstance(task_safety["foot_contact_count"], int)
             else "UNKNOWN") if final_result == "ARRIVED" else False),
        "arrived_with_foot_impact": (
            (task_safety["foot_impact_count"] > 0 if isinstance(task_safety["foot_impact_count"], int)
             else "UNKNOWN") if final_result == "ARRIVED" else False),
        "abs_reference_collision": task_safety["abs_reference_collision"],
        "s6_preparation_readiness": {
            "first_active_policy_frame_exactly_persisted": (
                context.get("initial_state", {}).get("first_active_policy_cycle", {}).get("status") == "CAPTURED" and
                context.get("initial_state", {}).get("first_active_policy_cycle", {}).get("same_cycle_runtime_frame_persisted_by_supervisor") is True),
            "recorder_ready_before_rl": (context.get("preflight", {}).get("recorder_readiness", {}).get("status") == "ready_readable_and_writable"),
            "pre_rl_standing_confirmed": (context.get("initial_state", {}).get("pre_rl_standing_preparation", {}).get("status") == "STABLE"),
            "pre_task_safety_boundary_live": (pre_task_boundary.get("status") == "LIVE"),
        },
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
    task_event = lifecycle.get("task_start") or lifecycle.get("policy_active_start") or {}
    motion_start = task_event.get("sim_time_s") if task_event.get("sim_clock_valid") else None
    displacement_start = lifecycle.get("motion_start_observed_sim_time_s")
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
        ax.axvline(motion_start, color="#333333", lw=0.9, ls=":", label="task start: first active policy cycle")
        if isinstance(displacement_start, (int, float)) and motion_start <= displacement_start <= motion_end:
            ax.axvline(displacement_start, color="#777777", lw=0.8, ls="--", label="legacy 0.10 m displacement start")
        ax.axvline(motion_end, color="#222222", lw=1.2, ls="-.", label=f"terminal: {result}")
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
        ax.text(0.01, 0.02, f"task window: first active policy cycle through terminal; invalid clock frames in full capture: {sum(p.get('sim_clock_valid') is not True for p in rows)}; mode band = Recovery",
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
            "s6_preparation_readiness", "task_window_safety", "nonfoot_collision_failure", "foot_contact_count", "foot_contact_duration_s",
            "foot_impact_count", "foot_impact_duration_s", "arrived_with_foot_contact",
            "arrived_with_foot_impact", "abs_reference_collision", "frame_count", "rl_step_gaps",
            "invalid_clock_frames", "policy_frequency_hz_overall", "rl_task_metric_boundary",
            "mode_transitions",
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
