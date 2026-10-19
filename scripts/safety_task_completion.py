"""Thin, explicit classifier for the formal safe-task-completion metric.

This module consumes the production analyzer's existing task-window results. It
does not detect collisions, falls, bounds violations, or rewrite raw terminals.
"""
from __future__ import annotations

import math
from typing import Any


TASK_NONARRIVAL_TERMINALS = {
    "COLLISION_TERMINATION",
    "FALL_TERMINATION",
    "SYSTEM_SAFETY_ABORT",
    "DIAGNOSTIC_SIM_TIME_LIMIT",
}
TASK_INCOMPLETE_TERMINALS = {"WALL_CLOCK_GUARD_INCOMPLETE", "OPERATOR_ABORT", "TIMEOUT"}


def _tri(value: Any) -> bool | str:
    return value if isinstance(value, bool) else "UNKNOWN"


def _ref_key(ref: Any) -> tuple[Any, ...] | None:
    if not isinstance(ref, dict):
        return None
    keys = ("session_id", "rl_step", "source_sequence", "monotonic_ns")
    if any(ref.get(key) is None for key in keys):
        return None
    return tuple(ref.get(key) for key in keys)


def _system_safety_abort_dimension(raw_terminal: str, context: dict[str, Any],
                                   terminal: dict[str, Any], task_started: bool) -> tuple[bool | str, list[dict[str, Any]]]:
    if raw_terminal == "SYSTEM_SAFETY_ABORT" and task_started:
        return True, [{"source": "raw_task_terminal", "classification": raw_terminal}]
    lifecycle = context.get("lifecycle", {})
    task_start = lifecycle.get("task_start") or lifecycle.get("policy_active_start") or {}
    terminal_ref = lifecycle.get("terminal_frame_ref") or terminal.get("run_terminal_frame_ref") or {}
    expected_session = task_start.get("session_id", context.get("session_id"))
    start_cycle = task_start.get("rl_step")
    end_cycle = terminal_ref.get("rl_step")
    events = list(terminal.get("external_safety_events") or [])
    boundary_event = context.get("result_contract", {}).get("external_safety_boundary")
    if isinstance(boundary_event, dict) and boundary_event not in events:
        events.append(boundary_event)
    findings: list[dict[str, Any]] = []
    unknown = False
    safety_names = {"POLICY_SAFETY_VETO", "SAFETY_ABORT", "SAFETY_FAULT"}
    for event in events:
        if not isinstance(event, dict) or event.get("event") not in safety_names:
            continue
        if event.get("phase") in {"cleanup", "standing", "post_terminal"}:
            findings.append({"event": event.get("event"), "status": "OUTSIDE_TASK_WINDOW_PHASE"})
            continue
        event_session = event.get("controller_session_id", event.get("session_id"))
        event_cycle = event.get("controller_policy_cycle", event.get("rl_step"))
        if (not isinstance(expected_session, int) or not isinstance(start_cycle, int) or
                not isinstance(end_cycle, int) or not isinstance(event_session, int) or
                not isinstance(event_cycle, int)):
            unknown = True
            findings.append({"event": event.get("event"), "status": "TASK_BOUNDARY_OR_EVENT_IDENTITY_UNKNOWN"})
            continue
        if event_session != expected_session:
            unknown = True
            findings.append({"event": event.get("event"), "status": "EVENT_SESSION_MISMATCH"})
        elif start_cycle <= event_cycle <= end_cycle:
            findings.append({"event": event.get("event"), "status": "IN_TASK_WINDOW",
                             "session_id": event_session, "cycle": event_cycle})
            return True, findings
        else:
            findings.append({"event": event.get("event"), "status": "OUTSIDE_TASK_CYCLE_WINDOW",
                             "session_id": event_session, "cycle": event_cycle})
    if unknown:
        return "UNKNOWN", findings
    if raw_terminal in {"ARRIVED", "COLLISION_TERMINATION", "FALL_TERMINATION",
                        "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE",
                        "OPERATOR_ABORT", "TIMEOUT", "SYSTEM_SAFETY_ABORT"} and task_started:
        return False, findings
    return "UNKNOWN", findings


def assess_task_identity(summary: dict[str, Any], context: dict[str, Any],
                         terminal: dict[str, Any] | None,
                         task_frames: list[dict[str, Any]] | None = None,
                         boundary_snapshots: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    """Check run/session/scene/model identity without relaxing event coverage."""
    terminal = terminal or {}
    settings = context.get("settings", {})
    lifecycle = context.get("lifecycle", {})
    reasons: list[str] = []
    invalid = False
    incomplete = False

    context_run = context.get("run_id")
    terminal_run = terminal.get("run_id")
    summary_run = summary.get("run_id")
    run_values = [value for value in (context_run, terminal_run, summary_run) if value is not None]
    if len(set(run_values)) > 1:
        invalid = True
        reasons.append("run_id_mismatch")
    if not run_values:
        incomplete = True
        reasons.append("run_id_missing")

    expected_session = context.get("session_id")
    observed_sessions = {frame.get("session_id") for frame in (task_frames or [])
                         if frame.get("session_id") is not None}
    if expected_session is not None and any(value != expected_session for value in observed_sessions):
        invalid = True
        reasons.append("policy_session_mismatch")
    if expected_session is None or not observed_sessions:
        incomplete = True
        reasons.append("policy_session_identity_missing")
    elif observed_sessions != {expected_session}:
        invalid = True
    terminal_session = terminal.get("last_session_id")
    if terminal_session is not None and expected_session is not None and terminal_session != expected_session:
        invalid = True
        reasons.append("terminal_policy_session_mismatch")

    context_ref = lifecycle.get("terminal_frame_ref")
    record_ref = terminal.get("run_terminal_frame_ref")
    if context_ref is not None or record_ref is not None:
        context_ref_key, record_ref_key = _ref_key(context_ref), _ref_key(record_ref)
        if context_ref_key is None or record_ref_key is None:
            incomplete = True
            reasons.append("terminal_frame_reference_incomplete")
        elif context_ref_key != record_ref_key:
            invalid = True
            reasons.append("terminal_frame_reference_mismatch")
        if expected_session is not None and isinstance(context_ref, dict) and context_ref.get("session_id") != expected_session:
            invalid = True
            reasons.append("terminal_frame_session_mismatch")
        if isinstance(context_ref, dict) and context_ref.get("same_snapshot_persisted") is not True:
            incomplete = True
            reasons.append("terminal_frame_not_proven_same_snapshot")

    expected_capture = context.get("capture_id")
    terminal_capture = terminal.get("capture_id")
    if expected_capture is not None and terminal_capture is not None and expected_capture != terminal_capture:
        invalid = True
        reasons.append("capture_id_mismatch")
    if expected_capture is None or terminal_capture is None:
        incomplete = True
        reasons.append("capture_id_missing")

    expected_scene = settings.get("scene_id")
    expected_model = settings.get("runtime_model_fingerprint")
    summary_scene = summary.get("scene_id")
    if expected_scene is not None and summary_scene not in (None, "UNKNOWN", expected_scene):
        invalid = True
        reasons.append("metrics_scene_id_mismatch")
    snapshots = list(boundary_snapshots or [])
    for frame in task_frames or []:
        for key in ("collision_snapshot", "base_bounds_snapshot"):
            snapshot = frame.get(key)
            if isinstance(snapshot, dict):
                snapshots.append(snapshot)
    observed_scene_ids: set[Any] = set()
    observed_model_ids: set[Any] = set()
    producer_sessions: set[Any] = set()
    for snapshot in snapshots:
        capture = snapshot.get("capture_id")
        scene = snapshot.get("scenario_id")
        model = snapshot.get("runtime_model_fingerprint")
        producer_session = snapshot.get("producer_session_id")
        if producer_session is not None:
            producer_sessions.add(producer_session)
        if expected_capture is not None and capture is not None and capture != expected_capture:
            invalid = True
            reasons.append("physical_capture_id_mismatch")
        elif capture is None:
            incomplete = True
            reasons.append("physical_capture_id_missing")
        if scene is not None:
            observed_scene_ids.add(scene)
        if model is not None:
            observed_model_ids.add(model)
        if expected_scene is not None and scene is not None and scene != expected_scene:
            invalid = True
            reasons.append("physical_scene_id_mismatch")
        elif scene is None:
            incomplete = True
            reasons.append("physical_scene_id_missing")
        if expected_model is not None and model is not None and model != expected_model:
            invalid = True
            reasons.append("physical_model_fingerprint_mismatch")
        elif model is None:
            incomplete = True
            reasons.append("physical_model_fingerprint_missing")
    if not snapshots:
        incomplete = True
        reasons.append("task_physical_identity_snapshots_missing")
    if len(producer_sessions) > 1:
        invalid = True
        reasons.append("physical_producer_session_mismatch")
    if expected_scene is None or expected_model is None:
        incomplete = True
        reasons.append("expected_scene_or_model_identity_missing")

    status = "INVALID" if invalid else "UNKNOWN" if incomplete else "VALID"
    return {
        "status": status,
        "reasons": sorted(set(reasons)),
        "run_id": {"context": context_run, "terminal": terminal_run, "metrics": summary_run},
        "session_id": {"expected": expected_session, "observed_policy": sorted(observed_sessions, key=str)},
        "capture_id": {"expected": expected_capture, "terminal": terminal_capture},
        "scene_id": {"expected": expected_scene, "observed": sorted(observed_scene_ids, key=str)},
        "runtime_model_fingerprint": {"expected": expected_model, "observed": sorted(observed_model_ids, key=str)},
        "physical_producer_sessions_observed": sorted(producer_sessions, key=str),
        "terminal_frame_ref_match": (context_ref is not None and record_ref is not None and
                                      _ref_key(context_ref) is not None and
                                      _ref_key(context_ref) == _ref_key(record_ref)),
        "scope": "task-window policy session and attached physical snapshots; model/scene attribution, not proof of event coverage",
    }


def classify_safety_task_completion(summary: dict[str, Any], context: dict[str, Any],
                                    terminal: dict[str, Any] | None,
                                    identity: dict[str, Any] | None = None) -> dict[str, Any]:
    """Return separate terminal, validity, safety, composite, and closeout facts."""
    terminal = terminal or {}
    lifecycle = context.get("lifecycle", {})
    contract = context.get("result_contract", {})
    raw_context = contract.get("classification", "UNKNOWN")
    raw_record = terminal.get("run_terminal_result", "UNKNOWN")
    derived_terminal = summary.get("terminal_result", "UNKNOWN")
    validity = summary.get("record_validity", "UNKNOWN")
    identity = identity or {"status": "UNKNOWN", "reasons": ["identity_not_assessed"]}
    task_boundary = summary.get("rl_task_metric_boundary", {})
    safety = summary.get("task_window_safety", summary.get("motion_safety", {}))
    bounds = summary.get("declared_boundary_physics_observation", {})
    start_event = lifecycle.get("task_start") or lifecycle.get("policy_active_start")
    arrival = lifecycle.get("arrival_observed")
    settings = context.get("settings", {})

    raw_terminal_conflict = (raw_context != "UNKNOWN" and raw_record != "UNKNOWN" and
                             raw_context != raw_record)
    derived_conflict = isinstance(derived_terminal, str) and derived_terminal.startswith("INVALID_TERMINAL")
    terminal_contract_conflict = raw_terminal_conflict or derived_conflict
    task_started = isinstance(start_event, dict) and isinstance(start_event.get("rl_step"), int)

    nonfoot = _tri(safety.get("nonfoot_collision_failure"))
    nonfoot_coverage = safety.get("coverage_complete_for_motion_interval") is True
    if nonfoot is False and not nonfoot_coverage:
        nonfoot = "UNKNOWN"
    fall = _tri(safety.get("fall"))
    fall_coverage = safety.get("coverage_complete_for_motion_interval") is True
    if fall is False and not fall_coverage:
        fall = "UNKNOWN"
    boundary_status = bounds.get("status")
    if boundary_status in {"BASE_LINK_IN_BOUNDS_PHYSICS_STEPS_OBSERVED",
                           "NO_BASE_LINK_OUTSIDE_STEP_AFTER_TASK_START_OBSERVED"}:
        out_of_bounds: bool | str = False
    elif boundary_status == "BASE_LINK_OUT_OF_BOUNDS_PHYSICS_STEP_OBSERVED":
        out_of_bounds = True
    else:
        out_of_bounds = "UNKNOWN"
    system_abort_value, external_safety_findings = _system_safety_abort_dimension(
        raw_context, context, terminal, task_started)

    feet = {
        "contact_count": safety.get("foot_contact_count", "UNKNOWN"),
        "impact_count": safety.get("foot_impact_count", "UNKNOWN"),
        "contact_events": safety.get("foot_contact_events", []),
        "impact_events": safety.get("foot_impact_events", []),
        "impact_unknown_overlaps_motion": safety.get("foot_impact_unknown_overlaps_motion", "UNKNOWN"),
    }
    dimensions = {
        "nonfoot_obstacle_collision": {"value": nonfoot, "coverage_complete": nonfoot_coverage,
                                        "reason": safety.get("coverage_reason")},
        "fall": {"value": fall, "coverage_complete": fall_coverage,
                 "basis": safety.get("fall_result_basis"),
                 "timing_conflicts": safety.get("fall_event_timing_conflicts", [])},
        "system_safety_abort": {"value": system_abort_value,
                                 "external_event_evaluation": external_safety_findings},
        "base_link_out_of_bounds": {"value": out_of_bounds, "status": boundary_status,
                                     "scope": bounds.get("scope")},
        "foot_events": feet,
    }

    reasons: list[str] = []
    result: bool | str = "UNKNOWN"
    record_state = "INVALID" if validity == "INVALID" else "VALID" if validity == "VALID" else "UNKNOWN"
    if not task_started:
        reasons.append("no_valid_rl_task_start_boundary")
    elif record_state != "VALID":
        reasons.append("record_invalid_or_validity_unknown")
    elif identity.get("status") == "INVALID":
        reasons.append("task_source_identity_" + str(identity.get("status", "UNKNOWN")).lower())
    elif raw_terminal_conflict:
        reasons.append("context_and_terminal_raw_results_conflict")
    elif raw_context in TASK_NONARRIVAL_TERMINALS:
        result = False
        reasons.append("valid_rl_task_did_not_finish_as_arrived_within_task_window")
    elif raw_context in TASK_INCOMPLETE_TERMINALS:
        reasons.append("task_window_or_terminal_is_incomplete_not_a_confirmed_task_failure")
    elif raw_context == "ARRIVED":
        if identity.get("status") != "VALID":
            reasons.append("task_source_identity_" + str(identity.get("status", "UNKNOWN")).lower())
        else:
            arrival_distance = arrival.get("distance_m") if isinstance(arrival, dict) else None
            arrival_limit = arrival.get("threshold_m") if isinstance(arrival, dict) else settings.get("arrival_threshold_m")
            arrival_time = arrival.get("sim_time_s") if isinstance(arrival, dict) else None
            task_start_time = start_event.get("sim_time_s") if isinstance(start_event, dict) else None
            arrival_session = arrival.get("session_id") if isinstance(arrival, dict) else None
            arrival_cycle = arrival.get("rl_step") if isinstance(arrival, dict) else None
            terminal_ref = lifecycle.get("terminal_frame_ref")
            arrival_identity_ok = (isinstance(terminal_ref, dict) and
                                   arrival_session == start_event.get("session_id") and
                                   arrival_session == terminal_ref.get("session_id") and
                                   arrival_cycle == terminal_ref.get("rl_step"))
            if (not isinstance(arrival_distance, (int, float)) or not math.isfinite(float(arrival_distance)) or
                    not isinstance(arrival_limit, (int, float)) or not math.isfinite(float(arrival_limit))):
                reasons.append("arrival_distance_or_threshold_missing")
            elif arrival_distance > arrival_limit:
                result = False
                terminal_contract_conflict = True
                reasons.append("ARRIVED_terminal_outside_recorded_distance_threshold")
            elif not arrival_identity_ok:
                reasons.append("arrival_event_not_bound_to_terminal_cycle")
            elif (not isinstance(arrival_time, (int, float)) or not math.isfinite(float(arrival_time)) or
                  not isinstance(task_start_time, (int, float)) or not math.isfinite(float(task_start_time))):
                reasons.append("arrival_or_task_start_sim_time_missing")
            else:
                elapsed = arrival_time - task_start_time
                limit_s = settings.get("diagnostic_sim_limit_after_motion_s")
                if not isinstance(limit_s, (int, float)) or not math.isfinite(float(limit_s)):
                    reasons.append("fixed_task_deadline_missing")
                elif elapsed < 0 or elapsed > limit_s:
                    result = False
                    terminal_contract_conflict = True
                    reasons.append("ARRIVED_terminal_outside_fixed_task_deadline")
                elif nonfoot is True or fall is True or system_abort_value is True or out_of_bounds is True:
                    result = False
                    reasons.append("confirmed_task_window_safety_veto")
                    terminal_contract_conflict = True
                    reasons.append("ARRIVED_conflicts_with_confirmed_safety_veto")
                elif any(value == "UNKNOWN" for value in (nonfoot, fall, system_abort_value, out_of_bounds)):
                    reasons.append("one_or_more_required_task_safety_dimensions_unknown")
                elif (task_boundary.get("cycle_identity_complete") is not True or
                      task_boundary.get("terminal_frame_payload_complete") is not True or
                      task_boundary.get("start_cycle") is None or task_boundary.get("end_cycle") is None):
                    reasons.append("task_cycle_or_terminal_payload_boundary_incomplete")
                elif identity.get("terminal_frame_ref_match") is not True:
                    reasons.append("terminal_frame_reference_not_exactly_matched")
                else:
                    result = True
                    reasons.append("ARRIVED_within_deadline_and_all_required_task_safety_dimensions_clear")
    if raw_context == "ARRIVED" and result == "UNKNOWN":
        if nonfoot == "UNKNOWN":
            reasons.append("nonfoot_collision_absence_not_supported_by_complete_task_window_coverage")
        if fall == "UNKNOWN":
            reasons.append("fall_absence_not_supported_by_complete_task_window_history")
        if out_of_bounds == "UNKNOWN":
            reasons.append("base_link_bounds_not_supported_by_complete_physics_coverage")
    elif raw_context != "ARRIVED" and raw_context not in TASK_NONARRIVAL_TERMINALS:
        reasons.append("raw_task_terminal_not_in_supported_task_result_set")

    if derived_conflict:
        reasons.append("production_terminal_analysis_reports_contract_conflict")
    standing = lifecycle.get("standing_observation")
    closeout = {
        "standing_observation": standing if standing is not None else "UNKNOWN",
        "normal_shutdown": terminal.get("normal_shutdown", "UNKNOWN"),
        "shutdown_complete": terminal.get("shutdown_complete", "UNKNOWN"),
        "process_exit_code": terminal.get("process_exit_code", "UNKNOWN"),
        "cleanup_events": terminal.get("cleanup_events", []),
        "cleanup_processes": lifecycle.get("cleanup_processes", []),
        "closeout_errors": lifecycle.get("closeout_errors", []),
        "scope": "post-terminal standing/cleanup; excluded from motion safety classification",
    }
    return {
        "status": result,
        "status_label": ("CONFIRMED_SAFE_TASK_COMPLETE" if result is True else
                         "NOT_SAFE_TASK_COMPLETE" if result is False else "UNKNOWN"),
        "reasons": reasons,
        "raw_terminal": {"context": raw_context, "record": raw_record, "derived": derived_terminal,
                         "contract_conflict": terminal_contract_conflict},
        "record_validity": {"status": record_state,
                             "reasons": summary.get("record_validity_reasons", [])},
        "task_window": {"definition": task_boundary.get("definition"),
                        "start_cycle": task_boundary.get("start_cycle"),
                        "end_cycle": task_boundary.get("end_cycle"),
                        "cycle_identity_complete": task_boundary.get("cycle_identity_complete"),
                        "terminal_frame_payload_complete": task_boundary.get("terminal_frame_payload_complete")},
        "task_source_identity": identity,
        "task_safety_dimensions": dimensions,
        "standing_and_cleanup": closeout,
        "muJoCo_command_consumption": summary.get("command_consumption", "UNKNOWN"),
        "scope": "formal task-window metric; True means arrival by fixed deadline and all required task safety dimensions are supported clear; foot events reported separately",
    }
