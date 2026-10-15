#!/usr/bin/env python3
"""Targeted B1 checks for safety events with an invalid policy start clock."""
from __future__ import annotations

from s1_analyze_run import (mode_edge_coverage, motion_safety,
                            physical_base_bounds_window, summarize_policy_cadence)


def snapshot(step: int, *, capture: str = "capture-a", sim_time: float | None = None,
             collisions: list[dict] | None = None, falls: list[dict] | None = None) -> dict:
    return {
        "status": "LIVE", "version": 5, "capture_id": capture,
        "sequence": step * 2, "monotonic_ns": step * 1_000_000,
        "physics_step": step, "sim_time": sim_time if sim_time is not None else step * 0.002,
        "physics_coverage_complete": True, "collision_history_overflow": False,
        "fall_history_overflow": False, "event_latches_complete": True,
        "collision_history": collisions or [], "fall_history": falls or [],
        "unknown_contact_steps": 0, "invalid_posture_steps": 0,
    }


def run() -> None:
    bounds_base = {"status": "LIVE", "version": 1, "sequence": 10,
        "producer_session_id": 77, "physics_coverage_complete": 1, "bounds_valid": 1,
        "physics_step": 100, "outside_physics_steps": 0, "currently_outside": 0,
        "xmin": -0.5, "xmax": 8.0, "ymin": -3.0, "ymax": 3.0,
        "scenario_id": "m1dod01", "capture_id": "p1-10-capture-" + "1" * 32,
        "runtime_model_fingerprint": "a" * 64, "sim_time": 2.0,
        "first_outside_sim_time": 0.0, "last_outside_sim_time": 0.0}
    bounds_end = {**bounds_base, "sequence": 30, "physics_step": 200, "sim_time": 4.0}
    assert physical_base_bounds_window(bounds_base, bounds_end, 2.1, 3.9)["status"] == \
        "BASE_LINK_IN_BOUNDS_PHYSICS_STEPS_OBSERVED"
    outside_end = {**bounds_end, "outside_physics_steps": 4,
                   "first_outside_sim_time": 3.0, "last_outside_sim_time": 3.2}
    assert physical_base_bounds_window(bounds_base, outside_end, 2.1, 3.9)["status"] == \
        "BASE_LINK_OUT_OF_BOUNDS_PHYSICS_STEP_OBSERVED"
    assert physical_base_bounds_window(bounds_base,
        {**outside_end, "physics_coverage_complete": 0}, 2.1, 3.9)["status"] == "UNKNOWN"
    assert physical_base_bounds_window(bounds_base,
        {**bounds_end, "producer_session_id": 78}, 2.1, 3.9)["status"] == "UNKNOWN"
    assert physical_base_bounds_window(bounds_base, bounds_end, None, None)["status"] == \
        "BASE_LINK_IN_BOUNDS_PHYSICS_STEPS_OBSERVED"

    cadence_frames = [
        {"session_id": 1, "rl_step": 0, "monotonic_ns": 0},
        {"session_id": 1, "rl_step": 1, "monotonic_ns": 20_000_000},
        {"session_id": 1, "rl_step": 3, "monotonic_ns": 60_000_000},
        {"session_id": 1, "rl_step": 4, "monotonic_ns": 80_000_000},
    ]
    cadence = summarize_policy_cadence(cadence_frames)
    assert abs(cadence["policy_frequency_hz_overall"] - 50.0) < 1e-9
    assert cadence["missing_policy_cycles_between_saved_frames"] == 1
    assert cadence["adjacent_consecutive_cycle_period_s"] == [0.02, 0.02]
    assert mode_edge_coverage(cadence_frames, True) == "partial_or_unknown"
    assert mode_edge_coverage(cadence_frames[:2], True) == "complete"

    collision = {"start_physics_step": 14, "end_physics_step": 16,
                 "start_sim_time": 0.028, "end_sim_time": 0.032,
                 "obstacle_geom_name": "ppt_obstacle_01"}
    fall = {"start_physics_step": 13, "end_physics_step": 17,
            "start_sim_time": 0.026, "end_sim_time": 0.034,
            "confirmed": True}
    first = snapshot(10)
    terminal = snapshot(20, collisions=[collision], falls=[fall])
    frames = [{"collision_snapshot": first}, {"collision_snapshot": terminal}]

    bounded = motion_safety(frames, None, 0.04,
                            start_boundary_snapshot=first,
                            end_boundary_snapshot=terminal)
    assert bounded["collision"] is True
    assert bounded["fall"] is True
    assert bounded["coverage_complete_for_motion_interval"] is False
    assert bounded["coverage_evidence"]["event_window_basis"].startswith("identity_bound_physics_steps")

    no_event = motion_safety(
        [{"collision_snapshot": snapshot(10)}, {"collision_snapshot": snapshot(20)}],
        None, 0.04,
        start_boundary_snapshot=snapshot(10), end_boundary_snapshot=snapshot(20))
    assert no_event["collision"] == "UNKNOWN"
    assert no_event["fall"] == "UNKNOWN"

    mismatched = motion_safety(
        frames, None, 0.04,
        start_boundary_snapshot=snapshot(10, capture="other-capture"),
        end_boundary_snapshot=terminal)
    assert mismatched["collision"] == "UNKNOWN"
    assert mismatched["fall"] == "UNKNOWN"
    assert mismatched["coverage_evidence"]["physics_step_window_identity_bound"] is False

    timed = motion_safety(frames, 0.02, 0.04,
                          start_boundary_snapshot=first,
                          end_boundary_snapshot=terminal)
    assert timed["collision"] is True
    assert timed["fall"] is True
    assert timed["coverage_complete_for_motion_interval"] is True

    # A stale post-terminal snapshot is not part of a completed task window.
    # Passing it into the task analysis would conservatively make coverage
    # UNKNOWN; the production calculate() caller must scope inputs by the
    # persisted task cycle boundary before reaching motion_safety().
    task_start = snapshot(100, sim_time=1.0)
    task_terminal = snapshot(120, sim_time=1.04)
    stale_closeout = snapshot(140, sim_time=2.0)
    stale_closeout["status"] = "STALE"
    task_only = motion_safety(
        [{"collision_snapshot": task_start}, {"collision_snapshot": task_terminal}],
        1.0, 1.04, end_boundary_snapshot=task_terminal,
        start_boundary_snapshot=task_start)
    contaminated = motion_safety(
        [{"collision_snapshot": task_start}, {"collision_snapshot": task_terminal},
         {"collision_snapshot": stale_closeout}],
        1.0, 1.04, end_boundary_snapshot=task_terminal,
        start_boundary_snapshot=task_start)
    assert task_only["collision"] is False and task_only["fall"] is False
    assert task_only["coverage_complete_for_motion_interval"] is True
    assert contaminated["collision"] == "UNKNOWN" and contaminated["fall"] == "UNKNOWN"
    assert "task_window_contains_non_live_physical_snapshot" in contaminated["coverage_reason"]

    print("B1 motion-safety boundary checks: PASS (including stale closeout isolation case)")


if __name__ == "__main__":
    run()
