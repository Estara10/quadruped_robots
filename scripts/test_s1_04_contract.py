#!/usr/bin/env python3
"""Small offline checks for S1-04 result priority, gaps, and dwell accounting."""
from __future__ import annotations

import dataclasses
import json
import math
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_record  # noqa: E402
from abs_rt_frame import FrameStatus, classify_frame  # noqa: E402
from abs_scene import resolve_scene  # noqa: E402
import s1_run_diagnostic  # noqa: E402
from s1_run_diagnostic import FrameCapture  # noqa: E402
from s1_analyze_run import calculate, motion_safety  # noqa: E402
from test_run_record import fixture, pack_frame  # noqa: E402


def _capture(path: Path, run_id: str, capture_id: str, run_start_ns: int = 0) -> FrameCapture:
    """Build the production recorder with the current required source/config binding."""
    return FrameCapture(path, run_id, capture_id, run_start_ns,
                        resolve_scene("scene_obstacle.xml"), candidate_mode=1,
                        entry_threshold=-0.05, exit_threshold=-0.08, hold_duration_s=0.0)


def _fall_snapshot(sequence: int, physics_step: int, sim_time: float, event=None,
                   *, overflow: bool = False, capture_id: str = "fall-test-capture") -> dict:
    return {
        "status": "LIVE", "version": 5, "capture_id": capture_id,
        "sequence": sequence, "physics_step": physics_step, "sim_time": sim_time,
        "physics_coverage_complete": True, "event_latches_complete": True,
        "collision_history_overflow": False, "fall_history_overflow": overflow,
        "fall_history": [event] if event else [], "collision_history": [],
        "unknown_contact_steps": 0, "unknown_contact_first_sim_time": 0.0,
        "unknown_contact_last_sim_time": 0.0, "invalid_posture_steps": 0,
        "first_invalid_posture_sim_time": 0.0, "last_invalid_posture_sim_time": 0.0,
        "foot_identity_valid": True, "foot_contact_history_overflow": False,
        "foot_impact_history_overflow": False,
    }


def _fall_window(start_snapshot: dict, end_snapshot: dict, end_s: float = 10.1) -> dict:
    frames = [{"collision_snapshot": start_snapshot}, {"collision_snapshot": end_snapshot}]
    return motion_safety(frames, 10.0, end_s, terminal_snapshot=end_snapshot,
                         start_boundary_snapshot=start_snapshot,
                         end_boundary_snapshot=end_snapshot)


def test_fall_event_time_is_independent_of_later_snapshot_publication() -> None:
    start = _fall_snapshot(2, 100, 10.0)
    event = {"start_physics_step": 105, "start_sim_time": 10.05,
             "end_physics_step": 112, "end_sim_time": 10.12,
             "confirmed": True, "confirmed_physics_step": 108,
             "confirmed_sim_time": 10.08, "closed": False}
    end = _fall_snapshot(4, 112, 10.12, event)
    result = _fall_window(start, end)
    assert result["coverage_complete_for_motion_interval"] is True
    assert result["fall"] is True, result
    assert result["fall_result_basis"] == "confirmed_fall_event_time_and_physics_step_within_motion_window"
    assert result["fall_lifecycle_assessments"][0]["status_at_task_terminal"] == "CONFIRMED_BY_TASK_TERMINAL"
    assert result["fall_lifecycle_assessments"][0]["event_last_abnormal_after_task_terminal"] is True


def test_fall_confirmation_after_terminal_is_unknown_not_backdated() -> None:
    start = _fall_snapshot(2, 100, 10.0)
    event = {"start_physics_step": 105, "start_sim_time": 10.05,
             "end_physics_step": 112, "end_sim_time": 10.12,
             "confirmed": True, "confirmed_physics_step": 111,
             "confirmed_sim_time": 10.11, "closed": False}
    result = _fall_window(start, _fall_snapshot(4, 112, 10.12, event))
    assert result["fall"] == "UNKNOWN", result
    assert result["fall_lifecycle_assessments"][0]["status_at_task_terminal"] == "CONFIRMATION_AFTER_TASK_TERMINAL"

    contradictory = {**event, "confirmed_physics_step": 108, "confirmed_sim_time": 10.11}
    result = _fall_window(start, _fall_snapshot(4, 112, 10.12, contradictory))
    assert result["fall"] == "UNKNOWN", result
    assert result["fall_result_basis"] == "fall_event_confirmation_time_step_conflict"


def test_fall_candidate_closed_only_after_terminal_remains_unknown() -> None:
    start = _fall_snapshot(2, 100, 10.0)
    event = {"start_physics_step": 105, "start_sim_time": 10.05,
             "end_physics_step": 109, "end_sim_time": 10.09,
             "confirmed": False, "confirmed_physics_step": 0,
             "confirmed_sim_time": 0.0, "closed": True}
    result = _fall_window(start, _fall_snapshot(4, 112, 10.12, event))
    assert result["fall"] == "UNKNOWN", result
    assert result["fall_lifecycle_assessments"][0]["status_at_task_terminal"] == "ACTIVE_OR_CLOSE_TIME_UNKNOWN_AT_TERMINAL"


def test_closed_preparation_fall_is_excluded_but_cross_boundary_confirmation_counts() -> None:
    prep = {"start_physics_step": 80, "start_sim_time": 9.8,
            "end_physics_step": 90, "end_sim_time": 9.9,
            "confirmed": False, "confirmed_physics_step": 0,
            "confirmed_sim_time": 0.0, "closed": True}
    result = _fall_window(_fall_snapshot(2, 100, 10.0, prep),
                          _fall_snapshot(4, 112, 10.12, prep))
    assert result["fall"] is False, result
    assert result["fall_events"] == []

    continuing = {"start_physics_step": 95, "start_sim_time": 9.95,
                  "end_physics_step": 108, "end_sim_time": 10.08,
                  "confirmed": True, "confirmed_physics_step": 106,
                  "confirmed_sim_time": 10.06, "closed": False}
    initial = {**continuing, "confirmed": False, "confirmed_physics_step": 0,
               "confirmed_sim_time": 0.0}
    result = _fall_window(_fall_snapshot(2, 100, 10.0, initial),
                          _fall_snapshot(4, 112, 10.12, continuing))
    assert result["fall"] is True, result


def test_fall_history_overflow_or_identity_conflict_does_not_prove_result() -> None:
    start = _fall_snapshot(2, 100, 10.0)
    event = {"start_physics_step": 105, "start_sim_time": 10.05,
             "end_physics_step": 108, "end_sim_time": 10.08,
             "confirmed": True, "confirmed_physics_step": 106,
             "confirmed_sim_time": 10.06, "closed": False}
    assert _fall_window(start, _fall_snapshot(4, 112, 10.12, event, overflow=True))["fall"] == "UNKNOWN"
    conflict = _fall_snapshot(4, 112, 10.12, event, capture_id="other-capture")
    assert _fall_window(start, conflict)["fall"] == "UNKNOWN"

    invalid = _fall_snapshot(4, 112, 10.12, event)
    invalid["invalid_posture_steps"] = 1
    invalid["first_invalid_posture_sim_time"] = 10.07
    invalid["last_invalid_posture_sim_time"] = 10.07
    assert _fall_window(start, invalid)["fall"] == "UNKNOWN"


def test_external_safety_event_has_terminal_priority() -> None:
    assert run_record._compute_terminal(exit_code=0, forced=False, shutdown_complete=True,
                                        safety_fault_seen=True, frames_observed=1)[1] == "SAFETY_FAULT"
    assert run_record._compute_terminal(exit_code=0, forced=False, shutdown_complete=True,
                                        safety_fault_seen=False, frames_observed=1)[1] == "FRAMES_ENDED_RC0"
    original = run_record.read_collision_snapshot
    run_record.read_collision_snapshot = lambda: b""
    try:
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "run.jsonl")
            recorder = run_record.RunRecordRecorder(path)
            recorder.start()
            frame = fixture(monotonic_ns=time.monotonic_ns())
            recorder.record_snapshot(pack_frame(frame), now_ns=frame.monotonic_ns + 1)
            event = {"event": "FROZEN_POSITION_PASSIVE", "source": "controller log",
                     "message": "position frozen; forcing PASSIVE", "observed_monotonic_ns": time.monotonic_ns()}
            terminal = recorder.finalize({"exit_code": 0, "forced_termination": False,
                "shutdown_complete": True, "shutdown_request_source": "offline fixture",
                "external_safety_events": [event]})
            assert terminal["termination_reason"] == "SAFETY_FAULT"
            assert terminal["external_safety_events"] == [event]
            assert run_record.summarize_record(path)["record_validity"] == "VALID"
    finally:
        run_record.read_collision_snapshot = original


def test_policy_safety_boundary_keeps_fault_frame_separate_from_last_policy_frame() -> None:
    line = ("[StateRL] [ABS-CONTRACT] event=detected clock=steady_clock_ns detection_ns=12345 "
            "session_id=71 policy_cycle=9 ray_reason=read_time_budget_last_odd_sequence_writer_in_progress "
            "sequence_first=3 sequence_last=17 attempts=16 read_duration_ns=249001 "
            "header_observed=0 stamp_observed=0 snapshot_consistent=0 ray_stamp_ns=0 "
            "validation_clock_observed=0 ray_validation_now_ns=0 ray_age_ns=0 "
            "stage=ray transition_request=PASSIVE")
    parsed = s1_run_diagnostic.parse_policy_safety_veto(line)
    assert parsed is not None
    assert parsed["detection_monotonic_ns"] == 12345
    assert parsed["session_id"] == 71 and parsed["policy_cycle"] == 9
    assert parsed["ray_reason"] == "read_time_budget_last_odd_sequence_writer_in_progress"
    assert parsed["sequence_first"] == 3 and parsed["sequence_last"] == 17
    assert parsed["attempts"] == 16 and parsed["read_duration_ns"] == 249001

    strategy = dataclasses.replace(fixture(sequence=2, rl_step=8, session_id=71,
        monotonic_ns=time.monotonic_ns()), policy_state=0, rl_entered=1, rl_active=1)
    fault = dataclasses.replace(fixture(sequence=4, rl_step=9, session_id=71,
        monotonic_ns=strategy.monotonic_ns + 20_000_000), policy_state=2,
        rl_entered=1, rl_active=0, safety_faulted=True)
    assert s1_run_diagnostic.same_cycle_fault_frame(fault, 71, 9)
    assert not s1_run_diagnostic.same_cycle_fault_frame(strategy, 71, 9)
    assert not s1_run_diagnostic.same_cycle_fault_frame(fault, 72, 9)

    original = run_record.read_collision_snapshot
    run_record.read_collision_snapshot = lambda: b""
    try:
        with tempfile.TemporaryDirectory() as tmp:
            capture = _capture(Path(tmp) / "runtime.jsonl", "fault-boundary",
                               "p1-10-capture-" + "a" * 32)
            capture.start()
            for frame in (strategy, fault):
                raw = pack_frame(frame)
                status, observed = classify_frame(raw, frame.monotonic_ns + 1)
                result = capture.observe(raw, status, observed, frame.monotonic_ns + 1)
                assert result["persisted"] is True
            assert capture.last_frame_metadata["policy_state"] == "FAULTED"
            assert capture.last_active_frame_metadata["rl_step"] == 8
            assert capture.last_active_frame_metadata["policy_state"] == "AGILE"
    finally:
        run_record.read_collision_snapshot = original


def test_step_gap_is_not_bridged_into_recovery_dwell() -> None:
    base_ns = time.monotonic_ns()
    frames = [
        dataclasses.replace(fixture(sequence=2, rl_step=1, session_id=71, monotonic_ns=base_ns),
                            policy_state=0, mode_before=0, sim_time_s=10.0, world_pose=(0.0, 0.0, 0.0)),
        dataclasses.replace(fixture(sequence=4, rl_step=2, session_id=71, monotonic_ns=base_ns + 20_000_000),
                            policy_state=1, mode_before=0, policy_mode_changed=1,
                            mode_change_ns=base_ns + 20_000_000, action_source=2,
                            sim_time_s=10.2, world_pose=(0.1, 0.0, 0.0)),
        dataclasses.replace(fixture(sequence=6, rl_step=4, session_id=71, monotonic_ns=base_ns + 60_000_000),
                            policy_state=1, mode_before=1, sim_time_s=10.6, world_pose=(0.3, 0.0, 0.0)),
        dataclasses.replace(fixture(sequence=8, rl_step=5, session_id=71, monotonic_ns=base_ns + 100_000_000),
                            policy_state=0, mode_before=1, policy_mode_changed=1,
                            mode_change_ns=base_ns + 100_000_000, action_source=1,
                            sim_time_s=11.0, world_pose=(0.5, 0.0, 0.0)),
    ]
    original = run_record.read_collision_snapshot
    run_record.read_collision_snapshot = lambda: b""
    try:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            record = root / "runtime_record.jsonl"
            recorder = run_record.RunRecordRecorder(str(record))
            recorder.start()
            for frame in frames:
                recorder.record_snapshot(pack_frame(frame), now_ns=frame.monotonic_ns + 1)
            recorder.finalize({"exit_code": 0, "forced_termination": False,
                               "shutdown_complete": True, "shutdown_request_source": "offline fixture"})
            context = root / "context.json"
            context.write_text(json.dumps({"run_id": "fixture", "settings": {"goal_world_m": [4.0, 0.0]},
                                           "lifecycle": {"motion_start_observed_sim_time_s": 10.2,
                                                         "movement_end_sim_time_s": 11.0,
                                                         "terminal_frame_ref": {"session_id": 71, "rl_step": 5, "sim_time_s": 11.0}},
                                           "result_contract": {"classification": "fixture"}}), encoding="utf-8")
            result = calculate(record, context)
            assert result["record_validity"] == "VALID", result["record_validity_reasons"]
            assert result["frame_count"] == 4
            assert result["rl_step_gaps"] == 1
            assert result["mode_transitions"] == 2
            # The current primary window starts at the first active policy cycle,
            # not at the historical 0.10 m displacement threshold. Step 1->2
            # and step 4->5 are the only valid adjacent intervals: 0.2 + 0.4 s.
            assert math.isclose(result["valid_motion_duration_s"], 0.6)
            assert math.isclose(result["recovery_sim_duration_s"], 0.4)
            assert math.isclose(result["recovery_share_valid_motion_time"], 2.0 / 3.0)
            assert math.isclose(result["legacy_displacement_window_metrics"]["valid_continuous_s"], 0.4)
            assert result["legacy_displacement_window_scope"].startswith("historical auxiliary window")
            assert any(item["censored"] for item in result["recovery_intervals"])
    finally:
        run_record.read_collision_snapshot = original


def test_exit_cycle_closes_recovery_and_cleanup_tail_is_excluded() -> None:
    base_ns = time.monotonic_ns()
    frames = [
        dataclasses.replace(fixture(sequence=20+2*i, rl_step=i, session_id=91, monotonic_ns=base_ns+i*100_000_000),
            policy_state=mode, mode_before=before, policy_mode_changed=int(mode != before),
            mode_change_ns=base_ns+i*100_000_000 if mode != before else 0,
            sim_time_s=10.0+i*0.1, world_pose=(i*0.1, 0.0, 0.0))
        for i, (before, mode) in enumerate([(0,0), (0,1), (1,0), (0,1)])
    ]
    original = run_record.read_collision_snapshot
    run_record.read_collision_snapshot = lambda: b""
    try:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); record = root / "runtime_record.jsonl"
            recorder = run_record.RunRecordRecorder(str(record)); recorder.start()
            for frame in frames:
                recorder.record_snapshot(pack_frame(frame), now_ns=frame.monotonic_ns+1)
            terminal = recorder.finalize({"exit_code": 0, "forced_termination": False,
                "shutdown_complete": True, "shutdown_request_source": "offline fixture",
                "run_terminal_result": "DIAGNOSTIC_SIM_TIME_LIMIT", "run_terminal_monotonic_ns": base_ns+250_000_000,
                "cleanup_events": [{"phase": "cleanup", "event": "LATE_SAFETY_EVENT"}]})
            assert terminal["termination_reason"] == "FRAMES_ENDED_RC0"
            assert terminal["run_terminal_result"] == "DIAGNOSTIC_SIM_TIME_LIMIT"
            context = root / "context.json"
            context.write_text(json.dumps({"run_id":"fixture", "settings":{"goal_world_m":[4.0,0.0]},
                "lifecycle":{"motion_start_observed_sim_time_s":10.1, "movement_end_sim_time_s":10.2,
                    "terminal_frame_ref":{"session_id":91,"rl_step":2,"sim_time_s":10.2}},
                "result_contract":{"classification":"DIAGNOSTIC_SIM_TIME_LIMIT"}}), encoding="utf-8")
            result = calculate(record, context)
            assert result["record_validity"] == "VALID", result.get("record_validity_reasons")
            assert result["terminal_result"] == "DIAGNOSTIC_SIM_TIME_LIMIT"
            assert result["mode_transitions"] == 2
            assert result["recovery_to_agile"] == 1
            assert math.isclose(result["recovery_intervals"][0]["duration_s"], 0.1)
            assert result["cleanup_events"][0]["event"] == "LATE_SAFETY_EVENT"
            assert result["motion_metric_time_boundary"]["end_cycle"] == 2
            assert result["motion_metric_time_boundary"]["terminal_frame_payload_complete"] is True
            assert math.isclose(result["path_length_m"], 0.2, abs_tol=1e-6), result
            assert math.isclose(result["legacy_displacement_window_metrics"]["path_length_m"], 0.1, abs_tol=1e-6)
    finally:
        run_record.read_collision_snapshot = original


def test_recovery_interval_ends_on_first_agile_cycle() -> None:
    base_ns = time.monotonic_ns()
    frames = [
        dataclasses.replace(fixture(sequence=30+2*i, rl_step=i, session_id=92, monotonic_ns=base_ns+i*100_000_000),
            policy_state=mode, mode_before=before, policy_mode_changed=int(mode != before),
            mode_change_ns=base_ns+i*100_000_000 if mode != before else 0,
            sim_time_s=20.0+i*0.1, world_pose=(0.1*i, 0.0, 0.0))
        for i, (before, mode) in enumerate([(0,0), (0,1), (1,1), (1,0)])
    ]
    original = run_record.read_collision_snapshot
    run_record.read_collision_snapshot = lambda: b""
    try:
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); record=root/"runtime_record.jsonl"
            recorder=run_record.RunRecordRecorder(str(record)); recorder.start()
            for frame in frames: recorder.record_snapshot(pack_frame(frame), now_ns=frame.monotonic_ns+1)
            recorder.finalize({"exit_code":0,"forced_termination":False,"shutdown_complete":True,
                               "shutdown_request_source":"offline fixture"})
            context=root/"context.json"
            context.write_text(json.dumps({"run_id":"fixture","settings":{"goal_world_m":[4.0,0.0]},
                "lifecycle":{"motion_start_observed_sim_time_s":20.0,"movement_end_sim_time_s":20.3,
                    "terminal_frame_ref":{"session_id":92,"rl_step":3,"sim_time_s":20.3}},
                "result_contract":{"classification":"fixture"}}),encoding="utf-8")
            result=calculate(record,context)
            assert result["recovery_intervals"][0]["exit_cycle"] == 3
            assert math.isclose(result["recovery_intervals"][0]["duration_s"],0.2)
            assert math.isclose(result["recovery_sim_duration_s"],0.2)
    finally:
        run_record.read_collision_snapshot = original


def test_missing_motion_interval_keeps_motion_metrics_unknown() -> None:
    base_ns = time.monotonic_ns()
    frame = dataclasses.replace(fixture(sequence=60, rl_step=1, session_id=93, monotonic_ns=base_ns),
                                policy_state=1, mode_before=0, policy_mode_changed=1,
                                mode_change_ns=base_ns, sim_time_s=30.0)
    original = run_record.read_collision_snapshot
    run_record.read_collision_snapshot = lambda: b""
    try:
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); record=root/"runtime_record.jsonl"
            recorder=run_record.RunRecordRecorder(str(record)); recorder.start()
            recorder.record_snapshot(pack_frame(frame), now_ns=frame.monotonic_ns+1)
            recorder.finalize({"exit_code":0,"forced_termination":False,"shutdown_complete":True,
                               "shutdown_request_source":"offline fixture"})
            context=root/"context.json"
            context.write_text(json.dumps({"run_id":"fixture","settings":{"goal_world_m":[4.0,0.0]},
                "lifecycle":{"run_terminal_event":"SYSTEM_SAFETY_ABORT"},
                "result_contract":{"classification":"SYSTEM_SAFETY_ABORT"}}),encoding="utf-8")
            result=calculate(record,context)
            assert result["terminal_result"] == "SYSTEM_SAFETY_ABORT"
            assert result["mode_transitions"] is None
            assert result["agile_to_recovery"] is None
            assert result["recovery_share_valid_motion_time"] is None
    finally:
        run_record.read_collision_snapshot = original


def test_terminal_snapshot_is_synchronously_persisted_once_with_full_payload() -> None:
    base_ns = time.monotonic_ns()
    terminal = dataclasses.replace(fixture(sequence=82, rl_step=9, session_id=771,
        monotonic_ns=base_ns, policy_state=0, ra_value=-0.12),
        mode_before=0, sim_time_s=4.2, world_pose=(0.6, 0.1, 0.0), exit_threshold=-0.05)
    raw = pack_frame(terminal)
    sampled_ns = base_ns + 1
    status, observed = classify_frame(raw, sampled_ns)
    assert status is FrameStatus.LIVE and observed is not None
    original = run_record.read_collision_snapshot
    run_record.read_collision_snapshot = lambda: b""
    try:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "runtime_record.jsonl"
            capture = FrameCapture(path, "terminal-capture", "p1-10-capture-" + "a" * 32, 0,
                resolve_scene("scene_obstacle.xml"), candidate_mode=1,
                entry_threshold=-0.05, exit_threshold=-0.05, hold_duration_s=0.0)
            capture.start()
            # The sampling worker has not seen this cycle; the supervisor's
            # terminal-triggering raw snapshot must still be written now.
            first = capture.observe(raw, status, observed, sampled_ns)
            duplicate = capture.observe(raw, status, observed, sampled_ns)
            assert first["status"] == "persisted" and first["persisted"] is True
            assert duplicate["status"] == "already_persisted" and duplicate["persisted"] is True
            capture.stop.set()
            capture.recorder.stop_sampling()
            capture.recorder.finalize({"exit_code": 0, "forced_termination": False,
                "shutdown_complete": True, "shutdown_request_source": "offline terminal capture test"})
            data = run_record.load_record(str(path))
            assert len(data.frames) == 1
            payload = data.frames[0]["payload"]
            assert (payload["session_id"], payload["source_sequence"], payload["rl_step"]) == (771, 82, 9)
            assert math.isclose(payload["ra_value"], -0.12, abs_tol=1e-6)
            assert payload["mode_before"] == 0 and payload["mode_after"] == 0
            assert payload["action_source"] == observed.action_source
            assert payload["action_raw"] == list(observed.action_raw)
            assert payload["world_pose"] == list(observed.world_pose)
            assert payload["sim_time_s"] == observed.sim_time_s
            assert capture.unique_frames == 1

            session_path = Path(tmp) / "session-mismatch.jsonl"
            session_capture = FrameCapture(session_path, "session-capture", "p1-10-capture-" + "b" * 32, 0,
                resolve_scene("scene_obstacle.xml"), candidate_mode=1,
                entry_threshold=-0.05, exit_threshold=-0.05, hold_duration_s=0.0)
            session_capture.start()
            session_capture.observe(raw, status, observed, sampled_ns)
            next_cycle = dataclasses.replace(terminal, sequence=84, rl_step=10, session_id=772,
                                              monotonic_ns=base_ns + 20_000_000)
            next_raw = pack_frame(next_cycle)
            next_status, next_frame = classify_frame(next_raw, base_ns + 20_000_001)
            # A changed writer session is rejected before it can be appended to
            # this run's record, and the closeout carries the merge failure.
            rejected = session_capture.observe(next_raw, next_status, next_frame, base_ns + 20_000_001)
            assert rejected["status"] == "failed" and "runtime_session_changed" in rejected["reason"]
            assert session_capture.source_mismatch == "runtime session changed 771->772", session_capture.source_mismatch
            session_capture.finish({"exit_code": 0, "forced_termination": False,
                "shutdown_complete": True, "shutdown_request_source": "offline session binding test"})
            assert len(run_record.load_record(str(session_path)).frames) == 1
            assert run_record.summarize_record(str(session_path))["record_validity"] == "INVALID"

            order_path = Path(tmp) / "cycle-order-mismatch.jsonl"
            order_capture = FrameCapture(order_path, "cycle-order-capture", "p1-10-capture-" + "d" * 32, 0,
                resolve_scene("scene_obstacle.xml"), candidate_mode=1,
                entry_threshold=-0.05, exit_threshold=-0.05, hold_duration_s=0.0)
            order_capture.start()
            order_capture.observe(raw, status, observed, sampled_ns)
            reversed_cycle = dataclasses.replace(terminal, sequence=84, rl_step=8,
                                                 monotonic_ns=base_ns + 20_000_000)
            reversed_raw = pack_frame(reversed_cycle)
            reversed_status, reversed_frame = classify_frame(reversed_raw, base_ns + 20_000_001)
            rejected = order_capture.observe(reversed_raw, reversed_status, reversed_frame, base_ns + 20_000_001)
            assert rejected["status"] == "failed" and "active_policy_cycle_not_increasing" in rejected["reason"]
            order_capture.finish({"exit_code": 0, "forced_termination": False,
                "shutdown_complete": True, "shutdown_request_source": "offline cycle order test"})
            assert len(run_record.load_record(str(order_path)).frames) == 1
            assert run_record.summarize_record(str(order_path))["record_validity"] == "INVALID"
    finally:
        run_record.read_collision_snapshot = original


def test_missing_terminal_frame_payload_is_rejected() -> None:
    base_ns = time.monotonic_ns()
    prior = dataclasses.replace(fixture(sequence=90, rl_step=0, session_id=981,
        monotonic_ns=base_ns), sim_time_s=5.0, world_pose=(0.0, 0.0, 0.0), exit_threshold=-0.05)
    terminal = dataclasses.replace(fixture(sequence=92, rl_step=1, session_id=981,
        monotonic_ns=base_ns + 100_000_000), sim_time_s=5.1, world_pose=(0.1, 0.0, 0.0), exit_threshold=-0.05)
    original = run_record.read_collision_snapshot
    run_record.read_collision_snapshot = lambda: b""
    try:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); record = root / "runtime_record.jsonl"
            capture = _capture(record, "terminal-write-failure", "p1-10-capture-" + "c" * 32)
            capture.start()
            prior_raw = pack_frame(prior)
            prior_status, prior_frame = classify_frame(prior_raw, base_ns + 1)
            assert capture.observe(prior_raw, prior_status, prior_frame, base_ns + 1)["persisted"]
            saved_method = capture.recorder.record_snapshot
            capture.recorder.record_snapshot = lambda *args, **kwargs: (_ for _ in ()).throw(OSError("simulated write failure"))
            terminal_raw = pack_frame(terminal)
            terminal_status, terminal_frame = classify_frame(terminal_raw, base_ns + 100_000_001)
            failed = capture.observe(terminal_raw, terminal_status, terminal_frame, base_ns + 100_000_001)
            assert failed["status"] == "failed" and failed["persisted"] is False
            assert "simulated write failure" in failed["reason"]
            capture.recorder.record_snapshot = saved_method
            capture.stop.set()
            capture.recorder.stop_sampling()
            capture.recorder.finalize({"exit_code": 0, "forced_termination": False,
                "shutdown_complete": True, "shutdown_request_source": "offline missing terminal test",
                "run_terminal_result": "ARRIVED", "run_terminal_monotonic_ns": base_ns + 200_000_000})
            context = root / "context.json"
            context.write_text(json.dumps({"run_id": "terminal-write-failure", "settings": {"goal_world_m": [4.0, 0.0]},
                "lifecycle": {"motion_start_observed_sim_time_s": 5.0, "movement_end_sim_time_s": 5.1,
                    "terminal_frame_ref": {"session_id": 981, "rl_step": 1, "source_sequence": 92,
                                            "monotonic_ns": base_ns + 100_000_000},
                    "terminal_frame_capture": {"status": "failed", "complete": False,
                                                "reason": failed["reason"]}},
                "result_contract": {"classification": "ARRIVED"}}), encoding="utf-8")
            result = calculate(record, context)
            assert result["record_validity"] == "INVALID"
            assert result["terminal_result"] == "INCOMPLETE_TERMINAL_FRAME_CAPTURE"
            assert result["motion_metric_time_boundary"]["terminal_frame_payload_complete"] is False
            assert "terminal_frame_payload_missing_or_incomplete" in result["record_validity_reasons"]
            assert result["mode_transitions"] is None and result["path_length_m"] is None
    finally:
        run_record.read_collision_snapshot = original


def test_production_closeout_end_to_end_and_failure_keeps_cleanup() -> None:
    base_ns = time.monotonic_ns()
    original_collision_reader = run_record.read_collision_snapshot
    original_history_reader = s1_run_diagnostic.read_shm_history
    run_record.read_collision_snapshot = lambda: b""
    try:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run_id = "closeout-e2e"
            capture_id = "p1-10-capture-" + "e" * 32
            capture = _capture(root / "runtime_record.jsonl", run_id, capture_id)
            capture.start()
            persisted_refs = []
            frames = [
                dataclasses.replace(fixture(sequence=102, rl_step=0, session_id=445, monotonic_ns=base_ns),
                    sim_time_s=10.0, world_pose=(0.0, 0.0, 0.0), mode_before=0, policy_state=0),
                dataclasses.replace(fixture(sequence=104, rl_step=1, session_id=445, monotonic_ns=base_ns+100_000_000),
                    sim_time_s=10.1, world_pose=(0.1, 0.0, 0.0), mode_before=0, policy_state=0),
                # This cleanup-tail frame must remain outside the motion metric boundary.
                dataclasses.replace(fixture(sequence=106, rl_step=2, session_id=445, monotonic_ns=base_ns+200_000_000),
                    sim_time_s=10.2, world_pose=(0.7, 0.0, 0.0), mode_before=0, policy_state=0),
            ]
            # This synthetic production-path case must model the current
            # producer-history contract as well as the direct live snapshots.
            # The old fixture omitted the ring entirely, which correctly made
            # FrameCapture fail closed with policy_cycle_history_incomplete.
            history_frames = [(index, pack_frame(frame)) for index, frame in enumerate(frames)]
            def read_fixture_history(*, cursor: int = 0, expected_session: int | None = None,
                                     max_source_sequence: int | None = None, **_kwargs: object) -> dict:
                selected = []
                for index, raw in history_frames:
                    source_frame = frames[index]
                    if index < cursor:
                        continue
                    if expected_session is not None and source_frame.session_id != expected_session:
                        continue
                    if max_source_sequence is not None and source_frame.sequence > max_source_sequence:
                        continue
                    selected.append((index, raw))
                return {"status": "OK", "session_id": 445, "produced_count": len(history_frames),
                        "overwritten_count": 0, "frames": selected, "cursor": cursor}
            s1_run_diagnostic.read_shm_history = read_fixture_history
            terminal_ref = None
            for index, frame in enumerate(frames):
                raw = pack_frame(frame)
                status, observed = classify_frame(raw, frame.monotonic_ns + 1)
                result = capture.observe(raw, status, observed, frame.monotonic_ns + 1)
                assert result["persisted"] is True
                persisted_refs.append(result["frame_ref"])
                if index == 1:
                    terminal_ref = result["frame_ref"]

            context = {"run_id": run_id, "session_id": 445,
                "settings": {"goal_world_m": [1.0, 0.0]},
                "lifecycle": {"run_terminal_event": "ARRIVED",
                    "motion_start_observed_sim_time_s": 10.0,
                    "movement_end_sim_time_s": 10.1,
                    "terminal_frame_ref": terminal_ref,
                    "terminal_frame_capture": {"status": "persisted_same_snapshot", "complete": True}},
                "result_contract": {"classification": "ARRIVED"}}
            process_facts = {"exit_code": 0, "forced_termination": False, "shutdown_complete": True,
                "shutdown_request_source": "offline closeout e2e",
                "run_terminal_result": "ARRIVED", "run_terminal_monotonic_ns": base_ns+100_000_000,
                "cleanup_events": [{"phase": "cleanup", "event": "SYNTHETIC_CLEANUP"}]}
            cleanup_calls = []
            terminal, errors = s1_run_diagnostic.finalize_run_closeout(
                root, context, process_facts, capture, True,
                lambda: cleanup_calls.append("process-groups-stopped"))
            assert errors == [] and terminal is not None
            assert cleanup_calls == ["process-groups-stopped"]
            assert (root / "terminal.json").is_file()
            assert (root / "process_facts.json").is_file()
            assert json.loads((root / "run_context.json").read_text())["lifecycle"]["terminal_json_saved"] is True
            stored_context = json.loads((root / "run_context.json").read_text())
            stored_terminal = json.loads((root / "terminal.json").read_text())
            assert stored_terminal["run_id"] == run_id
            assert stored_context["lifecycle"]["terminal_frame_ref"] == terminal_ref

            record_lines = [json.loads(line) for line in (root / "runtime_record.jsonl").read_text().splitlines()]
            terminal_lines = [line for line in record_lines if line.get("kind") == "terminal"]
            assert len(terminal_lines) == 1
            data = run_record.load_record(str(root / "runtime_record.jsonl"))
            assert len(data.frames) == 3 and data.terminal is not None
            terminal_payload = next(line["payload"] for line in data.frames
                if line["payload"]["session_id"] == terminal_ref["session_id"]
                and line["payload"]["rl_step"] == terminal_ref["rl_step"]
                and line["payload"]["source_sequence"] == terminal_ref["source_sequence"])
            assert terminal_payload["monotonic_ns"] == terminal_ref["monotonic_ns"]
            assert terminal_payload["ra_value"] is not None
            assert terminal_payload["mode_before"] == 0 and terminal_payload["mode_after"] == 0
            assert terminal_payload["action_raw"] and terminal_payload["world_pose"] and terminal_payload["sim_clock_valid"] is True
            metrics = calculate(root / "runtime_record.jsonl", root / "run_context.json")
            assert metrics["record_validity"] == "VALID", metrics.get("record_validity_reasons")
            assert metrics["terminal_result"] == "ARRIVED"
            assert metrics["motion_metric_time_boundary"]["end_cycle"] == 1
            assert metrics["motion_metric_time_boundary"]["terminal_frame_payload_complete"] is True
            assert math.isclose(metrics["path_length_m"], 0.1, abs_tol=1e-6)
            s1_run_diagnostic.write_json(root / "metrics.json", metrics)
            assert json.loads((root / "metrics.json").read_text())["record_validity"] == "VALID"
            s1_run_diagnostic.read_shm_history = original_history_reader

        # An artifact error after the process-cleanup callback is reported, while
        # cleanup still runs and the analyzer refuses to call the result complete.
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            failed_capture = _capture(root / "runtime_record.jsonl", "closeout-error",
                "p1-10-capture-" + "f" * 32)
            failed_capture.start()
            frame = dataclasses.replace(fixture(sequence=202, rl_step=0, session_id=446, monotonic_ns=base_ns+300_000_000),
                sim_time_s=4.0, world_pose=(0.0, 0.0, 0.0), mode_before=0, policy_state=0)
            raw = pack_frame(frame); status, observed = classify_frame(raw, frame.monotonic_ns+1)
            frame_ref = failed_capture.observe(raw, status, observed, frame.monotonic_ns+1)["frame_ref"]
            failed_context = {"run_id": "closeout-error", "session_id": 446, "settings": {"goal_world_m": [1.0, 0.0]},
                "lifecycle": {"run_terminal_event": "ARRIVED", "motion_start_observed_sim_time_s": 4.0,
                    "movement_end_sim_time_s": 4.0, "terminal_frame_ref": frame_ref,
                    "terminal_frame_capture": {"status": "persisted_same_snapshot", "complete": True}},
                "result_contract": {"classification": "ARRIVED"}}
            failed_facts = {"exit_code": 0, "forced_termination": False, "shutdown_complete": True,
                "shutdown_request_source": "offline closeout error", "run_terminal_result": "ARRIVED",
                "run_terminal_monotonic_ns": base_ns+300_000_000}
            cleanup_called = []
            original_write_json = s1_run_diagnostic.write_json
            injected = {"done": False}
            def fail_first_context_write(path, value):
                if Path(path).name == "run_context.json" and not injected["done"]:
                    injected["done"] = True
                    raise OSError("injected context write fault")
                return original_write_json(path, value)
            s1_run_diagnostic.write_json = fail_first_context_write
            try:
                _, closeout_errors = s1_run_diagnostic.finalize_run_closeout(
                    root, failed_context, failed_facts, failed_capture, True,
                    lambda: cleanup_called.append("process-groups-stopped"))
            finally:
                s1_run_diagnostic.write_json = original_write_json
            assert cleanup_called == ["process-groups-stopped"]
            assert any(item["stage"] == "run_context_pre_finalize" for item in closeout_errors)
            saved_context = json.loads((root / "run_context.json").read_text())
            assert saved_context["lifecycle"]["closeout_errors"] == closeout_errors
            failed_analysis = calculate(root / "runtime_record.jsonl", root / "run_context.json")
            assert failed_analysis["record_validity"] == "INVALID"
            assert failed_analysis["terminal_result"] == "INCOMPLETE_CLOSEOUT"
    finally:
        run_record.read_collision_snapshot = original_collision_reader


if __name__ == "__main__":
    test_external_safety_event_has_terminal_priority()
    test_policy_safety_boundary_keeps_fault_frame_separate_from_last_policy_frame()
    test_step_gap_is_not_bridged_into_recovery_dwell()
    test_exit_cycle_closes_recovery_and_cleanup_tail_is_excluded()
    test_recovery_interval_ends_on_first_agile_cycle()
    test_missing_motion_interval_keeps_motion_metrics_unknown()
    test_terminal_snapshot_is_synchronously_persisted_once_with_full_payload()
    test_missing_terminal_frame_payload_is_rejected()
    test_production_closeout_end_to_end_and_failure_keeps_cleanup()
    print("S1-04 targeted offline checks PASS (terminal capture/dedup/failure, inclusive motion end, cleanup exclusion, gaps, dwell)")
