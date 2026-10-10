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
from s1_analyze_run import calculate  # noqa: E402
from test_run_record import fixture, pack_frame  # noqa: E402


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
            assert math.isclose(result["valid_motion_duration_s"], 0.4)
            assert math.isclose(result["recovery_sim_duration_s"], 0.4)
            assert result["recovery_share_valid_motion_time"] == 1.0
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
            assert math.isclose(result["path_length_m"], 0.1, abs_tol=1e-6), result
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
            capture = FrameCapture(path, "terminal-capture", "p1-10-capture-" + "a" * 32, run_start_ns=0)
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
            session_capture = FrameCapture(session_path, "session-capture", "p1-10-capture-" + "b" * 32, run_start_ns=0)
            session_capture.start()
            session_capture.observe(raw, status, observed, sampled_ns)
            next_cycle = dataclasses.replace(terminal, sequence=84, rl_step=10, session_id=772,
                                              monotonic_ns=base_ns + 20_000_000)
            next_raw = pack_frame(next_cycle)
            next_status, next_frame = classify_frame(next_raw, base_ns + 20_000_001)
            # A changed writer session is retained as raw evidence but rejected
            # for run validity, preserving the one-session binding rule.
            session_capture.observe(next_raw, next_status, next_frame, base_ns + 20_000_001)
            assert session_capture.source_mismatch == "runtime session changed 771->772"
            session_capture.recorder.stop_sampling()
            session_capture.recorder.finalize({"exit_code": 0, "forced_termination": False,
                "shutdown_complete": True, "shutdown_request_source": "offline session binding test"})
            assert run_record.summarize_record(str(session_path))["record_validity"] == "INVALID"

            order_path = Path(tmp) / "cycle-order-mismatch.jsonl"
            order_capture = FrameCapture(order_path, "cycle-order-capture", "p1-10-capture-" + "d" * 32,
                                         run_start_ns=0)
            order_capture.start()
            order_capture.observe(raw, status, observed, sampled_ns)
            reversed_cycle = dataclasses.replace(terminal, sequence=84, rl_step=8,
                                                 monotonic_ns=base_ns + 20_000_000)
            reversed_raw = pack_frame(reversed_cycle)
            reversed_status, reversed_frame = classify_frame(reversed_raw, base_ns + 20_000_001)
            order_capture.observe(reversed_raw, reversed_status, reversed_frame, base_ns + 20_000_001)
            order_capture.recorder.stop_sampling()
            order_capture.recorder.finalize({"exit_code": 0, "forced_termination": False,
                "shutdown_complete": True, "shutdown_request_source": "offline cycle order test"})
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
            capture = FrameCapture(record, "terminal-write-failure", "p1-10-capture-" + "c" * 32, run_start_ns=0)
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
    run_record.read_collision_snapshot = lambda: b""
    try:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run_id = "closeout-e2e"
            capture_id = "p1-10-capture-" + "e" * 32
            capture = FrameCapture(root / "runtime_record.jsonl", run_id, capture_id,
                                   run_start_ns=0, scene=resolve_scene("scene_obstacle.xml"))
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

        # An artifact error after the process-cleanup callback is reported, while
        # cleanup still runs and the analyzer refuses to call the result complete.
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            failed_capture = FrameCapture(root / "runtime_record.jsonl", "closeout-error",
                "p1-10-capture-" + "f" * 32, 0, resolve_scene("scene_obstacle.xml"))
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
    test_step_gap_is_not_bridged_into_recovery_dwell()
    test_exit_cycle_closes_recovery_and_cleanup_tail_is_excluded()
    test_recovery_interval_ends_on_first_agile_cycle()
    test_missing_motion_interval_keeps_motion_metrics_unknown()
    test_terminal_snapshot_is_synchronously_persisted_once_with_full_payload()
    test_missing_terminal_frame_payload_is_rejected()
    test_production_closeout_end_to_end_and_failure_keeps_cleanup()
    print("S1-04 targeted offline checks PASS (terminal capture/dedup/failure, inclusive motion end, cleanup exclusion, gaps, dwell)")
