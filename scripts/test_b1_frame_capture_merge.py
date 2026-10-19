#!/usr/bin/env python3
"""Offline production-path checks for B1 policy-frame merge and closeout.

Uses the real FrameCapture, RunRecordRecorder, production closeout and analyzer;
only the producer history input and physical auxiliary authorities are controlled.
No simulator, controller or policy inference is started.
"""
from __future__ import annotations

import dataclasses
import hashlib
import json
import shutil
import sys
import tempfile
import time
from collections import deque
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))

import run_record
import s1_run_diagnostic
import abs_collision
from abs_rt_frame import FrameStatus, RuntimeFrame, classify_frame
from abs_scene import resolve_scene
from s1_analyze_run import calculate
from s1_run_diagnostic import (FrameCapture, finalize_run_closeout,
                               normal_closeout_gate, select_strategy_terminal,
                               task_window_history_events)
from test_run_record import fixture, pack_frame

SESSION = 880_001
CAPTURE_PREFIX = "p1-10-capture-"
CAPACITY = 2048


def policy_frames(count: int, base_ns: int, *, session: int = SESSION) -> list[RuntimeFrame]:
    modes = [0, 1, 0, 0, 1]
    result = []
    for cycle in range(count):
        state = modes[cycle] if cycle < len(modes) else 0
        before = modes[cycle - 1] if cycle and cycle < len(modes) else state
        result.append(dataclasses.replace(
            fixture(sequence=1000 + 2 * cycle, rl_step=cycle, session_id=session,
                    monotonic_ns=base_ns + cycle * 20_000_000),
            switching_mode=1, entry_threshold=-0.05, exit_threshold=-0.08,
            hold_enabled=0, recovery_min_hold_s=0.0, hold_status=0,
            policy_state=state, mode_before=before, policy_mode_changed=int(state != before),
            mode_change_ns=base_ns + cycle * 20_000_000 if state != before else 0,
            sim_time_s=5.0 + cycle * 0.02,
            world_pose=(cycle * 0.1, 0.0, 0.0), rl_entered=1, rl_active=1))
    return result


class ControlledHistory:
    def __init__(self, frames: list[RuntimeFrame], *, session: int = SESSION,
                 capacity: int = CAPACITY, reset_count: int | None = None,
                 status_script: list[str] | None = None, unread_loss: int = 0,
                 incoherent_after_index: int | None = None):
        self.frames = frames
        self.session = session
        self.capacity = capacity
        self.reset_count = reset_count
        self.status_script = list(status_script or [])
        self.unread_loss = unread_loss
        self.incoherent_after_index = incoherent_after_index
        self.unpersisted_count = 0
        self.calls = 0

    def read(self, *, cursor: int = 0, expected_session: int | None = None,
             max_source_sequence: int | None = None, **_kwargs: Any) -> dict:
        self.calls += 1
        produced = (len(self.frames) + self.unpersisted_count
                    if self.reset_count is None else self.reset_count)
        if expected_session is not None and expected_session != self.session:
            return {"status": "SESSION_CHANGED", "session_id": self.session,
                    "expected_session_id": expected_session, "frames": [], "cursor": cursor}
        if produced < cursor:
            return {"status": "RESET", "session_id": self.session,
                    "produced_count": produced, "overwritten_count": 0,
                    "frames": [], "cursor": 0}
        oldest = max(0, produced - self.capacity)
        reused = oldest
        if cursor < oldest:
            lost = max(oldest - cursor, self.unread_loss)
            return {"status": "OVERFLOW", "session_id": self.session,
                    "produced_count": produced, "overwritten_count": reused,
                    "first_available_index": oldest, "requested_cursor": cursor,
                    "consumer_unread_loss_count": lost,
                    "consumer_unread_loss_range": [cursor, cursor + lost],
                    "frames": [], "cursor": cursor}
        selected = []
        next_cursor = cursor
        for index in range(cursor, min(produced, len(self.frames))):
            raw = pack_frame(self.frames[index])
            frame = RuntimeFrame.from_bytes(raw)
            if max_source_sequence is not None and frame.sequence > max_source_sequence:
                break
            if index < oldest:
                continue
            selected.append((index, raw))
            next_cursor = index + 1
        status = self.status_script.pop(0) if self.status_script else "OK"
        if status == "INCOHERENT" and self.incoherent_after_index is not None:
            selected = [(index, raw) for index, raw in selected
                        if index < self.incoherent_after_index]
            next_cursor = selected[-1][0] + 1 if selected else cursor
        batch = {"status": status, "session_id": self.session,
                 "produced_count": produced, "overwritten_count": reused,
                 "frames": selected, "cursor": next_cursor}
        if status == "INCOHERENT":
            batch["reason"] = "controlled_uncommitted_slot"
        return batch


def live(frame: RuntimeFrame) -> tuple[bytes, RuntimeFrame]:
    raw = pack_frame(frame)
    status, checked = classify_frame(raw, frame.monotonic_ns + 1)
    assert status is FrameStatus.LIVE and checked is not None
    return raw, checked


def new_capture(path: Path, run_id: str, session: int = SESSION) -> FrameCapture:
    return FrameCapture(path, run_id, CAPTURE_PREFIX + hashlib.sha256(run_id.encode()).hexdigest()[:32], 0,
                        resolve_scene("scene_obstacle.xml"), candidate_mode=1,
                        entry_threshold=-0.05, exit_threshold=-0.08, hold_duration_s=0.0)


def valid_context(run_id: str, scene: Any, terminal_ref: dict, terminal_frame: RuntimeFrame) -> dict:
    return {"run_id": run_id, "session_id": terminal_frame.session_id,
        "settings": {"scene_filename": scene.filename, "scene_id": scene.scene_id,
                     "scene_obstacle_count": scene.obstacle_count, "scene": scene.filename,
                     "goal_world_m": [10.0, 0.0]},
        "events": [{"phase": "motion", "event": "motion_start",
                    "session_id": terminal_frame.session_id, "rl_step": 0}],
        "lifecycle": {"run_terminal_event": "ARRIVED",
            "task_start": {"session_id": terminal_frame.session_id, "rl_step": 0,
                           "sim_time_s": 5.0, "sim_clock_valid": True},
            "policy_active_start": {"session_id": terminal_frame.session_id,
                                     "rl_step": 0, "sim_time_s": 5.0},
            "motion_start_observed_sim_time_s": 5.0,
            "movement_end_sim_time_s": terminal_frame.sim_time_s,
            "terminal_frame_ref": terminal_ref,
            "terminal_frame_capture": {"status": "persisted_same_snapshot", "complete": True}},
        "result_contract": {"classification": "ARRIVED"}}


def facts(terminal_ref: dict, terminal_frame: RuntimeFrame) -> dict:
    return {"exit_code": 0, "forced_termination": False, "shutdown_complete": True,
        "shutdown_request_source": "B1 offline closeout validation",
        "run_terminal_result": "ARRIVED", "run_terminal_monotonic_ns": terminal_frame.monotonic_ns,
        "run_terminal_frame_ref": terminal_ref,
        "cleanup_events": [{"phase": "cleanup", "event": "offline-owned-process-cleanup"}]}


def pack_v4_safety_snapshot(scene: Any, capture_id: str, *, sequence: int,
                            physics_step: int, sim_time: float, monotonic_ns: int,
                            fall_history: list[dict] | None = None,
                            collision_history: list[dict] | None = None) -> bytes:
    """Build a v4 authority wire image; production decodes and serializes it."""
    falls = fall_history or []
    collisions = collision_history or []
    binding = scene.binding()
    base = abs_collision.BASE_STRUCT.pack(
        abs_collision.MAGIC, 4, sequence, monotonic_ns, physics_step, sim_time,
        1, int(bool(collisions)), int(bool(collisions)), len(collisions), 0,
        int(bool(collisions)), 0, 0, 0, 1 if collisions else 0, -1, -1, 0,
        binding["scene_id"].encode(), binding["root_sha256"].encode(),
        binding["closure_sha256"].encode(), capture_id.encode(),
        binding["model_fingerprint"].encode())[:-4]
    extension = abs_collision.EXTENSION_STRUCT.pack(
        0, 0, int(bool(collisions)), len(collisions),
        collisions[0]["start_physics_step"] if collisions else 0,
        collisions[-1]["end_physics_step"] if collisions else 0,
        0.002 * sum(max(0, item["end_physics_step"] - item["start_physics_step"] + 1)
                    for item in collisions),
        collisions[0]["start_sim_time"] if collisions else 0.0,
        collisions[-1]["end_sim_time"] if collisions else 0.0,
        0.445, 0.0, 0.0,
        falls[0]["start_sim_time"] if falls else 0.0,
        next((item["confirmed_sim_time"] for item in falls if item["confirmed"]), 0.0),
        falls[0]["start_physics_step"] if falls else 0,
        next((item["confirmed_physics_step"] for item in falls if item["confirmed"]), 0),
        int(bool(falls) and not falls[-1]["closed"]),
        int(bool(falls) and falls[-1]["confirmed"]), len(collisions), 0, 1, 0)
    extra = abs_collision.V4_EXTENSION_STRUCT.pack(
        0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, len(falls), 0)
    fall_rows = [abs_collision.FALL_EVENT_STRUCT.pack(
        event["start_physics_step"], event["end_physics_step"],
        event["confirmed_physics_step"], event["start_sim_time"],
        event["end_sim_time"], event["confirmed_sim_time"],
        int(event["confirmed"]), int(event["closed"])) for event in falls]
    fall_rows.extend([bytes(abs_collision.FALL_EVENT_STRUCT.size)] *
                     (abs_collision.FALL_EVENT_CAPACITY - len(fall_rows)))
    collision_rows = [abs_collision.EPISODE_STRUCT.pack(
        event["start_physics_step"], event["end_physics_step"],
        event["start_sim_time"], event["end_sim_time"],
        event["robot_geom_id"], event["obstacle_geom_id"],
        event["robot_geom_name"].encode(), event["obstacle_geom_name"].encode())
        for event in collisions]
    collision_rows.extend([bytes(abs_collision.EPISODE_STRUCT.size)] *
                          (abs_collision.COLLISION_EVENT_CAPACITY - len(collision_rows)))
    raw = base + extension + extra + b"".join(fall_rows) + b"".join(collision_rows)
    assert len(raw) == abs_collision.SNAPSHOT_V4_SIZE
    return raw


def install_controlled(history: ControlledHistory):
    old = s1_run_diagnostic.read_shm_history
    s1_run_diagnostic.read_shm_history = history.read
    return old


def run_terminal_case(root: Path, label: str, frames: list[RuntimeFrame], history: ControlledHistory,
                      terminal_cycle: int, *, expect_valid: bool = True) -> dict:
    record = root / "runtime_record.jsonl"
    capture = new_capture(record, label)
    capture.start()
    raw, checked = live(frames[terminal_cycle])
    terminal_observe = capture.observe(raw, FrameStatus.LIVE, checked, checked.monotonic_ns + 1)
    terminal_ref = terminal_observe["frame_ref"]
    boundary = capture.mark_task_terminal(terminal_ref)
    context = valid_context(label, resolve_scene("scene_obstacle.xml"), terminal_ref, frames[terminal_cycle])
    process_facts = facts(terminal_ref, frames[terminal_cycle])
    cleanup_calls: list[str] = []
    terminal, closeout_errors = finalize_run_closeout(
        root, context, process_facts, capture, True,
        lambda: cleanup_calls.append("cleanup-called"))
    assert cleanup_calls == ["cleanup-called"]
    assert closeout_errors == [] and terminal is not None
    lines = [json.loads(line) for line in record.read_text().splitlines()]
    frame_lines = [line for line in lines if line.get("kind") == "frame"]
    terminal_lines = [line for line in lines if line.get("kind") == "terminal"]
    assert len(terminal_lines) == 1
    cycles = [line["payload"]["rl_step"] for line in frame_lines if line.get("status") == "LIVE"]
    assert cycles == list(range(len(cycles))), cycles[:10]
    loaded = run_record.load_record(str(record))
    terminal_payloads = [line["payload"] for line in loaded.frames
        if line["payload"].get("session_id") == terminal_ref["session_id"]
        and line["payload"].get("rl_step") == terminal_ref["rl_step"]
        and line["payload"].get("source_sequence") == terminal_ref["source_sequence"]
        and line["payload"].get("monotonic_ns") == terminal_ref["monotonic_ns"]]
    assert len(terminal_payloads) == 1
    assert all(key in terminal_payloads[0] for key in (
        "ra_value", "mode_before", "mode_after", "action_raw", "action_clipped",
        "joint_target_rad", "world_pose", "sim_time_s", "sim_clock_valid"))
    stored_history = (terminal or {}).get("policy_cycle_history", {})
    metrics = calculate(record, root / "run_context.json")
    (root / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n",
                                       encoding="utf-8")
    assert stored_history.get("task_termination_boundary", {}).get("status") == boundary["status"]
    assert stored_history.get("final_closeout_drain", {}).get("complete") is expect_valid
    assert metrics["policy_cycle_history_complete_at_task_termination"] == (boundary["status"] == "complete")
    assert metrics["policy_cycle_history_complete_at_final_closeout_drain"] is expect_valid
    if expect_valid:
        assert metrics["record_validity"] == "VALID", metrics.get("record_validity_reasons")
        assert metrics["terminal_result"] == "ARRIVED"
        assert metrics["motion_metric_time_boundary"]["end_cycle"] == terminal_cycle
        assert metrics["frame_count"] == len(frames)
        assert abs(metrics["path_length_m"] - terminal_cycle * 0.1) < 1e-6
        assert metrics["mode_transitions"] == sum(
            1 for i in range(terminal_cycle + 1)
            if frames[i].mode_before != frames[i].policy_state)
        # Cleanup tail is present in the JSONL but excluded from task transitions/path.
        assert len(cycles) > terminal_cycle + 1
        assert metrics["policy_cycle_history"]["producer_slot_reuse_count"] == 0
        assert metrics["policy_cycle_history"]["consumer_unread_loss_count"] == 0
    return {"record": str(record), "terminal_ref": terminal_ref,
            "cycles_written": cycles, "task_boundary": boundary,
            "history": stored_history, "metrics": metrics}


def run_closeout_growth_case(root: Path) -> dict:
    """Exercise task-boundary freeze, producer shutdown, final drain and analysis."""
    root.mkdir(parents=True, exist_ok=True)
    frames = policy_frames(6, time.monotonic_ns())
    history = ControlledHistory(frames[:3])
    old = install_controlled(history)
    try:
        scene = resolve_scene("scene_obstacle.xml")
        record = root / "runtime_record.jsonl"
        capture = new_capture(record, "closeout_growth")
        capture.start()
        raw, checked = live(frames[2])
        observed = capture.observe(raw, FrameStatus.LIVE, checked, checked.monotonic_ns + 1)
        boundary = capture.mark_task_terminal(observed["frame_ref"])
        assert boundary["status"] == "complete", boundary
        context = valid_context("closeout_growth", scene, observed["frame_ref"], frames[2])
        process_facts = facts(observed["frame_ref"], frames[2])
        cleanup_observed: list[dict] = []

        def stop_owned_producers() -> None:
            # The simulator/controller finish producing before final drain.
            # The same production recorder must still accept those ring frames.
            assert capture.recorder.state == "CAPTURING"
            assert not capture.stop.is_set()
            history.frames.extend(frames[3:])
            cleanup_observed.append({"writer_state": capture.recorder.state,
                                     "produced_after_task_terminal": 3})

        terminal, errors = finalize_run_closeout(
            root, context, process_facts, capture, True, stop_owned_producers)
        assert not errors and terminal is not None, errors
        lines = [json.loads(line) for line in record.read_text(encoding="utf-8").splitlines()]
        frame_lines = [line for line in lines if line.get("kind") == "frame"]
        cycles = [line["payload"]["rl_step"] for line in frame_lines]
        assert cycles == list(range(6)), cycles
        assert len([line for line in lines if line.get("kind") == "terminal"]) == 1
        history_facts = terminal["policy_cycle_history"]
        assert history_facts["task_termination_boundary"]["terminal_frame_ref"]["rl_step"] == 2
        assert history_facts["final_closeout_drain"]["complete"] is True
        close_order = context["lifecycle"]["capture_close_order"]
        assert close_order["record_writer_state_before_final_drain"] == "CAPTURING"
        assert close_order["owned_producer_process_shutdown_confirmed_before_final_drain"] is True
        assert len(cleanup_observed) == 1

        metrics = calculate(record, root / "run_context.json")
        assert metrics["record_validity"] == "VALID", metrics.get("record_validity_reasons")
        assert metrics["motion_metric_time_boundary"]["end_cycle"] == 2
        assert metrics["frame_count"] == 6
        assert metrics["mode_transitions"] == 2  # task edges at cycles 1 and 2; cycle 4 is closeout-only
        assert abs(metrics["path_length_m"] - 0.2) < 1e-7

        # A damaged copy must not be recovered by the closeout-only path.
        damaged = root / "damaged"
        damaged.mkdir()
        damaged_record = damaged / "runtime_record.jsonl"
        raw_lines = [json.loads(line) for line in record.read_text(encoding="utf-8").splitlines()]
        raw_lines = [line for line in raw_lines if not (
            line.get("kind") == "frame" and line.get("payload", {}).get("rl_step") == 1)]
        damaged_terminal = next(line for line in raw_lines if line.get("kind") == "terminal")
        damaged_history = damaged_terminal["policy_cycle_history"]
        damaged_history["final_closeout_drain"]["complete"] = False
        damaged_history["final_closeout_drain"]["producer_count"] += 1
        damaged_history["complete_at_last_drain"] = False
        damaged_record.write_text("".join(json.dumps(line, separators=(",", ":")) + "\n"
                                                 for line in raw_lines), encoding="utf-8")
        damaged_context = dict(context)
        damaged_context_path = damaged / "run_context.json"
        damaged_context_path.write_text(json.dumps(damaged_context), encoding="utf-8")
        rejected = calculate(damaged_record, damaged_context_path)
        assert rejected["record_validity"] == "INVALID"
        assert rejected["task_window_metrics_status"].startswith("UNKNOWN"), rejected

        return {"record": str(record), "terminal": str(root / "terminal.json"),
                "context": str(root / "run_context.json"), "cycles_persisted": cycles,
                "task_terminal_cycle": 2,
                "task_boundary_status": history_facts["task_termination_boundary"]["status"],
                "final_drain_complete": history_facts["final_closeout_drain"]["complete"],
                "record_validity": metrics["record_validity"],
                "analyzed_motion_end_cycle": metrics["motion_metric_time_boundary"]["end_cycle"],
                "task_path_length_m": metrics["path_length_m"],
                "task_transition_count": metrics["mode_transitions"],
                "postterminal_edges_excluded": True,
                "incomplete_task_boundary_rejected": True}
    finally:
        s1_run_diagnostic.read_shm_history = old


def run_closeout_only_loss_case(root: Path) -> dict:
    """Prove closeout-only loss preserves task metrics but not full validity."""
    root.mkdir(parents=True, exist_ok=True)
    frames = policy_frames(3, time.monotonic_ns())
    history = ControlledHistory(frames)
    old = install_controlled(history)
    try:
        scene = resolve_scene("scene_obstacle.xml")
        record = root / "runtime_record.jsonl"
        capture = new_capture(record, "closeout_only_loss")
        capture.start()
        raw, checked = live(frames[2])
        observed = capture.observe(raw, FrameStatus.LIVE, checked, checked.monotonic_ns + 1)
        boundary = capture.mark_task_terminal(observed["frame_ref"])
        assert boundary["status"] == "complete"
        context = valid_context("closeout_only_loss", scene, observed["frame_ref"], frames[2])
        process_facts = facts(observed["frame_ref"], frames[2])

        def closeout_reports_unpersisted_tail() -> None:
            assert capture.recorder.state == "CAPTURING"
            history.unpersisted_count = 48

        terminal, errors = finalize_run_closeout(
            root, context, process_facts, capture, True, closeout_reports_unpersisted_tail)
        assert not errors and terminal is not None
        metrics = calculate(record, root / "run_context.json")
        (root / "task_window_reanalysis.json").write_text(
            json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        assert metrics["record_validity"] == "INVALID"
        assert metrics["record_validity_reasons"] == ["policy_cycle_history_incomplete"]
        assert metrics["terminal_result"] == "ARRIVED"
        assert metrics["task_window_metrics_status"] == "task_window_complete_closeout_incomplete"
        assert metrics["task_window_integrity"]["closeout_producer_entries_unpersisted"] == 48
        assert metrics["motion_metric_time_boundary"]["end_cycle"] == 2
        assert abs(metrics["path_length_m"] - 0.2) < 1e-7
        assert metrics["mode_transitions"] == 2
        return {"record": str(record),
                "derived_metrics": str(root / "task_window_reanalysis.json"),
                "full_record_validity": metrics["record_validity"],
                "validity_reasons": metrics["record_validity_reasons"],
                "task_window_status": metrics["task_window_metrics_status"],
                "task_end_cycle": metrics["motion_metric_time_boundary"]["end_cycle"],
                "closeout_entries_unpersisted": metrics["task_window_integrity"]["closeout_producer_entries_unpersisted"],
                "task_path_length_m": metrics["path_length_m"],
                "task_transition_count": metrics["mode_transitions"],
                "original_invalidity_preserved": True}
    finally:
        s1_run_diagnostic.read_shm_history = old


def run_startup_failure_cleanup_case(root: Path) -> dict:
    """Startup failure still invokes owned-process cleanup before artifact close."""
    root.mkdir(parents=True, exist_ok=True)
    capture = new_capture(root / "runtime_record.jsonl", "startup_failure_closeout")
    context = {"run_id": "startup_failure_closeout", "lifecycle": {}}
    process_facts = {"shutdown_complete": True, "cleanup_errors": [],
                     "closeout_errors": [], "closeout_complete": False}
    cleanup_calls: list[str] = []
    terminal, errors = finalize_run_closeout(
        root, context, process_facts, capture, False,
        lambda: cleanup_calls.append("owned-processes-stopped"))
    assert cleanup_calls == ["owned-processes-stopped"]
    assert terminal is None and errors == []
    assert process_facts["closeout_complete"] is True
    return {"cleanup_called": True, "terminal_written": False,
            "closeout_complete": process_facts["closeout_complete"], "errors": errors}


def run_deduplicated_safety_case(root: Path, label: str, *,
                                 fall_updates: list[list[dict]] | None = None,
                                 collision_event: dict | None = None,
                                 close_after_terminal: bool = False,
                                 missing_safety: bool = False) -> dict:
    """Exercise real history->record->dedupe->terminal->analyzer safety flow."""
    frames = policy_frames(4, time.monotonic_ns())
    history = ControlledHistory(frames)
    previous_history_reader = install_controlled(history)
    root.mkdir(parents=True, exist_ok=True)
    scene = resolve_scene("scene_obstacle.xml")
    record = root / "runtime_record.jsonl"
    capture = new_capture(record, label)
    capture.start()
    fall_updates = fall_updates or [[] for _ in frames]
    safety_bytes = []
    for index, frame in enumerate(frames):
        current_falls = fall_updates[index] if index < len(fall_updates) else fall_updates[-1]
        if close_after_terminal and index == 3 and current_falls:
            current_falls = [dict(event, closed=True) for event in current_falls]
        collisions = [collision_event] if collision_event is not None and index >= 2 else []
        safety_bytes.append(b"" if missing_safety else pack_v4_safety_snapshot(
            scene, capture.recorder.capture_id, sequence=2 * (index + 1),
            physics_step=400 + index * 10, sim_time=5.0 + index * 0.02,
            monotonic_ns=frame.monotonic_ns,
            fall_history=current_falls, collision_history=collisions))
    previous_collision_reader = run_record.read_collision_snapshot
    run_record.read_collision_snapshot = lambda: safety_bytes.pop(0)
    cleanup: list[str] = []
    try:
        raw, checked = live(frames[2])
        observed = capture.observe(raw, FrameStatus.LIVE, checked, checked.monotonic_ns + 1)
        assert observed["status"] == "already_persisted", observed
        assert observed.get("collision_snapshot_relation") == \
            "attached_physics_sample_in_same_persisted_policy_record"
        snapshot = observed.get("collision_snapshot")
        assert isinstance(snapshot, dict), snapshot
        if not missing_safety:
            assert snapshot.get("status") == "LIVE", snapshot
            assert observed["collision_snapshot_identity"]["physics_step"] == 420
            assert observed["collision_snapshot_identity"]["sim_time"] == 5.04
        collision_events, fall_events = task_window_history_events(snapshot, 5.0, 5.04, 390)
        if collision_event is not None:
            assert collision_events and collision_events[0]["obstacle_geom_name"] == "test_obstacle"
            assert select_strategy_terminal(False, bool(collision_events), False, True, False) == \
                "COLLISION_TERMINATION"
        if fall_events and any(event.get("confirmed") is True for event in fall_events):
            assert select_strategy_terminal(False, False, True, True, False) == "FALL_TERMINATION"
        eligible, reason = normal_closeout_gate("ARRIVED", False, [], False, snapshot)
        if collision_event is not None or any(not event.get("closed") for event in fall_events):
            assert eligible is False and reason is not None

        terminal_cause = ("COLLISION_TERMINATION" if collision_event is not None else
                          "FALL_TERMINATION" if any(event.get("confirmed") for event in fall_events) else
                          "ARRIVED")
        terminal_ref = observed["frame_ref"]
        boundary = capture.mark_task_terminal(terminal_ref)
        context = valid_context(label, scene, terminal_ref, frames[2])
        context["lifecycle"]["run_terminal_event"] = terminal_cause
        context["result_contract"]["classification"] = terminal_cause
        context["lifecycle"]["normal_closeout_gate"] = {
            "eligible": eligible, "reason": reason,
            "physics_status": snapshot.get("status")}
        process_facts = facts(terminal_ref, frames[2])
        process_facts["run_terminal_result"] = terminal_cause
        terminal, closeout_errors = finalize_run_closeout(
            root, context, process_facts, capture, True,
            lambda: cleanup.append("cleanup-called"))
        assert cleanup == ["cleanup-called"] and not closeout_errors and terminal
        lines = [json.loads(line) for line in record.read_text(encoding="utf-8").splitlines()]
        frame_lines = [line for line in lines if line.get("kind") == "frame"]
        assert [line["payload"]["rl_step"] for line in frame_lines] == [0, 1, 2, 3]
        assert len([line for line in lines if line.get("kind") == "terminal"]) == 1
        metrics = calculate(record, root / "run_context.json")
        (root / "metrics.json").write_text(
            json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        assert metrics["record_validity"] == "VALID", metrics.get("record_validity_reasons")
        assert metrics["terminal_result"] == terminal_cause, metrics["terminal_result"]
        assert metrics["motion_metric_time_boundary"]["end_cycle"] == 2
        result = {"artifact_directory": str(root),
                  "runtime_record": str(record), "terminal_json": str(root / "terminal.json"),
                  "run_context": str(root / "run_context.json"),
                  "metrics_json": str(root / "metrics.json"),
                  "terminal": terminal_cause, "dedupe_status": observed["status"],
                  "snapshot_identity": observed["collision_snapshot_identity"],
                  "closeout_gate": {"eligible": eligible, "reason": reason},
                  "record_validity": metrics["record_validity"],
                  "analyzed_terminal": metrics["terminal_result"],
                  "fall": metrics.get("task_window_safety", {}).get("fall"),
                  "fall_result_basis": metrics.get("task_window_safety", {}).get("fall_result_basis"),
                  "cycles": [line["payload"]["rl_step"] for line in frame_lines],
                  "cleanup_called": True,
                  "analyzed_safety": metrics.get("task_window_safety"),
                  "metrics": metrics}
        (root / "case_summary.json").write_text(
            json.dumps({key: value for key, value in result.items() if key != "metrics"},
                       indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return result
    finally:
        run_record.read_collision_snapshot = previous_collision_reader
        s1_run_diagnostic.read_shm_history = previous_history_reader


def main(output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    result: dict[str, Any] = {"scope": "offline production FrameCapture -> RunRecordRecorder -> closeout -> analyzer",
                              "scenario_diagnostics_started": False, "cases": {}}
    old_collision = run_record.read_collision_snapshot
    old_bounds = run_record.read_base_bounds_snapshot
    original_reader = s1_run_diagnostic.read_shm_history
    run_record.read_collision_snapshot = lambda: b""
    run_record.read_base_bounds_snapshot = lambda **_kwargs: {"status": "MISSING", "reason": "offline isolated fixture"}
    base_ns = time.monotonic_ns()
    try:
        closeout_root = output / "task_closeout_drain"
        if closeout_root.exists():
            shutil.rmtree(closeout_root)
        result["cases"]["producer_stops_then_final_drain_while_writer_open"] = run_closeout_growth_case(
            closeout_root)
        lost_tail_root = output / "closeout_only_loss"
        if lost_tail_root.exists():
            shutil.rmtree(lost_tail_root)
        result["cases"]["complete_task_window_closeout_loss_stays_invalid"] = run_closeout_only_loss_case(
            lost_tail_root)
        startup_root = output / "startup_failure_cleanup"
        if startup_root.exists():
            shutil.rmtree(startup_root)
        result["cases"]["startup_failure_still_runs_cleanup"] = run_startup_failure_cleanup_case(
            startup_root)

        # Live N races with already-published N+1/N+2. N is terminal; tail is
        # drained only after the terminal boundary has been frozen.
        root = output / "terminal_race"
        if root.exists():
            shutil.rmtree(root)
        root.mkdir(parents=True)
        frames = policy_frames(5, base_ns)
        history = ControlledHistory(frames)
        old = install_controlled(history)
        try:
            case = run_terminal_case(root, "terminal_race", frames, history, 2)
        finally:
            s1_run_diagnostic.read_shm_history = old
        assert case["cycles_written"] == [0, 1, 2, 3, 4]
        assert case["metrics"]["mode_transitions"] == 2
        assert case["metrics"]["policy_cycle_history"]["identity_consistent_duplicate_count"] >= 1
        result["cases"]["terminal_N_then_cleanup_N1_N2"] = {
            "artifact_directory": str(root),
            "runtime_record": str(root / "runtime_record.jsonl"),
            "terminal": str(root / "terminal.json"),
            "context": str(root / "run_context.json"),
            "metrics": str(root / "metrics.json"),
            "producer_cycles": [0, 1, 2, 3, 4], "consumer_cursor_final": 5,
            "persisted_cycles": case["cycles_written"], "terminal_cycle": 2,
            "task_cycle_history": case["task_boundary"],
            "final_drain_complete": case["history"]["final_closeout_drain"]["complete"],
            "analyzer_record_validity": case["metrics"]["record_validity"],
            "analyzer_motion_end_cycle": case["metrics"]["motion_metric_time_boundary"]["end_cycle"],
            "analyzer_mode_transitions": case["metrics"]["mode_transitions"],
            "deduped_duplicate_count": case["history"]["identity_consistent_duplicate_count"]}

        # A transient uncommitted slot after a readable prefix is retried from
        # the advanced cursor without reordering or inventing a lost cycle.
        with tempfile.TemporaryDirectory(dir=output) as temp:
            root = Path(temp); frames = policy_frames(4, base_ns + 1_000_000_000)
            history = ControlledHistory(frames, status_script=["INCOHERENT", "OK", "OK"],
                                        incoherent_after_index=2)
            old = install_controlled(history)
            try:
                case = run_terminal_case(root, "delayed_commit", frames, history, 2)
            finally:
                s1_run_diagnostic.read_shm_history = old
            retries = case["history"]["history_read_retries"]
            assert retries >= 1
            result["cases"]["delayed_uncommitted_slot_retried"] = {
                "history_reads": history.calls, "retries": retries,
                "persisted_cycles": case["cycles_written"],
                "unread_loss": case["history"]["consumer_unread_loss_count"],
                "task_boundary": case["task_boundary"]["status"],
                "final_drain": case["history"]["final_closeout_drain"]["complete"]}

        # Persistently uncommitted input exhausts the bounded retry budget and
        # remains incomplete without inventing a missing-index count.
        with tempfile.TemporaryDirectory(dir=output) as temp:
            root = Path(temp); frames = policy_frames(3, base_ns + 1_500_000_000)
            history = ControlledHistory(frames,
                status_script=["INCOHERENT"] * 6,
                incoherent_after_index=1)
            old = install_controlled(history)
            capture = new_capture(root / "persistent_uncommitted.jsonl", "persistent_uncommitted")
            capture.start(); raw, checked = live(frames[2])
            observed = capture.observe(raw, FrameStatus.LIVE, checked, checked.monotonic_ns + 1)
            boundary = capture.mark_task_terminal(observed["frame_ref"])
            terminal = capture.finish({"exit_code": 0, "forced_termination": False,
                "shutdown_complete": True, "shutdown_request_source": "persistent slot test"})
            s1_run_diagnostic.read_shm_history = old
            audit = terminal["policy_cycle_history"]
            assert boundary["status"] == "incomplete"
            assert audit["history_read_retries"] >= 2
            assert audit["consumer_unread_loss_count"] == 0
            assert audit["error"].startswith("INCOHERENT:")
            result["cases"]["persistent_uncommitted_slot_fails_closed"] = {
                "reader_calls": history.calls, "retry_limit": 2,
                "persisted_cycle_count": audit["runtime_record_unique_frames"],
                "task_boundary": boundary["status"], "unread_loss": audit["consumer_unread_loss_count"],
                "history_error": audit["error"], "final_complete": audit["final_closeout_drain"]["complete"]}

        # Multiple-cycle catch-up from cycle zero through terminal.
        with tempfile.TemporaryDirectory(dir=output) as temp:
            root = Path(temp); frames = policy_frames(5, base_ns + 2_000_000_000)
            history = ControlledHistory(frames)
            old = install_controlled(history)
            try:
                case = run_terminal_case(root, "multi_cycle_catchup", frames, history, 3)
            finally:
                s1_run_diagnostic.read_shm_history = old
            result["cases"]["multi_cycle_catchup"] = {
                "producer_cycles": [0, 1, 2, 3, 4], "consumer_cursor_final": 5,
                "persisted_cycles": case["cycles_written"], "terminal_cycle": 3,
                "task_boundary": case["task_boundary"]["status"],
                "analysis": case["metrics"]["record_validity"]}

        # A same-identity but different payload must be rejected, never silently
        # overwritten or removed from persisted output.
        with tempfile.TemporaryDirectory(dir=output) as temp:
            root = Path(temp); frames = policy_frames(1, base_ns + 3_000_000_000)
            history = ControlledHistory(frames)
            old = install_controlled(history)
            capture = new_capture(root / "conflict.jsonl", "conflict")
            capture.start(); raw, checked = live(frames[0])
            conflicting = dataclasses.replace(checked, ra_value=checked.ra_value + 0.25)
            failed = capture.observe(pack_frame(conflicting), FrameStatus.LIVE, conflicting,
                                     conflicting.monotonic_ns + 1)
            facts_ = {"exit_code": 0, "forced_termination": False, "shutdown_complete": True,
                      "shutdown_request_source": "conflict test", "run_terminal_result": "INVALID",
                      "run_terminal_monotonic_ns": conflicting.monotonic_ns}
            terminal = capture.finish(facts_)
            s1_run_diagnostic.read_shm_history = old
            assert failed["status"] == "failed" and "same_runtime_identity_different_content" in failed["reason"]
            assert capture.identity_conflict_count == 1 and capture.unique_frames == 1
            result["cases"]["same_identity_conflicting_content_rejected"] = {
                "persisted_frames": capture.unique_frames, "conflicts": capture.identity_conflict_count,
                "merge_error": capture.merge_error, "final_record_validity":
                    run_record.summarize_record(str(root / "conflict.jsonl"))["record_validity"]}

        # Increasing source sequence cannot legitimize a monotonic-clock rollback.
        with tempfile.TemporaryDirectory(dir=output) as temp:
            root = Path(temp); frames = policy_frames(1, base_ns + 3_500_000_000)
            history = ControlledHistory(frames)
            old = install_controlled(history)
            capture = new_capture(root / "clock_rollback.jsonl", "clock_rollback"); capture.start()
            raw0, checked0 = live(frames[0])
            assert capture.observe(raw0, FrameStatus.LIVE, checked0, checked0.monotonic_ns+1)["persisted"]
            rollback = dataclasses.replace(frames[0], sequence=frames[0].sequence + 2,
                                           rl_step=frames[0].rl_step + 1,
                                           monotonic_ns=frames[0].monotonic_ns)
            raw1, checked1 = live(rollback)
            rejected = capture.observe(raw1, FrameStatus.LIVE, checked1, checked1.monotonic_ns+1)
            capture.finish({"exit_code": 0, "forced_termination": False,
                "shutdown_complete": True, "shutdown_request_source": "monotonic rollback test"})
            s1_run_diagnostic.read_shm_history = old
            assert rejected["status"] == "failed" and "runtime_monotonic_not_strictly_increasing" in rejected["reason"]
            assert capture.unique_frames == 1
            result["cases"]["monotonic_rollback_rejected"] = {
                "persisted_frames": capture.unique_frames, "merge_error": capture.merge_error}

        # Session change and same-session producer counter reset are separately
        # visible and fail closed.
        for label, history in (
            ("session_change", ControlledHistory(policy_frames(1, base_ns + 4_000_000_000), session=999)),
            ("counter_reset", ControlledHistory(policy_frames(1, base_ns + 5_000_000_000))),
        ):
            with tempfile.TemporaryDirectory(dir=output) as temp:
                root = Path(temp); frames = policy_frames(1, base_ns + 4_000_000_000)
                # Session mismatch is enforced by reader; counter reset after a
                # direct frame models producer reset while consumer cursor=1.
                old = install_controlled(history)
                capture = new_capture(root / f"{label}.jsonl", label); capture.start()
                raw, checked = live(frames[0])
                capture.observe(raw, FrameStatus.LIVE, checked, checked.monotonic_ns + 1)
                if label == "counter_reset":
                    assert capture.history_cursor == 1
                    history.reset_count = 0
                batch = capture.drain_history()
                if label == "session_change":
                    assert batch["status"] == "SESSION_CHANGED"
                else:
                    assert batch["status"] == "RESET"
                capture.finish({"exit_code": 0, "forced_termination": False,
                    "shutdown_complete": True, "shutdown_request_source": "identity boundary test"})
                s1_run_diagnostic.read_shm_history = old
                result["cases"][label] = {"reader_status": batch["status"],
                    "history_error": capture.history_error, "final_complete":
                    capture.history_status == "OK" and capture.history_error is None}

        # Consumer loss beyond the declared ring window is reported once even
        # though final closeout retries the same unchanged cursor.
        with tempfile.TemporaryDirectory(dir=output) as temp:
            root = Path(temp); frames = policy_frames(5, base_ns + 6_000_000_000)
            history = ControlledHistory(frames, capacity=2, unread_loss=3)
            old = install_controlled(history)
            capture = new_capture(root / "overflow.jsonl", "overflow"); capture.start()
            raw, checked = live(frames[0]); capture.observe(raw, FrameStatus.LIVE, checked, checked.monotonic_ns+1)
            result_overflow = capture.finish({"exit_code": 0, "forced_termination": False,
                "shutdown_complete": True, "shutdown_request_source": "overflow test"})
            s1_run_diagnostic.read_shm_history = old
            history_audit = result_overflow["policy_cycle_history"]
            assert history_audit["consumer_unread_loss_count"] == 3
            result["cases"]["consumer_lag_exceeds_capacity"] = {
                "producer_count": history_audit["producer_count_at_last_drain"],
                "cursor": history_audit["consumer_cursor"],
                "unread_loss_count": history_audit["consumer_unread_loss_count"],
                "unread_loss_ranges": history_audit["consumer_unread_loss_ranges"],
                "final_complete": history_audit["final_closeout_drain"]["complete"]}

        # Consumer follows producer past capacity: the producer's cumulative
        # slot reuse is not loss when cursor remains at the oldest unread index.
        with tempfile.TemporaryDirectory(dir=output) as temp:
            root = Path(temp); frames = policy_frames(CAPACITY + 3, base_ns + 7_000_000_000)
            ring = deque(maxlen=CAPACITY)
            history = ControlledHistory([], capacity=CAPACITY)
            history.frames = frames
            history.calls = 0
            published = 0
            def keeping_up(*, cursor: int = 0, expected_session: int | None = None,
                           max_source_sequence: int | None = None, **_kwargs: Any) -> dict:
                nonlocal published
                history.calls += 1
                produced = min(history.calls, len(frames))
                while published < produced:
                    ring.append((published, pack_frame(frames[published])))
                    published += 1
                oldest = max(0, produced-CAPACITY)
                if cursor < oldest:
                    return {"status":"OVERFLOW","session_id":SESSION,"produced_count":produced,
                        "overwritten_count":oldest,"consumer_unread_loss_count":oldest-cursor,
                        "consumer_unread_loss_range":[cursor,oldest],"frames":[],"cursor":cursor}
                entries = list(ring); selected=[]; next_cursor=cursor
                for idx, raw_frame in entries:
                    if idx < cursor: continue
                    f=RuntimeFrame.from_bytes(raw_frame)
                    if max_source_sequence is not None and f.sequence > max_source_sequence: break
                    selected.append((idx,raw_frame)); next_cursor=idx+1
                return {"status":"OK","session_id":SESSION,"produced_count":produced,
                    "overwritten_count":oldest,"frames":selected,"cursor":next_cursor}
            old = s1_run_diagnostic.read_shm_history; s1_run_diagnostic.read_shm_history=keeping_up
            capture = new_capture(root / "slot_reuse.jsonl", "slot_reuse"); capture.start()
            for frame in frames:
                raw, checked = live(frame)
                item = capture.observe(raw, FrameStatus.LIVE, checked, checked.monotonic_ns+1)
                assert item.get("persisted") is True
            terminal_ref = capture.last_frame_metadata
            capture.mark_task_terminal(terminal_ref)
            final = capture.finish({"exit_code":0,"forced_termination":False,
                "shutdown_complete":True,"shutdown_request_source":"slot reuse test"})
            s1_run_diagnostic.read_shm_history=old
            hist=final["policy_cycle_history"]
            assert hist["producer_slot_reuse_count"] == 3
            assert hist["consumer_unread_loss_count"] == 0
            assert hist["final_closeout_drain"]["complete"] is True
            result["cases"]["consumer_keeps_up_past_capacity"] = {
                "produced": hist["producer_count_at_last_drain"], "cursor": hist["consumer_cursor"],
                "slot_reuse": hist["producer_slot_reuse_count"],
                "unread_loss": hist["consumer_unread_loss_count"],
                "persisted_frames": hist["runtime_record_unique_frames"],
                "final_complete": hist["final_closeout_drain"]["complete"]}

        # A write failure still invokes process cleanup; missing terminal frame
        # remains rejected by the production analyzer.
        with tempfile.TemporaryDirectory(dir=output) as temp:
            root = Path(temp); frames = policy_frames(2, base_ns + 8_000_000_000)
            history = ControlledHistory(frames)
            old = install_controlled(history)
            capture = new_capture(root / "write_failure.jsonl", "write_failure"); capture.start()
            terminal_raw, terminal_checked = live(frames[1])
            original_record = capture.recorder.record_snapshot
            def fail_terminal(raw: bytes, now_ns: int | None = None):
                if RuntimeFrame.from_bytes(raw).rl_step == 1:
                    raise OSError("injected terminal frame write failure")
                return original_record(raw, now_ns)
            capture.recorder.record_snapshot = fail_terminal
            observation = capture.observe(terminal_raw, FrameStatus.LIVE, terminal_checked,
                                          terminal_checked.monotonic_ns+1)
            assert observation["persisted"] is False
            ref = {"session_id": SESSION, "rl_step": 1, "source_sequence": terminal_checked.sequence,
                   "monotonic_ns": terminal_checked.monotonic_ns, "sim_time_s": terminal_checked.sim_time_s,
                   "same_snapshot_persisted": False}
            context = valid_context("write_failure", resolve_scene("scene_obstacle.xml"), ref, frames[1])
            facts_ = facts(ref, frames[1]); cleanup=[]
            terminal, errors = finalize_run_closeout(root, context, facts_, capture, True,
                lambda: cleanup.append("called"))
            s1_run_diagnostic.read_shm_history=old
            assert cleanup == ["called"] and terminal is not None and errors == []
            metrics = calculate(root / "write_failure.jsonl", root / "run_context.json")
            assert metrics["record_validity"] == "INVALID", metrics
            result["cases"]["terminal_write_failure_still_cleans_up"] = {
                "cleanup_called": cleanup == ["called"],
                "merge_error": terminal["policy_cycle_history"]["merge_error"],
                "analysis_validity": metrics["record_validity"],
                "analysis_reason": metrics["record_validity_reasons"]}

        # Deduplication must preserve the exact LIVE safety payload already
        # stored beside terminal policy frame N. These scenarios run through
        # real v4 decoding, JSONL writing, terminal closeout and analysis.
        collision = {"start_physics_step": 410, "end_physics_step": 410,
            "start_sim_time": 5.02, "end_sim_time": 5.02,
            "robot_geom_id": 42, "obstacle_geom_id": 7,
            "robot_geom_name": "RL_thigh", "obstacle_geom_name": "test_obstacle"}
        case_dir = output / "dedupe_nonfoot_collision"
        if case_dir.exists():
            shutil.rmtree(case_dir)
        case = run_deduplicated_safety_case(case_dir, "duplicate_nonfoot_collision",
                                            collision_event=collision)
        assert case["terminal"] == "COLLISION_TERMINATION"
        assert case["closeout_gate"]["eligible"] is False
        assert case["analyzed_terminal"] == "COLLISION_TERMINATION"
        result["cases"]["dedupe_preserves_nonfoot_collision_and_priority"] = {
            key: value for key, value in case.items() if key not in ("metrics", "analyzed_safety")}

        startup_closed = {"start_physics_step": 250, "end_physics_step": 390,
            "confirmed_physics_step": 275, "start_sim_time": 0.25,
            "end_sim_time": 4.90, "confirmed_sim_time": 0.55,
            "confirmed": True, "closed": True}
        task_open = {"start_physics_step": 410, "end_physics_step": 410,
            "confirmed_physics_step": 0, "start_sim_time": 5.02,
            "end_sim_time": 5.02, "confirmed_sim_time": 0.0,
            "confirmed": False, "closed": False}
        task_closed = dict(task_open, closed=True)
        fall_updates = [[startup_closed], [startup_closed, task_open],
                        [startup_closed, task_closed], [startup_closed, task_closed]]
        case_dir = output / "same_step_fall_close"
        if case_dir.exists():
            shutil.rmtree(case_dir)
        case = run_deduplicated_safety_case(case_dir, "same_step_fall_close",
                                            fall_updates=fall_updates)
        assert case["terminal"] == "ARRIVED"
        assert case["fall"] is False
        assert case["fall_result_basis"].startswith("closed_unconfirmed_posture_candidates")
        assert case["closeout_gate"]["eligible"] is True
        result["cases"]["same_end_step_close_adopted_startup_excluded"] = {
            key: value for key, value in case.items() if key not in ("metrics", "analyzed_safety")}

        confirmed_task_fall = dict(task_open, end_physics_step=420, end_sim_time=5.04,
                                   confirmed=True, closed=False,
                                   confirmed_physics_step=415, confirmed_sim_time=5.03)
        confirmed_updates = [[startup_closed], [startup_closed, confirmed_task_fall],
                             [startup_closed, confirmed_task_fall], [startup_closed, confirmed_task_fall]]
        case_dir = output / "duplicate_confirmed_fall"
        if case_dir.exists():
            shutil.rmtree(case_dir)
        case = run_deduplicated_safety_case(case_dir, "duplicate_confirmed_fall",
                                            fall_updates=confirmed_updates)
        assert case["terminal"] == "FALL_TERMINATION"
        assert case["fall"] is True
        assert case["closeout_gate"]["eligible"] is False
        result["cases"]["dedupe_preserves_confirmed_fall"] = {
            key: value for key, value in case.items() if key not in ("metrics", "analyzed_safety")}

        post_terminal_updates = [[startup_closed], [startup_closed, task_open],
                                 [startup_closed, dict(task_open, end_physics_step=420,
                                                       end_sim_time=5.04)],
                                 [startup_closed, dict(task_open, end_physics_step=420,
                                                       end_sim_time=5.04, closed=True)]]
        case_dir = output / "fall_closes_after_terminal"
        if case_dir.exists():
            shutil.rmtree(case_dir)
        case = run_deduplicated_safety_case(case_dir, "fall_closes_after_terminal",
                                            fall_updates=post_terminal_updates,
                                            close_after_terminal=True)
        assert case["fall"] == "UNKNOWN"
        assert case["fall_result_basis"] == "candidate_active_at_terminal_boundary"
        result["cases"]["post_terminal_close_remains_censored"] = {
            key: value for key, value in case.items() if key not in ("metrics", "analyzed_safety")}

        live_healthy = {"status": "LIVE", "unknown_contacts": 0,
                        "fall_candidate": False, "fall_confirmed": False,
                        "robot_obstacle_contacts": 0}
        assert normal_closeout_gate("ARRIVED", False, [], False, live_healthy) == (True, None)
        for unavailable in (None, {**live_healthy, "status": "MISSING"},
                            {**live_healthy, "status": "STALE"},
                            {**live_healthy, "status": "INVALID"}):
            assert normal_closeout_gate("ARRIVED", False, [], False, unavailable)[0] is False
        result["cases"]["healthy_gate_and_unavailable_rejections"] = {
            "healthy_live_eligible": True, "missing_stale_invalid_rejected": True}

        case_dir = output / "dedupe_missing_safety"
        if case_dir.exists():
            shutil.rmtree(case_dir)
        case = run_deduplicated_safety_case(case_dir, "duplicate_missing_safety",
                                            missing_safety=True)
        assert case["dedupe_status"] == "already_persisted"
        assert case["snapshot_identity"]["status"] == "MISSING"
        assert case["closeout_gate"]["eligible"] is False
        assert case["analyzed_safety"]["collision"] == "UNKNOWN"
        assert case["analyzed_safety"]["fall"] == "UNKNOWN"
        result["cases"]["dedupe_missing_safety_stays_unknown"] = {
            key: value for key, value in case.items() if key not in ("metrics", "analyzed_safety")}
        result["cases"]["dedupe_missing_safety_stays_unknown"]["analyzed_collision"] = \
            case["analyzed_safety"]["collision"]
        result["cases"]["dedupe_missing_safety_stays_unknown"]["analyzed_fall"] = \
            case["analyzed_safety"]["fall"]
    finally:
        s1_run_diagnostic.read_shm_history = original_reader
        run_record.read_collision_snapshot = old_collision
        run_record.read_base_bounds_snapshot = old_bounds

    summary_path = output / "summary.json"
    summary_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("logs/b1/offline_merge_validation")
    summary = main(target)
    print(json.dumps({"status": "PASS", "summary": str(target / "summary.json"),
                      "cases": list(summary["cases"])}))
