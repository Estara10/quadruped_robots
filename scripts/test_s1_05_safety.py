#!/usr/bin/env python3
"""Targeted v3/v4 collision, motion-boundary, and closeout checks for S1-05."""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import abs_collision
from abs_collision import CollisionStatus
from s1_analyze_run import motion_safety
from s1_run_diagnostic import finalize_run_closeout, select_strategy_terminal
from run_record import (RunRecordRecorder, collision_snapshot_payload,
                        _validate_collision_payload)


def snapshot_bytes(episode: bool = False, unknown: bool = False, invalid_posture: bool = False,
                  version: int = 4) -> bytes:
    capture = "p1-10-capture-" + "a" * 32
    fingerprint = "b" * 64
    base = abs_collision.BASE_STRUCT.pack(
        abs_collision.MAGIC, version, 2, 100, 10, 0.20,
        1, 0, 0, 2, 0, 0, 2, 0, 0, 2, -1, -1, 0,
        abs_collision.SCENARIO_ID.encode(),
        abs_collision.SCENE_ROOT_SHA256.encode(),
        abs_collision.MODEL_CLOSURE_SHA256.encode(),
        capture.encode(), fingerprint.encode())[:-4]
    count = int(episode)
    extension = abs_collision.EXTENSION_STRUCT.pack(
        2, 0,                 # foot / non-foot floor contacts
        int(episode), count, int(episode) * 5, int(episode) * 5,
        0.02 if episode else 0.0, 0.10 if episode else 0.0,
        0.10 if episode else 0.0, 0.445, 0.0, 0.0, 0.0, 0.0,
        0, 0,                 # fall start / confirmed source step
        0, 0, count, 0, 1, 0) # fall flags, history, overflow, coverage, reserved
    extra = abs_collision.V4_EXTENSION_STRUCT.pack(
        int(unknown), int(unknown), 11 if unknown else 0, 11 if unknown else 0,
        int(invalid_posture), int(invalid_posture), 12 if invalid_posture else 0,
        12 if invalid_posture else 0,
        0.11 if unknown else 0.0, 0.11 if unknown else 0.0,
        0.12 if invalid_posture else 0.0, 0.12 if invalid_posture else 0.0,
        0, 0)
    fall_events = [bytes(abs_collision.FALL_EVENT_STRUCT.size)] * abs_collision.FALL_EVENT_CAPACITY
    events = []
    if episode:
        events.append(abs_collision.EPISODE_STRUCT.pack(
            5, 5, 0.10, 0.10, 42, 7, b"FL", b"obstacle_1"))
    events.extend([bytes(abs_collision.EPISODE_STRUCT.size)] *
                  (abs_collision.COLLISION_EVENT_CAPACITY - len(events)))
    raw = base + extension + extra + b"".join(fall_events) + b"".join(events)
    if version == abs_collision.VERSION_V3:
        raw = base + extension + b"".join(events)
    assert len(raw) == (abs_collision.SNAPSHOT_SIZE if version == 4 else abs_collision.LEGACY_SNAPSHOT_SIZE)
    return raw


def test_v4_decode_and_payload() -> None:
    status, snapshot = abs_collision.classify_snapshot(snapshot_bytes(True), 110)
    assert status is CollisionStatus.LIVE and snapshot is not None
    assert snapshot.version == 4 and snapshot.layout_size == 5760
    assert snapshot.collision_history[0]["robot_geom_name"] == "FL"
    payload = collision_snapshot_payload(status, snapshot)
    assert not _validate_collision_payload(payload), _validate_collision_payload(payload)
    bad = dict(payload, physics_coverage_complete=False)
    assert not _validate_collision_payload(bad)  # valid data, but coverage becomes conservative UNKNOWN


def test_v3_legacy_layout_and_rejections() -> None:
    status, snapshot = abs_collision.classify_snapshot(snapshot_bytes(True, version=3), 110)
    assert status is CollisionStatus.LIVE and snapshot is not None
    assert snapshot.version == 3 and snapshot.layout_size == 3864
    assert snapshot.event_latches_complete is False and snapshot.fall_history == ()
    payload = collision_snapshot_payload(status, snapshot)
    assert not _validate_collision_payload(payload), _validate_collision_payload(payload)
    clean = {"status": "LIVE", "version": 3, "physics_coverage_complete": True,
             "collision_history_overflow": False, "fall_history_overflow": False,
             "event_latches_complete": False, "sim_time": 2.0,
             "collision_history": [], "fall_history": [], "unknown_contacts": 0,
             "unknown_contact_steps": 0, "invalid_posture_steps": 0,
             "first_unknown_contact_sim_time": 0, "last_unknown_contact_sim_time": 0,
             "first_invalid_posture_sim_time": 0, "last_invalid_posture_sim_time": 0}
    assert motion_safety([{"collision_snapshot": {**clean, "sim_time": 0.5}},
                          {"collision_snapshot": clean}], 1.0, 2.0)["collision"] == "UNKNOWN"
    mismatched_v3 = bytearray(snapshot_bytes(True, version=3))
    mismatched_v3[8:16] = (4).to_bytes(8, "little")
    mismatched_v4 = bytearray(snapshot_bytes(True))
    mismatched_v4[8:16] = (3).to_bytes(8, "little")
    for wrong in (bytes(mismatched_v3), bytes(mismatched_v4), snapshot_bytes(True)[:-1]):
        assert abs_collision.classify_snapshot(wrong, 110)[0] is CollisionStatus.INVALID
    unknown_version = bytearray(snapshot_bytes(True))
    unknown_version[8:16] = (99).to_bytes(8, "little")
    assert abs_collision.classify_snapshot(bytes(unknown_version), 110)[0] is CollisionStatus.INVALID
    with tempfile.TemporaryDirectory(prefix="s1-05-truncated-shm-") as directory:
        truncated = Path(directory) / "authority"
        truncated.write_bytes(bytes(abs_collision.SNAPSHOT_SIZE - 1))
        raw = abs_collision.read_collision_snapshot(str(truncated))
        assert abs_collision.classify_snapshot(raw, 110)[0] is CollisionStatus.INVALID


def test_physics_step_latches_reach_normal_reader() -> None:
    status, snapshot = abs_collision.classify_snapshot(
        snapshot_bytes(unknown=True, invalid_posture=True), 110)
    assert status is CollisionStatus.LIVE and snapshot is not None
    assert snapshot.unknown_contact_steps == 1 and snapshot.invalid_posture_steps == 1
    payload = collision_snapshot_payload(status, snapshot)
    assert payload["event_latches_complete"] is True
    assert payload["first_unknown_contact_sim_time"] == 0.11
    assert payload["first_invalid_posture_sim_time"] == 0.12
    assert not _validate_collision_payload(payload)


def test_motion_scope_and_unknown_coverage() -> None:
    clean = {
        "status": "LIVE", "version": 4, "physics_coverage_complete": True,
        "sim_time": 1.5,
        "collision_history_overflow": False, "fall_history_overflow": False,
        "event_latches_complete": True,
        "unknown_contacts": 0, "unknown_contact_steps": 0,
        "invalid_posture_steps": 0, "first_unknown_contact_sim_time": 0,
        "last_unknown_contact_sim_time": 0, "first_invalid_posture_sim_time": 0,
        "last_invalid_posture_sim_time": 0,
        "collision_history": [], "fall_confirmed": False,
        "fall_history": [],
        "foot_ground_contacts": 4, "nonfoot_ground_contacts": 0, "self_contacts": 0,
    }
    startup_contact = {**clean, "collision_history": [
        {"start_physics_step": 1, "end_physics_step": 2,
         "start_sim_time": 0.10, "end_sim_time": 0.12}]}
    motion_contact = {**clean, "collision_history": [
        {"start_physics_step": 5, "end_physics_step": 9,
         "start_sim_time": 1.20, "end_sim_time": 1.35}]}
    def covered(snapshot):
        return [{"collision_snapshot": {**snapshot, "sim_time": 0.8}},
                {"collision_snapshot": {**snapshot, "sim_time": 2.0}}]
    safe = motion_safety(covered(clean), 1.0, 2.0)
    assert safe["collision"] is False and safe["fall"] is False
    startup = motion_safety(covered(startup_contact), 1.0, 2.0)
    assert startup["collision"] is False  # startup episode excluded
    hit = motion_safety(covered(motion_contact), 1.0, 2.0)
    assert hit["collision"] is True
    incomplete = motion_safety([{"collision_snapshot": {**clean, "physics_coverage_complete": False,
                                                         "sim_time": 2.0}}], 1.0, 2.0)
    assert incomplete["collision"] == "UNKNOWN" and incomplete["fall"] == "UNKNOWN"
    confirmed_fall = {**clean, "fall_confirmed": True,
                      "fall_start_sim_time": 1.2, "fall_confirmed_sim_time": 1.5,
                      "fall_start_physics_step": 50, "fall_confirmed_physics_step": 65,
                      "fall_history": [{"start_physics_step": 50, "end_physics_step": 65,
                          "start_sim_time": 1.2, "end_sim_time": 1.5,
                          "confirmed_sim_time": 1.5, "confirmed_physics_step": 65,
                          "confirmed": True, "closed": False}]}
    fall = motion_safety(covered(confirmed_fall), 1.0, 2.0)
    assert fall["fall"] is True and fall["collision"] is False

    startup_then_motion_fall = {**clean, "fall_history": [
        {"start_physics_step": 2, "end_physics_step": 80, "start_sim_time": 0.2,
         "end_sim_time": 1.4, "confirmed_sim_time": 0.5,
         "confirmed_physics_step": 20, "confirmed": True, "closed": True},
        {"start_physics_step": 90, "end_physics_step": 120, "start_sim_time": 1.5,
         "end_sim_time": 2.2, "confirmed_sim_time": 1.8,
         "confirmed_physics_step": 105, "confirmed": True, "closed": True}]}
    fall = motion_safety(covered(startup_then_motion_fall), 1.0, 2.0)
    assert fall["fall"] is True and len(fall["fall_events"]) == 2
    crossing = {**clean, "fall_history": [
        {"start_physics_step": 2, "end_physics_step": 80, "start_sim_time": 0.2,
         "end_sim_time": 1.4, "confirmed_sim_time": 0.5,
         "confirmed_physics_step": 20, "confirmed": True, "closed": True}]}
    assert motion_safety(covered(crossing), 1.0, 2.0)["fall"] is True
    unresolved = {**clean, "fall_history": [
        {"start_physics_step": 2, "end_physics_step": 80, "start_sim_time": 0.2,
         "end_sim_time": 1.4, "confirmed_sim_time": 0.0,
         "confirmed_physics_step": 0, "confirmed": False, "closed": False}]}
    assert motion_safety(covered(unresolved), 1.0, 2.0)["fall"] == "UNKNOWN"
    latched_contact = {**clean, "unknown_contact_steps": 1,
                       "first_unknown_contact_sim_time": 1.2,
                       "last_unknown_contact_sim_time": 1.2}
    assert motion_safety(covered(latched_contact), 1.0, 2.0)["collision"] == "UNKNOWN"
    latched_invalid = {**clean, "invalid_posture_steps": 1,
                       "first_invalid_posture_sim_time": 1.2,
                       "last_invalid_posture_sim_time": 1.2}
    assert motion_safety(covered(latched_invalid), 1.0, 2.0)["fall"] == "UNKNOWN"


def test_closeout_error_does_not_skip_cleanup() -> None:
    class RecorderFile:
        closed = False
        def close(self):
            self.closed = True

    class Capture:
        def __init__(self):
            self.recorder = RecorderFile()
        def finish(self, facts):
            raise OSError("synthetic terminal write error")

    with tempfile.TemporaryDirectory(prefix="s1-05-closeout-") as directory:
        capture = Capture()
        cleaned = []
        context, facts = {}, {"shutdown_complete": True, "cleanup_errors": []}
        _, errors = finalize_run_closeout(Path(directory), context, facts, capture, True,
                                           lambda: cleaned.append("done"))
        assert cleaned == ["done"]
        assert capture.recorder.closed
        assert any(item["stage"] == "recorder_finalize" for item in errors)
        assert context["lifecycle"]["closeout_errors"]
        assert (Path(directory) / "run_context.json").is_file()


def test_same_observation_terminal_priority() -> None:
    assert select_strategy_terminal(True, True, True, True, True) == "SYSTEM_SAFETY_ABORT"
    assert select_strategy_terminal(False, True, True, True, True) == "COLLISION_TERMINATION"
    assert select_strategy_terminal(False, False, True, True, True) == "FALL_TERMINATION"
    assert select_strategy_terminal(False, False, False, True, True) == "ARRIVED"
    assert select_strategy_terminal(False, False, False, False, True) == "DIAGNOSTIC_SIM_TIME_LIMIT"


def test_terminal_snapshot_preserved_without_policy_frame() -> None:
    status, snapshot = abs_collision.classify_snapshot(snapshot_bytes(True), 110)
    assert status is CollisionStatus.LIVE and snapshot is not None
    payload = collision_snapshot_payload(status, snapshot)
    with tempfile.TemporaryDirectory(prefix="s1-05-terminal-snapshot-") as directory:
        path = Path(directory) / "runtime_record.jsonl"
        recorder = RunRecordRecorder(str(path), run_id="s1-05-fixture",
                                     capture_id=snapshot.capture_id,
                                     expected_fingerprint=snapshot.runtime_model_fingerprint)
        recorder.start()
        terminal = recorder.finalize({
            "exit_code": 0, "forced_termination": False, "shutdown_complete": True,
            "run_terminal_result": "SYSTEM_SAFETY_ABORT", "run_terminal_monotonic_ns": 111,
            "external_safety_events": [{
                "event": "POLICY_SAFETY_VETO", "source": "synthetic source-path fixture",
                "message": "test event", "observed_monotonic_ns": 110,
                "collision_snapshot": payload,
            }],
        })
        assert terminal["collision_events"] is True
        assert terminal["collision_coverage"]["last_snapshot"] == payload


if __name__ == "__main__":
    test_v4_decode_and_payload()
    test_v3_legacy_layout_and_rejections()
    test_physics_step_latches_reach_normal_reader()
    test_motion_scope_and_unknown_coverage()
    test_same_observation_terminal_priority()
    test_closeout_error_does_not_skip_cleanup()
    print("S1-05 targeted safety checks: PASS")
