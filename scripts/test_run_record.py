#!/usr/bin/env python3
"""Offline mechanical tests for the per-run record chain (P1-09 runtime record).

No MuJoCo, ROS2, benchmark, formal episode, pilot, or real robot is run here.
Fixtures are test-only frames. The record is fail-closed: ANY present
non-LIVE / malformed / non-authoritative frame, any continuity break, run
identity mismatch, non-unique / misplaced terminal, negative duration, or
malformed process fact invalidates the whole record.
"""

import json
import math
import struct
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

from abs_rt_frame import (  # noqa: E402
    RuntimeFrame,
    SOURCE_AUTHORITATIVE_RUNTIME,
    SOURCE_SYNTHETIC_TEST,
)
from run_record import (  # noqa: E402
    RunRecordRecorder,
    frame_availability,
    frame_payload,
    load_record,
    summarize_record,
)

# Mirror "<7Q11I81f" (see abs_rt_frame._FRAME_STRUCT).
_FRAME_STRUCT = struct.Struct("<7Q11I81f")


def pack_frame(frame: RuntimeFrame) -> bytes:
    ints = [
        frame.magic, frame.version, frame.sequence, frame.monotonic_ns,
        frame.session_id, frame.rl_step, frame.ray_age_ns,
    ]
    flags = [
        frame.source, frame.controller_active, frame.rl_entered, frame.rl_active,
        frame.safety_faulted, frame.policy_state, frame.ray_origin, frame.ray_valid,
        frame.collision_origin, frame.torque_saturated_computed, frame.reserved_pad,
    ]
    floats = [
        frame.ra_value,
        *frame.lin_vel, *frame.command, *frame.world_pose,
        *frame.ray2d,
        *frame.action_raw, *frame.action_clipped,
        *frame.joint_target_rad, *frame.torque_nm, *frame.torque_saturated,
    ]
    return _FRAME_STRUCT.pack(*ints, *flags, *floats)


def fixture(
    *,
    source=SOURCE_AUTHORITATIVE_RUNTIME,
    sequence=2,
    session_id=7,
    rl_step=1,
    monotonic_ns=100,
    rl_active=1,
    safety_faulted=0,
    policy_state=0,
    ra_value=0.1,
    lin_vel=(0.3, 0.4, 0.0),
    torque_saturated_computed=0,
):
    return RuntimeFrame(
        magic=0x414253525446524D, version=1, sequence=sequence,
        monotonic_ns=monotonic_ns, session_id=session_id, rl_step=rl_step,
        ray_age_ns=1, source=source, controller_active=1, rl_entered=1,
        rl_active=rl_active, safety_faulted=safety_faulted, policy_state=policy_state,
        ray_origin=1, ray_valid=1, collision_origin=0,
        torque_saturated_computed=torque_saturated_computed, reserved_pad=0,
        ra_value=ra_value, lin_vel=lin_vel, command=(0.5, 0.5, 0.1),
        world_pose=(1.0, 2.0, 0.3), ray2d=tuple(float(i) for i in range(11)),
        action_raw=tuple(float(i) for i in range(12)),
        action_clipped=tuple(float(i + 1) for i in range(12)),
        joint_target_rad=tuple(float(i + 2) for i in range(12)),
        torque_nm=tuple(float(i + 3) for i in range(12)),
        torque_saturated=tuple(0.0 for _ in range(12)),
    )


def _fresh(frame: RuntimeFrame) -> Tuple[RuntimeFrame, int]:
    return frame, frame.monotonic_ns + 1


def _record(path: str, samples: List[Tuple[RuntimeFrame, int]], process_facts=None):
    recorder = RunRecordRecorder(path)
    recorder.start()
    for frame, now_ns in samples:
        recorder.record_snapshot(pack_frame(frame), now_ns=now_ns)
    return recorder.finalize(process_facts)


# ----------------------------------------------------------- hand-crafted records
def _meta(run_id: str = "r") -> Dict[str, object]:
    return {"kind": "meta", "record_format_version": 2, "run_id": run_id,
            "source": "/dev/shm/mujoco_rt_frame", "created_at_ns": 1}


def _live_line(run_id: str = "r", *, sequence=2, rl_step=1, monotonic_ns=100,
               session_id=7, source=SOURCE_AUTHORITATIVE_RUNTIME,
               policy_state=0, safety_faulted=0) -> Dict[str, object]:
    f = fixture(sequence=sequence, rl_step=rl_step, monotonic_ns=monotonic_ns,
                session_id=session_id, source=source, policy_state=policy_state,
                safety_faulted=safety_faulted)
    return {"kind": "frame", "run_id": run_id, "status": "LIVE",
            "recorded_at_ns": monotonic_ns + 1, "payload": frame_payload(f),
            "availability": frame_availability(f)}


def _terminal(run_id: str = "r", **overrides: object) -> Dict[str, object]:
    terminal: Dict[str, object] = {"kind": "terminal", "run_id": run_id,
                                   "frames_observed": 1, "fact_validation_errors": []}
    terminal.update(overrides)
    return terminal


def _write_jsonl(path: str, lines: List[Dict[str, object]]) -> None:
    Path(path).write_text(
        "".join(json.dumps(line, sort_keys=True) + "\n" for line in lines),
        encoding="utf-8",
    )


# ----------------------------------------------------------------- positive paths
def test_recorder_writes_meta_frame_terminal():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        run_id = recorder.start()
        recorder.record_snapshot(pack_frame(fixture(monotonic_ns=100)), now_ns=101)
        recorder.record_snapshot(pack_frame(fixture(sequence=4, rl_step=2, monotonic_ns=110)), now_ns=111)
        recorder.finalize({"exit_code": 0, "forced_termination": False, "shutdown_complete": True})
        data = load_record(path)
        assert data.meta.get("run_id") == run_id
        assert len(data.frames) == 2
        assert data.terminal is not None
        assert data.terminal.get("run_id") == run_id
        assert data.terminal.get("frames_observed") == 2
        # every frame line carries the same run identity
        assert all(f.get("run_id") == run_id for f in data.frames)
        summary = summarize_record(path)
        assert summary["record_validity"] == "VALID"


def test_full_payload_round_trip():
    frame = fixture(sequence=2, rl_step=1, monotonic_ns=100, ra_value=-0.7,
                    lin_vel=(0.3, 0.4, -0.1))
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        recorder.start()
        line = recorder.record_snapshot(pack_frame(frame), now_ns=101)
        recorder.finalize()
        payload = line["payload"]
        assert math.isclose(payload["ra_value"], -0.7, abs_tol=1e-6)
        assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(payload["lin_vel"], (0.3, 0.4, -0.1)))
        assert payload["rl_step"] == 1
        assert payload["session_id"] == 7
        assert len(payload["ray2d"]) == 11
        assert len(payload["action_raw"]) == 12
        assert len(payload["action_clipped"]) == 12
        assert len(payload["joint_target_rad"]) == 12
        assert len(payload["torque_nm"]) == 12


def test_no_mock_fill_for_unavailable_fields():
    frame = fixture(monotonic_ns=100, torque_saturated_computed=0)
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        recorder.start()
        line = recorder.record_snapshot(pack_frame(frame), now_ns=101)
        recorder.finalize()
        assert line["status"] == "LIVE"
        assert line["availability"]["torque_saturated"] is False
        assert line["availability"]["collision"] is False
        assert line["payload"]["torque_saturated"] == [0.0] * 12  # raw bytes kept
        terminal = load_record(path).terminal
        assert terminal["collision_events"] == "UNKNOWN"
        assert terminal["fall_events"] == "UNKNOWN"
        assert terminal["reached_goal"] == "UNKNOWN"
        assert terminal["timeout"] == "UNKNOWN"
        assert terminal["simulation_time_s"] == "UNKNOWN"


def test_session_and_ordering_preserved():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _record(path, [
            _fresh(fixture(sequence=2, rl_step=1, monotonic_ns=100)),
            _fresh(fixture(sequence=8, rl_step=3, monotonic_ns=120)),
        ])
        data = load_record(path)
        assert [f["payload"]["session_id"] for f in data.frames] == [7, 7]
        assert [f["payload"]["source_sequence"] for f in data.frames] == [2, 8]
        assert [f["payload"]["rl_step"] for f in data.frames] == [1, 3]
        assert [f["payload"]["monotonic_ns"] for f in data.frames] == [100, 120]
        assert summarize_record(path)["record_validity"] == "VALID"


def test_clean_rc0_without_success_source_is_unknown():
    frames = [_fresh(fixture(monotonic_ns=100))]
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _record(path, frames, {"exit_code": 0, "forced_termination": False, "shutdown_complete": True})
        summary = summarize_record(path)
        assert summary["record_validity"] == "VALID"
        assert summary["authoritative_runtime_source"] is True
        assert summary["outcome"] == "UNKNOWN"
        assert "no_reached_goal_source" in summary["outcome_reasons"]
        assert summary["normal_shutdown"] is True
        assert summary["termination_reason"] == "FRAMES_ENDED_RC0"


def test_missing_only_record_not_authoritative():
    """MISSING gaps are tolerated, but with no LIVE frames there is no
    authoritative runtime source and the outcome is INVALID."""
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            {"kind": "frame", "run_id": "r", "status": "MISSING", "payload": None, "availability": None},
            _terminal("r"),
        ])
        summary = summarize_record(path)
        assert summary["record_validity"] == "VALID"
        assert summary["authoritative_runtime_source"] is False
        assert summary["live_frame_count"] == 0
        assert summary["outcome"] == "INVALID"
        assert "no_live_frames_in_record" in summary["outcome_reasons"]


def test_missing_process_facts_record_unknown():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _record(path, [_fresh(fixture(monotonic_ns=100))])
        summary = summarize_record(path)
        assert summary["record_validity"] == "VALID"
        assert summary["outcome"] == "UNKNOWN"
        assert summary["termination_reason"] == "UNKNOWN"
        assert summary["normal_shutdown"] is None  # UNKNOWN, not fabricatable
        assert summary["simulation_time_s"] == "UNKNOWN"


def test_summary_statistics_from_live_frames():
    samples = [
        _fresh(fixture(sequence=2, rl_step=1, monotonic_ns=100, ra_value=-0.1, lin_vel=(3.0, 4.0, 0.0))),
        _fresh(fixture(sequence=4, rl_step=2, monotonic_ns=110, ra_value=-0.2, lin_vel=(6.0, 8.0, 0.0), policy_state=1)),
        _fresh(fixture(sequence=6, rl_step=3, monotonic_ns=120, ra_value=-0.3, lin_vel=(0.0, 0.0, 0.0))),
    ]
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _record(path, samples)
        summary = summarize_record(path)
        assert summary["record_validity"] == "VALID"
        assert math.isclose(summary["velocity"]["horizontal_speed_avg_m_s"], 5.0, abs_tol=1e-6)
        assert math.isclose(summary["velocity"]["horizontal_speed_peak_m_s"], 10.0, abs_tol=1e-6)
        assert math.isclose(summary["ra_statistics"]["mean"], -0.2, abs_tol=1e-5)
        assert math.isclose(summary["ra_statistics"]["min"], -0.3, abs_tol=1e-5)
        assert math.isclose(summary["ra_statistics"]["max"], -0.1, abs_tol=1e-5)
        assert summary["recovery_usage"]["recovery_steps"] == 1
        assert summary["recovery_usage"]["recovery_fraction"] == 1 / 3
        assert summary["recovery_usage"]["transitions"] == 2
        assert summary["attitude_yaw"]["recorded"] is True
        assert math.isclose(summary["attitude_yaw"]["yaw_deg"]["mean"], math.degrees(0.3), abs_tol=1e-6)
        assert summary["collision"]["available"] is False
        assert summary["collision"]["event_count"] is None
        assert summary["outcome"] == "UNKNOWN"


# ----------------------------------------------------------------- fail-closed record
def test_live_plus_bad_status_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line("r", monotonic_ns=100),
            {"kind": "frame", "run_id": "r", "status": "INVALID", "payload": None, "availability": None},
            _terminal("r"),
        ])
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert any("non_live_frame_status:1:INVALID" in reason for reason in summary["record_validity_reasons"])
        assert summary["outcome"] == "INVALID"


def test_live_plus_malformed_payload_invalid():
    """A LIVE-status frame with no payload is malformed and invalidates."""
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line("r", monotonic_ns=100),
            {"kind": "frame", "run_id": "r", "status": "LIVE", "payload": None, "availability": None},
            _terminal("r"),
        ])
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert any("malformed_frame_payload" in reason for reason in summary["record_validity_reasons"])


def test_live_plus_synthetic_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        recorder.start()
        recorder.record_snapshot(pack_frame(fixture(monotonic_ns=100)), now_ns=101)
        line = recorder.record_snapshot(pack_frame(fixture(source=SOURCE_SYNTHETIC_TEST, sequence=4, rl_step=2, monotonic_ns=110)), now_ns=111)
        recorder.finalize({"exit_code": 0, "forced_termination": False, "shutdown_complete": True})
        assert line["status"] == "SYNTHETIC"
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert any("non_live_frame_status:1:SYNTHETIC" in reason for reason in summary["record_validity_reasons"])
        assert summary["outcome"] == "INVALID"


def test_live_plus_stale_invalid():
    """A STALE frame present in the record invalidates the whole record."""
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _record(path, [
            _fresh(fixture(monotonic_ns=100)),
            (fixture(sequence=4, rl_step=2, monotonic_ns=9_000_000_000), 9_600_000_000),
        ])
        data = load_record(path)
        assert data.frames[1]["status"] == "STALE"
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert any("non_live_frame_status:1:STALE" in reason for reason in summary["record_validity_reasons"])
        assert summary["live_frame_count"] == 1  # informational only; record still INVALID
        assert summary["outcome"] == "INVALID"


def test_non_authoritative_source_never_runtime_outcome():
    frame = fixture(source=SOURCE_SYNTHETIC_TEST, monotonic_ns=100)
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        recorder.start()
        line = recorder.record_snapshot(pack_frame(frame), now_ns=101)
        recorder.finalize({"exit_code": 0, "forced_termination": False, "shutdown_complete": True})
        assert line["status"] == "SYNTHETIC"
        assert line["payload"] is None
        summary = summarize_record(path)
        assert summary["authoritative_runtime_source"] is False
        assert summary["outcome"] == "INVALID"


# ------------------------------------------------------------- cross-frame continuity
def test_session_change_invalid():
    samples = [
        _fresh(fixture(sequence=2, session_id=7, rl_step=1, monotonic_ns=100)),
        _fresh(fixture(sequence=4, session_id=8, rl_step=2, monotonic_ns=110)),
    ]
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _record(path, samples)
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert "session_id_changed" in summary["record_validity_reasons"]
        assert summary["outcome"] == "INVALID"


def test_source_sequence_rollback_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line("r", sequence=4, rl_step=1, monotonic_ns=100),
            _live_line("r", sequence=2, rl_step=2, monotonic_ns=110),
            _terminal("r"),
        ])
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert "source_sequence_not_strictly_increasing" in summary["record_validity_reasons"]


def test_rl_step_rollback_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line("r", sequence=2, rl_step=3, monotonic_ns=100),
            _live_line("r", sequence=4, rl_step=1, monotonic_ns=110),
            _terminal("r"),
        ])
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert "rl_step_not_strictly_increasing" in summary["record_validity_reasons"]


def test_monotonic_rollback_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line("r", sequence=2, rl_step=1, monotonic_ns=120),
            _live_line("r", sequence=4, rl_step=2, monotonic_ns=100),
            _terminal("r"),
        ])
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert "monotonic_time_not_strictly_increasing" in summary["record_validity_reasons"]


def test_negative_duration_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line("r", monotonic_ns=100),
            _terminal("r", duration_ns=-5),
        ])
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert "negative_duration" in summary["record_validity_reasons"]


# ---------------------------------------------------------------------- safety fault
def test_safety_fault_drives_failure_outcome():
    samples = [
        _fresh(fixture(sequence=2, rl_step=1, monotonic_ns=100)),
        _fresh(fixture(sequence=4, rl_step=2, monotonic_ns=110, rl_active=0,
                       safety_faulted=1, policy_state=2)),
    ]
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _record(path, samples, {"exit_code": 0, "forced_termination": False, "shutdown_complete": True})
        summary = summarize_record(path)
        assert summary["record_validity"] == "VALID"
        assert summary["outcome"] == "FAILURE"
        assert "safety_fault" in summary["outcome_reasons"]
        assert summary["termination_reason"] == "SAFETY_FAULT"
        assert summary["safety_faults"]["faulted"] is True


def test_faulted_policy_state_without_safety_flag_is_failure():
    """policy_state == FAULTED with safety_faulted == 0 is still a safety fault."""
    samples = [
        _fresh(fixture(sequence=2, rl_step=1, monotonic_ns=100)),
        _fresh(fixture(sequence=4, rl_step=2, monotonic_ns=110, rl_active=0,
                       safety_faulted=0, policy_state=2)),
    ]
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        recorder.start()
        for frame, now_ns in samples:
            recorder.record_snapshot(pack_frame(frame), now_ns=now_ns)
        recorder.finalize({"exit_code": 0, "forced_termination": False, "shutdown_complete": True})
        terminal = load_record(path).terminal
        assert terminal["safety_fault_seen"] is True
        summary = summarize_record(path)
        assert summary["record_validity"] == "VALID"
        assert summary["outcome"] == "FAILURE"
        assert "safety_fault" in summary["outcome_reasons"]
        assert summary["safety_faults"]["faulted"] is True
        assert summary["termination_reason"] == "SAFETY_FAULT"


# ------------------------------------------------------------------ process facts
def test_forced_and_nonzero_exit_failure():
    frames = [_fresh(fixture(monotonic_ns=100))]
    with tempfile.TemporaryDirectory() as tmp:
        forced_path = str(Path(tmp) / "forced.jsonl")
        _record(forced_path, frames, {"exit_code": 143, "forced_termination": True, "shutdown_complete": False})
        forced = summarize_record(forced_path)
        assert forced["record_validity"] == "VALID"
        assert forced["outcome"] == "FAILURE"
        assert "forced_termination" in forced["outcome_reasons"]
        assert forced["termination_reason"] == "FORCED_TERMINATION"
        assert forced["normal_shutdown"] is False

        nonzero_path = str(Path(tmp) / "nonzero.jsonl")
        _record(nonzero_path, frames, {"exit_code": 3, "forced_termination": False, "shutdown_complete": False})
        nonzero = summarize_record(nonzero_path)
        assert nonzero["record_validity"] == "VALID"
        assert nonzero["outcome"] == "FAILURE"
        assert "nonzero_exit" in nonzero["outcome_reasons"]
        assert nonzero["termination_reason"] == "NONZERO_EXIT"


def test_malformed_process_fact_types_invalid_no_implicit_bool():
    """String 'false' must NOT become True; malformed fact types -> UNKNOWN + INVALID."""
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        recorder.start()
        recorder.record_snapshot(pack_frame(fixture(monotonic_ns=100)), now_ns=101)
        terminal = recorder.finalize({
            "exit_code": "0",            # str, not int
            "forced_termination": "false",  # str, must NOT become True
            "shutdown_complete": 1,      # int, not bool
            "shutdown_request_source": 5,  # int, not str
        })
        assert terminal["forced_termination"] is None  # never True
        assert terminal["process_exit_code"] is None
        assert terminal["shutdown_complete"] is None
        assert len(terminal["fact_validation_errors"]) == 4
        assert any("implicit conversion rejected" in e for e in terminal["fact_validation_errors"])
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert "malformed_process_facts" in summary["record_validity_reasons"]
        assert summary["outcome"] == "INVALID"


def test_valid_facts_with_proper_types_ok():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _record(path, [_fresh(fixture(monotonic_ns=100))],
                {"exit_code": 0, "forced_termination": False, "shutdown_complete": True,
                 "shutdown_request_source": "SIGINT"})
        summary = summarize_record(path)
        assert summary["record_validity"] == "VALID"
        assert summary["normal_shutdown"] is True
        assert summary["termination_reason"] == "FRAMES_ENDED_RC0"


# ------------------------------------------------------------------- run identity
def test_run_identity_mismatch_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("A"),
            _live_line("B", monotonic_ns=100),
            _terminal("A"),
        ])
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert any("frame_run_id_mismatch_or_missing:0" in reason for reason in summary["record_validity_reasons"])
        assert summary["outcome"] == "INVALID"


def test_terminal_run_id_mismatch_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("A"),
            _live_line("A", monotonic_ns=100),
            _terminal("B"),
        ])
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert "terminal_run_id_mismatch" in summary["record_validity_reasons"]


def test_duplicate_terminal_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line("r", monotonic_ns=100),
            _terminal("r"),
            _terminal("r"),
        ])
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert "terminal_not_unique" in summary["record_validity_reasons"]


def test_misplaced_terminal_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _terminal("r"),
            _live_line("r", monotonic_ns=100),
        ])
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert "terminal_not_at_record_boundary" in summary["record_validity_reasons"]


def test_missing_run_id_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        lines = [
            {"kind": "meta", "record_format_version": 2, "run_id": "r", "created_at_ns": 1},
            dict(_live_line("r", monotonic_ns=100), run_id=None),
            _terminal("r"),
        ]
        _write_jsonl(path, lines)
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert any("frame_run_id_mismatch_or_missing" in reason for reason in summary["record_validity_reasons"])


# ------------------------------------------------------------------ corrupt record
def test_corrupt_record_is_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        Path(path).write_text('{"kind": "meta", "run_id": "x"}\nnot-json\n', encoding="utf-8")
        summary = summarize_record(path)
        assert summary["record_validity"] == "INVALID"
        assert summary["outcome"] == "INVALID"
        assert any("line 2" in reason for reason in summary["record_validity_reasons"])


def test_duration_from_frames():
    samples = [
        _fresh(fixture(sequence=2, rl_step=1, monotonic_ns=100)),
        _fresh(fixture(sequence=4, rl_step=2, monotonic_ns=250)),
    ]
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _record(path, samples)
        summary = summarize_record(path)
        assert summary["record_validity"] == "VALID"
        assert summary["duration_ns"] == 150


# ------------------------------------------------------------------ payload schema
def _live_payload(**overrides: object) -> Dict[str, object]:
    """A well-formed LIVE payload with optional malformed overrides."""
    payload: Dict[str, object] = frame_payload(fixture())
    payload.update(overrides)
    return payload


def _live_line_payload(run_id: str = "r", payload: Optional[Dict[str, object]] = None,
                       status: object = "LIVE", availability: Optional[Dict[str, bool]] = None) -> Dict[str, object]:
    return {"kind": "frame", "run_id": run_id, "status": status, "recorded_at_ns": 101,
            "payload": payload,
            "availability": availability if availability is not None else
            (frame_availability(fixture()) if isinstance(payload, dict) else None)}


def _summary_no_raise(path: str):
    """Run the summary and return (summary, exception); the exception must be None."""
    try:
        return summarize_record(path), None
    except Exception as exc:  # noqa: BLE001 - explicit "must not crash" assertion
        return None, exc


# ----------------------------------------------------------------- unknown status
def test_live_plus_unknown_status_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line("r", monotonic_ns=100),
            _live_line_payload(run_id="r", payload=None, status="BOGUS"),
            _terminal("r"),
        ])
        summary, exc = _summary_no_raise(path)
        assert exc is None
        assert summary["record_validity"] == "INVALID"
        assert any("unknown_frame_status:1" in r for r in summary["record_validity_reasons"])
        assert summary["outcome"] != "SUCCESS"


def test_unknown_status_only_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line_payload(run_id="r", payload=None, status="BOGUS"),
            _terminal("r"),
        ])
        summary, exc = _summary_no_raise(path)
        assert exc is None
        assert summary["record_validity"] == "INVALID"
        assert any("unknown_frame_status:0" in r for r in summary["record_validity_reasons"])
        assert summary["outcome"] != "SUCCESS"


def test_wrong_type_status_invalid():
    for bad_status in (None, 5, ["LIVE"]):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "run.jsonl")
            _write_jsonl(path, [
                _meta("r"),
                _live_line_payload(run_id="r", payload=None, status=bad_status),
                _terminal("r"),
            ])
            summary, exc = _summary_no_raise(path)
            assert exc is None, f"status={bad_status!r}"
            assert summary["record_validity"] == "INVALID", f"status={bad_status!r}"
            assert any("unknown_frame_status" in r for r in summary["record_validity_reasons"]), f"status={bad_status!r}"
            assert summary["outcome"] != "SUCCESS", f"status={bad_status!r}"


def test_empty_string_status_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line_payload(run_id="r", payload=None, status=""),
            _terminal("r"),
        ])
        summary, exc = _summary_no_raise(path)
        assert exc is None
        assert summary["record_validity"] == "INVALID"
        assert any("unknown_frame_status:0:''" in r for r in summary["record_validity_reasons"])


# ------------------------------------------------------------- payload schema tests
def test_payload_missing_fields_invalid():
    for missing in ("lin_vel", "world_pose", "ra_value"):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "run.jsonl")
            payload = _live_payload()
            del payload[missing]  # type: ignore[misc]
            _write_jsonl(path, [_meta("r"), _live_line_payload(payload=payload), _terminal("r")])
            summary, exc = _summary_no_raise(path)
            assert exc is None, missing
            assert summary["record_validity"] == "INVALID", missing
            assert any(f"missing_field:{missing}" in r for r in summary["record_validity_reasons"]), missing
            assert summary["outcome"] != "SUCCESS", missing


def test_payload_wrong_scalar_type_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line_payload(payload=_live_payload(ra_value="not-a-number")),
            _terminal("r"),
        ])
        summary, exc = _summary_no_raise(path)
        assert exc is None
        assert summary["record_validity"] == "INVALID"
        assert any("ra_value_not_finite_number" in r for r in summary["record_validity_reasons"])


def test_payload_wrong_vector_length_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line_payload(payload=_live_payload(lin_vel=[0.0, 0.0])),
            _terminal("r"),
        ])
        summary, exc = _summary_no_raise(path)
        assert exc is None
        assert summary["record_validity"] == "INVALID"
        assert any("wrong_vector_length:lin_vel" in r for r in summary["record_validity_reasons"])


def test_payload_nan_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line_payload(payload=_live_payload(ra_value=float("nan"))),
            _terminal("r"),
        ])
        summary, exc = _summary_no_raise(path)
        assert exc is None
        assert summary["record_validity"] == "INVALID"
        assert any("ra_value_not_finite_number" in r for r in summary["record_validity_reasons"])


def test_payload_inf_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line_payload(payload=_live_payload(world_pose=[1.0, float("inf"), 0.3])),
            _terminal("r"),
        ])
        summary, exc = _summary_no_raise(path)
        assert exc is None
        assert summary["record_validity"] == "INVALID"
        assert any("vector_not_finite:world_pose[1]" in r for r in summary["record_validity_reasons"])


def test_payload_malformed_nested_structure_invalid():
    for field, bad in (("action_raw", {"a": 1}), ("command", "x"),
                       ("joint_target_rad", [0.0] * 5)):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "run.jsonl")
            _write_jsonl(path, [
                _meta("r"),
                _live_line_payload(payload=_live_payload(**{field: bad})),
                _terminal("r"),
            ])
            summary, exc = _summary_no_raise(path)
            assert exc is None, field
            assert summary["record_validity"] == "INVALID", field
            assert summary["outcome"] != "SUCCESS", field
            assert any(field in r for r in summary["record_validity_reasons"]), field


def test_non_object_json_lines_invalid():
    """A JSON array / number record line must not crash the summary."""
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        Path(path).write_text(
            json.dumps(_meta("r"), sort_keys=True) + "\n"
            + "[1, 2, 3]\n"
            + json.dumps(_terminal("r"), sort_keys=True) + "\n",
            encoding="utf-8",
        )
        summary, exc = _summary_no_raise(path)
        assert exc is None
        assert summary["record_validity"] == "INVALID"
        assert any("not a JSON object" in r for r in summary["record_validity_reasons"])


# ----------------------------------------------------------------- MISSING semantics
def test_live_plus_legitimate_missing_valid():
    """A MISSING gap with null payload AND null availability is a legal gap."""
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line("r", monotonic_ns=100),
            {"kind": "frame", "run_id": "r", "status": "MISSING", "payload": None, "availability": None},
            _terminal("r"),
        ])
        summary, exc = _summary_no_raise(path)
        assert exc is None
        assert summary["record_validity"] == "VALID"  # not invalid because of MISSING
        assert not any("malformed_missing_frame" in r for r in summary["record_validity_reasons"])
        assert summary["outcome"] != "SUCCESS"


def test_live_plus_missing_with_payload_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line("r", monotonic_ns=100),
            _live_line_payload(run_id="r", status="MISSING", payload=_live_payload(), availability=None),
            _terminal("r"),
        ])
        summary, exc = _summary_no_raise(path)
        assert exc is None
        assert summary["record_validity"] == "INVALID"
        assert any("malformed_missing_frame:1" in r for r in summary["record_validity_reasons"])
        assert summary["authoritative_runtime_source"] is False
        assert summary["outcome"] != "SUCCESS"


def test_live_plus_missing_with_availability_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line("r", monotonic_ns=100),
            _live_line_payload(run_id="r", status="MISSING", payload=None, availability=frame_availability(fixture())),
            _terminal("r"),
        ])
        summary, exc = _summary_no_raise(path)
        assert exc is None
        assert summary["record_validity"] == "INVALID"
        assert any("malformed_missing_frame:1" in r for r in summary["record_validity_reasons"])
        assert summary["authoritative_runtime_source"] is False
        assert summary["outcome"] != "SUCCESS"


def test_live_plus_missing_with_payload_and_availability_invalid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line("r", monotonic_ns=100),
            _live_line_payload(run_id="r", status="MISSING", payload=_live_payload(), availability=frame_availability(fixture())),
            _terminal("r"),
        ])
        summary, exc = _summary_no_raise(path)
        assert exc is None
        assert summary["record_validity"] == "INVALID"
        assert any("malformed_missing_frame:1" in r for r in summary["record_validity_reasons"])
        assert summary["authoritative_runtime_source"] is False
        assert summary["outcome"] != "SUCCESS"


def test_missing_with_empty_dict_payload_legit():
    """An empty mapping payload/availability is treated as a legal gap."""
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        _write_jsonl(path, [
            _meta("r"),
            _live_line("r", monotonic_ns=100),
            {"kind": "frame", "run_id": "r", "status": "MISSING", "payload": {}, "availability": {}},
            _terminal("r"),
        ])
        summary, exc = _summary_no_raise(path)
        assert exc is None
        assert summary["record_validity"] == "VALID"
        assert not any("malformed_missing_frame" in r for r in summary["record_validity_reasons"])


# --------------------------------------------------------------- two-phase lifecycle
def test_two_phase_capture_stop_finalize_valid():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        recorder.start()
        recorder.record_snapshot(pack_frame(fixture(monotonic_ns=100)), now_ns=101)
        recorder.record_snapshot(pack_frame(fixture(sequence=4, rl_step=2, monotonic_ns=110)), now_ns=111)
        assert recorder.state == "CAPTURING"
        recorder.stop_sampling()
        assert recorder.state == "STOPPED"
        assert recorder.finalized is False
        recorder.finalize({"exit_code": 0, "forced_termination": False, "shutdown_complete": True})
        assert recorder.state == "FINALIZED"
        assert recorder.finalized is True
        summary = summarize_record(path)
        assert summary["record_validity"] == "VALID"
        assert load_record(path).terminal_count == 1


def test_stop_sampling_rejects_frame_write():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        recorder.start()
        recorder.record_snapshot(pack_frame(fixture(monotonic_ns=100)), now_ns=101)
        recorder.stop_sampling()
        try:
            recorder.record_snapshot(pack_frame(fixture(sequence=4, rl_step=2, monotonic_ns=110)), now_ns=111)
            assert False, "frame write after stop_sampling must be rejected"
        except RuntimeError as exc:
            assert "sampling stopped" in str(exc)
        data = load_record(path)
        assert len(data.frames) == 1  # the post-stop frame was never written


def test_not_finalized_before_finalize():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        recorder.start()
        recorder.record_snapshot(pack_frame(fixture(monotonic_ns=100)), now_ns=101)
        recorder.stop_sampling()
        assert recorder.finalized is False
        assert recorder.state == "STOPPED"
        lines = Path(path).read_text(encoding="utf-8").splitlines()
        assert not any("terminal" in json.loads(line).get("kind", "") for line in lines)
        assert recorder.stop_sampling() is None  # idempotent


def test_duplicate_finalize_rejected():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        recorder.start()
        recorder.record_snapshot(pack_frame(fixture(monotonic_ns=100)), now_ns=101)
        recorder.finalize({"exit_code": 0})
        try:
            recorder.finalize({"exit_code": 0})
            assert False, "duplicate finalize must be rejected"
        except RuntimeError as exc:
            assert "already finalized" in str(exc)
        assert load_record(path).terminal_count == 1


def test_finalize_rejects_frame_write():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        recorder.start()
        recorder.record_snapshot(pack_frame(fixture(monotonic_ns=100)), now_ns=101)
        recorder.finalize()
        try:
            recorder.record_snapshot(pack_frame(fixture(sequence=4, rl_step=2, monotonic_ns=110)), now_ns=111)
            assert False, "frame write after finalize must be rejected"
        except RuntimeError as exc:
            assert "already finalized" in str(exc)


def test_two_phase_missing_exit_fact_unknown():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        recorder.start()
        recorder.record_snapshot(pack_frame(fixture(monotonic_ns=100)), now_ns=101)
        recorder.stop_sampling()
        recorder.finalize()  # no process facts
        terminal = load_record(path).terminal
        assert terminal["process_exit_code"] is None  # stays UNKNOWN, no default fill
        assert terminal["forced_termination"] is None
        assert terminal["normal_shutdown"] is None
        summary = summarize_record(path)
        assert summary["record_validity"] == "VALID"
        assert summary["outcome"] == "UNKNOWN"


def test_two_phase_real_exit_code_written():
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        recorder.start()
        recorder.record_snapshot(pack_frame(fixture(monotonic_ns=100)), now_ns=101)
        recorder.stop_sampling()
        recorder.finalize({"exit_code": 5, "forced_termination": False, "shutdown_complete": True})
        terminal = load_record(path).terminal
        assert terminal["process_exit_code"] == 5
        assert terminal["termination_reason"] == "NONZERO_EXIT"
        assert summarize_record(path)["outcome"] == "FAILURE"


def test_stopped_record_not_polluted_by_controller_exit_frames():
    """After stop_sampling, a controller-exit INVALID frame in shm is never
    written into the record."""
    with tempfile.TemporaryDirectory() as tmp:
        path = str(Path(tmp) / "run.jsonl")
        recorder = RunRecordRecorder(path)
        recorder.start()
        recorder.record_snapshot(pack_frame(fixture(monotonic_ns=100)), now_ns=101)
        recorder.stop_sampling()
        try:
            recorder.record_snapshot(b"garbage-not-a-frame")
            assert False, "must not sample after stop"
        except RuntimeError:
            pass
        recorder.finalize({"exit_code": 0, "forced_termination": False, "shutdown_complete": True})
        data = load_record(path)
        assert len(data.frames) == 1
        assert data.frames[0]["status"] == "LIVE"
        summary = summarize_record(path)
        assert summary["record_validity"] == "VALID"
        assert summary["live_frame_count"] == 1


if __name__ == "__main__":
    tests = [value for name, value in globals().items() if name.startswith("test_") and callable(value)]
    failures = 0
    for test in sorted(tests, key=lambda t: t.__name__):
        try:
            test()
            print(f"  PASS {test.__name__}")
        except Exception as exc:  # noqa: BLE001 - test harness
            failures += 1
            print(f"  FAIL {test.__name__}: {exc}")
    print(f"{len(tests) - failures}/{len(tests)} PASS")
    sys.exit(1 if failures else 0)
