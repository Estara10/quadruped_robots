#!/usr/bin/env python3
"""Offline rejection fixtures for formal_rt_frame_recorder (no runtime)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from abs_rt_frame import (  # noqa: E402
    FrameStatus,
    RuntimeFrame,
    SOURCE_AUTHORITATIVE_RUNTIME,
    SOURCE_SYNTHETIC_TEST,
)
from formal_rt_frame_recorder import (  # noqa: E402
    FIXED_RUNTIME_SOURCE,
    FormalRtFrameRecorder,
    _synthetic_probe_only,
)


def fixture(source=SOURCE_AUTHORITATIVE_RUNTIME, sequence=2, session_id=7, rl_step=1, monotonic_ns=100):
    values = dict(
        magic=0x414253525446524D, version=1, sequence=sequence,
        monotonic_ns=monotonic_ns, session_id=session_id, rl_step=rl_step,
        ray_age_ns=1, source=source, controller_active=1, rl_entered=1,
        rl_active=1, safety_faulted=0, policy_state=0, ray_origin=1,
        ray_valid=1, collision_origin=0, torque_saturated_computed=0,
        reserved_pad=0, ra_value=0.1, lin_vel=(0.1, 0.2, 0.3),
        command=(0.4, 0.5, 0.6), world_pose=(1.0, 2.0, 0.3),
        ray2d=tuple(float(i) for i in range(11)),
        action_raw=tuple(float(i) for i in range(12)),
        action_clipped=tuple(float(i) for i in range(12)),
        joint_target_rad=tuple(float(i) for i in range(12)),
        torque_nm=tuple(float(i) for i in range(12)),
        torque_saturated=tuple(0.0 for _ in range(12)),
    )
    return RuntimeFrame(**values)


def test_fixed_source_is_immutable_contract():
    assert FIXED_RUNTIME_SOURCE == "/dev/shm/mujoco_rt_frame"
    assert "path" not in FormalRtFrameRecorder.capture_once.__annotations__


def test_synthetic_probe_never_runtime_valid():
    decision = _synthetic_probe_only(fixture(SOURCE_SYNTHETIC_TEST), 200)
    assert not decision.runtime_valid
    assert decision.status == "INVALID"


def test_live_frame_still_invalid_until_missing_authority_closes():
    recorder = FormalRtFrameRecorder()
    decision = recorder._accept_live_frame(fixture())
    assert not decision.runtime_valid
    assert any("simulation_time_s" in reason for reason in decision.reasons)
    assert decision.formal_sequence is None
    assert decision.rejection_index == 1


def test_source_sequence_skip_allowed_and_reversal_rejected():
    recorder = FormalRtFrameRecorder()
    assert recorder._accept_live_frame(fixture(sequence=2, rl_step=1, monotonic_ns=100), authority_complete=True).status == "RUNTIME_VALID"
    assert recorder._accept_live_frame(fixture(sequence=8, rl_step=3, monotonic_ns=110), authority_complete=True).status == "RUNTIME_VALID"
    rejected = recorder._accept_live_frame(fixture(sequence=6, rl_step=4, monotonic_ns=120), authority_complete=True)
    assert "source_sequence_reversed_or_repeated" in rejected.reasons


def test_session_change_rejected():
    recorder = FormalRtFrameRecorder()
    recorder._accept_live_frame(fixture(session_id=7), authority_complete=True)
    rejected = recorder._accept_live_frame(fixture(sequence=4, session_id=8, rl_step=2, monotonic_ns=110), authority_complete=True)
    assert "session_id_changed" in rejected.reasons


def test_rl_step_and_time_reversal_rejected():
    recorder = FormalRtFrameRecorder()
    recorder._accept_live_frame(fixture(), authority_complete=True)
    rejected = recorder._accept_live_frame(fixture(sequence=4, rl_step=0, monotonic_ns=99), authority_complete=True)
    assert "rl_step_reversed_or_repeated" in rejected.reasons
    assert "monotonic_time_reversed_or_repeated" in rejected.reasons


def test_non_live_status_rejected_before_projection():
    recorder = FormalRtFrameRecorder()
    decision = recorder._accept_runtime_bytes(b"", 100)
    assert decision.status == "INVALID"
    assert "frame_status:MISSING" in decision.reasons


def test_no_writer_path_exists():
    recorder = FormalRtFrameRecorder()
    assert not hasattr(recorder, "writer")
    assert recorder.shutdown_complete is False


def test_incomplete_or_forced_shutdown_blocks_validity():
    recorder = FormalRtFrameRecorder()
    decision = recorder.finalize("SUCCESS", shutdown_rc=143, forced_termination=True)
    assert not decision.runtime_valid
    assert "shutdown_not_rc0" in decision.reasons
    assert "forced_termination_invalid" in decision.reasons
    assert "complete_shutdown_evidence_missing" in decision.reasons


def test_rejected_sample_does_not_consume_formal_sequence():
    recorder = FormalRtFrameRecorder()
    rejected = recorder._accept_live_frame(fixture(), authority_complete=False)
    assert rejected.formal_sequence is None
    accepted = recorder._accept_live_frame(fixture(sequence=8, rl_step=3, monotonic_ns=110), authority_complete=True)
    assert accepted.formal_sequence == 0


def test_formal_sequence_is_contiguous_only_for_eligible_samples():
    recorder = FormalRtFrameRecorder()
    first = recorder._accept_live_frame(fixture(), authority_complete=True)
    rejected = recorder._accept_live_frame(fixture(sequence=4, rl_step=0, monotonic_ns=99), authority_complete=True)
    second = recorder._accept_live_frame(fixture(sequence=8, rl_step=3, monotonic_ns=110), authority_complete=True)
    assert first.formal_sequence == 0
    assert rejected.formal_sequence is None
    assert second.formal_sequence == 1


def test_safety_evidence_cannot_be_overridden_by_success():
    recorder = FormalRtFrameRecorder()
    recorder.mark_shutdown_complete()
    decision = recorder.finalize("SUCCESS", collision=True, shutdown_rc=0)
    assert "safety_evidence_overrides_success" in decision.reasons
    assert not decision.runtime_valid


if __name__ == "__main__":
    tests = [value for name, value in globals().items() if name.startswith("test_") and callable(value)]
    for test in tests:
        test()
    print(f"{len(tests)}/{len(tests)} PASS")
