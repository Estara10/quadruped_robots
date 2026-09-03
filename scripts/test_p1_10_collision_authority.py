#!/usr/bin/env python3
"""Offline tests for P1-10 Stage-B collision authority and record binding."""

from __future__ import annotations

import copy
import json
import struct
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
from abs_collision import (  # noqa: E402
    CollisionStatus, MAGIC, MODEL_CLOSURE_SHA256, SCENARIO_ID,
    SCENE_ROOT_SHA256, SNAPSHOT_SIZE, SNAPSHOT_STRUCT, VERSION,
    canonical_model_fingerprint, classify_snapshot,
)
CAPTURE_ID = "p1-10-capture-0123456789abcdef0123456789abcdef"
FINGERPRINT = "0123456789abcdef" * 4
from run_record import RunRecordRecorder, summarize_record  # noqa: E402
from test_run_record import fixture, pack_frame  # noqa: E402


def pack_snapshot(*, authoritative=1, current=0, edge=0, classified=1,
                  unknown=0, robot_obstacle=0, ground=1, self_contacts=0,
                  other=0, last_class=2, physics_step=1, sim_time=0.01,
                  sequence=2, monotonic_ns=1000, scenario_id=SCENARIO_ID,
                  root_sha=SCENE_ROOT_SHA256, closure_sha=MODEL_CLOSURE_SHA256,
                  invalid_reason=0, capture_id=CAPTURE_ID,
                  runtime_model_fingerprint=FINGERPRINT):
    return SNAPSHOT_STRUCT.pack(
        MAGIC, VERSION, sequence, monotonic_ns, physics_step, sim_time,
        authoritative, current, edge, classified, unknown, robot_obstacle,
        ground, self_contacts, other, last_class, -1, -1, invalid_reason,
        scenario_id.encode(), root_sha.encode(), closure_sha.encode(),
        capture_id.encode(), runtime_model_fingerprint.encode(),
    )


def test_contact_classification_fixture_domains():
    # The bridge contract's source-level classification is reflected in the
    # serialized result: robot↔obstacle is the only positive formal event.
    positive = pack_snapshot(current=1, edge=1, classified=1, ground=0,
                             robot_obstacle=1, last_class=1)
    status, snapshot = classify_snapshot(positive, 1001)
    assert status is CollisionStatus.LIVE and snapshot.current_collision == 1
    for fixture in (
        pack_snapshot(current=0, classified=1, ground=1, last_class=2),  # floor
        pack_snapshot(current=0, classified=1, self_contacts=1, ground=0, last_class=3),  # self
        pack_snapshot(current=0, classified=0, unknown=1, ground=0, last_class=5),  # unknown
    ):
        status, snapshot = classify_snapshot(fixture, 1001)
        assert status is CollisionStatus.LIVE and snapshot.current_collision == 0


def test_snapshot_identity_version_sequence_and_time_fail_closed():
    valid = pack_snapshot()
    assert len(valid) == SNAPSHOT_SIZE
    assert classify_snapshot(valid, 1001)[0] is CollisionStatus.LIVE
    cases = [
        pack_snapshot(authoritative=0),
        pack_snapshot(scenario_id="wrong"),
        pack_snapshot(root_sha="0" * 64),
        pack_snapshot(closure_sha="0" * 64),
        pack_snapshot(sequence=3),
        pack_snapshot(sequence=2, monotonic_ns=0),
        pack_snapshot(sequence=2, monotonic_ns=1002),
        pack_snapshot(invalid_reason=1),
        pack_snapshot(current=0, robot_obstacle=1),
    ]
    expected = [CollisionStatus.UNKNOWN, CollisionStatus.INVALID, CollisionStatus.INVALID,
                CollisionStatus.INVALID, CollisionStatus.INVALID, CollisionStatus.INVALID,
                CollisionStatus.INVALID, CollisionStatus.INVALID, CollisionStatus.INVALID]
    for raw, status_expected in zip(cases, expected):
        assert classify_snapshot(raw, 1001)[0] is status_expected


def test_snapshot_stale_and_malformed_fail_closed():
    assert classify_snapshot(pack_snapshot(), 1000 + 500_000_001)[0] is CollisionStatus.STALE
    assert classify_snapshot(b"\0" * (SNAPSHOT_SIZE - 1), 1001)[0] is CollisionStatus.INVALID


def test_capture_identity_and_runtime_fingerprint_are_strictly_bound():
    raw = pack_snapshot()
    assert classify_snapshot(raw, 1001, expected_capture_id=CAPTURE_ID,
                             expected_fingerprint=FINGERPRINT)[0] is CollisionStatus.LIVE
    assert classify_snapshot(raw, 1001, expected_capture_id="p1-10-capture-" + "f" * 32)[0] is CollisionStatus.INVALID
    assert classify_snapshot(raw, 1001, expected_fingerprint="f" * 64)[0] is CollisionStatus.INVALID
    assert classify_snapshot(pack_snapshot(capture_id=""), 1001)[0] is CollisionStatus.INVALID
    assert classify_snapshot(pack_snapshot(runtime_model_fingerprint="UNKNOWN"), 1001)[0] is CollisionStatus.INVALID
    old = bytearray(raw)
    # Version is the second uint64. Old v1 bytes are never silently accepted.
    struct.pack_into("<Q", old, 8, VERSION - 1)
    assert classify_snapshot(bytes(old), 1001)[0] is CollisionStatus.INVALID


def test_python_canonical_model_fingerprint_is_fixed_width_and_mutation_sensitive():
    model = {
        "geom_id": 0, "geom_type": 5, "body_id": 0, "geom_group": 3,
        "geom_contype": 1, "geom_conaffinity": 1,
        "geom_pos": [1.0, 2.0, 3.0], "geom_quat": [1.0, 0.0, 0.0, 0.0],
        "geom_size": [0.5, 0.25, 0.125], "name": "obstacle",
        "body_name": "world",
    }
    baseline = canonical_model_fingerprint(1, [model])
    assert len(baseline) == 64
    for field, value in (("geom_type", 6), ("body_id", 1), ("geom_group", 4),
                         ("geom_contype", 2), ("geom_conaffinity", 2),
                         ("name", "renamed"), ("body_name", "renamed_body")):
        changed = dict(model)
        changed[field] = value
        assert canonical_model_fingerprint(1, [changed]) != baseline
    changed = dict(model)
    changed["geom_pos"] = [1.001, 2.0, 3.0]
    assert canonical_model_fingerprint(1, [changed]) != baseline


def test_record_consumes_real_filesystem_snapshot_and_reports_event():
    with tempfile.TemporaryDirectory(prefix="p1_10_collision_record_") as temp:
        path = Path(temp) / "runtime_record.jsonl"
        snapshot_path = Path(temp) / "mujoco_collision_v2"
        snapshot_path.write_bytes(pack_snapshot(current=1, edge=1, classified=1,
                                                 ground=0, robot_obstacle=1, last_class=1))
        with patch("run_record.read_collision_snapshot", lambda: snapshot_path.read_bytes()):
            recorder = RunRecordRecorder(str(path), capture_id=CAPTURE_ID,
                                         expected_fingerprint=FINGERPRINT)
            recorder.start()
            recorder.record_snapshot(pack_frame(fixture()), now_ns=1001)
            terminal = recorder.finalize({"exit_code": 0, "forced_termination": False,
                                          "shutdown_complete": True,
                                          "shutdown_request_source": "SIGINT"})
        assert terminal["collision_events"] is True
        assert terminal["collision_coverage"]["live_samples"] == 1
        summary = summarize_record(str(path))
        assert summary["collision"]["available"] is True
        assert summary["collision"]["event_count"] == 1


def test_recorder_rejects_snapshot_from_different_capture():
    with tempfile.TemporaryDirectory(prefix="p1_10_collision_capture_" ) as temp:
        path = Path(temp) / "runtime_record.jsonl"
        snapshot_path = Path(temp) / "mujoco_collision_v2"
        snapshot_path.write_bytes(pack_snapshot(capture_id="p1-10-capture-" + "f" * 32))
        with patch("run_record.read_collision_snapshot", lambda: snapshot_path.read_bytes()):
            recorder = RunRecordRecorder(str(path), capture_id=CAPTURE_ID,
                                         expected_fingerprint=FINGERPRINT)
            recorder.start()
            recorder.record_snapshot(pack_frame(fixture()), now_ns=1001)
            recorder.finalize({"exit_code": 0, "forced_termination": False,
                               "shutdown_complete": True,
                               "shutdown_request_source": "SIGINT"})
        assert summarize_record(str(path))["record_validity"] == "INVALID"


def test_no_false_collision_free_claim_when_sampled_physics_steps_have_gap():
    with tempfile.TemporaryDirectory(prefix="p1_10_collision_gap_") as temp:
        path = Path(temp) / "runtime_record.jsonl"
        snapshots = iter((pack_snapshot(physics_step=1), pack_snapshot(physics_step=3)))
        with patch("run_record.read_collision_snapshot", lambda: next(snapshots)):
            recorder = RunRecordRecorder(str(path), capture_id=CAPTURE_ID,
                                         expected_fingerprint=FINGERPRINT)
            recorder.start()
            recorder.record_snapshot(pack_frame(fixture(sequence=2, rl_step=1)), now_ns=1001)
            recorder.record_snapshot(pack_frame(fixture(sequence=4, rl_step=2, monotonic_ns=200)), now_ns=2001)
            terminal = recorder.finalize({"exit_code": 0, "forced_termination": False,
                                          "shutdown_complete": True,
                                          "shutdown_request_source": "SIGINT"})
        assert terminal["collision_events"] == "UNKNOWN"
        assert terminal["collision_coverage"]["physics_step_gaps"] == 1
        assert summarize_record(str(path))["collision"]["event_count"] is None


def test_contiguous_no_contact_samples_do_not_claim_episode_collision_free():
    with tempfile.TemporaryDirectory(prefix="p1_10_collision_coverage_") as temp:
        path = Path(temp) / "runtime_record.jsonl"
        snapshots = iter((pack_snapshot(physics_step=1), pack_snapshot(physics_step=2)))
        with patch("run_record.read_collision_snapshot", lambda: next(snapshots)):
            recorder = RunRecordRecorder(str(path), capture_id=CAPTURE_ID,
                                         expected_fingerprint=FINGERPRINT)
            recorder.start()
            recorder.record_snapshot(pack_frame(fixture(sequence=2, rl_step=1)), now_ns=1001)
            recorder.record_snapshot(pack_frame(fixture(sequence=4, rl_step=2, monotonic_ns=200)), now_ns=2001)
            terminal = recorder.finalize({"exit_code": 0, "forced_termination": False,
                                          "shutdown_complete": True,
                                          "shutdown_request_source": "SIGINT"})
        assert terminal["collision_events"] == "UNKNOWN"
        assert terminal["collision_coverage"]["complete_for_no_collision_claim"] is False
        assert summarize_record(str(path))["collision"]["event_count"] is None


def test_historical_record_without_snapshot_remains_unknown():
    with tempfile.TemporaryDirectory(prefix="p1_10_collision_legacy_") as temp:
        path = Path(temp) / "legacy.jsonl"
        recorder = RunRecordRecorder(str(path))
        recorder.start()
        recorder.record_snapshot(pack_frame(fixture()), now_ns=1001)
        terminal = recorder.finalize({"exit_code": 0, "forced_termination": False,
                                      "shutdown_complete": True,
                                      "shutdown_request_source": "SIGINT"})
        assert terminal["collision_events"] == "UNKNOWN"
        assert summarize_record(str(path))["collision"]["available"] is False


def test_existing_source_contract_remains_explicit():
    source = (ROOT / "unitree_mujoco/simulate/src/obstacle_collision_authority.h").read_text()
    assert "data->ncon" in source
    assert "classifyContact" in source
    assert "kObstacleSignatures" in source
    assert "mujoco_collision_v2" in (ROOT / "common/abs_collision_contract.h").read_text()


if __name__ == "__main__":
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_")]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print(f"{len(tests)} collision-authority tests PASS")
