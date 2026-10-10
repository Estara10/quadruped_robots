#!/usr/bin/env python3
"""Exercise the production MuJoCo publisher -> normal reader -> record/analyzer path."""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import abs_collision
from abs_collision import CollisionStatus, classify_snapshot, read_collision_snapshot
from run_record import RunRecordRecorder, collision_snapshot_payload, _validate_collision_payload
from s1_analyze_run import motion_safety

CAPTURE = "p1-10-capture-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
FINGERPRINT = "3e4d82cf204d5929ff98c11fed5b3918ba3b92f0d698215860097d734d239694"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args()
    shm_name = f"/s1_05_xlang_{os.getpid()}_{uuid.uuid4().hex[:10]}"
    shm_path = Path("/dev/shm") / shm_name.lstrip("/")
    assert not shm_path.exists(), f"test shm unexpectedly exists: {shm_path}"
    evidence: dict = {"test_shm_name": shm_name, "layout": {"version": 4,
                 "bytes": abs_collision.SNAPSHOT_SIZE, "expected_bytes": 5760,
                 "offsets_bytes": {"version": 8, "sequence": 16, "physics_step": 32,
                                   "sim_time": 40, "ground_contacts": 72,
                                   "foot_ground_contacts": 388, "unknown_contact_steps": 536,
                                   "invalid_posture_steps": 568, "fall_history": 640,
                                   "collision_history": 2432}}}
    try:
        with tempfile.TemporaryDirectory(prefix="s1-05-xlang-") as temp:
            subprocess.run([str(args.probe), str(args.scene), shm_name, temp],
                           check=True, text=True, capture_output=True, timeout=30)
            now = time.monotonic_ns()
            snapshots = {}
            for name in ("initial", "unknown_contact", "invalid_posture",
                         "startup_confirmed", "recovered", "final"):
                raw = (Path(temp) / f"{name}.bin").read_bytes()
                status, snapshot = classify_snapshot(
                    raw, now, expected_capture_id=CAPTURE,
                    expected_fingerprint=FINGERPRINT)
                assert status is CollisionStatus.LIVE and snapshot is not None, (name, status)
                assert snapshot.version == 4 and snapshot.layout_size == 5760
                assert snapshot.ground_contacts == snapshot.foot_ground_contacts + snapshot.nonfoot_ground_contacts
                payload = collision_snapshot_payload(status, snapshot)
                assert not _validate_collision_payload(payload), (name, _validate_collision_payload(payload))
                snapshots[name] = (snapshot, payload)

            # Exercise the ordinary fixed-layout shared-memory reader on the
            # producer's actual final snapshot (no version-tag substitution).
            shm_raw = read_collision_snapshot(str(shm_path))
            status, shm_snapshot = classify_snapshot(
                shm_raw, time.monotonic_ns(), expected_capture_id=CAPTURE,
                expected_fingerprint=FINGERPRINT)
            assert status is CollisionStatus.LIVE and shm_snapshot is not None
            assert shm_snapshot.version == 4 and len(shm_raw) == 5760
            assert shm_snapshot.unknown_contact_steps >= 1
            assert shm_snapshot.invalid_posture_steps >= 1
            assert shm_snapshot.fall_history_count >= 2
            assert shm_snapshot.collision_history_count >= 1

            ordered = [snapshots[key] for key in
                       ("initial", "unknown_contact", "invalid_posture", "recovered", "final")]
            frames = [{"collision_snapshot": payload} for _, payload in ordered]
            falls = shm_snapshot.fall_history
            assert len(falls) >= 2 and falls[0]["confirmed"] and falls[0]["closed"]
            assert falls[-1]["confirmed"]
            startup_then_motion_start = falls[0]["start_sim_time"] + 0.01
            after_recovery_start = falls[0]["end_sim_time"] + 0.001
            final_time = shm_snapshot.sim_time
            crossing = motion_safety(frames, startup_then_motion_start, final_time)
            later_fall = motion_safety(frames, after_recovery_start, final_time)
            assert crossing["fall"] is True
            assert later_fall["fall"] is True and len(later_fall["fall_events"]) == 1

            # The production terminal serializer stores the same live v4
            # payload. Read the actual JSONL terminal record back before analysis.
            record_path = Path(temp) / "runtime_record.jsonl"
            recorder = RunRecordRecorder(str(record_path), run_id="s1-05-xlang-v4",
                                         capture_id=CAPTURE,
                                         expected_fingerprint=FINGERPRINT)
            recorder.start()
            recorder.finalize({
                "exit_code": 0, "forced_termination": False, "shutdown_complete": True,
                "run_terminal_result": "FALL_TERMINATION",
                "run_terminal_monotonic_ns": time.monotonic_ns(),
                "external_safety_events": [{
                    "event": "FALL_TERMINATION", "source": "production MuJoCo v4 probe",
                    "message": "controlled headless publisher-to-reader verification",
                    "observed_monotonic_ns": shm_snapshot.monotonic_ns,
                    "collision_snapshot": collision_snapshot_payload(status, shm_snapshot),
                }],
            })
            lines = [json.loads(line) for line in record_path.read_text().splitlines()]
            terminal_lines = [line for line in lines if line.get("kind") == "terminal"]
            assert len(terminal_lines) == 1
            saved = terminal_lines[0]["collision_coverage"]["last_snapshot"]
            assert saved["version"] == 4 and saved["fall_history_count"] >= 2
            assert not _validate_collision_payload(saved)

            evidence.update({
                "result": "PASS",
                "actual_reader": "read_collision_snapshot -> classify_snapshot",
                "field_checks": {
                    "version_bytes_size": [shm_snapshot.version, len(shm_raw)],
                    "ground_split": [shm_snapshot.ground_contacts,
                                     shm_snapshot.foot_ground_contacts,
                                     shm_snapshot.nonfoot_ground_contacts],
                    "collision_history_count": shm_snapshot.collision_history_count,
                    "unknown_contact_steps": shm_snapshot.unknown_contact_steps,
                    "invalid_posture_steps": shm_snapshot.invalid_posture_steps,
                    "fall_history_count": shm_snapshot.fall_history_count,
                    "physics_coverage_complete": bool(shm_snapshot.physics_coverage_complete),
                },
                "fall_history": list(shm_snapshot.fall_history),
                "analysis": {"crossing_start_fall": crossing["fall"],
                             "after_recovery_fall": later_fall["fall"],
                             "after_recovery_events": len(later_fall["fall_events"]),
                             "crossing_collision": crossing["collision"]},
                "serialization": {"terminal_lines": len(terminal_lines),
                                  "payload_valid": True,
                                  "terminal_payload_version": saved["version"]},
                "historical_v3_support": "covered by scripts/test_s1_05_safety.py; parsed as v3/3864, event latches incomplete, analyzer returns UNKNOWN for negative collision conclusion",
                "mismatch_rejection": "covered by scripts/test_s1_05_safety.py: v3/5760, v4/3864, unknown version, truncated rejected",
            })
        args.evidence.parent.mkdir(parents=True, exist_ok=True)
        args.evidence.write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(evidence, indent=2))
    finally:
        # The object name is unique to this process and was created by this test.
        try:
            shm_path.unlink()
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    main()
