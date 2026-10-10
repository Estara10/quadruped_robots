#!/usr/bin/env python3
"""Validate C++ v5 snapshots through the production Python reader/serializer/analyzer."""
from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from abs_collision import (BASE_DATA_SIZE, COLLISION_EVENT_CAPACITY, EPISODE_STRUCT,
                           EXTENSION_STRUCT, LEGACY_SNAPSHOT_SIZE, SNAPSHOT_V4_SIZE, CollisionStatus,
                           classify_snapshot)  # noqa: E402
from abs_scene import SCENES  # noqa: E402
from run_record import _validate_collision_payload, collision_snapshot_payload  # noqa: E402
from s1_analyze_run import motion_safety  # noqa: E402


def read_live(path: Path, binding: dict) -> tuple[bytes, dict]:
    raw = path.read_bytes()
    monotonic_ns = struct.unpack_from("<Q", raw, 24)[0]
    status, snapshot = classify_snapshot(raw, monotonic_ns, expected_scene_binding=binding)
    assert status is CollisionStatus.LIVE, (path, status, snapshot)
    payload = collision_snapshot_payload(status, snapshot)
    errors = _validate_collision_payload(payload, binding)
    assert errors == [], (path, errors)
    assert payload["version"] == 5 and payload["foot_identity_valid"] is True
    return raw, payload


def main() -> int:
    prefix = Path(sys.argv[1])
    binding = SCENES["scene_ppt_sparse.xml"].binding()
    floor_raw, floor = read_live(prefix.with_suffix(".floor.bin"), binding)
    foot_raw, foot = read_live(prefix.with_suffix(".foot.bin"), binding)
    nonfoot_raw, nonfoot = read_live(prefix.with_suffix(".nonfoot.bin"), binding)
    assert len(floor_raw) == len(foot_raw) == len(nonfoot_raw) == 14472
    assert floor["current_collision"] is False and floor["ground_contacts"] >= 0
    assert foot["foot_obstacle_contacts"] > 0 and foot["current_collision"] is False
    assert foot["foot_contact_history"] and foot["foot_contact_history"][0]["foot_geom_name"] in {"FL", "FR", "RL", "RR"}
    assert nonfoot["nonfoot_obstacle_contacts"] > 0 and nonfoot["current_collision"] is True
    assert nonfoot["collision_history"] and nonfoot["nonfoot_collision_bodies"][0]["robot_body_name"] == "FL_thigh"

    collision_bytes = COLLISION_EVENT_CAPACITY * EPISODE_STRUCT.size
    v4_offset = SNAPSHOT_V4_SIZE
    v4_raw = bytearray(floor_raw[:v4_offset])
    struct.pack_into("<Q", v4_raw, 8, 4)
    status, v4_snapshot = classify_snapshot(bytes(v4_raw), struct.unpack_from("<Q", v4_raw, 24)[0],
                                             expected_scene_binding=binding)
    assert status is CollisionStatus.LIVE and v4_snapshot.version == 4
    collision_offset = v4_offset - collision_bytes
    v3_prefix_size = BASE_DATA_SIZE + EXTENSION_STRUCT.size
    assert v3_prefix_size == LEGACY_SNAPSHOT_SIZE - collision_bytes
    v3_raw = bytearray(floor_raw[:v3_prefix_size] + floor_raw[collision_offset:collision_offset + collision_bytes])
    struct.pack_into("<Q", v3_raw, 8, 3)
    status, v3_snapshot = classify_snapshot(bytes(v3_raw), struct.unpack_from("<Q", v3_raw, 24)[0],
                                             expected_scene_binding=binding)
    assert status is CollisionStatus.LIVE and v3_snapshot.version == 3

    malformed = bytearray(foot_raw[:-1])
    status, _ = classify_snapshot(bytes(malformed), struct.unpack_from("<Q", foot_raw, 24)[0],
                                  expected_scene_binding=binding)
    assert status is CollisionStatus.INVALID
    wrong_version = bytearray(foot_raw)
    struct.pack_into("<Q", wrong_version, 8, 4)
    status, _ = classify_snapshot(bytes(wrong_version), struct.unpack_from("<Q", foot_raw, 24)[0],
                                  expected_scene_binding=binding)
    assert status is CollisionStatus.INVALID
    unknown_version = bytearray(foot_raw)
    struct.pack_into("<Q", unknown_version, 8, 6)
    status, _ = classify_snapshot(bytes(unknown_version), struct.unpack_from("<Q", foot_raw, 24)[0],
                                  expected_scene_binding=binding)
    assert status is CollisionStatus.INVALID

    frames = [{"collision_snapshot": item} for item in (floor, foot, nonfoot)]
    safety = motion_safety(frames, floor["sim_time"], nonfoot["sim_time"])
    assert safety["nonfoot_collision_failure"] is True
    assert safety["foot_contact_count"] >= 1
    assert safety["abs_reference_collision"] is True
    foot_only = motion_safety([{"collision_snapshot": floor}, {"collision_snapshot": foot}],
                              floor["sim_time"], foot["sim_time"])
    assert foot_only["nonfoot_collision_failure"] is False
    assert foot_only["foot_contact_count"] >= 1

    # Verify historical v4 payload acceptance using an untouched saved S1-05 record.
    v4_payload = None
    for historical in ROOT.glob(
            "docs/thesis_project/evidence/S1-05/diagnostic_runs_20261008/*/runtime_record.jsonl"):
        for line in historical.read_text(encoding="utf-8").splitlines():
            record = json.loads(line)
            candidate = record.get("payload", {}).get("collision_snapshot")
            if isinstance(candidate, dict) and candidate.get("version") == 4:
                v4_payload = candidate
                break
        if v4_payload is not None:
            break
    assert v4_payload is not None
    historical_errors = _validate_collision_payload(v4_payload)
    assert historical_errors == [], historical_errors
    out = {"version": 5, "length_bytes": len(foot_raw), "reader": "LIVE",
           "serializer_validation": "PASS", "analyzer": "PASS",
           "floor_collision_terminal": floor["current_collision"],
           "foot_contact_nonterminal": foot["current_collision"] is False,
           "nonfoot_terminal": nonfoot["current_collision"] is True,
           "nonfoot_body": nonfoot["nonfoot_collision_bodies"][0]["robot_body_name"],
           "nonfoot_body_id": nonfoot["nonfoot_collision_bodies"][0]["robot_body_id"],
           "nonfoot_geom_id": nonfoot["collision_history"][0]["robot_geom_id"],
           "truncated_mismatched_and_unknown_versions_rejected": True,
           "v3_layout": "ACCEPTED_3864",
           "v4_layout": "ACCEPTED_5760",
           "historical_v4_payload": "ACCEPTED"}
    print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
