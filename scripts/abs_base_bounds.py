#!/usr/bin/env python3
"""Fail-closed reader for the PhysicsLoop base_link XY boundary latch."""

from __future__ import annotations

import math
import os
import struct
import time
from pathlib import Path

MAGIC = 0x414253424E445331
VERSION = 1
DEFAULT_SHM_NAME = "/mujoco_base_bounds_v1"
SNAPSHOT = struct.Struct("<12Q13d32s64s64s")
STALE_NS = 500_000_000


def _cstr(raw: bytes) -> str | None:
    value = raw.split(b"\0", 1)[0]
    if not value:
        return None
    try:
        return value.decode("ascii")
    except UnicodeDecodeError:
        return None


def read_snapshot(path: Path | None = None, now_ns: int | None = None,
                  expected_scene_id: str | None = None,
                  expected_capture_id: str | None = None,
                  expected_fingerprint: str | None = None,
                  retries: int = 8) -> dict:
    # ``now_ns`` is an explicit validation instant for deterministic callers
    # (for example, offline checks). Production callers leave it unset so the
    # clock is sampled only after a complete seqlock-consistent snapshot has
    # been read. Sampling before the read can make a legitimate concurrent
    # producer update look like a future timestamp.
    injected_validation_ns = now_ns
    if path is None:
        shm_name = os.environ.get("ABS_BASE_BOUNDS_SHM_NAME", DEFAULT_SHM_NAME)
        if not shm_name.startswith("/") or "/" in shm_name[1:]:
            return {"status": "INVALID", "reason": "invalid_shm_name"}
        path = Path("/dev/shm") / shm_name.lstrip("/")
    else:
        path = Path(path)
    try:
        fd = os.open(path, os.O_RDONLY | os.O_CLOEXEC)
    except FileNotFoundError:
        return {"status": "MISSING", "reason": "shm_missing"}
    except OSError as exc:
        return {"status": "INVALID", "reason": f"open:{exc.errno}"}
    try:
        try:
            size = os.fstat(fd).st_size
            if size != SNAPSHOT.size:
                return {"status": "INVALID", "reason": "size_mismatch",
                        "actual_size": size, "expected_size": SNAPSHOT.size}
            for _ in range(retries):
                raw = os.pread(fd, size, 0)
                if len(raw) != size:
                    continue
                first = struct.unpack_from("<Q", raw, 16)[0]
                if first & 1:
                    continue
                values = SNAPSHOT.unpack(raw)
                second = struct.unpack_from("<Q", os.pread(fd, 24, 0), 16)[0]
                if first != second or second & 1 or values[2] != second:
                    continue
                keys = ("magic", "version", "sequence", "monotonic_ns", "physics_step",
                        "producer_session_id", "physics_coverage_complete", "bounds_valid",
                        "currently_outside", "outside_physics_steps", "first_outside_step",
                        "last_outside_step")
                snap = dict(zip(keys, values[:12]))
                dkeys = ("sim_time", "base_x", "base_y", "xmin", "xmax", "ymin", "ymax",
                         "first_outside_sim_time", "last_outside_sim_time", "min_observed_x",
                         "max_observed_x", "min_observed_y", "max_observed_y")
                snap.update(zip(dkeys, values[12:25]))
                snap["scenario_id"] = _cstr(values[25])
                snap["capture_id"] = _cstr(values[26])
                snap["runtime_model_fingerprint"] = _cstr(values[27])
                if snap["magic"] != MAGIC or snap["version"] != VERSION:
                    return {"status": "INVALID", "reason": "magic_or_version_mismatch",
                            "snapshot": snap}
                if snap["sequence"] == 0 or snap["sequence"] & 1:
                    return {"status": "INVALID", "reason": "sequence_invalid", "snapshot": snap}
                if any(snap[k] not in (0, 1) for k in
                       ("physics_coverage_complete", "bounds_valid", "currently_outside")):
                    return {"status": "INVALID", "reason": "boolean_flag_invalid", "snapshot": snap}
                finite_keys = ("sim_time", "base_x", "base_y", "xmin", "xmax", "ymin", "ymax")
                if not all(math.isfinite(snap[k]) for k in finite_keys):
                    return {"status": "INVALID", "reason": "nonfinite_required_field", "snapshot": snap}
                if (snap["physics_step"] == 0 or snap["producer_session_id"] == 0 or
                    snap["xmin"] >= snap["xmax"] or snap["ymin"] >= snap["ymax"]):
                    return {"status": "INVALID", "reason": "identity_or_bounds_invalid", "snapshot": snap}
                identity_expectations = (("scenario_id", expected_scene_id),
                                         ("capture_id", expected_capture_id),
                                         ("runtime_model_fingerprint", expected_fingerprint))
                mismatches = {key: {"actual": snap[key], "expected": expected}
                              for key, expected in identity_expectations
                              if expected is not None and snap[key] != expected}
                if mismatches:
                    return {"status": "INVALID", "reason": "identity_mismatch",
                            "identity_mismatches": mismatches, "snapshot": snap}
                validation_now_ns = (time.monotonic_ns() if injected_validation_ns is None
                                     else injected_validation_ns)
                snap["validation_monotonic_ns"] = validation_now_ns
                age = validation_now_ns - snap["monotonic_ns"]
                snap["age_ns"] = age
                if age < 0:
                    return {"status": "INVALID", "reason": "future_timestamp", "snapshot": snap}
                if age > STALE_NS:
                    return {"status": "STALE", "reason": "snapshot_expired", "snapshot": snap}
                return {"status": "LIVE", "reason": None, "snapshot": snap}
            return {"status": "INVALID", "reason": "seqlock_unstable"}
        except OSError as exc:
            return {"status": "INVALID", "reason": f"read:{exc.errno}"}
    finally:
        os.close(fd)

