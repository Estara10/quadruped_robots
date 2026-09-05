#!/usr/bin/env python3
"""Reader and validator for the formal P1-10 collision snapshot.

The legacy five-int32 ``/mujoco_collision`` buffer is intentionally not read.
This reader only accepts the versioned physics-step snapshot written by the
simulator authority and preserves missing/unknown/stale/invalid states.
"""

from __future__ import annotations

import enum
import hashlib
import math
import mmap
import os
import re
import struct
from dataclasses import dataclass
from typing import Optional, Tuple

MAGIC = 0x414253434F4E5432
VERSION = 2
SHM_NAME = "/mujoco_collision_v2"
SHM_PATH = "/dev/shm/mujoco_collision_v2"
SCENARIO_ID = "obstacle_test1"
SCENE_ROOT_SHA256 = "e12a69fa5463e723d115696b8872c27c71b03a9d029a9ef933343ae93ba6dd5e"
MODEL_CLOSURE_SHA256 = "6ca5da14be6909815ac9c41bf6db0f8108e07082aea5aba22c91e833e6181746"
SNAPSHOT_STRUCT = struct.Struct("<5Qd10I2iI32s64s64s64s64s4x")
SNAPSHOT_SIZE = SNAPSHOT_STRUCT.size
STALE_TIMEOUT_NS = 500_000_000
CAPTURE_ID_RE = re.compile(r"^p1-10-capture-[0-9a-f]{32}$")
HEX64_RE = re.compile(r"^[0-9a-f]{64}$")
FINGERPRINT_SCHEMA = "abs-go2-collision-model-fingerprint/v1"


class CollisionStatus(enum.Enum):
    MISSING = "MISSING"
    INVALID = "INVALID"
    UNKNOWN = "UNKNOWN"
    STALE = "STALE"
    LIVE = "LIVE"


@dataclass(frozen=True)
class CollisionSnapshot:
    magic: int
    version: int
    sequence: int
    monotonic_ns: int
    physics_step: int
    sim_time: float
    authoritative: int
    current_collision: int
    collision_edge: int
    classified_contacts: int
    unknown_contacts: int
    robot_obstacle_contacts: int
    ground_contacts: int
    self_contacts: int
    other_contacts: int
    last_contact_class: int
    last_robot_geom_id: int
    last_obstacle_geom_id: int
    invalid_reason: int
    scenario_id: str
    scene_root_sha256: str
    model_closure_sha256: str
    capture_id: str
    runtime_model_fingerprint: str


def canonical_model_fingerprint(nbody: int, geoms) -> str:
    """Python implementation of the simulator's fixed-width model encoding.

    ``geoms`` is an ordered sequence of dictionaries with the exact fields
    consumed by the C++ header.  It is a test/provenance helper; production
    expected values come from the offline MuJoCo probe, never from defaults.
    """
    if type(nbody) is not int or nbody < 0:
        raise ValueError("invalid nbody")
    records = list(geoms)
    payload = bytearray(FINGERPRINT_SCHEMA.encode("ascii") + b"\0")
    payload.extend(struct.pack("<II", nbody, len(records)))
    for index, geom in enumerate(records):
        if not isinstance(geom, dict):
            raise ValueError("geom is not an object")
        if geom.get("geom_id", index) != index:
            raise ValueError("geom ids must be contiguous and ordered")
        values = [geom.get(key) for key in
                  ("geom_type", "body_id", "geom_group", "geom_contype", "geom_conaffinity")]
        if any(type(value) is not int for value in values):
            raise ValueError("invalid integer geom field")
        payload.extend(struct.pack("<Iiiiii", index, *values))
        for key, length in (("geom_pos", 3), ("geom_quat", 4), ("geom_size", 3)):
            vector = geom.get(key)
            if not isinstance(vector, (list, tuple)) or len(vector) != length:
                raise ValueError(f"invalid {key}")
            if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) for value in vector):
                raise ValueError(f"non-finite {key}")
            payload.extend(struct.pack("<" + "d" * length, *(float(value) for value in vector)))
        for key in ("name", "body_name"):
            value = geom.get(key, "")
            if not isinstance(value, str):
                raise ValueError(f"invalid {key}")
            encoded = value.encode("utf-8")
            payload.extend(struct.pack("<I", len(encoded)))
            payload.extend(encoded)
    return hashlib.sha256(payload).hexdigest()


def _decode(raw: bytes) -> CollisionSnapshot:
    if len(raw) != SNAPSHOT_SIZE:
        raise ValueError("wrong collision snapshot size")
    values = SNAPSHOT_STRUCT.unpack(raw)
    strings = [value.split(b"\0", 1)[0].decode("ascii") for value in values[-5:]]
    return CollisionSnapshot(*values[:-5], *strings)


def _valid(snapshot: CollisionSnapshot, now_ns: int,
           expected_capture_id: Optional[str] = None,
           expected_fingerprint: Optional[str] = None) -> bool:
    if snapshot.magic != MAGIC or snapshot.version != VERSION:
        return False
    if snapshot.sequence == 0 or snapshot.sequence & 1:
        return False
    if snapshot.monotonic_ns == 0 or snapshot.physics_step == 0:
        return False
    if now_ns < snapshot.monotonic_ns or not math.isfinite(snapshot.sim_time):
        return False
    if CAPTURE_ID_RE.fullmatch(snapshot.capture_id) is None:
        return False
    if HEX64_RE.fullmatch(snapshot.runtime_model_fingerprint) is None:
        return False
    if expected_capture_id is not None and snapshot.capture_id != expected_capture_id:
        return False
    if expected_fingerprint is not None and snapshot.runtime_model_fingerprint != expected_fingerprint:
        return False
    for value in (snapshot.authoritative, snapshot.current_collision, snapshot.collision_edge):
        if value not in (0, 1):
            return False
    if snapshot.classified_contacts < 0 or snapshot.unknown_contacts < 0:
        return False
    if snapshot.robot_obstacle_contacts < 0 or snapshot.ground_contacts < 0:
        return False
    if snapshot.self_contacts < 0 or snapshot.other_contacts < 0:
        return False
    if snapshot.classified_contacts != (
        snapshot.robot_obstacle_contacts + snapshot.ground_contacts +
        snapshot.self_contacts + snapshot.other_contacts
    ):
        return False
    if snapshot.last_contact_class not in (0, 1, 2, 3, 4, 5):
        return False
    if snapshot.authoritative:
        if snapshot.scenario_id != SCENARIO_ID:
            return False
        if snapshot.scene_root_sha256 != SCENE_ROOT_SHA256:
            return False
        if snapshot.model_closure_sha256 != MODEL_CLOSURE_SHA256:
            return False
        if snapshot.invalid_reason != 0:
            return False
        if snapshot.current_collision != (1 if snapshot.robot_obstacle_contacts > 0 else 0):
            return False
        if snapshot.collision_edge and not snapshot.current_collision:
            return False
    return True


def classify_snapshot(raw: Optional[bytes], now_ns: int,
                      stale_timeout_ns: int = STALE_TIMEOUT_NS,
                      expected_capture_id: Optional[str] = None,
                      expected_fingerprint: Optional[str] = None) -> Tuple[CollisionStatus, Optional[CollisionSnapshot]]:
    if not raw:
        return CollisionStatus.MISSING, None
    try:
        snapshot = _decode(raw)
    except (ValueError, UnicodeDecodeError, struct.error):
        return CollisionStatus.INVALID, None
    if not _valid(snapshot, now_ns, expected_capture_id, expected_fingerprint):
        return CollisionStatus.INVALID, snapshot
    if snapshot.authoritative == 0:
        return CollisionStatus.UNKNOWN, snapshot
    if now_ns - snapshot.monotonic_ns > stale_timeout_ns:
        return CollisionStatus.STALE, snapshot
    return CollisionStatus.LIVE, snapshot


def read_collision_snapshot(path: str = SHM_PATH, max_attempts: int = 3) -> bytes:
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError:
        return b""
    try:
        if os.fstat(fd).st_size < SNAPSHOT_SIZE:
            return b""
        buf = mmap.mmap(fd, SNAPSHOT_SIZE, access=mmap.ACCESS_READ)
        try:
            for _ in range(max_attempts):
                before = struct.unpack_from("<Q", buf, 16)[0]
                if before == 0 or before & 1:
                    continue
                raw = bytes(buf[:SNAPSHOT_SIZE])
                after = struct.unpack_from("<Q", buf, 16)[0]
                if before == after and not after & 1:
                    return raw
            return b""
        finally:
            buf.close()
    finally:
        os.close(fd)
