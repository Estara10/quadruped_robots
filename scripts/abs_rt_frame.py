#!/usr/bin/env python3
"""Offline frame contract + minimal read-only HUD state model for P1-09C.

This module is the consumer side of the single real-time data link described in
``common/abs_rt_frame_contract.h``. It does three things, all fail-closed:

1. Mirrors the fixed 424-byte frame layout and the magic/version/source enums so
   the Python side and the C++ writer (``StateRL::writeRtFrame``) agree exactly.
2. Classifies one frame snapshot into MISSING / INVALID / UNKNOWN_ORIGIN /
   LEGACY / SYNTHETIC / STALE / LIVE, applying the real-data boundary: only a
   frame whose ``source == AUTHORITATIVE_RUNTIME`` is even a candidate for LIVE.
3. Exposes a read-only HUD state model that surfaces only real live values and
   refuses to fabricate any value whose availability flag is 0 (torque-saturated,
   collision) or whose step was faulted.

No ROS2, MuJoCo, benchmark, pilot, formal episode, or real-robot process is
launched here. ``read_shm_frame`` is the runtime-only reader and is never
exercised by the offline tests.
"""

from __future__ import annotations

import math
import mmap
import os
import struct
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Tuple

# ---------------------------------------------------------------- frame layout
# These MUST stay byte-identical to common/abs_rt_frame_contract.h.
FRAME_MAGIC = 0x414253525446524D  # mnemonic "ABSRTFRM"
FRAME_VERSION = 1
JOINT_COUNT = 12
RAY_COUNT = 11
SHM_NAME = "/mujoco_rt_frame"
SHM_PATH = "/dev/shm/mujoco_rt_frame"

# Field order is chosen so the C++ struct has no implicit padding and matches
# "<7Q11I81f" exactly. 7 Q = header(4) + session_id + rl_step + ray_age_ns;
# 11 I = source, controller_active, rl_entered, rl_active, safety_faulted,
#        policy_state, ray_origin, ray_valid, collision_origin,
#        torque_saturated_computed, reserved_pad;
# 81 f = ra_value + lin_vel(3) + command(3) + world_pose(3) + ray2d(11)
#        + action_raw(12) + action_clipped(12) + joint_target_rad(12)
#        + torque_nm(12) + torque_saturated(12).
_FRAME_STRUCT = struct.Struct("<7Q11I81f")
FRAME_SIZE = _FRAME_STRUCT.size  # 424

_HEADER_SEQUENCE_OFFSET = 16  # 3rd uint64 in the header

# ------------------------------------------------------------------- enums
# Frame source classification (mirrors the P1-09B adapter input-origin boundary).
SOURCE_UNSET = 0
SOURCE_AUTHORITATIVE_RUNTIME = 1
SOURCE_SYNTHETIC_TEST = 2
SOURCE_LEGACY_ONLY = 3

POLICY_AGILE = 0
POLICY_RECOVERY = 1
POLICY_FAULTED = 2

RAY_UNAVAILABLE = 0
RAY_SHM_RUNTIME = 1

COLLISION_UNAVAILABLE = 0

_POLICY_NAMES = {POLICY_AGILE: "AGILE", POLICY_RECOVERY: "RECOVERY", POLICY_FAULTED: "FAULTED"}


class FrameStatus(Enum):
    """Fail-closed classification of one frame snapshot."""

    MISSING = "MISSING"                    # absent / empty input
    INVALID = "INVALID"                    # corrupt: wrong size, bad magic/version,
                                           # unarmed/odd sequence, inconsistent flags,
                                           # non-finite payload
    UNKNOWN_ORIGIN = "UNKNOWN_ORIGIN"      # source unset or unrecognized
    LEGACY = "LEGACY"                      # source == LEGACY_ONLY (rejected)
    SYNTHETIC = "SYNTHETIC"                # source == SYNTHETIC_TEST (never live)
    STALE = "STALE"                        # authoritative but timestamp too old
    LIVE = "LIVE"                          # authoritative, fresh, coherent, finite


# Freshness default: frames arrive at the policy cadence (~50 Hz), so 500 ms is
# ~25 missed frames. Callers may tighten this for a formal recorder.
DEFAULT_STALE_TIMEOUT_NS = 500_000_000


@dataclass(frozen=True)
class RuntimeFrame:
    magic: int
    version: int
    sequence: int
    monotonic_ns: int
    session_id: int
    rl_step: int
    ray_age_ns: int
    source: int
    controller_active: int
    rl_entered: int
    rl_active: int
    safety_faulted: int
    policy_state: int
    ray_origin: int
    ray_valid: int
    collision_origin: int
    torque_saturated_computed: int
    reserved_pad: int
    ra_value: float
    lin_vel: Tuple[float, float, float]
    command: Tuple[float, float, float]
    world_pose: Tuple[float, float, float]
    ray2d: Tuple[float, ...]           # 11
    action_raw: Tuple[float, ...]      # 12
    action_clipped: Tuple[float, ...]  # 12
    joint_target_rad: Tuple[float, ...]  # 12
    torque_nm: Tuple[float, ...]       # 12
    torque_saturated: Tuple[float, ...]  # 12

    @classmethod
    def from_bytes(cls, data: bytes) -> "RuntimeFrame":
        if len(data) != FRAME_SIZE:
            raise ValueError(
                f"frame size {len(data)} != {FRAME_SIZE}"
            )
        values = _FRAME_STRUCT.unpack(data)
        (
            magic, version, sequence, monotonic_ns,
            session_id, rl_step, ray_age_ns,
            source, controller_active, rl_entered, rl_active, safety_faulted,
            policy_state, ray_origin, ray_valid, collision_origin,
            torque_saturated_computed, reserved_pad,
        ) = values[:18]
        floats = values[18:]
        ra_value = floats[0]
        lin_vel = tuple(floats[1:4])
        command = tuple(floats[4:7])
        world_pose = tuple(floats[7:10])
        ray2d = tuple(floats[10:10 + RAY_COUNT])
        offset = 10 + RAY_COUNT
        action_raw = tuple(floats[offset:offset + JOINT_COUNT]); offset += JOINT_COUNT
        action_clipped = tuple(floats[offset:offset + JOINT_COUNT]); offset += JOINT_COUNT
        joint_target_rad = tuple(floats[offset:offset + JOINT_COUNT]); offset += JOINT_COUNT
        torque_nm = tuple(floats[offset:offset + JOINT_COUNT]); offset += JOINT_COUNT
        torque_saturated = tuple(floats[offset:offset + JOINT_COUNT])
        return cls(
            magic=magic, version=version, sequence=sequence, monotonic_ns=monotonic_ns,
            session_id=session_id, rl_step=rl_step, ray_age_ns=ray_age_ns,
            source=source, controller_active=controller_active, rl_entered=rl_entered,
            rl_active=rl_active, safety_faulted=safety_faulted, policy_state=policy_state,
            ray_origin=ray_origin, ray_valid=ray_valid, collision_origin=collision_origin,
            torque_saturated_computed=torque_saturated_computed, reserved_pad=reserved_pad,
            ra_value=ra_value, lin_vel=lin_vel, command=command, world_pose=world_pose,
            ray2d=ray2d, action_raw=action_raw, action_clipped=action_clipped,
            joint_target_rad=joint_target_rad, torque_nm=torque_nm,
            torque_saturated=torque_saturated,
        )


def _all_finite(frame: RuntimeFrame) -> bool:
    groups = (
        (frame.ra_value,),
        frame.lin_vel,
        frame.command,
        frame.world_pose,
        frame.ray2d,
        frame.action_raw,
        frame.action_clipped,
        frame.joint_target_rad,
        frame.torque_nm,
        frame.torque_saturated,
    )
    for group in groups:
        for value in group:
            if isinstance(value, bool) or not math.isfinite(value):
                return False
    return True


def classify_frame(
    data: Optional[bytes],
    now_ns: int,
    stale_timeout_ns: int = DEFAULT_STALE_TIMEOUT_NS,
) -> Tuple[FrameStatus, Optional[RuntimeFrame]]:
    """Classify one frame snapshot fail-closed.

    ``now_ns`` must be a steady-clock monotonic nanosecond timestamp in the same
    domain as the writer (``StateRL::monotonicNowNs``).
    """
    if not data:
        return FrameStatus.MISSING, None
    try:
        frame = RuntimeFrame.from_bytes(data)
    except (ValueError, struct.error):
        return FrameStatus.INVALID, None

    if frame.magic != FRAME_MAGIC or frame.version != FRAME_VERSION:
        return FrameStatus.INVALID, None
    if frame.sequence == 0 or (frame.sequence & 1):
        return FrameStatus.INVALID, None  # unarmed or writer-in-progress

    # Real-data boundary: only an authoritative runtime frame may be LIVE.
    if frame.source == SOURCE_UNSET:
        return FrameStatus.UNKNOWN_ORIGIN, None
    if frame.source == SOURCE_LEGACY_ONLY:
        return FrameStatus.LEGACY, None
    if frame.source == SOURCE_SYNTHETIC_TEST:
        return FrameStatus.SYNTHETIC, None
    if frame.source != SOURCE_AUTHORITATIVE_RUNTIME:
        return FrameStatus.UNKNOWN_ORIGIN, None

    # Strict enum/boolean domain validation (fail-closed). A value outside the
    # defined domain is corrupt and must be INVALID, never LIVE.
    if frame.controller_active not in (0, 1):
        return FrameStatus.INVALID, None
    if frame.rl_entered not in (0, 1):
        return FrameStatus.INVALID, None
    if frame.rl_active not in (0, 1):
        return FrameStatus.INVALID, None
    if frame.safety_faulted not in (0, 1):
        return FrameStatus.INVALID, None
    if frame.ray_valid not in (0, 1):
        return FrameStatus.INVALID, None
    if frame.torque_saturated_computed not in (0, 1):
        return FrameStatus.INVALID, None
    if frame.policy_state not in (POLICY_AGILE, POLICY_RECOVERY, POLICY_FAULTED):
        return FrameStatus.INVALID, None
    if frame.ray_origin not in (RAY_UNAVAILABLE, RAY_SHM_RUNTIME):
        return FrameStatus.INVALID, None
    if frame.collision_origin not in (COLLISION_UNAVAILABLE,):
        return FrameStatus.INVALID, None

    # Consistency.
    if frame.controller_active == 0:
        return FrameStatus.INVALID, None
    if frame.rl_active and not frame.rl_entered:
        return FrameStatus.INVALID, None
    if frame.rl_active and frame.safety_faulted:
        return FrameStatus.INVALID, None
    if frame.monotonic_ns == 0:
        return FrameStatus.INVALID, None
    if now_ns < frame.monotonic_ns:
        return FrameStatus.INVALID, None

    # Corrupt payload.
    if not _all_finite(frame):
        return FrameStatus.INVALID, None

    # Freshness.
    if (now_ns - frame.monotonic_ns) > stale_timeout_ns:
        return FrameStatus.STALE, frame
    return FrameStatus.LIVE, frame


def _read_sequence(buf: mmap.mmap) -> int:
    return struct.unpack_from("<Q", buf, _HEADER_SEQUENCE_OFFSET)[0]


def read_shm_frame(
    path: str = SHM_PATH,
    max_attempts: int = 3,
) -> bytes:
    """Read a coherent frame snapshot from the shared-memory file (runtime only).

    Returns the raw 424-byte snapshot, or ``b""`` when the file is missing, too
    small, or never yields a coherent (even, unchanged) sequence. This mirrors the
    two-read seqlock in ``StateRL::updateRay2d``.
    """
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError:
        return b""
    try:
        if os.fstat(fd).st_size < FRAME_SIZE:
            return b""
        buf = mmap.mmap(fd, FRAME_SIZE, access=mmap.ACCESS_READ)
        try:
            for _ in range(max_attempts):
                seq_before = _read_sequence(buf)
                if seq_before == 0 or (seq_before & 1):
                    continue
                snapshot = buf[:FRAME_SIZE]
                seq_after = _read_sequence(buf)
                if seq_before != seq_after or (seq_after & 1):
                    continue
                return snapshot
            return b""
        finally:
            buf.close()
    finally:
        os.close(fd)


class HudState:
    """Minimal read-only HUD state model over the real-time frame.

    ``update`` classifies the latest snapshot; ``display`` returns only real live
    values. Any field whose availability flag is 0 (torque-saturated, collision),
    or whose step was faulted, is surfaced as ``None`` — never a fabricated zero.
    A non-LIVE status suppresses every data field.
    """

    def __init__(self, stale_timeout_ns: int = DEFAULT_STALE_TIMEOUT_NS):
        self.stale_timeout_ns = stale_timeout_ns
        self.status: FrameStatus = FrameStatus.MISSING
        self._frame: Optional[RuntimeFrame] = None

    def update(self, data: Optional[bytes], now_ns: int) -> FrameStatus:
        self.status, self._frame = classify_frame(data, now_ns, self.stale_timeout_ns)
        return self.status

    def display(self) -> Dict[str, object]:
        if self.status is not FrameStatus.LIVE or self._frame is None:
            return {"status": self.status.value, "live": False}
        f = self._frame
        faulted = bool(f.safety_faulted)
        return {
            "status": self.status.value,
            "live": True,
            "session_id": f.session_id,
            "rl_step": f.rl_step,
            "monotonic_ns": f.monotonic_ns,
            "policy_state": _POLICY_NAMES.get(f.policy_state, "UNKNOWN"),
            "safety_faulted": faulted,
            "ra_value": f.ra_value,
            "lin_vel": list(f.lin_vel),
            "command": list(f.command),
            "world_pose": list(f.world_pose),
            "ray_valid": bool(f.ray_valid),
            "ray_age_ns": f.ray_age_ns,
            "ray2d": list(f.ray2d) if f.ray_valid else None,
            "action_raw": None if faulted else list(f.action_raw),
            "action_clipped": None if faulted else list(f.action_clipped),
            "joint_target_rad": None if faulted else list(f.joint_target_rad),
            "torque_nm": None if faulted else list(f.torque_nm),
            # torque_saturated_computed == 0 → never shown as a real value.
            "torque_saturated": None,
            # collision_origin == UNAVAILABLE (bridge-side only) → never shown.
            "collision": None,
        }
