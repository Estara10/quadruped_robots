#!/usr/bin/env python3
"""Offline frame contract + minimal read-only HUD state model for P1-09C.

This module is the consumer side of the single real-time data link described in
``common/abs_rt_frame_contract.h``. It does three things, all fail-closed:

1. Mirrors the fixed 536-byte frame layout and the magic/version/source enums so
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
FRAME_VERSION = 3
FRAME_VERSION_LEGACY = 2
JOINT_COUNT = 12
RAY_COUNT = 11
SHM_NAME = "/mujoco_rt_frame"
SHM_PATH = "/dev/shm/mujoco_rt_frame"

# Field order mirrors common/abs_rt_frame_contract.h: 14 Q, 20 I, one double,
# 83 floats, and four explicit ABI pad bytes.
_FRAME_V2_STRUCT = struct.Struct("<14Q20Id83f4x")
_FRAME_STRUCT = struct.Struct("<14Q20Id83f4xII3d")
FRAME_SIZE = _FRAME_STRUCT.size  # 568
FRAME_V2_SIZE = _FRAME_V2_STRUCT.size  # 536
HISTORY_MAGIC = 0x4142535254484953
HISTORY_VERSION = 1
HISTORY_CAPACITY = 2048
HISTORY_SHM_PATH = "/dev/shm/mujoco_rt_frame_history"
_HISTORY_HEADER = struct.Struct("<6Q")
_HISTORY_SLOT_PREFIX = struct.Struct("<2Q")
HISTORY_SLOT_SIZE = _HISTORY_SLOT_PREFIX.size + FRAME_SIZE

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

SWITCH_STABILIZED = 0
SWITCH_PAPER_FAITHFUL = 1
SWITCH_HYSTERESIS = 2
ACTION_UNKNOWN = 0
ACTION_AGILE = 1
ACTION_RECOVERY = 2
ACTION_NONE = 3
RISK_EDGE_FALSE = 0
RISK_EDGE_TRUE = 1
RISK_EDGE_UNKNOWN = 2
SIM_CLOCK_UNAVAILABLE = 0
SIM_CLOCK_FRESH = 1
SIM_CLOCK_STALE = 2
SIM_CLOCK_RESET = 3
SIM_CLOCK_STATIONARY = 4
SWITCH_MINIMUM_HOLD = 3
SWITCH_COMBINED = 4
HOLD_DISABLED = 0
HOLD_NOT_IN_RECOVERY = 1
HOLD_ACTIVE = 2
HOLD_EXPIRED = 3
HOLD_CLOCK_INVALID = 4

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
    sim_clock_sequence: int
    sim_clock_monotonic_ns: int
    sim_clock_segment_id: int
    sim_clock_age_ns: int
    risk_evaluation_ns: int
    risk_condition_entered_ns: int
    mode_change_ns: int
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
    mode_before: int
    switching_mode: int
    action_source: int
    transition_reason: int
    risk_condition_met: int
    risk_condition_entered: int
    policy_mode_changed: int
    sim_clock_status: int
    sim_clock_valid: int
    reserved_pad: int
    sim_time_s: float
    entry_threshold: float
    exit_threshold: float
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
    hold_status: int = HOLD_DISABLED
    hold_enabled: int = 0
    recovery_min_hold_s: float = 0.0
    hold_enter_sim_time_s: float = 0.0
    hold_elapsed_sim_s: float = 0.0

    @classmethod
    def from_bytes(cls, data: bytes) -> "RuntimeFrame":
        if len(data) not in (FRAME_SIZE, FRAME_V2_SIZE):
            raise ValueError(f"unsupported frame size {len(data)}")
        frame_struct = _FRAME_STRUCT if len(data) == FRAME_SIZE else _FRAME_V2_STRUCT
        values = frame_struct.unpack(data)
        extension = values[-5:] if len(data) == FRAME_SIZE else None
        base_values = values[:-5] if extension is not None else values
        (
            magic, version, sequence, monotonic_ns,
            session_id, rl_step, ray_age_ns, sim_clock_sequence,
            sim_clock_monotonic_ns, sim_clock_segment_id, sim_clock_age_ns,
            risk_evaluation_ns, risk_condition_entered_ns, mode_change_ns,
            source, controller_active, rl_entered, rl_active, safety_faulted,
            policy_state, ray_origin, ray_valid, collision_origin,
            torque_saturated_computed, mode_before, switching_mode, action_source,
            transition_reason, risk_condition_met, risk_condition_entered,
            policy_mode_changed, sim_clock_status, sim_clock_valid, reserved_pad,
            sim_time_s,
        ) = base_values[:35]
        floats = base_values[35:]
        entry_threshold, exit_threshold = floats[:2]
        floats = floats[2:]
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
            sim_clock_sequence=sim_clock_sequence,
            sim_clock_monotonic_ns=sim_clock_monotonic_ns,
            sim_clock_segment_id=sim_clock_segment_id,
            sim_clock_age_ns=sim_clock_age_ns,
            risk_evaluation_ns=risk_evaluation_ns,
            risk_condition_entered_ns=risk_condition_entered_ns,
            mode_change_ns=mode_change_ns,
            source=source, controller_active=controller_active, rl_entered=rl_entered,
            rl_active=rl_active, safety_faulted=safety_faulted, policy_state=policy_state,
            ray_origin=ray_origin, ray_valid=ray_valid, collision_origin=collision_origin,
            torque_saturated_computed=torque_saturated_computed,
            mode_before=mode_before, switching_mode=switching_mode,
            action_source=action_source, transition_reason=transition_reason,
            risk_condition_met=risk_condition_met,
            risk_condition_entered=risk_condition_entered,
            policy_mode_changed=policy_mode_changed,
            sim_clock_status=sim_clock_status, sim_clock_valid=sim_clock_valid,
            reserved_pad=reserved_pad, sim_time_s=sim_time_s,
            entry_threshold=entry_threshold, exit_threshold=exit_threshold,
            ra_value=ra_value, lin_vel=lin_vel, command=command, world_pose=world_pose,
            ray2d=ray2d, action_raw=action_raw, action_clipped=action_clipped,
            joint_target_rad=joint_target_rad, torque_nm=torque_nm,
            torque_saturated=torque_saturated,
            hold_status=extension[0] if extension is not None else HOLD_DISABLED,
            hold_enabled=extension[1] if extension is not None else 0,
            recovery_min_hold_s=extension[2] if extension is not None else 0.0,
            hold_enter_sim_time_s=extension[3] if extension is not None else 0.0,
            hold_elapsed_sim_s=extension[4] if extension is not None else 0.0,
        )


def _all_finite(frame: RuntimeFrame) -> bool:
    groups = (
        (frame.ra_value,),
        (frame.entry_threshold, frame.exit_threshold),
        (frame.recovery_min_hold_s, frame.hold_enter_sim_time_s, frame.hold_elapsed_sim_s),
        (frame.sim_time_s,),
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

    if frame.magic != FRAME_MAGIC or not (
        (frame.version == FRAME_VERSION and len(data) == FRAME_SIZE) or
        (frame.version == FRAME_VERSION_LEGACY and len(data) == FRAME_V2_SIZE)
    ):
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
    if frame.risk_condition_met not in (0, 1) or frame.policy_mode_changed not in (0, 1):
        return FrameStatus.INVALID, None
    if frame.sim_clock_valid not in (0, 1):
        return FrameStatus.INVALID, None
    if frame.policy_state not in (POLICY_AGILE, POLICY_RECOVERY, POLICY_FAULTED):
        return FrameStatus.INVALID, None
    if frame.ray_origin not in (RAY_UNAVAILABLE, RAY_SHM_RUNTIME):
        return FrameStatus.INVALID, None
    if frame.collision_origin not in (COLLISION_UNAVAILABLE,):
        return FrameStatus.INVALID, None
    if frame.mode_before not in (POLICY_AGILE, POLICY_RECOVERY, POLICY_FAULTED):
        return FrameStatus.INVALID, None
    if frame.switching_mode not in (SWITCH_STABILIZED, SWITCH_PAPER_FAITHFUL, SWITCH_HYSTERESIS,
                                     SWITCH_MINIMUM_HOLD, SWITCH_COMBINED):
        if frame.switching_mode != SWITCH_MINIMUM_HOLD:
            return FrameStatus.INVALID, None
    if frame.action_source not in (ACTION_UNKNOWN, ACTION_AGILE, ACTION_RECOVERY, ACTION_NONE):
        return FrameStatus.INVALID, None
    if frame.risk_condition_entered not in (RISK_EDGE_FALSE, RISK_EDGE_TRUE, RISK_EDGE_UNKNOWN):
        return FrameStatus.INVALID, None
    if frame.sim_clock_status not in (SIM_CLOCK_UNAVAILABLE, SIM_CLOCK_FRESH,
                                      SIM_CLOCK_STALE, SIM_CLOCK_RESET,
                                      SIM_CLOCK_STATIONARY):
        return FrameStatus.INVALID, None
    if frame.sim_clock_valid and frame.sim_clock_status not in (SIM_CLOCK_FRESH, SIM_CLOCK_STATIONARY):
        return FrameStatus.INVALID, None
    if frame.version >= FRAME_VERSION:
        if frame.hold_enabled not in (0, 1) or frame.hold_status not in (
            HOLD_DISABLED, HOLD_NOT_IN_RECOVERY, HOLD_ACTIVE, HOLD_EXPIRED, HOLD_CLOCK_INVALID
        ):
            return FrameStatus.INVALID, None
        if frame.hold_enabled:
            if frame.switching_mode not in (SWITCH_MINIMUM_HOLD, SWITCH_COMBINED) or frame.recovery_min_hold_s <= 0:
                return FrameStatus.INVALID, None
            if frame.hold_status == HOLD_DISABLED:
                return FrameStatus.INVALID, None
        elif frame.switching_mode in (SWITCH_MINIMUM_HOLD, SWITCH_COMBINED) or frame.hold_status != HOLD_DISABLED or frame.recovery_min_hold_s != 0.0:
            return FrameStatus.INVALID, None
    if frame.risk_condition_entered == RISK_EDGE_TRUE:
        if not frame.risk_condition_met or frame.risk_condition_entered_ns == 0:
            return FrameStatus.INVALID, None
    elif frame.risk_condition_entered_ns != 0:
        return FrameStatus.INVALID, None
    if frame.policy_mode_changed != (frame.mode_change_ns != 0):
        return FrameStatus.INVALID, None
    if frame.sim_clock_valid:
        if frame.sim_clock_sequence == 0 or frame.sim_clock_monotonic_ns == 0:
            return FrameStatus.INVALID, None
        if frame.sim_clock_age_ns > frame.monotonic_ns - frame.sim_clock_monotonic_ns:
            return FrameStatus.INVALID, None
    if frame.policy_state != POLICY_FAULTED and frame.policy_mode_changed != int(frame.mode_before != frame.policy_state):
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

    Returns the raw 536-byte snapshot, or ``b""`` when the file is missing, too
    small, or never yields a coherent (even, unchanged) sequence. This mirrors the
    two-read seqlock in ``StateRL::updateRay2d``.
    """
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError:
        return b""
    try:
        size = os.fstat(fd).st_size
        if size not in (FRAME_SIZE, FRAME_V2_SIZE):
            return b""
        buf = mmap.mmap(fd, size, access=mmap.ACCESS_READ)
        try:
            for _ in range(max_attempts):
                seq_before = _read_sequence(buf)
                if seq_before == 0 or (seq_before & 1):
                    continue
                snapshot = buf[:size]
                seq_after = _read_sequence(buf)
                if seq_before != seq_after or (seq_after & 1):
                    continue
                return snapshot
            return b""
        finally:
            buf.close()
    finally:
        os.close(fd)


def read_shm_history(path: str = HISTORY_SHM_PATH, cursor: int = 0,
                     expected_session: int | None = None,
                     max_source_sequence: int | None = None) -> dict:
    """Drain committed slots from the controller's bounded SPSC history.

    `cursor` is the next monotonically increasing producer index. When a live
    single-slot snapshot supplies `max_source_sequence`, return only the
    ordered prefix no newer than that snapshot, leaving later slots for the
    next drain. This prevents a racing producer from making persisted frames
    appear after an older direct snapshot.
    The function
    reports overrun and session reset explicitly and never silently skips a
    slot. Returned frame bytes are ordered and each slot seqlock is checked.
    """
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError as exc:
        return {"status": "MISSING", "reason": str(exc), "frames": [], "cursor": cursor}
    try:
        expected_size = _HISTORY_HEADER.size + HISTORY_CAPACITY * HISTORY_SLOT_SIZE
        if os.fstat(fd).st_size != expected_size:
            return {"status": "INVALID", "reason": "history_size_mismatch",
                    "actual_size": os.fstat(fd).st_size, "expected_size": expected_size,
                    "frames": [], "cursor": cursor}
        buf = mmap.mmap(fd, expected_size, access=mmap.ACCESS_READ)
        try:
            magic, version, capacity, session, produced, overwritten = _HISTORY_HEADER.unpack_from(buf, 0)
            if (magic, version, capacity) != (HISTORY_MAGIC, HISTORY_VERSION, HISTORY_CAPACITY):
                return {"status": "INVALID", "reason": "history_header_mismatch",
                        "magic": magic, "version": version, "capacity": capacity,
                        "frames": [], "cursor": cursor}
            if expected_session is not None and session != expected_session:
                return {"status": "SESSION_CHANGED", "session_id": session,
                        "expected_session_id": expected_session, "frames": [], "cursor": cursor}
            if produced < cursor:
                return {"status": "RESET", "reason": "producer_counter_reset",
                        "session_id": session, "produced_count": produced,
                        "overwritten_count": overwritten, "frames": [], "cursor": 0}
            oldest = max(0, produced - capacity)
            expected_slot_reuse_count = oldest
            if overwritten != expected_slot_reuse_count:
                return {"status": "INVALID", "reason": "slot_reuse_counter_mismatch",
                        "session_id": session, "produced_count": produced,
                        "overwritten_count": overwritten,
                        "expected_slot_reuse_count": expected_slot_reuse_count,
                        "frames": [], "cursor": cursor}
            if cursor < oldest:
                return {"status": "OVERFLOW", "session_id": session,
                        "produced_count": produced, "overwritten_count": overwritten,
                        "first_available_index": oldest, "requested_cursor": cursor,
                        "consumer_unread_loss_count": oldest - cursor,
                        "consumer_unread_loss_range": [cursor, oldest],
                        "frames": [], "cursor": cursor}
            result = []
            next_cursor = cursor
            for index in range(cursor, produced):
                offset = _HISTORY_HEADER.size + (index % capacity) * HISTORY_SLOT_SIZE
                before = struct.unpack_from("<Q", buf, offset)[0]
                if before == 0 or before & 1:
                    return {"status": "INCOHERENT", "reason": "slot_uncommitted",
                            "slot_index": index, "frames": result, "cursor": next_cursor,
                            "session_id": session, "produced_count": produced,
                            "overwritten_count": overwritten}
                stored_index = struct.unpack_from("<Q", buf, offset + 8)[0]
                raw = bytes(buf[offset + _HISTORY_SLOT_PREFIX.size:
                                 offset + _HISTORY_SLOT_PREFIX.size + FRAME_SIZE])
                after = struct.unpack_from("<Q", buf, offset)[0]
                if before != after or after & 1 or stored_index != index:
                    return {"status": "INCOHERENT", "reason": "slot_changed_or_wrong_index",
                            "slot_index": index, "stored_index": stored_index,
                            "frames": result, "cursor": next_cursor,
                            "session_id": session, "produced_count": produced,
                            "overwritten_count": overwritten}
                if max_source_sequence is not None:
                    frame = RuntimeFrame.from_bytes(raw)
                    if frame.sequence > max_source_sequence:
                        break
                result.append((index, raw))
                next_cursor = index + 1
            return {"status": "OK", "session_id": session,
                    "produced_count": produced, "overwritten_count": overwritten,
                    "frames": result, "cursor": next_cursor}
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
            "switching_mode": f.switching_mode,
            "entry_threshold": f.entry_threshold,
            "exit_threshold": f.exit_threshold,
            "hold_enabled": bool(f.hold_enabled),
            "hold_status": f.hold_status,
            "recovery_min_hold_s": f.recovery_min_hold_s,
            "hold_enter_sim_time_s": f.hold_enter_sim_time_s if f.hold_enabled else None,
            "hold_elapsed_sim_s": f.hold_elapsed_sim_s
                if f.hold_enabled and f.hold_status in (HOLD_ACTIVE, HOLD_EXPIRED) else None,
            "hold_remaining_sim_s": max(0.0, f.recovery_min_hold_s - f.hold_elapsed_sim_s)
                if f.hold_enabled and f.hold_status == HOLD_ACTIVE else None,
        }
