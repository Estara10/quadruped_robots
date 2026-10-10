#!/usr/bin/env python3
"""Offline/harness side of the opt-in P1-10 common-start contract.

The fixed layout is shared with ``common/abs_stage_a_common_start_contract.h``.
This module only coordinates the gate and validates the producer anchor; it
does not launch a process and never supplies a runtime fallback value.
"""

from __future__ import annotations

import hashlib
import copy
import math
import mmap
import os
import re
import struct
import time
from pathlib import Path
from typing import Any, Dict, Optional


SCHEMA = "abs-go2-p1-10-stage-a-common-start/v1"
VERSION = 1
DEFAULT_SHM_NAME = "/mujoco_p1_10_stage_a_common_start"
MAGIC = 0x4142535354474154
STATE_INVALID = 0
STATE_WAITING_FOR_RELEASE = 1
STATE_RELEASED = 2
STATE_FAILED = 3
FLAG_INITIAL_READY = 1 << 0
FLAG_RELEASE_RECORDED = 1 << 1
FLAG_RL_ENTER_RECORDED = 1 << 2
FLAG_FIRST_PHYSICS_RECORDED = 1 << 3
FLAG_FIRST_RUNTIME_FRAME_RECORDED = 1 << 4
CAPTURE_ID_RE = re.compile(r"^p1-10-capture-[0-9a-f]{32}$")
HEX64_RE = re.compile(r"^[0-9a-f]{64}$")

# magic/version/sequence/state/flags/capture/qpos/11 uint64-or-double fields.
GATE_STRUCT = struct.Struct("<QQQII64s64sQQQQQQQQQQQdII")
GATE_SIZE = GATE_STRUCT.size
STATE_OFFSET = 24
FLAGS_OFFSET = 28
SEQUENCE_OFFSET = 16
RELEASE_MONOTONIC_OFFSET = 168


class CommonStartError(ValueError):
    pass


def canonical_sha256(value: Any) -> str:
    import json

    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=False, allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _shm_path(name: str) -> Path:
    if not isinstance(name, str) or not name.startswith("/") or name == "/":
        raise CommonStartError("shared-memory name must be an absolute POSIX name")
    return Path("/dev/shm") / name[1:]


def _text(raw: bytes) -> str:
    return raw.split(b"\0", 1)[0].decode("ascii", errors="strict")


def _state_name(value: int) -> str:
    return {
        STATE_INVALID: "INVALID",
        STATE_WAITING_FOR_RELEASE: "WAITING_FOR_RELEASE",
        STATE_RELEASED: "RELEASED",
        STATE_FAILED: "FAILED",
    }.get(value, "UNKNOWN")


def _unpack(raw: bytes) -> Dict[str, Any]:
    if len(raw) < GATE_SIZE:
        raise CommonStartError("common-start shared record is truncated")
    values = GATE_STRUCT.unpack(raw[:GATE_SIZE])
    return {
        "magic": values[0],
        "version": values[1],
        "sequence": values[2],
        "state": values[3],
        "state_name": _state_name(values[3]),
        "flags": values[4],
        "capture_id": _text(values[5]),
        "initial_qpos_sha256": _text(values[6]),
        "gate_ready_monotonic_ns": values[7],
        "release_monotonic_ns": values[8],
        "rl_session_id": values[9],
        "rl_step": values[10],
        "rl_frame_sequence": values[11],
        "rl_monotonic_ns": values[12],
        "first_frame_sequence": values[13],
        "first_frame_rl_step": values[14],
        "first_frame_monotonic_ns": values[15],
        "first_physics_step": values[16],
        "first_physics_monotonic_ns": values[17],
        "first_sim_time": values[18],
        "failure_code": values[19],
        "reserved": values[20],
    }


class GateClient:
    """Harness client for one simulator-owned gate shared memory object."""

    def __init__(self, shm_name: str = DEFAULT_SHM_NAME):
        self.shm_name = shm_name
        self._fd = -1
        self._map: Optional[mmap.mmap] = None
        path = _shm_path(shm_name)
        try:
            self._fd = os.open(path, os.O_RDWR)
            size = os.fstat(self._fd).st_size
            if size < GATE_SIZE:
                raise CommonStartError("common-start shared record has wrong size")
            self._map = mmap.mmap(self._fd, GATE_SIZE, access=mmap.ACCESS_WRITE)
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        if self._map is not None:
            self._map.close()
            self._map = None
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    def __enter__(self) -> "GateClient":
        return self

    def __exit__(self, _type, _value, _traceback) -> None:
        self.close()

    def read_snapshot(self) -> Dict[str, Any]:
        if self._map is None:
            raise CommonStartError("common-start client is closed")
        for _ in range(100):
            raw = self._map[:GATE_SIZE]
            before = struct.unpack_from("<Q", raw, 16)[0]
            if before == 0 or before & 1:
                continue
            value = _unpack(raw)
            after = struct.unpack_from("<Q", self._map, 16)[0]
            if before == after and not after & 1:
                if value["magic"] != MAGIC or value["version"] != VERSION:
                    raise CommonStartError("common-start schema/magic mismatch")
                return value
        raise CommonStartError("common-start shared record could not be read coherently")

    def wait_ready(self, timeout_s: float) -> Dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        last: Optional[Dict[str, Any]] = None
        while time.monotonic() < deadline:
            try:
                last = self.read_snapshot()
            except CommonStartError:
                last = None
            if (last is not None and last["state"] == STATE_WAITING_FOR_RELEASE and
                    last["flags"] & FLAG_INITIAL_READY and
                    CAPTURE_ID_RE.fullmatch(last["capture_id"] or "") and
                    HEX64_RE.fullmatch(last["initial_qpos_sha256"] or "")):
                return last
            time.sleep(0.005)
        raise CommonStartError("common-start gate did not become ready")

    def release(self, capture_id: str) -> Dict[str, Any]:
        if CAPTURE_ID_RE.fullmatch(capture_id or "") is None:
            raise CommonStartError("release requires a harness-generated capture ID")
        current = self.read_snapshot()
        if current["state"] != STATE_WAITING_FOR_RELEASE:
            raise CommonStartError("gate release is missing or already consumed")
        if current["flags"] & FLAG_RELEASE_RECORDED:
            raise CommonStartError("duplicate gate release")
        if current["capture_id"] != capture_id:
            raise CommonStartError("capture identity does not match gate")
        if not current["flags"] & FLAG_INITIAL_READY:
            raise CommonStartError("gate is not initial-state ready")
        if self._map is None:
            raise CommonStartError("common-start client is closed")
        release_ns = time.monotonic_ns()
        # Use the same seqlock boundary as the C++ writers.  Release mutates
        # three shared fields; publishing them without an odd sequence marker
        # would let a reader observe a torn release/flag/state combination.
        sequence = current["sequence"]
        if sequence == 0 or sequence & 1:
            raise CommonStartError("common-start gate sequence is not writable")
        struct.pack_into("<Q", self._map, SEQUENCE_OFFSET, sequence + 1)
        struct.pack_into("<Q", self._map, RELEASE_MONOTONIC_OFFSET, release_ns)
        struct.pack_into("<I", self._map, FLAGS_OFFSET,
                         current["flags"] | FLAG_RELEASE_RECORDED)
        struct.pack_into("<I", self._map, STATE_OFFSET, STATE_RELEASED)
        struct.pack_into("<Q", self._map, SEQUENCE_OFFSET, sequence + 2)
        self._map.flush()
        # The state is published as part of the final even sequence: the
        # physics writer cannot pass the gate until timestamp, flags, and state
        # are all visible in one coherent snapshot.
        released = self.read_snapshot()
        if released["state"] != STATE_RELEASED or released["release_monotonic_ns"] != release_ns:
            raise CommonStartError("gate release was not durably observed")
        return released

    def wait_anchor(self, timeout_s: float, expected_capture_id: str,
                    expected_qpos_sha256: str) -> Dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                value = self.read_snapshot()
            except CommonStartError:
                time.sleep(0.005)
                continue
            if value["state"] != STATE_RELEASED:
                time.sleep(0.005)
                continue
            if value["capture_id"] != expected_capture_id or value["initial_qpos_sha256"] != expected_qpos_sha256:
                raise CommonStartError("common-start anchor binding mismatch")
            required = ("release_monotonic_ns", "rl_session_id", "rl_monotonic_ns",
                        "first_frame_sequence", "first_frame_monotonic_ns",
                        "first_physics_step", "first_physics_monotonic_ns")
            if all(isinstance(value[key], int) and value[key] > 0 for key in required):
                if value["first_sim_time"] != value["first_sim_time"]:
                    raise CommonStartError("common-start first sim time is NaN")
                return self.anchor_from_snapshot(value)
            time.sleep(0.005)
        raise CommonStartError("producer-side common-start anchor did not arrive")

    @staticmethod
    def anchor_from_snapshot(value: Dict[str, Any]) -> Dict[str, Any]:
        required = ("capture_id", "initial_qpos_sha256", "release_monotonic_ns",
                    "rl_session_id", "rl_step", "rl_frame_sequence",
                    "rl_monotonic_ns", "first_frame_sequence",
                    "first_frame_rl_step", "first_frame_monotonic_ns",
                    "first_physics_step", "first_physics_monotonic_ns",
                    "first_sim_time", "state")
        if any(key not in value for key in required):
            raise CommonStartError("common-start anchor is incomplete")
        return {
            "schema": SCHEMA,
            "version": VERSION,
            "capture_id": value["capture_id"],
            "initial_qpos_sha256": value["initial_qpos_sha256"],
            "gate_state": value["state_name"],
            "release_monotonic_ns": value["release_monotonic_ns"],
            "rl_session_id": value["rl_session_id"],
            "rl_step": value["rl_step"],
            "rl_frame_sequence": value["rl_frame_sequence"],
            "rl_monotonic_ns": value["rl_monotonic_ns"],
            "first_runtime_frame_sequence": value["first_frame_sequence"],
            "first_runtime_frame_rl_step": value["first_frame_rl_step"],
            "first_runtime_frame_monotonic_ns": value["first_frame_monotonic_ns"],
            "first_physics_step": value["first_physics_step"],
            "first_physics_monotonic_ns": value["first_physics_monotonic_ns"],
            "first_sim_time": value["first_sim_time"],
            "flags": value["flags"],
        }


def validate_anchor(anchor: Any, *, expected_capture_id: str,
                    expected_qpos_sha256: str) -> None:
    if not isinstance(anchor, dict):
        raise CommonStartError("common-start anchor must be an object")
    required = {
        "schema", "version", "capture_id", "initial_qpos_sha256", "gate_state",
        "release_monotonic_ns", "rl_session_id", "rl_step", "rl_frame_sequence",
        "rl_monotonic_ns", "first_runtime_frame_sequence",
        "first_runtime_frame_rl_step", "first_runtime_frame_monotonic_ns",
        "first_physics_step", "first_physics_monotonic_ns", "first_sim_time", "flags",
    }
    if set(anchor) != required:
        raise CommonStartError("common-start anchor fields are not exact")
    if anchor["schema"] != SCHEMA or anchor["version"] != VERSION:
        raise CommonStartError("common-start anchor schema/version mismatch")
    if anchor["capture_id"] != expected_capture_id or CAPTURE_ID_RE.fullmatch(anchor["capture_id"]) is None:
        raise CommonStartError("common-start anchor capture mismatch")
    if anchor["initial_qpos_sha256"] != expected_qpos_sha256 or HEX64_RE.fullmatch(anchor["initial_qpos_sha256"]) is None:
        raise CommonStartError("common-start anchor qpos binding mismatch")
    if anchor["gate_state"] != "RELEASED":
        raise CommonStartError("common-start anchor gate is not RELEASED")
    nonnegative_fields = {"rl_step", "rl_frame_sequence", "first_runtime_frame_rl_step"}
    integer_fields = {
        "version", "release_monotonic_ns", "rl_session_id", "rl_step",
        "rl_frame_sequence", "rl_monotonic_ns", "first_runtime_frame_sequence",
        "first_runtime_frame_rl_step", "first_runtime_frame_monotonic_ns",
        "first_physics_step", "first_physics_monotonic_ns", "flags",
    }
    if any(isinstance(anchor[key], bool) or not isinstance(anchor[key], int) or
           anchor[key] < (0 if key in nonnegative_fields else 1)
           for key in integer_fields):
        raise CommonStartError("common-start anchor contains invalid integer")
    if isinstance(anchor["first_sim_time"], bool) or not isinstance(anchor["first_sim_time"], (int, float)):
        raise CommonStartError("common-start anchor sim time is invalid")
    if not math.isfinite(float(anchor["first_sim_time"])) or anchor["first_sim_time"] < 0:
        raise CommonStartError("common-start anchor sim time is invalid")
    if isinstance(anchor["flags"], bool) or not isinstance(anchor["flags"], int):
        raise CommonStartError("common-start anchor flags are invalid")
    expected_flags = (FLAG_INITIAL_READY | FLAG_RELEASE_RECORDED |
                      FLAG_RL_ENTER_RECORDED | FLAG_FIRST_PHYSICS_RECORDED |
                      FLAG_FIRST_RUNTIME_FRAME_RECORDED)
    if anchor["flags"] & expected_flags != expected_flags:
        raise CommonStartError("common-start anchor flags are incomplete")


def build_environment(capture_id: str, qpos_sha256: str, manifest_sha256: str,
                      identity_sha256: str) -> Dict[str, str]:
    if CAPTURE_ID_RE.fullmatch(capture_id or "") is None:
        raise CommonStartError("invalid harness capture ID")
    if HEX64_RE.fullmatch(qpos_sha256 or "") is None:
        raise CommonStartError("invalid qpos binding")
    for label, value in (("manifest", manifest_sha256), ("identity", identity_sha256)):
        if HEX64_RE.fullmatch(value or "") is None:
            raise CommonStartError(f"invalid {label} binding")
    return {
        "ABS_P1_10_COMMON_START": "1",
        "ABS_P1_10_COMMON_START_CONTRACT_VERSION": str(VERSION),
        "ABS_P1_10_COMMON_START_SHM": DEFAULT_SHM_NAME,
        "ABS_P1_10_INITIAL_QPOS_SHA256": qpos_sha256,
        "ABS_P1_10_STAGE_A_COMMON_START_MANIFEST_SHA256": manifest_sha256,
        "ABS_P1_10_STAGE_A_COMMON_START_IDENTITY_SHA256": identity_sha256,
    }


def bind_context(resolved_context: Dict[str, Any], validation: Dict[str, Any],
                 capture_id: str) -> Dict[str, Any]:
    """Attach the explicit common-start contract to one resolved context."""
    bound = copy.deepcopy(resolved_context)
    qpos_sha256 = bound["initial_state"]["qpos_sha256"]
    manifest_sha256 = validation["manifest_sha256"]
    identity_sha256 = validation["identity_sha256"]
    contract = {
        "schema": SCHEMA,
        "version": VERSION,
        "enabled": True,
        "shared_memory_name": DEFAULT_SHM_NAME,
        "gate_release": {
            "producer": "scripts/p1_08_baseline_capture.py::GateClient.release",
            "one_shot": True,
            "requires_controller_active": True,
            "requires_rl_active": True,
            "requires_declared_goal_command": True,
            "capture_id_source": "harness-generated; not a CLI input",
            "timeout_is_failure": True,
        },
        "initial_state": {
            "source": "scene_default",
            "reset_source": "mj_makeData:qpos0; no keyframe reset",
            "qpos_sha256": qpos_sha256,
            "physics_steps_before_release": 0,
        },
        "anchor": {
            "producer": "StateRL::enter + StateRL::writeRtFrame",
            "required_fields": [
                "capture_id", "initial_qpos_sha256", "gate_state",
                "release_monotonic_ns", "rl_session_id", "rl_step",
                "rl_frame_sequence", "rl_monotonic_ns",
                "first_runtime_frame_sequence", "first_runtime_frame_rl_step",
                "first_runtime_frame_monotonic_ns", "first_physics_step",
                "first_physics_monotonic_ns", "first_sim_time", "flags",
            ],
            "record_binding": "runtime_record terminal.start_anchor and first LIVE frame.start_anchor_ref",
        },
        "manifest": {
            "path": validation["manifest_path"],
            "sha256": manifest_sha256,
            "identity_sha256": identity_sha256,
        },
        "formal_scope": "main.cc PhysicsLoop only; simulate.cc UI step-forward is excluded",
    }
    bound["common_start_contract"] = contract
    bound["launch_contract"]["common_start"] = True
    bound["launch_contract"]["common_start_contract_schema"] = SCHEMA
    bound["process_context"]["environment"].update(
        build_environment(capture_id, qpos_sha256, manifest_sha256, identity_sha256))
    bound["process_context"]["common_start_contract"] = contract
    return bound
