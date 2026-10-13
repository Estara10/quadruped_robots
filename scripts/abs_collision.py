#!/usr/bin/env python3
"""Reader and validator for the formal P1-10 collision snapshot.

The legacy five-int32 ``/mujoco_collision`` buffer is intentionally not read.
This reader only accepts the versioned physics-step snapshot written by the
simulator authority and preserves missing/unknown/stale/invalid states.
"""

from __future__ import annotations

import enum
import errno
import hashlib
import math
import mmap
import os
import re
import struct
import time
from dataclasses import dataclass, replace
from typing import Optional, Tuple

from abs_scene import LEGACY_BINDING

MAGIC = 0x414253434F4E5432
VERSION_V3 = 3
VERSION_V4 = 4
VERSION_V5 = 5
VERSION = VERSION_V5
SHM_NAME = "/mujoco_collision_v2"
SHM_PATH = "/dev/shm/mujoco_collision_v2"
SCENARIO_ID = str(LEGACY_BINDING["scene_id"])
SCENE_ROOT_SHA256 = str(LEGACY_BINDING["root_sha256"])
MODEL_CLOSURE_SHA256 = str(LEGACY_BINDING["closure_sha256"])
BASE_STRUCT = struct.Struct("<5Qd10I2iI32s64s64s64s64s4x")
BASE_DATA_SIZE = BASE_STRUCT.size - 4  # trailing alignment belongs after the complete C++ Snapshot
EXTENSION_STRUCT = struct.Struct("<2I4x4Q8d2Q6I")
EPISODE_STRUCT = struct.Struct("<QQddii32s32s")
COLLISION_EVENT_CAPACITY = 32
FALL_EVENT_STRUCT = struct.Struct("<QQQdddII")
V4_EXTENSION_STRUCT = struct.Struct("<8Q4d2I")
FALL_EVENT_CAPACITY = 32
LEGACY_SNAPSHOT_SIZE = BASE_DATA_SIZE + EXTENSION_STRUCT.size + EPISODE_STRUCT.size * COLLISION_EVENT_CAPACITY
SNAPSHOT_SIZE = (LEGACY_SNAPSHOT_SIZE + V4_EXTENSION_STRUCT.size +
                 FALL_EVENT_STRUCT.size * FALL_EVENT_CAPACITY)
SNAPSHOT_V4_SIZE = SNAPSHOT_SIZE
FOOT_EVENT_CAPACITY = 32
FOOT_EVENT_STRUCT = struct.Struct("<QQddiiII3d8s32s")
V5_EXTENSION_STRUCT = struct.Struct(
    "<12I11Q32d" + FOOT_EVENT_STRUCT.format[1:] * (2 * FOOT_EVENT_CAPACITY) +
    "32i" + "32s" * COLLISION_EVENT_CAPACITY
)
V5_FIXED_STRUCT = struct.Struct("<12I11Q32d")
SNAPSHOT_V5_SIZE = SNAPSHOT_V4_SIZE + V5_EXTENSION_STRUCT.size
SNAPSHOT_SIZE = SNAPSHOT_V5_SIZE
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
    foot_ground_contacts: int
    nonfoot_ground_contacts: int
    collision_contact_steps: int
    collision_episode_count: int
    last_collision_start_step: int
    last_collision_end_step: int
    collision_duration_s: float
    last_collision_start_sim_time: float
    last_collision_end_sim_time: float
    base_height_m: float
    base_roll_rad: float
    base_pitch_rad: float
    fall_start_sim_time: float
    fall_confirmed_sim_time: float
    fall_start_physics_step: int
    fall_confirmed_physics_step: int
    fall_candidate: int
    fall_confirmed: int
    collision_history_count: int
    collision_history_overflow: int
    physics_coverage_complete: int
    reserved_extension: int
    collision_history: tuple[dict, ...]
    unknown_contact_steps: int
    unknown_contact_episodes: int
    first_unknown_contact_step: int
    last_unknown_contact_step: int
    invalid_posture_steps: int
    invalid_posture_episodes: int
    first_invalid_posture_step: int
    last_invalid_posture_step: int
    first_unknown_contact_sim_time: float
    last_unknown_contact_sim_time: float
    first_invalid_posture_sim_time: float
    last_invalid_posture_sim_time: float
    fall_history_count: int
    fall_history_overflow: int
    fall_history: tuple[dict, ...]
    event_latches_complete: bool
    layout_size: int
    foot_identity_valid: Optional[int] = None
    nonfoot_obstacle_contacts: Optional[int] = None
    foot_obstacle_contacts: Optional[int] = None
    foot_contact_mask: Optional[int] = None
    foot_impact_mask: Optional[int] = None
    foot_force_valid_mask: Optional[int] = None
    foot_impact_unknown_mask: Optional[int] = None
    foot_contact_history_count: Optional[int] = None
    foot_contact_history_overflow: Optional[int] = None
    foot_impact_history_count: Optional[int] = None
    foot_impact_history_overflow: Optional[int] = None
    foot_contact_physics_steps: Optional[int] = None
    foot_contact_episode_count: Optional[int] = None
    foot_contact_first_step: Optional[int] = None
    foot_contact_last_step: Optional[int] = None
    foot_impact_physics_steps: Optional[int] = None
    foot_impact_episode_count: Optional[int] = None
    foot_impact_first_step: Optional[int] = None
    foot_impact_last_step: Optional[int] = None
    foot_impact_unknown_steps: Optional[int] = None
    foot_impact_unknown_first_step: Optional[int] = None
    foot_impact_unknown_last_step: Optional[int] = None
    foot_contact_duration_s: Optional[float] = None
    foot_contact_first_sim_time: Optional[float] = None
    foot_contact_last_sim_time: Optional[float] = None
    foot_impact_duration_s: Optional[float] = None
    foot_impact_first_sim_time: Optional[float] = None
    foot_impact_last_sim_time: Optional[float] = None
    foot_impact_unknown_first_sim_time: Optional[float] = None
    foot_impact_unknown_last_sim_time: Optional[float] = None
    foot_world_force: Optional[tuple] = None
    foot_fxy: Optional[tuple] = None
    foot_abs_fz: Optional[tuple] = None
    foot_impact_threshold: Optional[tuple] = None
    foot_contact_history: tuple = ()
    foot_impact_history: tuple = ()
    nonfoot_collision_bodies: tuple = ()
    # v5 reserved_flags bits: low-height and tilt conditions seen in the current/latest episode.
    fall_condition_flags: Optional[int] = None


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
    if len(raw) not in (LEGACY_SNAPSHOT_SIZE, SNAPSHOT_V4_SIZE, SNAPSHOT_V5_SIZE):
        raise ValueError("wrong collision snapshot size")
    base = BASE_STRUCT.unpack_from(raw, 0)
    if not ((base[1] == VERSION_V3 and len(raw) == LEGACY_SNAPSHOT_SIZE) or
            (base[1] == VERSION_V4 and len(raw) == SNAPSHOT_V4_SIZE) or
            (base[1] == VERSION_V5 and len(raw) == SNAPSHOT_V5_SIZE)):
        raise ValueError("collision snapshot version/size mismatch")
    ext = EXTENSION_STRUCT.unpack_from(raw, BASE_DATA_SIZE)
    strings = [value.split(b"\0", 1)[0].decode("ascii") for value in base[-5:]]
    count = ext[-4]
    events = []
    offset = BASE_DATA_SIZE + EXTENSION_STRUCT.size
    if len(raw) >= SNAPSHOT_V4_SIZE:
        offset += V4_EXTENSION_STRUCT.size + FALL_EVENT_STRUCT.size * FALL_EVENT_CAPACITY
    for index in range(COLLISION_EVENT_CAPACITY):
        item = EPISODE_STRUCT.unpack_from(raw, offset + index * EPISODE_STRUCT.size)
        if index >= count:
            continue
        events.append({"_ring_index": index, "start_physics_step": item[0], "end_physics_step": item[1],
                       "start_sim_time": item[2], "end_sim_time": item[3],
                       "robot_geom_id": item[4], "obstacle_geom_id": item[5],
                       "robot_geom_name": item[6].split(b"\0", 1)[0].decode("ascii"),
                       "obstacle_geom_name": item[7].split(b"\0", 1)[0].decode("ascii")})
    events.sort(key=lambda event: event["start_physics_step"])
    layout_size = len(raw)
    if len(raw) == LEGACY_SNAPSHOT_SIZE:
        return CollisionSnapshot(*base[:-5], *strings, *ext, tuple(events),
                                 0, 0, 0, 0, 0, 0, 0, 0,
                                 0.0, 0.0, 0.0, 0.0, 0, 0, (), False, layout_size)
    extra_offset = BASE_DATA_SIZE + EXTENSION_STRUCT.size
    extra = V4_EXTENSION_STRUCT.unpack_from(raw, extra_offset)
    fall_offset = extra_offset + V4_EXTENSION_STRUCT.size
    fall_count, fall_overflow = extra[-2:]
    falls = []
    for index in range(FALL_EVENT_CAPACITY):
        item = FALL_EVENT_STRUCT.unpack_from(raw, fall_offset + index * FALL_EVENT_STRUCT.size)
        if index < fall_count:
            if item[6] not in (0, 1) or item[7] not in (0, 1):
                raise ValueError("invalid fall event flags")
            falls.append({"start_physics_step": item[0], "end_physics_step": item[1],
                          "confirmed_physics_step": item[2], "start_sim_time": item[3],
                          "end_sim_time": item[4], "confirmed_sim_time": item[5],
                          "confirmed": bool(item[6]), "closed": bool(item[7])})
    falls.sort(key=lambda event: event["start_physics_step"])
    extra_payload = (*extra[:8], *extra[8:12], fall_count, fall_overflow, tuple(falls))
    snapshot = CollisionSnapshot(*base[:-5], *strings, *ext, tuple(events), *extra_payload, True, layout_size)
    if base[1] != VERSION_V5:
        return snapshot

    v5_offset = SNAPSHOT_V4_SIZE
    v5 = V5_EXTENSION_STRUCT.unpack_from(raw, v5_offset)
    integers = v5[:12]
    counters = v5[12:23]
    values = v5[23:55]  # eight event times + 24 per-foot force values
    event_offset = V5_FIXED_STRUCT.size
    contact_events, impact_events = [], []
    foot_contact_count = integers[7]
    foot_impact_count = integers[9]
    for event_index in range(2 * FOOT_EVENT_CAPACITY):
        item = FOOT_EVENT_STRUCT.unpack_from(raw, v5_offset + event_offset +
            event_index * FOOT_EVENT_STRUCT.size)
        if event_index >= FOOT_EVENT_CAPACITY:
            index = event_index - FOOT_EVENT_CAPACITY
            if index >= foot_impact_count:
                continue
            target = impact_events
        else:
            index = event_index
            if index >= foot_contact_count:
                continue
            target = contact_events
        if item[6] not in (0, 1) or item[7] not in (0, 1):
            raise ValueError("invalid foot event flags")
        target.append({"start_physics_step": item[0], "end_physics_step": item[1],
                       "start_sim_time": item[2], "end_sim_time": item[3],
                       "foot_geom_id": item[4], "obstacle_geom_id": item[5],
                       "closed": bool(item[6]), "force_known": bool(item[7]),
                       "trigger_fxy": item[8], "trigger_abs_fz": item[9],
                       "trigger_threshold": item[10],
                       "foot_geom_name": item[11].split(b"\0", 1)[0].decode("ascii"),
                       "obstacle_geom_name": item[12].split(b"\0", 1)[0].decode("ascii")})
    body_ids_offset = v5_offset + event_offset + 2 * FOOT_EVENT_CAPACITY * FOOT_EVENT_STRUCT.size
    body_ids = struct.unpack_from("<32i", raw, body_ids_offset)
    body_names = struct.unpack_from("<" + "32s" * COLLISION_EVENT_CAPACITY,
                                    raw, body_ids_offset + 32 * 4)
    nonfoot_bodies = []
    for index, event in enumerate(events):
        # The native ring is indexed by episode number modulo its capacity.
        slot = event.pop("_ring_index")
        event["robot_body_id"] = body_ids[slot]
        event["robot_body_name"] = body_names[slot].split(b"\0", 1)[0].decode("ascii")
        nonfoot_bodies.append({"robot_body_id": event["robot_body_id"],
                               "robot_body_name": event["robot_body_name"]})
    # v5 fixed data values: 8 times, 12 world-force components, then 3x4 scalars.
    world_force = tuple(tuple(values[8 + i * 3 + axis] for axis in range(3)) for i in range(4))
    cursor = 20
    fxy = tuple(values[cursor:cursor + 4]); cursor += 4
    abs_fz = tuple(values[cursor:cursor + 4]); cursor += 4
    thresholds = tuple(values[cursor:cursor + 4])
    kwargs = {
        "foot_identity_valid": integers[0], "nonfoot_obstacle_contacts": integers[1],
        "foot_obstacle_contacts": integers[2], "foot_contact_mask": integers[3],
        "foot_impact_mask": integers[4], "foot_force_valid_mask": integers[5],
        "foot_impact_unknown_mask": integers[6],
        "foot_contact_history_count": integers[7], "foot_contact_history_overflow": integers[8],
        "foot_impact_history_count": integers[9], "foot_impact_history_overflow": integers[10],
        "foot_contact_physics_steps": counters[0], "foot_contact_episode_count": counters[1],
        "foot_contact_first_step": counters[2], "foot_contact_last_step": counters[3],
        "foot_impact_physics_steps": counters[4], "foot_impact_episode_count": counters[5],
        "foot_impact_first_step": counters[6], "foot_impact_last_step": counters[7],
        "foot_impact_unknown_steps": counters[8], "foot_impact_unknown_first_step": counters[9],
        "foot_impact_unknown_last_step": counters[10],
        "fall_condition_flags": integers[11],
        "foot_contact_duration_s": values[0], "foot_contact_first_sim_time": values[1],
        "foot_contact_last_sim_time": values[2], "foot_impact_duration_s": values[3],
        "foot_impact_first_sim_time": values[4], "foot_impact_last_sim_time": values[5],
        "foot_impact_unknown_first_sim_time": values[6], "foot_impact_unknown_last_sim_time": values[7],
        "foot_world_force": world_force, "foot_fxy": fxy, "foot_abs_fz": abs_fz,
        "foot_impact_threshold": thresholds, "foot_contact_history": tuple(contact_events),
        "foot_impact_history": tuple(impact_events), "nonfoot_collision_bodies": tuple(nonfoot_bodies)}
    return replace(snapshot, **kwargs)


def _valid(snapshot: CollisionSnapshot, now_ns: int,
           expected_capture_id: Optional[str] = None,
           expected_fingerprint: Optional[str] = None,
           expected_scene_binding: Optional[dict] = None) -> bool:
    supported_layout = ((snapshot.version == VERSION_V3 and snapshot.layout_size == LEGACY_SNAPSHOT_SIZE) or
                        (snapshot.version == VERSION_V4 and snapshot.layout_size == SNAPSHOT_V4_SIZE) or
                        (snapshot.version == VERSION_V5 and snapshot.layout_size == SNAPSHOT_V5_SIZE))
    if snapshot.magic != MAGIC or not supported_layout:
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
    if snapshot.foot_ground_contacts + snapshot.nonfoot_ground_contacts != snapshot.ground_contacts:
        return False
    if (snapshot.version == VERSION_V5 and
            (snapshot.fall_condition_flags is None or snapshot.fall_condition_flags & ~0x3)):
        return False
    if snapshot.collision_history_count > COLLISION_EVENT_CAPACITY:
        return False
    if snapshot.collision_history_count != min(snapshot.collision_episode_count, COLLISION_EVENT_CAPACITY):
        return False
    if snapshot.collision_history_overflow not in (0, 1) or snapshot.fall_candidate not in (0, 1) or snapshot.fall_confirmed not in (0, 1):
        return False
    if snapshot.fall_history_count > FALL_EVENT_CAPACITY or snapshot.fall_history_overflow not in (0, 1):
        return False
    if snapshot.event_latches_complete not in (False, True):
        return False
    if snapshot.unknown_contact_steps == 0 and (snapshot.first_unknown_contact_step != 0 or
                                                 snapshot.last_unknown_contact_step != 0):
        return False
    if snapshot.invalid_posture_steps == 0 and (snapshot.first_invalid_posture_step != 0 or
                                                 snapshot.last_invalid_posture_step != 0):
        return False
    if snapshot.unknown_contact_steps > 0 and (
            snapshot.first_unknown_contact_step == 0 or snapshot.last_unknown_contact_step == 0 or
            snapshot.last_unknown_contact_step < snapshot.first_unknown_contact_step):
        return False
    if snapshot.invalid_posture_steps > 0 and (
            snapshot.first_invalid_posture_step == 0 or snapshot.last_invalid_posture_step == 0 or
            snapshot.last_invalid_posture_step < snapshot.first_invalid_posture_step):
        return False
    if (snapshot.unknown_contact_steps > 0 and
            snapshot.first_unknown_contact_sim_time > snapshot.last_unknown_contact_sim_time):
        return False
    if (snapshot.invalid_posture_steps > 0 and
            snapshot.first_invalid_posture_sim_time > snapshot.last_invalid_posture_sim_time):
        return False
    for event in snapshot.fall_history:
        if (event["start_physics_step"] == 0 or
                event["end_physics_step"] < event["start_physics_step"] or
                not all(math.isfinite(event[key]) for key in
                        ("start_sim_time", "end_sim_time", "confirmed_sim_time")) or
                event["end_sim_time"] < event["start_sim_time"]):
            return False
        if event["confirmed"] and (
                event["confirmed_physics_step"] < event["start_physics_step"] or
                event["confirmed_physics_step"] > event["end_physics_step"] or
                event["confirmed_sim_time"] < event["start_sim_time"] or
                event["confirmed_sim_time"] > event["end_sim_time"]):
            return False
        if not event["confirmed"] and event["confirmed_physics_step"] != 0:
            return False
    if snapshot.physics_coverage_complete not in (0, 1):
        return False
    if snapshot.fall_confirmed and (snapshot.fall_confirmed_physics_step == 0 or
                                    not math.isfinite(snapshot.fall_confirmed_sim_time)):
        return False
    if snapshot.fall_candidate and snapshot.fall_start_physics_step == 0:
        return False
    if snapshot.collision_duration_s < 0 or snapshot.collision_contact_steps < 0:
        return False
    if snapshot.collision_history_overflow != (1 if snapshot.collision_episode_count > COLLISION_EVENT_CAPACITY else 0):
        return False
    if any(event["end_physics_step"] < event["start_physics_step"] or
           event["end_sim_time"] < event["start_sim_time"] for event in snapshot.collision_history):
        return False
    if not all(math.isfinite(value) for value in (snapshot.collision_duration_s,
            snapshot.last_collision_start_sim_time, snapshot.last_collision_end_sim_time,
            snapshot.base_height_m, snapshot.base_roll_rad, snapshot.base_pitch_rad,
            snapshot.fall_start_sim_time, snapshot.fall_confirmed_sim_time)):
        return False
    if not all(math.isfinite(value) for value in (snapshot.first_unknown_contact_sim_time,
            snapshot.last_unknown_contact_sim_time, snapshot.first_invalid_posture_sim_time,
            snapshot.last_invalid_posture_sim_time)):
        return False
    if snapshot.unknown_contact_steps < 0 or snapshot.invalid_posture_steps < 0:
        return False
    if snapshot.last_contact_class not in (0, 1, 2, 3, 4, 5):
        return False
    if snapshot.authoritative:
        binding = expected_scene_binding or LEGACY_BINDING
        if snapshot.scenario_id != binding.get("scene_id"):
            return False
        if snapshot.scene_root_sha256 != binding.get("root_sha256"):
            return False
        if snapshot.model_closure_sha256 != binding.get("closure_sha256"):
            return False
        if (expected_scene_binding is not None and
                snapshot.runtime_model_fingerprint != binding.get("model_fingerprint")):
            return False
        if snapshot.invalid_reason != 0:
            return False
        if snapshot.version == VERSION_V5:
            if (snapshot.foot_identity_valid != 1 or
                    snapshot.nonfoot_obstacle_contacts is None or
                    snapshot.foot_obstacle_contacts is None or
                    snapshot.nonfoot_obstacle_contacts + snapshot.foot_obstacle_contacts !=
                    snapshot.robot_obstacle_contacts or
                    snapshot.current_collision != (1 if snapshot.nonfoot_obstacle_contacts > 0 else 0)):
                return False
            if any(value is None or value < 0 for value in (
                    snapshot.foot_contact_history_count, snapshot.foot_impact_history_count,
                    snapshot.foot_contact_episode_count, snapshot.foot_impact_episode_count,
                    snapshot.foot_contact_physics_steps, snapshot.foot_impact_physics_steps,
                    snapshot.foot_impact_unknown_steps)):
                return False
            if (snapshot.foot_contact_history_count > FOOT_EVENT_CAPACITY or
                    snapshot.foot_impact_history_count > FOOT_EVENT_CAPACITY or
                    snapshot.foot_contact_history_count != min(snapshot.foot_contact_episode_count, FOOT_EVENT_CAPACITY) or
                    snapshot.foot_impact_history_count != min(snapshot.foot_impact_episode_count, FOOT_EVENT_CAPACITY) or
                    snapshot.foot_contact_history_overflow != int(snapshot.foot_contact_episode_count > FOOT_EVENT_CAPACITY) or
                    snapshot.foot_impact_history_overflow != int(snapshot.foot_impact_episode_count > FOOT_EVENT_CAPACITY)):
                return False
            masks = (snapshot.foot_contact_mask, snapshot.foot_impact_mask,
                     snapshot.foot_force_valid_mask, snapshot.foot_impact_unknown_mask)
            if any(value is None or value < 0 or value > 0xF for value in masks):
                return False
            if (snapshot.foot_impact_mask & ~snapshot.foot_contact_mask or
                    snapshot.foot_impact_unknown_mask & ~snapshot.foot_contact_mask or
                    snapshot.foot_impact_mask & snapshot.foot_impact_unknown_mask or
                    snapshot.foot_impact_unknown_mask & snapshot.foot_force_valid_mask):
                return False
            v5_times = (snapshot.foot_contact_duration_s, snapshot.foot_contact_first_sim_time,
                        snapshot.foot_contact_last_sim_time, snapshot.foot_impact_duration_s,
                        snapshot.foot_impact_first_sim_time, snapshot.foot_impact_last_sim_time,
                        snapshot.foot_impact_unknown_first_sim_time,
                        snapshot.foot_impact_unknown_last_sim_time)
            if any(value is None or not math.isfinite(value) for value in v5_times):
                return False
            if any(len(values) != 4 for values in (snapshot.foot_world_force, snapshot.foot_fxy,
                                                   snapshot.foot_abs_fz, snapshot.foot_impact_threshold)):
                return False
            if any(not math.isfinite(value) for row in snapshot.foot_world_force for value in row):
                return False
            if any(not math.isfinite(value) for values in (snapshot.foot_fxy, snapshot.foot_abs_fz,
                                                            snapshot.foot_impact_threshold) for value in values):
                return False
            for event in (*snapshot.foot_contact_history, *snapshot.foot_impact_history):
                if (event["start_physics_step"] == 0 or
                        event["end_physics_step"] < event["start_physics_step"] or
                        event["end_sim_time"] < event["start_sim_time"] or
                        event["foot_geom_name"] not in {"FL", "FR", "RL", "RR"} or
                        not event["obstacle_geom_name"] or
                        (event["force_known"] and not all(math.isfinite(event[key]) for key in
                         ("trigger_fxy", "trigger_abs_fz", "trigger_threshold")))):
                    return False
            for event in snapshot.foot_impact_history:
                if (not event["force_known"] or
                        not event["trigger_fxy"] > event["trigger_threshold"] or
                        abs(event["trigger_abs_fz"]) < 0.0):
                    return False
            for event in snapshot.nonfoot_collision_bodies:
                if event["robot_body_id"] < 0 or not event["robot_body_name"]:
                    return False
        elif snapshot.current_collision != (1 if snapshot.robot_obstacle_contacts > 0 else 0):
            return False
        if snapshot.collision_edge and not snapshot.current_collision:
            return False
    return True


def classify_snapshot(raw: Optional[bytes], now_ns: int,
                      stale_timeout_ns: int = STALE_TIMEOUT_NS,
                      expected_capture_id: Optional[str] = None,
                      expected_fingerprint: Optional[str] = None,
                      expected_scene_binding: Optional[dict] = None) -> Tuple[CollisionStatus, Optional[CollisionSnapshot]]:
    if not raw:
        return CollisionStatus.MISSING, None
    try:
        snapshot = _decode(raw)
    except (ValueError, UnicodeDecodeError, struct.error):
        return CollisionStatus.INVALID, None
    if not _valid(snapshot, now_ns, expected_capture_id, expected_fingerprint,
                  expected_scene_binding):
        return CollisionStatus.INVALID, snapshot
    if snapshot.authoritative == 0:
        return CollisionStatus.UNKNOWN, snapshot
    if now_ns - snapshot.monotonic_ns > stale_timeout_ns:
        return CollisionStatus.STALE, snapshot
    return CollisionStatus.LIVE, snapshot


def first_snapshot_rejection(snapshot: CollisionSnapshot, now_ns: int,
                             expected_capture_id: Optional[str] = None,
                             expected_fingerprint: Optional[str] = None,
                             expected_scene_binding: Optional[dict] = None) -> Optional[dict]:
    """Describe the first existing validity rejection without changing policy.

    ``_valid`` remains the authority for acceptance. This ordered diagnostic
    mirrors its predicates only to expose the first rejected field and the
    observed/required values in startup evidence.
    """
    def fail(field, actual, expected):
        return {"field": field, "actual": actual, "expected": expected}

    supported = ((snapshot.version == VERSION_V3 and snapshot.layout_size == LEGACY_SNAPSHOT_SIZE) or
                 (snapshot.version == VERSION_V4 and snapshot.layout_size == SNAPSHOT_V4_SIZE) or
                 (snapshot.version == VERSION_V5 and snapshot.layout_size == SNAPSHOT_V5_SIZE))
    if snapshot.magic != MAGIC:
        return fail("magic", snapshot.magic, MAGIC)
    if not supported:
        return fail("version/layout_size", {"version": snapshot.version, "layout_size": snapshot.layout_size},
                    {VERSION_V3: LEGACY_SNAPSHOT_SIZE, VERSION_V4: SNAPSHOT_V4_SIZE,
                     VERSION_V5: SNAPSHOT_V5_SIZE})
    if snapshot.sequence == 0 or snapshot.sequence & 1:
        return fail("sequence", snapshot.sequence, "nonzero even sequence")
    if snapshot.monotonic_ns == 0:
        return fail("monotonic_ns", snapshot.monotonic_ns, "> 0")
    if snapshot.physics_step == 0:
        return fail("physics_step", snapshot.physics_step, "> 0")
    if now_ns < snapshot.monotonic_ns:
        return fail("monotonic_ns_vs_validation_time",
                    {"snapshot_monotonic_ns": snapshot.monotonic_ns, "validation_now_ns": now_ns},
                    "snapshot_monotonic_ns <= validation_now_ns")
    if not math.isfinite(snapshot.sim_time):
        return fail("sim_time", snapshot.sim_time, "finite")
    if CAPTURE_ID_RE.fullmatch(snapshot.capture_id) is None:
        return fail("capture_id_format", snapshot.capture_id, "p1-10-capture-<32 lowercase hex>")
    if HEX64_RE.fullmatch(snapshot.runtime_model_fingerprint) is None:
        return fail("runtime_model_fingerprint_format", snapshot.runtime_model_fingerprint,
                    "64 lowercase hex characters")
    if expected_capture_id is not None and snapshot.capture_id != expected_capture_id:
        return fail("capture_id", snapshot.capture_id, expected_capture_id)
    if expected_fingerprint is not None and snapshot.runtime_model_fingerprint != expected_fingerprint:
        return fail("runtime_model_fingerprint", snapshot.runtime_model_fingerprint, expected_fingerprint)
    for field in ("authoritative", "current_collision", "collision_edge"):
        value = getattr(snapshot, field)
        if value not in (0, 1):
            return fail(field, value, [0, 1])
    for field in ("classified_contacts", "unknown_contacts", "robot_obstacle_contacts",
                  "ground_contacts", "self_contacts", "other_contacts"):
        value = getattr(snapshot, field)
        if value < 0:
            return fail(field, value, ">= 0")
    contact_sum = (snapshot.robot_obstacle_contacts + snapshot.ground_contacts +
                   snapshot.self_contacts + snapshot.other_contacts)
    if snapshot.classified_contacts != contact_sum:
        return fail("classified_contacts", snapshot.classified_contacts, contact_sum)
    ground_split = snapshot.foot_ground_contacts + snapshot.nonfoot_ground_contacts
    if ground_split != snapshot.ground_contacts:
        return fail("foot_ground_contacts+nonfoot_ground_contacts", ground_split, snapshot.ground_contacts)
    if snapshot.version == VERSION_V5 and (snapshot.fall_condition_flags is None or
                                            snapshot.fall_condition_flags & ~0x3):
        return fail("fall_condition_flags", snapshot.fall_condition_flags, "present and only bits 0..1")
    if snapshot.collision_history_count > COLLISION_EVENT_CAPACITY:
        return fail("collision_history_count", snapshot.collision_history_count, f"<= {COLLISION_EVENT_CAPACITY}")
    expected_history_count = min(snapshot.collision_episode_count, COLLISION_EVENT_CAPACITY)
    if snapshot.collision_history_count != expected_history_count:
        return fail("collision_history_count", snapshot.collision_history_count, expected_history_count)
    for field in ("collision_history_overflow", "fall_candidate", "fall_confirmed"):
        value = getattr(snapshot, field)
        if value not in (0, 1):
            return fail(field, value, [0, 1])
    if snapshot.fall_history_count > FALL_EVENT_CAPACITY:
        return fail("fall_history_count", snapshot.fall_history_count, f"<= {FALL_EVENT_CAPACITY}")
    if snapshot.fall_history_overflow not in (0, 1):
        return fail("fall_history_overflow", snapshot.fall_history_overflow, [0, 1])
    if snapshot.event_latches_complete not in (False, True):
        return fail("event_latches_complete", snapshot.event_latches_complete, [False, True])
    if snapshot.unknown_contact_steps == 0 and (snapshot.first_unknown_contact_step != 0 or
                                                  snapshot.last_unknown_contact_step != 0):
        return fail("unknown_contact_step_bounds", [snapshot.unknown_contact_steps,
                    snapshot.first_unknown_contact_step, snapshot.last_unknown_contact_step],
                    "zero steps requires zero first/last step")
    if snapshot.invalid_posture_steps == 0 and (snapshot.first_invalid_posture_step != 0 or
                                                 snapshot.last_invalid_posture_step != 0):
        return fail("invalid_posture_step_bounds", [snapshot.invalid_posture_steps,
                    snapshot.first_invalid_posture_step, snapshot.last_invalid_posture_step],
                    "zero steps requires zero first/last step")
    for count_field, first_field, last_field in (
            ("unknown_contact_steps", "first_unknown_contact_step", "last_unknown_contact_step"),
            ("invalid_posture_steps", "first_invalid_posture_step", "last_invalid_posture_step")):
        count, first, last = (getattr(snapshot, f) for f in (count_field, first_field, last_field))
        if count > 0 and (first == 0 or last == 0 or last < first):
            return fail(f"{count_field}_step_bounds", [count, first, last], "valid nonzero ordered bounds")
    for label, count, first, last in (
            ("unknown_contact", snapshot.unknown_contact_steps, snapshot.first_unknown_contact_sim_time,
             snapshot.last_unknown_contact_sim_time),
            ("invalid_posture", snapshot.invalid_posture_steps, snapshot.first_invalid_posture_sim_time,
             snapshot.last_invalid_posture_sim_time)):
        if count > 0 and first > last:
            return fail(f"{label}_sim_time_bounds", [first, last], "first <= last")
    for index, event in enumerate(snapshot.fall_history):
        if event["start_physics_step"] == 0:
            return fail(f"fall_history[{index}].start_physics_step", 0, "> 0")
        if event["end_physics_step"] < event["start_physics_step"]:
            return fail(f"fall_history[{index}].end_physics_step", event["end_physics_step"],
                        f">= {event['start_physics_step']}")
        for field in ("start_sim_time", "end_sim_time", "confirmed_sim_time"):
            if not math.isfinite(event[field]):
                return fail(f"fall_history[{index}].{field}", event[field], "finite")
        if event["end_sim_time"] < event["start_sim_time"]:
            return fail(f"fall_history[{index}].end_sim_time", event["end_sim_time"],
                        f">= {event['start_sim_time']}")
        if event["confirmed"]:
            for field, lower, upper in (("confirmed_physics_step", event["start_physics_step"],
                                         event["end_physics_step"]),
                                        ("confirmed_sim_time", event["start_sim_time"],
                                         event["end_sim_time"])):
                if not lower <= event[field] <= upper:
                    return fail(f"fall_history[{index}].{field}", event[field], f"[{lower}, {upper}]")
        elif event["confirmed_physics_step"] != 0:
            return fail(f"fall_history[{index}].confirmed_physics_step",
                        event["confirmed_physics_step"], 0)
    if snapshot.physics_coverage_complete not in (0, 1):
        return fail("physics_coverage_complete", snapshot.physics_coverage_complete, [0, 1])
    if snapshot.fall_confirmed and (snapshot.fall_confirmed_physics_step == 0 or
                                    not math.isfinite(snapshot.fall_confirmed_sim_time)):
        return fail("fall_confirmed_fields", [snapshot.fall_confirmed_physics_step,
                    snapshot.fall_confirmed_sim_time], "step > 0 and confirmation time finite")
    if snapshot.fall_candidate and snapshot.fall_start_physics_step == 0:
        return fail("fall_start_physics_step", 0, "> 0 when candidate")
    for field in ("collision_duration_s", "collision_contact_steps"):
        value = getattr(snapshot, field)
        if value < 0:
            return fail(field, value, ">= 0")
    overflow = int(snapshot.collision_episode_count > COLLISION_EVENT_CAPACITY)
    if snapshot.collision_history_overflow != overflow:
        return fail("collision_history_overflow", snapshot.collision_history_overflow, overflow)
    for index, event in enumerate(snapshot.collision_history):
        if event["end_physics_step"] < event["start_physics_step"]:
            return fail(f"collision_history[{index}].end_physics_step", event["end_physics_step"],
                        f">= {event['start_physics_step']}")
        if event["end_sim_time"] < event["start_sim_time"]:
            return fail(f"collision_history[{index}].end_sim_time", event["end_sim_time"],
                        f">= {event['start_sim_time']}")
    finite_fields = ("collision_duration_s", "last_collision_start_sim_time",
                     "last_collision_end_sim_time", "base_height_m", "base_roll_rad",
                     "base_pitch_rad", "fall_start_sim_time", "fall_confirmed_sim_time",
                     "first_unknown_contact_sim_time", "last_unknown_contact_sim_time",
                     "first_invalid_posture_sim_time", "last_invalid_posture_sim_time")
    for field in finite_fields:
        if not math.isfinite(getattr(snapshot, field)):
            return fail(field, getattr(snapshot, field), "finite")
    for field in ("unknown_contact_steps", "invalid_posture_steps"):
        if getattr(snapshot, field) < 0:
            return fail(field, getattr(snapshot, field), ">= 0")
    if snapshot.last_contact_class not in (0, 1, 2, 3, 4, 5):
        return fail("last_contact_class", snapshot.last_contact_class, [0, 1, 2, 3, 4, 5])
    if snapshot.authoritative:
        binding = expected_scene_binding or LEGACY_BINDING
        for field, expected in (("scenario_id", binding.get("scene_id")),
                                ("scene_root_sha256", binding.get("root_sha256")),
                                ("model_closure_sha256", binding.get("closure_sha256"))):
            actual = getattr(snapshot, field)
            if actual != expected:
                return fail(field, actual, expected)
        if (expected_scene_binding is not None and
                snapshot.runtime_model_fingerprint != binding.get("model_fingerprint")):
            return fail("runtime_model_fingerprint", snapshot.runtime_model_fingerprint,
                        binding.get("model_fingerprint"))
        if snapshot.invalid_reason != 0:
            return fail("invalid_reason", snapshot.invalid_reason, 0)
        if snapshot.version == VERSION_V5:
            split = (None if snapshot.nonfoot_obstacle_contacts is None or
                     snapshot.foot_obstacle_contacts is None else
                     snapshot.nonfoot_obstacle_contacts + snapshot.foot_obstacle_contacts)
            if snapshot.foot_identity_valid != 1:
                return fail("foot_identity_valid", snapshot.foot_identity_valid, 1)
            if snapshot.nonfoot_obstacle_contacts is None:
                return fail("nonfoot_obstacle_contacts", None, "present")
            if snapshot.foot_obstacle_contacts is None:
                return fail("foot_obstacle_contacts", None, "present")
            if split != snapshot.robot_obstacle_contacts:
                return fail("foot+nonfoot_obstacle_contacts", split, snapshot.robot_obstacle_contacts)
            expected_collision = int(snapshot.nonfoot_obstacle_contacts > 0)
            if snapshot.current_collision != expected_collision:
                return fail("current_collision", snapshot.current_collision, expected_collision)
            nonnegative = ("foot_contact_history_count", "foot_impact_history_count",
                           "foot_contact_episode_count", "foot_impact_episode_count",
                           "foot_contact_physics_steps", "foot_impact_physics_steps",
                           "foot_impact_unknown_steps")
            for field in nonnegative:
                value = getattr(snapshot, field)
                if value is None or value < 0:
                    return fail(field, value, ">= 0")
            for history_field, episode_field in (("foot_contact_history_count", "foot_contact_episode_count"),
                                                 ("foot_impact_history_count", "foot_impact_episode_count")):
                count, episodes = getattr(snapshot, history_field), getattr(snapshot, episode_field)
                if count > FOOT_EVENT_CAPACITY:
                    return fail(history_field, count, f"<= {FOOT_EVENT_CAPACITY}")
                if count != min(episodes, FOOT_EVENT_CAPACITY):
                    return fail(history_field, count, min(episodes, FOOT_EVENT_CAPACITY))
            for history_overflow, episode_field in (("foot_contact_history_overflow", "foot_contact_episode_count"),
                                                    ("foot_impact_history_overflow", "foot_impact_episode_count")):
                expected = int(getattr(snapshot, episode_field) > FOOT_EVENT_CAPACITY)
                actual = getattr(snapshot, history_overflow)
                if actual != expected:
                    return fail(history_overflow, actual, expected)
            masks = ("foot_contact_mask", "foot_impact_mask", "foot_force_valid_mask", "foot_impact_unknown_mask")
            for field in masks:
                value = getattr(snapshot, field)
                if value is None or value < 0 or value > 0xF:
                    return fail(field, value, "integer in [0, 15]")
            mask_pairs = ((snapshot.foot_impact_mask & ~snapshot.foot_contact_mask,
                           "foot_impact_mask subset of foot_contact_mask"),
                          (snapshot.foot_impact_unknown_mask & ~snapshot.foot_contact_mask,
                           "foot_impact_unknown_mask subset of foot_contact_mask"),
                          (snapshot.foot_impact_mask & snapshot.foot_impact_unknown_mask,
                           "foot_impact_mask disjoint from foot_impact_unknown_mask"),
                          (snapshot.foot_impact_unknown_mask & snapshot.foot_force_valid_mask,
                           "foot_impact_unknown_mask disjoint from foot_force_valid_mask"))
            for actual, expected in mask_pairs:
                if actual:
                    return fail("foot_contact_masks", actual, expected)
            v5_time_fields = ("foot_contact_duration_s", "foot_contact_first_sim_time",
                              "foot_contact_last_sim_time", "foot_impact_duration_s",
                              "foot_impact_first_sim_time", "foot_impact_last_sim_time",
                              "foot_impact_unknown_first_sim_time", "foot_impact_unknown_last_sim_time")
            for field in v5_time_fields:
                value = getattr(snapshot, field)
                if value is None or not math.isfinite(value):
                    return fail(field, value, "finite")
            for field in ("foot_world_force", "foot_fxy", "foot_abs_fz", "foot_impact_threshold"):
                values = getattr(snapshot, field)
                if len(values) != 4:
                    return fail(field, len(values), 4)
            for index, row in enumerate(snapshot.foot_world_force):
                for axis, value in enumerate(row):
                    if not math.isfinite(value):
                        return fail(f"foot_world_force[{index}][{axis}]", value, "finite")
            for field in ("foot_fxy", "foot_abs_fz", "foot_impact_threshold"):
                for index, value in enumerate(getattr(snapshot, field)):
                    if not math.isfinite(value):
                        return fail(f"{field}[{index}]", value, "finite")
            for history_name, history in (("foot_contact_history", snapshot.foot_contact_history),
                                          ("foot_impact_history", snapshot.foot_impact_history)):
                for index, event in enumerate(history):
                    if event["start_physics_step"] == 0:
                        return fail(f"{history_name}[{index}].start_physics_step", 0, "> 0")
                    if event["end_physics_step"] < event["start_physics_step"]:
                        return fail(f"{history_name}[{index}].end_physics_step", event["end_physics_step"],
                                    f">= {event['start_physics_step']}")
                    if event["end_sim_time"] < event["start_sim_time"]:
                        return fail(f"{history_name}[{index}].end_sim_time", event["end_sim_time"],
                                    f">= {event['start_sim_time']}")
                    if event["foot_geom_name"] not in {"FL", "FR", "RL", "RR"}:
                        return fail(f"{history_name}[{index}].foot_geom_name", event["foot_geom_name"],
                                    ["FL", "FR", "RL", "RR"])
                    if not event["obstacle_geom_name"]:
                        return fail(f"{history_name}[{index}].obstacle_geom_name", "", "nonempty")
                    if event["force_known"]:
                        for field in ("trigger_fxy", "trigger_abs_fz", "trigger_threshold"):
                            if not math.isfinite(event[field]):
                                return fail(f"{history_name}[{index}].{field}", event[field], "finite")
            for index, event in enumerate(snapshot.foot_impact_history):
                if not event["force_known"]:
                    return fail(f"foot_impact_history[{index}].force_known", False, True)
                if not event["trigger_fxy"] > event["trigger_threshold"]:
                    return fail(f"foot_impact_history[{index}].trigger_fxy", event["trigger_fxy"],
                                f"> {event['trigger_threshold']}")
                if abs(event["trigger_abs_fz"]) < 0.0:
                    return fail(f"foot_impact_history[{index}].trigger_abs_fz",
                                event["trigger_abs_fz"], ">= 0")
            for index, event in enumerate(snapshot.nonfoot_collision_bodies):
                if event["robot_body_id"] < 0:
                    return fail(f"nonfoot_collision_bodies[{index}].robot_body_id",
                                event["robot_body_id"], ">= 0")
                if not event["robot_body_name"]:
                    return fail(f"nonfoot_collision_bodies[{index}].robot_body_name", "", "nonempty")
        elif snapshot.current_collision != int(snapshot.robot_obstacle_contacts > 0):
            return fail("current_collision", snapshot.current_collision,
                        int(snapshot.robot_obstacle_contacts > 0))
        if snapshot.collision_edge and not snapshot.current_collision:
            return fail("collision_edge", snapshot.collision_edge, "0 when current_collision is 0")
    return None


def classify_snapshot_diagnostic(raw: Optional[bytes], now_ns: int,
                                 stale_timeout_ns: int = STALE_TIMEOUT_NS,
                                 expected_capture_id: Optional[str] = None,
                                 expected_fingerprint: Optional[str] = None,
                                 expected_scene_binding: Optional[dict] = None
                                 ) -> tuple[CollisionStatus, Optional[CollisionSnapshot], Optional[dict]]:
    """Classify as usual and expose the first failed invariant for diagnostics."""
    status, snapshot = classify_snapshot(raw, now_ns, stale_timeout_ns,
                                         expected_capture_id, expected_fingerprint,
                                         expected_scene_binding)
    rejection = None
    if status is CollisionStatus.INVALID and snapshot is not None:
        rejection = first_snapshot_rejection(snapshot, now_ns, expected_capture_id,
                                             expected_fingerprint, expected_scene_binding)
        if rejection is None:
            rejection = {"field": "snapshot_invariant", "actual": "_valid rejected snapshot",
                         "expected": "all existing validity predicates pass"}
    elif status is CollisionStatus.INVALID:
        rejection = {"field": "snapshot_decode", "actual": "undecodable bytes",
                     "expected": "supported stable snapshot layout"}
    return status, snapshot, rejection


def read_collision_snapshot_diagnostic(path: str = SHM_PATH, max_attempts: int = 32,
                                       retry_interval_s: float = 0.00025) -> tuple[bytes, dict]:
    """Read one stable physics snapshot and preserve absent/contention errors.

    The previous tight three-read loop returned ``b""`` both when the object
    was absent and when all reads raced the writer's odd seqlock. Callers then
    mislabeled a transient read collision as MISSING. Retry briefly; if a
    mapped object remains unstable, return an invalid sentinel and its reason.
    """
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError as exc:
        absent = exc.errno == errno.ENOENT
        return (b"" if absent else b"\0"), {
            "read_status": "MISSING" if absent else "INVALID",
            "reason": "shared_memory_absent" if absent else "shared_memory_open_error",
            "errno": exc.errno, "error": str(exc),
        }
    try:
        size = os.fstat(fd).st_size
        if size not in (LEGACY_SNAPSHOT_SIZE, SNAPSHOT_V4_SIZE, SNAPSHOT_V5_SIZE):
            # Path exists but its ABI extent is invalid/truncated. Preserve
            # that distinction from an absent authority so classify_snapshot
            # returns INVALID rather than MISSING.
            return b"\0", {"read_status": "INVALID", "reason": "unsupported_shared_memory_size",
                            "size": size}
        buf = mmap.mmap(fd, size, access=mmap.ACCESS_READ)
        try:
            for attempt in range(1, max_attempts + 1):
                before = struct.unpack_from("<Q", buf, 16)[0]
                if before != 0 and not before & 1:
                    raw = bytes(buf[:size])
                    after = struct.unpack_from("<Q", buf, 16)[0]
                    if before == after and not after & 1:
                        return raw, {"read_status": "READ", "reason": None,
                                     "attempts": attempt, "sequence": after, "size": size}
                if attempt < max_attempts and retry_interval_s > 0:
                    time.sleep(retry_interval_s)
            return b"\0", {"read_status": "INVALID", "reason": "seqlock_unstable",
                            "attempts": max_attempts, "size": size}
        finally:
            buf.close()
    finally:
        os.close(fd)


def read_collision_snapshot(path: str = SHM_PATH, max_attempts: int = 32,
                            retry_interval_s: float = 0.00025) -> bytes:
    raw, _ = read_collision_snapshot_diagnostic(path, max_attempts, retry_interval_s)
    return raw
