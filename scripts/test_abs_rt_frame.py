#!/usr/bin/env python3
"""Offline frame-contract + HUD tests for P1-09C.

SYNTHETIC-TEST-ONLY. Every frame here is built in Python with ``struct.pack``;
nothing is read from a live shared-memory region, and no ROS2, MuJoCo, benchmark,
pilot, formal episode, or real-robot process is launched.

The default fixture source is ``SYNTHETIC_TEST`` so a synthetic frame is never
accidentally mistaken for authoritative runtime data. A small number of tests
explicitly opt into ``source=AUTHORITATIVE_RUNTIME`` to verify the binary-format
parse and the classification branches (LIVE / STALE / INVALID). Those fixtures
are also synthetic and are format/branch verification only — they are NOT runtime
evidence and must never be reported as real simulation results.
"""

from __future__ import annotations

import unittest

from abs_rt_frame import (
    COLLISION_UNAVAILABLE,
    DEFAULT_STALE_TIMEOUT_NS,
    FRAME_MAGIC,
    FRAME_SIZE,
    FRAME_VERSION,
    JOINT_COUNT,
    POLICY_AGILE,
    POLICY_FAULTED,
    RAY_COUNT,
    RAY_SHM_RUNTIME,
    SOURCE_AUTHORITATIVE_RUNTIME,
    SOURCE_LEGACY_ONLY,
    SOURCE_SYNTHETIC_TEST,
    SOURCE_UNSET,
    FrameStatus,
    HudState,
    RuntimeFrame,
    _FRAME_STRUCT,
    classify_frame,
)

_NOW_NS = 10_000_000_000


def _pack_frame(
    *,
    magic=FRAME_MAGIC,
    version=FRAME_VERSION,
    sequence=2,
    monotonic_ns=9_999_000_000,
    session_id=42,
    rl_step=7,
    ray_age_ns=1_000,
    source=SOURCE_SYNTHETIC_TEST,  # default fixture source is synthetic, never authoritative
    controller_active=1,
    rl_entered=1,
    rl_active=1,
    safety_faulted=0,
    policy_state=POLICY_AGILE,
    ray_origin=RAY_SHM_RUNTIME,
    ray_valid=1,
    collision_origin=COLLISION_UNAVAILABLE,
    torque_saturated_computed=0,
    reserved_pad=0,
    ra_value=0.0,
    lin_vel=(0.0, 0.0, 0.0),
    command=(1.0, 0.0, 0.0),
    world_pose=(0.0, 0.0, 0.0),
    ray2d=None,
    action_raw=None,
    action_clipped=None,
    joint_target_rad=None,
    torque_nm=None,
    torque_saturated=None,
):
    """Build a synthetic frame fixture (default source = SYNTHETIC_TEST)."""
    ray2d = [2.5] * RAY_COUNT if ray2d is None else list(ray2d)
    action_raw = [0.0] * JOINT_COUNT if action_raw is None else list(action_raw)
    action_clipped = [0.0] * JOINT_COUNT if action_clipped is None else list(action_clipped)
    joint_target_rad = [0.0] * JOINT_COUNT if joint_target_rad is None else list(joint_target_rad)
    torque_nm = [0.0] * JOINT_COUNT if torque_nm is None else list(torque_nm)
    torque_saturated = [0.0] * JOINT_COUNT if torque_saturated is None else list(torque_saturated)

    qs = [
        magic, version, sequence, monotonic_ns,
        session_id, rl_step, ray_age_ns,
    ]
    uints = [
        source, controller_active, rl_entered, rl_active, safety_faulted,
        policy_state, ray_origin, ray_valid, collision_origin,
        torque_saturated_computed, reserved_pad,
    ]
    floats = (
        [ra_value]
        + list(lin_vel)
        + list(command)
        + list(world_pose)
        + ray2d
        + action_raw
        + action_clipped
        + joint_target_rad
        + torque_nm
        + torque_saturated
    )
    assert len(qs) == 7 and len(uints) == 11 and len(floats) == 81
    return _FRAME_STRUCT.pack(*(qs + uints + floats))


def _authoritative_fixture(**kwargs):
    """A synthetic fixture that opts into the AUTHORITATIVE_RUNTIME parse branch.

    Format/branch verification only — this is NOT runtime evidence.
    """
    return _pack_frame(source=SOURCE_AUTHORITATIVE_RUNTIME, **kwargs)


class FrameContractTests(unittest.TestCase):
    def test_frame_size_matches_c_contract(self):
        self.assertEqual(FRAME_SIZE, 424)
        self.assertEqual(_FRAME_STRUCT.size, 424)

    def test_default_fixture_is_synthetic_not_authoritative(self):
        # The default fixture source is SYNTHETIC_TEST, so an otherwise valid
        # synthetic frame can never be classified LIVE.
        status, _ = classify_frame(_pack_frame(), _NOW_NS)
        self.assertEqual(status, FrameStatus.SYNTHETIC)
        self.assertNotEqual(status, FrameStatus.LIVE)

    def test_authoritative_parse_live_frame_is_synthetic_fixture(self):
        # Format/branch verification only (synthetic fixture), not runtime evidence.
        data = _authoritative_fixture()
        status, frame = classify_frame(data, _NOW_NS)
        self.assertEqual(status, FrameStatus.LIVE)
        self.assertIsInstance(frame, RuntimeFrame)
        self.assertEqual(frame.session_id, 42)
        self.assertEqual(frame.rl_step, 7)
        self.assertEqual(frame.source, SOURCE_AUTHORITATIVE_RUNTIME)

    def test_missing_empty_data(self):
        self.assertEqual(classify_frame(None, _NOW_NS), (FrameStatus.MISSING, None))
        self.assertEqual(classify_frame(b"", _NOW_NS), (FrameStatus.MISSING, None))

    def test_invalid_wrong_size(self):
        for size in (10, 1, 423, 425):
            with self.subTest(size=size):
                status, frame = classify_frame(b"\x00" * size, _NOW_NS)
                self.assertEqual(status, FrameStatus.INVALID)
                self.assertIsNone(frame)

    def test_invalid_magic_version(self):
        self.assertEqual(classify_frame(_pack_frame(magic=0), _NOW_NS)[0], FrameStatus.INVALID)
        self.assertEqual(classify_frame(_pack_frame(version=99), _NOW_NS)[0], FrameStatus.INVALID)

    def test_invalid_sequence(self):
        for sequence in (0, 1, 3, 5):
            with self.subTest(sequence=sequence):
                status, _ = classify_frame(_pack_frame(sequence=sequence), _NOW_NS)
                self.assertEqual(status, FrameStatus.INVALID)

    def test_unknown_origin(self):
        self.assertEqual(classify_frame(_pack_frame(source=SOURCE_UNSET), _NOW_NS)[0], FrameStatus.UNKNOWN_ORIGIN)
        self.assertEqual(classify_frame(_pack_frame(source=99), _NOW_NS)[0], FrameStatus.UNKNOWN_ORIGIN)

    def test_legacy_origin_rejected_never_live(self):
        status, _ = classify_frame(_pack_frame(source=SOURCE_LEGACY_ONLY), _NOW_NS)
        self.assertEqual(status, FrameStatus.LEGACY)
        self.assertNotEqual(status, FrameStatus.LIVE)

    def test_synthetic_origin_never_live(self):
        status, _ = classify_frame(_pack_frame(source=SOURCE_SYNTHETIC_TEST), _NOW_NS)
        self.assertEqual(status, FrameStatus.SYNTHETIC)
        self.assertNotEqual(status, FrameStatus.LIVE)

    def test_inconsistent_flags_invalid(self):
        # Authoritative parse branch (synthetic fixture), format/branch only.
        cases = [
            dict(rl_active=1, safety_faulted=1),   # active and faulted together
            dict(rl_active=1, rl_entered=0),       # active without being entered
            dict(controller_active=0),             # not written by an active controller
        ]
        for kwargs in cases:
            with self.subTest(kwargs=kwargs):
                status, _ = classify_frame(_authoritative_fixture(**kwargs), _NOW_NS)
                self.assertEqual(status, FrameStatus.INVALID)

    def test_nonfinite_payload_invalid(self):
        cases = [
            dict(ra_value=float("nan")),
            dict(ra_value=float("inf")),
            dict(lin_vel=(float("nan"), 0.0, 0.0)),
            dict(lin_vel=(0.0, float("-inf"), 0.0)),
            dict(command=(float("nan"), 0.0, 0.0)),
            dict(world_pose=(0.0, float("inf"), 0.0)),
            dict(ray2d=[2.5] * 10 + [float("nan")]),
            dict(action_raw=[float("nan")] + [0.0] * 11),
            dict(action_clipped=[0.0] * 11 + [float("inf")]),
            dict(joint_target_rad=[float("-inf")] + [0.0] * 11),
            dict(torque_nm=[0.0] * 11 + [float("nan")]),
        ]
        for kwargs in cases:
            with self.subTest(kwargs=kwargs):
                status, _ = classify_frame(_authoritative_fixture(**kwargs), _NOW_NS)
                self.assertEqual(status, FrameStatus.INVALID)

    def test_stale_timestamp(self):
        stale_ns = _NOW_NS - DEFAULT_STALE_TIMEOUT_NS - 1
        status, frame = classify_frame(
            _authoritative_fixture(monotonic_ns=stale_ns), _NOW_NS
        )
        self.assertEqual(status, FrameStatus.STALE)
        self.assertIsNotNone(frame)  # stale keeps the payload for diagnosis

    def test_unarmed_or_backwards_clock_invalid(self):
        self.assertEqual(
            classify_frame(_authoritative_fixture(monotonic_ns=0), _NOW_NS)[0],
            FrameStatus.INVALID,
        )
        self.assertEqual(
            classify_frame(_authoritative_fixture(monotonic_ns=_NOW_NS + 1), _NOW_NS)[0],
            FrameStatus.INVALID,
        )

    def test_strict_enum_and_bool_domain_invalid(self):
        # Every boolean/enum field must hold a defined legal value; anything
        # else is corrupt and must be INVALID, never LIVE.
        cases = [
            dict(policy_state=99),              # not AGILE/RECOVERY/FAULTED
            dict(policy_state=3),
            dict(ray_origin=99),                # not UNAVAILABLE/SHM_RUNTIME
            dict(controller_active=2),          # bool domain {0,1}
            dict(controller_active=255),
            dict(rl_entered=2),                 # bool domain {0,1}
            dict(rl_active=2),                  # bool domain {0,1}
            dict(safety_faulted=2),             # bool domain {0,1}
            dict(ray_valid=2),                  # bool domain {0,1}
            dict(torque_saturated_computed=2),  # bool domain {0,1}
            dict(collision_origin=99),          # only 0 is currently defined
        ]
        for kwargs in cases:
            with self.subTest(kwargs=kwargs):
                status, frame = classify_frame(_authoritative_fixture(**kwargs), _NOW_NS)
                self.assertEqual(status, FrameStatus.INVALID)
                self.assertIsNone(frame)

    def test_cpp_invalidate_signature_is_immediately_invalid(self):
        # Equivalent offline verification of StateRL::invalidateRtFrame(): the
        # C++ exit path writes header magic=0/version=0 with a stable even
        # sequence (seqlock-safe), then unmaps/closes. The reader must classify
        # this INVALID at once — not STALE, never LIVE, no stale-timeout wait.
        data = _pack_frame(magic=0, version=0, sequence=2)
        status, frame = classify_frame(data, _NOW_NS)
        self.assertEqual(status, FrameStatus.INVALID)
        self.assertIsNone(frame)
        hud = HudState()
        hud.update(data, _NOW_NS)
        shown = hud.display()
        self.assertFalse(shown["live"])
        self.assertEqual(shown["status"], FrameStatus.INVALID.value)

    def test_faulted_frame_is_live_authoritative_data(self):
        # A fault is real, fresh, authoritative runtime data; the HUD surfaces the
        # fault (safety_faulted=True) and suppresses the command chain. Synthetic
        # fixture — format/branch verification only.
        data = _authoritative_fixture(
            policy_state=POLICY_FAULTED,
            rl_active=0,
            rl_entered=1,
            safety_faulted=1,
            command=(0.0, 0.0, 0.0),
            ra_value=0.05,
        )
        status, frame = classify_frame(data, _NOW_NS)
        self.assertEqual(status, FrameStatus.LIVE)
        self.assertEqual(frame.policy_state, POLICY_FAULTED)


class HudStateTests(unittest.TestCase):
    def test_live_display_exposes_real_values(self):
        hud = HudState()
        hud.update(
            _authoritative_fixture(ra_value=0.03, command=(1.5, -0.2, 0.4)), _NOW_NS
        )
        shown = hud.display()
        self.assertTrue(shown["live"])
        self.assertAlmostEqual(shown["ra_value"], 0.03, places=5)
        self.assertAlmostEqual(shown["command"][0], 1.5, places=5)
        self.assertAlmostEqual(shown["command"][1], -0.2, places=5)
        self.assertAlmostEqual(shown["command"][2], 0.4, places=5)
        self.assertFalse(shown["safety_faulted"])

    def test_synthetic_fixture_never_displayed_live(self):
        # A fully well-formed (finite, fresh, coherent) synthetic frame is never
        # shown as live simulation data.
        hud = HudState()
        hud.update(_pack_frame(source=SOURCE_SYNTHETIC_TEST), _NOW_NS)
        shown = hud.display()
        self.assertEqual(shown["status"], FrameStatus.SYNTHETIC.value)
        self.assertFalse(shown["live"])
        self.assertNotIn("ra_value", shown)

    def test_non_live_status_suppresses_all_data(self):
        for data, expected in (
            (b"", FrameStatus.MISSING),
            (_pack_frame(source=SOURCE_SYNTHETIC_TEST), FrameStatus.SYNTHETIC),
            (_pack_frame(source=SOURCE_LEGACY_ONLY), FrameStatus.LEGACY),
            (_pack_frame(magic=0), FrameStatus.INVALID),
            (
                _authoritative_fixture(monotonic_ns=_NOW_NS - DEFAULT_STALE_TIMEOUT_NS - 1),
                FrameStatus.STALE,
            ),
        ):
            with self.subTest(expected=expected):
                hud = HudState()
                hud.update(data, _NOW_NS)
                shown = hud.display()
                self.assertEqual(shown["status"], expected.value)
                self.assertFalse(shown["live"])
                self.assertNotIn("ra_value", shown)

    def test_torque_saturated_never_shown(self):
        hud = HudState()
        hud.update(_authoritative_fixture(torque_saturated_computed=0), _NOW_NS)
        self.assertIsNone(hud.display()["torque_saturated"])

    def test_collision_never_shown_from_controller_frame(self):
        hud = HudState()
        hud.update(_authoritative_fixture(collision_origin=COLLISION_UNAVAILABLE), _NOW_NS)
        self.assertIsNone(hud.display()["collision"])

    def test_invalid_rays_suppressed(self):
        hud = HudState()
        hud.update(_authoritative_fixture(ray_valid=0), _NOW_NS)
        shown = hud.display()
        self.assertFalse(shown["ray_valid"])
        self.assertIsNone(shown["ray2d"])

    def test_faulted_step_suppresses_command_chain(self):
        hud = HudState()
        hud.update(
            _authoritative_fixture(policy_state=POLICY_FAULTED, rl_active=0, safety_faulted=1),
            _NOW_NS,
        )
        shown = hud.display()
        self.assertTrue(shown["live"])
        self.assertTrue(shown["safety_faulted"])
        self.assertEqual(shown["policy_state"], "FAULTED")
        self.assertIsNone(shown["action_raw"])
        self.assertIsNone(shown["action_clipped"])
        self.assertIsNone(shown["joint_target_rad"])
        self.assertIsNone(shown["torque_nm"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
