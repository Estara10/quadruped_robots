#!/usr/bin/env python3
"""Offline tests for the P1-09D terminal HUD renderer.

SYNTHETIC-TEST-ONLY. Every frame is built in Python with ``struct.pack``; no ROS2,
MuJoCo, simulation, benchmark, pilot, formal episode, or real-robot process is
launched.

The LIVE display branch is exercised with explicitly marked format fixtures
(``_authoritative_fixture`` from ``test_abs_rt_frame``) that opt into
``source=AUTHORITATIVE_RUNTIME`` purely to verify the render logic. Those
fixtures are format/branch verification only — they are NOT runtime evidence and
must never be reported as real simulation results.
"""

from __future__ import annotations

import unittest

from abs_rt_frame import (
    SOURCE_LEGACY_ONLY,
    SOURCE_SYNTHETIC_TEST,
    SOURCE_UNSET,
    HudState,
)
from abs_live_hud import _LIVE_FIELD_LABELS, render

from test_abs_rt_frame import _authoritative_fixture, _pack_frame

_NOW_NS = 10_000_000_000


def _render(data, now_ns=_NOW_NS):
    hud = HudState()
    hud.update(data, now_ns)
    return render(hud, now_ns)


def _field(out, label):
    """Return the rendered value of one labeled field line, e.g. 'N/A' or '1'."""
    line = next(ln for ln in out.splitlines() if ln.strip().startswith(label))
    return line.split("=", 1)[1].strip()


class RenderNonLiveTests(unittest.TestCase):
    def _assert_status_only(self, status, data):
        out = _render(data)
        self.assertIn(status, out)
        for label in _LIVE_FIELD_LABELS:
            self.assertNotIn(label, out, f"live field '{label}' leaked into {status} render")

    def test_missing_shows_status_only(self):
        self._assert_status_only("MISSING", b"")

    def test_invalid_shows_status_only(self):
        self._assert_status_only("INVALID", _pack_frame(magic=0))

    def test_unknown_origin_shows_status_only(self):
        self._assert_status_only("UNKNOWN_ORIGIN", _pack_frame(source=SOURCE_UNSET))

    def test_legacy_shows_status_only(self):
        self._assert_status_only("LEGACY", _pack_frame(source=SOURCE_LEGACY_ONLY))

    def test_synthetic_shows_status_only(self):
        self._assert_status_only("SYNTHETIC", _pack_frame(source=SOURCE_SYNTHETIC_TEST))

    def test_stale_shows_status_only(self):
        # A complete, finite, authoritative frame older than the freshness timeout
        # is STALE; the HUD must not show any of its values.
        stale_ns = _NOW_NS - 1_000_000_000
        self._assert_status_only("STALE", _authoritative_fixture(monotonic_ns=stale_ns))

    def test_default_synthetic_frame_never_shows_live_values(self):
        # The default fixture source is SYNTHETIC_TEST; even a fully well-formed
        # synthetic frame renders only the status block, never live numbers.
        out = _render(_pack_frame())
        self.assertIn("SYNTHETIC", out)
        self.assertNotIn("AGILE", out)
        for label in _LIVE_FIELD_LABELS:
            self.assertNotIn(label, out)

    def test_no_residue_after_live_then_missing(self):
        # A LIVE frame is rendered, then the link goes missing. The next render
        # must not retain any value from the previous LIVE frame.
        hud = HudState()
        hud.update(_authoritative_fixture(session_id=12345, rl_step=9, ra_value=0.03), _NOW_NS)
        self.assertIn("LIVE", render(hud, _NOW_NS))
        self.assertIn("12345", render(hud, _NOW_NS))
        hud.update(b"", _NOW_NS)
        missing_out = render(hud, _NOW_NS)
        self.assertIn("MISSING", missing_out)
        for label in _LIVE_FIELD_LABELS:
            self.assertNotIn(label, missing_out, f"residual live field '{label}' after MISSING")

    def test_no_residue_after_live_then_stale(self):
        hud = HudState()
        hud.update(_authoritative_fixture(session_id=12345, rl_step=9, ra_value=0.03), _NOW_NS)
        self.assertIn("LIVE", render(hud, _NOW_NS))
        stale_ns = _NOW_NS - 1_000_000_000
        hud.update(
            _authoritative_fixture(session_id=12345, rl_step=9, ra_value=0.03, monotonic_ns=stale_ns),
            _NOW_NS,
        )
        stale_out = render(hud, _NOW_NS)
        self.assertIn("STALE", stale_out)
        for label in _LIVE_FIELD_LABELS:
            self.assertNotIn(label, stale_out, f"residual live field '{label}' after STALE")

    def test_hud_invalid_never_live_for_bad_enums(self):
        # Defect 1: a frame with a non-legal bool/enum value must render only the
        # INVALID status block; none of the LIVE data fields may appear.
        cases = [
            ("policy_state=99", _authoritative_fixture(policy_state=99)),
            ("ray_origin=99", _authoritative_fixture(ray_origin=99)),
            ("controller_active=2", _authoritative_fixture(controller_active=2)),
            ("rl_entered=2", _authoritative_fixture(rl_entered=2)),
            ("rl_active=2", _authoritative_fixture(rl_active=2)),
            ("safety_faulted=2", _authoritative_fixture(safety_faulted=2)),
            ("ray_valid=2", _authoritative_fixture(ray_valid=2)),
            ("torque_saturated_computed=2", _authoritative_fixture(torque_saturated_computed=2)),
            ("collision_origin=99", _authoritative_fixture(collision_origin=99)),
        ]
        for name, data in cases:
            with self.subTest(name=name):
                out = _render(data)
                self.assertIn("INVALID", out)
                for label in _LIVE_FIELD_LABELS:
                    self.assertNotIn(label, out, f"{name}: live field '{label}' leaked")

    def test_hud_invalid_immediately_after_exit_invalidation(self):
        # Defect 3: after the C++ exit path calls invalidateRtFrame() (magic=0,
        # version=0, stable even sequence), the HUD must show INVALID at once —
        # not STALE (no wait for the freshness timeout), never a residual LIVE.
        hud = HudState()
        hud.update(_authoritative_fixture(session_id=555, ra_value=0.03), _NOW_NS)
        self.assertTrue(hud.display()["live"])
        hud.update(_pack_frame(magic=0, version=0, sequence=2), _NOW_NS)
        shown = hud.display()
        self.assertFalse(shown["live"])
        self.assertEqual(shown["status"], "INVALID")
        out = render(hud, _NOW_NS)
        for label in _LIVE_FIELD_LABELS:
            self.assertNotIn(label, out, f"live field '{label}' leaked after exit invalidation")


class RenderLiveTests(unittest.TestCase):
    def test_live_render_shows_authoritative_values(self):
        # Format/branch verification only (synthetic fixture), not runtime evidence.
        out = _render(
            _authoritative_fixture(
                session_id=777,
                rl_step=42,
                ra_value=0.03,
                command=(1.5, -0.2, 0.4),
                lin_vel=(0.2, 0.0, 0.0),
                ray2d=[2.5] * 11,
            )
        )
        self.assertIn("[ LIVE ]", out)
        self.assertIn("777", out)
        self.assertIn("42", out)
        self.assertIn("AGILE", out)
        self.assertIn("0.0300", out)
        self.assertIn("1.5000", out)
        self.assertIn("-0.2000", out)
        self.assertIn("0.4000", out)
        self.assertIn("2.5000", out)

    def test_live_render_marks_collision_and_torque_sat_unavailable(self):
        out = _render(_authoritative_fixture())
        lines = out.splitlines()
        collision_line = next(ln for ln in lines if "collision" in ln)
        sat_line = next(ln for ln in lines if "torque_saturated" in ln)
        self.assertIn("N/A", collision_line)
        self.assertIn("N/A", sat_line)
        self.assertNotIn("= 0", collision_line)
        self.assertNotIn("= 0", sat_line)

    def test_live_render_computes_no_conclusion(self):
        out = _render(_authoritative_fixture())
        lowered = out.lower()
        self.assertNotIn("success", lowered)
        self.assertNotIn("arrival", lowered)
        self.assertNotIn("collision-free", lowered)
        self.assertNotIn("no-collision", lowered)

    def test_faulted_live_render_marks_fault_and_suppresses_command_chain(self):
        data = _authoritative_fixture(
            policy_state=2,  # FAULTED
            rl_active=0,
            safety_faulted=1,
        )
        out = _render(data)
        self.assertIn("FAULTED", out)
        self.assertEqual(_field(out, "safety_faulted"), "1")
        self.assertEqual(_field(out, "action_raw[12]"), "N/A")
        self.assertEqual(_field(out, "action_clipped[12]"), "N/A")
        self.assertEqual(_field(out, "joint_target[12]"), "N/A")
        self.assertEqual(_field(out, "torque_nm[12]"), "N/A")

    def test_invalid_ray_suppresses_ray_values(self):
        out = _render(_authoritative_fixture(ray_valid=0))
        self.assertEqual(_field(out, "ray_valid"), "0")
        self.assertEqual(_field(out, "ray2d[11]"), "N/A")

    def test_frame_age_renders(self):
        # fixture monotonic_ns = 9_999_000_000, now = 10_000_000_000 → 1 ms age
        out = _render(_authoritative_fixture())
        self.assertEqual(_field(out, "frame age"), "1.0 ms")


if __name__ == "__main__":
    unittest.main(verbosity=2)
