#!/usr/bin/env python3
"""P1-09D — minimal read-only live terminal HUD over the /mujoco_rt_frame link.

This is the human-facing consumer of the single real-time data link created in
P1-09C (``common/abs_rt_frame_contract.h`` published by ``StateRL::writeRtFrame``
into ``/mujoco_rt_frame``). It reads ONLY the fixed path ``/dev/shm/mujoco_rt_frame``
via ``abs_rt_frame.read_shm_frame``, classifies every snapshot with
``abs_rt_frame.HudState``, and renders a terminal block.

Boundaries (unchanged from P1-09C):

- Only a frame classified LIVE (``source == AUTHORITATIVE_RUNTIME``, coherent,
  finite, fresh) is rendered with data values.
- Any non-LIVE status renders ONLY the status block — MISSING / INVALID / STALE /
  SYNTHETIC / LEGACY / UNKNOWN_ORIGIN — and NEVER shows previous frame values (no
  residual data) and NEVER zero-fills an unavailable field.
- ``collision`` and ``torque_saturated`` have no authoritative producer in this
  frame today; they render as ``N/A (unavailable)``, never ``0``.
- This HUD computes NO success / arrival / no-collision conclusion. It only
  echoes the controller's frame fields.
- No ROS2, MuJoCo, simulation, benchmark, pilot, or real-robot process is
  launched here; the loop only reads shared memory.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Dict, List, Optional

from abs_rt_frame import HudState, SHM_PATH, read_shm_frame


def monotonic_ns() -> int:
    """Steady-clock ns in the same domain as the C++ writer (steady_clock)."""
    return time.monotonic_ns()


# Human hint for each non-LIVE status. The canonical name comes from FrameStatus.
_STATUS_HINT = {
    "MISSING": f"no frame present at {SHM_PATH}",
    "INVALID": "frame is corrupt (size/magic/version/sequence/flags/non-finite)",
    "UNKNOWN_ORIGIN": "frame source is unset or unrecognized",
    "LEGACY": "legacy-only data — never shown as live",
    "SYNTHETIC": "synthetic test frame — never shown as live",
    "STALE": "authoritative but stale (older than the freshness timeout)",
}

# Every field label the LIVE block can render. Tests use this to prove a non-LIVE
# block never leaks any live data field.
_LIVE_FIELD_LABELS = (
    "session_id",
    "rl_step",
    "frame age",
    "policy_state",
    "safety_faulted",
    "ra_value",
    "lin_vel (actual)",
    "command (target)",
    "world_pose",
    "ray_valid",
    "ray_age_ns",
    "ray2d[11]",
    "action_raw[12]",
    "action_clipped[12]",
    "joint_target[12]",
    "torque_nm[12]",
    "collision",
    "torque_saturated",
)

_CLEAR_SCREEN = "\x1b[2J\x1b[H"


def _fmt_vec(values: Optional[List[float]]) -> str:
    """Format a vector of floats, or N/A when the field has no data."""
    if values is None:
        return "N/A"
    return " ".join(f"{v: .4f}" for v in values)


def _line(label: str, value: str) -> str:
    return f"  {label:<20} = {value}"


def _render_status_block(status: str) -> str:
    hint = _STATUS_HINT.get(status, "unknown frame state")
    return (
        f"[ {status} ] no live simulation data\n"
        f"  reason: {hint}\n"
        f"  previous frame values are NOT shown (no residual data)."
    )


def _render_live(shown: Dict[str, object], now_ns: int) -> str:
    age_ns = max(0, now_ns - int(shown["monotonic_ns"]))
    ray2d = shown["ray2d"] if shown["ray_valid"] else None
    lines = [
        "[ LIVE ] authoritative runtime frame",
        _line("session_id", str(shown["session_id"])),
        _line("rl_step", str(shown["rl_step"])),
        _line("frame age", f"{age_ns / 1e6:.1f} ms"),
        _line("policy_state", str(shown["policy_state"])),
        _line("safety_faulted", str(int(shown["safety_faulted"]))),
        _line("ra_value", f"{shown['ra_value']:.4f}"),
        _line("lin_vel (actual)", _fmt_vec(shown["lin_vel"])),
        _line("command (target)", _fmt_vec(shown["command"])),
        _line("world_pose", _fmt_vec(shown["world_pose"])),
        _line("ray_valid", str(int(shown["ray_valid"]))),
        _line("ray_age_ns", str(shown["ray_age_ns"])),
        _line("ray2d[11]", _fmt_vec(ray2d)),
        _line("action_raw[12]", _fmt_vec(shown["action_raw"])),
        _line("action_clipped[12]", _fmt_vec(shown["action_clipped"])),
        _line("joint_target[12]", _fmt_vec(shown["joint_target_rad"])),
        _line("torque_nm[12]", _fmt_vec(shown["torque_nm"])),
        _line("collision", "N/A (unavailable — bridge-side only)"),
        _line("torque_saturated", "N/A (unavailable — not computed)"),
    ]
    return "\n".join(lines) + "\n"


def render(state: HudState, now_ns: int) -> str:
    """Render one HUD snapshot as a terminal block.

    Non-LIVE renders the status block only (no data, no residue). LIVE renders
    the authoritative frame fields; unavailable fields render ``N/A``.
    """
    shown = state.display()
    if not shown.get("live"):
        return _render_status_block(str(shown["status"]))
    return _render_live(shown, now_ns)


def run(refresh_interval_s: float = 0.5, max_iterations: Optional[int] = None) -> None:
    """Read /mujoco_rt_frame periodically and redraw the terminal HUD.

    The screen is cleared before every redraw, so a non-LIVE status never leaves
    stale LIVE data visible (no residual values). ``max_iterations`` is provided
    for scripting; ``None`` means run until interrupted (Ctrl-C).
    """
    hud = HudState()
    iteration = 0
    try:
        while max_iterations is None or iteration < max_iterations:
            now_ns = monotonic_ns()
            data = read_shm_frame()
            hud.update(data, now_ns)
            sys.stdout.write(_CLEAR_SCREEN)
            sys.stdout.write(
                "P1-09D read-only terminal HUD — source: only /mujoco_rt_frame "
                "(no benchmark, no success/arrival/collision conclusions)\n"
            )
            sys.stdout.write(render(hud, now_ns))
            sys.stdout.flush()
            iteration += 1
            if max_iterations is None or iteration < max_iterations:
                time.sleep(refresh_interval_s)
    except KeyboardInterrupt:
        sys.stdout.write("\nHUD stopped.\n")
        sys.stdout.flush()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="refresh interval in seconds (default 0.5)",
    )
    parser.add_argument(
        "--iters",
        type=int,
        default=None,
        help="refresh a fixed number of times then exit (default: run until Ctrl-C)",
    )
    args = parser.parse_args()
    run(refresh_interval_s=args.interval, max_iterations=args.iters)


if __name__ == "__main__":
    main()
