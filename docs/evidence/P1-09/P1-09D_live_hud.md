# P1-09D — Minimal Read-Only Live Terminal HUD

Date: 2026-08-28
Scope: offline, additive implementation increment of P1-09. No runtime process was run.
Status: P1-09 remains **EXECUTING**; this increment is not an Acceptance claim.

## What was implemented

`scripts/abs_live_hud.py` — a minimal, read-only terminal HUD over the P1-09C
single real-time data link. It reads **only** the fixed path
`/dev/shm/mujoco_rt_frame` via `abs_rt_frame.read_shm_frame`, classifies every
snapshot with `abs_rt_frame.HudState`, and renders a terminal block via
`render(state, now_ns)`.

No change to controller, policy, thresholds, gains, switching, solver, dynamics,
or config. No P1-02 formal-recorder wiring. The HUD computes **no** success /
arrival / no-collision conclusion — it only echoes the controller's frame fields.

## Rendering rules (fail-closed, same boundary as P1-09C)

| Status | Render |
|---|---|
| `LIVE` | full data block (authoritative runtime frame) |
| `MISSING` | status block only: "no frame present at /dev/shm/mujoco_rt_frame" |
| `INVALID` | status block only: frame is corrupt (size/magic/version/sequence/flags/non-finite) |
| `UNKNOWN_ORIGIN` | status block only: source unset or unrecognized |
| `LEGACY` | status block only: legacy data is never shown as live |
| `SYNTHETIC` | status block only: synthetic test frame is never shown as live |
| `STALE` | status block only: authoritative but older than the freshness timeout |

- A non-LIVE status **never** renders any data field (no residual values from a
  previous LIVE frame). The terminal loop clears the screen before every redraw,
  and `HudState.display()` returns no fields on non-LIVE, so nothing can leak.
- Unavailable fields render `N/A`, never `0`: `collision` (bridge-side only) and
  `torque_saturated` (not computed anywhere) both render `N/A (unavailable)`.
- A faulted LIVE frame renders `FAULTED` + `safety_faulted = 1` and suppresses
  the command chain (`action_raw[12]`, `action_clipped[12]`,
  `joint_target[12]`, `torque_nm[12]` all `N/A`); a fault is real, fresh,
  authoritative data and must not be hidden.
- `ray_valid = 0` suppresses `ray2d[11]` (renders `N/A`).

## LIVE display fields

session_id, rl_step, frame age (ms), policy_state (AGILE / RECOVERY / FAULTED),
safety_faulted, ra_value, lin_vel (actual), command (target), world_pose,
ray_valid, ray_age_ns, ray2d[11], action_raw[12], action_clipped[12],
joint_target[12], torque_nm[12], collision (N/A), torque_saturated (N/A).

## Offline tests (`scripts/test_abs_live_hud.py`, 15 tests)

SYNTHETIC-TEST-ONLY. No ROS2, MuJoCo, simulation, benchmark, pilot, formal
episode, or real-robot process is launched.

- Default synthetic fixture renders only the `SYNTHETIC` status block — never
  live numbers.
- `MISSING` / `INVALID` / `UNKNOWN_ORIGIN` / `LEGACY` / `SYNTHETIC` / `STALE`
  each render only their status block; none of the 18 LIVE field labels appear.
- No-residue checks: LIVE → then MISSING, and LIVE → then STALE, both drop every
  previous data value.
- The LIVE display branch is exercised with explicitly marked format fixtures
  (`_authoritative_fixture` from `test_abs_rt_frame`, which opts into
  `source = AUTHORITATIVE_RUNTIME`). These fixtures are format/branch
  verification **only** — they are NOT runtime evidence and must never be
  reported as real simulation results.
- Faulted LIVE, ray-invalid, N/A-for-collision/torque-saturated (never `0`),
  no-conclusion, and frame-age render checks.

## Validation results (all offline)

| Command | Result |
|---|---|
| `rtk python3 scripts/test_abs_live_hud.py` | **15/15 PASS** (new) |
| `rtk python3 scripts/test_abs_rt_frame.py` | **22/22 PASS** (P1-09C green) |
| `rtk python3 scripts/test_formal_runtime_adapter.py` | **16/16 PASS** (P1-09B green) |
| `rtk python3 scripts/test_formal_experiment_contract.py` | **22/22 PASS** (P1-02 green) |
| `rtk python3 -m py_compile abs_live_hud.py test_abs_live_hud.py abs_rt_frame.py test_abs_rt_frame.py` | **PASS** |
| `rtk python3 scripts/abs_live_hud.py --iters 1` (no frame present) | renders `[ MISSING ]`, no data fields, exit 0 |
| `rtk git diff --check` | **clean** |

## Honest delivery statement

This task did **not** start any simulation, so the HUD has full read capability
but has **not yet been observed displaying real simulation data**. The `run()`
loop was never run against a live `/mujoco_rt_frame`; only the offline synthetic
tests and the no-frame smoke check ran. Displaying real live data requires an
authorized runtime episode whose `StateRL` controller publishes authoritative
frames to `/mujoco_rt_frame`.

## Relationship to P1-09C / P1-09B / P1-02

P1-09C created the frame contract, the `StateRL::writeRtFrame` publisher, and
the fail-closed classifier/HUD state model; P1-09D adds the human terminal layer
over that same classifier with the same boundaries. Nothing is bound to the P1-02
`FormalRunWriter`; that binding remains a future, separately-authorized increment.
