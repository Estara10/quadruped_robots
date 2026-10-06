# P1-09C/D — Real-Time Frame Strict Fail-Closed Closure

Date: 2026-08-28
Scope: offline closure of the three defects rejected by the independent Reviewer
for P1-09C/D. No runtime process was run.
Status: P1-09 remains **EXECUTING**; this closure is not an Acceptance claim and
awaits the next independent Reviewer re-review.

## Defect 1 — Python classifier strict enum/boolean validation

### Old behavior

`classify_frame()` validated only `controller_active == 0` and the `rl_active`
flag consistency; any nonzero/odd boolean value (e.g. `controller_active = 2`,
`ray_valid = 2`) or an out-of-domain enum (`policy_state = 99`, `ray_origin = 99`,
`collision_origin = 99`) sailed through the consistency checks and, if fresh and
finite, could be classified **LIVE**.

### New behavior

`scripts/abs_rt_frame.py` now rejects, as `INVALID`, any authoritative frame
whose fields fall outside the defined domain, **before** the consistency checks:

- `controller_active`, `rl_entered`, `rl_active`, `safety_faulted`, `ray_valid`,
  `torque_saturated_computed` → must be exactly `0` or `1`;
- `policy_state` → must be `AGILE (0)`, `RECOVERY (1)`, or `FAULTED (2)`;
- `ray_origin` → must be `UNAVAILABLE (0)` or `SHM_RUNTIME (1)`;
- `collision_origin` → must be the currently defined `UNAVAILABLE (0)`;
- `source` — unchanged, still strictly classified (UNSET/unrecognized →
  `UNKNOWN_ORIGIN`, LEGACY → `LEGACY`, SYNTHETIC → `SYNTHETIC`).

`INVALID` returns no payload, so the HUD renders only the status block and can
never show live fields for these frames.

## Defect 2 — C++ incomplete vectors are never zero-padded

### Old behavior

`StateRL::writeRtFrame()` clamped the ray count with
`std::min(kRayCount, obs_.ray2d.size(1))` and each command-chain tensor with
`std::min(kJointCount, src.size(1))`, then zero-filled the remainder. A shorter
ray buffer or an incomplete 12-dim chain was therefore padded with `0.0f` and
published as a LIVE authoritative frame. No finiteness check existed.

### New behavior

`StateRL::writeRtFrame()` now validates **before** building the frame:

- the 4 command-chain tensors must each be defined, 2-D, `[1, 12]` exactly;
- `obs_.ray2d` must be defined and exactly `[1, 11]`;
- `obs_.lin_vel` must be defined and exactly `[1, 3]`;
- every value copied into the frame (rays, 4×12 command chain, lin_vel, command,
  world_pose, `ra_value`) must be finite (`std::isfinite`).

On any dimension mismatch, undefined tensor, or non-finite value, the shared
frame is explicitly invalidated (`invalidateRtFrame()`) and the method returns
without publishing — a short ray buffer or truncated chain can no longer be
padded to 11/12 and shown as LIVE. `torque_saturated[12]` remains an explicit
`0.0f` with `torque_saturated_computed = 0` (documented unavailable, never a
claimed computed value).

Because the invalidation is transient (the next successful step re-publishes a
valid frame with `magic = kMagic`), a one-step bad vector degrades to `INVALID`
for that step and recovers — no control output is touched.

## Defect 3 — controller exit invalidates the frame immediately

### Old behavior

On destructor the frame shm was `munmap`'d/`close`'d, leaving the last published
bytes in `/dev/shm/mujoco_rt_frame`. The HUD kept showing the last frame as LIVE
until the 500 ms stale timeout elapsed — a stale-residual window after the
controller had already stopped publishing.

### New behavior

`StateRL::invalidateRtFrame()` (seqlock-safe: sequence odd → write
`magic=0`/`version=0` → stable even sequence) is called

- in the **destructor before `munmap`/`close`**, and
- in **`exit()`** (the explicit RL exit path).

`magic = 0 ≠ kMagic` makes the reader classify the frame **INVALID immediately**
— no stale-timeout wait, never LIVE. The normal faulted frame is untouched: a
fault while the controller is still running publishes a real, fresh,
authoritative `LIVE` frame with `safety_faulted = 1` (a fault is real evidence
and is not treated as an exit invalidation); `exit()` invalidation only happens
once the controller actually stops publishing.

## New fail-closed tests

`scripts/test_abs_rt_frame.py` (22 → 24):

- `test_strict_enum_and_bool_domain_invalid` — `policy_state=99/3`,
  `ray_origin=99`, `controller_active=2/255`, `rl_entered=2`, `rl_active=2`,
  `safety_faulted=2`, `ray_valid=2`, `torque_saturated_computed=2`,
  `collision_origin=99` each classify `INVALID` with no payload.
- `test_cpp_invalidate_signature_is_immediately_invalid` — equivalent offline
  verification of `invalidateRtFrame()`: `magic=0`/`version=0`/even sequence
  classifies `INVALID` at once (not `STALE`, never `LIVE`), and `HudState`
  shows `live=False`.

`scripts/test_abs_live_hud.py` (15 → 17):

- `test_hud_invalid_never_live_for_bad_enums` — for each illegal enum/boolean
  value the HUD renders only `INVALID` and none of the 18 LIVE field labels leak.
- `test_hud_invalid_immediately_after_exit_invalidation` — after a LIVE frame,
  feeding the exit-invalidation signature makes the HUD show `INVALID` at once
  with no residual live fields (no stale-timeout wait).

## Validation results (all offline)

| Command | Result |
|---|---|
| `rtk python3 scripts/test_abs_rt_frame.py` | **24/24 PASS** |
| `rtk python3 scripts/test_abs_live_hud.py` | **17/17 PASS** |
| `rtk python3 scripts/test_formal_runtime_adapter.py` | **16/16 PASS** (P1-09B green) |
| `rtk python3 scripts/test_formal_experiment_contract.py` | **22/22 PASS** (P1-02 green) |
| `rtk python3 -m py_compile abs_rt_frame.py test_abs_rt_frame.py abs_live_hud.py test_abs_live_hud.py` | **PASS** |
| `colcon build --packages-select rl_quadruped_controller --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DTorch_DIR=.../Torch` | **BUILD_EXIT=0**, no warnings/errors; `StateRL.cpp.o` + `librl_quadruped_controller.so` rebuilt; `nm` shows `StateRL::invalidateRtFrame` and the new `writeRtFrame` lambdas |
| `rtk git diff --check` | **clean** |

## Boundary confirmation

No ROS2 node, MuJoCo, simulation, benchmark, pilot, formal episode, or real-robot
process was run. The build is compile-only (the shared library was compiled but
never executed). No control semantics, policy, thresholds, gains, switching,
solver, dynamics, or config behavior changed; the changes touch only the
observation-frame classifier and the frame publisher/invalidation. No P1-01 /
P1-02 / P1-03 Acceptance was modified.

## Remaining UNKNOWN (unchanged)

- `collision` and `torque_saturated` have no authoritative producer (rendered
  `N/A`, never `0`).
- Measured cadence and the active ray source mode remain `UNKNOWN` (require an
  authorized runtime capture).
- The frame is not yet bound to the P1-02 `FormalRunWriter` (future,
  separately-authorized increment).
- This closure does not demonstrate real live data: no simulation was started.

P1-09 remains **EXECUTING** and awaits the next independent Reviewer re-review.
