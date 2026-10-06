# P1-09R — RobotBridge / PhysicsLoop m/d mutex and reload cleanup

## Scope

The implementation uses the existing `Simulate::mtx` as the sole direct m/d
mutex. No snapshot path, policy, controller, threshold, solver, dynamics,
configuration, or runtime behavior was intentionally changed. No MuJoCo,
ROS2, benchmark, formal run, pilot, or real robot was run.

## Changes

- `BridgeLifecycle` carries the existing `sim.mtx` to `RobotBridge`.
- RobotBridge copies LowCmd under `lowcmd->mutex_`, releases it, then acquires
  the m/d guard. Sensor, contact, control, and ray work using m/d is inside
  that guard.
- LowState, HighState, and WirelessController publication happens after the
  m/d guard is released. No lock nesting between LowCmd and m/d is used.
- Both `droploadrequest` and `uiloadrequest` reject active-bridge reloads before
  deleting old m/d. If `mnew` succeeds but `mj_makeData(mnew)` fails, `mnew` is
  now explicitly deleted; the old m/d remain untouched.
- P1-09P's fail-closed reload reservation is unchanged.

## Mechanical validation

- `scripts/test_p1_09r_static_contract.py`: PASS.
- `p1_09o_joinable_thread_test`: PASS.
- `p1_09p_reload_barrier_test`: PASS.
- `unitree_mujoco` simulator target build: PASS.
- `rtk git diff --check`: PASS.

The static test checks the shared `sim.mtx` guard, separation of LowCmd access
from m/d access, publication after the guard, both reload barrier checks, and
both failed-candidate cleanup branches.

## Remaining UNKNOWN

- Runtime proof that bridge and PhysicsLoop are race-free remains pending.
- The SDK DDS/ChannelFactory's internal thread ownership and any hidden m/d
  access remain UNKNOWN.
- The existing global m/d publication and model-reload synchronization outside
  the guarded paths require separate review.

P1-09 and Phase 1 remain NOT ACCEPTED. Independent Reviewer review is required.
