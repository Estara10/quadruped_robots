# P1-09S — m/d lock boundary, initial publication, and fault-path closure

## Scope and status

Lifecycle-only correction. No MuJoCo, ROS2, benchmark, formal run, pilot, or
real-robot execution was performed. P1-09 and Phase 1 remain unaccepted.

## Corrections

- `sim.Load()` remains outside the PhysicsLoop `sim.mtx` critical section.
- After `sim.Load()` returns, candidate installation, old m/d deletion,
  pointer replacement, `mj_forward`, and control-noise state updates are inside
  `sim.mtx`.
- Initial model creation and first `mj_forward` complete before
  `BridgeLifecycle::markInitialReady()`. The bridge waits for this readiness
  signal rather than observing partially published global `d`.
- The bridge constructor locks only `_check_sensor()` metadata access. Logging,
  scene printing, joystick/DDS construction, shared-memory setup, and fault
  arming remain outside that lock.
- `ray_exit` again calls `_Exit(EXIT_SUCCESS)` at the armed fault branch before
  any ray payload publish. Periodic ray diagnostics remain deferred until after
  the m/d guard.

## Mechanical evidence

- `scripts/test_p1_09s_static_contract.py`: PASS.
- `scripts/test_p1_09r_static_contract.py`: PASS.
- `p1_09o_joinable_thread_test`: PASS.
- `p1_09p_reload_barrier_test`: PASS.
- `unitree_mujoco` simulator target build: PASS.
- `rtk git diff --check`: PASS.

The static checks cover reload lock placement, initial-ready ordering,
constructor lock contents, bridge m/d guard, publication after unlock,
immediate pre-publish `ray_exit`, and failed candidate cleanup.

## Remaining limitations

- Runtime proof of lock contention and reload behavior remains UNKNOWN.
- SDK DDS/ChannelFactory internal lifetime remains UNKNOWN.
- No automatic bridge rebind/restart was added.
