# P1-09T — Lock-I/O and Terminal Bridge Lifecycle

## Scope and disposition

P1-09S is recorded as **REJECTED** for lock-internal diagnostics, restart
ambiguity, and insufficient mechanical coverage. This implementation is for
independent review only; it does not accept P1-09 or Phase 1.

## Changes

- Both `PhysicsLoop` reload paths record rejection locally under `sim.mtx`,
  then clear the UI message and log after unlocking. Candidate `dnew`/`mnew`
  cleanup is retained.
- `JoinableThread` is one-shot: `start()` is rejected after stop/join and while
  joining; it uses condition-variable wakeup and has no detach path.
- `BridgeLifecycle` records terminal state after bridge inactivity and rejects a
  later `markBridgeActive()`.

## Validation

- `python3 scripts/test_p1_09r_static_contract.py`: PASS
- `python3 scripts/test_p1_09s_static_contract.py`: PASS
- `python3 scripts/test_p1_09t_static_contract.py`: PASS
- simulator target plus `p1_09p_reload_barrier_test` and
  `p1_09o_joinable_thread_test`: PASS
- both lifecycle test binaries: PASS
- `git diff --check`: PASS (exit code 0)

No MuJoCo, ROS2, benchmark, pilot, formal run, or real robot was run. Runtime
clean-shutdown and SDK/DDS thread lifetime remain UNKNOWN.

## Lifecycle invariant

`initial model ready → one bridge start → stop request → bridge join → terminal
state → m/d release`. Later bridge restart/rebind is rejected.
