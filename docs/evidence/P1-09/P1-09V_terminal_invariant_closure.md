# P1-09V — Terminal Invariant and Constructor m/d Closure

## Scope

P1-09U findings are addressed without runtime execution. The bridge lifecycle
now uses only the mutex-protected enum
`INITIAL → RESERVED → ACTIVE → STOPPING → TERMINAL`.

`completeTerminal()` succeeds only from `STOPPING` and otherwise preserves the
state. `main()` reserves before worker creation, requests stop, joins the bridge
worker after its interface destruction, completes terminal state, then joins
physics before releasing final m/d. The bridge worker no longer commits terminal
state itself.

`mj_name2id(m, ...)` and `m->nu` in `UnitreeSdk2BridgeThread` are now read under
`sim.mtx` into local values. Logging and SDK construction remain outside the
guard; no logging, SHM, DDS, joystick, sleep, or blocking operation is inside it.

## Commands and actual exit codes

| Command | Exit code | Result |
|---|---:|---|
| `rtk python3 scripts/test_p1_09r_static_contract.py` | 0 | PASS |
| `rtk python3 scripts/test_p1_09s_static_contract.py` | 0 | PASS |
| `rtk python3 scripts/test_p1_09t_static_contract.py` | 0 | PASS |
| `rtk python3 scripts/test_p1_09u_static_contract.py` | 0 | PASS |
| first `rtk python3 scripts/test_p1_09v_static_contract.py` | 1 | FAIL: assertion still expected the pre-V terminal API; corrected and rerun |
| `rtk python3 scripts/test_p1_09v_static_contract.py` | 0 | PASS |
| initial combined simulator/O/P/U/V build | 2 | FAIL: stale V target rule; retained |
| `rtk cmake -S unitree_mujoco/simulate -B unitree_mujoco/simulate/build2` | 0 | PASS |
| subsequent combined build | 2 | FAIL: multi-target V rule check; retained |
| `rtk cmake --build ... --target unitree_mujoco -j2` | 0 | PASS |
| `rtk cmake --build ... --target p1_09o_joinable_thread_test -j2` | 0 | PASS |
| `rtk cmake --build ... --target p1_09p_reload_barrier_test -j2` | 0 | PASS |
| `rtk cmake --build ... --target p1_09u_bridge_lifecycle_state_test -j2` | 0 | PASS |
| `rtk cmake --build ... --target p1_09v_terminal_invariant_test -j2` | 0 | PASS |
| `rtk .../p1_09o_joinable_thread_test` | 0 | PASS |
| `rtk .../p1_09p_reload_barrier_test` | 0 | PASS |
| `rtk .../p1_09u_bridge_lifecycle_state_test` | 0 | PASS |
| `rtk .../p1_09v_terminal_invariant_test` | 0 | PASS |
| `rtk git diff --check` | 0 | PASS |

After the final `beginStop()` API naming and main-order correction, the
following reruns also exited 0: `test_p1_09r_static_contract.py`,
`test_p1_09s_static_contract.py`, `test_p1_09t_static_contract.py`,
`test_p1_09u_static_contract.py`, `test_p1_09v_static_contract.py`; simulator,
O, P, U, and V targets built separately; and O/P/U/V lifecycle binaries ran
successfully. The final `rtk git diff --check` exited 0.

No MuJoCo, ROS2, benchmark, formal run, pilot, or real robot was run. DDS
lifetime and runtime clean shutdown remain UNKNOWN. P1-09 and Phase 1 are not
accepted.

## Reviewer request

P1-09V is IMPLEMENTED / AWAITING INDEPENDENT REVIEW. Review stop/activate
linearization, the required STOPPING predecessor, constructor m/d locking, and
main join/terminal/release ordering.
