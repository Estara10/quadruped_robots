# P1-09U — BridgeLifecycle Single State Machine

## Reviewer defects closed

P1-09T was **REJECTED** for the separately updated `bridge_active_` and
`terminal_` flags, lack of a real concurrent lifecycle test, trailing whitespace,
and an earlier build claim made before the new target was regenerated.

`BridgeLifecycle` now has one mutex-protected state:

`INITIAL → RESERVED → ACTIVE → STOPPING → TERMINAL`

`main()` reserves before starting either worker, so reload is disallowed during
the startup window. `requestStop()` moves the reserved/active lifecycle to
`STOPPING`; `markBridgeTerminal()` is idempotent. `reloadAllowed()` reads only
this state and returns true only in `INITIAL` or `TERMINAL`. `ACTIVE`,
`STOPPING`, and `TERMINAL` reject activation/restart as applicable; a terminal
lifecycle cannot be reserved or activated again.

The external SDK/DDS lifetime remains **UNKNOWN**. No runtime behavior was
tested or claimed.

## Mechanical tests

`p1_09u_bridge_lifecycle_state_test` uses real `std::thread` instances. In 64
iterations, a stopper and an activation attempt are released concurrently from a
reserved state. Each iteration must finish terminal, then reject later activate
and reserve calls. It also checks the complete sequential state/reload table and
idempotent stop/terminal calls.

## Commands and actual results

| Command | Exit code | Output summary |
|---|---:|---|
| `rtk python3 scripts/test_p1_09r_static_contract.py` | 0 | `P1-09R static contract: PASS` |
| `rtk python3 scripts/test_p1_09s_static_contract.py` | 0 | `P1-09S static contract: PASS` |
| `rtk python3 scripts/test_p1_09t_static_contract.py` | 0 | `P1-09T static contract: PASS` |
| `rtk python3 scripts/test_p1_09u_static_contract.py` | 0 | `P1-09U static contract: PASS` |
| first `rtk cmake --build ... p1_09u_bridge_lifecycle_state_test ...` | 2 | stale generated build rules: no rule for the new U target; no PASS claimed |
| `rtk cmake -S unitree_mujoco/simulate -B unitree_mujoco/simulate/build2` | 0 | regenerated build rules; Boost development-policy warning only |
| second `rtk cmake --build ... unitree_mujoco p1_09o_joinable_thread_test p1_09p_reload_barrier_test p1_09u_bridge_lifecycle_state_test -j2` | 0 | all requested targets built |
| `rtk .../p1_09o_joinable_thread_test && rtk .../p1_09p_reload_barrier_test && rtk .../p1_09u_bridge_lifecycle_state_test` | 0 | all three binaries passed silently |
| `rtk git diff --check` | 0 | no diff whitespace errors; RTK printed only its hook advisory |

No MuJoCo, ROS2, benchmark, formal run, pilot, or real robot was run.

## Review request

P1-09U is **IMPLEMENTED / AWAITING INDEPENDENT REVIEW**. It does not accept
P1-09 or Phase 1. A later, separately authorized runtime clean-exit validation
is still required.
