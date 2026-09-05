# P1-09W — MuJoCo Lifecycle Consolidated Self-Check

## Self-check findings

The pre-change checklist found three items requiring correction:

| Reviewer problem | Consolidated correction | Verification |
|---|---|---|
| `beginStop()` could transition `INITIAL` to `STOPPING` | `beginStop()` accepts only `RESERVED` or `ACTIVE`; startup failure has no fabricated bridge terminal path | V and W static checks; V lifecycle test |
| Worker could submit `TERMINAL` before main joined/reset the bridge | only `completeTerminal()` transitions `STOPPING → TERMINAL`; main joins bridge before completion, then joins physics before m/d release | V/W static checks; P/U/V lifecycle binaries |
| Constructor scene metadata was printed through live model pointers | `printSceneInformation()` collects `SceneInfoSnapshot` under `sim.mtx` and prints only local copies outside the lock | W static check; simulator build |

The consolidated audit also retained and checked the existing R/S/T invariants:
both active reload paths remain fail-closed with candidate cleanup; reload lock
regions contain no logging/I/O; RobotBridge has one m/d guard; `ray_exit`
remains before payload publication; and no SDK detach path was introduced.

## Final lifecycle contract

`INITIAL → RESERVED → ACTIVE → STOPPING → TERMINAL` is the only lifecycle.
`reloadAllowed()` is true only in `INITIAL` or `TERMINAL`. `INITIAL` cannot be
stopped through the bridge API. A bridge is reserved before worker creation;
main begins stop, joins the bridge (including interface destruction), completes
terminal state, joins physics, and only then releases m/d. Terminal state is
idempotently observable but cannot be re-entered or restarted.

## Actual commands and exit codes

| Command | Exit code | Result |
|---|---:|---|
| `rtk python3 scripts/test_p1_09r_static_contract.py` | 0 | PASS |
| `rtk python3 scripts/test_p1_09s_static_contract.py` | 0 | PASS |
| `rtk python3 scripts/test_p1_09t_static_contract.py` | 0 | PASS |
| `rtk python3 scripts/test_p1_09u_static_contract.py` | 0 | PASS |
| `rtk python3 scripts/test_p1_09v_static_contract.py` | 0 | PASS |
| `rtk python3 scripts/test_p1_09w_static_contract.py` | 0 | PASS |
| `rtk cmake -S unitree_mujoco/simulate -B unitree_mujoco/simulate/build2` | 0 | configured |
| simulator target build | 0 | `unitree_mujoco` built |
| O/P/U/V lifecycle target builds | 0 | all built |
| O lifecycle binary | 0 | PASS |
| P lifecycle binary | 0 | PASS |
| U concurrent lifecycle binary | 0 | PASS |
| V terminal invariant binary | 0 | PASS |
| `rtk git diff --check` | 0 | PASS |

The W static check had one earlier test-script assertion failure (rc=1) while
it still referenced the pre-V terminal API; after correction the recorded W
command exited 0. No MuJoCo, ROS2, benchmark, formal run, pilot, or real robot
was run. This is offline evidence only.

## Status

P1-09W is **IMPLEMENTED / AWAITING INDEPENDENT REVIEW**. P1-09 and Phase 1
remain unaccepted. DDS/ChannelFactory lifetime and runtime clean shutdown remain
UNKNOWN.
