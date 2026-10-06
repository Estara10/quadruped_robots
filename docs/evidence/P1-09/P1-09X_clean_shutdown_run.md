# P1-09X — MuJoCo-only Controlled Clean-Shutdown Run

## Result

**BLOCKED**. This was the single authorized run. The simulator failed during
GLFW initialization before reaching a running state, so no valid SIGINT
shutdown path was exercised. No retry was performed.

## Boundary

- Simulator only; no ROS2, controller-manager, StateRL/StateRLRec, HUD,
  benchmark, FormalRunWriter, pilot, or real robot.
- Existing flat scene command: `unitree_mujoco -s scene_flat.xml`.
- No source/configuration changes were made for this run.

## Command and process evidence

| Item | Observed |
|---|---|
| Command | `/home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco -s scene_flat.xml` |
| Working directory | `unitree_mujoco/simulate/build2` |
| Start time | `2026-08-29T14:36:04,117115665+08:00` |
| PID / PPID / PGID | `6 / 3 / 6` (orchestrator namespace) |
| MuJoCo version | `3.3.3` |
| Startup result | `ERROR: could not initialize GLFW` |
| `/proc/<pid>/status` | unavailable because the process had already exited |
| SIGINT | attempted once after startup failure; target process group `6` no longer existed; `kill` rc=1 |
| Wait | 12 ms; `SELF_EXITED=1` |
| MuJoCo exit code | `1` |
| TERM/KILL escalation | none |
| orphan after wait | `0` |

The PID/PGID, command, timestamps, wait result, and signal attempt are
preserved verbatim in [`P1-09X_orchestrator_raw.log`](P1-09X_orchestrator_raw.log).
The application output is preserved in
[`P1-09X_mujoco_raw.log`](P1-09X_mujoco_raw.log).

## Required lifecycle evidence

| Evidence | Result |
|---|---|
| Bridge started | UNKNOWN / not applicable: bridge was not started in this MuJoCo-only run |
| Physics thread started | UNKNOWN / not applicable: simulator failed before runtime initialization |
| Bridge stop/join | UNKNOWN |
| Physics join | UNKNOWN |
| m/d release | UNKNOWN |
| normal `main` return | UNKNOWN |
| terminate / SIGABRT / abort / core dump | No such text observed in captured output; full runtime crash evidence is not established |

Because GLFW initialization failed, this evidence cannot prove clean shutdown,
and it is not a benchmark, formal experiment, or P1-09 acceptance result.

## Post-run validation

Command: `rtk git diff --check`  
Exit code: `0`  
Output summary: no whitespace errors; RTK emitted only its informational
“No hook installed” warning. No simulator/controller code was changed for
P1-09X.

## Remaining UNKNOWN

- Whether the repaired MuJoCo process exits rc=0 after SIGINT in a functioning
  GLFW environment.
- Runtime bridge stop/join, physics join, m/d release, and normal-main-return
  evidence.
- DDS lifetime and other existing P1-09 UNKNOWN items.
