# P1-09G — StateRLRec Controlled Simulation Shutdown Verification

Date: 2026-08-28  
Status: **BLOCKED / REJECTED EVIDENCE — the post-unlock attempt entered and
exited `StateRLRec`, but MuJoCo timed out after SIGINT, required SIGTERM, and
ended with `rc=143`.**

## Authorized boundary

Both captured attempts were limited to the existing simulation-only boundary:

- MuJoCo binary: `unitree_mujoco/simulate/build2/unitree_mujoco`;
- scene: `unitree_robots/go2/scene_flat.xml` (`scene_flat.xml` passed through
  the existing simulator lookup);
- simulator configuration: `interface: "lo"`, `domain_id: 1`;
- intended controller launch: `mujoco.launch.py simulation_test:=0`;
- intended Recovery transition: existing `/control_input`, `command=4` from
  `StateRL`, then `command=3` to leave `StateRLRec`.

No real-robot launch, real network interface, benchmark, formal writer,
formal/VALID artifact, or performance measurement was used.

## Attempt 1 — blocked before ROS2

The orchestration started at `2026-08-28T10:20:36Z` and confirmed no matching
MuJoCo/controller-manager process before launch. The simulator was then started
as the first required process, but exited before the three-second startup check:

```text
MuJoCo version 3.3.3
ERROR: could not initialize GLFW
```

Consequences:

| Required evidence | Result |
|---|---|
| MuJoCo startup | **FAIL** — GLFW initialization failed. |
| ROS2 controller launch | **NOT STARTED** — precondition failed. |
| StateRLRec enter (`[REC-ENTER]`) | **NOT OBSERVED**. |
| StateRLRec exit (`[REC-EXIT]`) | **NOT OBSERVED**. |
| controller-manager/plugin unload | **NOT OBSERVED**. |
| `terminate` / SIGABRT / abort / core dump in captured logs | **NOT OBSERVED**; this is not a teardown pass because teardown was never reached. |
| MuJoCo exact child exit code | **NOT RECORDED** — the first-attempt orchestrator captured the failed liveness check but did not persist the child wait status before it exited. Its own early-stop path is code 20; that is not substituted for the simulator exit code. The later post-unlock attempt and P1-09I provide the definitive shutdown result: timeout after SIGINT, SIGTERM, `rc=143`. |
| ROS2 launch exit code | **NOT_APPLICABLE** — process was not started. |

Raw evidence:

- [`P1-09G_orchestrator_raw.log`](P1-09G_orchestrator_raw.log)
- [`P1-09G_mujoco_raw.log`](P1-09G_mujoco_raw.log)

No matching simulation/controller process remained after the failed startup
check.

## Attempt 2 — explicitly authorized after the workstation was unlocked

After the user explicitly authorized a new attempt following the lock-screen
condition, one separate runner and three new raw logs were used. It remained the
same simulation-only `scene_flat.xml` / loopback boundary; it did not run a
formal writer, evaluator, benchmark, or real-robot path.

| Evidence | Result |
|---|---|
| graphical session | `DISPLAY=:0`; MuJoCo passed startup |
| controller readiness | `rl_quadruped_controller` was active |
| RL entry | confirmed at `2026-08-28T10:52:49Z` |
| `StateRLRec` entry | **PASS** — `[REC-ENTER] 49-dim recovery policy active` at `10:52:50Z` after `command=4` |
| `StateRLRec` exit | **PASS** — `[REC-EXIT] RL steps executed: 106` at `10:52:52.869Z` after `command=3` |
| controller/plugin shutdown | **PASS** — controller manager deactivated and shut down `rl_quadruped_controller`; `ros2_control_node` finished cleanly |
| ROS2 launch exit | **PASS** — orchestrator recorded `ros2_launch rc=0` |
| `terminate` / SIGABRT / abort / core dump | **NOT OBSERVED** in attempt-2 orchestration, ROS2, or MuJoCo logs |
| MuJoCo shutdown | **FAIL** — after `SIGINT` at `2026-08-28T10:52:54Z`, the runner waited until `10:53:10Z`, recorded `STOP_TIMEOUT`, sent `SIGTERM`, and recorded `mujoco rc=143` |

The second runner must not be interpreted as a performance, arrival, collision,
or formal-episode result. It proves only the scope-specific lifecycle fact:
the actual Recovery worker entered, exited, and survived normal controller
plugin teardown without an observed `terminate`/SIGABRT. It does **not** meet
the complete P1-09G evidence requirement: MuJoCo did not exit on SIGINT and was
force-terminated with SIGTERM. This is not a clean shutdown.

Attempt-2 raw evidence:

- [`P1-09G_attempt2_orchestrator_raw.log`](P1-09G_attempt2_orchestrator_raw.log)
- [`P1-09G_attempt2_ros2_launch_raw.log`](P1-09G_attempt2_ros2_launch_raw.log)
- [`P1-09G_attempt2_mujoco_raw.log`](P1-09G_attempt2_mujoco_raw.log)

## Post-attempt offline regression

| Command | Result |
|---|---|
| `rtk python3 scripts/test_abs_rt_frame.py` | **PASS** — 24 tests. |
| `rtk python3 scripts/test_abs_live_hud.py` | **PASS** — 17 tests. |
| `rtk python3 scripts/test_formal_runtime_adapter.py` | **PASS** — 16 tests. |
| `rtk python3 scripts/test_formal_experiment_contract.py` | **PASS** — 22 tests. |
| `rtk git diff --check` | **PASS**. |

## Remaining evidence required

The P1-09F live-unload question is **PASS for the observed controller/plugin
shutdown path**. P1-09G is **BLOCKED / REJECTED EVIDENCE** because MuJoCo did
not exit after SIGINT and required SIGTERM (`rc=143`). Any future
Director-authorized, separate bounded capture must record all of:

1. `[REC-ENTER]` after `command=4`;
2. `[REC-EXIT]` after `command=3`;
3. normal controller-manager/plugin shutdown;
4. explicit key-process exit codes; and
5. absence of `terminate`, SIGABRT, abort, and core-dump evidence.

Neither attempt is P1-09 Acceptance or Phase 1 evidence.
