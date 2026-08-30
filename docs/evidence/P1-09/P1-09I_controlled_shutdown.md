# P1-09I — MuJoCo Controlled Exit Behaviour Reproduction

Date: 2026-08-28  
Status: **FAIL — MuJoCo did not exit after SIGINT and required SIGTERM.**

## Scope and boundary

Exactly one P1-09I run was executed after the P1-09G evidence correction. It
used only the existing flat MuJoCo scene, loopback simulation and existing ROS2
launch path:

- scene: `scene_flat.xml`;
- MuJoCo binary: `unitree_mujoco/simulate/build2/unitree_mujoco`;
- interface: `lo`, domain 1;
- launch: `ros2 launch rl_quadruped_controller mujoco.launch.py simulation_test:=0`;
- no real robot, benchmark, formal episode, `FormalRunWriter`, pilot or
  performance measurement.

No retry was performed.

## Process provenance

The orchestrator persisted the following at process start:

| Process | PID | PPID | PGID | SID | Command |
|---|---:|---:|---:|---:|---|
| MuJoCo | 116590 | 116542 | 116590 | 116590 | `/home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco -s scene_flat.xml` |
| ROS2 launch wrapper | 116705 | 116542 | 116705 | 116705 | `bash -lc source /opt/ros/humble/setup.bash && source /home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash && ros2 launch rl_quadruped_controller mujoco.launch.py simulation_test:=0` |

Display evidence: `DISPLAY=:0`, `WAYLAND_DISPLAY=UNSET`,
`XDG_RUNTIME_DIR=/run/user/1000`.

## Lifecycle timeline

| UTC timestamp | Evidence |
|---|---|
| `11:08:35.065Z` | P1-09I started; scene and loopback boundary recorded |
| `11:08:45.914Z` | Controller became active |
| `11:08:56.736Z` | RL entry confirmed |
| `11:08:57.927Z` | `[REC-ENTER]` confirmed after `command=4` |
| `11:08:59.252Z` | `[REC-EXIT]` confirmed after `command=3` |
| `11:08:59.270Z` | ROS2 launch process group sent `SIGINT` |
| `11:08:59.691Z` | ROS2 launch exited within the 5-second bound |
| `11:08:59.697Z` | ROS2 launch final wait result: `rc=0` |
| `11:08:59.729Z` | MuJoCo process group sent `SIGINT` |
| `11:09:04.972Z` | MuJoCo `SIGINT` wait timed out (`5 s` bound) |
| `11:09:04.978Z` | MuJoCo process group sent `SIGTERM`, reason `SIGINT_TIMEOUT` |
| `11:09:05.094Z` | MuJoCo exited after SIGTERM |
| `11:09:05.099Z` | MuJoCo final wait result: `rc=143` |

## Exit and safety result

| Component | Result |
|---|---|
| StateRLRec enter | **PASS** — actual `[REC-ENTER]` observed |
| StateRLRec exit | **PASS** — actual `[REC-EXIT]` observed |
| controller-manager/plugin shutdown | **PASS for observed log path** — `rl_quadruped_controller` was deactivated and `ros2_control_node` reported clean process finish |
| ROS2 launch | **PASS** — final wait `rc=0` |
| MuJoCo self-exit after SIGINT | **FAIL** — no exit within 5 seconds |
| MuJoCo forced termination | **FAIL** for clean-shutdown acceptance — SIGTERM was required; final `rc=143` |
| abort signatures | **PASS / not observed** — no `terminate called`, `SIGABRT`, `abort`, `std::terminate` or core-dump text in the captured logs |

The `rc=143` result is not a clean shutdown and is not converted into a pass.
The controller-manager clean-finish log is not a separately captured direct
wait status; the exact controller-manager child exit code remains `UNKNOWN`.

## Raw evidence

- [`P1-09I_orchestrator_raw.log`](P1-09I_orchestrator_raw.log)
- [`P1-09I_ros2_launch_raw.log`](P1-09I_ros2_launch_raw.log)
- [`P1-09I_mujoco_raw.log`](P1-09I_mujoco_raw.log)

The prior contradiction is corrected in
[`P1-09G_controlled_shutdown.md`](P1-09G_controlled_shutdown.md): its attempt-2
raw log already records `SIGINT`, `STOP_TIMEOUT`, `SIGTERM`, `rc=143`, and ROS
launch `rc=0`.

## Required offline regression

| Command | Result |
|---|---|
| `rtk python3 scripts/test_abs_rt_frame.py` | **PASS** — 24 tests |
| `rtk python3 scripts/test_abs_live_hud.py` | **PASS** — 17 tests |
| `rtk python3 scripts/test_formal_runtime_adapter.py` | **PASS** — 16 tests |
| `rtk python3 scripts/test_formal_experiment_contract.py` | **PASS** — 22 tests |
| `rtk git diff --check` | **PASS** |

This evidence is neither a formal run nor P1-09/Phase 1 Acceptance. P1-09
remains `EXECUTING`; the clean MuJoCo shutdown gap remains open.
