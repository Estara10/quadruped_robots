# P1-09AE — Single Real Runtime Record Capture (attempted 2026-08-30)

Status: **FAIL / BLOCKED at controller-ready stage**. Exactly one
simulation-only run was attempted and it did not reach the data chain.
Per the task's no-retry rule no re-run was performed; the failure is preserved
verbatim. This is not a benchmark, formal experiment, pilot, or acceptance
claim. P1-09 and Phase 1 remain **EXECUTING / NOT ACCEPTED**.

## What was attempted

One simulation-only run in the graphical session (`DISPLAY=:0`,
`XAUTHORITY=/run/user/1000/gdm/Xauthority`, `xdpyinfo` rc=0):

1. `unitree_mujoco/simulate/build2/unitree_mujoco -s scene_flat.xml`
   (`interface: "lo"`, `scene_flat.xml`)
2. `ros2 launch rl_quadruped_controller mujoco.launch.py simulation_test:=0`
3. existing HUD + existing two-phase runtime recorder
4. control sequence `/control_input` `2 → 2 → 3`
5. STOP sampling → normal process exit → real process facts → FINALIZE →
   `post_run_summary.py <record> --json`

## Timeline (orchestrator raw log)

| time (+08:00) | event |
|---|---|
| 11:28:05 | P1-09AE_BEGIN; preflight OK (DISPLAY set, no shm, no leftover processes) |
| 11:28:05 | `PROCESS_START mujoco pid=25049 pgid=25049` |
| 11:28:08 | MuJoCo alive; `SigIgn=0x0`, `SigCgt=0x100004002` |
| 11:28:08 | `PROCESS_START ros2_launch pid=25115 pgid=25115` |
| 11:28:08–11:28:56 | `ros2 control list_controllers` returned empty every poll |
| 11:28:56 | `CONTROLLER_READY=0` → `CONTROLLER_READY_TIMEOUT` → abort, no retry |

No HUD, recorder, control sequence, RL, STOP, process exit, facts, FINALIZE or
post-run summary occurred. `record.jsonl`, `process_facts.json`,
`post_run_summary.json/.txt`, `hud_raw.txt` were **not** produced.

## Root cause (from raw logs)

The ros2 `controller_manager` resource manager could not load the hardware
plugin (`ros2_launch_raw.log`):

```text
[controller_manager] [ERROR] ... Failed to load library
 .../libhardware_unitree_mujoco.so. ... Could not load library
 dlopen error: libddsc.so.0: cannot open shared object file: No such file or directory
```

`libddsc.so.0` exists only in `/home/lidio/Libraries/unitree_sdk2/lib/` and is
not in the system `ldconfig` cache. `scripts/launch_abs_sim.sh` line 24 exports
`LD_LIBRARY_PATH="${UNITREE_SDK2_LIB}:${LIBTORCH_LIB}:${LD_LIBRARY_PATH}"`
**before** starting both MuJoCo and the ros2 launch, so both inherit it. The
run orchestrator's ros2-launch child inherited the base environment *without*
that path, so the plugin load failed, the controller never became active, and
`list_controllers` stayed empty.

This is a **launch-environment defect of the run orchestrator**, not a change
or defect in project code, configuration, schema, controller, DDS, thread, or
reload. MuJoCo itself started and initialized normally (`/mujoco_ray2d`,
`/mujoco_qpos`, `/mujoco_collision` shared memory initialized; ray source
`geometric`).

## Acceptance

**NOT MET.** The required data chain (record with real, complete, continuous
LIVE frames; HUD and record from the same session; unique terminal at the end;
summary computed from the record; exit facts from actual waits) was never
exercised because the controller never became active.

## Remaining UNKNOWN (unchanged)

All P1-09AE runtime outcomes remain UNKNOWN: record content and LIVE
continuity, HUD/record session identity, terminal uniqueness/position, summary
outputs (duration/velocity/Recovery/RA/safety faults/normal-exit status), and
process-exit facts from waits. This launch failure does not change the offline
Reviewer PASS of the two-phase recorder (54/54 tests) or any other P1-09 state.

## Evidence

| File | SHA-256 | bytes |
|---|---|---|
| `P1-09AE_record_capture_fail_20260830_orchestrator_raw.log` | `7a2c5aa4ec442362c98a8322c4b33e1d9739101fdb6639dcd5fe50b3d6ef3027` | 832 |
| `P1-09AE_record_capture_fail_20260830_ros2_launch_raw.log` | `5e73fddf4e209837d3e12cdb03a3e13dbb36e80b1d06f7f11d5efa34af9aea49` | 7568 |
| `P1-09AE_record_capture_fail_20260830_mujoco_raw.log` | `9c66aede407b56b0c0d0e0e23c67968bd32613bad23f6ca0c18cb09fd4f9c208` | 3851 |
| `P1-09AE_record_capture_fail_20260830_orchestrator_script.py` | `c30c34b338aec17b5c263e2715f0b9d5bef193e72771ddc0c342298411d76b5f` | 14957 |

## Recommended next step (Director decision only — NOT performed here)

A corrected single run would need the ros2-launch child to inherit the same
`LD_LIBRARY_PATH` that `launch_abs_sim.sh` exports (unitree_sdk2/lib +
libtorch). Because the task forbids retry, no re-run was attempted; a
corrected run requires separate Director authorization.
