# P1-09AE — Single Real Runtime Record Capture (2026-08-30, env-corrected)

## Status

One Director-authorized, **environment-corrected** simulation-only run completed
the runtime-record main chain end-to-end:

```
real MuJoCo + StateRL (/mujoco_rt_frame)
  → HUD LIVE (same session)
  → two-phase runtime recorder (CAPTURE → STOP → FINALIZE)
  → real process facts (actual waits)
  → post_run_summary on the same saved record only
```

Not a benchmark, formal experiment, pilot, FormalRunWriter use, or Acceptance
claim. P1-09 and Phase 1 remain **EXECUTING / NOT ACCEPTED**.

This is the direct successor of the 2026-08-30 11:28 launch failure
([`P1-09AE_record_capture_fail_20260830.md`](P1-09AE_record_capture_fail_20260830.md)),
whose evidence is retained untouched. The only change in this run is the launch
environment: every child process (MuJoCo, ros2 launch, HUD, recorder, control
pub) inherited `LD_LIBRARY_PATH` containing
`/home/lidio/Libraries/unitree_sdk2/lib` and
`/home/lidio/Libraries/libtorch-cpu-2.0.1/lib`, matching
`scripts/launch_abs_sim.sh` line 24.

## Preflight (all PASS)

| Check | Result |
|---|---|
| LD_LIBRARY_PATH (child env) | `unitree_sdk2/lib` + `libtorch-cpu-2.0.1/lib` present |
| `ldd libhardware_unitree_mujoco.so` (child env) | `libddsc.so.0` + `libddscxx.so.0` resolved; `not_found=0` |
| X11 | `DISPLAY=:0`, `XAUTHORITY=/run/user/1000/gdm/Xauthority`, `xdpyinfo` rc=0 |

## Run facts (orchestrator raw log)

| Item | Value |
|---|---|
| MuJoCo | pid=28924 pgid=28924, `scene_flat.xml`, interface `lo`, SIGINT → `rc=0` |
| ros2 launch | pid=28982 pgid=28982, `mujoco.launch.py simulation_test:=0`, controller active 11:41:13, SIGINT → `rc=0` |
| HUD | pid=29337, 31 LIVE blocks, `session_id=8049381969251` |
| Recorder | `run_id=d9c988223cec4b1385a8fd031abc385f` |
| Control | `/control_input` `2→2→3` (reliable QoS); RL confirmed (`rl_active`, policy AGILE) |
| Capture window | **306 LIVE + 176 MISSING** frames, `duration_ns=15524508836` (~15.5 s) |
| STOP | recorder `state=STOPPED`, `finalized=False`, last line kind=`frame`, **no terminal yet** |
| Shutdown | ros2 SIGINT wait `rc=0`; MuJoCo SIGINT wait `rc=0`; no TERM/KILL; residue none |
| Process facts | `exit_code=0`, `forced_termination=false`, `shutdown_complete=true`, `shutdown_request_source="recorder_stop;SIGINT_ros_launch;SIGINT_mujoco"` |
| FINALIZE | `normal_shutdown=true`, `termination_reason=FRAMES_ENDED_RC0`, `frames_observed=306` |
| Post-run summary | `record_validity=VALID`, `authoritative_runtime_source=true`, `outcome=UNKNOWN` |

All exit facts come from the orchestrator's **actual `wait()` return codes** of
the two target processes; nothing was inferred from appearance.

## Record verification

- 484 lines = 1 meta + 482 frame + 1 terminal; terminal **unique and final line**;
  `run_id` consistent across meta/frames/terminal.
- 306 LIVE frames, all `source=1` (`AUTHORITATIVE_RUNTIME`); 176 MISSING gaps
  (legal, no payload/availability); no other status.
- Continuity over LIVE payloads: one `session_id=8049381969251`;
  `source_sequence` / `rl_step` / `monotonic_ns` strictly increasing.
- HUD `session_id=8049381969251` **matches** the record session (same session).

## Post-run summary (from the saved record only)

- `record_validity=VALID`; `authoritative_runtime_source=true`
- `normal_shutdown=true`; `termination_reason=FRAMES_ENDED_RC0`
- duration 15.5 s; velocity `vx_avg=0.573 m/s`, horizontal peak `2.577 m/s`
- RA statistics 306 samples, mean `-0.956`, min/max `-1.0/-0.724`
- Recovery usage 0 steps / 0 transitions; safety faults 0
- attitude yaw recorded (mean `-14.5 deg`); collision reported unavailable
- `simulation_time_s` = UNKNOWN (no source in frame)

## UNKNOWN (preserved, never fabricated)

`reached_goal`, `timeout`, `collision_events`, `fall_events`,
`simulation_time_s` → UNKNOWN with explicit reasons; therefore
`outcome=UNKNOWN`, **never SUCCESS**. Internal bridge/physics join and DDS
teardown ordering after SIGINT remain UNKNOWN (not emitted by the raw logs).

## Boundary

- One run only; no retry; no code/config/schema/controller/DDS/thread/reload/
  FormalRunWriter change; no benchmark/pilot/formal/real-robot run.
- Not P1-09 Acceptance, not P1-02 runtime integration, not Phase 1 Acceptance.
  Independent Reviewer review is still required.

## Evidence (archived, byte-identical to /tmp/p1_09ae_capture_v2)

| File | bytes |
|---|---|
| `P1-09AE_record_capture_20260830_orchestrator_raw.log` | 3839 |
| `P1-09AE_record_capture_20260830_mujoco_raw.log` | 3851 |
| `P1-09AE_record_capture_20260830_ros2_launch_raw.log` | 22637 |
| `P1-09AE_record_capture_20260830_hud_raw.txt` | 48058 |
| `P1-09AE_record_capture_20260830_record.jsonl` | 772251 |
| `P1-09AE_record_capture_20260830_process_facts.json` | 199 |
| `P1-09AE_record_capture_20260830_post_run_summary.json` | 3412 |
| `P1-09AE_record_capture_20260830_post_run_summary.txt` | 1467 |
| `P1-09AE_record_capture_20260830_orchestrator_script.py` | 14675 |

Prior failure evidence (`P1-09AE_record_capture_fail_20260830.*`) is retained
unchanged.
