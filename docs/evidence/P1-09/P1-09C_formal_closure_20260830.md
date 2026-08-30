# P1-09 — Formal Closure: authoritative events → P1-02 FormalRunWriter / validator (2026-08-30)

## Status

One real simulation-only capture wired the full chain end-to-end, and the P1-02
validator **actually output a verdict** (`validator_completed=true`,
`episode_state=INVALID`) on a representative real episode. The two remaining
original-acceptance blockers — (1) an authoritative structured safety/terminal
event source, and (2) a real episode classified `VALID`/`INVALID` by the P1-02
validator — are both demonstrated. The verdict is the honest fail-closed result
(missing authoritative data sources ⇒ INVALID), never a fabricated SUCCESS/VALID.

Not a benchmark, pilot, formal comparative experiment, Phase 1 Acceptance, or
real-robot run.

## Required main chain (completed in one run)

```
real MuJoCo + StateRL (/mujoco_rt_frame)
  → two-phase JSONL runtime record (VALID; 301 LIVE frames, single session)
  → authoritative structured events (single reducer: frame + real wait facts)
  → P1-02 FormalRunWriter (manifest/telemetry/events/summary/plots)
  → P1-02 validate_run
  → formal verdict = INVALID (validator_completed=true)
```

## Exact authoritative event-source mapping (no text-log parsing)

| Event type | Authoritative source |
|---|---|
| `episode_start` | record meta `created_at_ns` (monotonic domain) |
| `controller_active` | first LIVE frame `controller_active==1` |
| `rl_entered` | first LIVE frame `rl_entered==1` |
| `valid_ready` | last LIVE frame (record VALID in its own chain) |
| `terminal` | record terminal; `outcome=UNKNOWN` (no authoritative goal/collision/fall/timeout source) |
| `shutdown` | orchestrator real wait facts (`process_exit_code`, `forced_termination`, `shutdown_complete`, `shutdown_request_source`, `shutdown_monotonic_ns`) |
| safety fault | frame `safety_faulted` / `policy_state==FAULTED` (authoritative) |
| Recovery usage | frame `policy_state==RECOVERY` transitions (authoritative) |
| collision / fall / goal / timeout | **absent** — no verified authoritative runtime producer; validator classifies INVALID |

`safety_faulted`/`FAULTED` come only from the frame; process facts come only
from the orchestrator's actual `wait()`s. No ROS/MuJoCo text log is consulted.

## Identity binding (fail-closed)

`FormalRunWriter`-allocated `run_id` ↔ runtime-record `run_id` ↔ single
`session_id` ↔ frame `source_sequence`/`rl_step`/`monotonic_ns` ↔ process facts.
Any inconsistency refuses the formal write (see rejection tests).

- `formal_run_id` = `run-0cbf4a31137e48c79a1f61b4bb639a3e`
- `runtime_record_run_id` = `26861e6c08d5483ea5e911d2aeaf959e`
- `runtime_session_id` = `15818838355107` (matches the HUD live blocks)
- 301 LIVE authoritative frames in one record

## Run facts (orchestrator raw log)

| Item | Value |
|---|---|
| MuJoCo | pid=55645 pgid=55645, `scene_flat.xml`, interface `lo`, SIGINT → `rc=0` |
| ros2 launch | pid=55719 pgid=55719, `mujoco.launch.py simulation_test:=0`, controller active, SIGINT → `rc=0` |
| HUD | pid=56079, LIVE blocks, `session_id=15818838355107` |
| Recorder | `run_id=26861e6c08d5483ea5e911d2aeaf959e`; 301 LIVE + 183 MISSING |
| STOP | `state=STOPPED`, no terminal yet |
| Facts | `exit_code=0`, `forced_termination=false`, `shutdown_complete=true`, `shutdown_monotonic_ns=15834904293179`, `shutdown_request_source="recorder_stop;SIGINT_ros_launch;SIGINT_mujoco"` |
| FINALIZE | `normal_shutdown=true`, `FRAMES_ENDED_RC0` |

## P1-02 validator verdict

```json
{
  "episode_state": "INVALID",
  "reasons": ["invalid_event_clock:1..6", "invalid_terminal_event", "non_numeric_telemetry_clock"],
  "validator_completed": true,
  "schema_version": "abs-go2-formal-run/v1"
}
```

Every reason is a missing-authority signal (`simulation_time_s` absent from the
frame today; terminal `outcome=UNKNOWN`). No field was fabricated; the episode
is `INVALID`, not `SUCCESS`.

## Fail-closed rejection tests (offline, 12/12)

Source forgery (synthetic/legacy/non-LIVE frame), session change, sequence
rollback, monotonic rollback, duplicate terminal, misplaced terminal, missing
facts, facts/terminal contradiction, safety-fault never SUCCESS, plus the
positive case (valid record → formal INVALID verdict, summary
`terminal_outcome=UNKNOWN`, never SUCCESS).

Regression: P1-02 22, run-record 54/54, frame 24, HUD 17, adapter 16,
formal-frame-recorder 12/12; `git diff --check` clean.

## Remaining UNKNOWN (unchanged)

`simulation_time_s`, `collision`, `fall`, `reached_goal`, `timeout` have no
authoritative runtime producer in the frame today → the validator classifies
INVALID (never fabricated). P1-01 provenance/order, P1-08 cadence/dynamics
freeze, bridge/physics join and DDS teardown order remain UNKNOWN and are not
claimed. The manifest `rates_hz`/`thresholds`/`seeds` are DECLARED run-contract
values for this single run (not a P1-08 frozen baseline); `git.commit` is
recorded as sha256 of the raw SHA-1 to satisfy the accepted schema's 64-hex
constraint (raw SHA-1 in the orchestrator log).

## Evidence (archived under `docs/evidence/P1-09/`)

| File | bytes |
|---|---|
| `P1-09C_formal_closure_20260830_orchestrator_raw.log` | 4418 |
| `P1-09C_formal_closure_20260830_mujoco_raw.log` | 3851 |
| `P1-09C_formal_closure_20260830_ros2_launch_raw.log` | 22730 |
| `P1-09C_formal_closure_20260830_hud_raw.txt` | 47071 |
| `P1-09C_formal_closure_20260830_record.jsonl` | 762289 |
| `P1-09C_formal_closure_20260830_process_facts.json` | 242 |
| `P1-09C_formal_closure_20260830_post_run_summary.json` | 3414 |
| `P1-09C_formal_closure_20260830_manifest_context.json` | 3155 |
| `P1-09C_formal_closure_20260830_formal_verdict.json` | 611 |
| `P1-09C_formal_closure_20260830_formal_run/` | manifest.json 3337, telemetry.csv 440790, events.jsonl 1033, summary.json 2093, plots/ 5×436 |
| `P1-09C_formal_closure_20260830_orchestrator_script.py` | 14158 |

Prior evidence (incl. `P1-09AE_record_capture_fail_20260830.*`) is retained
untouched.

## Boundary

One run only; no retry. Only the launch environment was set for child processes
(`LD_LIBRARY_PATH` with unitree_sdk2/lib + libtorch). No project code/config/
schema/controller/DDS/thread/reload change. Not P1-09 Acceptance, not Phase 1
Acceptance — independent Reviewer review is still required.
