# P1-09AE — Runtime Record Completion (frame → same-run record → post-run summary)

## Status and boundary

Offline implementation plus mechanical tests only. No MuJoCo, ROS2, benchmark,
formal episode, pilot, or real robot was run. No formal `VALID` artifact was
generated, and the task does not claim P1-09 or Phase 1 Acceptance. The
P1-09X/Y/Z/AD clean-shutdown status is unchanged.

The data chain completed here is:

```
StateRL frame (/dev/shm/mujoco_rt_frame, real)
  → per-run JSONL record (full payload + terminal block, same run)
  → post-run summary computed ONLY from that saved record
```

It does **not** bypass the reviewed fail-closed boundaries: the P1-02
`FormalRunWriter` chain (`formal_experiment_contract.py`,
`formal_runtime_adapter.py`, `formal_rt_frame_recorder.py`) is unchanged and
still refuses to fabricate formal fields whose authority is missing. This task
adds a separate, explicitly non-formal RAW run archive.

## Implementation

New files (additive; no existing source was modified):

- `scripts/run_record.py` — per-run record + summary primitives:
  - `RunRecordRecorder` writes one JSONL per run: a `meta` line (run_id, record
    format version, source path), one `frame` line per snapshot (full payload +
    `availability` map derived from the controller's own flags), and one
    `terminal` line (terminal/event fields).
  - `frame_payload` / `frame_availability` preserve every controller-computed
    value verbatim; a flag-0 field (torque-saturated, collision) is recorded as
    unavailable, never as a fabricated result.
  - `load_record` / `summarize_record` / `report_record` compute the post-run
    summary from the saved record only.
  - Classification/reuse: the frame source is the existing
    `abs_rt_frame.read_shm_frame` + `classify_frame`; no frame source is
    reimplemented.
- `scripts/record_runtime_run.py` — thin production loop over the fixed path
  `/dev/shm/mujoco_rt_frame`; writes snapshots; finalizes with
  orchestrator-supplied process facts (exit code, forced termination, request
  source, shutdown completeness). Any fact not supplied is recorded UNKNOWN.
- `scripts/post_run_summary.py` — CLI that reads ONLY a saved run record and
  reports the summary (text or `--json`).
- `scripts/test_run_record.py` — 14 offline mechanical tests.

## Runtime fields recorded (all real, from the authoritative frame)

| Field | Frame member | Availability |
|---|---|---|
| session identity | `session_id` | yes |
| rl / control step | `rl_step` | yes |
| monotonic timestamp | `monotonic_ns` | yes |
| controller alive | `controller_active`, `rl_entered`, `rl_active` | yes |
| safety fault | `safety_faulted` | yes |
| Agile/Recovery state | `policy_state` | yes |
| RA value | `ra_value` | yes |
| actual linear velocity | `lin_vel[3]` | yes |
| command velocity | `command[3]` | yes |
| world pose | `world_pose[3]` (x, y, yaw) | yes |
| 11 rays | `ray2d[11]` | only when `ray_valid` |
| raw action | `action_raw[12]` | yes |
| clipped action | `action_clipped[12]` | yes |
| joint target | `joint_target_rad[12]` | yes |
| output torque | `torque_nm[12]` | yes |
| torque-saturated | `torque_saturated[12]` | NO — `torque_saturated_computed == 0`; recorded unavailable |
| collision | `collision_origin` | NO — always UNAVAILABLE today; recorded unavailable |

No mock/default/synthetic value fills any field. Missing fields are recorded
explicitly as unavailable/UNKNOWN.

## Terminal / event fields recorded

| Field | Source | Recorded value today |
|---|---|---|
| run duration | first→last `monotonic_ns` (real) | `duration_ns` |
| safety fault | frame `safety_faulted` / `policy_state==FAULTED` (real) | `safety_fault_seen` / `safety_fault_last` |
| process exit code | orchestrator process fact | recorded when supplied; else UNKNOWN |
| forced termination | orchestrator process fact | recorded when supplied; else UNKNOWN |
| shutdown request source | orchestrator process fact | recorded when supplied; else UNKNOWN |
| normal shutdown | derived from exit code / forced / complete facts | true/false/UNKNOWN (never fabricated) |
| termination reason | derived from real facts (safety > forced > rc≠0 > frames-ended-rc0) | e.g. SAFETY_FAULT / FORCED_TERMINATION / NONZERO_EXIT / FRAMES_ENDED_RC0 / UNKNOWN |
| simulation_time_s | no source in frame | UNKNOWN (reason recorded) |
| reached goal | no source | UNKNOWN (reason recorded) |
| timeout | no source | UNKNOWN (reason recorded) |
| collision events | collision_origin always UNAVAILABLE | UNKNOWN (reason recorded) |
| fall events | no source | UNKNOWN (reason recorded) |

## Post-run summary outputs (computed only from the saved record)

- `record_validity` + reasons (structural: meta/frame/terminal present, lines
  parse, no corrupt lines)
- `authoritative_runtime_source` (all LIVE frames are AUTHORITATIVE_RUNTIME)
- `outcome`: SUCCESS / FAILURE / INVALID / UNKNOWN — from the record only.
  SUCCESS is never produced today (no verifiable success source); FAILURE from
  real safety fault / forced termination / nonzero exit; INVALID on corrupt or
  non-authoritative records; else UNKNOWN.
- `termination_reason`, `normal_shutdown`, `duration_ns`
- velocity: horizontal-speed avg/peak + per-axis avg/peak (from LIVE frames)
- attitude: yaw (world_pose[2]) stats only — roll/pitch not in frame (noted)
- collision statistics: reported unavailable (no authoritative source)
- Recovery usage: recovery-step count, fraction, policy-transition count
- RA statistics: count/mean/min/max/last
- safety faults: faulted steps, first fault rl_step

## Fail-closed record validity (Reviewer REJECT closure)

A Reviewer REJECT required the record to be fail-closed. The following rules
were added to `run_record.py` (record format bumped to v2; frame lines now carry
`run_id`):

1. Whole-record invalidation: any PRESENT frame line that is INVALID,
   UNKNOWN_ORIGIN, LEGACY, SYNTHETIC, STALE, malformed (LIVE but no payload), or
   non-authoritative invalidates the ENTIRE record. Bad frames are never
   filtered out to keep a VALID label. `MISSING` lines (no frame at poll time)
   are gaps, not bad frames; they are tolerated but contribute no data, and a
   record with no LIVE frames is INVALID.
2. Cross-frame continuity over payload-bearing lines: `session_id` unchanged and
   `source_sequence` / `rl_step` / `monotonic_ns` strictly increasing; any
   violation invalidates the record; `duration_ns < 0` also invalidates.
3. Mixed-origin rejection covered by tests: LIVE+SYNTHETIC, LIVE+malformed,
   LIVE+bad-status, LIVE+STALE → all INVALID.
4. Safety fault = `safety_faulted` OR `policy_state == FAULTED`; it enters
   outcome precedence (FAILURE) ahead of forced/nonzero-exit causes.
5. Process facts are strictly type-checked at `finalize`
   (`_validate_process_facts`): no implicit bool conversion; the string
   `"false"` never becomes True. A fact of an invalid type is recorded UNKNOWN
   and listed in `fact_validation_errors`, which invalidates the record.
6. Run identity: meta / frames / terminal must share one `run_id`; the terminal
   line must be unique and be the final record line.

Negative/rejection tests added: LIVE+malformed, LIVE+SYNTHETIC, session change,
source_sequence rollback, rl_step rollback, monotonic rollback, FAULTED
policy_state with safety_faulted=0, malformed process-fact types (incl. `"false"`
not coerced to True), run-identity mismatch, terminal_run_id mismatch, duplicate
terminal, misplaced terminal, negative duration, missing run_id, and a
MISSING-only (no LIVE frame) record.

## Final fail-closed closure (Reviewer round 2)

Two remaining blockers were closed without expanding scope:

1. Unknown frame status fails closed. Frame status is validated against an
   explicit whitelist `{LIVE, MISSING}`. Any unrecognized status — `"BOGUS"`,
   an empty string, `null`, or a wrong type (int/list) — is never treated as
   MISSING or an ignorable frame; it produces `unknown_frame_status:<i>:<status>`
   and invalidates the whole record. The previously-known invalid statuses
   (INVALID / UNKNOWN_ORIGIN / LEGACY / SYNTHETIC / STALE) keep their
   `non_live_frame_status` reason.
2. Full LIVE payload schema validation. Before any statistics access a payload
   field, `_validate_live_payload` validates the full schema of every field the
   record stores and the summary depends on: required keys present; scalar int
   types legal (bool rejected); boolean and enum domains legal (policy_state
   0..2, source 0..3, ray_origin 0..1, collision_origin 0); vector/list lengths
   exact (lin_vel / command / world_pose = 3, ray2d = 11, 5×12 command chain);
   every numeric value finite (NaN/Inf rejected). Any missing field, wrong type,
   wrong vector length, NaN/Inf, or malformed nested structure →
   `invalid_payload:<i>:<...>` → record INVALID. The summary computes statistics
   only over payloads that passed validation, so it cannot KeyError/TypeError on
   a malformed record; `load_record` also rejects non-object JSON lines instead
   of crashing.

The summary returns normally (never raises) on any malformed record. 11 new
negative tests were added (LIVE+unknown / unknown-only / empty-string / wrong-
type status; missing lin_vel / world_pose / ra_value; wrong scalar type; wrong
vector length; NaN; Inf; malformed nested action/command structure; non-object
JSON lines), each asserting no uncaught exception, `record_validity == INVALID`,
outcome is not SUCCESS, and a traceable reason.

## MISSING gap semantics (Reviewer round 3)

A MISSING frame is a legitimate sampling gap only when it carries NO payload
and NO availability (both null or an empty mapping). If a MISSING frame carries
a non-null / non-empty `payload` and/or `availability`, the whole record is
INVALID via `malformed_missing_frame:<i>`; the frame is never ignored and the
record is never marked authoritative VALID. Tests: LIVE + legitimate empty
MISSING (VALID — not invalid just because of MISSING), LIVE + MISSING-with-
payload (INVALID), LIVE + MISSING-with-availability (INVALID), LIVE +
MISSING-with-both (INVALID), and empty-dict payload/availability (VALID).

## Two-phase capture / finalize lifecycle (main-chain blocker)

The recorder now runs the runtime flow in two explicit phases:

1. CAPTURE — `start()` then repeated `record_snapshot()` write real LIVE frame
   lines; no terminal is written.
2. STOP — `stop_sampling()` ends capture; the record stays unfinalized
   (`state == STOPPED`), and no further frame line can be written, so a
   controller-exit INVALID frame in shared memory is never sampled into the
   record.
3. FINALIZE — after the MuJoCo/controller process exits, `finalize()` writes the
   single terminal line from real process facts (exit code / forced termination /
   shutdown complete / request source; any missing fact stays UNKNOWN, never
   default-filled) and closes the record. A duplicate `finalize()` is rejected,
   and frame writes after finalize are rejected. The post-run summary reads only
   this final record.

`state` / `stopped` / `finalized` are exposed for observability; the CLI
`record_runtime_run.py` now calls `stop_sampling()` before `finalize()`. Record
lines are flushed per write so the on-disk record always reflects the lifecycle
state and survives an unexpected exit.

Two-phase tests added (54 total): capture→stop→finalize VALID; frame write
rejected after stop; not-finalized before finalize; duplicate finalize rejected;
frame write rejected after finalize; missing exit fact stays UNKNOWN; real
exit_code written (NONZERO_EXIT → FAILURE); stopped record not polluted by
controller-exit frames.

## Reviewer problem → implementation → test

| Design requirement | Implementation | Fixture coverage |
|---|---|---|
| Same-run record keeps session/frame/step/time | run_id in meta+terminal; session/sequence/rl_step/monotonic_ns in payload | meta/frame/terminal + ordering tests |
| Full real payload saved, no mock fill | payload = frame bytes verbatim; availability from flags | round-trip + unavailable-fields tests |
| Missing fields marked unavailable/UNKNOWN | terminal block UNKNOWN with reasons | terminal-unknown test |
| Post-run summary reads only the record | summarize_record(load_record(path)) | corrupt record → INVALID |
| Outcome from real facts only | safety/forced/nonzero → FAILURE; rc0+no source → UNKNOWN; never SUCCESS | safety / forced / clean-rc0 tests |
| Synthetic never promoted | non-AUTHORITATIVE source → non-authoritative, INVALID | synthetic-source test |
| Stale real frames archived but not scored | STALE line kept; summary scores LIVE only | stale-frame test |
| Facts not fabricated when orchestrator absent | UNKNOWN terminal fields, normal_shutdown=None | missing-facts test |

## Command evidence

| Command | Exit code | Result |
|---|---:|---|
| `rtk python3 scripts/test_run_record.py` | 0 | 54/54 PASS (incl. fail-closed rejection cases) |
| `rtk python3 -m py_compile scripts/run_record.py scripts/record_runtime_run.py scripts/post_run_summary.py scripts/test_run_record.py` | 0 | PASS |
| `rtk python3 scripts/test_abs_rt_frame.py` | 0 | regression PASS |
| `rtk python3 scripts/test_formal_rt_frame_recorder.py` | 0 | 12/12 PASS (regression) |
| `rtk python3 scripts/test_formal_runtime_adapter.py` | 0 | regression PASS |
| `rtk python3 scripts/test_formal_experiment_contract.py` | 0 | regression PASS |
| `rtk python3 scripts/record_runtime_run.py --output <tmp> --iters 2 --facts <facts.json>` | 0 | records meta + MISSING frames + terminal (no live shm in this environment) |
| `rtk python3 scripts/post_run_summary.py <record>` | 0 | text + `--json` summary verified on a 3-live-frame record |
| `rtk git diff --check` | 0 | no whitespace errors |

A 3-LIVE-frame fixture record produced: validity VALID, authoritative True,
outcome UNKNOWN (reasons: no reached-goal/timeout/collision/fall source),
normal_shutdown True, termination FRAMES_ENDED_RC0, horizontal speed avg
5.0 m/s peak 10.0 m/s, Recovery 1/3 (2 transitions), RA mean/min/max
-0.2/-0.3/-0.1, yaw recorded.

## Remaining UNKNOWN (recorded, not fabricated)

- `simulation_time_s` — no producer in the frame; a bridge-side sim-time source
  would be needed (out of scope for this chain).
- `reached_goal`, `timeout`, `fall_events` — no authoritative event producer
  exists; adding one is outside this task and forbidden by scope.
- `collision_events` — `collision_origin` is always UNAVAILABLE today
  (bridge-side only; no authoritative event source).
- `process_exit_code` / `forced_termination` / `shutdown_request_source` —
  recorded only when the run orchestrator supplies them; otherwise UNKNOWN.
- Existing P1-09 UNKNOWN items are unchanged: measured cadence, torque
  saturation authority, active-ray-source runtime provenance, P1-01 model
  provenance/order, and clean all-process shutdown (P1-09X/Y/Z/AD).

## Deferred findings (not blockers for this chain)

Per the task scope rule, each of these is deferred because this chain works
without it; none is required to complete
“real data → same-run save → post-run summary”:

1. Bridge-side authoritative `simulation_time_s` producer (would require a
   bridge change outside this task).
2. Authoritative collision/fall/goal/timeout structured-event producers (new
   safety-event architecture; forbidden in this task).
3. Run orchestrator sidecar that supplies process facts to
   `record_runtime_run.py --facts` (needs a future authorized runtime run; the
   facts interface is defined and tested).

## Acceptance

This is the data-chain completion only. It is not a benchmark, formal
experiment, formal `VALID` run, P1-09 Acceptance, or Phase 1 Acceptance. P1-09
remains **EXECUTING / NOT ACCEPTED** and Phase 1 remains **NOT ACCEPTED**.
Independent Reviewer review is required before any runtime validation.
