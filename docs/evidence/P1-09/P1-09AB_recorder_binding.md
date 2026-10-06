# P1-09AB — Fail-Closed Runtime-Frame Recorder Binding

## Status and boundary

Offline implementation plus synthetic rejection fixtures only. No MuJoCo,
ROS2, benchmark, formal episode, pilot, or real robot was run. No formal
`VALID` artifact was generated. P1-09X/Z GLFW BLOCKED status is unchanged.

## Implementation

Added `scripts/formal_rt_frame_recorder.py` and its offline tests.

- Production `capture_once()` has one fixed source:
  `/dev/shm/mujoco_rt_frame`, read through the existing `read_shm_frame` seqlock
  reader. It has no production arbitrary path or frame-dictionary injection
  API.
- Only stable, fresh, coherent `AUTHORITATIVE_RUNTIME` frames are candidates.
  Missing, stale, invalid, synthetic, legacy, and unknown-origin frames are
  rejected.
- The first formally eligible frame binds the controller `session_id`; later
  changes, source-sequence reversal, `rl_step` reversal, and monotonic-time
  reversal are rejected. Source sequence gaps are allowed. Recorder-generated
  formal sequence starts at zero and increments continuously only for samples
  that are actually eligible to be written; rejected samples use a separate
  rejection index and consume no formal sequence.
- The current frame is deliberately never passed to `FormalRunWriter`: the
  recorder returns `INVALID` while simulation time, seed/config provenance,
  collision/fall authority, torque-saturation authority, active ray-source
  provenance, measured cadence, and complete shutdown evidence remain absent.
- `finalize()` applies safety-before-arrival ordering and rejects nonzero or
  forced shutdown. It never creates a summary or claims `SUCCESS`.
- Synthetic input is available only through an explicitly named private test
  probe and can never be runtime-valid.

## Reviewer issue → implementation → test

| Design requirement | Implementation | Fixture coverage |
|---|---|---|
| Fixed authoritative source | `capture_once()` uses the constant `/dev/shm/mujoco_rt_frame` and existing reader | fixed-source/no-writer-path tests |
| Synthetic/legacy/non-LIVE rejection | classifier result and private synthetic probe always produce `INVALID` | synthetic and missing/non-LIVE tests |
| Session binding | first live candidate binds `session_id`; change rejects | session-change test |
| Source ordering | strict increase; gaps allowed | skip accepted, reversal rejected |
| Formal ordering | recorder-owned counter starts at 0 and is continuous only for eligible candidates; rejected samples do not consume it | formal-sequence/rejection-index assertions |
| `rl_step`/time ordering | strict increase; rollback rejects | reversal test |
| Missing formal authority | explicit missing-authority reasons; no writer member/path | missing-authority and no-writer tests |
| Shutdown safety | rc must be 0, no forced termination, explicit complete-shutdown marker | incomplete/forced shutdown test |
| Safety outcome precedence | collision/fall cannot coexist with SUCCESS | safety-overrides-success test |

## Command evidence

| Command | Exit code | Result |
|---|---:|---|
| `rtk python3 scripts/test_formal_rt_frame_recorder.py` | 0 | 12/12 PASS |
| `rtk python3 scripts/test_formal_runtime_adapter.py` | 0 | 16 tests OK |
| `rtk python3 scripts/test_abs_rt_frame.py` | 0 | 24 tests OK |
| `rtk python3 scripts/test_formal_experiment_contract.py` | 0 | 22 tests OK |
| `rtk python3 -m py_compile scripts/formal_rt_frame_recorder.py scripts/test_formal_rt_frame_recorder.py` | 0 | PASS |
| `rtk git diff --check` | 0 | no whitespace errors |

An initial recorder test invocation returned rc=1 because one fixture expected
the internal missing-source reason rather than the existing classifier's
`FrameStatus.MISSING` result. The test was corrected to assert the actual
contract and the complete suite above was rerun successfully; no acceptance
criterion was lowered.

## Remaining UNKNOWN / not closed

- No authoritative runtime producer is connected to the adapter, so no formal
  runtime `VALID` run is possible.
- `simulation_time_s`, seed lineage, effective config, collision/fall events,
  torque saturation, active ray-source provenance, measured cadence and full
  shutdown evidence remain unavailable/UNKNOWN.
- P1-01 model provenance/order UNKNOWN remains blocked by training-server
  availability.
- P1-09X/Z GLFW/display BLOCKED status remains unchanged.
