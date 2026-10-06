# P1-09AC — Formal Telemetry Sequence and Synthetic-Origin Closure

## Status

Offline-only correction. No MuJoCo, ROS2, benchmark, formal episode, pilot,
real robot, `FormalRunWriter`, or formal artifact was used.

## Corrections

- Source frame sequence is transport/seqlock ordering: it must increase, but
  gaps are allowed.
- `rl_step` is controller-step ordering: it must increase, but sampled gaps are
  allowed.
- Formal telemetry sequence is recorder-owned and starts at zero. It advances
  only when a sample has passed all runtime-eligibility and authority checks.
- Rejected samples do not mutate accepted session/order state and do not
  consume formal sequence. They receive only an independent `rejection_index`.
- The first formally eligible runtime sample binds `session_id`; a later
  session change is rejected.
- Production capture has no arbitrary path/dict injection and always reads
  `/dev/shm/mujoco_rt_frame` through the existing reader.
- Synthetic, legacy, unknown-origin, stale, invalid, and non-LIVE inputs never
  obtain runtime eligibility. A synthetic fixture that uses the
  `AUTHORITATIVE_RUNTIME` enum is still rejected by the test-only synthetic
  boundary and cannot reach a writer.
- Current production capture has `authority_complete=False` by construction,
  so even a coherent LIVE frame returns INVALID with explicit missing-source
  reasons. The private complete-authority fixture path exists only to prove
  future sequence semantics; it does not write artifacts.

## Reviewer problem → fix → test

| Problem | Fix | Test |
|---|---|---|
| Rejected frame consumed formal sequence | Commit accepted ordering/counter only after authority-complete eligibility | rejection does not consume sequence; eligible sequence 0/1 contiguous |
| Source sequence and rl_step conflated with formal sequence | Separate fields and state variables | source/rl gaps allowed; rollback/repeat rejected |
| Synthetic authoritative-looking fixture could be promoted | Explicit private synthetic probe always returns INVALID | synthetic-origin rejection |
| Diagnostics needed an independent number | `rejection_index`, never named telemetry sequence | rejection index assertion |
| Safety/incomplete shutdown must remain invalid | `finalize()` retains safety-first and shutdown fail-closed rules | safety conflict and forced/incomplete shutdown tests |

## Actual command results

| Command | Exit code | Result |
|---|---:|---|
| `rtk python3 scripts/test_formal_rt_frame_recorder.py` | 0 | 12/12 PASS |
| `rtk python3 scripts/test_formal_runtime_adapter.py` | 0 | 16/16 PASS |
| `rtk python3 scripts/test_abs_rt_frame.py` | 0 | 24/24 PASS |
| `rtk python3 scripts/test_formal_experiment_contract.py` | 0 | 22/22 PASS |
| `rtk python3 -m py_compile scripts/formal_rt_frame_recorder.py scripts/test_formal_rt_frame_recorder.py` | 0 | PASS |
| `rtk git diff --check` | 0 | no whitespace errors |

## Remaining UNKNOWN

Runtime authority is still incomplete: simulation time, seed/config
provenance, collision/fall events, torque saturation, active ray source,
measured cadence and clean shutdown are unresolved. P1-01 provenance and
P1-09X/Z GLFW BLOCKED status remain unchanged. No formal VALID run is possible.
