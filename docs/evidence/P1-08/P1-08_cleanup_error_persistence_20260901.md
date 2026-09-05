# P1-08 — Cleanup-Error Persistence and Poll-Exception Closure (2026-09-01)

Status: **REPAIRED / AWAITING INDEPENDENT REVIEW**.
Phase 1 remains **NOT ACCEPTED**.

Scope: closes the two remaining harness gaps — per-child `cleanup_errors` were
not persisted to `process_facts.json`, and a `proc.poll()` exception made
cleanup return early without recording signal/wait attempts. **No MuJoCo/ROS2
runtime recapture was run**; no manifest/identity/sim-clock/model/controller/
scene/algorithm/parameter change.

## 1. Cleanup-error persistence schema

`build_process_facts` now carries per-child structured facts into the final
`process_facts.json`:

```
child.<name>.cleanup_errors: [{ "stage", "exception_type", "exception_message", "time_s" }]
child.<name>.poll_attempts:   [{ "stage": "poll", "result": running|exited|exception,
                                 "rc"?, "exception_type"?, "exception_message"?, "time_s" }]
```

plus a top-level `cleanup_error_count` summary (which does NOT replace the
per-child originals). `recorder.finalize()` receives the same top-level facts
that were durably written. Each error keeps at least stage, exception type,
exception message, and time.

## 2. Poll-exception lifecycle behavior

`_handle_child` no longer early-returns on a poll exception:

- the poll exception is recorded as a `poll_attempt` (result="exception" with
  type/message/time);
- the child is treated as state-unknown/running and still receives a recorded
  SIGINT signal-attempt and a wait-attempt;
- if the wait cannot confirm the exit, `wait_rc` stays `None` (UNKNOWN) — never
  fabricated as exited or rc=0;
- remaining children are still processed (one child's failure never blocks
  another);
- `wait_pid` is poll-exception-tolerant (retries with sleep, returns None at the
  deadline rather than fabricating an rc);
- `process_facts.json` durable write → `recorder.finalize(same facts)` →
  stats/logs are never bypassed.

## 3. Real persisted-facts negative tests (test_p1_08_harness.py)

Both new tests drive the REAL `_finalize_capture` and READ the generated
`process_facts.json` (they do NOT mock `build_process_facts`):

| Case | Assertion (from the real file) |
|---|---|
| signal/wait exception | `child.mujoco.cleanup_errors` contains a `signal_or_wait` entry with stage/`RuntimeError`/"wait boom"/time |
| poll exception | `child.mujoco.poll_attempts[0]` = poll exception with type/message |
| poll exception → signal+wait attempts | SIGINT/TERM/KILL all attempted after the poll exception (all `delivered=False` because killpg fails) |
| poll exception → no fabrication | `child.mujoco.exit_code is None`, `escalated False`, top-level `exit_code None`, `shutdown_complete False`, source != SIGINT, forced != true |
| multi-child isolation | the already-exited `ros2_launch` still cleans up (exit_code 0) |
| persistence/finalize | `process_facts.json` written + `recorder.finalize` called |

All use fake processes + injected sender (`mock os.killpg` / `mock wait_pid`);
no real child is launched.

## 4. Regression

- C++ `p1_08_sim_clock_test` **PASS (45)**; `ctest` **1/1**.
- `test_p1_08_sim_clock.py` **PASS (32)**; `test_p1_08_baseline_identity.py`
  **PASS (21)**; `test_p1_08_harness.py` **PASS (77)**.
- `py_compile` all scripts OK; JSON validity OK; `git diff --check` PASS.

## 5. Old-capture boundary

The 2026-09-01 v1 capture (`capture_20260901_rerun`) and its identities
(`bdd47a0d…`, `99b995b0…`, `9840462e…`) remain **superseded / non-acceptance**;
the 2026-09-01 v2 preflight failure
(`P1-08_v2_capture_preflight_fail_20260901.md`) remains an archived single
failure. **No new v2 recapture was performed.**

Phase 1 remains **NOT ACCEPTED**; P1-10 not started; P1-08 not accepted.
