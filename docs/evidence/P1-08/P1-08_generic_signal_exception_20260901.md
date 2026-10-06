# P1-08 — Generic Signal-Exception Fail-Closed Cleanup Closure (2026-09-01)

Status: **REPAIRED / AWAITING INDEPENDENT REVIEW**.
Phase 1 remains **NOT ACCEPTED**.

Scope: closes the `_signal_pg()` non-OSError-exception blocker (an ordinary
exception was neither archived nor contained, and could interrupt
`_finalize_capture()`). **No MuJoCo/ROS2 runtime recapture was run**; no
manifest/identity/sim-clock/controller/model/algorithm change.

## 1. `_signal_pg()` generic-exception facts

`_signal_pg()` now catches **`Exception`** (never `BaseException`/`SystemExit`/
`KeyboardInterrupt`). On ANY ordinary exception it records a structured entry:

```
{"signal", "time_s", "delivered": false,
 "result": "failed:<exc>", "target_pid", "target_pgid",
 "exception_type": <type name>, "exception_message": <str(exc)>}
```

and returns `False`. The exception is never misread as delivered, never turned
into a `SIGINT` source, and never into `forced_termination`. Signal name,
target PID/PGID and request time are always preserved.

## 2. `_finalize_capture()` continuity

Each child's signal/wait handling is now wrapped in a per-child exception guard
(`_handle_child`):

- one child's signal/wait exception is recorded (`cleanup_errors`) and a wait is
  still attempted (recorded as UNKNOWN/failed, never fabricated success);
- the remaining children are still processed;
- the durable `process_facts.json` write and `recorder.finalize(same_facts)` are
  unconditional and cannot be bypassed;
- multiple cleanup exceptions are all retained (per-child `cleanup_errors`),
  and TERM/KILL escalation still only occurs on the timeout branch with
  `delivered` semantics.

## 3. Negative tests (test_p1_08_harness.py)

| Case | Assertion |
|---|---|
| SIGINT sender raises `RuntimeError("boom")` | delivered=False; `exception_type`/`exception_message` present; source != SIGINT |
| TERM sender raises ordinary exception | forced_termination != true |
| signal exception during `_finalize_capture` | `process_facts.json` written AND `recorder.finalize` called |
| one child's exception vs another child | both children signal-attempted (one does not block the other) |
| run_record top-level facts on failure | `shutdown_complete=False`, `exit_code=None` (no fabricated normal shutdown) |

All use fake processes + injected sender (`mock os.killpg`); no real child is
launched.

## 4. Regression

- C++ `p1_08_sim_clock_test` **PASS (45)**; `ctest` **1/1**.
- `test_p1_08_sim_clock.py` **PASS (32)**; `test_p1_08_baseline_identity.py`
  **PASS (21)**; `test_p1_08_harness.py` **PASS (62)**.
- `py_compile` all scripts OK; JSON validity OK; `git diff --check` PASS.

## 5. Old-capture boundary

The 2026-09-01 v1 capture (`capture_20260901_rerun`) and its identities
(`bdd47a0d…`, `99b995b0…`, `9840462e…`) remain **superseded / non-acceptance**;
the 2026-09-01 v2 preflight failure
(`P1-08_v2_capture_preflight_fail_20260901.md`) remains an archived single
failure. **No new v2 recapture was performed.**

Phase 1 remains **NOT ACCEPTED**; P1-10 not started; P1-08 not accepted.
