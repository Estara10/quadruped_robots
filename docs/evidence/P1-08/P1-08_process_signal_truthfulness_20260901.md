# P1-08 — Process-Signal Fact Truthfulness Closure (2026-09-01)

Status: **REPAIRED / AWAITING INDEPENDENT REVIEW**.
Phase 1 remains **NOT ACCEPTED**.

Scope: closes the two process-facts truthfulness defects (SIGINT source and
`forced_termination` were derived from "a signal was attempted", not "a signal
was actually delivered"). **No MuJoCo/ROS2 runtime recapture was run**; no model/
controller/sim-clock/identity/manifest/scene/algorithm change.

## 1. SIGINT truthfulness

`shutdown_request_source` is now `"SIGINT"` **only** when at least one target
child/process-group actually **received** SIGINT (`os.killpg` returned without
error). `_signal_pg` records, per signal, `{"signal", "time_s", "delivered",
"result"}` where `delivered` is `True` only on a successful `killpg`; a failed/
raised/absent target records `delivered=False` with a `failed:<reason>` result.
If no SIGINT was actually delivered, the coordinator source is `"UNKNOWN"` —
never fabricated as `"SIGINT"` merely because the signal function was invoked.

## 2. forced_termination truthfulness

`forced_termination` is `True` **only** when a TERM or KILL was actually
delivered. `_wait_or_escalate` now sets the per-child `escalated` flag only when
`_signal_pg` for TERM/KILL returns `delivered=True`; a failed escalation is
recorded in the signal timeline (delivered=False) and does NOT mark the child
escalated. `build_process_facts` recomputes the per-child `escalated` flag from
the delivered signal timeline (`_delivered(signals, ("SIGTERM","SIGKILL"))`) so
the signal timeline, `escalated`, and top-level `forced_termination` are mutually
consistent. A nonzero natural exit is never forced.

## 3. Negative tests (test_p1_08_harness.py)

| Case | Assertion |
|---|---|
| SIGINT delivered | `source == "SIGINT"` |
| SIGINT send failed | `source != "SIGINT"`; `delivered=False` fact traceable |
| TERM delivered | `forced_termination is True`, `escalated True` |
| TERM send failed | `forced_termination is not True`, `escalated False` |
| KILL delivered | `forced_termination is True` |
| KILL send failed (TERM also failed) | `forced_termination is not True` |
| natural nonzero exit | `forced_termination is False` |
| top-level facts match recorder | `exit_code/forced/source/complete` read by `RunRecordRecorder.finalize` |

`test_signal_delivery_truthfulness` injects `os.killpg` (success / `OSError`) to
prove `delivered` stays truthful with no real child/signal. All signal-sender
tests use fake processes + mock `os.killpg`; no real child is launched.

## 4. Regression

- C++ `p1_08_sim_clock_test` **PASS (45)**; `ctest` **1/1**.
- `test_p1_08_sim_clock.py` **PASS (32)**; `test_p1_08_baseline_identity.py`
  **PASS (21)**; `test_p1_08_harness.py` **PASS (51)**.
- `py_compile` all scripts OK; JSON validity OK; `git diff --check` PASS.
- No existing two-phase-order / manifest / identity / reader-stat test was
  weakened.

## 5. Old-capture boundary

The 2026-09-01 v1 capture (`capture_20260901_rerun`) and its identities
(`bdd47a0d…`, `99b995b0…`, `9840462e…`) remain **superseded / non-acceptance**;
the 2026-09-01 v2 preflight failure
(`P1-08_v2_capture_preflight_fail_20260901.md`) remains an archived single
failure. **No new v2 recapture was performed.**

Phase 1 remains **NOT ACCEPTED**; P1-10 not started; P1-08 not accepted.
