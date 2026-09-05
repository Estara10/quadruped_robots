# P1-08 — Harness Lifecycle, Full Model Closure, and Deterministic Clock-Test Closure (2026-09-01)

Status: **REPAIRED / AWAITING INDEPENDENT REVIEW**.
Phase 1 remains **NOT ACCEPTED**.

Scope: closes the Harness-v2 REJECT blockers — full model-closure validation,
preflight exception/lock/shm fail-closed, unified two-phase cleanup with
correct process-facts semantics, and the deterministic (non-flaky) C++ sim-clock
stress test. **No MuJoCo/ROS2 runtime recapture was run**; no baseline produced;
no P1-10; no policy/controller/RA/Recovery/scene/solver/timestep/gains change.

## A. Full model-closure validation

`build_p1_08_manifest.py::resolve_closure` now recursively discovers and hashes
the WHOLE MuJoCo closure:

- `<include>` XMLs resolved **relative to the current XML's directory**;
- **escape rejection** (`is_within`): any include/asset outside the closure root
  (default = root XML's directory) is a failure;
- **cycle detection** (visiting set) — a cycle is a failure, not silently skipped;
- **missing** include/asset is a failure;
- manifest records every included XML + asset with its SHA-256 (`xml_files`,
  `asset_files`, `included_xml_files`), plus `failures` and `closure_sha256`.

`verify_manifest_hashes` re-runs `resolve_closure` on the manifest's canonical
root and compares the FULL closure (closure_sha256 + every XML hash) against the
manifest — not just the root XML. Any fresh-closure failure or hash drift →
PRECHECK FAIL.

Offline tests (`test_p1_08_harness.py`): clean closure no failures; mutating an
included XML changes the closure hash; include escape/cycle/missing each
detected; the real manifest + real scene verifies clean.

## B. Preflight exception / lock / shm fail-closed

- `preflight` is wrapped in `capture()` with try/except: any exception becomes a
  **structured PRECHECK FAIL** archived to `<out>_preflight_fail.json` (reason
  preserved) — no bare traceback, no child launch.
- `pgrep` distinguishes "not found" (rc=1) from exec failure/timeout (rc=2,
  exception) — failure/uncertain is PRECHECK FAIL, never treated as "none".
- **Harness-owned exclusive capture lock** (`flock`, non-blocking) is acquired
  before preflight and released after all child cleanup. A held lock → PRECHECK
  FAIL. Documented scope: prevents two instances of THIS harness from racing;
  it does NOT control external non-cooperating processes (those are checked
  fail-closed separately).
- Narrow shm cleanup: only the two task shms; before/after state recorded; a
  still-present or unlink failure is fail-closed; a second residual-process
  check runs after cleanup (spawn guard).

## C. Two-phase cleanup + process-facts semantics

Single `_finalize_capture` runs for ALL post-launch branches (success,
controller-not-ready, sim-clock-not-ready, reader failure, exception, timeout) —
no early return bypasses the finally.

Exact lifecycle order:
1. `recorder.stop_sampling()` (only if started, not finalized);
2. SIGINT (recorded) per child, ordered ROS-then-MuJoCo;
3. real `wait()` per child;
4. TERM then KILL escalation only on timeout, each signal recorded with
   time/PID/PGID/result; `escalated=true` only on actual TERM/KILL;
5. `process_facts.json` written BEFORE finalize;
6. `recorder.finalize()` with the same top-level facts;
7. reader stats, raw logs, cleanup result saved.

Facts semantics (fail-closed, no fabrication):
- `forced_termination=true` ONLY when TERM/KILL actually sent (plain nonzero
  exit is NOT forced);
- `shutdown_request_source="SIGINT"` only when SIGINT actually sent; otherwise
  "UNKNOWN";
- coordinator `exit_code=0` only when all required children wait rc==0; nonzero
  → deterministic nonzero; missing/unwaited → None (never 0);
- per-child PID/PGID/signals/wait-rc/escalation preserved; missing child marked
  `not_launched`.

Offline tests cover stop-before-signal order, facts-before-finalize, and
natural/SIGINT/nonzero/TERM/missing-wait/missing-child semantics.

## D. Deterministic C++ sim-clock stress test (flake fixed)

**Root cause (test bug, not a contract bug):** the stress test's reader loop
began reading the shm immediately after spawning writer threads, but the shm
still held the hook round-trip test's final snapshot `{mono=9000, sim=0.018,
seq=16}` — a perfectly VALID, consistent seqlock snapshot from the PREVIOUS test
phase. It tripped the stress invariant `sim == mono*1e-9` (0.018 ≠ 9000×1e-9).
No cross-field torn read occurred; the seqlock and mutex are correct.

**Fix (test only, no relaxation):**
- skip snapshots whose `monotonic_ns < baseline` (the pre-stress phase), since
  they are consistent-but-not-stress-phase;
- tighten the torn-pair assertion from `fabs > 1e-6` to **exact inequality**
  (`sim_time != mono*1e-9`), so ANY torn pair — including adjacent-publish tears
  — is caught.

**50-run proof (after fix, final build):** `p1_08_sim_clock_test` 60 direct
runs → **0/60 failed**; `ctest -R p1_08_sim_clock_test` 50 runs → **0/50 failed**
(total **45 checks** each, 0 failure).

## E. Test matrix

| Test | Result |
|---|---|
| C++ `p1_08_sim_clock_test` (CTest) | **PASS (45)**; ctest 1/1 |
| `test_p1_08_sim_clock.py` (reader + C++→Python bridge) | **PASS (32)** |
| `test_p1_08_baseline_identity.py` (v2 identity) | **PASS (21)** |
| `test_p1_08_harness.py` (closure/ldd/lock/facts/record) | **PASS (40)** |
| `py_compile` all scripts / JSON validity / `git diff --check` | **OK / OK / PASS** |

## Old-capture boundary

The 2026-09-01 v1 capture (`capture_20260901_rerun`) and its identities
(`bdd47a0d…`, `99b995b0…`, `9840462e…`) remain **superseded / non-acceptance**.
The 2026-09-01 v2 preflight failure
(`P1-08_v2_capture_preflight_fail_20260901.md`) remains an archived single
failure. **No new v2 recapture was performed** in this increment.

Phase 1 remains **NOT ACCEPTED**; P1-10 not started; P1-08 not accepted.
