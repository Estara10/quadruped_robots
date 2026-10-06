# P1-08 — Recapture Harness Fail-Closed and Identity-v2 Closure (2026-09-01)

Status: **REPAIRED / AWAITING INDEPENDENT REVIEW**.
Phase 1 remains **NOT ACCEPTED**.

Scope: closes the Recapture Harness REJECT blockers (child-env preflight /
launch identity binding, narrow shm cleanup, fail-closed runtime record,
two-phase exit facts + try/finally cleanup, fixed 25 s window, canonical
identity v2 with required inputs, offline test matrix). **No MuJoCo/ROS2 runtime
capture was run**; no P1-10; no simulator/controller/policy/RA/Recovery/scene/
model/solver/timestep/gains change; no parameter tuning; old v1 capture /
identity not upgraded.

## A. Child-environment preflight and launch identity binding

- `ldd` now runs under the **exact `child_env()` object** reused for the ROS
  launch; `run_ldd` returns an evidence dict (command, `LD_LIBRARY_PATH`
  summary, stdout/stderr, returncode, exception, not-found list) that is
  archived in `<out>/preflight_evidence.json` (success) or
  `<out>_preflight_fail.json` (failure). Nonzero returncode, timeout,
  exception, or any `not found` → **PRECHECK FAIL**. No manual-shell or
  different-env substitute.
- **Launch identity binding** (`verify_manifest_hashes`):
  - the **actual `args.mujoco_bin`** hash must equal the manifest
    `mujoco_executable` hash (not a fixed manifest first path);
  - `--scene` must resolve (by the MuJoCo rule
    `exe_dir.parent.parent/unitree_robots/go2/<scene>`) to the manifest
    canonical `model_closure.root_xml`, its hash must match the closure, and
    any path escape (`/`, `..`, non-bare filename) → **PRECHECK FAIL**;
  - every recorded binary/plugin/policy/config/closure-asset hash is
    independently recomputed and must match;
  - fresh capture-directory check retained.
- **Narrow shm cleanup**: only the task's exact named shms
  (`/dev/shm/mujoco_sim_clock`, `/dev/shm/mujoco_rt_frame`) are handled, and
  only after the residual-process check confirms no live user; an unlink
  failure → **PRECHECK FAIL** (no "log and continue"). Cleanup scope, checks
  and results are archived.

## B. Runtime record fail-closed

- Every **distinct present** shared-memory snapshot (LIVE or malformed /
  non-authoritative / STALE / SYNTHETIC) is passed **raw** to
  `RunRecordRecorder.record_snapshot()` (dedup by snapshot bytes, so a
  duplicate read of the same frame is not re-recorded); MISSING is recorded
  only as a legal gap (empty payload + availability). Present bad frames are
  **never filtered out** — `run_record` invalidates the whole record on any
  non-LIVE/non-MISSING status.
- Reader stats record rejection reasons and are consistent with record
  invalidation: a rejected rt_frame snapshot corresponds to an INVALID-status
  line in the record → the record is INVALID.

## C. Two-phase exit facts and failure cleanup

- `build_process_facts` emits a traceable top-level coordinator result:
  `exit_code == 0` **only** when both required children actually `wait()==0`;
  any nonzero/timeout/missing wait → nonzero `exit_code`, `forced_termination`
  true where appropriate, `shutdown_complete` reflects real waits;
  `shutdown_request_source == "SIGINT"` (real, never missing); per-process
  PID/PGID/wait-rc/signal timeline are preserved as `child.*`.
- All post-launch branches enter one `try/finally`: stop sampling (if started),
  real SIGINT + wait per started child, TERM escalation **only** on timeout
  with each actual signal logged, facts + raw logs saved, and the recorder
  finalized with real facts (never disappearing or fabricating); no orphans.
- Fixed window: **strictly 25.0 s**; any other value → **PRECHECK FAIL**
  (offline-tested).

## D. Canonical identity v2

- New schema `abs-go2-p1-08-baseline-identity/v2`, generator **2.0** (v1
  schema superseded / non-acceptance). Canonical serialization unchanged
  (`json.dumps(input, sort_keys=True, separators=(",",":")).encode("utf-8")`
  → SHA-256), independently reproducible.
- v2 generation **requires and hash-binds**: raw `rt_frame_timing.jsonl`,
  raw `sim_clock_timing.jsonl`, finalized `runtime_record.jsonl`,
  `process_facts.json`, `orchestrator_raw.log`, `mujoco_raw.log`,
  `ros2_launch_raw.log`, `reader_stats.json`, the manifest, model/binary/
  plugin/policy/config hashes, git commit/dirty, generator version/hash.
  Missing any required input → `FileNotFoundError`; a v2 identity is never
  generated from an incomplete capture. The old v1 capture is rejected by the
  v2 generator with the explicit missing-`runtime_record.jsonl`/
  `reader_stats.json` reason (no silent upgrade).

## E. Automated offline tests (no MuJoCo/ROS2)

| Test | Result |
|---|---|
| C++ v2 sim-clock (`p1_08_sim_clock_test`, CTest) | **PASS (45)**, ctest 1/1 |
| `test_p1_08_sim_clock.py` (reader, real C++→Python bridge) | **PASS (32)** |
| `test_p1_08_baseline_identity.py` (v2 determinism / mutation / missing-input / old-v1-reject) | **PASS (21)** |
| `test_p1_08_harness.py` (ldd normal/nonzero/not-found/timeout/exception + exact-env; binary/scene mismatch; preflight-fail→no-launch; bad-frame→record INVALID; top-level facts; cleanup ordering; 25 s window) | **PASS (29)** |

## Old-capture boundary

The 2026-09-01 v1 capture (`capture_20260901_rerun`) and identities
`bdd47a0d…`, `99b995b0…`, `9840462e…` remain **superseded / non-acceptance**.
The 2026-09-01 v2 preflight failure (`P1-08_v2_capture_preflight_fail_20260901.md`)
remains an archived single failure. **No new v2 recapture was performed** in this
increment.

Phase 1 remains **NOT ACCEPTED**; P1-10 not started; P1-08 not accepted.
