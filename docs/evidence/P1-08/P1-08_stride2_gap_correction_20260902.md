# P1-08 — v2 Sequence-Gap Correction and Orphan-Claim Reconciliation (2026-09-02)

> **Historical pre-final-review record** (2026-09-02, before the Final Independent Review). Superseded by [`REVIEW_2026-09-02_FINAL.md`](REVIEW_2026-09-02_FINAL.md) — P1-08 ACCEPT WITH KNOWN ISSUES.

Status: **P1-08 — IMPLEMENTED / AWAITING INDEPENDENT REVIEW** (not accepted).
Phase 1 remains **NOT ACCEPTED**.

Offline, traceable correction of the 2026-09-02 v2 capture's derived statistics
and doc wording. **No MuJoCo/ROS2 runtime recapture was run.** No model /
controller / sim-clock runtime contract / scene / algorithm / parameter change.

## 1. Preserved raw files and hashes (UNCHANGED)

| raw | SHA-256 (16) |
|---|---|
| sim_clock_timing.jsonl | d7568d6824305c0e |
| rt_frame_timing.jsonl | 89ff99379203e97b |
| runtime_record.jsonl | 28f1a0fdc714c80c |
| process_facts.json | 1b8b34988381f54d |
| orchestrator_raw.log | ece3fab3c3a6d8cd |
| mujoco_raw.log | a6aec6978f1b518a |
| ros2_launch_raw.log | 35a7b34b3de91a31 |

None were modified; all hashes re-verified identical.

## 2. Old stride-1 artifact → corrected stride-2 artifact

- Original `reader_stats.json` (`d3b0d28bb11f8fcb`, 356 B) preserved as
  `reader_stats_pre_stride2_correction.json` with its SHA + reason (wrong
  stride-1 formula: `seq_gaps=12512` under a v2 stride-2 even sequence).
- Corrected `reader_stats.json` (`18734b5485ff8317`, schema
  `abs-go2-p1-08-reader-stats/v2`, generator `build_p1_08_corrected_reader_stats.py`
  v1.0) records `sequence_stride=2`, distinct accepted 12 489,
  `seq_gaps_total_missing=12`, `seq_gap_max_single=10`, gap_errors [],
  rejected=0, raw-timing source SHAs, and the supersedes provenance.

## 3. Exact corrected gap math / results

`missing = (next - prev) / 2 - 1` over distinct strictly-increasing even
sequences from the unmodified raw file:

- distinct accepted physics sequences: 12 489
- advancing pairs: 12 488
- **total real missing publishes: 12**
- **max single gap: 10**
- non-even / rollback / duplicate → fail-closed (gap_errors empty for this run)
- cross-gap timing normalization: `timing_stats.json` divides each
  wall-clock/sim-time delta by the actual steps in the pair (duplicate rows
  have zero sim advance and are skipped), so periods are gap-normalized.
- Sequence gaps (missing publishes) are explicitly distinct from reader
  rejected reads (rejected=0 for both streams).

Independent raw reanalysis reproduces total=12, max=10 — matches the Reviewer
facts exactly.

## 4. Regenerated derived reports and new identity hash

- `timing_stats.json` regenerated (raw-driven, deterministic).
- Canonical identity v2 regenerated to bind the corrected `reader_stats.json`:
  **`59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0`**
  (schema `abs-go2-p1-08-baseline-identity/v2`, generator 2.0; all 8 required
  capture inputs + manifest + asset/binary/config hashes + git + generator bound).
  Recomputed from the saved canonical input → identical. Only the
  `reader_stats_sha256` input changed (raw-input hashes unchanged).
- Old identity `14e8d14f…` preserved and marked **superseded** (it bound the
  stride-1 reader stats). Identity contract/schema unchanged (the corrected
  reader_stats is still the `reader_stats.json` input).

## 5. Orphan-claim wording correction

- This capture archived **no** independent post-run orphan inventory artifact, so
  the claim is now recorded as:
  **`post-run orphan inventory: UNKNOWN`** — no independent inventory artifact
  was archived at capture end.
- A later check saw no related processes, but that is **not capture-end proof**.
- This UNKNOWN is not auto-interpreted as a runtime failure.
- `p1_08_baseline_capture.py` now persists an `orphan_inventory.json`
  (capture-end pgrep snapshot, documented as not-an-independent-supervisor-proof)
  for FUTURE captures.

## 6. Harness future-computation fix + tests

- `sample_and_record` (live) now collects DISTINCT accepted sim-clock sequences
  and computes gaps post-window via `compute_stride2_gaps` (stride-2,
  fail-closed on non-even/rollback/duplicate). rt_frame rl_step remains stride-1
  (correct).
- `test_p1_08_harness.py` adds `test_stride2_gap_math` (consecutive stride-2 →
  0; single gap → 1; multi gap → 9; (26-4)/2-1=10; non-even / rollback /
  duplicate fail-closed) → **PASS (84 checks)**.
- Sim-clock writer/reader itself unchanged.

## 7. Test/regression results

- C++ `p1_08_sim_clock_test` **PASS (45)**; `ctest` **1/1**.
- `test_p1_08_sim_clock.py` **PASS (32)**; `test_p1_08_baseline_identity.py`
  **PASS (21)**; `test_p1_08_harness.py` **PASS (84)**.
- Corrected JSONs all parse; identity recompute matches; `py_compile` OK;
  `git diff --check` PASS.

## 8. Boundaries

- Old v1 capture (`capture_20260901_rerun`) + its identities (`bdd47a0d…`,
  `99b995b0…`, `9840462e…`) remain superseded/non-acceptance; the 2026-09-01
  preflight failure remains an archived single failure.
- P1-10 not started; P1-08 not accepted; Phase 1 NOT ACCEPTED.
