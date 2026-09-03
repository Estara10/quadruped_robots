# P1-08 — One Director-Authorized v2 Runtime Baseline Capture (2026-09-02)

> **Historical pre-final-review record** (2026-09-02, before the Final Independent Review). Superseded by [`REVIEW_2026-09-02_FINAL.md`](REVIEW_2026-09-02_FINAL.md) — P1-08 ACCEPT WITH KNOWN ISSUES.

Status: **P1-08 — IMPLEMENTED / AWAITING INDEPENDENT REVIEW** (not accepted).
Phase 1 remains **NOT ACCEPTED**.

The single Director-authorized v2 MuJoCo + ROS/controller capture was executed
**once** and completed. All raw evidence is under `capture_20260901_v2/`.

## Run identity

- run_id: `4f14416672244cbfb4af93573bd9d86c` (P1-09 `RunRecordRecorder` UUID)
- session_id: `1970665031624` (StateRL rt_frame session)
- start_wall 2026-09-02T13:07:28+0800 → end_wall 13:08:09+0800; **fixed 25.0 s**
  window (unchanged by any observation).

## Preflight (all PASS, archived `preflight_evidence.json`)

capture lock acquired; residual processes none; task shm cleanup ok; X11
reachable; hardware-plugin `ldd` under child env ok (returncode 0, no
`not found`); manifest identity binding ok (actual mujoco bin `1e9b330f…` +
scene closure + plugins + artifacts + configs all match the refreshed manifest,
failures empty); fresh dir; window=25.0 s.

## Actual launched identity (refreshed manifest, single launch identity)

- scene `scene_flat.xml` (root) → includes `go2.xml`; closure sha256
  `8d9218de…`, 18 present files, **failures empty**; assets 16, xml 2.
- binaries: mujoco_executable `1e9b330f2b6c39da…` (4 197 424 B), libmujoco
  `e35ba7f6…`, controller plugin `2b31e558…`, hardware plugin `9c56d00d…`.
- artifacts: Agile `5a87d692…` (801 726 B), RA `05c40ff7…` (32 011 B), Recovery
  `e3047a21…` (775 715 B).
- configs + git commit `d27f34d` dirty=True. `switching_mode=stabilized_switch`
  (unchanged default).

## Process/signal facts (real waits, no fabrication)

- MuJoCo pid/pgid 19077; ROS launch pid/pgid 19141.
- Both received **SIGINT delivered=true** (recorded with target pid/pgid); wait
  rc **0 / 0**; `escalated=false` (no TERM/KILL); `cleanup_error_count=0`;
  `forced_termination=false`; `shutdown_request_source=SIGINT`;
  `shutdown_complete=true`; top-level `exit_code=0`.
- **post-run orphan inventory: UNKNOWN** — no independent inventory artifact was
  archived at capture end (the harness did not yet persist an `orphan_inventory`
  for this capture; the 2026-09-02 capture predates that artifact). A later
  check saw no related processes, but that is **not capture-end proof**. Two
  inert shm objects (`/dev/shm/mujoco_sim_clock`, `/dev/shm/mujoco_rt_frame`)
  remain with no live user.

## Runtime record (two-phase, fail-closed)

- 1252 lines = 1 meta + **1250 LIVE** frame + 1 terminal (unique, final line).
- `record_validity=VALID`; `termination_reason=FRAMES_ENDED_RC0`;
  `normal_shutdown=True`; `fact_validation_errors=[]`; terminal `process_exit_code
  =0`, `forced=False`, source SIGINT, complete True.
- STOP occurred before terminal; FINALIZE used the same real process facts.

## Reader stats / timing / drops

- sim-clock: **0 rejected**, attempts/accepted 753 069 (poll reads; 12 489
  DISTINCT physics steps observed → ~500 Hz). rt_frame: **0 rejected**,
  attempts/accepted 753 069 reads; 1250 distinct rl_step.
- **True drop semantics (stride-2 corrected)**: 12 dropped physics steps total,
  max single drop 10, over 12 488 advancing pairs — the reader caught essentially
  every physics step.
- The original `reader_stats.json` (`d3b0d28b…`, 356 B) used a stride-1 formula
  (`seq_gaps=12512`), over-counting under the v2 stride-2 even seqlock; it is
  preserved as `reader_stats_pre_stride2_correction.json` and superseded by the
  corrected `reader_stats.json` (`18734b54…`, schema `abs-go2-p1-08-reader-stats/v2`,
  `sequence_stride=2`, `seq_gaps_total_missing=12`, `seq_gap_max_single=10`).
  Sequence gaps are MISSING PUBLISHES over distinct accepted sequences and are
  NOT the same concept as reader rejected reads (rejected=0 for both streams).

## Observed / derived / UNKNOWN timing

| Quantity | Value | Basis |
|---|---|---|
| Physics timestep (sim-time/step) | **0.002 s exactly** | sim_clock |
| Physics wall-clock period | mean 2.0 ms → **500 Hz** | sim_clock |
| Policy (RL-step) tick | **49.97 Hz** (mean 20.014 ms, median 19.994 ms; min 2.355, P95 21.174, P99 28.738, max 67.043 ms; 1249 periods) | rt_frame |
| RA tick | **49.97 Hz** (= policy; runRAModel per RL step) | rt_frame |
| Recovery tick | **not active** (0 active samples, 0 transitions) | rt_frame policy_state |
| Controller callback | **DERIVED** 5.003 ms (~200 Hz) under periodic-callback assumption; direct per-callback timestamps UNKNOWN | static + derived |

## Canonical identity v2

`abs-go2-p1-08-baseline-identity/v2` (generator 2.0), all 8 required capture
inputs + manifest + asset/binary/config hashes + git + generator bound.
**identity `59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0`**
(see `P1-08_simulation_baseline_identity.json`). The earlier identity `14e8d14f…` is **superseded** because it bound the original reader_stats.json (stride-1 seq_gaps=12512); the raw inputs are unchanged.

## Raw/derived file SHA-256 + bytes

| file | SHA-256 (16) | bytes |
|---|---|---|
| preflight_evidence.json | ccb8049720b314a2 | 6 145 |
| process_facts.json | 1b8b34988381f54d | 1 468 |
| reader_stats.json (corrected, stride-2) | 18734b5485ff8317 | (see file) |
| reader_stats_pre_stride2_correction.json | d3b0d28bb11f8fcb | 356 |
| timing_stats.json | 476a7f803a56c061 | 2 841 |
| rt_frame_timing.jsonl | 89ff99379203e97b | 161 426 |
| runtime_record.jsonl | 28f1a0fdc714c80c | 3 030 806 |
| sim_clock_timing.jsonl | d7568d6824305c0e | 62 140 599 |
| mujoco_raw.log | a6aec6978f1b518a | 3 851 |
| orchestrator_raw.log | ece3fab3c3a6d8cd | 768 |
| ros2_launch_raw.log | 35a7b34b3de91a31 | 22 898 |
| P1-08_baseline_manifest.json | 2667ed37a854f85e | (refreshed) |
| P1-08_simulation_baseline_identity.json | (identity JSON) | — |

## Boundary / known notes

- `sim_clock_timing.jsonl` records every poll read (753 k rows, 62 MB) — verbose
  but complete; distinct steps (12 489) are derivable and the timing stats dedup
  naturally (zero-advance rows skipped). A future capture could dedup at write.
- Recovery was not active (0 transitions) — its period is `UNKNOWN / not
  observed`.
- Not P1-08 acceptance, not benchmark, not paper/Sim-to-Real equivalence.
- Old v1 capture (`capture_20260901_rerun`) + its identities and the 2026-09-01
  preflight failure remain **superseded / non-acceptance**.
- P1-10 not started. Independent Reviewer review still required.
