# P1-08 — Final Independent Review (2026-09-02)

Decision: **P1-08 — ACCEPT WITH KNOWN ISSUES**.
Final independent review date: **2026-09-02**.

This records the accepted Final Reviewer result for P1-08 (Freeze MuJoCo Model,
Effective Timing, and Dynamics Baseline) into the source of truth. No code,
script, capture data, identity, test, or runtime configuration was changed by
this review.

## 1. Acceptance scope

- **v2 MuJoCo model / config / artifact hash-bound baseline**: refreshed manifest
  (closure `8d9218de…`, 18 files, failures empty), binaries, plugins, deployed
  Agile/RA/Recovery artifacts, configs, git commit/dirty — all hash-verified and
  bound to the single launch identity.
- **Authoritative sim-clock / timing capture**: one controlled v2 capture
  (run_id `4f14416672244cbfb4af93573bd9d86c`, session `1970665031624`, fixed
  25 s); 0 rejected reads on both streams; raw timing files complete.
- **Observed physics and Policy/RA cadence**: physics timestep 0.002 s exactly /
  wall-clock 500 Hz; Policy and RA tick ≈49.97 Hz (rt_frame, 1250 LIVE, mean
  20.014 ms).
- **Independently reproducible baseline identity**: identity v2
  `59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0`
  (recomputed-from-input identical; all required inputs hash-bound).
- **Real runtime record, two-phase finalize, real process facts**: record
  `VALID` with 1250 LIVE; terminal unique + final, real facts (SIGINT delivered
  to both children, rc 0, no escalation, `FRAMES_ENDED_RC0`).

## 2. Retained Known Issues (only these)

- **Capture-end orphan inventory UNKNOWN**: no independent inventory artifact was
  archived at the 2026-09-02 capture end (a later check saw no related
  processes, but that is not capture-end proof).
- **Recovery cadence UNKNOWN / not observed**: Recovery was not active during the
  capture (0 transitions).
- **Direct controller-callback cadence UNKNOWN**: 5.003 ms is **DERIVED** only
  (no authoritative per-callback timestamp source).
- **Corrected reader-stats `generated_at` byte-stability boundary**: the
  corrected `reader_stats.json` (schema `abs-go2-p1-08-reader-stats/v2`) embeds a
  `generated_at` timestamp, so re-running the generator on identical inputs does
  not reproduce byte-identical output (only the gap math is deterministic).
- **No benchmark / paper-equivalence / Sim-to-Real / performance conclusion**.

## 3. Historical markers

Any earlier "v2 recapture pending / awaiting recapture" wording is a
**Historical pre-2026-09-02 capture finding** and is superseded by this
acceptance. It is not in present-tense conflict with the current status.

## 4. Explicit boundaries

- The old v1 capture (`capture_20260901_rerun`), its identities
  (`bdd47a0d…`, `99b995b0…`, `9840462e…`), the superseded identity `14e8d14f…`,
  and the 2026-09-01 preflight failure remain **superseded / non-acceptance**.
- **P1-10 does not start automatically.**
- **Phase 1 remains NOT ACCEPTED.**

## Evidence

- `P1-08_v2_baseline_capture_20260902.md` + `capture_20260901_v2/` (raw/derived)
- `P1-08_stride2_gap_correction_20260902.md`
- `P1-08_simulation_baseline_identity.json` (identity `59dd13fe…`)
- `docs/exec-plans/P1-08.md`
