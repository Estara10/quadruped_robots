# Current State

## Current Phase

Phase 1 — MuJoCo Simulation Validation

## Current Task

P1-01 — Policy Artifact Provenance and Joint/Contact/Action Order Contract

Status: **NOT STARTED**

## Phase Acceptance

- Phase 1: **NOT ACCEPTED**
- Phase 2: **NO-GO**
- Phase 3: **NOT STARTED**

## Critical Blockers

- Agile/Recovery joint, contact and action order is `UNKNOWN` for the deployed artifacts.
- RA model provenance and binding to the deployed Agile Policy is `UNKNOWN`.
- Recovery solver and switching contain known paper mismatches.
- Formal experiment validity, seed control and event recording are not yet closed.
- Exact MuJoCo dependency versions and clean-checkout build have not yet been verified.

## Current Metrics

- All existing simulation results: **LEGACY / NON-ACCEPTANCE**.
- Historical paired report: Full ABS 38/40; Agile-only 30/40. No matched seeds or true-contact metric at that time.
- Later true-contact session: 5/6 with one collision.
- Latest four-scene session: 3/4 with one terrain fall.
- The old 12/12 result is an arrival baseline, not a formal collision-free score.

## Known Dirty Changes

- User deletion of `paper.txt`.
- User path gains changed from 3.0 to 2.5.
- User real-network comment changed to `enp7s0`.
- Untracked legacy report generator with stale claims; intentionally not committed.

## Real Robot Gate

Allowed: `PASSIVE`, `FIXEDDOWN`, `FIXEDSTAND`, emergency-stop checks and software dry-run.

ABS/RL real test: **NO-GO**

## Next

P1-01. Do not start P1-02 or change core algorithms before P1-01 evidence and Reviewer completion.
