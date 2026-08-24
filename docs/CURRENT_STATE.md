# Current State

## Current Phase

Phase 1 — MuJoCo Simulation Validation

## Current Task

P1-01 — Policy Artifact Provenance and Joint/Contact/Action Order Contract

Status: **BLOCKED / PARTIALLY COMPLETE — local 61/19/49 parity and P1-01F deployment corrections PASS; provenance and live fault evidence remain**

## Phase Acceptance

- Phase 1: **NOT ACCEPTED**
- Phase 2: **NO-GO**
- Phase 3: **NOT STARTED**

## Critical Blockers

- Server-side Agile/Recovery checkpoint/export and RA dataset/binding evidence is unavailable; all use original ABS training code, but deployed lineage/order remains `UNKNOWN` (**blocked by training-server availability**).
- Real Go2 `foot_force[0..3]` semantics are not independently captured.
- Isaac Gym Go2 `terminate_after_contacts_on=["base"]` currently matches no runtime body.
- Recovery solver and switching contain known paper mismatches.
- Formal experiment validity, seed control and event recording are not yet closed.

## P1-01 Evidence

- Three deployed hashes, installed bindings, executable shapes and deterministic outputs: **PASS**.
- Isaac Gym DOF/body/feet order and ROS2→motor→MuJoCo mapping: **PASS**.
- Current remap is bijective and correct for the captured training order, but correctness for the actual deployed artifacts: **UNKNOWN**.
- Production-linked asymmetric golden parity: Agile 61 / RA 19 / Recovery 49: **PASS**.
- P1-01F corrected rolling timer, contact temporal filter, nominal bias, fail-closed ray freshness and finite-value vetoes; helper-level fault tests **PASS**.
- Contract: [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md). Goal shaping remains **INTENTIONAL ENGINEERING VARIANT**; live fault injection is not yet evidence.
- Reviewer: prior **REJECT**; provenance and the remaining semantic gaps require re-review after closure.

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

Run P1-01F live ROS2/MuJoCo fault injection, then close server-dependent provenance when access returns. Do not start P1-02 or change core algorithms.
