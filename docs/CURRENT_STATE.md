# Current State

## Current Phase

Phase 1 — MuJoCo Simulation Validation

## Current Task

P1-01 — Policy Artifact Provenance and Joint/Contact/Action Order Contract

Status: **BLOCKED — current evidence captured; independent Reviewer REJECTED Acceptance**

## Phase Acceptance

- Phase 1: **NOT ACCEPTED**
- Phase 2: **NO-GO**
- Phase 3: **NOT STARTED**

## Critical Blockers

- Agile and Recovery source checkpoints/exports are unavailable, so their deployed joint/contact/action order remains `UNKNOWN`.
- RA source model, dataset and exact binding to the deployed Agile Policy remain `UNKNOWN`.
- Real Go2 `foot_force[0..3]` semantics are not independently captured.
- Isaac Gym Go2 `terminate_after_contacts_on=["base"]` currently matches no runtime body.
- Recovery solver and switching contain known paper mismatches.
- Formal experiment validity, seed control and event recording are not yet closed.

## P1-01 Evidence

- Three deployed hashes, installed bindings, executable shapes and deterministic outputs: **PASS**.
- Isaac Gym DOF/body/feet order and ROS2→motor→MuJoCo mapping: **PASS**.
- Current remap is bijective and correct for the captured training order, but correctness for the actual deployed artifacts: **UNKNOWN**.
- Contract: [`POLICY_IO_CONTRACT.md`](POLICY_IO_CONTRACT.md).
- Reviewer: **REJECT**; implementation-level golden parity and artifact provenance are not closed.

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

Resolve P1-01 provenance blockers and obtain Reviewer acceptance. Do not start P1-02 or change core algorithms.
