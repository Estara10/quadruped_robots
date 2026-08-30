# P1-01 Scope Alignment Independent Review — 2026-08-30

## Decision

**ACCEPT WITH KNOWN ISSUES.** This review accepts P1-01 under the
Director-approved DEC-010 boundary: current deployed mapping correctness and
simulation operational validity, rather than forensic reconstruction of the
historical training environment.

## Accepted evidence

- Agile `model_4000.pt` → export → deployed identity is verified by 8/8 exact
  actor-tensor equality and export/deployed byte equality.
- Named RA → JIT → deployed identity is verified by 6/6 exact parameter equality
  and JIT/deployed byte equality.
- Recovery `model_15000.pt` → export → deployed identity is verified by 8/8
  exact actor-tensor equality and export/deployed byte equality.
- The declared policy order `FL,FR,RL,RR` maps through the documented action
  remap `[3,4,5,0,1,2,9,10,11,6,7,8]` and contact remap `[1,0,3,2]` to the
  controller/MuJoCo order `FR,FL,RR,RL`. Existing asymmetric 61/19/49 contract
  and remap tests support this operational mapping.
- P1-09 independently demonstrated the simulation-only runtime chain through
  MuJoCo, StateRL, runtime recording, and a P1-02 validator verdict.

## Boundary and known issues

- RA training on `model_4000.pt` is **OPERATOR_DECLARED**, not immutable
  historical execution proof.
- Historical config, seed, command, Git revision, export invocation, and raw RA
  dataset evidence are deferred reproducibility, not P1-01 blockers.
- Real Go2 `foot_force[0..3]` slot semantics remain Phase 2 hardware-only.
- This decision makes no real-robot performance, safety, benchmark, or Phase 1
  Acceptance claim.

## Independent checks

- `conda run -n abs python scripts/validate_p1_01_contract.py`: `pass=135`,
  `known=8`, `fail=0`; `P1-01 CONTRACT REGRESSION: PASS`. Its exit code `2` and
  embedded `P1-01 ACCEPTANCE: BLOCKED` summary are pre-DEC-010 validator logic,
  not a contract regression.
- `git diff --check`: pass.

Phase 1 remains **NOT ACCEPTED**.
