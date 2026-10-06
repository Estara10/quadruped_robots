# P1-06 — Independent Reviewer Disposition (2026-08-31)

Decision: **REJECT** (then documentation correction applied; P1-06 remains
`IMPLEMENTED / AWAITING INDEPENDENT REVIEW` — not self-accepted).

## Exact rejection reason

The Reviewer rejected the original P1-06 evidence because it **conflated three
distinct comparisons**:

1. paper parity;
2. recovered-testbed ↔ deployment parity;
3. items that are absent / unspecified in the paper.

This is a **classification overclaim** (paper-undefined constants, clipping,
feasibility, fallback, and full candidate-twist input construction were
described as paper `MATCH`). It is **not** a discovery of an algorithm
mismatch. No algorithm/state-machine/solver/code difference was newly found by
the rejection; the numeric fixture's Eq.22 and gradient-clip differences were
already recorded.

## Reviewer-confirmed boundaries

- **Source integrity is PARTIAL**, not whole-file identity: the audited
  `testbed.py` lines (55-72, 337-348) match between `ABS/` and `ABS_fuwuqi/ABS/`,
  but no claim is made that the entire files are byte-identical.
- The **14/14 fixture result proves independent arithmetic only** — it does not
  execute `StateRL` or testbed and is not runtime parity.
- **No runtime parity claim** is made; no MuJoCo/ROS2/benchmark/formal run was
  performed.

## Correction applied

Every Eq.21 verdict is now **two-dimensional**:

- `paper relationship`
- `testbed ↔ deployment relationship`

and paper-undefined items (objective constants λ/ε/coeff, learning rate,
gradient-clip method, output-clamp type, feasibility check, infeasible
fallback, full candidate-twist → 19-D RA concatenation) are marked paper
**UNKNOWN**, never paper `MATCH`. The following remain **MISMATCH** (recorded,
not fixed):

- Eq.22 deployment omission of both yaw-coupled second-order terms (vs paper +
  reference).
- First-order goal-penalty consequence (deployment `pos=vx·tau`/`vy·tau`).
- Gradient clip: reference L2-norm vs deployment per-element clamp (not an
  approved stabilized variant — no decision record).
- Iteration count: testbed 10 (not paper MATCH, exceeds paper ≤5) vs
  deployment 3 (upper-bound MATCH only); testbed↔deployment MISMATCH.

Paper **MATCH** is now limited to items the paper directly specifies: twist
variables, goal-deviation concept, gradient descent, current-twist
initialization, ≤5-iteration upper bound (deployment only), and twist bounds
`±[1.5, 0.3, 3.0]` (ABS_PAPER_NOTES:110-114).

## Status

- P1-06: **IMPLEMENTED / AWAITING INDEPENDENT REVIEW** after correction.
- No active engineering task after closure; Phase 1 remains **NOT ACCEPTED**.
- P1-07/P1-08 must not start automatically.

Evidence: `P1-06_recovery_optimizer_parity.md`,
`P1-06_recovery_optimizer_matrix.json`, `scripts/test_p1_06_recovery_optimizer.py`.

## Re-review disposition — 2026-08-31 (final plan consistency)

A second review returned **REJECT** for a documentation-consistency reason only:

- `docs/exec-plans/P1-06.md` ("Key findings", Eq.21 bullet) still retained a
  **single-dimensional** statement: "objective structure/constants … bounds,
  init, lr, RA-obs construction MATCH", without separating the `paper
  relationship` from the `testbed ↔ deployment relationship`.
- This is **not** an algorithm mismatch requiring a fix. No code, test, model,
  config, optimizer, threshold, or evidence-matrix fact was changed.

The exec-plan Eq.21 bullet was corrected to match the evidence `.md` and matrix
`.json` two-dimensional verdicts verbatim in meaning:

- Objective constants / RA penalty (λ, ε, coeff) / learning rate / candidate
  twist → 19-D RA input: paper **UNKNOWN**; testbed↔deployment **MATCH**.
- Initialization: paper **MATCH** (paper: "from the current twist",
  ABS_PAPER_NOTES:120); testbed↔deployment **MATCH**.
- Twist bounds ±[1.5,0.3,3.0]: paper **MATCH** (ABS_PAPER_NOTES:108-114);
  testbed↔deployment **MATCH**.
- Output clamp type / feasibility / fallback: paper **UNKNOWN**;
  testbed↔deployment **MATCH** (observed behavior only; not paper MATCH).
- Gradient clip: paper **UNKNOWN**; testbed↔deployment **MISMATCH**.
- Iterations: testbed 10 not paper MATCH; deployment 3 upper-bound MATCH only;
  testbed↔deployment **MISMATCH**.

The P1-06 algorithm MISMATCH / UNKNOWN conclusions are unchanged. Status remains
**IMPLEMENTED / AWAITING INDEPENDENT REVIEW**; not self-accepted. Phase 1
remains NOT ACCEPTED. P1-07/P1-08 must not start automatically.
