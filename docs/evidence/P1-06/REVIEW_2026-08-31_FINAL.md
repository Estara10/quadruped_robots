# P1-06 — Final Independent Review (2026-08-31)

Decision: **P1-06 — ACCEPT WITH KNOWN ISSUES**.
Final independent review date: **2026-08-31**.

This records the accepted Final Reviewer result for P1-06 (Recovery Eq.21/Eq.22
and Safe-Twist Optimizer Parity) into the source of truth. No algorithm, test,
model, config, threshold, solver, or runtime behavior was changed.

## 1. Acceptance scope

- **Offline Eq.21 / Eq.22 source trace** across paper (`ABS_PAPER_NOTES.md`),
  recovered testbed (`ABS/` + `ABS_fuwuqi/ABS/` `testbed.py`), and deployment
  (`StateRL.cpp::computeRecoveryTwist`).
- **Three-way two-dimensional classification** (each Eq.21 verdict separates the
  `paper relationship` from the `testbed ↔ deployment relationship`; paper-
  undefined items are paper `UNKNOWN`, never paper `MATCH`).
- **Deterministic arithmetic fixture** `scripts/test_p1_06_recovery_optimizer.py`
  (14/14 PASS) — independent arithmetic only, not runtime parity.
- **Identified differences, not fixed differences** — every MISMATCH below is
  recorded and not auto-fixed.

## 2. Retained MISMATCH (recorded, not fixed)

- Deployment `StateRL.cpp:623-624` **omits the Eq.22 yaw-coupled second-order
  terms** (`x=vx·tau`, `y=vy·tau` instead of
  `x=vx·tau−0.5·vy·wz·tau²`, `y=vy·tau+0.5·vx·wz·tau²`).
- **First-order displacement changes the goal penalty** consequence (deployment
  pos-penalty uses the first-order displacement).
- **Iteration count**: recovered testbed 10 vs deployment 3
  (paper ≤5; testbed 10 not paper MATCH, deployment 3 upper-bound MATCH only).
- **Gradient clip**: recovered testbed L2-norm `_clip_grad` vs deployment
  per-element `torch::clamp(grad,-1,1)` (not an approved stabilized variant —
  no decision record).

## 3. Retained UNKNOWN

- Paper does not specify: λ/ε/goal-deviation coefficient, learning rate,
  gradient-clip method, output-clamp type, feasibility check / infeasible
  fallback, and the full candidate-twist → 19-D RA input concatenation.
- Whether the current deployment simplification (3 iterations, first-order
  displacement, per-element clip) is an **approved stabilized variant** (no
  decision record).
- **Runtime parity, feasibility rate, and final RA-constraint satisfaction
  rate** — not measured (no runtime run authorized).

## 4. Explicit boundaries

- P1-06 does **not** represent paper equivalence, runtime parity, benchmark
  results, or safety-performance conclusions.
- P1-07 / P1-08 do **not** start automatically.
- There is currently **no active engineering task** after P1-06 closure.
- **Phase 1 remains NOT ACCEPTED.**

## Evidence

- `docs/evidence/P1-06/P1-06_recovery_optimizer_parity.md`
- `docs/evidence/P1-06/P1-06_recovery_optimizer_matrix.json`
- `scripts/test_p1_06_recovery_optimizer.py`
- `docs/evidence/P1-06/REVIEW_2026-08-31_OPTIMIZER_PARITY.md` (prior REJECT
  dispositions)
