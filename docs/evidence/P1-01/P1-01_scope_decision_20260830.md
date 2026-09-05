# P1-01 — Scope Alignment and Acceptance Reconciliation (2026-08-30)

Dated evidence of a Director-approved scope decision. Documentation and status
reconciliation only — no code, policy, config, controller mapping, model,
schema, runtime recorder, MuJoCo, ROS2, or hardware was changed.

## Scope decision (explicitly approved by the Director / project owner)

P1-01's objective is **current deployment mapping correctness and simulation
operational validity**, not forensic reconstruction of the historical training
environment. Historical training reproducibility is **no longer a P1-01
Acceptance condition**.

- **Weight lineage remains independently confirmed**:
  - Agile `model_4000.pt` → export → deployed `policy.pt` (weight-equal 8/8;
    export↔deployed byte-equal SHA `5a87d6…`);
  - named RA → JIT → deployed `ra_value.pt` (weight-equal 6/6; JIT↔deployed
    byte-equal SHA `05c40f…`);
  - Recovery `model_15000.pt` → export → deployed `policy.pt` (weight-equal 8/8;
    export↔deployed byte-equal SHA `e3047a…`).
- **Owner-declared historical fact** (recorded exactly as `OPERATOR_DECLARED`,
  **not** independently immutable historical proof):
  - Agile was trained by the project owner;
  - RA was trained using Agile checkpoint `model_4000.pt`.
- This **closes the historical RA-binding requirement for P1-01 engineering
  scope**, but it does **not** convert the owner statement into immutable /
  reproducible provenance evidence.
- Historical config / seed / command / Git revision / export invocation / raw RA
  rollout dataset / episode count / shell-job logs become **deferred
  reproducibility**, **not P1-01 blockers**.
- Real Go2 `foot_force[0..3]` slot semantics become **Phase 2 hardware-only**;
  neither simulation nor documentation is claimed to prove the real DDS slot map.
- The training policy order and the controller order may differ; that difference
  is normal. The only operational requirement is that the documented remap maps
  the declared policy order into controller/MuJoCo actuator order.
- **Current operational order** (declared):
  `policy side FL, FR, RL, RR` → documented action remap
  → `controller/MuJoCo FR, FL, RR, RL`.
- **Operational evidence**: the existing asymmetric mapping/contract evidence and
  the real P1-09 simulation runtime record. Visual locomotion alone is not proof.

## P1-01 Acceptance boundary (post-reconciliation)

The Acceptance criteria now cover the current deployed engineering chain:

- deployed artifact identity / weight lineage is verified;
- declared policy-order → remap → controller/MuJoCo actuator mapping is
  documented and covered by existing asymmetric contract evidence;
- the current simulation-only runtime chain has been demonstrated by P1-09;
- unknown historical training metadata is preserved as **deferred
  reproducibility**, not silently discarded;
- real-hardware foot-force slot semantics are explicitly deferred to **Phase 2**;
- no real-robot performance or safety result is claimed.

The underlying controller remap is **unchanged**.

## Proposed status (not self-accepted)

`P1-01 — ACCEPT WITH KNOWN ISSUES — AWAITING INDEPENDENT REVIEW`

Known issues are limited to:
- historical reproducibility evidence is incomplete (deferred);
- real Go2 foot-force slot semantics await Phase 2 hardware evidence;
- no real-robot RL/ABS result is claimed.

No new blockers are added; P1-09 is not reopened; Phase 1 remains NOT ACCEPTED.

## Validator reproducibility (2026-08-30)

Reproduced with an already-available local conda environment (nothing was
installed):

- Environment activation: `source ~/anaconda3/etc/profile.d/conda.sh && conda run -n abs python ...`
- Exact validator command (from `scripts/` cwd): `conda run -n abs python validate_p1_01_contract.py`
- Python version: `3.8.20`
- PyTorch version: `2.0.1+cu118`
- Actual exit code: `2` (the validator's documented Acceptance-blocked exit
  code, not a regression failure)
- `CONTRACT REGRESSION` result: **PASS** — `SUMMARY pass=135 known=8 fail=0`
- Validator's legacy Acceptance summary (separately, explicitly noted):
  `P1-01 ACCEPTANCE: BLOCKED (known gaps remain)` — this is the validator's old
  embedded P1-01 Acceptance verdict, **pre-DEC-010**, and is **not** the current
  scope-decision status (`ACCEPT WITH KNOWN ISSUES — AWAITING INDEPENDENT
  REVIEW`).

No MuJoCo, ROS2, training, benchmark, real robot, or model execution was run.

## Cross-reference

- Scope decision recorded formally in [`DECISIONS.md`](../../DECISIONS.md) (DEC-010).
- Acceptance table updated in [`exec-plans/P1-01.md`](../../exec-plans/P1-01.md).
- Classifications updated in [`CURRENT_STATE.md`](../../CURRENT_STATE.md) and
  [`GAP_MATRIX.md`](../../GAP_MATRIX.md).
