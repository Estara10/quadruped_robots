# Project

Reproduce ABS on Unitree Go2 + MuJoCo, then build a safe, measurable Sim-to-Real and graduation-project experiment system.

@/home/lidio/.codex/RTK.md

## Source of Truth

- Current project state: `docs/CURRENT_STATE.md`
- Overall roadmap: `docs/ROADMAP.md`
- Known gaps: `docs/GAP_MATRIX.md`
- ABS math/spec: `docs/ABS_PAPER_NOTES.md`
- Experiment rules: `docs/EXPERIMENT_PROTOCOL.md`
- Metrics and gates: `docs/METRICS.md`
- Architecture decisions: `docs/DECISIONS.md`
- Repository/artifact baseline: `docs/REPOSITORY_BASELINE.md`
- Current task: `docs/exec-plans/<TASK-ID>.md`

## Rules

- Priority: Correctness > Stability > Observability > Safety > Performance > Paper Speed.
- Never declare algorithm correctness from visual MuJoCo behavior alone.
- Preserve `UNKNOWN`; do not replace missing evidence with assumptions.
- Keep `paper-faithful` and `stabilized` implementations and results separate.
- Formal experiments must satisfy `docs/EXPERIMENT_PROTOCOL.md`.
- Read `CURRENT_STATE.md` and the current exec plan before changing code.
- Update `CURRENT_STATE.md` after every formal task; update `DECISIONS.md` only for an actual decision.
- Critical algorithm tasks require an independent Reviewer before the next gate.
- Do not advance a gate without its recorded Acceptance evidence.
- Real-robot safety overrides policy performance. Phase 2 ABS/RL is NO-GO until the documented gate changes.
- Controller/hardware order is FR, FL, RR, RL; policy order is currently `UNKNOWN` and must not be assumed before P1-01.
