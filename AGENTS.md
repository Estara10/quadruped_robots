# Project

Complete the passed undergraduate thesis project on risk-driven safe policy switching for Unitree Go2 in complex static-obstacle environments.

@/home/lidio/.codex/RTK.md

## Current Source of Truth

Read only this lightweight set for current planning:

- `docs/thesis_project/OVERVIEW.md`
- `docs/thesis_project/ROADMAP.md`
- `docs/thesis_project/CURRENT_STATE.md`
- `docs/thesis_project/EXPERIMENT_PLAN.md`
- the single active file under `docs/thesis_project/tasks/`

The passed thesis proposal is the highest research authority. Code and direct repository evidence determine what is actually complete.

## Legacy Boundary

Old P1 tasks, gates, reviewers, exec plans, evidence bundles and top-level governance Markdown are `LEGACY`. Preserve them, but do not continue, repair or close them unless the current thesis task explicitly needs a reusable technical asset.

## Working Rules

- Use minimum sufficient verification: enough to trust the next step and the thesis result, not an industrial assurance programme.
- Keep only one main Active Task.
- A blocker must prevent the next technical step, invalidate the experiment, create a clear real-robot safety risk, or deviate from the passed proposal.
- Do not require nonessential hashes, manifests, validators, clean worktrees, historical provenance recovery or legacy P1 closure.
- Before formal experiments, verify only the interfaces and semantics that can change the result: dimensions, orders, units, frequency, thresholds, hold time, RA/Recovery behavior and key metrics.
- Record Git/model/config versions once when freezing the formal experiment; hash only critical models when useful.
- Simulation precedes real-robot testing. Real testing stays low-speed, monitored and `NO-GO` until basic safety readiness passes.
- Never present PLANNED or UNKNOWN work as completed fact.
- Do not train models, start formal experiments or perform real RL unless the active thesis stage authorizes it.

