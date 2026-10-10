# Undergraduate Thesis Project Entry

This repository is an undergraduate thesis project on risk-driven safe policy
switching for Unitree Go2 in complex static-obstacle environments.

## Authority and Current Planning

The passed thesis proposal is the highest authority for **what the project is
intended to do**. Repository code, configuration, and preserved evidence are
the authority for **what has actually been completed**.

Before planning or making a change, read only this current Source of Truth:

1. `AGENTS.md`
2. `docs/thesis_project/OVERVIEW.md`
3. `docs/thesis_project/ROADMAP.md`
4. `docs/thesis_project/CURRENT_STATE.md`
5. `docs/thesis_project/EXPERIMENT_PLAN.md`
6. The single active task under `docs/thesis_project/tasks/`

The active task, its scope, and its entry conditions govern the next action.
Do not infer a task from an old task ID or an unfinished historical document.

## Legacy Boundary

The following are preserved historical materials, not current instructions:

- old P1 plans in `docs/legacy/exec-plans/`;
- old P1 evidence in `docs/evidence/`;
- old top-level governance, Gate, Phase, reviewer, and acceptance documents;
- old P1 scripts, scenarios, and formalization assets unless an active thesis
  task explicitly selects one as a reusable technical asset.

Preserve historical facts, failures, risks, artifacts, and UNKNOWN items. Do
not treat a legacy `PASS`, plan, or checklist as current completion evidence.

## Working Rules

- Keep exactly one main Active Task.
- Never present PLANNED or UNKNOWN work as FACT.
- Use simulation before any real-robot work.
- Real-robot ABS/RL remains NO-GO until the current thesis safety-readiness
  conditions are met; low-speed monitored preparation is separate from formal
  real-robot validation.
- Do not train models, run formal experiments, or change control algorithms
  unless the active thesis stage authorizes it.
- Before formal experiments, verify the interfaces and semantics that can
  affect results: dimensions, order, units, frequency, thresholds, hold time,
  RA/Recovery behavior, and metrics.
- Keep code and documentation changes traceable to the thesis objective or the
  active task. Preserve useful technical evidence rather than rewriting it.

## Technical Orientation

The current executable system is centered on:

- `quadruped_ros2_control_humble/` for ROS 2 control and policy runtime;
- `unitree_mujoco/` for simulation and obstacle/ray infrastructure;
- `ABS/` for paper and training-side reference code;
- `common/` for runtime contracts shared by simulator and controller;
- `archive/artifacts/manifest.yaml` for the archived model and external-artifact inventory (historical reference; verify against current files before use).

Technical reference documents such as `docs/POLICY_IO_CONTRACT.md`,
`docs/ABS_PAPER_NOTES.md`, and `docs/REPOSITORY_BASELINE.md` may be consulted
when their subject is relevant, but they do not supersede the current Source
of Truth above.
