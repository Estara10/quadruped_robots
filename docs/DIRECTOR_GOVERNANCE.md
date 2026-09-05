# Director Governance — Proportional Rigor and Scope Control

## Purpose

Phase 1 is to establish a credible ABS-Go2 research and graduation-project
evidence loop on Go2 + MuJoCo. It is not a safety-certification programme or an
industrial, high-assurance runtime-infrastructure project.

The governing standard is:

> sufficiently rigorous for credible research and graduation-project evidence

Correctness and evidence integrity remain strict. Infrastructure hardening must
remain proportional to the actual project risk.

## Director blocker test

A newly reported issue may become a blocker for the active task only if not
fixing it could directly make at least one of these false or unreliable:

1. The current algorithm or interface semantics are correct.
2. Actual runtime data is real and interpreted correctly.
3. The accepted baseline's environment, model, and configuration are reliably
   bound to the run.
4. Episode boundaries, terminal/process facts, or a formal artifact correspond
   to the actual run.
5. The P1-02 validator or current Phase acceptance could reach an incorrect
   conclusion.
6. A fixed baseline cannot support reasonable repeat experiments.

Before accepting a Reviewer finding as a blocker, the Director must state the
direct mapping to one or more of the above. A finding from a Reviewer is not by
itself a gate blocker.

## Default classification

If a finding has no direct mapping to the blocker test, classify it as one of:

- `FOLLOW-UP DEFECT`
- `KNOWN ISSUE`
- `FUTURE IMPROVEMENT`

Do not implement it in the active task. If the mapping is genuinely uncertain,
only a minimal investigation is allowed; it must not become a broad hardening
programme by default.

## Explicit non-blockers unless direct impact is shown

Do not block Phase 1 solely because of:

- a theoretical or very low-probability TOCTOU race;
- a complex concurrency case absent from the controlled single-machine
  experiment path;
- process-supervisor features intended for future industrial deployment;
- extra exception recovery that cannot affect the formal capture result;
- broad end-to-end test infrastructure added only for coverage;
- a pressure-test weakness that cannot affect accepted baseline semantics;
- defensive refactoring unrelated to current Acceptance;
- opportunistic optimisation, cleanup, or future-facing schema work.

These items may be recorded for later work, but must not silently expand the
current gate.

## Reviewer and Director boundary

Reviewers may report every real defect and must preserve `UNKNOWN`. The Director
independently decides whether the defect directly affects the active task's
Acceptance. Neither role may use edge cases to turn Phase 1 into an endless
infrastructure-hardening effort.

## Required Phase 1 rigor

Phase 1 requires:

- real facts and non-fabricated records;
- correct semantics;
- traceable accepted baselines;
- auditable formal experiments;
- fail-closed handling for critical failures.

Phase 1 does not require:

- eliminating every theoretical race;
- 100% coverage of every exceptional path;
- industrial safety certification for all components;
- complex infrastructure for deployment scenarios that are not in scope.

## Scope-control question

For every new Reviewer finding, answer before assigning work:

> If this is not fixed now, could it make a Phase 1 experiment's authenticity,
> correctness, reproducibility, or Acceptance conclusion wrong?

- **YES:** it may be an active-task blocker, with the direct mapping recorded.
- **NO:** defer it as a follow-up, known issue, or future improvement.
- **UNKNOWN:** perform only the smallest investigation needed to classify it.

## Behavioral Validation Priority and Infrastructure Stop Rule

Phase 1's final objective is not to keep expanding experiment infrastructure.
It is to establish that ABS on Go2 + MuJoCo is structurally correct, behaves
reasonably, and has credible, repeatable, interpretable evidence across
multiple obstacle scenarios.

Infrastructure exists only to support credible behavioral experiments. Once it
is sufficient for that purpose, infrastructure expansion must stop and
engineering effort must move to algorithm behavior and scenario validation.

Do not promote additional hardening, boundary tests, exceptional-path coverage,
or infrastructure work to a current gate blocker unless it could directly make
the experiment result distorted, the configuration unbound, the data
uninterpretable, the formal artifact incorrect, or the accepted baseline
unrepeatable.

### P1-08 / P1-10 direction

A flat-ground replay is only a minimal infrastructure and repeatability check.
It is not formal proof that the ABS algorithm is effective.

After the flat replay is working, use this order:

1. Freeze the first obstacle map.
2. Complete one real obstacle-scenario run.
3. Confirm that ray, RA, Agile-to-Recovery switching, collision, and terminal
   behavior are real and usable.
4. Extend the remaining obstacle maps into a formal scenario suite.
5. Run P1-11 pilot work.
6. Run P1-12 multi-seed evaluation.

Do not remain in replay-harness hardening after the flat replay passes merely
for low-value defensive improvements.

### Scenario requirement

P1-10 must not finish with only flat scenarios and then advance directly to
P1-11 or P1-12. Formal Phase 1 behavior evidence must cover multiple obstacle
scenarios. Each needs, in proportion to the research goal:

- a scenario definition;
- bound initial state and goal;
- XML/asset closure binding;
- collision and terminal authority;
- runtime record; and
- repeatable execution.

These requirements do not impose industrial-safety-certification or formal
verification standards on every map.

### Director review rule

For every infrastructure finding, the Director must answer:

> If this is not fixed now, will it materially affect the authenticity,
> correctness, interpretability, repeatability, or formal Acceptance of ABS
> behavioral experiments?

- **YES:** it may be a current blocker.
- **NO:** classify it as `FOLLOW-UP DEFECT`, `KNOWN ISSUE`, or `FUTURE
  IMPROVEMENT`.
- **UNKNOWN:** conduct only the smallest investigation required to classify it.

A Reviewer finding is not automatically a current blocker.

### Behavioral evidence priority

Once minimum credible infrastructure exists, prioritize real behavioral
evidence:

- ray behavior while approaching obstacles;
- RA risk response;
- Agile and Recovery switching under meaningful conditions;
- actual Recovery behavior;
- collision, fall, timeout, and arrival outcomes;
- measurable Full ABS versus Agile-only differences;
- reasonable same-scenario repeatability; and
- multi-scenario, multi-seed support for the final Phase 1 conclusion.

Phase 1 must not become an experiment-infrastructure project. Its Acceptance
focus remains real ABS behavioral validation and multi-scenario results,
supported by credible infrastructure.
