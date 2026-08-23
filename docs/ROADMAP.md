# 12-Week Roadmap

This is the project-level plan. Only the current task receives a detailed file under `docs/exec-plans/`.

## Phase 1 — MuJoCo Simulation Validation, W1–W5

Milestone: prove the ABS mechanism and experiment system on Go2 + MuJoCo. Phase 1 ends only with a recorded Acceptance report.

| Task | Week | Objective | Dependencies |
|---|---:|---|---|
| P1-01 | W1 | Establish model provenance and joint/contact/action/observation contracts | None |
| P1-02 | W1 | Freeze the formal experiment schema, validity and provenance contract | None; parallel with P1-01 |
| P1-03 | W1 | Map paper equations and parameters to training/reference/runtime code | None; parallel with P1-01 |
| P1-04 | W2 | Prove 61-dimensional Agile observation parity | P1-01 |
| P1-05 | W2 | Prove RA label/model provenance and Agile-policy binding | P1-01, P1-03 |
| P1-06 | W2 | Validate Recovery Eq.21/Eq.22 and optimizer parity | P1-03, P1-05 |
| P1-07 | W2–W3 | Separate and test paper-faithful versus stabilized switching | P1-05, P1-06 |
| P1-08 | W2–W3 | Freeze MuJoCo model, timing and dynamics baseline | P1-01 |
| P1-09 | W3 | Complete telemetry, structured events, HUD and plots | P1-02 |
| P1-10 | W3 | Establish deterministic scenario suite and seed propagation | P1-02, P1-08 |
| P1-11 | W4 | Run pilot, estimate variance and preregister Acceptance settings | P1-04 through P1-10 |
| P1-12 | W4–W5 | Run paired multi-seed Full/Agile/variant evaluation | P1-11 |
| P1-13 | W5 | Attribute failures and issue Phase 1 Go/No-Go | P1-12 |

### Phase 1 Acceptance Gate

- no unresolved Critical input/output contract;
- deterministic, hash-bound valid runs with structured collision and switching evidence;
- paper-faithful and stabilized results reported separately;
- formal paired scenario statistics meet `METRICS.md`;
- independent Reviewer accepts P1-01 and later Critical algorithm tasks;
- Phase 1 Acceptance report explicitly says ACCEPTED or NOT ACCEPTED.

## Phase 2 — Safety-Gated Sim-to-Real, W6–W10

Milestone: demonstrate low-speed Go2 behavior through sequential safety gates. Phase 2 remains NO-GO until Phase 1 Acceptance.

| Task | Week | Objective | Dependencies |
|---|---:|---|---|
| P2-01 | W6 | Complete hazard analysis, latency budget and staged speed limits | P1-13 ACCEPTED |
| P2-02 | W6 | Make sim/real hardware mode explicit and fail-fast | P2-01 |
| P2-03 | W6–W7 | Implement the versioned Perception Adapter contract | P2-01 |
| P2-04 | W7 | Calibrate and benchmark the real depth-camera pipeline | P2-03 |
| P2-05 | W6–W8 | Implement independent Safety Supervisor and final command veto | P2-01 |
| P2-06 | W8 | Complete software dry-run/HIL and fault injection | P2-02 through P2-05 |
| P2-07 | W8–W9 | Commission PASSIVE, lifted joints and standing | P2-06 |
| P2-08 | W9 | Validate low-speed locomotion and Agile Policy | P2-07 |
| P2-09 | W9–W10 | Validate Recovery, static obstacle and staged multi-obstacle trials | P2-04, P2-08 |
| P2-10 | W10 | Issue Phase 2 Acceptance and Sim-to-Real gap report | P2-09 |

### Phase 2 Acceptance Gate

- safety and sensor faults fail closed in every controller state;
- independent emergency stop and supervisor are validated;
- staged trials follow Simulation → dry-run → standing → low-speed → Agile → Recovery → obstacles;
- no gate is bypassed to improve performance;
- Phase 2 report has zero unresolved Critical safety hazard.

## Phase 3 — Comparison and Graduation Package, W11–W12

Milestone: produce an evidence-linked comparison and reproducible graduation-project experiment package.

| Task | Week | Objective | Dependencies |
|---|---:|---|---|
| P3-01 | W11 | Benchmark Paper vs Go2 MuJoCo vs Go2 real using one metric schema | P1-13, P2-10 |
| P3-02 | W11 | Attribute residual gaps to robot, dynamics, perception or policy | P3-01 |
| P3-03 | W11–W12 | Decide whether DR, fine-tuning or retraining is justified | P3-02 |
| P3-04 | W12 | Freeze thesis figures, reports, manifests and reproduction package | P3-01 through P3-03 |

### Phase 3 Acceptance Gate

- every reported conclusion traces to a valid run and raw data;
- platform differences and UNKNOWNs remain explicit;
- any training change has an evidence-based hypothesis and stopping condition;
- the final package can be reconstructed from a clean checkout plus declared external artifacts.

## Rolling Planning

The only detailed current plan is `docs/exec-plans/P1-01.md`. Create P1-02 only when it becomes the next executable task and P1-01 evidence has changed the planning assumptions. Do not pre-create future exec plans.

## Roles

- Lead Agent: implementation, integration and project progression.
- Reviewer Agent: independent paper/code/evidence review after P1-01 and every Critical algorithm task.
- Evaluator Agent: benchmark, statistics and Acceptance analysis once the experiment system is stable, primarily from P1-09/P1-11 onward.
