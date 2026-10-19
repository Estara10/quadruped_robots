# Project Goal（LEGACY / SUPERSEDED）

> 本文件保留 2026-10-06 前的三阶段目标，供历史追溯；不再定义当前项目范围、阶段或验收。当前目标与轻量治理见 `thesis_project/OVERVIEW.md`。

## Three-Phase Programme

### Phase 1 — System Simulation

Platform: Unitree Go2 + MuJoCo.

Phase 1 must establish system-level results, not merely implementation
completeness:

1. **Locomotion:** stable, fast locomotion in MuJoCo.
2. **Obstacle navigation:** fast progress to a bound goal in formal obstacle
   scenes, with arrival, collision, fall, timeout and elapsed time recorded.
3. **ABS strategy usage:** real runtime evidence that Agile, RA and Recovery
   are used; code presence, model loading and source-level call paths are not
   substitutes.
4. **Switching behavior:** structured evidence of Agile/Recovery transitions,
   their reason, RA state, Recovery duration and recovery outcome.
5. **Runtime observability:** a real-time simulation HUD that exposes enough
   state to understand the run, including speed, active strategy, RA, elapsed
   time, goal/trajectory state and necessary collision/safety state.
6. **Post-run results:** trustworthy arrival, elapsed-time, mean/peak-speed,
   collision, fall, timeout, RA, Recovery and necessary trajectory/safety
   statistics.
7. **Correctness and provenance:** results originate in the real MuJoCo and
   controller runtime, never from mocks, synthetic fallback, default-filled
   facts, log-text inference or filename-based provenance.
8. **Reproducibility and auditability:** fixed scenes and settings support
   reasonable repeat runs, and raw records, configuration, scene, model and
   conclusions remain reviewable.

Phase 1 does not require real-robot execution, a new algorithm, extensions
beyond the paper, extreme performance, industrial-purpose general
infrastructure, or default bitwise/value equality for every floating-point
sample.

### Phase 2 — Real-Robot Validation

Phase 2 begins only after the Phase 1 Gate passes. It covers conservative,
stepwise Unitree Go2 testing, Sim-to-Real validation, real sensors and control,
independent safety mechanisms, and real obstacle/goal tasks. Until then, real
ABS/RL is **NO-GO**.

### Phase 3 — Extension and Research

Phase 3 covers justified performance and speed improvements, new algorithms,
strategies or perception, additional scenarios, paper/method comparisons and
other research extensions. These must not be pulled forward as Phase 1
blockers.

## Graduation Project Goal

Reproduce the core ABS structure on Unitree Go2 and establish a correct, stable, observable, measurable and reproducible MuJoCo experiment system, followed by a safety-gated low-speed Sim-to-Real validation.

The graduation project prioritizes:

- correct Agile, RA, switching and Recovery data flow;
- complete Go2 + MuJoCo simulation validation;
- structured telemetry, metrics and repeatable experiments;
- explicit failure analysis and explainable conclusions;
- independent real-robot safety supervision;
- conservative, low-speed Sim-to-Real progression.

Reaching the paper's maximum speed is not an early-stage requirement.

The priority order is **Correctness > Stability > Observability > Safety >
Performance > Paper Speed**. Its purpose is to make Phase 1 behavioral results
credible. It must not be interpreted as requiring every thread, schema,
exception boundary or hypothetical future risk to be solved before real
behavioral experiments. Supporting infrastructure remains subordinate to
CORE behavior evidence.

## Final Reproduction Goal

After Phase 1 and Phase 2 acceptance, compare the paper, Go2 MuJoCo and Go2 real robot under explicitly separated protocols. Then decide whether domain randomization, system identification, fine-tuning or retraining is justified before attempting paper-level performance.

## Platform Differences

| Dimension | ABS paper | Current project |
|---|---|---|
| Robot | Unitree Go1 | Unitree Go2 |
| Simulation | Original Isaac Gym/PhysX setup | MuJoCo deployment validation; Isaac Gym remains the training reference |
| Middleware | Original ROS1 deployment | ROS 2 Humble + ros2_control + Unitree SDK2/DDS |
| Perception | ZED Mini / paper Ray-Pred pipeline | MuJoCo geometric rays plus separate ZED/D435i prototypes |
| Objective | High-speed agile-but-safe locomotion | Correctness and evidence first; speed last |

Every result must state which platform, perception source, policy artifact and experiment protocol produced it. Cross-platform numbers are comparisons, not automatic equivalence claims.
