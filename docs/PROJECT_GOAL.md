# Project Goal

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
