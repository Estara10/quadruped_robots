# Metrics and Acceptance Targets

Metric definitions are frozen before a formal run. A run that violates `EXPERIMENT_PROTOCOL.md` is INVALID and excluded from all rates.

## Metric Definitions

| Metric | Definition |
|---|---|
| Success | Robot reaches the configured goal region, remains upright and collision-free for the configured arrival hold, with valid telemetry throughout. |
| Collision | Robot collision geometry contacts a scenario obstacle under the registered contact definition. Count events on contact false→true; record object, duration and force when available. |
| Fall | Registered fall condition, reported separately for base contact, base height and roll/pitch criteria. |
| Timeout | A valid episode reaches its time limit without Success, Collision or Fall. |
| Average Speed | Time integral of registered base planar speed divided by valid locomotion duration. State frame and sampling rate. |
| Peak Speed | Maximum registered planar speed after filtering rules fixed before the run. |
| Path Length | Sum of planar trajectory segment lengths over fresh simulation-time poses. |
| Path Efficiency | Straight-line start-to-goal distance divided by path length; report only for valid arrivals. |
| Recovery Duration | Time from structured Recovery ENTER to EXIT; report each event plus total/max per episode. |
| Switch Count | Number of structured Agile↔Recovery state edges, not text-log matches. |
| Minimum Obstacle Distance | Minimum valid clearance under the registered ray/contact geometry; never substitute it for collision. |
| RA Behavior | RA timeline, threshold crossings, entry/exit value, time-to-collision relation, calibration/discrimination and constraint margin after Recovery twist optimization. |

## Paper Reference

ABS nominal simulation evaluation used three independently trained policy seeds and 10,000 random episodes per seed.

| Metric | Paper value |
|---|---:|
| Success | 79.1 ± 4.4% |
| Collision | 5.7 ± 2.9% |
| Timeout | 15.2 ± 2.1% |
| Peak speed | 3.48 ± 0.06 m/s |
| Average speed | 2.08 ± 0.01 m/s |

These Go1/original-simulator values are references, not automatic Go2 graduation gates.

## Graduation Project Acceptance

### Phase 1 initial preregistration basis

- At least 30 fixed seeds per formal scenario for the engineering gate; report 95% Wilson/bootstrap intervals.
- Stable thesis subsets should expand to 100 seeds per scenario when practical.
- Full ABS, Agile-only, paper-faithful and stabilized comparisons use the same seeds.
- Formal-suite Success point estimate: at least the paper nominal 79.1% as the initial reference gate.
- Collision point estimate: no more than 8.6%, the paper nominal plus one reported standard deviation, as the initial reference gate.
- Full ABS must reduce paired collision-or-fall rate versus Agile-only with a 95% paired bootstrap interval not crossing zero.
- Basic flat/static-obstacle S0–S5 suite has no base-contact fall.
- Any threshold adjustment after the pilot must be preregistered before the formal run and justified in the Acceptance report.

### Phase 2 initial gate

- All required software fault-injection cases pass.
- Speed, angular-rate and acceleration limits come from P2-01 hazard analysis and measured stopping/latency data; no paper-speed requirement is imposed.
- Each accepted physical obstacle stage uses at least 10 trials, matching the paper's real-test sample count.
- Initial static-obstacle gate: at least 9/10 task success, zero falls and zero hard collisions.

## Final Reproduction Target

- Three independent training seeds and 10,000 evaluation episodes per seed on the paper-equivalent simulation suite.
- Match or statistically justify deviation from the paper Success/Collision/Timeout and speed ranges.
- Report Go2 MuJoCo and Go2 real results separately from the paper-equivalent benchmark.
- Increase speed only after correctness, stability, observability and safety gates remain satisfied.
