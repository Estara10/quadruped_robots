# ABS Paper Specification Notes

This file records paper definitions. Runtime deviations belong in `GAP_MATRIX.md`; they must not rewrite the paper specification.

Source: `agile but safe(1).pdf`, local copy of *Agile But Safe: Learning Collision-Free High-Speed Legged Locomotion*.

## Runtime Architecture

Paper deployment rates:

- PD control: **200 Hz**;
- Agile Policy: **50 Hz**;
- Recovery Policy: **50 Hz**;
- RA Value evaluation: **50 Hz**;
- Ray-Pred perception: **40 Hz**.

The paper switches to Recovery when estimated RA Value is at or above the threshold and returns to Agile when it is below the threshold. The reported threshold is **−0.05**.

Reference: deployment architecture figure in the paper; runtime rates correspond to [`robot_control.yaml`](../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml) and training decimation is defined in [`legged_robot_config.py`](../ABS/training/legged_gym/legged_gym/envs/base/legged_robot_config.py).

## Agile Policy

### Observation: 61 dimensions

| Component | Dimensions |
|---|---:|
| Foot contacts | 4 |
| Base angular velocity | 3 |
| Projected gravity in base frame | 3 |
| Goal command: relative x, relative y, heading | 3 |
| Time left | 1 |
| Joint position relative to default | 12 |
| Joint velocity | 12 |
| Previous action | 12 |
| Log ray distances | 11 |
| Total | 61 |

Training construction: [`legged_robot_pos.py`](../ABS/training/legged_gym/legged_gym/envs/base/legged_robot_pos.py).

### Action: 12 dimensions

The policy outputs 12 joint-position targets. The paper deployment uses a PD law of the form

```text
tau = Kp * (q_target - q) - Kd * q_dot
```

with paper real-robot gains `Kp=30`, `Kd=0.65`. Current Go2 training also uses action scale `0.25`.

## Ray Representation

- Count: **11** horizontal rays;
- body-frame angles: **−π/4 to +π/4**;
- spacing: **π/20** (9 degrees);
- distance range: **0.1–6.0 m**;
- policy representation: logarithmic distance; repository reference uses `log2`;
- training origin in repository: x = −0.05 m, y = 0.

Paper perception uses a depth image resized to 160×90 and a ResNet18-based predictor. Exact equivalence between paper ray geometry and every current MuJoCo primitive is `UNKNOWN`.

## Reach-Avoid Value

The paper defines a reach-avoid objective from a reach signal `l(s)` and an avoid/collision signal `g(s)`, then derives a discounted Bellman form (paper Eq.4–5). The learned RA network approximates the policy-conditioned value of the Agile Policy.

### RA Observation: 19 dimensions, paper Eq.14

| Component | Dimensions |
|---|---:|
| Base linear velocity | 3 |
| Base angular velocity | 3 |
| Goal relative x/y | 2 |
| Log ray distances | 11 |
| Total | 19 |

Joint position and velocity are not RA inputs.

Paper training details:

- approximately 200,000 Agile-policy episodes;
- discounted Bellman regression, paper Eq.16–17;
- `gamma = 0.999999`;
- collision target is softened over the last ten timesteps;
- reach signal is based on `tanh(log(d_goal / sigma_tight))`, paper Eq.18.

The exact dataset/checkpoint used by the current `ra_value.pt` is `UNKNOWN` and is a P1-01/P1-05 blocker.

## Recovery Policy

### Observation: 49 dimensions

| Component | Dimensions |
|---|---:|
| Foot contacts | 4 |
| Base angular velocity | 3 |
| Projected gravity | 3 |
| Safe twist command | 3 |
| Joint position | 12 |
| Joint velocity | 12 |
| Previous action | 12 |
| Total | 49 |

Recovery intentionally has no exteroceptive ray input; obstacle information affects it through the optimized safe twist.

### Action

Twelve joint-position targets, executed by PD control.

Paper Recovery training twist ranges:

- `vx`: ±1.5 m/s;
- `vy`: ±0.3 m/s;
- `wz`: ±3 rad/s.

These are training ranges, not approved Go2 real-robot speed limits.

## Safe Twist Optimization

### Eq.21

Choose a short-horizon twist that minimizes predicted goal-position deviation subject to the predicted RA Value being below the switching threshold. The paper describes gradient-based constrained optimization initialized from the current twist and completed within five iterations.

### Eq.22

For `delta_t = 0.05 s`, the paper's planar displacement approximation includes yaw coupling:

```text
delta_x = vx * delta_t - 0.5 * vy * wz * delta_t^2
delta_y = vy * delta_t + 0.5 * vx * wz * delta_t^2
```

The current ROS 2 implementation omits these second-order terms; that is a gap, not a revised specification.

## Switching Logic

Paper-faithful conceptual sequence:

```text
state + goal + rays
        ↓
     RA Value
        ↓
RA < -0.05  ──→ Agile Policy
RA >= -0.05 ──→ optimize safe twist ──→ Recovery Policy
```

The paper does not define the current project's `−0.08` exit threshold or 30-step forced hold. Those belong only to a separately reported stabilized variant until validated.

## Paper Simulation Statistics

Nominal ABS result, three policy seeds and 10,000 random evaluation episodes per seed:

| Metric | Value |
|---|---:|
| Success | 79.1 ± 4.4% |
| Collision | 5.7 ± 2.9% |
| Timeout | 15.2 ± 2.1% |
| Peak speed | 3.48 ± 0.06 m/s |
| Average speed | 2.08 ± 0.01 m/s |

Agile-only nominal reference:

- Success: 77.3%;
- Collision: 21.7%;
- Timeout: 1.0%;
- Peak speed: 3.55 m/s;
- Average speed: 2.39 m/s.

Paper real trials used Go1, Orin NX and ZED Mini. Reported 10-trial scenario outcomes must not be presented as Go2 results.

## Known Paper-to-Repository Uncertainties

- Exact current model export order and provenance: `UNKNOWN`.
- Exact current RA target implementation relative to paper Eq.18: `UNKNOWN` until provenance is established.
- Exact equivalence of current box/terrain ray geometry to paper training: `UNKNOWN`.
- Exact simulator/system-identification parameters used for current deployment artifacts: `UNKNOWN`.
