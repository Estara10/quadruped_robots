# Ray-Pred Offline Comparison

Generated: 2026-11-03T13:47:55
Scene: `/home/lidio/quadruped_robots/unitree_mujoco/unitree_robots/go2/scene_flat.xml`
Model: `archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt`
CSV: `docs/thesis_project/evidence/R1/GO2_RAY_PRED_12/input_link_review/scene_flat_fixed6/ray_pred_compare.csv`

## Overall

- Samples: **6**
- Mean abs error: **0.582 m** / **0.207 log2**
- RMSE: **1.058 m** / **0.325 log2**
- Max abs error: **2.662 m** / **0.846 log2**
- False-safe rays: **0**
- False-danger rays: **18**
- Flipped mean abs error: **0.582 m** / **0.207 log2**
- Flipped false-safe/danger rays: **0 / 18**
- Mean render/inference: **41.3 ms / 37.9 ms**

## Per-Ray Metrics

| Ray | Mean target(m) | Mean pred(m) | Bias(m) | MAE(m) | Max abs(m) |
|---:|---:|---:|---:|---:|---:|
| 0 | 6.000 | 6.000 | 0.000 | 0.000 | 0.000 |
| 1 | 6.000 | 6.000 | 0.000 | 0.000 | 0.000 |
| 2 | 6.000 | 6.000 | 0.000 | 0.000 | 0.000 |
| 3 | 6.000 | 6.000 | 0.000 | 0.000 | 0.000 |
| 4 | 6.000 | 6.000 | 0.000 | 0.000 | 0.000 |
| 5 | 6.000 | 6.000 | 0.000 | 0.000 | 0.000 |
| 6 | 6.000 | 5.803 | -0.197 | 0.197 | 0.197 |
| 7 | 6.000 | 5.593 | -0.407 | 0.407 | 0.407 |
| 8 | 6.000 | 4.672 | -1.328 | 1.328 | 1.328 |
| 9 | 6.000 | 4.193 | -1.807 | 1.807 | 1.807 |
| 10 | 6.000 | 3.338 | -2.662 | 2.662 | 2.662 |

## Interpretation

- False-safe: target <= 1.50m and prediction is >= target + 0.50m.
- False-danger: target >= 3.00m and prediction is <= target - 0.50m.
- Flipped metrics compare the same prediction after reversing ray order; a large improvement indicates a likely left-right ray-order mismatch.
- This script never writes `/mujoco_ray2d`; it only compares Ray-Pred output with an in-process geometric target.
