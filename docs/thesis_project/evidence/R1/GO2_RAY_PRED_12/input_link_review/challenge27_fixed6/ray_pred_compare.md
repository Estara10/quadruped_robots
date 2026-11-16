# Ray-Pred Offline Comparison

Generated: 2026-11-03T13:48:00
Scene: `/home/lidio/quadruped_robots/unitree_mujoco/unitree_robots/go2/challenge27.xml`
Model: `archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt`
CSV: `docs/thesis_project/evidence/R1/GO2_RAY_PRED_12/input_link_review/challenge27_fixed6/ray_pred_compare.csv`

## Overall

- Samples: **6**
- Mean abs error: **2.058 m** / **0.960 log2**
- RMSE: **2.737 m** / **1.269 log2**
- Max abs error: **5.357 m** / **3.223 log2**
- False-safe rays: **0**
- False-danger rays: **37**
- Flipped mean abs error: **2.604 m** / **1.523 log2**
- Flipped false-safe/danger rays: **10 / 38**
- Mean render/inference: **33.5 ms / 27.5 ms**

## Per-Ray Metrics

| Ray | Mean target(m) | Mean pred(m) | Bias(m) | MAE(m) | Max abs(m) |
|---:|---:|---:|---:|---:|---:|
| 0 | 4.516 | 2.083 | -2.433 | 2.433 | 4.921 |
| 1 | 3.472 | 1.715 | -1.757 | 1.779 | 4.829 |
| 2 | 3.479 | 1.607 | -1.872 | 1.934 | 5.347 |
| 3 | 4.211 | 2.039 | -2.172 | 2.172 | 3.858 |
| 4 | 2.139 | 1.436 | -0.703 | 0.758 | 2.720 |
| 5 | 3.004 | 1.239 | -1.765 | 1.776 | 5.268 |
| 6 | 3.383 | 1.591 | -1.793 | 1.949 | 4.685 |
| 7 | 4.178 | 1.686 | -2.493 | 2.531 | 5.130 |
| 8 | 3.275 | 1.951 | -1.324 | 1.683 | 4.707 |
| 9 | 3.787 | 2.406 | -1.381 | 2.019 | 4.160 |
| 10 | 6.000 | 2.396 | -3.604 | 3.604 | 5.357 |

## Interpretation

- False-safe: target <= 1.50m and prediction is >= target + 0.50m.
- False-danger: target >= 3.00m and prediction is <= target - 0.50m.
- Flipped metrics compare the same prediction after reversing ray order; a large improvement indicates a likely left-right ray-order mismatch.
- This script never writes `/mujoco_ray2d`; it only compares Ray-Pred output with an in-process geometric target.
