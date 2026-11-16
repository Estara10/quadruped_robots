# Ray-Pred Offline Comparison

Generated: 2026-11-02T19:30:51
Scene: `/home/lidio/quadruped_robots/unitree_mujoco/unitree_robots/go2/challenge27.xml`
Model: `archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt`
CSV: `docs/thesis_project/evidence/R1/GO2_RAY_PRED_12/static/challenge27/ray_pred_compare.csv`

## Overall

- Samples: **4**
- Mean abs error: **1.355 m** / **0.671 log2**
- RMSE: **2.067 m** / **0.929 log2**
- Max abs error: **4.963 m** / **2.533 log2**
- False-safe rays: **0**
- False-danger rays: **19**
- Flipped mean abs error: **2.526 m** / **1.787 log2**
- Flipped false-safe/danger rays: **14 / 20**
- Mean render/inference: **31.4 ms / 35.4 ms**

## Per-Ray Metrics

| Ray | Mean target(m) | Mean pred(m) | Bias(m) | MAE(m) | Max abs(m) |
|---:|---:|---:|---:|---:|---:|
| 0 | 3.861 | 2.545 | -1.316 | 1.316 | 2.957 |
| 1 | 3.300 | 1.648 | -1.652 | 1.652 | 4.163 |
| 2 | 3.384 | 1.504 | -1.880 | 1.880 | 4.870 |
| 3 | 2.224 | 1.410 | -0.813 | 0.813 | 2.528 |
| 4 | 1.142 | 0.911 | -0.232 | 0.266 | 0.790 |
| 5 | 1.108 | 0.934 | -0.174 | 0.335 | 0.602 |
| 6 | 3.429 | 1.176 | -2.253 | 2.331 | 4.963 |
| 7 | 3.358 | 1.768 | -1.590 | 1.681 | 4.568 |
| 8 | 3.358 | 2.005 | -1.353 | 1.473 | 4.254 |
| 9 | 4.106 | 2.527 | -1.579 | 1.595 | 3.023 |
| 10 | 3.952 | 2.862 | -1.091 | 1.560 | 3.102 |

## Interpretation

- False-safe: target <= 1.50m and prediction is >= target + 0.50m.
- False-danger: target >= 3.00m and prediction is <= target - 0.50m.
- Flipped metrics compare the same prediction after reversing ray order; a large improvement indicates a likely left-right ray-order mismatch.
- This script never writes `/mujoco_ray2d`; it only compares Ray-Pred output with an in-process geometric target.
