#!/usr/bin/env bash
# Prepared only; this command is not run automatically.
set -euo pipefail
cd /home/lidio/quadruped_robots

python_bin=/home/lidio/anaconda3/envs/abs/bin/python
base=logs/real_go2/20261116_d435i_ray_adaptation_data/adaptation_v2_current_mount
supplement=logs/real_go2/20261117_d435i_near_coverage_supplement_v1
output_root=logs/real_go2/20261117_d435i_near_coverage_training
run_name="${1:-medium_fullnet_near_coverage_60ep}"

for required in \
  archive/ray_pred_finetune/mujoco_finetune_medium_20260611/ray_pred_mujoco_finetuned_best.pt \
  "$base/data/train/label.pkl" "$base/data/validation/label.pkl" \
  "$base/ray_projection_visibility.csv" \
  "$supplement/data/train/label.pkl" "$supplement/data/validation/label.pkl" \
  "$supplement/ray_projection_visibility.csv"; do
  test -f "$required" || { echo "Required input missing: $required" >&2; exit 2; }
done

if [[ -e "$output_root/$run_name" || -e "$output_root/$run_name.log" ]]; then
  echo "Output already exists: $output_root/$run_name; preserve it and choose another run name." >&2
  exit 2
fi

mkdir -p "$output_root"
"$python_bin" -u scripts/finetune_mujoco_ray_pred.py \
  --base-model archive/ray_pred_finetune/mujoco_finetune_medium_20260611/ray_pred_mujoco_finetuned_best.pt \
  --datasets "$base/data/train" "$supplement/data/train" \
  --validation-datasets "$base/data/validation" "$supplement/data/validation" \
  --validation-visibility-csv "$base/ray_projection_visibility.csv" "$supplement/ray_projection_visibility.csv" \
  --output-root "$output_root" --run-name "$run_name" \
  --device cuda:0 --trainable all --epochs 60 --batch-size 16 \
  --lr 1e-5 --weight-decay 1e-4 --near-weight 4 --false-safe-weight 8 --false-danger-weight 0 \
  --selection-priority visible_near --seed 20261117 --num-workers 0 \
  2>&1 | tee "$output_root/$run_name.log"
