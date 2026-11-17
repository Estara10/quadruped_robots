#!/usr/bin/env bash
# Prepared commands only: run explicitly with benchmark or train.
set -euo pipefail
cd /home/lidio/quadruped_robots
phase="${1:-}"
case "$phase" in
  benchmark) epochs=1; run_name=d435i_fullnet_benchmark_20261117 ;;
  train) epochs=300; run_name=d435i_fullnet_300epoch_20261117 ;;
  *) echo "Usage: bash scripts/run_d435i_fullnet_20261117.sh benchmark|train [run_name]"; exit 2 ;;
esac
run_name="${2:-$run_name}"
python_bin=/home/lidio/anaconda3/envs/abs/bin/python
dataset=logs/real_go2/20261116_d435i_ray_adaptation_data/adaptation_v2_current_mount
holdout=logs/real_go2/20261117_d435i_fullnet_preparation/final_holdout_v1
output_root=logs/real_go2/20261117_d435i_fullnet_preparation/training_runs
if [[ -e "$output_root/$run_name" || -e "$output_root/$run_name.log" ]]; then
  echo "Output already exists: $output_root/$run_name; preserve it and choose a new run name." >&2
  exit 2
fi
test_args=()
if [[ "$phase" == train ]]; then
  test -f "$holdout/data/test/label.pkl"
  test_args=(--test-datasets "$holdout/data/test" --test-visibility-csv "$holdout/ray_projection_visibility.csv")
fi
mkdir -p "$output_root"
"$python_bin" -u scripts/finetune_mujoco_ray_pred.py \
  --base-model archive/ray_pred_finetune/mujoco_finetune_medium_20260611/ray_pred_mujoco_finetuned_best.pt \
  --datasets "$dataset/data/train" \
  --validation-datasets "$dataset/data/validation" \
  --validation-visibility-csv "$dataset/ray_projection_visibility.csv" \
  "${test_args[@]}" \
  --output-root "$output_root" --run-name "$run_name" \
  --device cuda:0 --trainable all --epochs "$epochs" --batch-size 16 \
  --lr 1e-5 --weight-decay 1e-4 --near-weight 4 --false-safe-weight 8 --false-danger-weight 0 \
  --selection-priority visible_near --seed 20261117 --num-workers 0 \
  2>&1 | tee "$output_root/$run_name.log"
