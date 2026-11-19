#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."

python_bin="${ABS_PYTHON:-$HOME/anaconda3/envs/abs/bin/python}"
base=logs/real_go2/20261116_d435i_ray_adaptation_data/adaptation_v2_current_mount
scenes=logs/real_go2/20261119_d435i_scene_coverage_v1
near=logs/real_go2/20261117_d435i_near_coverage_supplement_v1
initial=logs/real_go2/20261117_d435i_fullnet_preparation/training_runs/d435i_fullnet_300epoch_20261117/ray_pred_mujoco_finetuned_best.pt
output_root=logs/real_go2/20261119_d435i_scene_coverage_ab
epochs=60
test_options=(--test-datasets "$scenes/data/test" --test-visibility-csv "$scenes/ray_projection_visibility.csv")
checkpoint_options=()
extra_training_data=()

case "${1:-both}" in
  both) runs=(baseline expanded) ;;
  baseline|expanded) runs=("$1") ;;
  repeat)
    runs=()
    for seed in 20261117 20261118 20261119; do
      runs+=("baseline_seed${seed}" "expanded_seed${seed}")
    done
    ;;
  extended)
    runs=(baseline_1000ep_seed20261117 expanded_1000ep_seed20261117)
    epochs=1000
    test_options+=(--test-last --test-separate-hit test_table_top)
    ;;
  near)
    runs=(expanded_near_1000ep_seed20261117)
    epochs=1000
    extra_training_data=("$near/data/train")
    checkpoint_options=(--save-epochs 60 250 500 750 1000)
    test_options=()
    ;;
  *) echo "Usage: bash $0 [both|baseline|expanded|repeat|extended|near]" >&2; exit 2 ;;
esac

test -x "$python_bin" || { echo "ABS Python not found: $python_bin" >&2; exit 2; }
for required in "$initial" "$base/data/train/label.pkl" "$scenes/data/train/label.pkl" \
  "$base/data/validation/label.pkl" "$scenes/data/validation/label.pkl" \
  "$base/ray_projection_visibility.csv" "$scenes/ray_projection_visibility.csv"; do
  test -f "$required" || { echo "Required input missing: $required" >&2; exit 2; }
done
if ((${#test_options[@]})); then
  test -f "$scenes/data/test/label.pkl" || { echo "Test labels missing: $scenes/data/test/label.pkl" >&2; exit 2; }
fi
for directory in "${extra_training_data[@]}"; do
  test -f "$directory/label.pkl" || { echo "Required input missing: $directory/label.pkl" >&2; exit 2; }
done

for run in "${runs[@]}"; do
  if [[ -e "$output_root/$run" || -e "$output_root/$run.log" ]]; then
    echo "Output already exists: $output_root/$run" >&2
    exit 2
  fi
done

"$python_bin" -c 'import torch; torch.cuda.init(); print("GPU:", torch.cuda.get_device_name(0))'
mkdir -p "$output_root"
for run in "${runs[@]}"; do
  seed=20261117
  if [[ "$run" == *_seed* ]]; then
    seed="${run##*_seed}"
  fi
  training_data=("$base/data/train")
  if [[ "$run" == expanded* ]]; then
    training_data+=("$scenes/data/train")
  fi
  training_data+=("${extra_training_data[@]}")
  echo "Starting $run (seed $seed)"
  "$python_bin" -u scripts/finetune_mujoco_ray_pred.py \
    --base-model "$initial" --datasets "${training_data[@]}" \
    --validation-datasets "$base/data/validation" "$scenes/data/validation" \
    --validation-visibility-csv "$base/ray_projection_visibility.csv" "$scenes/ray_projection_visibility.csv" \
    "${test_options[@]}" "${checkpoint_options[@]}" \
    --output-root "$output_root" --run-name "$run" \
    --device cuda:0 --trainable all --epochs "$epochs" --batch-size 16 --steps-per-epoch 105 \
    --lr 1e-5 --weight-decay 1e-4 --near-weight 4 --false-safe-weight 8 --false-danger-weight 0 \
    --selection-priority visible_near --seed "$seed" --num-workers 0 \
    2>&1 | tee "$output_root/$run.log"
done
