# P1-10 Stage-A Operator Runbook — 2026-09-07

本说明只适用于 Director 提供并经 Independent Reviewer 审核通过的新 pair：

`docs/evidence/P1-10/replay_pair_20260907_stage_a_current_instrumented/`

当前 pair 仅完成离线冻结，`run_A/` 与 `run_B/` 尚不存在。旧的
`replay_pair_20260905_stage_a_current_instrumented/` 是
`FAILED_FOR_THIS_PAIR`，不得重试、覆盖或删除。

## 0. 固定边界

固定绑定为 `flat_goal_forward`、`stabilized`、root seed `20260902`、
`25.0 s`、`scene_default / mj_makeData:qpos0`，以及当前 instrumented
executable。P1-08 canonical identity
`59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0`
仅作 parent/provenance，不表示本次使用 P1-08 executable。

Operator 不得手动修改 `DISPLAY`、`XAUTHORITY`、`LD_LIBRARY_PATH` 或任何
模型、配置、场景、参数；不得使用 UI reset、keyframe、step-forward 或
teleop；不得运行旧 pair、benchmark、FormalRun 或后续 P1 任务。

## 1. 宿主 X11 readiness

从仓库根目录执行：

```bash
cd /home/lidio/quadruped_robots
source /opt/ros/humble/setup.bash
source /home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash

printf 'DISPLAY=%s\n' "${DISPLAY-<UNSET>}"
printf 'XAUTHORITY=%s\n' "${XAUTHORITY-<UNSET>}"
printf 'uid=%s gid=%s\n' "$(id -u)" "$(id -g)"

xdpyinfo >/dev/null
rc=$?
printf 'direct_xdpyinfo_rc=%s\n' "$rc"
[ "$rc" -eq 0 ] || exit "$rc"

python3 - <<'PY'
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path("scripts").resolve()))
from p1_08_baseline_capture import child_env

rc = subprocess.run(
    ["xdpyinfo"], env=child_env(), stdout=subprocess.DEVNULL,
).returncode
print(f"child_env_xdpyinfo_rc={rc}")
raise SystemExit(rc)
PY
```

两次返回码都必须为 `0`。任一失败立即停止，不运行 A/B。

## 2. 冻结身份与目录检查

```bash
python3 scripts/p1_10_stage_a_execution.py \
  --validate docs/evidence/P1-10/stage_a_execution_manifest_20260905.json \
  --executable /home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco

sha256sum docs/evidence/P1-10/replay_pair_20260907_stage_a_current_instrumented/pair_manifest.json

test -f docs/evidence/P1-10/replay_pair_20260907_stage_a_current_instrumented/pair_manifest.json
test ! -e docs/evidence/P1-10/replay_pair_20260907_stage_a_current_instrumented/run_A
test ! -e docs/evidence/P1-10/replay_pair_20260907_stage_a_current_instrumented/run_B
```

若 pair、Stage-A manifest、当前 executable 或固定绑定核验失败，立即停止。

## 3. Run A（只允许一次）

仅在第 1、2 节全部成功后执行：

```bash
python3 scripts/p1_08_baseline_capture.py \
  --out-dir /home/lidio/quadruped_robots/docs/evidence/P1-10/replay_pair_20260907_stage_a_current_instrumented/run_A \
  --window-s 25.0 \
  --scene scene_flat.xml \
  --mujoco-bin /home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco \
  --manifest docs/evidence/P1-08/P1-08_baseline_manifest.json \
  --stage-a-execution-manifest docs/evidence/P1-10/stage_a_execution_manifest_20260905.json \
  --scenario flat_goal_forward \
  --root-seed 20260902 \
  --variant stabilized \
  --initial-state-source scene_default
```

Run A 返回非零、preflight failure、证据缺失、记录无效、异常/强制终止或
残留进程，均视为 A 失败：立即停止，不运行 B，不重试 A。

A 成功后，先确认对应 `process_facts.json`、`runtime_record.jsonl`、
`scenario_resolved_manifest.json`、`p1_10_context.json`、
`preflight_evidence.json` 均存在并通过其 post-check；否则按 A 失败处理。

## 4. Run B（只允许一次）

只有 A 完整成功时才执行一次，命令仅将输出目录改为 `run_B`：

```bash
python3 scripts/p1_08_baseline_capture.py \
  --out-dir /home/lidio/quadruped_robots/docs/evidence/P1-10/replay_pair_20260907_stage_a_current_instrumented/run_B \
  --window-s 25.0 \
  --scene scene_flat.xml \
  --mujoco-bin /home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco \
  --manifest docs/evidence/P1-08/P1-08_baseline_manifest.json \
  --stage-a-execution-manifest docs/evidence/P1-10/stage_a_execution_manifest_20260905.json \
  --scenario flat_goal_forward \
  --root-seed 20260902 \
  --variant stabilized \
  --initial-state-source scene_default
```

B 返回非零或任何 post-check 失败，立即停止，不重试 B。A/B 都完整成功
之前，不得执行 comparator。

## 5. Saved-record comparison（仅 A/B 均成功后）

比较器只读取新 pair 内的落盘文件，不读取 live shared memory：

```bash
python3 scripts/p1_10_stage_a_saved_record_compare.py \
  --pair-dir /home/lidio/quadruped_robots/docs/evidence/P1-10/replay_pair_20260907_stage_a_current_instrumented
```

Comparator 失败也不重试。Operator 不创建 pair、不修改 manifest、不覆盖
比较输出，不进行 commit/push。
