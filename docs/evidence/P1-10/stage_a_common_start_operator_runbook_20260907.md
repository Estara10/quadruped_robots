# P1-10 Stage-A Common-Start Operator Runbook

本说明只适用于 Independent Reviewer 通过离线 common-start contract、且
Director 已经为一个新的 fresh pair 明确授权之后。当前没有可运行的
common-start pair；不要使用或覆盖 `replay_pair_20260907_stage_a_current_instrumented`，
也不要使用任何历史 pair。

## 固定身份

运行参数由新的 frozen pair 和
`docs/evidence/P1-10/stage_a_common_start_execution_manifest_20260907.json`
固定：`flat_goal_forward`、`stabilized`、root seed `20260902`、`25.0 s`、
`scene_default / mj_makeData:qpos0`。当前 instrumented executable 是独立的
Stage-A artifact，不是 P1-08 accepted executable；P1-08 canonical identity
只作为 parent/provenance。

`<FRESH_COMMON_START_PAIR_DIR>` 必须由 Director 在新授权中给出。Operator
不得创建 pair 目录、手写 capture ID 或修改 pair manifest。

## 1. 宿主 readiness

```bash
cd /home/lidio/quadruped_robots
source /opt/ros/humble/setup.bash
source /home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash

printf 'DISPLAY=%s\n' "${DISPLAY-<UNSET>}"
printf 'XAUTHORITY=%s\n' "${XAUTHORITY-<UNSET>}"
xdpyinfo >/dev/null
direct_rc=$?
printf 'direct_xdpyinfo_rc=%s\n' "$direct_rc"
[ "$direct_rc" -eq 0 ] || exit "$direct_rc"

python3 - <<'PY'
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path("scripts").resolve()))
from p1_08_baseline_capture import child_env

rc = subprocess.run(["xdpyinfo"], env=child_env(),
                    stdout=subprocess.DEVNULL).returncode
print(f"child_env_xdpyinfo_rc={rc}")
raise SystemExit(rc)
PY
```

两项返回码必须都是 `0`。任一失败立即停止。Operator 不得手动设置或
修改 `DISPLAY`、`XAUTHORITY`、`LD_LIBRARY_PATH` 或其他运行环境变量。

## 2. Run A（只运行一次）

先确认 Director 给出的 pair 根目录已经含有 frozen `pair_manifest.json`，
且 `run_A` 尚不存在。不要创建目录或修改 manifest。然后仅执行以下命令，
把 `<FRESH_COMMON_START_PAIR_DIR>` 替换成 Director 明确给出的、位于
`docs/evidence/P1-10/` 下的 pair 目录名：

```bash
cd /home/lidio/quadruped_robots
source /opt/ros/humble/setup.bash
source /home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash

python3 scripts/p1_08_baseline_capture.py \
  --out-dir /home/lidio/quadruped_robots/docs/evidence/P1-10/<FRESH_COMMON_START_PAIR_DIR>/run_A \
  --window-s 25.0 \
  --scene scene_flat.xml \
  --mujoco-bin /home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco \
  --manifest docs/evidence/P1-08/P1-08_baseline_manifest.json \
  --stage-a-common-start-execution-manifest docs/evidence/P1-10/stage_a_common_start_execution_manifest_20260907.json \
  --scenario flat_goal_forward \
  --root-seed 20260902 \
  --variant stabilized \
  --initial-state-source scene_default
```

Run A 返回非零、preflight 失败、gate/anchor 缺失或不一致、record 无效、
进程事实不完整、强制终止、残留进程或证据缺失，均视为 A 失败：立即停止，
不运行 B，不重试 A，不修改 A 目录。

## 3. Run B（仅在 A 完整成功后运行一次）

只有 Director 认可 A 的全部 post-check 后才可运行 B。只把输出目录改为
`run_B`，其余命令完全不变：

```bash
cd /home/lidio/quadruped_robots
source /opt/ros/humble/setup.bash
source /home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash

python3 scripts/p1_08_baseline_capture.py \
  --out-dir /home/lidio/quadruped_robots/docs/evidence/P1-10/<FRESH_COMMON_START_PAIR_DIR>/run_B \
  --window-s 25.0 \
  --scene scene_flat.xml \
  --mujoco-bin /home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco \
  --manifest docs/evidence/P1-08/P1-08_baseline_manifest.json \
  --stage-a-common-start-execution-manifest docs/evidence/P1-10/stage_a_common_start_execution_manifest_20260907.json \
  --scenario flat_goal_forward \
  --root-seed 20260902 \
  --variant stabilized \
  --initial-state-source scene_default
```

B 返回非零或任何 post-check 失败，立即停止且不重试 B。任一 A/B 失败都
不会产生可接受的 pair 结论。

## 4. 比较和禁止事项

只有 A、B 都成功且各自 evidence 完整后，才可按 Director 随新 pair 提供
的 comparator 命令执行 saved-record comparison。比较输入只能是该 frozen
pair 的 `run_A/runtime_record.jsonl` 与 `run_B/runtime_record.jsonl`；不得读
live shared memory，不得使用历史 comparator 的固定 pair 路径，不得覆盖已有
comparison outputs。当前 common-start manifest 本身不构成 runtime evidence，
也不授权 Operator 运行。

Operator 不得通过 UI 执行 reset、keyframe、step-forward 或 teleop，不得手动
修改模型、场景、配置、参数、环境变量或控制输入；不得启动 benchmark、
FormalRun、P1-11、P1-12、P1-13；不得 commit/push。
