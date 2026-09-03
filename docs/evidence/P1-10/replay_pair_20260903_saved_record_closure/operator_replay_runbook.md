# P1-10 宿主 Operator Replay Runbook

状态：**离线 pair 已冻结，等待独立 Reviewer 审核。**

在 Reviewer 明确通过本目录的 pair 冻结和 saved-record comparator 之前，
Operator **不得启动** Run A 或 Run B。

本 runbook 对应唯一 pair：

```text
P1-10-REPLAY-20260903-saved-record-closure-flat_goal_forward-stabilized
```

Operator 不创建 pair、不创建输出目录、不修改代码/config/scenario/baseline，
不使用历史失败目录 `replay_pair_20260902` 或 `replay_pair_20260903`。

## 固定绑定

```text
scenario                         flat_goal_forward
scenario_sha256                  beba99ed4e6f6c8f84eb1ac514f2da4b6e910c1587fdf91f5e95ac6bc639e092
suite                            scenarios/p1_10/scenario_suite_manifest.json
suite_sha256                     eb81d60742864fe9c870e957ba3ab601e80da3e64bc48a42c26f849570f3152d
variant                          stabilized
switching_mode                   stabilized_switch
root_seed                        20260902
run_window_s                    25.0
scene                            scene_flat.xml
initial_state_source             scene_default
initial_state_reset              mj_makeData:qpos0; no keyframe reset
startup_path                     main.cc:PhysicsThread: mj_loadXML -> mj_makeData -> sim.Load -> mj_forward
model_closure_sha256             8d9218de0dc02978fc0ef4ba1c790fa3b968fbdbfdb945e14522436a2574ea07
initial_state_qpos_sha256        a604dd11dc57ea655bf6d746dcf068a91e80a0a1eddc73d20c1a3800468f59d8
initial_state_binding_sha256     f7907a927c31d3d6a5d497ab274b3d913bf4fc8ccb0e9713a9dbd1e182d0a9a0
baseline_manifest                 docs/evidence/P1-08/P1-08_baseline_manifest.json
baseline_manifest_sha256         2667ed37a854f85e5a7c493e7d4a8b1871a84ce95d3e3b0742801d383f8dc915
baseline_identity_document        docs/evidence/P1-08/P1-08_simulation_baseline_identity.json
baseline_identity_document_sha256 6c3563c25d45cc275db6b083f9f0fc0cc2067b48bc8f4a93dcace9f6d42817ea
canonical_baseline_identity       59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0
variant_binding_sha256            2f0dfc4e8bf5237a578d99030facc38459fd5f899af49b508e48e29b7e8a4e1c
```

Root seed 在当前固定场景中仅用于 pairing/provenance，不作为运行时随机数注入。

## 1. 宿主 X11 readiness

在可用图形终端执行。不得手动设置 `DISPLAY`、`XAUTHORITY` 或
`LD_LIBRARY_PATH`；`child_env()` 由 harness 原样调用。

```bash
cd /home/lidio/quadruped_robots
source /opt/ros/humble/setup.bash
source /home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash

printf 'DISPLAY=%s\n' "${DISPLAY-<UNSET>}"
printf 'XAUTHORITY=%s\n' "${XAUTHORITY-<UNSET>}"
printf 'uid=%s gid=%s\n' "$(id -u)" "$(id -g)"
for ns in user mnt pid net ipc uts cgroup time; do
    printf 'ns_%s=' "$ns"
    readlink "/proc/self/ns/$ns"
done

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

child_rc = subprocess.run(
    ["xdpyinfo"],
    env=child_env(),
    stdout=subprocess.DEVNULL,
).returncode
print(f"child_env_xdpyinfo_rc={child_rc}")
raise SystemExit(child_rc)
PY
```

两个 `xdpyinfo` 返回码都必须为 `0`。任一失败，停止，不启动 A/B。

## 2. Run A 前检查

pair 根目录和冻结 manifest 必须已经存在；Operator 不执行 `mkdir`、不删除
文件、不修改 manifest：

```bash
test -d /home/lidio/quadruped_robots/docs/evidence/P1-10/replay_pair_20260903_saved_record_closure
test -f /home/lidio/quadruped_robots/docs/evidence/P1-10/replay_pair_20260903_saved_record_closure/pair_manifest.json
test ! -e /home/lidio/quadruped_robots/docs/evidence/P1-10/replay_pair_20260903_saved_record_closure/run_A
test ! -e /home/lidio/quadruped_robots/docs/evidence/P1-10/replay_pair_20260903_saved_record_closure/run_A_preflight_fail.json
sha256sum /home/lidio/quadruped_robots/docs/evidence/P1-10/replay_pair_20260903_saved_record_closure/pair_manifest.json
```

上述 pair manifest 的 SHA-256 必须与 Director 提供的冻结值一致。

记录进程和任务 shm 状态，不手动清理：

```bash
ps -eo pid=,ppid=,pgid=,stat=,comm=,args=
ls -l /dev/shm/mujoco_sim_clock /dev/shm/mujoco_rt_frame 2>&1
```

以下运行时身份在 A 启动前必须不存在：

- `/home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco`
- `ros2 launch rl_quadruped_controller mujoco.launch.py`
- 使用 Go2 `robot_control.yaml` 的 `ros2_control_node`

harness 的精确 `/proc` executable/argv identity 检查是权威判定；不得使用宽泛
`pgrep -af` 字符串匹配替代它。

关键文件必须与 pair manifest 中的 hash 一致：

```text
mujoco executable       1e9b330f2b6c39dabaaa8424ee53c41d3be08ea00eb3e69ba71f332de50654e2
scene_flat.xml          9ce83b3e61c722a523d0359536cee803f17610f95d2275fc32e96801ec3c1908
simulate config         86fcfab9ecdf888901340697ef9c99fcc72bbd3d88c86f2178b4ce2ab2c88b95
robot_control.yaml      59c61ad4f29c2b37b3741236b35ce11c773fb6cfe16ed8d58b92757febf8bf7c
ABS config              1cd42c4bb29baad1873bd55e7f2f1d82fb0c8ec8bf35fb8e4eff61338758c586
launch file             ed87b9204cf9d5dde83e4ff54bcafac16aae5e1407ecd8b394d2c5e777a17fb9
controller plugin        2b31e558471227a385906239a4fd20d1f9cb759a4960c743b6f5065f13fe6d4e
hardware plugin          9c56d00d1cca1d396d760c21bce54bbb4dc0e9f7cd7453e4060940fb8c13918d
libmujoco                e35ba7f65d2eeccfeda2c5f251d49e26de6fbbc1067170a889fefaa4a35aa24e
```

任一 hash、目录、pair binding、进程或 X11 检查失败，停止。

## 3. Run A 完整命令

Reviewer 通过前禁止执行：

```bash
cd /home/lidio/quadruped_robots
source /opt/ros/humble/setup.bash
source /home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash

python3 scripts/p1_08_baseline_capture.py \
  --out-dir /home/lidio/quadruped_robots/docs/evidence/P1-10/replay_pair_20260903_saved_record_closure/run_A \
  --window-s 25.0 \
  --scene scene_flat.xml \
  --mujoco-bin /home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco \
  --manifest docs/evidence/P1-08/P1-08_baseline_manifest.json \
  --scenario flat_goal_forward \
  --root-seed 20260902 \
  --variant stabilized \
  --initial-state-source scene_default
```

Run A 必须返回 `0`，且必须有：

- `process_facts.json` 中 `exit_code=0`、`shutdown_complete=true`、`forced_termination=false`；
- SIGINT delivered，MuJoCo/ROS launch wait rc 均为 `0`，无 TERM/KILL escalation；
- `runtime_record.jsonl` 为 `VALID` 且有 authoritative runtime source；
- `reader_stats.json` 满足 harness 的 accepted-frame 要求；
- `orphan_inventory.json` 捕获结束时无相关残留进程。

A 的任何非零返回、preflight failure、证据缺失、记录 INVALID、异常退出、
强制终止或残留进程，都使 A 失败。A 失败后不运行 B、不重试 A。

## 4. Run B 完整命令

只有 A 完整成功且通过 A 的 post-check 后，才允许执行一次：

```bash
cd /home/lidio/quadruped_robots
source /opt/ros/humble/setup.bash
source /home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash

python3 scripts/p1_08_baseline_capture.py \
  --out-dir /home/lidio/quadruped_robots/docs/evidence/P1-10/replay_pair_20260903_saved_record_closure/run_B \
  --window-s 25.0 \
  --scene scene_flat.xml \
  --mujoco-bin /home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco \
  --manifest docs/evidence/P1-08/P1-08_baseline_manifest.json \
  --scenario flat_goal_forward \
  --root-seed 20260902 \
  --variant stabilized \
  --initial-state-source scene_default
```

B 失败不重试。

## 5. Run 后证据

每个成功运行目录必须包含并回传：

```text
scenario_resolved_manifest.json
p1_10_context.json
preflight_evidence.json
runtime_record.jsonl
sim_clock_timing.jsonl
rt_frame_timing.jsonl
reader_stats.json
process_facts.json
orphan_inventory.json
mujoco_raw.log
ros2_launch_raw.log
orchestrator_raw.log
```

每次运行后重新记录：

```bash
ps -eo pid=,ppid=,pgid=,stat=,comm=,args=
ls -l /dev/shm/mujoco_sim_clock /dev/shm/mujoco_rt_frame 2>&1
```

对应目录必须分别为：

```text
docs/evidence/P1-10/replay_pair_20260903_saved_record_closure/run_A/
docs/evidence/P1-10/replay_pair_20260903_saved_record_closure/run_B/
```

任何 `*_preflight_fail.json` 都不是成功运行证据。

## 6. Saved-record comparison

只有 A、B 均成功并通过 post-check 后，才允许执行以下离线比较器。比较器只接受
`--pair-dir`，从该目录的冻结 `pair_manifest.json` 推导 `run_A`/`run_B`；不得
提供任意外部 run 路径。它会从每个 run 目录读取并强制核验以下四个文件：
`process_facts.json`、`runtime_record.jsonl`、`scenario_resolved_manifest.json`、
`p1_10_context.json`。两个 context 文件都必须显式包含 pair-required 的完整
binding（包括 scene root、model closure、initial-state qpos hash）；缺失字段
不得从 pair manifest、另一份 context、process facts、默认值或 `None` 回填。
这些字段由 `scripts/p1_08_baseline_capture.py::write_p1_10_context` 从已验证的
scenario/closure resolution 结果直接写出；Operator 不生成或修改 context。
比较阶段只读取落盘文件，不读取 live shared memory：

```bash
cd /home/lidio/quadruped_robots
python3 scripts/p1_10_saved_record_compare.py \
  --pair-dir docs/evidence/P1-10/replay_pair_20260903_saved_record_closure
```

比较器在比较前验证 process facts、唯一且末尾的 terminal、VALID runtime
record、run/session/binding 一致性、两个 context 与 pair manifest 的完整绑定。
成功关键字段不得为缺失、`None` 或 `UNKNOWN`；当前没有权威生产者的
`reached_goal`、`timeout`、`collision_events`、`fall_events` 仅可按冻结规则
以显式 `UNKNOWN` 保留，不能把缺失或双方 UNKNOWN 当成一致。
`termination_reason` 必须是 recorder 产生的五个值之一；安全 fault 字段必须是
JSON bool；四个事件字段必须是精确字符串 `UNKNOWN`。pair 根目录、run 目录及
四份 required artifact 必须是 frozen pair 内的普通目录/文件，禁止 symlink 或
resolve 后落到 pair 外。

比较规则已冻结在 `pair_manifest.json`：

- frame count、strict `rl_step` sequence、controller/RL flags、policy state、safety flags、Recovery entry/exit sequence、terminal/process success semantics 必须 exact match；
- `world_pose`、`command`、`ra_value`、`action_raw`、`action_clipped`、`joint_target_rad`、`torque_nm`、`ray2d` 必须 canonical JSON exact match；任何非零 numeric delta 都失败，同时报告 max/mean delta；
- `run_id`、`session_id`、`source_sequence`、record timestamps、wall-clock、PID/PGID、reader polling cadence、以及 manifest 明确列出的诊断字段均排除。

比较器不得覆盖既有输出；任一记录 INVALID、pair manifest binding 不匹配或比较失败，均停止，不重试。

比较器成功输出：

```text
canonical_identity_input.json
canonical_identity_output.json
diff_report.json
saved_record_comparison_report.md
```

比较器输出文件不可覆盖；任一 contract reject 返回非零并停止。两次运行均成功
后才允许生成上述 saved-record replay comparison；A 失败不运行 B，B 失败不重试。

不得将 saved-record comparison 宣称为 benchmark、FormalRun acceptance 或 Phase 1 acceptance。

## 7. Operator 禁止事项

正式 P1-10 capture 的 collision-authority 范围仅是 harness 控制的
`main.cc` PhysicsLoop 两条 `mj_step` 路径；每个受控 PhysicsLoop step 后
发布一次 snapshot。`simulate.cc` 的 UI step-forward 只用于交互调试，不属于
正式 capture authority scope。Operator 不得通过 UI reset、keyframe、
step-forward、teleop 或其他方式干预运行。

本 runbook 的 flat replay 不包含 Stage-B obstacle authority。未来
`obstacle_test1` 运行必须使用独立的 Stage-B manifest/identity（包括新的
instrumented executable 与 capture identity），不得复用 accepted P1-08
executable identity 或任何历史 pair。

- 不手动修改环境变量、模型、配置、场景或参数；
- 不通过 MuJoCo UI 执行 reset、keyframe、teleop 或其他干预；
- 不运行历史失败 pair，不重试 A/B；
- 不 commit/push；
- 不启动 P1-11、P1-12、P1-13、benchmark 或 FormalRun；
- 未获独立 Reviewer 通过前，不启动 A/B。
