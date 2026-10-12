# S6-01 初态与来源记录

固定场景 `scene_random_04.xml` / S2-08-r1；本批 C、D 的 reset probe 均来自 `mj_loadXML→mj_makeData→mj_forward`。它记录构造初态，不等于 RL 开始时的实测状态。

| 组 | reset base pose / yaw | reset qpos | RL 前 `/joint_states` 采样 | 样本→CLI 调用开始；样本→首个策略活跃帧 | 位移锚点及异步关节样本 |
|---|---|---|---|---|---|
| A | `(0,0,0.445 m)` / `0 rad` | 12 joints all 0 rad | 不可用；首批旧采集器落到 Anaconda Python 3.13，未有 rclpy 样本 | UNKNOWN | 旧运动锚点：session `1719084890764`，cycle 49，sim_time `25.076 s`，pose `(0.07766,-0.00359,-0.04215)`；joint sample unavailable |
| B | `(0,0,0.445 m)` / `0 rad` | 12 joints all 0 rad | 不可用；同 A 的旧 Python ABI 问题 | UNKNOWN | 旧运动锚点：session `1776891406391`，cycle 45，sim_time `26.764 s`，pose `(0.03660,-0.00014,-0.00574)`；joint sample unavailable |
| C | `(0.000,0.000,0.445 m)` / `0 rad` | reset qpos: base `(0,0,0.445)`, quaternion `(1,0,0,0)`, all 12 joint qpos `0`; qvel UNKNOWN | status `captured`; `/usr/bin/python3`, ROS Humble rclpy; sample monotonic `9277530208212`; ROS header `1791778774.807882843` | `146.992 ms`; `1934.728 ms` | session `9279184632536`, cycle `46`, sequence `94`, sim_time `23.974 s`, pose `(0.044228, -0.001777, -0.012682)`; named `/joint_states` sample arrived `783.855 ms` after anchor, not same-cycle |
| D | `(0.000,0.000,0.445 m)` / `0 rad` | reset qpos: base `(0,0,0.445)`, quaternion `(1,0,0,0)`, all 12 joint qpos `0`; qvel UNKNOWN | status `captured`; `/usr/bin/python3`, ROS Humble rclpy; sample monotonic `9329921746789`; ROS header `1791778827.199520270` | `141.445 ms`; `1604.514 ms` | session `9331242659334`, cycle `46`, sequence `94`, sim_time `27.262 s`, pose `(0.047281, -0.004075, -0.016276)`; named `/joint_states` sample arrived `952.596 ms` after anchor, not same-cycle |

## C/D RL 前具名关节 position / velocity

下表来自 `/joint_states`，映射按消息中的 joint name 生成，不依赖数组顺序。采样在 PASSIVE/FIXEDSTAND 准备期间；C/D 样本分别比首个 `rl_entered=1 && rl_active=1` 帧早约 1.935 s 和 1.605 s，不能视为同周期状态。原始消息顺序、完整数组、时间戳、ROS 环境及逐关节映射保存在对应 `run_context.json` 的 `initial_state.pre_rl_joint_state.sample`。

| 具名关节 | C 位置 (rad) | C 速度 (rad/s) | D 位置 (rad) | D 速度 (rad/s) |
|---|---:|---:|---:|---:|
| `FL_calf_joint` | -1.861816 | 0.417048 | -1.761269 | 0.289651 |
| `FL_hip_joint` | 0.019678 | 0.000636 | 0.019845 | 0.000532 |
| `FL_thigh_joint` | 0.890150 | -0.147627 | 0.854558 | -0.103514 |
| `FR_calf_joint` | -1.861815 | 0.417048 | -1.761268 | 0.289650 |
| `FR_hip_joint` | -0.019675 | -0.000636 | -0.019842 | -0.000532 |
| `FR_thigh_joint` | 0.890148 | -0.147629 | 0.854556 | -0.103515 |
| `RL_calf_joint` | -1.868274 | 0.442622 | -1.761725 | 0.309094 |
| `RL_hip_joint` | -0.042664 | 0.029461 | -0.035505 | 0.020247 |
| `RL_thigh_joint` | 0.923884 | -0.173587 | 0.882098 | -0.121763 |
| `RR_calf_joint` | -1.868271 | 0.442626 | -1.761721 | 0.309097 |
| `RR_hip_joint` | 0.042663 | -0.029465 | 0.035503 | -0.020251 |
| `RR_thigh_joint` | 0.923905 | -0.173600 | 0.882117 | -0.121773 |

## 解释与限制

- A/B 历史 motion-start ROS joint-state 样本缺失，原始启动失败记录保留；不能由目标关节配置补值。
- C/D pre-RL 样本成功且覆盖 12 个具名 Go2 关节，但 RL 首帧前仍有 1.6–1.9 s 时间间隔；运动锚点后的异步关节样本分别晚 0.784 s / 0.953 s。没有精确同周期关节快照。
- 四组起始位置、朝向和关节状态未严格一致，也没有确定性重放。C 与 D 的具名样本可用于描述，不构成配对初态。
- 每个 run 的 `run_context.json` 保存真实 reset、实测位置姿态、样本 monotonic/ROS header 时间和周期关联。动作顺序等既有接口证据范围不在本记录重新推断。
