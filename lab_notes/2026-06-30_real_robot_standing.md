# 2026-06-30 实机站立测试记录

## 环境

- 地点：实验室
- 机器人：Go2（网线直连）
- PC 网络：enp7s0 → 192.168.123.100/24
- Go2 IP：192.168.123.161
- 分支：feat/ray-pred-source-switch

## 本次修改

### 1. 网络接口切换
- `ros2_control.xacro`：网络参数从注释（仿真 lo 模式）改为 `enp7s0, domain=0`

### 2. FIXEDDOWN / FIXEDSTAND 软启动（新加）
**问题**：进入 FIXEDDOWN/FIXEDSTAND 时 Kp 从 0 瞬间跳到 80，关节用全力往目标位置冲 → 抽搐异响。

**修改文件**：
- `libraries/controller_common/src/FSM/StateFixedDown.cpp` — enter() 设 Kp/Kd=0，run() 中 250 步渐变到目标值
- `libraries/controller_common/include/controller_common/FSM/StateFixedDown.h` — 加 `soft_start_step_`, `soft_start_steps_=250`
- `libraries/controller_common/src/FSM/BaseFixedStand.cpp` — 同上
- `libraries/controller_common/include/controller_common/FSM/BaseFixedStand.h` — 同上

### 3. DDS 超时误报修复
**问题**：PASSIVE 状态下关节自然不动，100 步后被 DDS 超时误判为通信断开 → 误触急停。

**修改文件**：
- `controllers/rl_quadruped_controller/src/RlQuadrupedController.cpp` — DDS 超时检测跳过 PASSIVE 状态

### 4. Kp/Kd 降低（真机适配）
**问题**：FIXEDSTAND 默认 Kp=80 Kd=3.5，对真机 Go2 太高。高 Kd 放大速度噪声 → 关节持续异响。

**修改**：
- `controllers/rl_quadruped_controller/src/RlQuadrupedController.h` — 默认值改为 Kp=30, Kd=0.65（匹配 RL 策略）
- `descriptions/unitree/go2_description/config/robot_control.yaml` — 加 `stand_kp: 30.0`, `stand_kd: 0.65`

## 当前状态

### ✅ 能工作的
- DDS 通信正常（读到 LowState，发 LowCmd）
- 策略模型加载成功（agile + recovery + RA）
- FIXEDDOWN → FIXEDSTAND 过渡平滑（tanh 位置插值 + 软启动 Kp/Kd）

### 🔴 未解决
- **站立时关节持续异响**：Kp/Kd 从 80/3.5 降到 30/0.65，尚未在真机验证是否消除噪声
- **Sport Lease 缺失**：Go2 实机可能需要 sport lease 才能接受 LowCmd。HardwareUnitree 目前无此代码
- **从趴到站**：FIXEDDOWN 目标姿态是 crouch（大腿 1.27, 小腿 -2.8），不是趴平姿态。从趴地起始可能不适合

## 测试命令

```bash
# 1. 设 IP（如果还没设）
sudo ip addr add 192.168.123.100/24 dev enp7s0
ping -c 3 192.168.123.161

# 2. 启动控制器
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rl_quadruped_controller mujoco.launch.py

# 3. 另开终端 — 键盘
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run keyboard_input keyboard_input

# 4. 操作
#    2 → FIXEDDOWN（趴→蹲，软启动 250 步）
#    2 → FIXEDSTAND（蹲→站）
#    1 → PASSIVE（急停卸力）
```

## 重新判断（Opus 复核）

前面“主要是 Kp/Kd 太大”的判断不充分，甚至可能不是根因。理由：

1. 站立/趴下阶段没有进入 RL，不会用到策略模型；换模型不能解决 FIXEDDOWN/FIXEDSTAND 异响。
2. `rl_sar` 官方 Go2 配置中固定站立增益本来就是 `fixed_kp=80`, `fixed_kd=3.0`，所以 Kp=80 并非天然错误。
3. 当前更可疑的是硬件层和 Go2 原生服务冲突：
   - `HardwareUnitree` 没有像 `rl_sar` 一样调用 `MotionSwitcherClient::ReleaseMode()` 关闭 `sport_mode`；可能出现原生运动服务和我们的 LowCmd 同时控制，产生对抗/异响。
   - `HardwareUnitree::initLowCmd()` 用 `q=0,dq=0,kp=0,kd=0`，而 Unitree 官方示例使用 `PosStopF=2.146e9`, `VelStopF=16000` 作为停用哨兵值。
   - `StatePassive` 当前设置 `kd=1,dq=0`，这不是完全卸力；真机 PASSIVE 仍会施加速度阻尼，可能产生嗡鸣/异响。
   - controller_manager 当前 1000Hz 写 LowCmd，而 Unitree Go2 low-level 示例是 500Hz（2ms），可作为次要排查项。

## 更新后的待办

- [x] 先不要换模型；站立异响与 RL policy 无关
- [x] 修 `HardwareUnitree`：初始化/停用电机使用 `PosStopF` / `VelStopF`
- [x] 修 `StatePassive`：`kd=0`，真正 passive 卸力，不发 `dq=0` 阻尼
- [x] 修 `HardwareUnitree`：实机启动时释放 Go2 `sport_mode` / motion service，避免和 LowCmd 对抗
- [ ] 必要时将 `controller_manager.update_rate` 从 1000Hz 改为 500Hz，匹配 Unitree low-level 示例
- [ ] 以上硬件层修复后，再真机验证 FIXEDDOWN/FIXEDSTAND
- [ ] 首次 RL 行走测试必须等站立无异响后再做

## 已应用硬件层修复（2026-06-30 后续）

已按复核结论改代码并编译通过：

- `StatePassive.cpp`
  - `kd=1` → `kd=0`，PASSIVE 真正卸力
- `HardwareUnitree.cpp/.h`
  - 加 `PosStopF=2.146e9`, `VelStopF=16000`
  - `initLowCmd()` 使用 Unitree 官方 stop sentinel 初始化 20 个 motor slot
  - `write()` 每次先把 20 个 motor slot 置 stop，再覆盖 12 个实际关节命令，避免未用 slot 残留旧命令
  - 若 12 个关节命令为 `kp=kd=tau=0`，也发 stop sentinel，而不是 `q=0,dq=0`
  - 实机接口（非 `lo`）启动时调用 `MotionSwitcherClient::ReleaseMode()`，释放 Go2 原生 `sport_mode` / motion service，避免和 LowCmd 对抗
- `RlQuadrupedController.h` + `robot_control.yaml`
  - 恢复 fixed stand 增益到 Go2 参考值：`stand_kp=80.0`, `stand_kd=3.0`

编译命令已通过：

```bash
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
colcon build --packages-select hardware_unitree_mujoco rl_quadruped_controller go2_description --symlink-install \
  --cmake-args -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch
```

## 后腿先动/前腿不明显问题（2026-06-30 后续）

现象：按 `2` 后后腿明显动作，前腿肉眼看不明显；立即按 `1` 卸力，未出现异响。

处理：暂停真机继续测试，复查接口映射。`/joint_states` 发布顺序不是配置顺序，因此不能假设 `ros2_control` loaned interfaces 按 FR/FL/RR/RL 顺序排列。已在 `RlQuadrupedController::on_activate()` 中按 YAML `joints` 列表对所有关节 command/state interface 显式排序：

- command: effort/position/velocity/kp/kd
- state: effort/position/velocity

新增启动日志：

```text
[VERIFY] joint interface order: FR_hip_joint FR_thigh_joint FR_calf_joint ... RL_calf_joint
```

编译已通过：

```bash
colcon build --packages-select rl_quadruped_controller --symlink-install \
  --cmake-args -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch
```

## 今日最终状态

今日停止前的最终状态：

- 实机模式已恢复：`ros2_control.xacro` 为 `domain=0`, `network_interface=enp7s0`
- `go2_description` 已在实机模式下重新编译
- 启动控制器成功，日志正常：
  - `network_interface: enp7s0, domain: 0`
  - `Motion service 'sport_mode' is active; releasing before LowCmd control`
  - `Motion service is already deactivated`
  - `[VERIFY] joint interface order: FR_hip_joint FR_thigh_joint FR_calf_joint ... RL_calf_joint`
  - `Configured and activated rl_quadruped_controller`
- 用户今日未在排序修复后再次按 `2` 测 FIXEDDOWN；下一次从这里继续。

## 安全验证新增改动（后续）

根据迁移到真机前的三步计划，已完成：

1. `HardwareUnitree` 显式 motor index mapping：
   - 默认 `FR,FL,RR,RL = motor[0..11]`
   - `read()` 和 `write()` 都通过 `motor_index_map_` 访问 `LowState/LowCmd`
   - 启动时打印 `[MOTOR-MAP] controller[i] joint -> Unitree motor[j]`
2. 全局硬急停：
   - `command=1` 或 `command=9` 在 `LeggedGymController::update()` 最前面强制进入 PASSIVE
   - 适用于 FIXEDDOWN/FIXEDSTAND/RL/RL_REC，仿真和真机通用
3. 键盘提示更新：
   - `1 = PASSIVE`
   - `9 = HARD STOP`

编译已通过：

```bash
colcon build --packages-select hardware_unitree_mujoco rl_quadruped_controller keyboard_input --symlink-install \
  --cmake-args -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch
```

## 后续导航层更新（2026-07-09）

为解决仿真中“最终能到目标，但途中横向偏移过大，可能撞到旁边障碍”的问题，已加入 recovery-aware 直线路径约束：

- 非 recovery：沿起点→目标直线行走，`path_on=1`
- recovery：放开直线约束，允许绕障，`path_on=0`
- recovery 退出：重新拉回直线路径，`path_on=1`

配置位于 `abs/config.yaml`：

```yaml
path_tracking_enabled: true
path_lateral_gain: 1.5
path_heading_gain: 1.5
```

日志新增：

```text
[PATH] start=(...) goal=(...) tracking=1 lateral_gain=1.50 heading_gain=1.50
[GOAL] ... path_err=... path_on=...
```

真机注意：该导航层只影响 RL/ABS，当前真机仍禁止按 `3` 进入 RL，直到真实 ray2d 感知接入。
