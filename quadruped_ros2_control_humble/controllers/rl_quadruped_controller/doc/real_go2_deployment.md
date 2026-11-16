# Go2 真机部署手册

> **历史手册（2026-07-09），不作为当前现场启动指令。** 2026-11-09 的最新状态见[实机入口](../../../../docs/thesis_project/real_go2/README.md)和[机身坐标转换](../../../../scripts/go2_vins/BODY_FRAME.md)。D435i/VINS 已完成部分只读采集，安装转换待测量；完整 ABS 实机尚未验证。下文的“急停”实际涉及 PASSIVE／卸力，不保证站立、制动或电脑失联时独立停止；原生服务恢复曾触发起立。不得按旧步骤直接接管或运行策略。

> 状态：低层控制安全验证中  
> 最后更新：2026-07-09

本手册只覆盖当前阶段：**Go2 站立/趴下/急停验证**。  
由于真机尚未接入 D435i/LiDAR/深度相机 ray2d，**暂不允许真机进入 ABS/RL 行走**。

---

## 1. 当前真机状态

已实现：

- `enp7s0` DDS 通信
- Go2 `sport_mode` 释放后 LowCmd 接管
- Unitree `PosStopF / VelStopF` stop sentinel
- PASSIVE 真卸力：`kp=0,kd=0,tau=0`
- FIXEDDOWN/FIXEDSTAND Kp/Kd 软启动
- ros2_control command/state interface 显式排序
- Go2 motor index mapping 显式打印
- 全局急停：`1` 或 `9` → PASSIVE/stop sentinel

仍未完成：

- 真实 ray2d 感知源
- 遥控器硬急停映射
- 温度监控
- 真机 RL/ABS 行走验证

---

## 2. 网络连接

Go2 默认有线 IP：

```text
192.168.123.161
```

PC 有线口：

```text
enp7s0
```

配置：

```bash
sudo ip addr add 192.168.123.100/24 dev enp7s0
ping -c 3 192.168.123.161
```

如果提示：

```text
RTNETLINK answers: File exists
```

说明 IP 已存在，可继续。

---

## 3. 实机/仿真 xacro 切换

真机模式：

```xml
<param name="domain">0</param>
<param name="network_interface">enp7s0</param>
```

仿真模式：

```xml
<!--<param name="domain">0</param>-->
<!--<param name="network_interface">enp7s0</param>-->
```

文件：

```text
quadruped_ros2_control_humble/descriptions/unitree/go2_description/xacro/ros2_control.xacro
```

切换后必须编译：

```bash
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_description --symlink-install
```

---

## 4. 启动前安全要求

1. 机器人必须先趴下。
2. 不要站着启动控制器，因为启动时会释放 `sport_mode` 并发送 stop sentinel。
3. 真机阶段不要按 `3` 进入 RL。
4. 键盘终端提前准备好，异常时按 `9`。
5. `9` 是卸力急停，不是锁死刹车；站立时按下会下沉/趴下。

---

## 5. 启动控制器

终端 1：

```bash
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/home/lidio/Libraries/unitree_sdk2/lib:/home/lidio/Libraries/libtorch-cpu-2.0.1/lib:$LD_LIBRARY_PATH
ros2 launch rl_quadruped_controller mujoco.launch.py
```

必须确认日志：

```text
network_interface: enp7s0, domain: 0
Motion service 'sport_mode' is active; releasing before LowCmd control
Motion service is already deactivated
[MOTOR-MAP] controller[0] FR_hip_joint -> Unitree motor[0]
[MOTOR-MAP] controller[1] FR_thigh_joint -> Unitree motor[1]
...
[VERIFY] joint interface order: FR_hip_joint FR_thigh_joint FR_calf_joint ... RL_calf_joint
Configured and activated rl_quadruped_controller
```

如果出现：

```text
ReleaseMode failed
Motion service ... still active
```

不要继续按键，先停掉排查。

---

## 6. 键盘控制

终端 2：

```bash
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/home/lidio/Libraries/unitree_sdk2/lib:/home/lidio/Libraries/libtorch-cpu-2.0.1/lib:$LD_LIBRARY_PATH
ros2 run keyboard_input keyboard_input
```

按键：

```text
1 -> PASSIVE/卸力
2 -> FIXEDDOWN / FIXEDSTAND 切换
3 -> RL（真机当前禁止）
4 -> 手动 recovery（真机当前禁止）
9 -> HARD STOP/卸力急停
Space -> 清零输入
```

当前真机验证顺序：

```text
机器人趴下
启动控制器
确认日志正常
启动 keyboard_input
先按 9：确认无异常
按 2：测试 FIXEDDOWN
如正常，再按 2：测试 FIXEDSTAND
异常立即 9 或 Ctrl+C
```

---

## 7. 急停语义

当前 `1` 和 `9` 都会强制进入 PASSIVE：

```text
command=1 或 command=9
→ LeggedGymController::update() 最前面捕获
→ current_state = PASSIVE
→ StatePassive: kp=0,kd=0,tau=0
→ HardwareUnitree: PosStopF/VelStopF stop sentinel
```

区别只是语义：

- `1`：普通 PASSIVE
- `9`：明确 HARD STOP

实际效果都是**卸力停止控制**，不是保持当前姿态。

---

## 8. 为什么要释放 sport_mode

Go2 原生控制链：

```text
遥控器/App -> sport_mode -> 电机
```

我们的低层控制链：

```text
ROS2 -> LowCmd -> 电机
```

如果不释放 `sport_mode`，两套控制器可能同时抢电机，引起：

- 异响
- 抖动
- 腿部动作不一致
- LowCmd 被覆盖或部分生效

所以实机 LowCmd 控制前必须：

```text
MotionSwitcherClient::ReleaseMode()
```

释放后，遥控器原生站立/行走通常不再接管；但遥控器按键数据仍可能发布。当前主要急停手段是键盘 `9` / Ctrl+C / 物理断电。

---

## 9. 当前禁止事项

真机当前不要执行：

```text
按 3 进入 RL
按 4 进入 RL_REC
启动 ABS 避障行走
高速前进
障碍物测试
```

原因：真机还没有真实 ray2d 感知输入。仿真中使用 MuJoCo 几何 ray2d，真机必须接入 D435i/LiDAR/深度相机后才能跑 ABS。

---

## 10. 故障处理

| 现象 | 处理 |
|---|---|
| 启动即异响 | 不按键，Ctrl+C 停控制器 |
| 按 2 后腿部异常 | 按 `9`，必要时 Ctrl+C |
| 只有某些腿动 | 停止，检查 `[MOTOR-MAP]` 和 LowState motor index |
| `ReleaseMode failed` | 停止，不继续测试 |
| DDS 崩溃 | 检查 `enp7s0` 是否有 `192.168.123.100/24` |
| ping 不通 | 检查网线、Go2 开机、IP 配置 |

---

## 11. 监控命令

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /imu_sensor_broadcaster/imu --once
ros2 topic list
```

`/joint_states` 发布顺序不一定是控制器内部顺序，控制器内部以日志为准：

```text
[VERIFY] joint interface order: FR_hip_joint FR_thigh_joint FR_calf_joint ... RL_calf_joint
```

---

## 12. 独立状态来源（候选，默认关闭）

原生运动服务释放后 `SportModeState` 停更，odometer 接口会变成 NaN。ABS 支持把导航/速度来源显式切到独立的 D435i/VINS 机身里程计：

```yaml
rl_quadruped_controller:
  ros__parameters:
    navigation_state_source: "vins_body"   # 默认 "odom"
    external_state_topic: "/go2/vins/base_odometry"
    external_state_world_frame: "vins_world"
    external_state_body_frame: "base"
    external_state_timeout_ms: 300          # 暂定值，未实机验证
```

- 候选文件：`config/real_go2_vins_state.candidate.yaml`，**未被任何 launch 引用**，需手动作为额外参数文件传入。
- 该来源的 twist 已经是 base 机体系，代码不再做 world→body 旋转；导航位置与航向来自同一位姿，进入 RL 时捕获一次导航原点。
- 样本缺失/非有限/超出超时/帧不符时 fail-closed（`safetyVeto → PASSIVE`），**不回退** SportModeState 或 Estimator，也不补零。PASSIVE 只是 Unitree 停止哨兵（卸力请求），不保证制动、停稳或保持站立。
- 时间戳回退会被**锁存**：100→90→91 这类序列即使中间回调落在两个控制周期之间，控制器读到的仍然不可用；后续正常消息也不能自动恢复。恢复只能重新进入 RL（新的导航会话），届时清除锁存、重置导航原点并把目标恢复为配置值。重复时间戳只被丢弃，不刷新时龄，也不会触发锁存。
- 该来源没有实测机身高度，因此诊断用正常收尾确认保持不可用。
- 前置条件：安装变换必须先测量（`scripts/go2_vins/BODY_FRAME.md`），转换节点先只读核对。核对 `frame_id`、时间戳单调性和频率时**不要直接启动本 launch**（它会创建 LowCmd 发布器并释放原生模式），应先只启动相机/VINS/转换节点并用独立只读订阅检查。低层停止/交接链仍未闭环，实机运动继续 NO-GO。
