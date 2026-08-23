# Go2 Quadruped Robot — ABS Reproduction & Deployment

本仓库用于复现 **ABS (Agile But Safe)** 论文，并将双策略避障框架迁移到 Unitree Go2：

```text
MuJoCo 仿真 + 几何 ray2d → ROS2 Humble + LibTorch → Go2 实机 LowCmd
```

Paper: https://agile-but-safe.github.com/  
Reference code: https://github.com/LeCAR-Lab/ABS

---

## 当前状态

项目状态、Acceptance和当前任务只以 [`docs/CURRENT_STATE.md`](docs/CURRENT_STATE.md) 为准。

当前处于 **Phase 1 — MuJoCo Simulation Validation**，正式任务为 **P1-01（尚未开始）**。历史仿真结果均按新实验协议标记为 `LEGACY / NON-ACCEPTANCE`。

真机当前只允许 `PASSIVE`、`FIXEDDOWN`、`FIXEDSTAND` 和 software dry-run；ABS/RL real test 为 **NO-GO**。

---

## 目录

| 路径 | 作用 |
|------|------|
| `quadruped_ros2_control_humble/` | ROS2 Humble 控制器、硬件接口、Go2 描述 |
| `unitree_mujoco/` | MuJoCo 仿真器与 Go2 场景 |
| `ABS/` | ABS 论文原始训练/部署参考 |
| `scripts/` | 启动、评估、数据采集脚本 |
| `仿真部署手册.md` | MuJoCo + ROS2 仿真说明 |
| `quadruped_ros2_control_humble/controllers/rl_quadruped_controller/doc/real_go2_deployment.md` | Go2 真机部署说明 |
| `scene.txt` | 场景列表和推荐测试顺序 |
| `command.txt` / `命令.txt` | 常用命令速查 |
| `lab_notes/` | 实验记录 |

---

## 仿真快速启动

### 避障自动演示

```bash
cd ~/quadruped_robots
source /opt/ros/humble/setup.bash
source quadruped_ros2_control_humble/install/setup.bash
export LD_LIBRARY_PATH=/home/lidio/Libraries/unitree_sdk2/lib:/home/lidio/Libraries/libtorch-cpu-2.0.1/lib:$LD_LIBRARY_PATH
MUJOCO_SCENE=scene_obstacle.xml ./scripts/launch_abs_obstacle.sh
```

### 平地直线测试

```bash
MUJOCO_SCENE=scene_flat.xml ./scripts/launch_abs_obstacle.sh
```

### 手动 FSM 测试

```bash
AUTO_RL=false MUJOCO_SCENE=scene_flat.xml ./scripts/launch_abs_obstacle.sh
```

另开终端：

```bash
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run keyboard_input keyboard_input
```

按键：

```text
2 -> FIXEDDOWN
2 -> FIXEDSTAND
3 -> RL（仿真可用，真机暂不按）
4 -> 手动 RL_REC
1 -> PASSIVE/卸力
9 -> HARD STOP/卸力急停
```

---

## 真机快速入口

真机测试前必须把 `ros2_control.xacro` 切到：

```xml
<param name="domain">0</param>
<param name="network_interface">enp7s0</param>
```

并重新编译：

```bash
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_description --symlink-install
```

真机只测站立/趴下：

```bash
sudo ip addr add 192.168.123.100/24 dev enp7s0  # File exists 可忽略
ping -c 3 192.168.123.161

cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/home/lidio/Libraries/unitree_sdk2/lib:/home/lidio/Libraries/libtorch-cpu-2.0.1/lib:$LD_LIBRARY_PATH
ros2 launch rl_quadruped_controller mujoco.launch.py
```

日志必须出现：

```text
network_interface: enp7s0, domain: 0
Motion service 'sport_mode' is active; releasing before LowCmd control
Motion service is already deactivated
[MOTOR-MAP] controller[0] FR_hip_joint -> Unitree motor[0]
[VERIFY] joint interface order: FR_hip_joint FR_thigh_joint FR_calf_joint ... RL_calf_joint
```

然后开键盘，只按 `2` 测 FIXEDDOWN；异常按 `9` 或 Ctrl+C。  
**真机不要按 `3` 进入 RL，直到真实 ray2d 感知接入。**

---

## 构建

```bash
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
colcon build --packages-select hardware_unitree_mujoco rl_quadruped_controller keyboard_input go2_description --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch
```

配置文件只改 Go2 描述时：

```bash
colcon build --packages-select go2_description --symlink-install
```

---

## 关键约定

- 控制器关节顺序：`FR, FL, RR, RL`，每条腿 `hip, thigh, calf`
- 策略导出顺序为 ROS1/FL-first，通过 `policy_joint_order: ros1_fl_fr_rl_rr` remap
- 真机 DDS motor mapping 当前假设：`FR,FL,RR,RL -> motor[0..11]`
- 仿真 ray2d 使用 MuJoCo 几何射线；真机 ABS 需要真实相机/LiDAR ray2d
- `1` 和 `9` 当前都是卸力急停到 PASSIVE，不是锁死刹车
