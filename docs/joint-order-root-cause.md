# 关节顺序映射链完整分析与修复

> **日期**: 2026-06-01  
> **问题**: Agile Policy 在 MuJoCo 仿真中不走直线（偏左）  
> **根因**: `robot_control.yaml` 关节顺序与训练不匹配  
> **状态**: ✅ 已修复，待测试验证

---

## 1. 问题背景

Go2 机器人加载训练好的 Agile Policy 后，在 MuJoCo 仿真中能走但偏左，始终不走直线。经过多次修改（commands 映射、timer 方向、接触力重排、偏航修正等）均未解决。

## 2. 完整映射链分析

数据从 MuJoCo 物理引擎 → 机器人身上执行器，完整链路如下：

```
MuJoCo XML (go2.xml)
    ↓ actuator/motor 顺序
DDS motor_state (unitree_sdk2)
    ↓ 直接索引映射 HardwareUnitree::read()
joint_position_[0..11] (硬件接口)
    ↓ ros2_control.xacro 定义顺序 + 名称
ROS2 State Interfaces (按名称匹配)
    ↓ robot_control.yaml joints 参数顺序
控制器 ctrl_interfaces_ 向量
    ↓ StateRL::getState() 直接索引读取
obs_.dof_pos → policy 推理 → action
    ↓ setCommand() 直接索引写入
DDS motor_cmd → MuJoCo ctrl[] → 物理执行
```

### 2.1 各层实际顺序

| 层级 | 文件 | 关节顺序 | 备注 |
|------|------|---------|------|
| MuJoCo XML | `unitree_mujoco/unitree_robots/go2/go2.xml` | **FR**, FL, RR, RL | `<actuator>` 中 motor 定义顺序 |
| 训练 URDF | `ABS/.../resources/robots/go2/urdf/go2.urdf` | **FR**, FL, RR, RL | IsaacGym `get_asset_dof_names()` 返回此顺序 |
| DDS motor_state | `unitree_sdk2_bridge.h` L148-154 | **FR**, FL, RR, RL | 直接映射 `mj_data_->sensordata[i]` |
| ros2_control.xacro | `descriptions/.../xacro/ros2_control.xacro` | **FR**, FL, RR, RL | 硬件接口导出顺序 |
| ❌ robot_control.yaml (旧) | `descriptions/.../config/robot_control.yaml` | **FL**, FR, RL, RR | **与上方全部不匹配！** |
| ✅ robot_control.yaml (新) | 同上 | **FR**, FL, RR, RL | 已修复 |

### 2.2 触地传感器

| 层级 | 顺序 |
|------|------|
| MuJoCo touch sensor | FR_touch, FL_touch, RR_touch, RL_touch |
| DDS foot_force | FR, FL, RR, RL |
| ros2_control.xacro | FR, FL, RR, RL |
| ✅ robot_control.yaml (新) | FR, FL, RR, RL |

训练 `feet_indices` 顺序（`compute_observations` 中 contact）：**FR, FL, RR, RL**（与 URDF body 顺序一致）

## 3. 错误影响分析

旧 YAML 用 FL-first 顺序：

```
YAML joints: [FL_hip, FL_thigh, FL_calf, FR_hip, ..., RR_calf]
              idx 0    idx 1     idx 2     idx 3         idx 11
```

控制器按 YAML 顺序请求接口：

```
ctrl_interfaces_[0] → "FL_hip_joint/position" → hw 中 FL_hip 数据 = FL_hip 实际值
ctrl_interfaces_[3] → "FR_hip_joint/position" → hw 中 FR_hip 数据 = FR_hip 实际值
```

但由于 YAML 索引命名（idx0=FL，idx3=FR）与训练期望（idx0=FR，idx3=FL）不同：

### 3.1 观测错位

```
dof_pos[0] = motor_state.q[0] = "FL_hip" 值 = FL_hip 实际位置
训练期望: dof_pos[0] = FR_hip 实际位置  ← ❌ 错误！

dof_pos[3] = motor_state.q[3] = "FR_hip" 值 = FR_hip 实际位置
训练期望: dof_pos[3] = FL_hip 实际位置  ← ❌ 错误！
```

**FR 和 FL 腿的观测数据互换了！**

### 3.2 动作错位

```
policy 输出 action[0]（FR_hip 目标）→ motor_command.q[0] → 
  setCommand idx 0 → "FL_hip_joint" → motor_cmd[0] → MuJoCo FL_hip

实际效果: FR_hip 的策略动作施加到了 FL_hip 上！ ← ❌ 错误！
```

### 3.3 Contact 错位

旧 contact_map = {0,2,1,3} 将硬件 [FL,RL,FR,RR] 映射到 [FL,FR,RL,RR]，
但训练期望 [FR,FL,RR,RL]。所以 contact 也是 FR↔FL 互换的。

### 3.4 为什么不翻车但偏左？

FR 和 FL 是左右对称腿，将它们的观测+动作互换后：
- 策略的 gait pattern 基本结构仍然有效（对称性）
- 但 FR 腿的具体动力学参数（正负方向、摩擦等）与 FL 不完全对称
- 导致左侧和右侧腿力不平衡 → 机器人持续偏左

## 4. 修复内容

### 4.1 robot_control.yaml（核心修复）

```yaml
# 修改前（❌ FL-first）
joints: [FL_hip, FL_thigh, FL_calf, FR_hip, ..., RR_calf]
stand_pos: [0, 0.8, -1.5, 0, 0.8, -1.5, ...]  # FL-first
foot_force_interfaces: [FL, RL, FR, RR]
feet_names: [FL_foot, FR_foot, RL_foot, RR_foot]

# 修改后（✅ FR-first，匹配训练 + MuJoCo + DDS + xacro）
joints: [FR_hip, FR_thigh, FR_calf, FL_hip, ..., RL_calf]
stand_pos: [0, 0.8, -1.5, 0, 0.8, -1.5, ...]  # FR-first
foot_force_interfaces: [FR, FL, RR, RL]
feet_names: [FR_foot, FL_foot, RR_foot, RL_foot]
```

### 4.2 RlQuadrupedController.h

C++ 中 `stand_pos_` / `down_pos_` 默认值注释更新为 FR-first。

### 4.3 StateRL.cpp + StateRLRec.cpp

- **删除** `contact_map = {0, 2, 1, 3}` — 所有层级已对齐，不再需要重排
- 更新 DEBUG/TARGET_Q/SENSOR_Q 日志标签为 FR-first

## 5. 修改文件清单

| 文件 | 改动类型 |
|------|---------|
| `descriptions/unitree/go2_description/config/robot_control.yaml` | joints, stand_pos, down_pos, feet_names, foot_force 改为 FR-first |
| `controllers/rl_quadruped_controller/src/RlQuadrupedController.h` | stand_pos_ / down_pos_ 注释 |
| `controllers/rl_quadruped_controller/src/FSM/StateRL.cpp` | 删除 contact_map，更新日志 |
| `controllers/rl_quadruped_controller/src/FSM/StateRLRec.cpp` | 删除 contact_map，更新注释 |

## 6. 验证方法

启动仿真后，检查 ROS2 日志输出：

```
[VERIFY] default_dof_pos: 0.0000 0.8000 -1.5000 (first 3 joints = FR hip/thigh/calf)
[VERIFY-CONTACT] forces=[...] (FR, FL, RR, RL) contact=[1 1 1 1]
[DEBUG-0] TARGET_Q: FR=... FL=... RR=... RL=...
[DEBUG-0] SENSOR_Q: FR=... FL=... RR=... RL=...
```

关键验证点：
1. `default_dof_pos` 前 3 个是 FR hip/thigh/calf = 0, 0.8, -1.5
2. Contact 顺序标注为 (FR, FL, RR, RL)
3. 机器人能走直线，不再偏左

## 7. 经验教训

1. **关节顺序一致性至关重要**：训练、仿真、硬件接口、控制器配置四层必须严格一致
2. **不要依赖名称匹配做隐式重排**：ROS2 controller_interface 按名称匹配，但 C++ 代码用索引直接读取，名称顺序（YAML）决定实际数据排列
3. **对称性不能掩盖错误**：FR↔FL 互换因对称性不会导致翻车，但会产生微妙的不对称行为
4. **不要轻易加 hack**：contact_map 这种临时的索引重排，掩盖了真正的系统性错误
