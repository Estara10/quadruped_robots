# D435i VINS → Go2 固定机身参考系

当前阶段：转换程序已实现，安装变换尚未测量，未连接设备、未验证实机转换、不接控制。目标是为电脑 A→B 和后续 ABS 提供同一份机身位姿/速度；不改变 VINS 算法、仿真或 R1 任务。

## 1. 机身原点在哪里

当前部署模型 `quadruped_ros2_control_humble/descriptions/unitree/go2_description/xacro/robot.xacro` 的 `floating_base` 是零平移、零旋转固定连接：`base` 与 `trunk` 重合。本程序直接使用 **base**，不新增另一个不同位置的 `base_link`。`imu_link` 也是模型零变换，但不代表真实机身 IMU 的物理位置已标定。

`xacro/leg.xacro` 与 `const.xacro` 定义四髋关节连接原点：

| 连接原点 | x / m（前正） | y / m（左正） | z / m（上正） |
|---|---:|---:|---:|
| FR | +0.1934 | -0.0465 | 0 |
| FL | +0.1934 | +0.0465 | 0 |
| RR | -0.1934 | -0.0465 | 0 |
| RL | -0.1934 | +0.0465 | 0 |

因此 base 原点是四个髋关节连接原点的平均点，前后间距 38.68 cm、连接原点左右间距 9.30 cm。这里是 URDF 关节连接坐标，不是外壳宽度或四足落地点；不要直接把外侧电机盖边缘间距当作 9.30 cm。

现场寻找基准：机器人关机、稳定趴伏，找前后髋关节轴线的中间截面、左右对称面、髋关节轴线高度；三者交会对应模型 base 的近似物理位置。原点可能在外壳内，可在机身外表面做三个基准的投影/测量线。四髋连接点在硬件上的辨认精度需记录；没有机械基准图时，外壳测量只能算近似，不宣称毫米精度。机身惯性参数中的 `(0.021112, 0, -0.005366)` 是 trunk 质心偏移，**不是本任务原点**。

## 2. 需要测什么

填写测量表，全部用同一刚性机身基准，不用会随站立变化的脚或地面高度推断安装偏移：

| 项目 | 实测值 | 方法/不确定度 |
|---|---|---|
| base 到相机左红外光学参考点，前 x（m） | 待测 | 镜头表面与光心并不完全相同，注明取点 |
| 同上，左 y（m） | 待测 | 不默认相机居中 |
| 同上，上 z（m） | 待测 | 相对髋轴高度，不是离地高度 |
| 相机朝向、俯仰、侧倾 | 待测 | 给出轴映射/角度；正装水平也需明确确认 |
| 支架刚性/是否会相对机身晃动 | 待确认 | 记录固定方式 |

不用拆相机寻找内部 IMU。先测左 IR 光学系 C0 相对 base 的 `T_BC0`，再接设备给出的 `T_IC0`：

`T_BI = T_BC0 × inverse(T_IC0)`。

当前 `d435i_480x270/realsense_stereo_imu.yaml` 的 `body_T_cam0` 就是 VINS 初始 `T_IC0`，来源为本机 SDK 枚举，尚未独立校准。用它推导安装参数时必须保留这一局限。不要使用异常运行在线优化到米级的外参，也不要把 VINS 在线外参直接当作相机到机器人安装位置。

对于**仅用于理解的**正装水平光学系，光学 x右/y下/z前到 base x前/y左/z上通常写作 `R_BC0=[[0,0,1],[-1,0,0],[0,-1,0]]`。实物倾角未知时不能照填此矩阵。SDK gyro 轴、VINS body 和 ROS IMU frame 的实际一致性还需现场只读核对。

测量后另存一份真实安装 JSON，填写 `translation_base_imu_m`、`quaternion_base_imu_xyzw`、来源说明及 `mount_measured=true`。仓库提供 `config/go2_mount.unmeasured.json`，其 null 值不能运行，不能填零绕过测量。`mount_measured` 只表示安装参数已填有测量来源，不表示 VINS 精度或控制安全通过。

## 3. 数据合同与公式

W = `vins_world`，I = `camera_imu_optical_frame`，B = `base`。

- `T_BI` 表示 **I→B**：`p_B = R_BI p_I + t_BI`；t 是 IMU 原点在 base 中的位置。
- 输入 `/vins/imu_propagate` 来自本仓库 CPU/Humble 补丁版本：位姿 `T_WI`，线速度 `v_I` **已经在 I 系**，不再做一次 world→body 旋转。
- 输出 `T_WB = T_WI inverse(T_BI)`，即 `R_WB=R_WI R_BI^T`、`p_WB=p_WI−R_WB t_BI`。
- `omega_B=R_BI omega_I`；`v_B=R_BI v_I−omega_B×t_BI`。减去转动产生的 IMU 参考点速度，原地转向才不会把相机绕圈误当成机身平移。

检查当前 VINS `visualization.cpp` 可知，两种 odometry 的 angular 字段均未赋值，默认零**不是有效角速度**。本程序订阅 VINS 使用的同一相机 IMU，按 header 配对原始 gyro；允许 ≤1 us，仅容纳 VINS double 时间戳往返截断误差，不是图像/IMU同步容差。无对应 IMU 时不发布该样本；不按到达时间或最新一条代替，不插值。缓冲有界，丢弃数量每 5 s 显示。

本版本用**原始 gyro（含零偏）**计算杠杆臂项，未使用 VINS 内部 gyro bias；因此机身速度仍是候选估计。原始 gyro 零偏误差会通过安装偏移进入速度误差，现场需评价。只支持传播输出，不能把优化 odom 换入后宣称同一时间配对已成立（其图像时刻还涉及 td）。

输出 `/go2/vins/base_odometry`（`nav_msgs/Odometry`）：

- header stamp 原样保留；frame=`vins_world`，child=`base`；
- pose：base 在 VINS 固定世界中的位置/姿态；twist：base 参考点在 base 中的线/角速度；
- 协方差没有估计，数组保持零/未指定，**不表示零误差，不可直接作为滤波器可信权重**；
- 不发布 TF，不覆盖原 robot_state_publisher，不自动把世界原点改到当前机身；后续 A→B 只需在已站稳时一次性记录 base 起点与航向，将目标固定转换到 W；
- 没有健康判断、精度保证或机器人停止能力。VINS 跳变会传播到输出；本程序不掩盖、不平滑或修复它。

## 4. 离线验证与之后的只读命令

离线（NumPy/SciPy，不初始化 ROS/DDS）：

```bash
cd /home/lidio/quadruped_robots
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 scripts/go2_vins/test_body_frame.py -v
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 scripts/go2_vins/go2_body_odometry.py --help
# 未测量配置应以退出码 2 拒绝，发生在 rclpy.init 之前：
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 scripts/go2_vins/go2_body_odometry.py \
  --mount scripts/go2_vins/config/go2_mount.unmeasured.json
```

安装参数填写并核对后，复用原相机/VINS 观察环境，电脑单独启动此只读转换程序：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.123.99</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
cd /home/lidio/quadruped_robots
# measured_mount.json 是实际测量生成的文件；当前尚不存在。
PYTHONNOUSERSITE=1 /usr/bin/python3 scripts/go2_vins/go2_body_odometry.py --mount /实际路径/measured_mount.json
```

本轮不执行该命令。后续先只读静止、原生遥控直行/小角度转向核对机身轴、偏移和速度方向。转换节点 Ctrl+C 后检查本次进程退出；相机/VINS/记录器仍由各自会话收尾，不关闭 Unitree 服务。未核对前不接 goto、ABS 或自动运动。

2026-11-09 离线验证：7 项检查通过，覆盖零变换、90°世界航向、光学轴映射、原地转向杠杆臂符号、非平面安装、IMU/传播到达顺序及时间戳舍入、缺失安装参数和非法数值。`--help` 正常退出；未测量 JSON 在 DDS 初始化前退出码 2。初次运行遇到用户目录 NumPy 2.2.6 与 APT SciPy 二进制不兼容，使用 `PYTHONNOUSERSITE=1` 选择系统环境后通过，未改系统包。ROS 在线消息收发、实际安装精度、gyro 零偏影响及停止收尾尚未现场验证；这些测试不证明动态定位准确。

## 5. 后续成熟路径算法

转换核对后，优先评估 **Nav2 Humble Regulated Pure Pursuit (RPP)**：它是成熟的路径跟踪器，输出前进与转向，可用于腿式机器人，符合先朝目标转向、沿曲线前进而非横移的需求。它本身不是从 A/B 两点自动生成所有路径的完整规划器；还需可跟踪的路径、机身位姿、速度反馈及 Nav2 的 TF/costmap 接口。

下一阶段先离线接入/运行成熟跟踪实现，检查 A→B、B→C 的固定目标、朝向与曲率；不把自行写的简化 Pure Pursuit 宣称为已经部署 RPP。平地空场先验证，未知障碍不在此阶段承诺处理。进入原生运动测试时，速度输出只经唯一 SportClient 发送入口，保留人工 stop、状态过期和停止失败锁存；不启动 ABS LowCmd。VINS 异常/超期应停止任务，不能静默换回有已知误差的原生位置。具体停止阈值按观测链测试制定，不在本次转换中猜定。

原生运动 A→B 能验证定位/导航基础，不替代 ABS 实机 Agile/RA/Recovery 与低层停止/交接的验证，也不预设 5 cm 精度已经满足。

参考：

- [ROS 2 Humble Odometry 合同](https://docs.ros.org/en/humble/p/nav_msgs/msg/Odometry.html)：pose 在 header frame，twist 在 child frame。
- [RealSense D435i 轴与 IMU 说明](https://github.com/realsenseai/librealsense/blob/master/doc/d435i.md)：光学轴与设备外参；本机仍以已保存的版本/标定和现场消息核对为准。
- [Nav2 Humble RPP](https://api.nav2.org/nav2-humble/html/md_nav2_regulated_pure_pursuit_controller_README.html)：成熟路径跟踪器及适用范围。
