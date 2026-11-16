# Go2 Estimator 慢走动态异常：离线诊断

日期：2026-11-05。仅分析已有只读记录和代码；没有连接机器人、初始化DDS、启动控制器或操作运动服务。

## 结论

已定位一处明确的Estimator测量坐标/运动学错误：Estimator状态位置和速度按世界系传播，但输入的足端位置 `p_body` 及KDL足端相对速度 `J(q)dq` 仍在机体系，直接与世界系状态方程比较；速度约束也没有加入机身角速度造成的足端速度项。代码还把四路足力都高于1的足一律作为接触，使四条腿都以正常接触协方差参与约束。

这是足以解释动态速度偏差的实现缺陷和强候选原因，但现有日志没有真实摆/支撑标签、足端残差/卡尔曼创新量或独立瞬时速度，故不能断言它单独造成两次振荡，也不能从这两次记录判定修复后的精度。本轮只修CSV格式，不改Estimator或接触阈值。

## 数据完整性与时间

| 记录 | 主CSV格式 | 探索窗有效行 | 更新间隔 | 窗内重复LowState | 足力/接触 |
|---|---:|---:|---:|---:|---|
| [第一次](../../../logs/real_go2/20261105_104408_native_slow_walk/RESULT.md) | 23,994完整；启动4行88列（表头89列）排除 | 78–90s：2,400行 | 4.766–5.233ms | 0 | 四腿2,400/2,400行均接触；各腿力最小10/13/13/14 |
| [第二次](../../../logs/real_go2/20261105_104924_native_slow_walk_repeat/RESULT.md) | 21,813完整；启动3行88列（表头89列）排除 | 25–40s：3,000行 | 4.780–5.232ms | 0 | 四腿3,000/3,000行均接触；各腿力最小8/13/12/9 |

两批原始文件均未改。只用字段数等于89的行分析；短列行在更新初段，未参与动态窗。使用的是观察器按200Hz、固定5ms更新时实际读到的最新LowState快照，非完整约500Hz LowState原始流。运动窗内每个Estimator周期都有新LowState序号（多数每次递增2–3），没有把重复快照计成新样本。Estimator更新平均周期约5.000ms；数据不支持固定dt与实际调用周期不匹配是主因。

SportModeState逐消息文件保留独立接收时间。主CSV内的Sport字段只是该Estimator更新时最近一次收到的值；分析按各自接收时间和样本龄理解，不把同行当作同步对，也不把Sport速度当真值。用户报告两次均约2m/10s，仅为近似反馈；窗口为事后探索范围，不用于精确误差率。

运动表现再次出现：第一次78–90s机体系vx标准差0.266m/s、范围−0.645至0.832m/s，Estimator世界系速度积分水平净位移约0.248m；第二次25–40s vx标准差0.272m/s、范围−0.707至0.841m/s，积分水平净位移约0.309m。窗口长度分别12s、15s，边界不是人工事件时刻，净位移不等于行走路程，不据此宣称精确百分比误差。

## 足力、接触与运动学链

1. [Estimator默认阈值为1](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/include/rl_quadruped_controller/control/Estimator.h:19)，实现按 `force < threshold` 判非接触，否则判接触。两次探索窗四腿每一行都大于等于1，所以当前算法确定把四腿同时当接触。接触时 `windowFunc(0.5,0.2)=1`，相应足位置/速度噪声使用名义协方差；非接触时才放大到100。行走时各腿`dq`确有大幅变化（例如大腿角速度跨越约−8至+4 rad/s），因此摆动腿的运动学速度可能作为“足端不动”约束进入滤波器。这是明确的算法行为和相关线索；原始足力单位/标定以及每一时刻哪条腿实际离地都未知，不能据此认定传感器误判，也不据此调整阈值。
2. [Go2硬件插件](../../../quadruped_ros2_control_humble/hardwares/hardware_unitree_mujoco/include/hardware_unitree_mujoco/HardwareUnitree.h:61)使用LowState电机索引恒等映射0–11；接口配置、Estimator索引和四条KDL链均按FR、FL、RR、RL，每腿hip/thigh/calf顺序连接。插件对q/dq不另做符号或单位变换，[URDF](../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/urdf/robot.urdf:402)定义了相应关节轴和腿部几何。代码内部顺序一致，没有发现软件层面的重排错误；仅凭记录不能验证真实Go2固件电机ID和正方向是否与该映射完全一致。
3. [QuadrupedRobot](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/robot/QuadrupedRobot.cpp:71)用KDL计算足端相对 `base` 的位置；[速度](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/robot/QuadrupedRobot.cpp:98)是 `J(q)dq`。这些量由机身基坐标系中的URDF/KDL链得到。Estimator却把它们直接写入 `feet_pos_body_`/`feet_vel_body_`，再与状态方程组合：[位置写入](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/control/Estimator.cpp:189)、[速度写入](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/control/Estimator.cpp:190)、[测量组合](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/control/Estimator.cpp:213)。矩阵将状态位置/速度作为世界系量，并把足位置约束写成 `p_foot_world - p_base_world`、足速度约束写成 `-v_base_world`（[C矩阵](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/control/Estimator.cpp:40)）。
4. 因而按通常的body-to-world旋转R与机体系角速度ω，固定支撑足的关系应为 `p_foot_world = p_base_world + R p_body`，`0 = v_base_world + R(ω × p_body + J(q)dq)`。当前实现缺了位置/相对速度的R变换，也没有`ω × p_body`项。`gyro_`确实从IMU读入，但在Estimator更新方程中没有使用。日志按现有wxyz旋转公式解出的航向约−169°至−156°，不是接近单位旋转；角速度各轴绝对峰值第一次约1.35、第二次约1.89（单位语义未独立确认）。所以忽略坐标变换和旋转项并非可凭“小角度/低角速”排除的差异。若陀螺单位为rad/s，角速度叉乘约0.3–0.5m量级足端偏置可形成与观测速度同量级的项；这是量级估算，不是实测足端速度。

## IMU与时间检查

LowState四元数模长约1；动态窗加速度模长均值约9.59/9.60，z分量均值约9.49m/s²，静止数据与当前“旋转加速度再加世界重力”写法的方向相容。动态窗中按该公式计算的 `R*a+g` 模长均值约1.24/1.25m/s²、峰值约10.5/11.8，可能包含真实躯干加速度及振动；仅凭这些不能确认加速度标定、重力补偿在快速姿态变化中的正确性。四元数坐标语义、gyro单位、IMU与base安装轴关系仍未由这两次走动独立验证。当前200Hz周期和新鲜度没有明显异常。

## CSV修复与验证

没有Sport时主CSV原本为六个Sport原始速度字段只写出五个空字段，后面的Estimator速度整体左移一列，最后一列缺失；这与两次启动行88列/表头89列一致。已补齐第六个空字段；Estimator速度无效时的六个尾部空值也改为完整列宽。

`--self-check`新增无DDS定向检查：有Sport与无Sport时均要求主CSV行与89列表头等宽；无Sport时确认` sport_received=0`、相关原始字段为空；无效Estimator输出也检查总列数。离线目标构建和自检通过。历史CSV只读保留，未尝试改写或按错位字段恢复它们。

## 原因分级与后续边界

- **已定位实现缺陷：**世界系状态与机体系足端测量混用，且足端速度约束漏掉基座角速度项。
- **已确认算法状态：**两次动态窗所有四腿均按阈值1当接触；真实摆动标签未知，因果贡献未测。
- **时间/格式：**运动段89列完整、约5ms更新；无重复快照误用。启动缺列由CSV writer造成，已修复，不是Estimator振荡的来源。
- **仍未排除：**真实motor ID/符号与URDF一致性、原始q/dq及gyro单位、足力物理含义/真实离地时刻、IMU安装轴和四元数动态坐标语义、脚滑/机身振动。Sport仍非真值；无精确位移事件标注和瞬时外部速度。

本轮没有改Estimator算法、模型、控制器、接触阈值或仿真配置，因此仿真行为没有改变。只改只读观察器CSV空字段和离线自检。

**唯一推荐下一步：**做一次限定于这两份原始日志的离线Estimator输入回放，对现有方程与“足位置/速度转世界系并补入`ω × p`”的候选测量方程作同数据对照。按原200Hz记录顺序、原始LowState样本、固定5ms dt和构造时零状态重放；报告两次静止偏差、动态vx振荡和世界速度积分位移。该结果只决定是否值得提交Estimator修正，不以接近用户近似2m为验收目标；在完成前不改生产Estimator、不启用实机Recovery、不安排现场重测。
