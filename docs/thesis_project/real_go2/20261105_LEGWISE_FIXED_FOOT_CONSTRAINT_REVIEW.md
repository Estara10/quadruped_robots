# Go2慢走逐腿固定足约束对照

日期：2026-11-05。只处理已保存的两次原生遥控慢走记录；未连接机器人、初始化DDS、启动控制器或执行现场实验。

## 问题与方法

要检查的是四腿各自按固定支撑足假设反推的机身速度是否彼此相容，以及分歧是否随Estimator振荡、足力和关节运动同步变化。候选速度只由每行LowState输入与Go2 URDF/KDL计算，不用Estimator输出参与。Estimator A/C仅作为同一时刻的对照曲线。

若足端在世界系固定：

```text
0 = v_base_world + R * (omega_body × p_body + J(q) dq)
v_base_world_i = -R * (omega_body × p_body_i + J_i(q) dq_i)
```

这里`p_body`是base原点到足端的位置，`Jdq`是足端相对base原点的关节诱导线速度，均由FR/FL/RR/RL各自KDL链给出并表达在base轴中。固定足关系反推的是**base原点**线速度；`Jdq`描述的端点是足端。`R`来自记录中的wxyz四元数，按代码约定作为body-to-world旋转。无角速度候选保留为`-Jdq`（body系）及其`R`旋转结果，用来观察不依赖gyro轴语义的腿间分歧；它只对应忽略机身转动的截断关系。

body系角速度候选进一步假定LowState gyro与base轴重合，且单位为rad/s；Go2 EDU本机IMU到base轴映射仍未知。因此报告将无omega结果作为主要独立腿间对照，带omega结果作为条件性比较。两种情况下，每条腿的速度都是“若该足固定时的候选值”，不是真实机身速度、不是真值，也不据此挑选支撑腿。

逐行处理观察器已有200Hz快照，按原时间戳对齐足力、q/dq及历史A/C速度，没有补入约500Hz消息、插值或平滑。第一次78–90s/运动前35–45s/停止后100–110s，重复25–40s/运动前10–20s/停止后45–55s均固定沿用先前探索窗口，边界不精确。候选计算复用`go2_estimator_replay.cpp`中的`QuadrupedRobot` KDL与`Estimator::footMeasurementToWorld`坐标函数（`Estimator.cpp:152–160`），不读取Estimator状态；腿间距离和分歧指标只比较候选之间，不把多数腿当作支撑标签。

## 定量结果

“两两速度差”是每个时刻六组腿对的三维欧氏速度差；均值和P95汇总整个窗口的六组差值。`vx`腿间SD是在每个时刻对四条腿求标准差后再汇总。单位均为m/s。

| 记录/窗口 | 候选 | 两两速度差均值 / P95 | 每时刻vx腿间SD均值 / P95 |
|---|---|---:|---:|
| 第一次运动前35–45s | 无omega | .0279 / .0529 | .0110 / .0193 |
|  | 有omega | .0284 / .0537 | .0111 / .0192 |
| 第一次动态78–90s | 无omega | .4064 / 1.9507 | .2306 / 1.0070 |
|  | 有omega | .4087 / 1.9657 | .2312 / 1.0100 |
| 第一次停止后100–110s | 无omega | .0217 / .0399 | .0089 / .0155 |
|  | 有omega | .0222 / .0409 | .0090 / .0157 |
| 重复运动前10–20s | 无omega | .0217 / .0400 | .0088 / .0155 |
|  | 有omega | .0221 / .0408 | .0089 / .0157 |
| 重复动态25–40s | 无omega | .4431 / 2.0242 | .2539 / 1.0069 |
|  | 有omega | .4420 / 2.0272 | .2526 / 1.0091 |
| 重复停止后45–55s | 无omega | .0225 / .0416 | .0094 / .0163 |
|  | 有omega | .0229 / .0422 | .0095 / .0165 |

静止窗口中各腿候选接近；两次走动窗口的腿间分歧比静止明显增大。带omega与无omega的总体差异很小；这次数据中补入角速度项没有消除腿间冲突。动态窗口里无omega的body `vx`候选按腿统计如下，格式为均值±标准差 [最小,最大]；`vy`为同一候选的横向均值±标准差。图和`metrics.json`另含带omega候选、三轴极值及各腿配对结果。

| 记录/腿 | vx候选 m/s | vy候选 m/s | 足力最小原始值 |
|---|---:|---:|---:|
| 第一次 FR | −.004±.525 [−2.527,1.047] | −.008±.085 | 10 |
| 第一次 FL | .010±.514 [−2.364,1.391] | −.004±.123 | 13 |
| 第一次 RR | −.002±.476 [−2.122,1.010] | .003±.123 | 13 |
| 第一次 RL | .007±.495 [−2.500,1.121] | .009±.088 | 14 |
| 重复 FR | −.004±.535 [−2.451,.998] | −.006±.092 | 8 |
| 重复 FL | .004±.539 [−2.369,1.234] | −.003±.134 | 13 |
| 重复 RR | −.004±.495 [−2.095,1.446] | .005±.135 | 12 |
| 重复 RL | .002±.524 [−2.387,1.714] | .003±.104 | 9 |

两次动态窗口中所有四路原始足力始终大于等于1，故现有阈值列每行仍标为接触。与此同时，四足关节速度范数有重复脉冲，P95约4.5–5.8 rad/s（第一次）和5.4–5.7 rad/s（重复）。图中可见候选速度分歧峰反复出现，时间上与关节速度脉冲、原始足力周期变化及Estimator A/C的机身vx振荡大体相邻；足力谷值仍高于阈值1。

分歧指标与Estimator C机体系`|Δvx|/Δt`的同期Pearson相关系数为第一次约0.616、重复约0.592（无omega）；带omega分别约0.616和0.593。与`|C vx|`的相关为约0.50和0.48。候选本身没有使用Estimator，但该同步相关不能证明因果；Estimator C本来就把四腿固定足观测融合进速度，输出与约束分歧有结构性联系。

逐腿候选、A/C速度、四路原始足力、阈值1标记、腿关节速度范数及腿间vx分歧均以原始采样曲线绘制，没有平滑：

- [第一次，78–90s](../../../logs/real_go2/20261105_leg_constraint_review/first_leg_constraints.png)
- [重复，25–40s](../../../logs/real_go2/20261105_leg_constraint_review/repeat_leg_constraints.png)
- [逐时刻指标和配对差值](../../../logs/real_go2/20261105_leg_constraint_review/metrics.json)
- [逐腿固定足候选原始派生CSV目录](../../../logs/real_go2/20261105_leg_constraint_review/)

## 判断边界

1. **静止一致性：**是。各腿候选的两两差值均值约0.02–0.03m/s，P95约0.04–0.05m/s，明显小于运动窗。
2. **走动冲突：**是。两次运动中四腿候选反复分离，三维两两差P95约1.95–2.03m/s；vx候选逐腿出现约−2.5到+1.7m/s的周期变化。no-omega结果也有同样冲突，所以这一观察不依赖未确认的gyro轴映射。
3. **与Estimator/足力/关节运动关系：**时间上有明显重复对应。四足阈值标记始终为接触，但候选分歧峰与关节速度脉冲和Estimator振荡相邻，原始足力也呈周期起伏而未跌破1。这支持“把四腿始终当固定足会同时融合互不相容约束，并可能参与速度振荡”这一解释；它仍是强相关线索，不是已证实的唯一根因。脚滑、柔顺、q/dq符号/轴、IMU与base几何、采样不同步及人工窗口边界均可能贡献。足力大于1并不证明四足同时静止支撑；候选多数接近也不能当作地面接触真值。

现有日志没有视频或独立脚相标签，无法判断每个冲突脉冲具体哪条腿处于摆动、落地、滑动或受载。要完成这一区分，需要逐帧覆盖第一次观察器时间78–90s及重复观察器时间25–40s的视频，画面同时看见四足与地面，且视频时间能映射到CSV的`update_time_s`；逐腿标注“可见落地/疑似承重、明确离地、遮挡不确定”，并标出落地/抬脚时刻。不要只抽选速度极值片段。若原视频不存在，才需要另行安排一次同类原生遥控只读观察；本轮没有安排或执行。

没有发现足以在本轮修改生产Estimator或接触阈值的新实现错误。阈值、Estimator模式、模型观测、控制器和仿真配置都未改；历史A/C回放一致性未重跑，A/C仅从既有回放结果读取对照。逐腿候选只是新的离线输出，不能作为真实运动速度或支撑状态标注。

## 复现

仓库根目录执行。只重建扩展后的离线回放工具；不启动Estimator控制器或DDS。输入为两批原始`estimator.csv`，输出在新目录：

```bash
rtk cmake --build /tmp/go2_estimator_build/controller --target go2_estimator_replay --parallel 2
rtk env LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu /tmp/go2_estimator_build/controller/go2_estimator_replay \
  --leg-constraints --input logs/real_go2/20261105_104408_native_slow_walk/estimator.csv \
  --urdf quadruped_ros2_control_humble/descriptions/unitree/go2_description/urdf/robot.urdf \
  --output logs/real_go2/20261105_leg_constraint_review/first_leg_constraints.csv
rtk env LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu /tmp/go2_estimator_build/controller/go2_estimator_replay \
  --leg-constraints --input logs/real_go2/20261105_104924_native_slow_walk_repeat/estimator.csv \
  --urdf quadruped_ros2_control_humble/descriptions/unitree/go2_description/urdf/robot.urdf \
  --output logs/real_go2/20261105_leg_constraint_review/repeat_leg_constraints.csv
rtk env MPLCONFIGDIR=/tmp/go2_leg_constraint_mpl python3 logs/real_go2/20261105_leg_constraint_review/analyze_leg_constraints.py
```
