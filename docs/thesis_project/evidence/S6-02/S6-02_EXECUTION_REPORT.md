# S6-02 执行报告：初态、覆盖判定与站立收尾

**最新状态：COMPLETE — Director 有条件通过（2026-10-12）。** 通过范围：首周期实测初态日志、运动窗口碰撞/跌倒覆盖判定、当前格式针对性核验、构建及 A/D 各一次诊断，以及 D 一次正常到达后的站立收尾。此前“提交 Director 审阅”等执行记录保留为历史状态。本轮运行数据只作流程核验，不用于成功率或性能优劣结论。

**审阅保留限制：** 周期0由 controller companion 日志保存，A/D JSONL 分别从周期12/15开始，任务开头记录不完整；后续无周期缺口不等于从任务起点完整采集；实测初态不代表组间初态完全相同；安全否定结论仅适用于声明的运动窗口，不能外推到首个策略周期或全采集范围；A运动期跌倒为 UNKNOWN，D保持审计4/8 episode不完整；旧站立 MISSING 的唯一原因未证实；四元数顺序及 MuJoCo 具体命令消费仍有限制；单次结果不证明性能优劣或成功率。

## 完成内容

1. **首个活跃策略周期初态。** `StateRL::writeRtFrame()` 在成功发布 `rl_active=1, rl_step=0` 的运行帧后，同步写出一次 `[ABS-INITIAL-STATE]` companion 记录：session、cycle、帧单调时间、仿真时钟有效性/时间、控制器接口具名顺序、实测关节位置/速度、IMU 姿态与角速度、策略观测的机身线速度，以及可取得的世界 XY/yaw。关节单位为 rad、rad/s；角速度为 rad/s；线速度为 m/s；位置为 m；yaw 为 rad。姿态四元数保留框架原生顺序，未另行解释。

   `s1_run_diagnostic.py` 将该记录写入 `run_context.json`，并要求周期 0 的同周期身份符合生产者写入条件。A/D 均保存了实测周期 0。周期 0 的 companion 记录在控制器发布该周期帧的同一函数中生成；监督 recorder 分别从周期 12、15 才开始持久化 runtime JSONL，所以 `same_cycle_runtime_frame_persisted_by_supervisor=false`。这不是同周期 JSONL 配对，记录中明确保存了 recorder 首周期及 11/14 个周期的覆盖缺口。原 S6-01 A/B 缺失初态没有补造。

   线速度来自 `obs_.lin_vel`：控制器把可用的世界线速度来源（odom 或 estimator）按姿态转换为机身坐标后形成策略输入；不将它称为世界 XY 速度。传感器底层采样是否完全同一时刻仍未知。

2. **碰撞/跌倒覆盖口径。** 分析器现在分别报告全采集范围的 `capture_collision_coverage` 与运动区间 `collision_coverage`。运动区间结果附版本、边界前后快照、PhysicsLoop 连续覆盖、历史溢出和未知接触/姿态重叠证据；模式边沿仍按已记录事实计数，有无效仿真时钟的边沿不进入精确时间指标。全采集字段不再覆盖或否定运动区间结论。跌倒同样区分 `capture_fall_events` 与 `motion_fall_result`；不足以否定时仍为 UNKNOWN。

3. **站立收尾读取。** `read_collision_snapshot_diagnostic()` 以有界短重试读取 seqlock 快照，并区分 ENOENT（MISSING）、文件/长度错误及持续不稳定 seqlock（INVALID）。站立观察遇到非 LIVE 时，新增记录分类、读取原因、快照年龄/序号/仿真时间、共享内存状态及 MuJoCo/controller PID 与返回码。旧 D 的 2.092 s 后 MISSING 记录及其 MuJoCo 在 command 9 后才停止的证据见 [S6-01 执行报告](../S6-01/S6-01_EXECUTION_REPORT.md) 和 [PREFLIGHT](../S6-01/PREFLIGHT.md)。旧 reader 将“不存在”和多次 seqlock 读取不稳定都压成空字节，是一个已确认的诊断歧义；旧运行没有保存 errno、seqlock 序号或共享对象身份，所以这不是旧故障唯一原因的证明。新 D 运行未复现故障并完成站立观察；这证明当前正常路径可用，不证明旧故障的唯一根因。

4. **验证及构建。** `/usr/bin/python3 -m py_compile` 通过；修正后的 `scripts/test_s1_05_safety.py` 通过，包含旧 fixture v3=3864/v4=5760 的准确长度、错误组合拒绝、MISSING/截断/不稳定 seqlock/稳定读取、首态精确配对和运动覆盖边界检查。现有 `scripts/test_s1_05_cross_language.py` 的 C++ MuJoCo 发布器 → 正常 Python reader → 记录/分析定向检查通过，证据为 cross_language_check.json（原始运行/环境明细保留本地，未纳入分享提交）。该 probe 固定绑定 `scene_obstacle.xml` 身份；误用 `scene_random_04.xml` 时按预期以 fingerprint mismatch 拒绝，未启动任务场景运行，也未改 probe 身份。

   控制器包增量构建成功，安装产物为 `quadruped_ros2_control_humble/install/rl_quadruped_controller/lib/rl_quadruped_controller/librl_quadruped_controller.so`。A/D 的运行前进程映射均确认 controller 实际加载该路径。两个运行 overlay 是 `copytree(..., symlinks=False)` 建立的独立实体副本；配置消费日志与路径均绑定对应 overlay。A/D 场景身份、模型指纹、配置、controller/writer PID、session 均由预检与运行来源记录核对。重算派生分析时一次误用系统 Python，因系统 Matplotlib 与 NumPy ABI 不兼容而退出；未改原始记录，随后使用对照工具的 Anaconda 绘图解释器成功重生成分析和时间线。

## 图形预检

当前桌面 `the active DISPLAY value`、`XAUTHORITY=current desktop-session authorization file`。沙箱内 XOpenDisplay 失败；按已有授权在沙箱外运行隐藏 GLFW 窗口预检成功：X 连接、GLFW 初始化、窗口和 OpenGL context 均成功。首次 ctypes 探针因自身 XCloseDisplay 指针签名遗漏而在清理调用中崩溃，进程退出后窗口随之关闭；修正探针签名后预检通过。Renderer 为 `Mesa hardware renderer / OpenGL 4.6 Compatibility Profile, Mesa 23.x`。没有修改授权、系统环境或驱动。

## A/D 实际诊断

共同设置：`scene_random_04.xml`（S2-08-r1）、目标 `(7,0)`、到达半径 0.5 m、运动仿真上限 20 s、墙钟上限 120 s、到达后站立观察 5 仿真秒。两组 controller、安装库和模型路径一致，使用相同初态流程，不代表数值相同或严格重放。

| 组别 | run/session | 实际消费配置 | 终局与目标距离 | 运动窗口和指标 | 安全/收尾 |
|---|---|---|---|---|---|
| A | `[local run ID omitted]` / `11590280830900` | A，`paper_faithful_switch`；E=-0.05，退出=-0.05，无滞回/保持；目标 `(7,0)` | `VALID / COLLISION_TERMINATION`；距目标 3.9274 m | 位移起点 cycle 48 / 23.734 s 至 terminal cycle 704 / 36.856 s；经过 13.122 s，有效连续 12.674 s；有效片段路径 4.486 m，均速 0.354 m/s，峰值区间速度 1.336 m/s。310 个模式边沿，其中 306 个有有效仿真时间；11 个无效时钟周期、0 个策略周期缺口；全记录策略频率 50.008 Hz。 | 碰撞事件为 `RL_calf` 所属 body 与 `ppt_obstacle_15` 接触（geom name 未提供），非足端碰撞终止为 true；运动区间碰撞覆盖完整。足端接触/撞击各记录 1。运动期跌倒为 UNKNOWN：有未确认姿态异常候选跨入统计窗口。碰撞后发 command 1，controller 日志确认切至 PASSIVE；不等于 MuJoCo 已消费。未执行到达后站立观察。 |
| D | `[local run ID omitted]` / `11637190646158` | D，`combined_switch`；E=-0.05、X=-0.08、H=0.30 仿真秒，滞回/保持开启；目标 `(7,0)` | `VALID / ARRIVED`；距目标 0.4835 m | 位移起点 cycle 44 / 29.696 s 至 terminal cycle 486 / 38.536 s；经过 8.840 s，有效连续 8.356 s；有效片段路径 7.790 m，均速 0.932 m/s，峰值区间速度 2.852 m/s。16 个模式边沿全部有有效仿真时间；12 个无效时钟周期、0 个策略周期缺口；全记录策略频率 50.059 Hz。 | 运动窗口碰撞 false、跌倒 false，二者均有完整运动窗口覆盖；足端接触/撞击和非足端碰撞均为 0。全采集范围 `terminal.json` 仍记录 `fall_events=true`，来源是运动开始前的启动阶段事件，不与运动窗口结论冲突。8 个 Recovery episode 均未观察到保持违规；其中 4 个 episode 的时钟覆盖 incomplete，保持审计整体 incomplete。到达后 command 6、controller 站立观察完成 5.010 s；无 PASSIVE/趴下/卸力请求。 |

各组完整初态、来源和事件位于对应 `run_context.json`、`ros_launch.log`、`runtime_record.jsonl`、`terminal.json`；分析结果和时间线位于各自 `analysis/metrics.json`、`analysis/run_results.md`、`analysis/timeline.png`。汇总表位于 comparison.md（原始运行/环境明细保留本地，未纳入分享提交）。A/D 各一次，仅代表流程诊断，不支持算法优劣比较。碰撞/跌倒否定结论仅针对明确的运动窗口，不外推到启动和收尾全采集范围。

## 仍未解决

- S6-01 历史 D 在站立观察 2.092 s 后出现 MISSING 的具体历史原因无法回溯；当时未保存 errno、seqlock 状态、共享对象 inode 和 MuJoCo poll 状态。新 D 完成约 5.010 s 观察，但不足以唯一解释旧故障。
- runtime JSONL 未持久化策略周期 0；首态值由控制器同周期 companion 日志保存，来源清楚但没有同周期 recorder 帧副本。后续若要求单一 JSONL 证据，可在 RL 前让 recorder 就绪，但不影响本次已保存的实测 q/dq 与身份。
- A 运动期跌倒结论 UNKNOWN；D 仅能在有连续覆盖的运动窗口否定碰撞和跌倒，全采集否定结论仍不可用。D 保持审计 4/8 个 episode 时钟覆盖不完整。
- A/D 的实测初态数值和仿真时钟不同，不能作为严格配对实验。单次结果不代表成功率或性能排序。
- 四元数分量顺序、底层传感器物理采样同步性、策略命令被 MuJoCo 消费的直接确认仍未证实。未训练、未做正式实验或实机测试。

## 运行清理

A 与 D `process_facts.json` 均记录 `shutdown_complete=true`、`closeout_complete=true`、ROS/controller 退出码 0，未强制终止；A/D run 后均按 MuJoCo→controller→停止发布器顺序清理。任务结束后的 `/proc` 定向核对未发现 `unitree_mujoco`、`simulate`、`ros2_control_node` 或 `rviz2` 残留；测试共享内存对象已由 probe 清理，未删除来源不明对象。当前项目运行环境已关闭。
