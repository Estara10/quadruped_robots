# S2-01 Execution Report — A 组单阈值实现与功能基线核验

**状态：COMPLETE — Director 有条件通过（2026-10-10）。**通过限于 A 配置/生产路径核验及既有 legacy 行为与接触来源证据；不表示新配置已取得运动周期证据，也不表示 S2 阶段整体完成。

## 结论摘要

- 此前有效运行采用 legacy `paper_faithful_switch` 配置，实际日志和周期帧确认 `E=-0.05` 的单阈值行为，并观察到真实 RA 驱动的 6 次 Agile→Recovery 与 6 次 Recovery→Agile。该运行没有消费本轮新增的 `abs.switching` 配置入口。
- MuJoCo 仿真到 controller loaned foot-force 名称的四段顺序相互对应；PASSIVE／固定站立准备阶段记录到 `foot_force/FR, FL, RR, RL`，与 controller 接触转换一致，所以允许进入 RL。
- 唯一实际场景诊断完整记录后以障碍碰撞终止。不是到达或无碰撞结果。控制接口曾写出 6 个 Recovery episode 的首条命令；MuJoCo 是否消费命令仍 UNKNOWN。
- 本次策略整体频率约 `50.04 Hz`。停止策略确认没有成功，尽管清理时 controller 和仿真进程组均以退出码 0 关闭。

## 运行前固定选择

参数在 [S2-01_RUN_PLAN.md](S2-01_RUN_PLAN.md) 中先行记录。选择既有 `scene_obstacle.xml` 和目标 `(7.0, 0.0) m`，目标距离长于过去很快结束的 1 m 诊断，可在既定静态障碍场景中观察更多自然 RA 决策周期；这不保证触发切换。到达半径 `0.50 m`，运动仿真上限 `6 s`，墙钟保护 `50 s`。没有暂停、重置、参数扫描、热更新、RA 注入或关闭安全检查。A 组 `E=-0.05`，Recovery hold 不启用。

## 此前运行的 legacy 候选、生产路径与针对性检查

- **运行时旧配置说明（本轮迁移前）：**当时部署 YAML 显式设置 `switching_mode: "paper_faithful_switch"` 和 `ra_threshold: -0.05`；这三个 legacy 字段已在本轮移出当前配置，见 [当前 ABS config.yaml](../../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml)。前次运行的 controller 通过 `effectiveExitThreshold()` 将退出阈值取为进入阈值，并把真实 `ra_value_` 传入生产 `stepSwitching()`；该分支按 `RA >= E` / `RA < E` 并清零 hold 计数。
- **旧运行配置与日志事实：**前次运行时 `recovery_hold_steps: 30` 是旧配置字段，paper 候选不以它延迟退出。周期帧 entry/exit 均约 `-0.0500000007`、`switching_mode=1`；该次 `[RA-REC]` 入口原始日志仍打印“保持步数=30”，不能作为 A 组 hold 生效证据。当前 A 配置已删除该字段，当前入口日志也已修正。历史原始日志保留不改。
- 源码检查了状态进入时 `in_recovery_` 与 hold 计数重置。复用实际生产 `RASwitchingLogic::stepSwitching()` 的定向离线检查，手工 RA 输入覆盖 `< E`、`= E`、`> E`、初始化/重入、NaN/±Inf 和 paper 候选不受 active hold 影响，结果 `PASS (292 checks)`。这些人工 RA 值只核验切换函数，不是模型推理或仿真证据。
- 增量构建命令：`rtk bash -lc 'source /opt/ros/humble/setup.bash && cd quadruped_ros2_control_humble && colcon build --packages-select rl_quadruped_controller --cmake-args -DBUILD_TESTING=ON'`；结果 `Finished <<< rl_quadruped_controller`。不改切换条件、阈值、模型、观测、动作、频率或 Recovery 路径。
- 为在 RL 前观察 controller 实际收到的接口，在 `RlQuadrupedController::on_activate()` 增加仅输出接口索引/名称的 `[FOOT-LOANED]` 日志；诊断脚本在固定站立准备阶段校验四项必须严格匹配，否则不发 command 3。脚本同时按目标参数核验 config，不再把目标固定写死为 1.0 m。

## 足接触四段来源核验

| 段 | 证据与结果 |
|---|---|
| MuJoCo 具名传感器 → SDK 数组 | `unitree_mujoco/unitree_robots/go2/go2.xml` 的 touch sensor 顺序为 FR、FL、RR、RL；`simulate/src/unitree_sdk2_bridge.h` 从 `sensordata[dim_motor_sensor_ + 16 + i]` 顺序读取，并原序写入 `lowstate->msg_.foot_force()[i]`。仿真桥在 MuJoCo 中建立了具名对应。 |
| SDK 数组 → ROS state interface | `hardware_unitree_mujoco/HardwareUnitree.cpp` 原序读取四个 SDK 槽位；导出接口时按 `info_.sensors[1].state_interfaces[i].name` 与数组同索引绑定。Go2 `ros2_control.xacro` 声明顺序是 FR、FL、RR、RL。 |
| Controller 实际 loaned 名称 | 本次运行 controller PID `40029` 在发出 RL 请求前记录索引 `0..3 = foot_force/FR, foot_force/FL, foot_force/RR, foot_force/RL`；诊断门禁验证通过。原始证据见本次 `ros_launch.log` 和 `run_context.json` 的 `preflight.foot_force_loaned_interfaces`。 |
| Controller → 策略排列 | RL controller 以 FR、FL、RR、RL 接口顺序构造 contact 向量，生产 observation helper 按 `[1,0,3,2]` 转为策略 FL、FR、RL、RR。S1-06 的唯一标记 helper 检查已获 Director 有条件通过。 |

本结论适用于本次 MuJoCo/ROS 仿真接口来源；不外推为实机 SDK 传感器值的物理语义验证。

## 启动尝试与实际诊断

### 启动失败尝试（保留）

`6725394299864997989fc2862c9bb429` 在任何仿真进程启动前因命令参数漏掉 workspace 路径段，overlay 核验失败（0 策略帧；`mujoco_started=false`、`ros_launch.started=false`）。错误路径已定位并在唯一实际诊断前纠正。失败记录保留在 [该目录](diagnostics_20261010/6725394299864997989fc2862c9bb429/run_context.json)，没有把它当成有效运行结果。

### 唯一实际场景诊断

- Run ID：`a5b88cf10d4748d4b15ce1b9566e524e`；实际调用 `rtk bash -lc 'DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority python3 scripts/s1_run_diagnostic.py ...'`；场景 `scene_obstacle.xml`；目标 `(7.0,0.0) m`；到达半径 `0.50 m`；候选 `paper_faithful_switch`；配置 `entry=exit≈-0.05`；运动仿真上限 `6 s`、墙钟保护 `50 s`。
- 实际 ROS launch 参数指向本工作区 `install/go2_description/share/go2_description/config/robot_control.yaml`；同一 overlay 的 `abs/config.yaml`、Agile `policy.pt`、RA `ra_value.pt` 和 Recovery `policy.pt` 均由本次 `ros2_control_node` PID `40029` 的来源日志/配置解析核对。唯一 controller writer PID 为 `40029`，RT frame session 为 `6813863628793`；逐帧来源绑定通过。记录器先于 MuJoCo 启动。
- 共保存 171 个连续策略帧，`rl_step_gaps=0`。全采集有 166 个有效仿真时钟帧、5 个 STALE 帧，没有把缺时钟样本插值。周期统计按同 session 且相邻 `rl_step` 的 monotonic timestamp 计算：整体频率 `50.0374 Hz`；间隔中位数 `20.030 ms`、P95 `25.781 ms`、范围 `3.485–39.275 ms`。这是本次观测结果；没有无记录对照，不能归因或排除记录开销。
- 运动起点 cycle 51、`sim_time=24.296 s`；终止 cycle 207、`sim_time=27.416 s`，含边界运动时段为 `3.120 s`。终止原因是 PhysicsLoop 权威累计障碍接触：机器人 geom `RL` 对 obstacle geom id `2`（源记录没有保存障碍名称），episode 覆盖 `27.400–27.422 s`。到达未发生，终点距目标 `2.485 m`。该运动窗口碰撞覆盖标记完整，跌倒规则未检出事件；这不等于其他场景、规则或全采集范围的否定结论。
- 按真实策略帧，6 次 Agile→Recovery 进入周期为 `117, 124, 127, 170, 174, 184`；对应 RA 为 `0.1218, 0.0836, 0.0246, 0.0843, 0.1445, -0.0442`，都满足 `RA >= -0.05`。6 次 Recovery→Agile 退出周期为 `119, 126, 131, 173, 177, 190`；RA 为 `-0.1505, -0.0638, -0.1427, -0.0598, -0.5665, -0.2910`，都满足 `RA < -0.05`。RA 来自运行中的部署模型；没有替换模型输出。转换及动作来源见 [switch_events.json](diagnostics_20261010/a5b88cf10d4748d4b15ce1b9566e524e/switch_events.json)。
- 六个 episode 均有 `[ABS-POLICY-CMD] recovery_command_issued`，来源 session 为 `6813863628793`、策略周期对应 `117,124,127,170,174,184`，episode id 为 `1..6`。这证明命令写入控制接口，不证明 MuJoCo 消费或动作物理生效。
- 指标为有效连续窗口内 Recovery 时间 `0.384 s`、有效运动仿真时间 `2.966 s`、Recovery 占比 `12.95%`；一个 Recovery 区间因 STALE 时钟缺口被标为删失，不跨缺口累计。路径长度 `4.213 m`，平均路径速度 `1.420 m/s`。这些指标只描述本次诊断。
- 无 controller safety veto；运动阶段确有障碍碰撞。收尾发送 PASSIVE 请求并观察到全局 hard-stop 日志，但 `strategy_stop_confirmed=false`，请求后至停止观察的输出区间约 `0.25 s`。随后本次 ROS launch/controller、MuJoCo 进程组退出码均为 `0`、未强杀，`closeout_complete=true`。因此进程环境已关闭，但策略停止请求没有单独确认；MuJoCo 消费命令仍 UNKNOWN。

证据文件：原始 `runtime_record.jsonl`、`ros_launch.log`、`mujoco.log`、`run_context.json`、`terminal.json`、`process_facts.json`；派生 [metrics.json](diagnostics_20261010/a5b88cf10d4748d4b15ce1b9566e524e/metrics.json)、[timeline.png](diagnostics_20261010/a5b88cf10d4748d4b15ce1b9566e524e/timeline.png) 和 [run_results.md](analysis_20261010/run_results.md)。原始帧和终局文件未改写。

**记录标签说明：**复用的 S1 bounded recorder 在原始 `run_context.json` 的静态 `classification` 字段仍写着“S1-05”；其余 source/config/session/terminal 字段绑定到本次 run。为保持原始记录不变，未改写该字段；本报告、预先计划、run_id 与运行来源共同标明此为 S2-01。时间线输出已用 `--task-label S2-01` 重新生成，避免旧图题混淆。

## 结果与保留事项

- **此前运行直接证实：**旧 legacy 候选的仿真 RA 边界符合 A 单阈值规则；仿真足接触四段顺序相符；观察到真实模型驱动切换、Recovery 命令接口写出、障碍碰撞终局、完整终止载荷、频率统计、指标和周期时间线。该 run 不是新配置入口的消费证据。
- **代码推断：**旧 paper 分支没有滞回或保持；旧 `recovery_hold_steps=30` 不参与该次退出判断。当前新的 A parser 将 hold 明确设为关闭。程序的 PASSIVE 请求路径预期转入安全状态，但旧运行未取得策略已停止的确认。
- **仍为 UNKNOWN / 限制：**MuJoCo 是否消费每条 Recovery 命令；PASSIVE 请求后策略停止确切确认；实机 foot-force 数组的物理足语义；其他场景、正式实验和硬件行为。旧 run 出现快速阈值往返，但不是参数扫描或正式切换效果评价。其原始入口日志“保持步数=30”与 paper 候选语义不符，保留为历史日志事实；本轮已修正当前日志格式。
- **第一次启动失败尝试**为参数路径错误，未启动环境；**实际运行后环境已关闭**，事后进程核对为空，临时 GLFW 探针源码/二进制也已删除。没有影响无关进程。

## 定向配置补齐（2026-10-10）

### 生产入口与合同

- 按已通过 [SWITCH_CONTRACT.md 第 6 节](../../SWITCH_CONTRACT.md#6-配置与保持时间合同步骤-3) 将工作区部署 YAML 改为唯一入口：`abs.switching.group: A`、`entry_threshold: -0.05`。A 的有效退出阈值由 E 推导为 E；解析后内部复用 `paper_faithful_switch` helper 并将 hold 步数设为 0。模型、观测/动作路径、Recovery 路径和研究阈值没有改变。
- 新生产解析器拒绝缺失配置、缺失字段、非有限或超范围 E、未知/错层字段、A 不允许的 `exit_threshold` / `recovery_min_hold_s`，以及并列 legacy `switching_mode` / `ra_threshold` / `recovery_hold_steps`；B/C/D 明确报告尚未支持。解析失败有 `[S2-SWITCH-CONFIG] invalid ...; refusing to start` 错误日志，不做默认或回退。
- controller 来源行与专用配置消费行记录 `group=A`、实际 E、推导 exit、`hysteresis=false`、`hold=false`。Recovery 进入/退出日志也不再把旧 `保持步数=30` 显示为 A 组设置。策略帧仍记录实际 entry/exit 数值；run context/source recheck 记录组别和两个关闭标志。
- 定向 C++ 检查调用生产 `parseAGroupConfig()` 与 `stepSwitching()`：27 项通过，包括部署 YAML、A 合法/缺失/冲突/未知/禁止/错层/NaN/±Inf/越界输入，`RA<E`、`RA==E`、`RA>E` 边界、初始化状态及注入旧 hold 状态仍立即清零退出。控制器增量构建通过，Python source harness `py_compile` 通过。
- 生产来源匹配器增加 group/E/有效 exit/flags 核对；脚本把 overlay 解析为绝对路径。原第一次 config-only 执行之前监督器因相对的 overlay 路径与日志中的绝对 `--params-file` 比较而错误拒绝；这只是匹配器自身的路径规范化缺陷。修正后的匹配器对同一保存日志离线重验通过，没有重写原始 run_context 或 terminal。

### 新入口实际消费证据（初始化阶段，无 RL）

- 本次仅做初始化 preflight，run ID `20df7fd42a3e4dc4aede8579c7117c72`。记录器先就绪，MuJoCo 与唯一 `ros2_control_node` 启动并激活 controller；没有发送 RL 请求、没有策略帧、没有运动。
- 原监督器终局保留 `INVALID_RUNTIME_SOURCE`，因为其对比的 expected `--params-file` 是相对路径，实际同一 manager PID `49900` 的命令行和 ROS launch 日志为绝对路径。其原始 terminal/context 保持不变。
- 独立只读重验 [config_source_recheck.json](diagnostics_20261010/config_consumption_preflight/20df7fd42a3e4dc4aede8579c7117c72/config_source_recheck.json) 使用该 run 原始 `ros_launch.log` 和唯一 manager PID，确认：overlay `package_share` 与实际加载 `abs/config.yaml` 为工作区同一安装前缀；candidate helper 为 `paper_faithful_switch`，group=A，E=`-0.05`，effective exit=`-0.05`，两个标志均 false；Agile、RA、Recovery 模型路径各自与本次预期绝对路径完全相同；配置行、专用 A 配置行、Recovery 模型加载行同属 `ros2_control_node-3`。这直接证实 controller 初始化消费新配置，但不证明任何策略周期的切换行为。
- 同次保存的原始 `run_context.json` 仍记监督器 `INVALID_RUNTIME_SOURCE`，`terminal.json` 保存且 `process_facts.json` 为 `shutdown_complete=true`、`exit_code=0`、未强杀。离线重验另存独立文件，没有把原始失败判为通过。事后进程检查未见本任务的 MuJoCo/controller/RViz/采集器残留。

### 旧行为证据、短时反向切换和限制

- 旧运行 `a5b88cf10d4748d4b15ce1b9566e524e` 保留为 legacy paper 候选的真实 RA 行为证据；不能称为已消费新 `abs.switching`。本轮初始化 preflight 无 RL，所以新配置还没有策略周期/帧级切换行为证据；第二次额度没有用于运动诊断，也没有追加第三次场景运行。
- 本报告将“短时反向切换”定义为：同一 session 连续有效策略帧中，一个模式边沿后 5 个策略周期内出现相反边沿。描述性诊断窗口为每个边沿前后各 10 个连续有效周期；缺周期、STALE 时钟、session 改变时不跨越缺口计算。该定义只用于观察旧运行中的快速返回，不是 A 的切换条件、正式验收值或新配置实测结论。旧运行 RA 模式转换间隔保持原值，不据此判定切换优劣。
- 旧运行的驻留时间统计仍受 5 个 STALE 仿真时钟帧删失限制；未跨缺口插值。碰撞终局为原有效运行直接观察结果，保留其原运动窗口覆盖范围，不因本轮配置变更而重跑。策略停止确认、MuJoCo 对 Recovery 命令的消费、实机 foot-force 物理语义及其他场景/正式实验仍 UNKNOWN。

本轮完成了 A 唯一配置入口、严格拒绝规则、日志与来源核对、针对性离线验证和构建，并从 controller 初始化日志直接确认新配置已消费。原执行阶段曾以 ACTIVE 状态提交；最终审阅结论见下文。

## Director 审阅后结论（2026-10-10）

Director 有条件通过 S2-01，并认可以下三类证据，各自范围如下：

1. **Legacy 候选真实 RA 运行行为：**`a5b88cf10d4748d4b15ce1b9566e524e` 的真实 RA 输出驱动 6 次进入和 6 次退出，终局为障碍碰撞。它证明此前 legacy `paper_faithful_switch` 候选的行为，不证明新 `abs.switching` 已进入运动周期。原运行的 5 个 STALE 时钟帧仍使驻留统计删失，碰撞事实及原始记录保留。
2. **新 A 配置生产解析器与切换函数检查：**生产路径对唯一 `abs.switching` 入口、冲突/非法配置拒绝、边界和无保持行为进行 27 项定向离线检查并通过，控制器构建通过。这证明配置与 helper 的代码行为，不是模型或仿真切换证据。
3. **Init-only 配置消费：**同一 controller 初始化日志的独立重验确认 group=A、E 与有效退出阈值均为 `-0.05`、`hysteresis=false`、`hold=false`，且模型路径匹配。该运行没有请求 RL、没有策略帧或运动周期。supervisor 原始 `INVALID_RUNTIME_SOURCE` 分类和原文件保留不变；独立重验只证明启动时配置消费，不改判该 run。

因此 S2-01 的 A 基线阶段获有条件通过并标记 COMPLETE；新 A 配置下的运动周期验证仍缺。策略停止确认、MuJoCo 命令消费、实机接口语义等限制继续保留。未来 B 组核验应确认统一 A/B 配置的实际运行消费，并保持两组除滞回外的控制链一致。

## 修改文件

- 新增生产 A 配置解析器 `AbsSwitchConfig.hpp` 与 27 项定向检查；`StateRL.cpp` 使用该解析结果、输出明确的组别/阈值/flags 来源行，并修正 A 入口/退出日志；更新 helper/state 成员注释，不改变状态机条件。
- 部署 `abs/config.yaml` 改用 `abs.switching.group` 和 `entry_threshold`；`SWITCH_CONTRACT.md` 第 6 节示例与当前运行时支持范围同步为 A-only。
- `s1_runtime_source.py` 核验新来源字段和模型路径；`s1_run_diagnostic.py` 解析 overlay 为绝对路径，增加不请求 RL 的 config-only 初始化模式及消费来源门禁。
- 更新本报告、运行计划补充记录、`tasks/S2-01.md` 与 `CURRENT_STATE.md`。config-only 原始 JSON/log/terminal 保留；独立 matcher 重验结果另存 `config_source_recheck.json`。此前有效 run 与失败 run 的原始记录均未改写；未提交 Git。
