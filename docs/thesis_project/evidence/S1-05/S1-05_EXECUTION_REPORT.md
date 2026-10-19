# S1-05 Execution Report

**Execution status: COMPLETE — Director 有条件通过（2026-10-08）。** 通过范围为本任务安全记录链、针对性离线及真实 C++→Python 跨语言验证，以及一次最小正常诊断的记录、分析与收尾；不代表切换效果或正式实验条件全部验证。历史失败与原始 INVALID 记录保持不变。

## 结论概览

- 碰撞发布器已改为每个 MuJoCo PhysicsLoop 物理步累计 robot-obstacle episode，并保存起止物理步、仿真时间、接触对象 ID/可得名称、持续时间、ground/self/other/unknown 分类及物理覆盖标志。
- 跌倒按 episode 记录：正常观察关闭事件并重新武装，持续异常不重复计数，启动期与运动期依据 episode 区间重叠判定；未确认候选跨越运动起点或无效姿态覆盖运动期时输出 UNKNOWN。
- 每个物理步累计未分类接触、无效姿态，并保留首末时刻；发生范围与运动区间重叠时，不输出无碰撞/未跌倒。物理步覆盖断裂与历史溢出同样使否定结论 UNKNOWN。
- 受控 headless MuJoCo 检查曾直接看到一个障碍接触 episode 和 0.300 仿真秒跌倒确认，但该检查发生在本轮 ground 分类修正之前，且其输出快照没有由正常 Python reader 接受。因此它只证明当时生产者路径有相应累计输出，不证明最终生产者—reader—analyzer 链完整通过。
- 历史补跑 run 64d80227 在启动控制器前因 producer 写 version=4、旧 reader 要求 version=3 而停止；0 策略帧且无运行绑定。该 INVALID 记录保持不变，见原始目录与后验快照。
- 新 ABI 严格规定 v3=3864、v4=5760；reader 仅接受对应版本/长度组合。历史 v3 可解析但没有新事件锁存能力，不可用于相应窗口的无碰撞/未跌倒结论。新 v4 含跌倒历史、未分类接触/无效姿态累计字段及连续覆盖事实。
- 最新一次正常诊断以唯一 session/writer 和本轮构建的 v4 publisher 完成。37 个有效策略帧均读到 LIVE v4 authority，runtime source、candidate、阈值、目标及模型路径均匹配；record validity=VALID。终局为 ARRIVED，终止周期同一快照已保存。
- 该 run 的运动窗口（23.996–24.516 sim s）在本次诊断规则下覆盖完整，分析为 collision=false、fall=false。此结论仅适用于已覆盖的该运动窗口和本次诊断规则，不是对全采集区间或其他规则的无碰撞/未跌倒结论；有两个 posture-rule 确认 episode 均在运动开始前结束。策略全程 Agile，未发生 Recovery，不能用于切换效果结论。

## 规则与直接依据

### 碰撞

- 直接来源是 MuJoCo 接触数据，在每次物理步发布；当前七个障碍几何签名与 robot geom group 3 的接触记为障碍碰撞。
- 正常脚掌地面接触使用 FL/FR/RL/RR 几何名计数；机器人其他几何与 floor 接触另计。静态障碍物与 floor 接触归入 other；自碰撞和无法分类接触分别保留。
- 从无接触到接触开始新 episode；连续接触不逐物理步重复计数；接触中断后再接触才开始新 episode。最多保留 32 个 episode，超过容量会标记溢出。
- 当前运行中的 authority 会把物理步跳号或仿真步长与 MuJoCo timestep 不符设为覆盖不完整。累计器以 authority 初始化为起点；控制器/仿真共同重启会重新初始化。UI 单步不属于物理发布器覆盖。
- 没有有效 v4 authority、存在未分类接触、覆盖中断/历史溢出，或未覆盖运动起点至终止周期时，运动区间的“无碰撞”结论必须为 UNKNOWN。v3 可有限读取，但不具备新事件锁存字段，不能支持该否定结论。

### 跌倒

- 直接位姿来源是 MuJoCo 世界坐标下 base_link 的 xpos.z（米）和 xquat（w,x,y,z）；从四元数换算 roll/pitch（弧度）。
- 本次诊断规则：高度 <0.22 m，或 |roll|>60°，或 |pitch|>60°；任一条件连续满足 >=0.30 有效仿真秒确认。规则是 OR，姿态严格等于 60°不触发，持续时间恰为 0.30 s 可确认。
- 正常姿态关闭 episode 并重新武装。无效姿态或物理步/仿真时钟不连续会打断未确认候选的连续计时；下一有效异常从新时刻继续同一未闭合 episode。无效姿态不会被误当成恢复；已确认 episode 不会被重复确认。确认起止、物理步、仿真时刻保留于 32 项环形历史。
- 每物理步记录无效姿态累计次数及首末发生范围。若该范围覆盖运动区间，跌倒结论为 UNKNOWN；若未确认候选与运动区间重叠，也为 UNKNOWN。已确认 episode 与运动区间相交则判定运动期跌倒，包括启动时确认但持续进入运动区间的情形。历史溢出或覆盖缺失时不允许判“未跌倒”。0.22 m/60°/0.30 s 仍是诊断规则，不是 S6 冻结的正式阈值。

### 终局和指标边界

- 运动起点为首个有效策略位姿相对首个策略位姿位移 >=0.10 m；终止周期包含在运动指标内，清理尾段排除。
- 同一策略观察点的优先级为：系统安全中止、障碍碰撞、跌倒、到达、仿真时间上限；已观察到的其他事件仍保存在帧/事件记录中。人工中止单独记录。墙钟保护只产生 WALL_CLOCK_GUARD_INCOMPLETE，不伪装为仿真超时。
- 到达为进入目标 0.5 m 范围；PASSIVE 只是停止请求。策略停止确认、进程退出和 MuJoCo 命令消费分别记录。无 safety veto 不等于无碰撞。
- 碰撞/跌倒/安全中止属于任务终局；来源冲突、混流或缺少完整终止载荷属于数据有效性问题，不互相覆盖。

## 实现、验证与运行证据

### 修改和离线检查

- 仿真 ABI/解析：v3 保持 3864 字节旧布局；新增 v4 为 5760 字节，含 32 项跌倒 episode 历史及未分类接触/无效姿态物理步累计与覆盖范围。Python reader 严格按 version+length 配对，显式处理 C++ 对齐填充并校验字段、计数与历史。
- 规则复用：新增 common/abs_safety_rules.h；仿真生产路径和 C++ 针对性用例共用分类、episode 边沿及跌倒计时规则。
- 采集与分析：记录器序列化 v4 快照并保留有限读取的 v3 数据；分析器只有在 v4 事件锁存完整时才允许给出无碰撞/未跌倒结论。按运动起止 sim_time 筛选累计 episode，不将运动前/清理尾段事件计入运动安全结果。外部安全事件可携带终止时的最新 collision snapshot。
- 终局路径：同一观察点使用明确优先级；收尾仍先清理进程，收尾记录错误单独保存。策略帧缺失时分析结果保持 INVALID/UNKNOWN。
- 验证通过：Python 编译；scripts/test_s1_05_safety.py；直接以断言启用方式编译并运行 C++ s1_05_safety_rules_test；unitree_mujoco 与 s1_05_collision_probe 增量构建；scripts/test_run_record.py 54/54。collision_probe 本轮只构建、未执行，以免其测试清理逻辑触碰持久共享内存。
- 针对性用例覆盖：启动期确认—恢复—运动期新确认；异常跨运动起点；持续异常去重；两策略采样间事件锁存与 reader 解码；未分类接触/无效姿态覆盖运动期使结论 UNKNOWN；物理覆盖不足使结论 UNKNOWN；机器人—floor/障碍—floor ground 分类和 Python reader 不变量；终止快照落盘；终止帧缺失/写入失败拒绝完整性；收尾写入失败仍执行进程清理。

### 受控 headless 物理检查

- 命令和原始结果见 [controlled_contact_posture.json](controlled_contact_posture.json) 与 [controlled_contact_posture.log](controlled_contact_posture.log)。
- 使用 scene_obstacle.xml 和生产 ObstacleCollisionAuthority；人为将 base_link 固定到已绑定障碍附近，再固定于低机身姿态；不加载策略、不启动控制器、不改场景模型。
- 观察到 authoritative=1、物理覆盖标志为 1、1 个 episode、5 个接触物理步，以及 step 11 至 161、0.300 仿真秒后的跌倒确认。障碍名称在该 probe 输出为空；生产快照保留 geom ID。该 probe 发生在下面发现的 ground 分类修正之前，因此不作为最终 v3 reader 接受证据。

### 显示预检及正常短诊断

- [display_preflight.json](display_preflight.json)：现有 DISPLAY=:0/XAUTHORITY 下 X 连接成功；系统 GLFW 3.3.6 创建隐藏窗口成功，OpenGL vendor Intel、renderer Mesa Intel(R) Graphics (RPL-P)、Mesa OpenGL 4.6。只使用一次性临时环境，没有改授权、系统服务、驱动或安装组件。
- 运行配置预检解析到 package share /tmp/s1_04_overlay，配置文件候选 paper_faithful_switch、goal_x=1.0 m，三个模型文件存在。但进程在控制器启动前停止，因此这些是配置文件/路径预检，不是已生效的运行配置证据。
- Run ID 为 60f5f44663104b6c883b4668ba1473e4。原始目录：[diagnostic run](diagnostic_runs_20261008/60f5f44663104b6c883b4668ba1473e4/)；启动命令和清理结果见 [launch_evidence.json](diagnostic_runs_20261008/60f5f44663104b6c883b4668ba1473e4/launch_evidence.json)。
- recorder 已就绪，MuJoCo 启动；collision authority 快照有本次 capture_id、authoritative=1、模型 fingerprint 匹配，但 Python reader 将其判为 INVALID。只读回读证据见 [authority_readback.json](diagnostic_runs_20261008/60f5f44663104b6c883b4668ba1473e4/authority_readback.json)：ground_contacts=9，foot_ground_contacts=5，nonfoot_ground_contacts=0，违反 reader 的 ground 拆分不变量；确切多出的四项在快照中没有对象分类，不能追溯猜测。
- 因而 0 策略帧、0 运动周期；controller/ROS/RViz 未启动。分析结果 [run_results.md](analysis_20261008/run_results.md) 与 run 目录 metrics.json 将其列为 INVALID / STARTUP_FAILURE，所有策略指标 UNKNOWN。该运行不支持配置生效、切换、到达、无碰撞或未跌倒结论。
- 找到不变量后已修正生产分类：只有机器人—floor 接触计入 ground，障碍物—floor 计入 other。修正通过离线 fixture 和增量构建；没有重跑场景。受控 probe 已使用一次，失败的启动诊断已使用一次，本任务最多两次短仿真已达到上限。

### 用户授权的单次补证例外

- 用户明确授权在原“两次短仿真”上限之外补一次最短正常策略诊断。运行 ID 为 64d80227fba24d75954fc4da979cd82e，原始目录：[run 64d80227](diagnostic_runs_20261008/64d80227fba24d75954fc4da979cd82e/)。配置为 scene_obstacle.xml、paper_faithful_switch、目标 x=1.0 m、0.5 m 到达半径、既定 /tmp/s1_04_overlay 配置与三个模型路径、6.0 sim-s/50 s wall 上限；没有暂停、重置或改变研究参数。
- 启动前 X 连接与隐藏 GLFW/OpenGL 预检通过，结果见 [display_preflight_retry_20261008.json](display_preflight_retry_20261008.json)。renderer 为 Intel / Mesa Intel(R) Graphics (RPL-P)，OpenGL 4.6。采集器先就绪，随后 MuJoCo 启动；碰撞源未被 supervisor 的正常 reader 接受，controller/ROS/RViz 未启动，因此配置和模型只由启动前路径检查确认，不是已生效 controller 运行事实。
- Run 的 mujoco.log、runtime_record.jsonl、terminal.json、process_facts.json 和 [metrics.json](diagnostic_runs_20261008/64d80227fba24d75954fc4da979cd82e/metrics.json) 保留了失败边界：0 policy frames、0 session/writer 绑定、0 运动终局，结果 STARTUP_FAILURE，指标 INVALID/UNKNOWN，没有策略频率或安全效果结论。
- 只读快照后验见 [authority_reader_postmortem.json](diagnostic_runs_20261008/64d80227fba24d75954fc4da979cd82e/authority_reader_postmortem.json)。快照含匹配 capture/fingerprint、authoritative=1、physics coverage=1，ground split 为 9=4 foot+5 nonfoot；当次 producer 二进制写 version=4，而 supervisor reader 要求 version=3，因此该运行被拒绝。事后在内存中把版本标签替换为 3 后校验通过，但这只是 reader 诊断，不改判运行通过。之后源码/构建恢复到 version=3，并修正 C++ 对齐填充的解析偏移，离线 fixture 通过；按失败停止要求没有再次运行。
- 该只读快照还显示一条启动期姿态 episode：physics step 125 开始、step 275 确认、采样末 step 5124 仍未闭合。它发生在任何 controller/policy frame 和运动边界之前，因此不是运动期跌倒结果；它说明该次启动快照携带了持续姿态异常证据。
- 原有 INVALID 原始运行与本次启动失败记录均保持原样，没有拼接或挑选成有效结果。

## 证据等级与未解决事项

**前次尝试的直接证据（历史）：** X/GLFW 隐藏窗口建立成功；受控 probe 输出障碍 episode 和跌倒确认；run 64d80227 的 authority 曾发布匹配 capture/fingerprint、覆盖有效及 ground split=9(4+5)，但因当时 reader/producer 版本不匹配没有启动 controller/policy。该原始运行仍是 STARTUP_FAILURE。

**代码推断：** 当前 v4 publisher 在每个 MuJoCo 物理步更新碰撞、姿态 episode 与不确定性范围；有效版本/长度配对、无溢出且覆盖完整时，分析器才可对运动窗口给出否定结论。新跨语言记录证明该生产字段能经 Python reader、序列化和 analyzer，受控 fixture 仅作为代码链路验证，不代表策略效果。

**仍为 UNKNOWN：**

- 更长时段或其他场景下版本/来源及累计覆盖能否保持；本轮单次诊断不能证明广泛运行稳定性。
- 单次诊断的 XAUTHORITY 记录路径错误（见最新补证）；诊断实际 renderer 未直接保存，不能声称与预检完全一致。
- 该次已覆盖运动窗口在本诊断规则下分析为无碰撞/未跌倒并到达；这一结论不延伸到全采集区间，也不是正式实验结论。
- 静态障碍几何名称缺失时，仅有 geom ID 的具体可读性；UNKNOWN 接触、episode 溢出或 publisher 中断时结果仍需保守 UNKNOWN。
- 候选模型真实动作关节顺序、MuJoCo 消费命令时刻，以及诊断规则对正式实验的适用性仍 UNKNOWN。本次运行只从 ROS transition 日志确认策略停止状态，不代表 MuJoCo 已消费命令。

## 前次失败补跑的改动文件与收尾（历史记录）

本轮代码修改：common/abs_collision_contract.h、common/abs_safety_rules.h、unitree_mujoco/simulate/src/obstacle_collision_authority.h、unitree_mujoco/simulate/test/s1_05_safety_rules.cpp、scripts/abs_collision.py、scripts/run_record.py、scripts/s1_run_diagnostic.py、scripts/s1_analyze_run.py、scripts/test_s1_05_safety.py。必要增量构建更新了 unitree_mujoco 与 s1_05_collision_probe；probe 本轮只构建、未执行。

证据文档：本文件、[S1-05_RUN_CONTRACT.md](S1-05_RUN_CONTRACT.md)、GLFW [v4 显示预检](display_preflight_v4_20261008.json)、[跨语言 v4 核验](cross_language_v4_validation.json)、以及 diagnostic_runs_20261008/ 中保留的原始运行和分析结果。当前任务引用见 tasks/S1-05.md 与 CURRENT_STATE.md。

补跑的 process facts：MuJoCo 退出码 0、未强杀、shutdown_complete=true、closeout_complete=true；authority 未通过启动门，因此 ROS/controller/RViz 未启动。运行后未发现本任务 MuJoCo、controller manager、RViz、采集器或诊断进程，见 [process_closeout_postmortem.json](diagnostic_runs_20261008/64d80227fba24d75954fc4da979cd82e/process_closeout_postmortem.json)。GLFW 隐藏窗口随探针正常退出销毁；临时探针文件随后清理。/dev/shm 中仿真共享对象保留，未 unlink，以免影响潜在 reader；当前无活动 writer。未影响无关会话。

## 最新补证（2026-10-08）

### ABI 与真实跨语言路径

- C++ `Snapshot` 明确为 version 4，`sizeof=5760`；v3 原 ABI 保持 3864 字节。正常 Python `read_collision_snapshot()`、`classify_snapshot()` 只接受 v3/3864 与 v4/5760，拒绝错配、未知版本和截断。v3 可读但 `event_latches_complete=false`，分析器对无碰撞/未跌倒输出 UNKNOWN。
- 使用真实 `scene_obstacle.xml` 和 C++ `ObstacleCollisionAuthority::publish()`，通过唯一 `/s1_05_xlang_...` 临时共享内存生成快照。正常 reader 从该共享对象读取最终快照；生产记录器写出一条带 v4 payload 的 terminal line，记录回读有效；生产 `motion_safety()` 使用来自同一 C++ 物理发布序列的快照，识别启动期异常、恢复后新跌倒及跨边界事件。证据：[cross_language_v4_validation.json](cross_language_v4_validation.json)。v4 关键字段 offset 也在该记录中列出。唯一测试共享内存由测试清理，未触碰默认 `/mujoco_collision_v2`。
- 受控 probe 发现并修复两处问题：无效姿态原 NaN 使 Python reader 拒绝整帧；FallTracker 重置当前确认位时会把已闭合旧 episode 的确认状态清掉。v4 累计锁存保留 occurrence，序列化数值保持有限，历史 episode 的确认事实保持粘滞。probe 受控且不含策略，不用于策略效果。
- 定向验证：`scripts/test_s1_05_safety.py` PASS；`scripts/test_run_record.py` 54/54；Python compile PASS；`unitree_mujoco`、`s1_05_collision_probe` 和 `s1_05_safety_rules_test` 增量构建通过，C++ safety rules 可执行通过。

### 唯一正常短诊断

- Run ID `8ee542c305fd4ee6a755294e73f8e5db`，scene_obstacle.xml，paper_faithful_switch，goal=(1.0,0.0) m，arrival radius 0.5 m，entry=exit=−0.05，sim 上限6s / wall 上限50s。配置文件、package_share 和三个模型路径均来自 `/tmp/s1_04_overlay/share/go2_description`；运行源码绑定到同一 controller PID/writer PID=260310、session=35641272835553，运行时 source validation=confirmed。
- recorder 在 MuJoCo 和 controller 前就绪；MuJoCo 先发布 LIVE v4 authority 才启动 ROS/controller。37 个同 session 连续策略周期（rl_step_gaps=0）均持有 LIVE v4 authority，记录 validity=VALID；终止帧 cycle=76、sequence=154、sim_time=24.516 s 的完整 payload 已保存，runtime_record.jsonl 恰有一条 terminal line，terminal.json 保存同一终局。
- 终局 ARRIVED，距离0.487 m，符合0.5 m判据。有效运动窗23.996–24.516 sim s：analysis `coverage_complete_for_motion_interval=true`、collision=false、fall=false；未分类接触和无效姿态均未覆盖该窗口。原始 authority 历史另有两个跌倒规则 episode，但分别在 sim s 18.190 和23.698结束，早于运动起点；不能据此说运动期跌倒。策略全程 Agile、无 Recovery 转换，不能判断切换效果。
- 36 个相邻 LIVE 帧间隔使用 `monotonic_ns` 差计算：整体 50.045 Hz；中位周期19.892 ms，P95 28.411 ms，最小2.538 ms、最大38.144 ms。统计范围是本次全部 LIVE 策略帧；仅一条运行记录，不用于归因观测开销或证明无调度影响。
- 运动期无安全 veto。清理阶段收到 PASSIVE 停止请求，随后 ROS transition 日志确认策略进入停止状态；终端记录另外保留 cleanup 阶段的 `GLOBAL_HARD_STOP`。MuJoCo 是否消费停止/控制命令仍 UNKNOWN。MuJoCo 与 ROS launch 均退出码0、未强杀、closeout 无错误；事后进程盘点未发现本任务 controller、MuJoCo、RViz 或采集器残留。
- 显示预检使用正确 XAUTHORITY `/run/user/1000/gdm/Xauthority`，X连接和 GLFW 隐藏窗口通过，Intel/Mesa RPL-P、OpenGL 4.6；证据见 [display_preflight_v4_20261008.json](display_preflight_v4_20261008.json)。诊断命令参数误将 XAUTHORITY 写成不存在的 `/run/user/1000/gdm/gdm/Xauthority`，但 MuJoCo/控制器实际启动并完成；由于本次 run_context 记载的路径错误且运行未保存 renderer 读数，本次授权上限不再重跑，报告保留该环境限制。
- 本轮修改文件：`common/abs_collision_contract.h`、`unitree_mujoco/simulate/src/obstacle_collision_authority.h`、`unitree_mujoco/simulate/test/s1_05_collision_probe.cpp`、`scripts/abs_collision.py`、`scripts/run_record.py`、`scripts/s1_analyze_run.py`、`scripts/test_s1_05_safety.py`、`scripts/test_s1_05_cross_language.py`，以及本报告、run 合同、任务执行记录和 CURRENT_STATE。历史诊断 raw files 未修改。

### 当前状态与限制

Director 有条件通过，S1-05 状态为 COMPLETE。通过范围是安全记录实现、针对性离线/跨语言验证，以及本次最小诊断的记录、分析和收尾流程。本次无 Recovery，不能判断切换效果。collision=false、fall=false 仅适用于已覆盖的运动窗口（23.996–24.516 sim s）和本次诊断规则；全采集区间的覆盖字段不构成同一否定结论。诊断的 XAUTHORITY 参数路径记录错误，实际 renderer 未保存；正式跌倒参数、其他场景和长期稳定性尚未验证。模型真实动作关节顺序、暂停恢复、时钟重连、MuJoCo 命令消费仍 UNKNOWN；瞬态短于一个物理步的碰撞/姿态覆盖也有限。

简单来说：版本和长度现在一一对应，真实 C++→Python→记录→分析链通过；一次正常运行到达目标并完整收尾。仅在覆盖完整的运动窗口及本次规则下分析为没有碰撞或跌倒，没有进入 Recovery。运行后相关环境已关闭；默认仿真共享内存对象保留，未删除。
