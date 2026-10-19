# S4-01 Execution Report — COMPLETE（Director 有条件通过）

> Director 于 2026-10-11 有条件通过 S4-01。通过范围为 C 配置实际消费、按仿真秒执行最小保持、保持起算/到期前保持/到期退出/风险再升高不重启计时、同代码 A/C 各一次功能诊断，以及到达后的站立观察和环境清理。最新 C run 为 884 个 v3/C/H=0.30 策略帧，结果 VALID / ARRIVED，并完成约 5 仿真秒站立观察。

## Director 审阅后的状态（2026-10-11）

- 保持审计覆盖 19 个 Recovery episode：10 个完整、9 个不完整，整体仍为 `incomplete`。分析未发现保持违规，但这不能证明缺失周期期间也符合规则。
- A 仅一次碰撞、C 仅一次到达，不能据此判断两组性能优劣或切换效果。H=0.30 秒是功能诊断参数，不是正式研究参数。
- 未取得实时 C 面板截图。碰撞否定结论、运动期跌倒、暂停恢复、时钟重连及 MuJoCo 命令消费仍有相应覆盖或验证限制。
- stop publisher 退出码 1 及历史失败记录均保留；既有 C run 的收尾证据确认环境清理完成。以下历史执行记录和原始数据分类不因本次审阅而改写。

## 最新补证：重复节点诊断修正与 C 运行

### 重复节点名的证据判断

- 前一失败 C run `[local run ID omitted]` 的 launch log 仅记一条 `ros2_control_node` 启动、一条 manager 初始化，以及一次 `rl_quadruped_controller` 配置/激活；run context 中 launch process group 也只有一个 manager writer。`mujoco.launch.py` 只创建一个 manager Node 和一个 controller spawner。没有证据证明当时存在第二个活动 manager/writer，也没有同一 launch 内创建两个 manager 的证据。
- 失败时 `ros2 node list` 通过默认 ROS CLI daemon 返回 manager/controller 名称各两次。旧记录没有节点实例身份或同期 `--no-daemon` 查询，因此无法回溯证明是 daemon 留存项还是 DDS 图发现时序；这两种情况仍是可能原因，不能写成已确诊历史根因。
- 修复 `scripts/s1_run_diagnostic.py`：节点门槛改用 `ros2 node list --no-daemon --spin-time 2` 的新鲜图查询，默认 daemon 输出仅作为差异诊断保存；要求 `/controller_manager` 和 `/rl_quadruped_controller` 各恰好一项，同时仍要求 launch process group 中恰好一个 manager writer、库映射唯一、controller 为 active。任何直接图重名、缺项、查询失败仍停止，不删除或绕过重复检查。
- 定向回归 `scripts/test_s4_01_controller_graph.py` 覆盖唯一节点通过、manager/controller 各自重复时拒绝、空图/不完整图拒绝。检查通过。

### C 运行来源及结果

- 唯一一次本轮 C run：`[local run ID omitted]`，记录目录 `logs/manual_ab_compare/20261011_151659_d4a7f6/C/[local run ID omitted]/`。配置为 `scene_random_04.xml` / `random_04`（S2-08-r1、19 个障碍）、目标 `(7,0)`、半径 `0.5 m`、运动上限 `20 sim s`、墙钟上限 `120 s`、站立观察 `5 sim s`；C overlay 独立于安装配置。
- controller PID/writer PID 均为 `141268`；`/proc/<pid>/maps` 直接确认加载本工作区 install 下的 `librl_quadruped_controller.so`。该 PID 是本次 launch group 中唯一 manager writer。manager 与 controller 各唯一；直接 ROS 图和 daemon 图本次都返回一份 `/controller_manager`、一份 `/rl_quadruped_controller`，`list_controllers` 显示 active。
- 实际初始化日志及首帧来源校验确认 group=C、E=-0.05、有效退出阈值=-0.05、无滞回、hold=true、H=0.30 s；模型/配置路径属于本次 C overlay。所有 884 个策略帧均为 frame v3、group=C、H=0.30、hold enabled；单 session，source binding 确认 writer/controller/session 一致。
- 结果 `VALID / ARRIVED`，终点距 `(7,0)` 为 `0.4931 m`。运动起点 cycle 48 / `23.616 sim s`，终止 cycle 884 / `40.336 sim s`；起终点跨度 `16.720 s`，有效连续时间 `16.086 s`，有效路径 `10.777 m`，平均速度 `0.670 m/s`。策略频率 `50.045 Hz`，median `19.955 ms`、P95 `26.268 ms`；有 16 个无效时钟帧和 1 个策略周期缺口，保留在统计中。
- 记录 38 次模式边沿（Agile→Recovery 19、Recovery→Agile 19），38 次边沿均带有效仿真时间；0.5 秒短反向切换计数为 30。保持周期审计详见 [hold_audit.md](hold_audit.md) 与该 run 的 `metrics.json`：19 次进入/19 次退出；219 个有效到期前低风险周期仍保持 Recovery；22 个到期后仍高风险周期继续 Recovery；所有 19 次退出均 elapsed≥0.300 s 且 RA<-0.05；Recovery 中 5 次风险重新升高均保留原始 `hold_enter_sim_time_s`。分析无检测到违规，但 10 个 episode 时钟覆盖完整、9 个不完整，aggregate 为 `incomplete`；不跨缺口补插。
- 实际 MuJoCo framebuffer 截图保存为该目录下 `standing_hold.ppm`，无重绘转换副本为 `standing_hold.png`。画面显示 `ARRIVED - HOLDING STAND`，HUD 为 `INVALID / No current policy data`；截图是在策略停止后的站立阶段，不是实时 C policy/HUD 截图。没有为截图追加运行。
- 收尾先关闭 MuJoCo（退出码 0），再关闭 ROS/controller（退出码 0）；未强杀。stop publisher 退出码 1，但 closeout 与 shutdown 均确认完成。站立观察 `5.006 sim s`，记录姿态稳定样本；未请求 PASSIVE。无安全 veto。运动记录未观察到障碍接触，但 collision 否定结论因 physics-step 覆盖缺口仍为 UNKNOWN；运动期跌倒为 UNKNOWN，有未确认姿态候选。收尾后直接 ROS 图和 daemon 图均无项目节点，进程清单无本次仿真/controller/RViz/stop publisher；保留的 ROS CLI daemon 是运行前已存在的用户级进程，本轮未终止。
- 首次自动分析调用的 `/usr/bin/python3` 因系统 Matplotlib 与 NumPy 2.2.6 二进制不兼容退出；失败日志 `C_analysis.log` 保留。之后使用已有 Anaconda Python（NumPy 2.1.3 / Matplotlib 3.10.0）对同一原始 run 成功离线生成 `C/analysis/run_results.*` 和该 C run 目录内的 `timeline.png`，未重跑仿真。A/C 指标表见 comparison.md（原始运行/环境明细保留本地，未纳入分享提交）。

### 新 A/C 对照范围

新 A `[local run ID omitted]` 不重跑，保留其 `VALID / COLLISION_TERMINATION` 结果。A/C 对照表将该 A 与本轮 C 分组引用；两者用相同安装 controller 构建、模型、scene_random_04/S2-08-r1、目标和初态流程，区别为 A 无保持、C 增加 H=0.30 s。每组一次不能用于成功率或性能优劣推断。历史 C startup failure 与早先 v2 A 均保留但不纳入新对照。

## 前一轮 A/C 诊断与核验记录（历史保留）

### 前一轮启动前核验

- build 与 install controller ELF 大小均为 2,217,880 bytes，Build ID 均为 `edf3bb6dd8e0270e75c42af6387a76557d627971`。`ros2 pkg prefix rl_quadruped_controller` 指向本工作区 install 前缀。
- A run 的 `run_context.json` 记录 controller PID [omitted] 的 `/proc/<pid>/maps` 中实际库路径为 `<workspace-root>/quadruped_ros2_control_humble/install/rl_quadruped_controller/lib/rl_quadruped_controller/librl_quadruped_controller.so`；A 的策略帧全部为 v3，模式码与 A 相符。
- 手动运行器使用 `copytree(..., symlinks=False)`。实际生产准备函数的 A/C 隔离检查通过，原安装配置字节未变；C 的 `0.30`、`0.300` 以数值等价比较。
- 两次 run 启动前没有已知 controller-manager 进程。A 的唯一 writer/session 与 source log 绑定成功；C 没有开始策略帧/writer。
- 当前 `the active DISPLAY value`、`XAUTHORITY=current desktop-session authorization file` 通过 X11 和 16×16 隐藏 GLFW/OpenGL 检查，renderer 是 `Mesa hardware renderer`，OpenGL 4.6。见 display_preflight.json（原始运行/环境明细保留本地，未纳入分享提交）。

### 前一轮 A/C 诊断（旧 C startup failure）

同一目录：A/C 运行目录（原始运行/环境明细保留本地，未纳入分享提交）。两组使用 `scene_random_04.xml`（S2-08-r1，身份 `random_04`，19 个障碍）、目标 `(7,0)`、到达半径 `0.5 m`、20 仿真秒运动上限、120 秒墙钟上限、到达后 5 仿真秒站立观察配置；模型路径分别指向 A/C 独立 overlay。

| 组 | 终局与指标 | 来源及边界 |
|---|---|---|
| A | run `[local run ID omitted]`，`VALID / COLLISION_TERMINATION`，终点距目标 3.7569 m；终局为 robot body `RL_calf`（geom 61 名为空）与 `ppt_obstacle_15` 接触。运动有效连续时间 7.774 s，路径 3.858 m，平均速度 0.496 m/s。策略记录 451 帧，频率 50.284 Hz（周期中位数 19.962 ms，P95 31.612 ms），sim clock 无效帧 7。共 137 次模式变化（A→R 68，R→A 69；137 次边沿均有有效 sim time）；0.5 秒短反向切换计数 134。分析器统计的有效片段 Recovery 时间 3.430 s，不能代表无缺口的完整 episode 覆盖。RA 捕获范围 `[-0.9518, 0.3373]`。 | 配置消费 A，E=X=-0.05、滞回关闭、保持关闭；唯一 session。451 帧均为 frame v3、mode=1。碰撞记录含足端 FL/RL 与同一障碍的接触/撞击事件及 RL_calf 终止碰撞；publisher 覆盖不足，累计足端/非足端数量和跌倒否定结论仍 UNKNOWN，分析器跌倒为 UNKNOWN，运动区间存在未确认姿态候选。碰撞终局后 PASSIVE 转换确认，controller 报告转换延迟约 2.355 ms；这不证明 MuJoCo 消费命令。A 的原始帧、context、终局、指标和时间线见本节运行目录下 `A/`。 |
| C | run `[local run ID omitted]`，`STARTUP_FAILURE`，0 个策略帧；未请求 RL、无运动指标，分析结果 INVALID / `no_frame_lines`。 | controller 初始化日志由生产来源核验函数确认：C、E=-0.05、有效 X=-0.05、hysteresis=false、hold=true、H=0.3，Agile/RA/Recovery 路径均属本次 C overlay。复核结果见 C_initialization_source_binding.json（原始运行/环境明细保留本地，未纳入分享提交）。随后唯一性预检发现 ROS node list 出现两条 `/controller_manager` 和两条 `/rl_quadruped_controller`；启动前进程检查为空。重复来源的原因 UNKNOWN。未进入 RL，所以没有 C 的真实 v3 策略帧、保持起算/退出、面板或 Recovery 审计。原始日志和终局保存在同一目录下 `C/`。 |

前一轮 A/C 对照表：comparison.md（原始运行/环境明细保留本地，未纳入分享提交）。按组分别生成的指标位于各自 `analysis/`；前一轮 A 时间线位于该 A run 目录内 `timeline.png`，前一轮 C 是零帧 INVALID，没有运行时间线或策略指标。逐组保持审计历史见 [hold_audit.md](hold_audit.md)。

### 前一轮 C 保持证据范围

C 配置解析和状态机的 56 项 C++ 定向检查通过，A/B 的 292 项回归通过。离线覆盖 RA 等于 E、有效时钟起算、H 到期前不因 RA 降低而退出、elapsed 恰好到 H 且 RA<E 时退出、到期仍高风险时继续 Recovery 且不重启起点、无效/变段时钟不伪造到期；分析对周期缺失、无效时钟和未闭合终局保留删失/UNKNOWN。前一轮仅证实 controller 初始化消费，未观察 C 策略周期保持；本轮 C 的策略周期保持证据及覆盖缺口见本报告开头和 [hold_audit.md](hold_audit.md)。

其他离线检查：runtime frame/HUD 29 项，run recorder 54 项，source binding（含本轮真实 C 初始化日志）通过；hold analyzer、overlay 实际函数隔离、数值格式检查、Python 编译和 `git diff --check` 通过。针对性汇总见 [offline_verification.md](offline_verification.md)。

### 前一轮停止、清理与未解决事项

- A 是有效的碰撞诊断结果，不为改善结果重跑。诊断器记录 PASSIVE 后 controller 状态转换确认，MuJoCo、ROS launch 和 stop publisher 均退出，未强制终止；A 不是 ARRIVED，因此没有 5 秒站立观察。
- C 的基础设施唯一性检查失败后立即关闭本次 MuJoCo、ROS/controller，退出码 0、未强制结束；策略未启动。没有第二次 C 或额外 A/C 运行。
- 两次之后核对没有残留的 MuJoCo、controller-manager、RViz、ROS CLI、诊断器或停止发布器进程；预检窗口、探针及临时缓存已清理。未保存真实 C 面板截图；C 未进入 RL，本轮没有为截图另启运行。
- 本轮失败点是 ROS graph node list 重复项，具体根因未确定。最小后续办法是查明并解决重复节点来源，然后在新授权额度下各运行一次相同安装代码的 A/C；当前不继续重试。controller library mapping 已确认并写入 context 的代码路径已改为在唯一性门槛前立即落盘，避免此类失败丢失证据。
- 截至前一轮，C hold 真实周期行为尚未观察；本轮已取得单次运行证据，但其中 9/19 episode 时钟覆盖不完整。正式阈值/保持参数冻结、暂停与时钟重连、完整碰撞/跌倒覆盖、MuJoCo 命令消费和实机表现仍未由本任务证实。两组单次诊断不用于成功率或性能优劣结论。

## 上轮补证历史（保留）

上轮 A run `[local run ID omitted]` 是 v2 installed controller 下的 VALID / ARRIVED；上轮 C run `[local run ID omitted]` 在 `0.30`/`0.3` 文本比较处 STARTUP_FAILURE。两份原始记录保存在 `logs/manual_ab_compare/20261011_145733_075bc8/`，状态未改写，也不并入本轮 A/C 对照。本轮已修复 hold 数值比较和启动失败摘要；C 的新失败发生在另一个唯一性预检条件。

本轮 S4-01 保持 ACTIVE 等待 Director。用简单的话说：修正后直接检查 ROS 图确认节点唯一，C 组使用真实策略数据完成到达和站立观察，并记录到保持规则按预期工作的周期；时钟缺口使 9 个 episode 的覆盖不完整。C 的真实策略保持证据已补上，但不完整覆盖、运动期跌倒/碰撞否定结论及非实时截图仍有限制。相关项目仿真、controller、ROS launch、RViz、采集器和停止发布器已退出；预先存在的 ROS CLI daemon 未由本次启动、未终止，运行后图中无项目节点。
