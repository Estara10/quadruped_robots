# S6-03 执行报告

**最新状态：COMPLETE — Director 有条件通过（2026-10-12）。** 通过范围为准备准入、实际配置消费、周期0完整帧与初态 companion 关联、任务窗口记录、失败终局分析和进程清理。A、D 均在进入 RL 约 0.46 秒后以 `FALL_TERMINATION` 结束，未接近障碍且未触发 Recovery；这不能归因于退出阈值或保持时间，也不证明算法优劣。工具准备基本完成，但当前不具备启动参数比较的条件。周期缺口、无效时钟、初态差异、未覆盖 Recovery 与到达后站立收尾，以及 controller PASSIVE 不等于 MuJoCo 已消费命令等限制继续保留。历史失败和原始数据未改写。下方执行记录保留当时状态与完整证据。

## Director 有条件通过结论（2026-10-12）

Director 接受本任务在准备准入、配置实际消费、周期0与初态精确关联、任务窗口记录、失败终局分析及进程清理方面的证据，并将 S6-03 标记 COMPLETE。该结论不授权或证明已具备参数比较条件。

最新 A/D 补证均在进入 RL 后约 0.46 秒发生确认跌倒终局；距目标约 7.08 m，未接近障碍，没有 Recovery 周期或模式切换。因没有观察到相关切换行为，结果不能归因于退出阈值或 H，也不支持算法性能结论。两组仍存在策略周期缺口，A 另有无效时钟帧；实测初态不相同。Recovery、ARRIVED 后站立观察/正常到达收尾均未覆盖。PASSIVE 只证明 controller 状态转换确认，不证明 MuJoCo 消费命令。以上限制和所有较早失败、INVALID 及原始数据均保留。

## 本轮 A/D 起立准入与任务窗口补证（2026-10-12）

### 起立门槛修正

源码核对发现，`StateFixedStand::checkChange()` 只有在 `percent_ >= 1.5` 后才接受 command 3；此前单看 FIXEDSTAND 状态或 `[STAND-SYMM]` 只能证明正在执行插值，不能证明已可转换到 RL。`BaseFixedStand::run()` 现在在首次跨过该既有门槛时输出一次 `[STAND-READY] command3_gate=passed percent=... phase=...`。该日志只增加状态证据，不改变插值、控制命令或切换条件。准备观察器将其作为准入条件；未出现时站立稳定计时不开始，保留 20 秒有界准备超时和原物理/安全门槛。A、D 的实测日志均记录 `percent=1.504167` 后才通过连续 0.50 秒物理稳定检查，随后才采具名关节状态和发送 command 3。两次日志还都记录了 `[ABS-INITIAL-STATE]`，且生产 `extract_first_active_policy_state()` 对各自 ROS 日志与 JSONL 返回 `CAPTURED`。

### 实际运行结果

| 组 | 实际消费 | 首个 active 周期 | 终局 / 有效性 | 运动窗口和记录质量 | 收尾 |
|---|---|---|---|---|---|
| A | `paper_faithful_switch`；E=-0.05、有效 X=-0.05、滞回=false、保持=false；独立 A overlay。controller/writer PID [omitted]，运行库映射到本次安装的 `librl_quadruped_controller.so`。 | session `17060890498465`，cycle 0，sim_time=24.608 s，时钟有效；companion 与完整 runtime cycle 0 精确匹配。首态具名 q/dq 各12项；A/D 首态不相同（最大关节位置差约0.00744 rad，速度差约0.00247 rad/s），不视为严格配对。 | Supervisor 与分析器均为 `FALL_TERMINATION`；`VALID`。未到达，终点距目标 7.0783 m；运动窗口跌倒episode sim 24.768 s开始、25.070 s确认。 | 21 个持久帧，3 个策略周期缺口、2 个无效时钟帧；总体捕获策略频率约49.98 Hz。完整经过0.460 s，有效连续0.322 s；有效片段路径0.03192 m、平均速度0.0991 m/s。Agile 21周期、Recovery 0周期、切换0次。声明运动窗口内碰撞覆盖完整，非足端碰撞=false；这仅表示该窗口的已覆盖结果。 | 到达后5秒站立观察不适用；正常 standing hold 未完成。command 1 后 controller `PASSIVE` 已在4.605 ms内确认。清理观察在请求后0.463 ms读到 cycle23，但该帧自身时间早于请求3.753 ms，不证明请求后又作出决策；MuJoCo消费未知。stop publisher退出码1，无 closeout error、未强杀，进程已退出。 |
| D | `combined_switch`；E=-0.05、X=-0.08、H=0.30仿真秒、滞回=true、保持=true；独立 D overlay。controller/writer PID [omitted]，运行库映射同一新安装库。 | session `17109532772547`，cycle 0，sim_time=26.616 s，时钟有效；companion 与完整 runtime cycle 0 精确匹配。首态具名 q/dq 各12项；与 A 有上述可测差异。 | Supervisor 与分析器均为 `FALL_TERMINATION`；`VALID`。未到达，终点距目标 7.0787 m；运动窗口跌倒episode sim 26.776 s开始、27.078 s确认。 | 22 个持久帧，2 个策略周期缺口、0 个无效时钟帧；总体捕获策略频率约49.17 Hz。完整经过0.462 s，有效连续0.402 s；有效片段路径0.04770 m、平均速度0.1186 m/s。Agile 22周期、Recovery 0周期、切换0次。声明运动窗口内碰撞覆盖完整，非足端碰撞=false；这仅表示该窗口的已覆盖结果。 | 到达后5秒站立观察不适用；正常 standing hold 未完成。command 1 后 controller `PASSIVE` 已在3.017 ms内确认。清理观察在请求后1.127 ms读到 cycle23，但该帧自身时间早于请求6.555 ms，不证明请求后又作出决策；MuJoCo消费未知。stop publisher退出码1，无 closeout error、未强杀，进程已退出。 |

两次启动前的 authority 历史各自保留一个早期确认跌倒 episode（sim 0.250 s 开始、0.550 s 确认）。各自的 episode 后来恢复并关闭：A 在 sim 21.854 s / step 10927 关闭，早于任务起点 24.608 s；D 在 sim 23.876 s / step 11938 关闭，早于任务起点 26.616 s。它们由 RL 前边界快照保留，不与任务窗口事件混淆；随后 A、D 各自在任务窗口产生了表中独立的新跌倒 episode。逐 run 的完整配置、初态、两段 fall history、记录身份和质量摘要见 A/D 补证 JSON（原始运行/环境明细保留本地，未纳入分享提交）。A 的原始记录、terminal、metrics、timeline 和日志位于 `logs/manual_abcd_compare/20261012_142851_eea811/A/rep_01/[local run ID omitted]`（原始运行/环境明细保留本地，未纳入分享提交）；D 对应文件位于 `logs/manual_abcd_compare/20261012_142938_246492/D/rep_01/[local run ID omitted]`（原始运行/环境明细保留本地，未纳入分享提交）。每批次的计划、summary、结果表和 timeline 分别见两个批次目录。记录中的 FALL 是合法运动期诊断结果，因而 D 在 A 完成并确认清理后按额度运行；不是准备/基础设施失败。两个 session 的实际 E/X/H、group、candidate、模型/配置路径和 writer/controller PID 均由同一个 launch 来源匹配器确认为一致。

显示预检使用本机 `the active DISPLAY value` 和 `current desktop-session authorization file`，16×16 隐藏 GLFW 窗口成功建立并关闭；renderer 为 Mesa hardware renderer，OpenGL 4.6。controller 增量构建命令 `source /opt/ros/humble/setup.bash && colcon build --build-base build --install-base install --packages-up-to rl_quadruped_controller --symlink-install` 成功完成 `control_input_msgs`、`controller_common` 和 `rl_quadruped_controller` 三包；两次运行的 `/proc/<controller PID>/maps` 都实际指向刚安装的工作区 controller library。构建后定向 Python 编译和 `scripts/test_s6_03_preparation.py` 通过。

**本轮结论和剩余限制：** 新起立门槛记录和物理准备检查在 A/D 两次运行中均通过；两次都保存了周期0初态并打通记录与终止分析。两次初态并非完全一致，且都很快确认跌倒，未触发 Recovery、未到达目标、未完成到达后站立观察。它们只证明本次准备/记录工具链和合法跌倒终局可记录，不证明 A/D 效果或参数优劣。A 有无效仿真时钟帧和策略周期缺口，D 也有周期缺口；完整经过时间与有效连续时间已分开。姿态四元数语义、MuJoCo 对 PASSIVE 命令的消费、跨次启动准备稳定性、其他起始条件及正式跌倒判据仍有既有限制。

## 站立准备失败定向排查（2026-10-12）

**先前 A 失败的直接证据：** run context（原始运行/环境明细保留本地，未纳入分享提交） 只在准备检查首次读到 `fall_confirmed=true` 时保存一个采样：仿真 26.128 s / physics step 13064；该 episode 从 0.250 s / step 125 开始，在 0.550 s / step 275 确认，读取时仍 open。对应 ROS 日志（原始运行/环境明细保留本地，未纳入分享提交） 证明 command 2 后控制器依次切到 FIXEDDOWN、FIXEDSTAND；FIXEDSTAND phase 从 0.004 到 0.398。原监督器在约 0.67 s 的 FIXEDSTAND 运行观察后立即停止。因此，历史数据既没有证明姿态已恢复，也没有给起立过程足够时间；该次 PREPARATION_FAILURE 保持原样。

**最小代码修正：** `confirm_pre_rl_standing()` 不再在第一次读到确认跌倒时立即返回。它在原定墙钟上限内继续读取去重后的物理快照，记录姿态、支撑、tracker 当前状态、episode 开闭、控制器状态和最近监督命令。episode 未关闭或姿态未达既有门槛时仍拒绝准入；若 tracker 观测到正常姿态并关闭 episode，且之后连续 0.50 仿真秒满足原门槛，则保留跌倒历史并允许准备状态变为 STABLE。无效 authority 和超时仍拒绝。新增 `--prepare-only` 只观察准备状态，在任何关节采集和 RL 准入之前退出，代码路径显式禁止 command 3。

**本轮唯一准备阶段诊断：** run `[local run ID omitted]`，capture `p1-10-capture-[local run ID omitted]`；使用 `scene_random_04.xml` / random_04、A (`paper_faithful_switch`, E=-0.05)、goal=(7,0)、既有模型与生产 MuJoCo/controller。控制器实际来源、A 配置、单一活动 controller/ROS 图、场景 authority、预先就绪的记录器及足接口 `FR, FL, RR, RL` 均记于 run context（原始运行/环境明细保留本地，未纳入分享提交）。恢复观察包含 76 个不同 authority sequence，便于检索的 CSV（原始运行/环境明细保留本地，未纳入分享提交） 由该 context 派生。

| 阶段 | 仿真时间 / step | 直接观察到的物理/控制状态 |
|---|---|---|
| 起立仍处于异常 episode | 26.364 s / 13182 | `base_link` 世界 Z=0.20246 m；roll=-0.0000015 rad、pitch=0.00981 rad；4 足支撑；candidate/confirmed=true、episode open；控制器日志态 FIXEDSTAND，最近监督命令 2。该高度低于既有 0.22 m 跌倒阈值。 |
| tracker 关闭 episode | episode end 26.478 s / 13239；首个记录到 closed 为 26.486 s / 13243 | 正常姿态样本令 candidate/confirmed=false、episode `closed=true`；首个 closed 样本 base Z=0.22091 m、roll=-0.0000018 rad、pitch=0.00933 rad、4 足支撑。此前历史 `confirmed=true` 保留。 |
| 稳定准备门槛 | 26.716–27.222 s | 连续 0.506 仿真秒；末样本 base Z=0.29048 m、roll=-0.0000015 rad、pitch=0.01254 rad；4 足支撑、unknown contacts=0、robot-obstacle contacts=0、candidate/confirmed=false、episode closed。生产准备函数返回 STABLE，标记 `recovered_after_confirmed_preparation_fall=true`。 |

**原因判断及边界：** 这次数值表明原失败时 tracker 反映的是实际低机身高度和仍在进行的起立恢复；不是仅因 `fall_confirmed` 是历史累计值。生产 `updateFall()` 把 `fall_confirmed` 作为当前活动异常状态，正常有效姿态会清除当前 candidate/confirmed 并关闭 episode，历史项保持 `confirmed=true, closed=true`。高度来自 MuJoCo `base_link` body origin 的世界 Z；姿态来自 `mjData.xquat` 的世界四元数，源码按 MuJoCo `w,x,y,z` 顺序计算标准 roll/pitch。运行恢复数值与此一致，没有发现四元数排列或转换错误证据。MuJoCo 初始 `base_link` 构造高度为 0.445 m、单位四元数；物理首次跌倒发生在仿真 0.250 s。新 run 的同一 supervisor 单调时钟显示 MuJoCo 启动于 15.825 s、controller active 于 15.835 s、command 2 调用记录约在 15.847 s；状态日志随后确认 FIXEDDOWN/FIXEDSTAND。故物理步在控制器接管前先运行、产生初始低姿态是强支持的启动顺序解释，但 sim_time=0.250 的样本没有附同周期 monotonic timestamp，不能据此认定启动延迟是唯一原因。

截至该次 prepare-only 定向排查时，检查确实在 FIXEDSTAND 起立尚早时结束，内部 ramp 是否达到 `percent_>=1.5` 未由那次短观察实测。随后本报告顶部记录的 A/D 新运行首次取得了实际 `[STAND-READY]` 门槛日志和 RL 周期0；此历史限制对 prepare-only 记录仍成立，但已由本轮新增的 A/D 运行补证。

**记录和清理：** runtime JSONL（原始运行/环境明细保留本地，未纳入分享提交） 只有 meta 与 terminal；策略帧为 0，任务窗/碰撞/跌倒分析均不适用。原始终局为 `PREPARATION_ONLY_COMPLETE`，context 明确 `command_3_sent=false`。cleanup 发布 command 1，controller 日志确认切到 PASSIVE；MuJoCo 在 ROS/controller 前退出且 exit 0，launch 和辅助发布器亦结束（publisher exit 1；controller child 在 launch 收尾时收到 SIGINT）。收尾事实无错误，运行后定向进程检查未见本任务的 MuJoCo、controller、launch、RViz 或 publisher 残留。PASSIVE 确认不代表 MuJoCo 命令消费。

**针对性离线验证：** Python 编译检查通过。`scripts/test_s6_03_preparation.py` 调用生产准备观察函数，覆盖正常起立、confirmed episode 恢复后关闭并保留历史、持续异常超时拒绝、STALE 来源拒绝、准备稳定窗口超时；测试通过。只改 Python 准备观察/终局时间戳记录路径，不需 native 构建。该次运行 terminal JSON 的 `run_terminal_monotonic_ns` 保持当时原始值（null）；同一终局的单调时间在 run context lifecycle event 中有记录。之后已修正 runner，使后续 prepare-only terminal 保存完成时间；不回写本次原始文件。

**该次排查的历史结论：** 已查明早先监督器过早中断了起立观察；物理跌倒恢复和 tracker episode 关闭由 prepare-only 观察直接证实。该次没有 command 3 和 A/D 运行。此后用户另行授权的 A/D 运行及其新证据见本报告顶部；任务仍 ACTIVE/INCOMPLETE。

## 先前 A/D 运行补证（2026-10-12，本次定向排查前；保留历史）

**本次批次：** batch plan（原始运行/环境明细保留本地，未纳入分享提交）、自动汇总（原始运行/环境明细保留本地，未纳入分享提交）、对照表（原始运行/环境明细保留本地，未纳入分享提交）。这是 S6-03 准备核验，不是参数比较或正式实验。

| 核对项 | 直接证据与结果 |
|---|---|
| A/D 参数与目标 | 批次工具为两个独立 overlay 调用 `s1_run_diagnostic.overlay_configuration_matches`。A 接受 group=A、E=-0.05、goal=(7,0)，且不含 X/H；D 接受 E=-0.05、X=-0.08、H=0.30 s 和相同目标。参数按数值比较，A/D overlay 配置是临时目录中的独立普通文件。批次计划保存校验器、配置路径和结果。 |
| A 实际配置/来源消费 | `ros_launch.log`（原始运行/环境明细保留本地，未纳入分享提交） 中 PID [omitted] 的 `[S1-RUN-SOURCE]` 和 `[S2-SWITCH-CONFIG]` 明确记录 paper_faithful_switch、group=A、E=-0.05、有效退出=-0.05、无滞回/保持、目标(7,0)。同一日志分别记录 Agile、RA 及 Recovery artifact 路径；run context 将 controller-manager、唯一活动 controller list、直接 ROS 图、预期 controller library mapping 和场景 authority 绑定到本次 launch。场景为 random_04、19 个障碍，模型与碰撞 authority 指纹一致。 |
| A 记录器及准备顺序 | `run_context.json`（原始运行/环境明细保留本地，未纳入分享提交） 记录 RL 前 recorder meta 已 flush 且回读匹配 run/capture id，唯一 supervisor JSONL writer 打开；本次 launch 中唯一 controller-manager PID [omitted]、唯一活动 controller list、bounded direct ROS graph 及唯一预期 controller library mapping 均有确认。PASSIVE publisher 在 RL 前就绪，controller 收到的足力接口顺序为 FR、FL、RR、RL。策略 session/frame writer 尚未产生，因为 RL 被拒绝。固定站立准备观察发现持续已确认跌倒，因此未采集具名 q/dq，未取得最终 RL 前安全边界，也未发送 command 3。 |
| 准备期安全事实 | 同一 context 的 fall history 显示事件从 sim_time=0.25 s / physics step 125 开始，0.55 s / step 275 已确认；在准备检查时 26.128 s / step 13064 仍为 confirmed 且未关闭。门槛给出 `PREPARATION_FAILURE / refuse_RL_entry`。此为运动任务起点之前的准备期事件，不是任务窗口跌倒指标；它被保留，没有重置历史或延后任务起点。 |
| 监督器终局与分析有效性 | 原始 supervisor 终局为 `PREPARATION_FAILURE`，没有 ARRIVED、任务起点、task-window 碰撞/跌倒结论或终止策略帧。JSONL 仅有 meta 与一条 terminal，`frames_observed=0`；分析器明确 `INVALID / no_frame_lines`，安全指标 UNKNOWN，而不是无安全事件。自动生成的对照表分别显示 supervisor 原始终局、分析终局、有效性和 D 未运行。 |
| cycle 0 与初态 | 本次未进入 RL，故没有 `rl_step=0` frame 或 `[ABS-INITIAL-STATE]` companion；同周期初态状态是 UNKNOWN/未取得，不能由后来周期替代。reset probe 只记录构造态 base z=0.445 m、单位姿态及 qpos，reset qvel UNKNOWN；这不是首个策略周期初态。 |
| 收尾 | publisher 在准备失败后请求 PASSIVE，controller 日志确认完成 PASSIVE 转换；这不是 MuJoCo 命令消费确认。MuJoCo 先于 ROS/controller 关闭，退出码 0；stop publisher 退出码 1但进程已退出；`shutdown_complete=true`、`closeout_complete=true`、无 closeout errors，运行后 `/proc` 定向检查没有本项目 MuJoCo/controller/manager/RViz 残留。正常到达后站立观察未发生。 |
| 桌面预检 | 按已有授权和工具审批，在临时环境下使用 `the active DISPLAY value`、`XAUTHORITY=current desktop-session authorization file` 运行隐藏 GLFW 探针。X 连接、glfwInit、16×16 隐藏窗口均成功；renderer 为 Mesa hardware renderer，OpenGL 4.6 Compatibility Profile Mesa 23.x。临时 `LD_LIBRARY_PATH` 与诊断器构建环境一致；探针和窗口已清理。 |

本轮只启动了 A 一次。由于任务要求准备期安全失败后停止后续场景，D 的配置 overlay 虽经生产校验，场景运行状态仍是 `NOT_STARTED`。本轮没有周期 0 持久化实测、具名 q/dq 样本、运动任务指标、有效 LIVE 策略面板或正常站立收尾证据。因此 S6-03 仍 INCOMPLETE，不能开始候选参数比较；最小后续需求是先由 Director 审阅这一准备期跌倒证据，再明确准备期历史事件的恢复/准入判据后另行授权运行。不得清除历史事件或仅把任务窗口推迟到有利周期。

A 的派生产物可分别复核：metrics.json（原始运行/环境明细保留本地，未纳入分享提交）、analysis result table（原始运行/环境明细保留本地，未纳入分享提交）、timeline.png（原始运行/环境明细保留本地，未纳入分享提交）、raw runtime JSONL（原始运行/环境明细保留本地，未纳入分享提交） 和终局记录（原始运行/环境明细保留本地，未纳入分享提交）。派生分析表里的运动终局为 UNKNOWN 是正确结果：该 INVALID 记录没有策略帧或运动任务窗口；批次对照表另列 supervisor 原始 PREPARATION_FAILURE。

定向代码修正：`extract_first_active_policy_state()` 现要求 companion 和完整持久化策略帧均为周期 0 且 session/cycle/monotonic 时间完全相等；缺周期 0 或必要 payload 字段时返回 UNKNOWN，不接受后续周期代替。批次工具现对本批实际 A/D overlay 调用生产配置校验函数，并将 supervisor 原始终局与分析终局分开列示。针对性检查及 Python 编译通过，见下节。

## 本轮完成的技术准备

- 主任务边界统一为首个持久化 `rl_entered=1 && rl_active=1` 周期，记录 session、cycle、source sequence、monotonic 时间和该周期时钟有效性；终局使用完整终止周期。策略起点时钟无效时保留周期身份并将依赖仿真时间的指标标为 UNKNOWN，不向后移动起点。
- 运行时碰撞、跌倒和 20 仿真秒限制现在从首个活跃策略周期起算。运行时安全事件依据 v5 累计 episode 与首个活跃周期时的仿真时间筛选；起点时钟无效时依据 RL 前 LIVE 边界的 physics step 保守筛选。分析器从同一任务起点统计切换和安全，清理与站立尾段仍排除。首个位移达到 0.10 m 的起点保留为旧口径辅助值。
- recorder 在 RL 请求之前启动，读取回 meta 并核对 run/capture identity；策略 supervisor 对共享帧同步持久化。companion 初态必须与首个 active JSONL 帧的 session、cycle 和 monotonic 时间精确相等，否则标记 UNKNOWN，并由批次流程停止后续组。
- 共用 PASSIVE/FIXEDSTAND 准备门槛固定为连续 0.50 仿真秒：PhysicsLoop 覆盖有效、未知接触为 0、至少两个足端支撑、机身高度不低于 0.25 m、绝对 roll/pitch 不超过 0.30 rad、无跌倒候选/确认、无机器人障碍接触；准备墙钟上限 20 s。随后采集具名 12 关节 position/velocity，要求有限值且最大绝对关节速度不超过 0.50 rad/s。失败分类为 PREPARATION_FAILURE，不进入 RL；已观察到的姿态异常保留在准备记录中。
- 提取 `rl_entry_precondition_failure()` 作为 command 3 前的最终 fail-closed 检查，覆盖 recorder、站立、关节样本/速度和 RL 前安全边界。
- `manual_ab_compare.py` 支持 E、X、H、组顺序和有限重复数，先保存批次计划，再创建独立实体 overlay；参数数值不从字符串拼写判断。A/C overlay 不含退出阈值，A/B 不含保持字段。汇总分开列计划、已启动、有效、INVALID、未开始、终局、碰撞/跌倒 UNKNOWN 和准备就绪状态；没有周期数据时不伪造指标。单次运行的 metrics、Markdown 对照表和 timeline 由现有分析器生成。
- analyzer 的离线回归发现任务窗口距离计算读取目标坐标早于赋值，已修复初始化顺序，并由实际 analyzer 重跑副本验证。

## 定向离线验证

以下表格记录的是 A/D 运行前、当时 Python-only 修改阶段的既有检查结果；当前 controller 增量构建与 A/D runtime library 映射证据见本报告顶部“本轮 A/D 起立准入与任务窗口补证”。

初态和任务窗口口径细节见 [首周期/初态/任务窗证据说明](INITIAL_STATE_AND_WINDOWS.md)，工具使用与失败停止步骤见[运行准备指南](PREPARATION_AND_RUN_GUIDE.md)，场景既有覆盖见[场景覆盖表](SCENE_COVERAGE.md)，指标定义见[指标可用性清单](METRIC_AVAILABILITY.md)。

| 检查 | 结果 |
|---|---|
| Python 编译：supervisor、analyzer、批次工具、发布器及针对性检查脚本 | 通过 |
| `scripts/test_s6_03_preparation.py` | 通过：参数有限性/边界、A-D overlay 字段隔离、0.30 与 0.3 数值等价、完整 cycle-0 payload 精确配对、精确配对但仅有较晚周期时拒绝、cycle-0 payload 缺字段时拒绝、任务窗口安全事件边界、覆盖不足 UNKNOWN、失败准备阻止 RL、批次 UNKNOWN/未运行及原始终局分类 |
| `scripts/test_s1_runtime_source.py` | 通过，既有来源匹配回归仍正常 |
| 首批 A 事故保存的 overlay config（当时离线复核） | 通过：`group=A`、数值 E=-0.05、goal=(7,0)、无退出/保持字段被接受；当时没有 controller 实际重新消费。最新实际 A/D overlay 检查及 A 的 controller 消费证据见本报告顶部 |
| 实际 analyzer + plotting interpreter 对既有 S6-01 D 记录副本运行 | 首次测试暴露目标变量赋值顺序错误，修复后生成 metrics、Markdown 结果表和 timeline.png；未改历史记录。扩展到首个策略周期的安全窗口发现一条从 26.500 s 延续至 27.078 s 的已确认跌倒，覆盖任务起点 26.602 s，因此新口径将该旧 ARRIVED 记录标成 `INVALID_TERMINAL_SAFETY_CONFLICT`；碰撞仍为 UNKNOWN，因为原记录没有任务起点之前的边界快照。此为离线重算发现，未回写或改判旧报告和原始数据。详见 历史窗口离线复核（原始运行/环境明细保留本地，未纳入分享提交）。 |
| Native 构建（当时） | 该 Python-only 阶段未执行，诊断在加载控制器前失败，故当时未取得运行中的 library mapping。后续 controller 起立日志改动的增量构建已通过，且顶部两次运行均实测映射到新安装库。 |

## 先前获准批次（首个 INVALID，原始记录保留）

计划及结果：批次计划（原始运行/环境明细保留本地，未纳入分享提交）、自动汇总 JSON（原始运行/环境明细保留本地，未纳入分享提交）、对照表（原始运行/环境明细保留本地，未纳入分享提交）。实际场景固定为 `scene_random_04.xml` / S2-08-r1，目标 `(7,0)`、半径 0.5 m，运动上限 20 仿真秒、墙钟 120 秒、到达后站立目标 5 仿真秒；参数 E=-0.05、X=-0.08、H=0.30 仿真秒。预览 renderer 为 Mesa hardware renderer / OpenGL 4.6 Mesa，与已有 S6 预检记录一致。

对照表如实列出 A=INVALID、D=未运行及各任务指标 UNKNOWN；没有有效策略帧，故本批没有可复核的策略时间线、Recovery 指标或 LIVE 面板截图。批次表不是有效 A/D 对照。

| 组 | 结果 | 首帧/任务数据 | 停止情况 |
|---|---|---|---|
| A | `STARTUP_FAILURE / INVALID`，run `[local run ID omitted]`，0 policy frames | runtime recorder 未启动；无策略初态、joint sample、配置消费、session/writer 绑定或任务安全窗口，均 UNKNOWN | controller、MuJoCo 和 publisher 均未启动；`shutdown_complete=true`、`closeout_complete=true` |
| D | 未运行 | 不适用 | 按基础设施失败停止规则，不启动 |

A 被拒绝的具体原因为 overlay 将 E 序列化为 `-0.050000000000000003`，启动预检却以字符串要求恰好等于 `-0.05`。goal、组别与本次计划参数已保存在 context/overlay；但没有 controller、MuJoCo 或策略帧的实际消费证据。之后已修正为有限数值解析和容差比较，离线覆盖 `-0.05`、`0.30`/`0.3`、NaN 与错误数值；由于失败停止点，本轮不再启动场景。该配置错误为本次直接证实的失败，不证明其是历史相同失败的原因。

GLFW/OpenGL：默认沙箱下 XOpenDisplay 和 GLFW 初始化失败；按已授权的最小沙箱外访问，使用当前 `the active DISPLAY value`、`XAUTHORITY=current desktop-session authorization file` 成功连接、创建隐藏 GLFW 窗口并读取上述 renderer。探针窗口已销毁。没有更改授权文件、权限、驱动、安装或系统设置。

## 先前执行后的缺口记录（本轮新补证见报告顶部）

- 首帧 companion/JSONL 精确配对、RL 前稳定门槛、实测关节初态、实时配置/模型/唯一 writer 来源和自动收尾只完成实现/离线核验；本批没有有效场景运行来证明它们实际打通。
- A 的唯一场景尝试已消耗本轮失败停止点，D 未启动。继续完成 A/D 运行核验需要 Director/用户重新明确允许补跑额度；不能自行重跑或将该 INVALID 改作成功数据。
- 用当前新窗口规则离线复核旧记录会识别此前位移起点遗漏的事件。正式比较前需要新口径的有效运行和明确的安全边界；不能将历史 A/B/C/D 当作同口径参数比较。
- 首态四元数日志标注为 `framework-native`，目前没有直接证据确认其分量顺序/参考系约定；保留原值，不用于初态相似度。足接触来源、模型动作顺序的既有限制、碰撞覆盖和跌倒参数适用范围、PASSIVE 与 MuJoCo 命令消费的区别继续保留。
- 已运行一次默认图形预检；没有有效 LIVE 面板截图。D 未运行，不能报告 D 的 hold 审计或实时显示。

## 变更文件

`scripts/s1_run_diagnostic.py`、`scripts/s1_analyze_run.py`、`scripts/manual_ab_compare.py`、`scripts/s2_03_stop_publisher.py` 和新增的 `scripts/test_s6_03_preparation.py`。未修改模型、研究参数、地图或控制器 C++ 算法。本轮没有提交 Git。

**先前批次环境：** GLFW 预检窗口已关闭；当时 A 在配置预检失败后核对相关项目进程无残留，D 未启动。本轮补证清理情况见报告顶部。
