# T1（原S6-12）执行报告：B组退出阈值粗选

**最新状态：ACTIVE / INCOMPLETE — 第2项的一次补做有效，随后继续冻结计划至第66项；第66项因任务期安全覆盖为UNKNOWN而按停止规则停批，剩余30项未启动。**

## 最新续跑补证与停止原因（2026-10-15）

按用户授权，保留首项原有结果，对计划项`002_m1ssd01_x17_r1`只补做一次。生产reader读后取校验时间的修复已通过定向离线检查；恢复入口也只接受冻结批次精确状态（首项有效、第2项恰有一个已清理的INVALID准备失败、其余94项未启动），并拒绝重复补跑、清理未确认和后续项已启动等变体。

本次GLFW最小预检与实际续跑在同一获批沙箱外执行上下文中完成：`DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority`、`MUJOCO_GL=glfw`、`/usr/bin/python3`；16×16窗口及OpenGL context创建成功，renderer为Intel / Mesa Intel(R) Graphics (RPL-P)，OpenGL 4.6。直接预检结果的结构化留档见[同权限上下文GLFW记录](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/glfw_t1_same_context_capture.json)；之前的系统库路径与`ldd`记录见[同环境预检详情](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/glfw_same_environment_preflight.json)。

计划项2在独立的[attempt_02目录](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/runs/002_m1ssd01_x17_r1/attempt_02/result/B/rep_01/772774129dda4ef8aa78ed3d87b8a41d/)完成：实际消费B组、E=-0.05、X=-0.17、无保持，目标与冻结计划一致。任务以ARRIVED结束，距目标0.4995 m；策略cycle 0–451连续，终止帧完整持久化；共519个策略帧，末次排空无丢失/覆盖，完成约5秒站立观察，诊断器确认本次MuJoCo、ROS/controller和查询/命令会话退出，记录VALID、清理完成。原attempt仍为`PREPARATION_FAILURE / INVALID`，未改写。

项2通过后按原顺序继续，未改参数、场景或顺序。运行器到第66项停止：计划项`066_m1dld01_x09_r1`（dev27，B，X=-0.09）收到`SYSTEM_SAFETY_ABORT`，command 1到PASSIVE已由controller日志确认，安全事件保留。该run的JSONL记录本身为`record_validity=VALID`，224帧cycle 0–223连续，最终producer/consumer排空完整；但终止发生时没有可关联的任务终止周期或终止物理快照，`motion_metric_time_boundary.end_sim_time_s=null`，碰撞/跌倒任务窗口分析均为UNKNOWN，原因`task_window_frames_or_time_bounds_missing`。因此它不是普通任务失败可继续的完整安全结果，而是本任务要求停止的安全记录覆盖缺口。原始数据见[第66项运行目录](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/runs/066_m1dld01_x09_r1/result/B/rep_01/a93bdb05a7fe4686856b7bffd79249ce/)；未重试，也未启动第67项。

批次最终为96项计划、66项已尝试、68次attempt（包括首项和第2项各自保留的历史失败）、65项`VALID_TASK_RESULT`、1项因任务安全覆盖UNKNOWN停止、30项未启动。全部attempt终局计数为：ARRIVED 39、COLLISION_TERMINATION 18、DIAGNOSTIC_SIM_TIME_LIMIT 8、SYSTEM_SAFETY_ABORT 1、PREPARATION_FAILURE 1、STARTUP_FAILURE 1。此处每档最多两次且批次提前停止，只是开发集粗选流程记录，不构成成功率或性能排名；当前证据不足以估计细选区间。

第66项停止后核对当前进程视图，没有本项目MuJoCo、controller、RViz或诊断runner进程；证据见[进程清理核对](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/postrun_process_check_t1.json)。运行器增加的第2项恢复入口有严格冻结状态门槛；停止后调用生产部分汇总时，发现旧绘图辅助函数在ARRIVED耗时序列引用局部变量错误，并且`attempted`字段按96行而不是有attempt的计划项计数。已做针对性修正、Python编译并重新生成当前部分结果表、图表和代表时间线清单；不涉及任何场景重跑。第67–96项保持未运行，需Director后续决定；本轮不细选、不用留出集、不提交Git。

此前的reader离线修复和首次批次记录如下，均为历史阶段，不覆盖上述最新续跑状态。

本轮只修正生产 reader 的校验时序，验证 pre-RL gate，并更正上一轮故障解释。没有启动仿真、续跑、参数变更或留出集运行。批次仍是96项计划、2个计划项曾尝试、94项未启动、含原始GLFW失败共3次attempt、1条有效任务结果。

## 历史补证：场地边界快照读时序（仅离线；最新状态见报告顶部）

第2项直接触发RL准入拒绝的记录是 `run_context.initial_state.pre_task_safety_boundary.base_bounds_snapshot`：版本1、场景`m1ssd01`、capture ID及模型指纹与本run相符，物理step 3129 / sim_time 6.258 s，`bounds_valid=1`、`physics_coverage_complete=1`；reader记为`INVALID / future_timestamp`、`age_ns=-163750`。这是该次边界读取的原始结论，原始JSON未改写。

reader源码在获取一致快照前调用`time.monotonic_ns()`，随后再打开并读取共享快照。若生产者在该校验时刻之后、读到一致快照之前发布，快照时间戳会合法地略晚于过早取得的校验时间。原始记录未保存该reader自己的校验时间，因此无法从历史记录证明这次发布恰好发生在该竞态窗口内；代码时序漏洞确实存在，且与负年龄现象相符，但对本次故障的唯一因果仍不是运行时实测结论。

另一个时间点是此前记录的`pre_task_safety_boundary.observed_monotonic_ns=37634655105467`，它对应独立物理/碰撞快照路径，不是base-bounds reader实际用于计算`age_ns`的时刻。base-bounds时间戳为`37634656408048`；历史快照没有保存其校验时刻，不能把这两个字段拼接成该reader的年龄。终端中`STALE / snapshot_expired / age_ns=854600025`来自稍后的整段采集边界检查（物理step 3136、sim_time约6.272 s），不是RL准入拒绝的直接快照；它仍说明采集区间覆盖不完整，但不能替代准入根因。上一版把第2项归因为该STALE，是阶段间/快照间混读，现予更正；该旧表述作为历史报告解释保留，不改原始数据。

修正见[`abs_base_bounds.py`](../../../../scripts/abs_base_bounds.py)与[`s1_run_diagnostic.py`](../../../../scripts/s1_run_diagnostic.py)：生产reader在读取完整且seqlock一致的快照后才取得默认单调校验时间，并用同一个时刻计算年龄、future与STALE。显式`now_ns`仍可注入，语义明确为“调用方指定的校验时刻”，仅供确定性离线检查；生产pre-RL调用不注入读前时刻。返回快照现在附带`validation_monotonic_ns`，便于解释校验年龄。准入helper直接消费reader结果，仍要求LIVE、`bounds_valid=1`及完整物理覆盖；失败仍拒绝RL，command 3仍只在所有准入通过后发布。没有改变publisher、500 ms时限、身份条件或任务行为规则。

定向离线检查调用生产reader及实际pre-RL边界准入helper，覆盖并通过：模拟合法并发发布落在旧读时刻与一致读取之间；真正未来时间戳拒绝；恰好500 ms仍LIVE、超过500 ms转STALE；身份错配、版本/长度错误和seqlock不稳定拒绝；显式校验时间注入；边界无效或覆盖不完整时gate拒绝。具体检查记录见[reader离线验证说明](S6-12_BASE_BOUNDS_READER_OFFLINE.md)。`/usr/bin/python3`定向检查及三个受影响Python文件的编译检查通过。未做MuJoCo publisher握手或运行时重验，因为本轮明确不启动仿真。

本修正仅影响边界快照reader的年龄校验与准入元数据，不修改状态机、参数、地图、控制链或运动指标定义。**尚无修复后的现场读取证据**；原计划项2仍是`PREPARATION_FAILURE / INVALID`，不改判为有效任务结果。修复审阅后，是否对第2项作一次补做及继续剩余计划，需另行授权。

本轮按最新授权保留首次GLFW失败，使用与实际MuJoCo相同的获批执行环境补做首项一次；该项得到VALID任务记录和完整清理后，继续冻结顺序的后续项。下一项未能通过RL前场地边界数据准入，按规则停止。没有启动细选或留出集。

## 历史批次状态（截至读时序补证；非当前）

| 口径 | 数量 | 说明 |
|---|---:|---|
| 冻结计划项 | 96 | A参考12项、B候选84项 |
| 本轮新增场景启动 | 2 | 首项补做一次；成功后继续到原顺序第2项 |
| 全批次尝试（含历史首次失败） | 3 | 首项原始失败、首项补做、计划项2准备失败 |
| 有效任务结果 | 1 | dev03 / B / X=-0.15，仿真时限结束 |
| INVALID或停止项 | 1个计划项 | dev03 / B / X=-0.17，直接准入快照为`future_timestamp`；保留INVALID |
| 未启动 | 94 | 计划项3–96 |

逐计划项汇总、逐attempt台账、冻结计划和部分图表位于[批次目录](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/)。原始首项失败不改写；获准补做写入独立`attempt_02`目录。

## GLFW执行环境一致性

本次最小GLFW预检与真实批次入口在同一获准的沙箱外父执行进程中顺序运行。预检通过后，启动批次的Python进程及其MuJoCo/ROS子进程继承同一授权执行上下文。预检与仿真使用相同的用户`lidio`（uid/euid 1000）、`DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority`、`MUJOCO_GL=glfw`及MuJoCo运行时`LD_LIBRARY_PATH`；授权文件可读。Anaconda Python没有加入仿真库搜索路径。

预检建立并关闭16×16隐藏窗口，renderer为Intel / Mesa Intel(R) Graphics (RPL-P)，OpenGL 4.6。预检二进制和MuJoCo实际主程序都加载`/usr/lib/x86_64-linux-gnu/libglfw.so.3`；详细用户、环境、renderer及两者`ldd`输出见[同环境预检记录](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/glfw_same_environment_preflight.json)。此次实际MuJoCo成功初始化GLFW并发布本次物理authority，首项补跑通过；因此之前沙箱探针与实际仿真权限不一致的问题在本次补跑路径中得到运行证据支持。没有修改桌面授权、驱动或全局环境。

## 历史首项补跑：dev03 / B / X=-0.15

计划项`001_m1ssd01_x15_r1`的原始attempt为`STARTUP_FAILURE / INVALID`，`mujoco.log`记录`could not initialize GLFW`；该记录保留在[原始attempt目录](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/runs/001_m1ssd01_x15_r1/result/B/rep_01/81cc988dca294a9d8490f1f287333919/)。用户授权的唯一补做保存到[attempt_02](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/runs/001_m1ssd01_x15_r1/attempt_02/result/B/rep_01/7d9ba20b205b4704af31ff6bac290f14/)，没有覆盖前一条。

实际来源确认controller/writer/session、模型及配置对应本次B overlay；消费组别B、E=-0.05、X约-0.15、滞回开启、保持关闭、目标(7.5,0)。采集器在启动前就绪。周期0与初态companion同session/cycle关联，周期0及终止帧payload均持久化。策略任务窗口cycle 0–1001连续、无周期缺口、无无效策略时钟；全run共保存1082帧，producer与consumer均为1082，最终排空完成。

| 指标 | 本次单条有效记录 |
|---|---:|
| supervisor终局 | `DIAGNOSTIC_SIM_TIME_LIMIT`，不是到达 |
| 到终点距离 | 2.561 m |
| 任务耗时 / 有效连续时间 | 20.018 s / 20.018 s（覆盖率1.0） |
| 路径长度 / 平均速度 | 11.227 m / 0.561 m/s |
| policy频率 | 50.15 Hz；中位间隔19.991 ms，P95为25.829 ms |
| 模式边沿 | 99次完整；Agile→Recovery 50次、Recovery→Agile 49次 |
| 0.5秒短反向切换 | 88次，覆盖完整，约263.76次/有效任务分钟 |
| Recovery | 9.218 s，占有效任务时间46.05%；461个Recovery周期 |
| 任务窗口碰撞/跌倒 | 指标分析为false/false，使用本任务窗口完整的物理覆盖；4个姿态候选均关闭且未确认 |
| 收尾 | 站立观察COMPLETE；正常关闭请求已确认，相关进程正常退出 |

该运行任务终局由20秒仿真任务上限触发。run从任务起点sim_time约6.392 s运行至约26.410 s，终止时处于Recovery，距离目标2.561 m。终止策略帧cycle 1001、sequence 2004与持久化payload对应。一个有效B观测只能说明此场景此阈值的一次结果，不构成档位比较。

安全范围需区分：任务窗口内，指标器依据连续物理发布历史评估为碰撞false、跌倒false；未观察到足端接触/撞击或非足端碰撞。全采集窗口的`terminal.json`仍记录碰撞UNKNOWN，并记录一次已确认低高度事件：sim_time约0.25 s开始、0.55 s确认、4.72 s关闭，早于RL任务起点6.392 s。因此不能把全run概括为“无跌倒”或“全采集无碰撞”。任务窗口判断不能外推到准备阶段；base_link边界监测也不等于整机足迹监测。命令接口记录不证明MuJoCo消费。

## 计划项2原始attempt_01：dev03 / B / X=-0.17（attempt_02见报告顶部）

原始attempt_01在修复前因pre-task base-bounds snapshot `INVALID / future_timestamp`、age -163750 ns而拒绝RL，结果为`PREPARATION_FAILURE / INVALID`，0策略帧、没有实际配置消费或任务指标；历史原始数据未改写。terminal稍后记录的step 3136 `STALE / snapshot_expired`属于另一个采集覆盖检查，不是准入失败原因。计划项同一冻结ID下的有效attempt_02和最新结果见本报告顶部。

原始记录：[run_context](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/runs/002_m1ssd01_x17_r1/result/B/rep_01/33a9ddddc59a44e3a41e358e95b3704f/run_context.json)、[terminal](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/runs/002_m1ssd01_x17_r1/result/B/rep_01/33a9ddddc59a44e3a41e358e95b3704f/terminal.json)和[process facts](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/runs/002_m1ssd01_x17_r1/result/B/rep_01/33a9ddddc59a44e3a41e358e95b3704f/process_facts.json)。没有重试，也未启动计划项3–96。

## 当前部分结果与图表

部分结果表按计划项显示最新attempt；完整历史attempt见[attempt台账CSV](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/coarse_attempt_results.csv)。计划项CSV、attempt CSV和[机器可读摘要](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/coarse_partial_summary.json)分别保留VALID结果、安全覆盖UNKNOWN停止项、历史INVALID attempt及未启动分母。当前摘要为66项有attempt、68次attempt、65项有效任务结果、30项未启动。

当前诊断图只纳入有VALID任务结果的计划项；无效启动/准备失败与安全覆盖UNKNOWN停止项留在台账/执行状态图中，不冒充导航终局：

- [粗选计划执行状态：65有效、1安全覆盖停止、30未启动](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/execution_status.png)
- [有效记录终局构成](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/terminal_composition.png)
- [短反向切换频率](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/short_reverse_switch_count.png)
- [Recovery占比](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/recovery_share_valid_motion_time.png)
- [到达耗时](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/elapsed_task_duration_s.png)：仅ARRIVED有效样本有到达耗时，样本数见图标题/表格。
- [地图×阈值ARRIVED热图](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/map_threshold_heatmap.png)：只反映已运行且VALID的粗略计数，覆盖未满，不能排名。
- [按预定规则选取的代表时间线清单](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/representative_timelines.json)；原始逐run时间线位于各计划项结果目录中。

图仅展示部分粗选覆盖，不足以比较阈值优劣或估计细选区间。

## 当前结论、剩余工作与清理

- 当前65个有效计划结果覆盖多个阈值，但粗选在第66项停批、30项未启动，且一次任务窗口安全覆盖UNKNOWN；不足以给出完整矩阵比较或细选区间，不构成成功率或性能排名。
- 第66项`SYSTEM_SAFETY_ABORT`与PASSIVE确认保留；其记录及最终排空完整，但任务终止周期/物理窗口边界缺失，所以碰撞和跌倒结论为UNKNOWN。该缺口触发停批，没有重试。
- 第2项恢复入口、attempt级CSV和部分汇总已针对续跑核对；汇总工具的两个定向缺陷修正后，表、图和代表时间线清单重新生成。生产base-bounds读取标准没有放宽。
- 最终[进程清理核对](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/postrun_process_check_t1.json)未发现本任务MuJoCo、controller、RViz或诊断runner残留。第2项及之后逐项process facts均显示shutdown/closeout完成；第66项controller PASSIVE转换已确认。GLFW预检窗口已关闭。
- 留出集没有用于运行或选参；未启动细选、训练、正式实验或实机测试；未提交Git。历史原始运行保持不变。

T1保持ACTIVE / INCOMPLETE，阶段2粗选尚未完成；第67–96项是否续跑、如何补足第66项终止边界等待Director审阅。
