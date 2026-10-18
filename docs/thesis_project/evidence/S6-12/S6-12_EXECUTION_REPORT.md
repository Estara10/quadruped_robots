# T1（原S6-12）执行报告：B组退出阈值粗选

**最新状态（2026-10-16）：T1 ACTIVE，96项粗选矩阵已完成，等待 Director 审阅。** 本报告新增“跌倒事件时间修复与最后13项”作为当前结果；下方较早的第83项停批文字均为历史记录，保留原判断以供追溯，不代表当前状态。

## 最新执行：跌倒事件时间修复与第84–96项（2026-10-16）

按[本轮授权提示词](../../../../archive/markdown/20261018/docs/thesis_project/T1_FALL_ANALYSIS_AND_FINAL_13_PROMPT.md)完成生产分析修正、离线针对性验证、独立重分析及冻结矩阵最后13项。没有改变控制器、射线、RA、模型、参数、地图、安全规则或运行时限；没有重跑第83项、没有增加超出授权的场景启动、没有启动细选或使用留出集。

### 分析器修正与第83项

旧逻辑按物理快照的发布时间截断整个跌倒历史。本次改为按同一来源下事件自己的开始、确认、关闭时间和物理步判断任务窗口归属：终止后发布的同run边界快照可证明其内明确早于任务终止的确认事件；终止后才确认不倒算为任务期跌倒；终止后才关闭的候选不能当作终止前关闭；没有可用关闭时刻或物理步依据时保留UNKNOWN。准备期已关闭事件仍排除，跨入运动期且在任务期确认的事件纳入；来源冲突、历史溢出和无效姿态继续按原覆盖要求拒绝否定结论。新增回归核验覆盖这些边界、覆盖完整要求和实际生产分析入口。

第83项原始记录未改写，派生分析位于[独立重分析目录](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/derived_reanalysis/fall_timefix_20261016_001/)。策略终止为13.874 s；周期205附带的同run物理快照发布时间为13.878 s、physics step 6939，其中低高度事件已在13.862 s、step 6931确认。快照身份与来源匹配，任务窗口物理发布覆盖完整且fall history无溢出。生产分析现确认该事件属于任务窗口，结果为`VALID / FALL_TERMINATION`、`motion_fall_result=true`；按合法失败保留，不重跑。

针对性验证与编译通过：`python3 -m py_compile`覆盖批次入口、分析器及两份受影响检查；`scripts/test_s1_04_contract.py`验证终止后发布的快照携带终止前确认、终止后确认/关闭、准备期关闭/跨界、来源冲突、溢出、无效姿态和完整覆盖要求；`scripts/test_s6_12_ray_repair_resume.py`验证只接受本批第83项派生重分析已完成且84–96未启动的精确恢复状态，并拒绝变更的批次/前缀/证据/后缀。

全run捕获的碰撞覆盖仍有无效样本/producer覆盖不全事实，与任务窗口覆盖是不同统计范围。第83项的全采集限制仍保留，不能把本次修复解释为整个采集期覆盖完整。

对修复前已经存在的86次 attempts 全部使用实际 `scripts/s1_analyze_run.py` 生产入口做独立派生分析，旧输出保留。对比显示两项安全解释变化：第51项仍是碰撞终局，但一个终止时未关闭的姿态候选使跌倒由False改为UNKNOWN；第83项从`INVALID_TERMINAL_SAFETY_CONFLICT / UNKNOWN`改为与监督器一致的`FALL_TERMINATION / true`。没有ARRIVED记录被重评为已确认的任务期跌倒。差异明细见[重分析差异表](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/derived_reanalysis/fall_timefix_20261016_001/reanalysis_delta.json)，86条 attempt 的派生结果与来源索引见[重分析结果](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/derived_reanalysis/fall_timefix_20261016_001/all_attempts_analysis/run_results.md)和`attempt_provenance.json`。原始JSONL、context、terminal以及原先指标/时间线没有被覆盖。

### 最后13项运行结果

按冻结顺序完成计划项84–96，各一次、无重试。每项均确认实际配置和来源；周期0与任务记录、完整终止帧、末次排空及自动分析齐全，策略记录无本次新增的缺周期/无效策略时钟；13条记录均为`VALID_TASK_RESULT`。12项到达或仿真时限正常完成约5秒站立观察；第89项碰撞终止，没有站立观察。所有项目的controller/MuJoCo/ROS及辅助进程清理事实均为完成。

| 项目 | 阈值X | 终局 | 终点距离 | 运动耗时 | 任务窗碰撞/跌倒 | 站立观察 |
|---:|---:|---|---:|---:|---|---|
|84|-0.19|仿真限时|4.080 m|20.020 s|False / False|完成|
|85|-0.09|到达|0.483 m|8.720 s|False / False|完成|
|86|-0.13|到达|0.491 m|5.720 s|False / False|完成|
|87|-0.07|到达|0.485 m|7.000 s|False / False|完成|
|88|-0.17|仿真限时|4.056 m|20.020 s|False / False|完成|
|89|-0.07|碰撞终止|2.964 m|6.360 s|True / False|未到达|
|90|-0.13|到达|0.491 m|6.740 s|False / False|完成|
|91|-0.17|到达|0.499 m|7.318 s|False / False|完成|
|92|-0.09|到达|0.495 m|8.180 s|False / False|完成|
|93|-0.15|到达|0.496 m|9.520 s|False / False|完成|
|94|-0.19|仿真限时|4.153 m|20.020 s|False / False|完成|
|95|-0.05（A参考）|到达|0.492 m|8.220 s|False / False|完成|
|96|-0.11|到达|0.497 m|7.620 s|False / False|完成|

项目级完整指标见[96项结果表](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/coarse_run_results.csv)，全部99次attempt见[attempt表](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/coarse_attempt_results.csv)。当前计划项汇总和主要图表：

- [跨地图阈值汇总与候选建议](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/coarse_comparison_summary.md)；[地图×阈值48行明细](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/map_threshold_summary.csv)
- [终局构成](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/terminal_composition.png)、[短反向切换](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/short_reverse_switch_count.png)、[Recovery有效运动时间占比](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/recovery_share_valid_motion_time.png)、[到达耗时](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/elapsed_task_duration_s.png)
- [地图×阈值到达热图](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/map_threshold_heatmap.png)、[碰撞结果](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/collision_outcome_by_threshold.png)、[跌倒结果](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/fall_outcome_by_threshold.png)、[执行状态](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/execution_status.png)
- [本轮同环境GLFW预检/renderer记录](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/glfw_t1_same_context_capture.json)；[13项清理核对](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/postrun_process_check_t1_fall_final13.json)

### 全批粗选与候选范围

冻结矩阵96项全部有最新有效任务结果；含保留重试共99次attempt，其中97次记录VALID、2次历史INVALID。99次原始attempt终局构成为ARRIVED 65、COLLISION_TERMINATION 19、DIAGNOSTIC_SIM_TIME_LIMIT 11、FALL_TERMINATION 1、SYSTEM_SAFETY_ABORT 1、PREPARATION_FAILURE 1、STARTUP_FAILURE 1。96个计划项最新结果为ARRIVED 65、碰撞19、仿真限时11、跌倒1。该口径不把两条历史INVALID attempt伪装成有效导航结果；对计划项“有最新有效结果”也不意味着所有安全指标都为False。

新逐地图×阈值表按每个计划项最新有效尝试汇总，8个X档各12项、覆盖dev03/09/11/15/27/29六图；每档描述性到达数为5–10，碰撞终局为1–4，跌倒终局仅X=-0.05有1次（第83项）。修复后当前96项任务窗口安全字段为碰撞19 True/77 False，跌倒1 True/94 False/1 UNKNOWN。否定值仍按各自任务窗口覆盖判断，不外推到全采集期。短反向统计按记录范围报告；早停运行的较低切换量不能解释为机制改善。Recovery驻留/占比只使用当前有效时间范围。

粗选只支持后续设计候选，**不支持稳定排名或最优参数结论**。若Director授权细选，可优先考虑X=-0.11至-0.17的探索带，再用预先定义的配对重复检验；其中-0.11至-0.13在本批到达数相对较多，-0.15至-0.17有不同的碰撞/耗时权衡。该范围是基于每档12项的描述性观察，不是正式选择：运行跨越射线修复前后两个controller阶段（65项旧阶段由台账顺序推定、31项新阶段为`cd6021faa96b0cc8`；旧run context没有逐条保存stage ID），地图与初态不同，合法早停改变可观测切换和时间，且安全覆盖仍有适用边界。X=-0.19出现较多仿真限时，暂不优先但不据此排除。未启动细选、未使用留出集。

### 收尾

每个新增run均有shutdown与closeout事实。最终通过精确进程名检查，未发现本轮MuJoCo、controller/ROS、RViz或批次辅助进程残留；批次wrapper退出码0、13个子运行清理事实完整，见[清理核对](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/postrun_process_check_t1_fall_final13.json)。原始第83项、其旧分析和所有历史失败均保留。T1保持ACTIVE等待Director最终审阅，不提交Git。

以下各段记录本次授权前的停批、早期运行和历史分析，均按原状保留，不代表当前状态。

## 本轮射线修复、第66项补做及粗选续跑（2026-10-16）

按[T1执行提示词](../../../../archive/markdown/20261018/docs/thesis_project/T1_RAY_REPAIR_AND_COARSE_COMPLETION_PROMPT.md)完成定向修复后，在同一获准桌面执行上下文中通过GLFW/OpenGL预检并运行批次。预检使用`DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority`、`MUJOCO_GL=glfw`和系统`/usr/bin/python3`；实际renderer为Intel / Mesa Intel(R) Graphics (RPL-P)，OpenGL 4.6。窗口已关闭，详细环境记录见[本轮预检](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/ray_repair_display_preflight_20261016.json)。

生产射线读取现在最多尝试16次、总读取预算250,000 ns（约为5 ms控制周期的5%），只对sequence未武装、奇数写入中或复制期间前后不一致进行有界重读。达到稳定一致快照后，才读取一次校验用单调时间；版本、magic、11维、零/未来/过期时间戳和非有限数等稳定帧语义错误仍直接拒绝。持续冲突继续走原安全停止路径。修复没有改变射线值、RA/状态机、控制周期或安全门。定向验证通过：生产helper的确定性C++交错检查、`python3 scripts/test_s1_04_contract.py`、`python3 scripts/test_s6_12_ray_repair_resume.py`、受影响Python文件`py_compile`及`colcon build --packages-select rl_quadruped_controller`。构建后的实际安装controller库和MuJoCo主程序已记录SHA-256。实际运行代码阶段为`cd6021faa96b0cc8`，冻结Git基点`8655995ff265ef2a18656c7a0b956ee889c64559`；controller库SHA-256为`c94903e50f5d1d751d4be1f9effb7a10be775d8daee0aec08736b794719327ac`，MuJoCo主程序SHA-256为`8ae02f1c6f20a00f51e7a9bbefa7ef31c718830460179df75120ceccbd4290d7`。完整源码阶段记录在冻结计划的`code_stages`中。

第66项原失败attempt仍保留。独立`attempt_02`使用dev27、B组、E=-0.05、X=-0.09；实际controller来源日志确认本阶段配置、场景、模型路径及目标(7.5,0)。该次`ARRIVED`，终点距目标0.4805 m；cycle 0–291连续，终止cycle 291的同一策略快照完整持久化，共349帧，末次排空完整，约5仿真秒站立观察完成，record `VALID`，清理已确认。原始记录见[第66项修复后attempt](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/runs/066_m1dld01_x09_r1/attempt_02/result/B/rep_01/1f814d3311684f928e3c7e9fc719079a/)。策略频率50.26 Hz，中位周期19.985 ms、P95 20.838 ms；既有阶段记录约为50.15 Hz/19.991 ms/P95 25.829 ms，单次且阶段条件不同，不能据此归因或计算提速，但未见明显频率退化。该run未记录射线瞬时重读事件；它证明修复版本下读取与运行链可工作，但不能证明旧第66项一定由瞬时竞态造成，也没有在实测中复现竞态。原第66项历史原因仍不能唯一归因。

随后依冻结顺序完成第67–82项，共16项新增有效结果，均为`ARRIVED`、record `VALID`，均保存cycle 0及完整终止帧、完成最终排空和站立观察，且每项的诊断进程关闭记录为完成。第66项补做加第67–82项共17个有效新增矩阵结果；没有重试合法任务终局，没有改变参数或运行顺序。

第83项`083_m1dfd01_x05_r1`为dev29、A组，source/config及记录本身`VALID`，周期0–205连续，终止帧完整持久化，206帧完成最终排空。监督器终局为`FALL_TERMINATION`；controller确认PASSIVE，收尾进程退出。原始物理快照记录在sim_time 13.862 s确认`low_height`跌倒事件，至sim_time 13.878 s快照中事件仍未关闭。可是派生分析将`terminal_result`和`navigation_terminal_result`标为`INVALID_TERMINAL_SAFETY_CONFLICT`，`motion_fall_result=UNKNOWN`；碰撞覆盖摘要也显示全捕获区间有一个invalid sample及producer覆盖不完整。故该记录不能当作完整、可解释的跌倒任务结果来继续粗选。按提示词的安全/记录停止规则，保留原始run并停批；没有重跑第83项，也没有启动第84–96项。监督终局、原始跌倒事实、派生UNKNOWN/冲突状态分别保留，未将UNKNOWN改为false。原始证据见[第83项停止记录](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/runs/083_m1dfd01_x05_r1/result/A/rep_01/8fecf8c227d54c199a63b7530e51eab0/)。

本轮累计台账：96项计划；83项至少有一次attempt；总计86次attempt（含之前保留的三次失败attempt）；82项为有效任务结果；第83项为安全分析冲突停止项；13项未启动。全部attempt终局计数为ARRIVED 56、COLLISION_TERMINATION 18、DIAGNOSTIC_SIM_TIME_LIMIT 8、FALL_TERMINATION 1、SYSTEM_SAFETY_ABORT 1、PREPARATION_FAILURE 1、STARTUP_FAILURE 1。82个有效矩阵结果之外的第83项虽原始记录完整且来源验证通过，仍因安全分类冲突从有效任务结果分母中排除，不能把record `VALID`等同于安全指标完整。

本轮自动重生成的逐项结果、attempt台账、部分汇总、执行状态图、终局/切换/Recovery/耗时图、地图×阈值图和代表时间线清单见[冻结批次目录](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/)。图表是混合修复前原65项和修复后新增结果的开发集粗选摘要；新增项有独立code-stage标记，不能将阶段差异归因于阈值。因新增覆盖主要集中在dev27及dev29部分档位，且13项未运行、一个安全分析冲突停止，当前证据为**区分不足**，不推荐任何细选区间。没有使用留出集或启动细选。

最终进程核对未发现本轮MuJoCo、controller/ROS、RViz或诊断/批次脚本残留，见[本轮清理检查](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/postrun_process_check_t1_ray_repair.json)。GLFW隐藏预检窗口已关闭。T1保持ACTIVE / INCOMPLETE，等待Director审阅；不提交Git。

以下内容是此前阶段的历史记录，保留以说明旧的停批状态，不代表当前批次进度。

---

## 历史续跑状态（2026-10-15）

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

## 图表与批次汇总（按2026-10-16停止状态重新生成）

部分结果表按计划项显示最新attempt；完整历史attempt见[attempt台账CSV](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/coarse_attempt_results.csv)。计划项CSV、attempt CSV和[机器可读摘要](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/coarse_partial_summary.json)分别保留VALID结果、安全分析冲突停止项、历史INVALID attempt及未启动分母。最新摘要为83项有attempt、86次attempt、82项有效任务结果、第83项安全分析冲突停止、13项未启动；本段后续2026-10-15文字仅描述当时状态。

当前诊断图只纳入有VALID任务结果的计划项；无效启动/准备失败与安全覆盖UNKNOWN停止项留在台账/执行状态图中，不冒充导航终局：

- [粗选计划执行状态：82有效、1安全分析冲突停止、13未启动](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/execution_status.png)
- [有效记录终局构成](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/terminal_composition.png)
- [短反向切换频率](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/short_reverse_switch_count.png)
- [Recovery占比](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/recovery_share_valid_motion_time.png)
- [到达耗时](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/elapsed_task_duration_s.png)：仅ARRIVED有效样本有到达耗时，样本数见图标题/表格。
- [地图×阈值ARRIVED热图](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/map_threshold_heatmap.png)：只反映已运行且VALID的粗略计数，覆盖未满，不能排名。
- [按预定规则选取的代表时间线清单](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/representative_timelines.json)；原始逐run时间线位于各计划项结果目录中。

图仅展示部分粗选覆盖，不足以比较阈值优劣或估计细选区间。

## 历史结论（2026-10-15停止状态；当前结论见顶部）

- （历史截至2026-10-15）65个有效计划结果覆盖多个阈值，但粗选在第66项停批、30项未启动；当时不足以给出完整矩阵比较或细选区间。当前覆盖和结论以本报告顶部2026-10-16章节为准。
- 第66项`SYSTEM_SAFETY_ABORT`与PASSIVE确认保留；其记录及最终排空完整，但任务终止周期/物理窗口边界缺失，所以碰撞和跌倒结论为UNKNOWN。该缺口触发停批，没有重试。
- 第2项恢复入口、attempt级CSV和部分汇总已针对续跑核对；汇总工具的两个定向缺陷修正后，表、图和代表时间线清单重新生成。生产base-bounds读取标准没有放宽。
- 最终[进程清理核对](../../../../logs/s6_12/b_exit_coarse/20261015_192706_3/postrun_process_check_t1.json)未发现本任务MuJoCo、controller、RViz或诊断runner残留。第2项及之后逐项process facts均显示shutdown/closeout完成；第66项controller PASSIVE转换已确认。GLFW预检窗口已关闭。
- 留出集没有用于运行或选参；未启动细选、训练、正式实验或实机测试；未提交Git。历史原始运行保持不变。

（历史截至2026-10-15：T1保持ACTIVE / INCOMPLETE，阶段2粗选尚未完成；第67–96项是否续跑、如何补足第66项终止边界等待Director审阅。当前状态以本报告顶部为准。）
