# T1 M2第二轮执行报告

## 最新续跑结果（第73项补做及原74–144项完成，2026-10-16）

**状态：RUNS / ANALYSIS COMPLETE，T1保持ACTIVE，提交Director审阅。** 原72条有效记录未重跑；第73项原始INVALID及所有旧失败保留。第73项仅补做一次并有效，之后按冻结顺序完成原第74–144项；没有额外场景重试。实际MuJoCo场景启动累计146次，达到本次授权上限146；另有4条已确认未创建场景的前置/编排失败单独计数。144个计划项现均有有效任务记录、完整辅助评分，未运行项0。

### 第73项覆盖失效排查与补做

原始第73项的首次拒绝样本仍是LIVE（年龄约1.237 ms、sim_time约4.050 s、physics_step 2559），场景/capture/模型身份一致，但`physics_coverage_complete=0`；较晚样本才为STALE（年龄约504.731 ms、sim_time约15.374 s、step 8221）。旧run未记录覆盖从完整转为不完整的首个物理步，不能据此唯一确定其根因，也不能称STALE是唯一故障。

本轮在`physics_coverage_diagnostic.h`与`obstacle_collision_authority.h`中增加同判据诊断helper及首次覆盖丢失日志，保留原步号、仿真时间和身份信息；未放宽连续性、安全或STALE门槛。`test/t1_m2_physics_coverage_diagnostic.cpp`定向C++检查覆盖正常步进、首步/时间异常、跳步/回退、时间差错误和非有限值；`scripts/test_t1_m2_item73_repair.py`覆盖生产恢复校验和入口失败隔离；Python编译及实际MuJoCo主程序增量构建通过。旧`test_t1_m2_preparation_recovery.py`仍固定断言历史停止序列6，与本轮序列73台账不符，出现fixture口径失败；未将其当成生产回归，也未改写该历史测试。第73项attempt_02由生产发布/读取链正常进入策略并取得VALID记录；该run的MuJoCo日志没有覆盖丢失标记，说明此次未重现原故障，不能反证或解释原run的根因。原始attempt_01保持不变。

第73项补做为`ARRIVED`（终距约0.485 m），周期0至终止周期连续，终止快照完整保存，5秒站立观察及运行清理完成。原始记录：[attempt_01](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/runs/073_r2_m2d01_x12/result/B/rep_01/bd6502665c9a40ac832c7d971e19214f/)；[attempt_02有效补做](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/runs/073_r2_m2d01_x12/attempt_02/result/B/rep_01/c157a6e7cb004b5f98d26cc13405ceb7/)。第69项不重跑：原始`normal_shutdown_not_attempted.reason`为`fall state is uncertain or confirmed`；其任务终局ARRIVED，但跌倒状态UNKNOWN，所以正常站立收尾门未准入。不能写成已确认跌倒，也不能声称完成站立收尾。

### 最终矩阵和评分

| 项目 | 最终结果 |
|---|---:|
| 计划项 / 有效任务记录 | 144 / 144 |
| 未运行项 / UNKNOWN评分项 | 0 / 0 |
| 实际场景启动 | 146 / 授权上限146 |
| 确认无场景前置/编排失败 | 4（1条计划attempt未确认场景启动＋3条独立wrapper失败，不占场景额度） |
| 总attempt账目 | 150（147条计划attempt记录＋3条独立wrapper失败） |
| 有效终局 | ARRIVED 98；COLLISION_TERMINATION 26；DIAGNOSTIC_SIM_TIME_LIMIT 20 |
| 任务窗碰撞判定 | True 26；False 118；UNKNOWN 0（按各run声明的覆盖及分析规则） |
| 跌倒判定 | True 0；False 143；UNKNOWN 1 |
| 策略周期缺口 / 无效策略时钟帧 | 0 / 0（最终144条表的字段值） |
| 站立观察字段 | COMPLETE 117；NOT_REACHED_OR_UNKNOWN 27 |

按冻结权重和归一化常数生成144条逐run评分、48个地图×阈值单元、8档汇总及图表。各X的等权地图辅助分为：−0.06 65.320（95%描述区间44.735–85.009），−0.07 66.412（41.885–85.508），−0.08 62.495（37.649–84.349），−0.09 62.884（40.829–84.339），−0.10 66.414（54.185–80.096），−0.11 68.168（46.071–89.226），−0.12 71.142（50.616–90.881），−0.13 61.367（36.344–84.702）。这些是开发地图上的描述性辅助统计，不据此选定最优参数或推断正式成功率；M2没有A参考，不计算B−A。跌倒UNKNOWN、接触/覆盖边界和收尾状态仍独立报告，评分不替代安全判断。

完整产物见[结果索引](RESULT_INDEX.md)、[最终运行与分析摘要](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/final_completion_summary.json)、[最终机器可读评分摘要](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/scoring_summary.json)、[逐run分数](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/run_scores_144.csv)、[48格地图×阈值表](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/map_threshold_summary_48.csv)及[8档表](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/threshold_summary_8.csv)。图表：[终局构成](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/terminal_composition.png)、[评分热图](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/map_threshold_score_heatmap.png)、[分项评分](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/score_components.png)、[到达时间](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/arrival_time.png)、[短反向频率](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/short_reverse_frequency.png)、[Recovery占比](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/recovery_share.png)。批次日志、台账和最终清理记录仍保留于`logs/t1/m2_exit_round2/20261016_174309_9/`；最终清理清点见[本轮进程检查](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/final_item73_completion_process_check.json)，匹配进程为空，预检窗口已关闭。批次约1.76 GiB，可用磁盘约56.5 GiB。

该批仍是非正式M2开发参数预实验，不使用留出集、不训练、不实机；无论逐项结果或辅助分数如何，都不能据此形成论文性能结论。历史失败、原始记录和旧评分/图表均未覆盖。本轮相关进程已核对退出；没有提交Git。

以下为本轮前的历史快照，内容保留作执行历史；其“第73项后停止”等状态不代表当前批次状态。

## 最新续跑结果（2026-10-16，第73项启动后停止）

**状态：STOPPED / INCOMPLETE，T1保持ACTIVE，提交Director审阅。** 本轮按真实恢复入口先完成定向离线联测，再从原第6项继续；原前5条有效结果没有重跑。联测覆盖恢复授权、参数/运行顺序、目录分配、尝试账本、场景额度及真实main入口派发；`py_compile`和针对性检查通过。相同执行环境的GLFW预检通过，Intel/Mesa RPL-P、OpenGL 4.6，探针窗口已关闭。

本轮从原第6项连续运行至第73项，共启动68次MuJoCo场景。第6–72项共67项有效，含原前5项后，矩阵有72项有效任务结果：47次ARRIVED、19次COLLISION_TERMINATION、6次DIAGNOSTIC_SIM_TIME_LIMIT。第73项实际启动了MuJoCo（计划配置B、X=−0.12，run_id `bd6502665c9a40ac832c7d971e19214f`），但在RL前因物理authority为STALE发生`PREPARATION_FAILURE / INVALID`；策略帧数为0，PASSIVE请求与controller确认、shutdown和closeout均有记录。依失败停止规则立即停止，未重试第73项，也未启动第74–144项。该失败属于实际场景启动，不是“启动前脚本失败”。

场景额度与前置编排失败分开计数：当前累计确认MuJoCo场景启动74次（原6次＋本轮68次），授权上限145；未知启动状态0；确认未创建场景的编排失败4条（其中一条保留为计划attempt，另三条为独立wrapper失败），不占场景额度。动态attempt账本有75条计划attempt记录；重复的`engineering_attempt_history`和`engineering_retry_history`只作事件摘要，不另计。144项中72项有效、1项已尝试但INVALID、71项未启动。历史147/148总调用额度仍保留为旧政策记录，本次授权改用145次实际场景启动上限。

按既定评分器生成的部分结果包含72条完整评分、72条UNKNOWN（第73项准备失败及71项未启动，不补零）。有效终局仅是本批描述数据；每个地图×阈值三次重复尚未齐全，没有可用的阈值完整分或B−A对照，不据此排名或推荐参数。第69项虽为ARRIVED，但站立观察未完成/未知；该任务终局与收尾结果分开保留。第73项记录了准备失败及清理完成，不进入任务评分。

本轮修改仅涉及批次runner/定向检查中的额度及失败分类、恢复入口交接；未修改控制器、模型、场景、参数或安全门。产物：逐项结果与attempt表、部分摘要、恢复批次计划、部分评分表和图表见下列结果索引；关键原始证据为[第73项run](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/runs/073_r2_m2d01_x12/result/B/rep_01/bd6502665c9a40ac832c7d971e19214f/)、[恢复计划台账](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/m2_round2_plan.json)、[部分摘要](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/m2_partial_summary.json)、[评分目录](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/weighted_score_scene_budget_stop/)、[同环境GLFW预检](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/glfw_preflight_scene_budget_resume.json)和[最终进程清点](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/final_scene_budget_stop_process_check.json)。最终相关进程清单为空；第73项`shutdown_complete=true`、`closeout_complete=true`、controller退出码0；探针窗口已关闭。没有提交Git。

### 最新部分汇总

| 项目 | 结果 |
|---|---:|
| 计划项 | 144 |
| 有效任务结果 | 72（含原先5项，本轮新增67项） |
| 有效终局 | ARRIVED 47；COLLISION_TERMINATION 19；DIAGNOSTIC_SIM_TIME_LIMIT 6 |
| 实际MuJoCo场景启动 | 74（授权上限145） |
| 启动后INVALID | 第73项1次：pre-RL物理authority STALE |
| 未启动计划项 | 第74–144项，共71项 |
| 未评分/UNKNOWN | 72项（第73项失败1＋未启动71） |
| 确认无场景编排失败 | 4条，单独计数，不占场景额度 |

下文较早的“最新续跑”段落均为本轮之前的历史快照；原始记录及历史失败保留，不改写。

## 历史续跑快照：入口拒绝且场景未启动（当前已于第73项停止）

**状态：STOPPED / INCOMPLETE，T1保持ACTIVE，等待Director审阅。** 按最终续跑授权修复计数守卫后，对原台账做了定向离线核验：计划项attempt记录7条、实际场景启动6次、无场景计划attempt 1条、不同的无场景wrapper失败2条；总计9次，5项有效，待执行139项。重复的`engineering_attempt_history`和`engineering_retry_history`只是已有事件摘要，没有重复计数。原`attempt_limits.attempts_total_max=147`保持不变；计划台账另记用户批准的有效上限148及范围“原第6–144项各一次”。

同环境GLFW预检通过（Intel/Mesa RPL-P，OpenGL 4.6），预检窗口已关闭，启动前项目进程清单为空。实际续跑入口在调用第6项前因dispatcher没有把显式恢复授权传给`run_one()`，将保留的工程失败项误拒绝为非新项目；**没有启动子进程或场景**，不存在新的run_id或PID。依停止规则立即结束本批，没有重试。此次wrapper失败保留后，总计10次attempt：6次场景启动、4条无场景失败、5个有效计划项；有效额度上限148下只余138次，而139个计划项仍未完成，因此本授权范围不足以续完。

随后只修正dispatcher参数传递和行失败标记，并完成Python编译及定向离线检查；测试证明第6项现在能通过恢复身份门并进入运行时冻结核验，但**修复后入口未再次调用，未运行任何新场景**。剩余139项、全部新参数对照指标和最终图表均未完成；目前只保留原5项的既有部分结果，不把它们当成完整矩阵分析。第6项原目录分配失败、旧计数恢复失败和本次授权传递失败都作为无场景工程记录保留。后续若要继续，需Director/用户审阅目前10次账目与148上限之间的缺口并另行授权；本执行器不自动续跑。

证据：批次计划和动态台账见[原批次目录](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/m2_round2_plan.json)及[最新部分摘要](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/m2_partial_summary.json)；本次GLFW结果见[同环境预检](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/glfw_preflight_final_139.json)，进程清点见[停止后清点](../../../../../logs/t1/m2_exit_round2/20261016_174309_9/final_139_stop_process_check.json)。首次5项执行状态、原始失败及部分评分见下方历史记录，均未改写。

## 历史续跑快照：第5项有效后、第6项计数修正前（2026-10-16）

**批次 STOPPED / INCOMPLETE，T1 保持 ACTIVE，提交 Director 审阅。** 原第5项 attempt_02 独立补做有效：m2dev06、B、X=-0.11，`VALID / ARRIVED`，距目标0.4848 m，任务耗时6.360 s，路径7.616 m，约50 Hz，短反向28次；周期记录完整、站立观察完成，`shutdown_complete=true`、`closeout_complete=true`。原attempt_01的`PREPARATION_FAILURE / INVALID`保持原样。

第5项原失败的具体拒绝字段仍不能从旧记录唯一确定：旧准备日志把无效时钟、覆盖不全和未知接触合并；后段姿态恢复正常、四足接地、起立门槛通过且低高度episode关闭，只能证明后来姿态正常，不能反推此前某个具体安全门字段。诊断器现已在不改准入条件的前提下记录首个/最后失败样本的物理快照身份、年龄、sim_time/step/sequence、覆盖、未知接触、接触、姿态和tracker字段。attempt_02通过不证明原失败是程序错误，也没有复现其拒绝原因。

第6项没有启动场景：一次已记录的启动目录分配缺陷（`FileExistsError`）发生在子进程启动前，已修正为首次目录只创建一次、重试隔离到`attempt_02`；其后一次恢复入口在场景启动前因台账断言仍按7次预期、实际为8次而停止；随后将断言修正为8并通过离线检查，但未再次调用批次入口。GLFW预检在同一获准环境通过，但没有为第6项启动仿真。两次无场景失败均保留，未启动139个场景计划项。原额度147次中已记9次（含原5个场景attempt、原wrapper失败、第5项补做、第6项目录工程失败及恢复入口失败），剩138次额度，少于剩余139个场景项；按原总额度无法完整执行剩余矩阵，因此停止，不再重试。第6项有工程恢复状态，另138项未开始。

离线定向检查通过：生产站立门对稳定样本通过，并分别对非有限仿真时间、覆盖不完整、未知接触和非LIVE状态保留拒绝；启动目录首次分配/attempt_02隔离测试通过；生产配置、八份overlay和冻结的controller库、MuJoCo、模型、地图输入核对通过；Python编译及站立保持针对性检查通过。未修改地图、模型、controller行为、研究参数或安全门。第5项的5条有效运行当前使用原固定评分完整计分，139条未运行/工程未执行项为UNKNOWN，不补零；这不是阈值比较，样本不足以排序或推荐参数。派生表图位于`logs/t1/m2_exit_round2/20261016_174309_9/weighted_score_recovery_partial/`；系统Python绘图因Matplotlib与NumPy ABI不兼容失败，使用原批次指定的Anaconda Python重新生成，不改原始记录。

本轮最后进程扫描无本次相关进程残留；GLFW探针窗口已关闭；第5项process facts确认关闭完成。当前不继续矩阵；恢复剩余项目需Director对额度/停止点另行审阅。正文后续原始执行记录描述首次停在第5项的历史快照，不覆盖本节最新状态。

---

## 首次执行报告快照（历史：首次停止于第5项）

**状态：STOPPED / INCOMPLETE，提交Director审阅。** 144项矩阵已经冻结，但仅运行5项；出现一次未能通过站立准备的`PREPARATION_FAILURE`后，按失败停止规则结束，139项未运行。本报告不是144项完成结果，也不支持退出阈值排序。

## 做了什么

读取当前权威文档、T1任务、M2目录、运行器/分析器和旧评分定义；确认T1为唯一ACTIVE。固定6张M2开发候选、八档B退出阈值、每图每档3次的144项顺序。新增独立批次编排和评分工具，复用既有`manual_ab_compare.py`及生产分析链，没有修改controller、模型、场景、切换逻辑、安全判据或运行参数。独立overlay通过生产配置解析，数值比较兼容`-0.10`/`-0.1`，B配置不含保持字段。调度、评分、UNKNOWN处理及截图额度的定向离线检查通过，Python编译通过。

运行前冻结了计划、模型/控制器/MuJoCo构建与源码身份、M2布局/场景身份、overlay配置、评分常量和调度。运行前存储投影约2.0 GiB，低于4 GiB预算；可用空间约58.6 GiB，高于15 GiB保护线。

GLFW预检与运行在同一沙箱外执行上下文，显示与系统库变量临时设置为：`DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority`、`MUJOCO_GL=glfw`、系统`/usr/bin/python3`及既有本地`LD_LIBRARY_PATH`。隐藏窗口和OpenGL context成功；renderer为Intel / Mesa Intel(R) Graphics (RPL-P)，OpenGL 4.6。没有修改系统授权、驱动或全局环境。

首次批次启动在任何场景前因阶段快照的`created_at`每次变化而退出。识别为非语义时间字段比较问题后，仅从稳定快照比较中排除该时间戳，保留阶段ID、源代码哈希、安装库与MuJoCo二进制哈希；重做定向离线检查后继续。原失败作为一条无场景启动的编排attempt保留，不覆盖或计入计划项。

## 实际运行

| 序号 | 地图 | 组/X | Supervisor终局 | 数据/关键指标 | 收尾 |
|---:|---|---|---|---|---|
| 1 | m2dev06.xml / m2d06 | B / −0.13 | ARRIVED | 6.200 s；末距0.491 m；路径7.651 m；50.21 Hz；20次模式边沿；Recovery 0.954 s，占有效任务时间15.4%；18次短反向；任务RA范围[−0.9995, 0.0932]；任务碰撞False、跌倒False，覆盖完整 | 5 s站立观察完成；关闭确认 |
| 2 | m2dev06.xml / m2d06 | B / −0.08 | ARRIVED | 6.700 s；末距0.485 m；路径7.575 m；50.19 Hz；68次边沿；Recovery 0.922 s/13.8%；66次短反向；任务RA[−0.9995, 0.1894]；碰撞False、跌倒False，覆盖完整 | 完成；关闭确认 |
| 3 | m2dev06.xml / m2d06 | B / −0.09 | ARRIVED | 7.820 s；末距0.481 m；路径7.910 m；50.15 Hz；90次边沿；Recovery 1.468 s/18.8%；88次短反向；任务RA[−0.9991, 0.1406]；碰撞False、跌倒False，覆盖完整 | 完成；关闭确认 |
| 4 | m2dev06.xml / m2d06 | B / −0.06 | ARRIVED | 6.398 s；末距0.499 m；路径7.625 m；50.22 Hz；46次边沿；Recovery 0.906 s/14.2%；44次短反向；任务RA[−0.9994, 0.1027]；碰撞False、跌倒False，覆盖完整 | 完成；关闭确认 |
| 5 | m2dev06.xml / m2d06 | B / −0.11 | PREPARATION_FAILURE | 固定站立准备在墙钟上限前未确认稳定，未进入RL、无策略帧；记录INVALID，任务指标UNKNOWN，不作为导航失败或合法导航结果 | 诊断器请求command 1；controller转PASSIVE已确认；清理完成 |

计划随机顺序首项地图为m2dev06，因此四个有效导航观察只覆盖该地图的四档各一次。不能从这些单次记录判断阈值效果。第五项没有策略周期，不能评价其RA切换。

## 计数与评分

- 计划144；MuJoCo/controller run 5；其中4条`VALID / ARRIVED`、1条`PREPARATION_FAILURE / INVALID`；139项未运行。
- 另有1次无场景启动的wrapper快照检查失败并修复。按批次执行attempt口径为6次（5场景attempt + 1前置编排attempt），未使用计划项重试。实际运行已停止，不续跑。
- 4条有效记录均完整评分，个人辅助总分：89.768（−0.13）、86.337（−0.08）、84.943（−0.09）、87.447（−0.06）。每个X只一条、同一张图，不能据此排名或推荐候选。
- 其余140条评分UNKNOWN：139未运行，1条准备失败无任务数据。没有M2 A参考，不作伪B−A对照。阈值综合分要求每个地图×阈值单元三次评分完整；当前没有完整单元，因此没有阈值总分。地图簇bootstrap使用固定种子2026101603及10,000次流程，但每个已有阈值只含一个地图簇；综合分区间为空，ARRIVED/碰撞比例区间为单簇退化值，不具跨地图信息。缺数据不补零。
- 四条有效run没有周期缺口或无效策略时钟（0 gaps/0 invalid frames），短反向统计覆盖complete；约50.15–50.22 Hz。第五项没有策略帧。短反向次数、Recovery、碰撞/跌倒字段均保持各自任务窗口边界，站立尾段不计入运动指标。命令输出/PASSIVE不等于MuJoCo命令消费确认。

## 产物与证据

矩阵、评分和原始数据位置见[结果索引](RESULT_INDEX.md)与[冻结方法](FROZEN_MATRIX_AND_METHOD.md)。批次原始记录在`logs/t1/m2_exit_round2/20261016_174309_9/`；运行表含144计划项状态和指标，attempt表保留5个场景启动。截图策略按计划预选了首轮首图八个阈值；本次已保存8张PPM（4个有效run各live/standing一张），低于16张上限；没有录像。批次最终约285 MiB，可用磁盘约59 GiB。

## 停止原因、限制与下一步

第5项无法在限定准备时窗内确认站稳，且发生真实`PREPARATION_FAILURE`及PASSIVE收尾。该任务指定了失败即停止后续组，因此没有重试第5项，也没有启动剩余139项。要继续原矩阵需要Director审阅这次准备失败并另行授权；不能自行延长站立等待、放宽站稳门槛或恢复续跑。

当前四项均从同一图开始、各X仅一次，且矩阵未完成；不能得出成功率、性能排序、最优X或跨地图泛化结论。M2六图仍是开发候选，策略表现只在m2dev06上有四条此次运行证据；M1留出集未使用。保留跌倒/碰撞覆盖的解释边界、命令消费未确认等既有限制。

## 环境收尾

五个场景run的process facts均报告`shutdown_complete=true`和`closeout_complete=true`；有效ARRIVED项完成5秒站立观察，第5项PASSIVE转换确认。任务结束后的定向进程清单未发现本轮MuJoCo、controller/ROS launch、RViz、采集器、停止发布器或辅助进程残留。GLFW隐藏窗口已关闭。没有影响其他用户进程，没有删除共享内存对象，没有提交Git。
