# 最新状态：144项矩阵完成，等待Director审阅（2026-10-16）

第73项attempt_02一次有效补做后，原第74–144项按原序各运行一次。实际MuJoCo启动146/146；确认的无场景失败4条另计（147条计划attempt记录中1条未确认场景启动，另有3条独立wrapper失败；总attempt账目150）。144计划项均有有效记录和完整评分（ARRIVED 98、碰撞终止26、仿真限时20），未运行项及UNKNOWN评分项均为0。原第73项INVALID和其余所有历史失败原样保留。

- [最新执行报告](T1_M2_ROUND2_EXECUTION_REPORT.md)
- [冻结条件与执行后修订附记](FROZEN_MATRIX_AND_METHOD.md)
- [最终144行结果表](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/m2_run_results_144.csv)
- [attempt台账](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/m2_attempt_results.csv)、[最终运行与分析摘要](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/final_completion_summary.json)、[运行器最终局部摘要（原件保留）](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/m2_partial_summary.json)
- [第73项原始失败](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/runs/073_r2_m2d01_x12/result/B/rep_01/bd6502665c9a40ac832c7d971e19214f/)、[第73项唯一补做](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/runs/073_r2_m2d01_x12/attempt_02/result/B/rep_01/c157a6e7cb004b5f98d26cc13405ceb7/)
- 最终评分目录：[144行逐项评分](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/run_scores_144.csv)、[48格地图×阈值表](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/map_threshold_summary_48.csv)、[8档汇总](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/threshold_summary_8.csv)、[评分计数](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/scoring_summary.json)
- 图表：[终局构成](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/terminal_composition.png)、[评分热图](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/map_threshold_score_heatmap.png)、[分项评分](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/score_components.png)、[到达时间](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/arrival_time.png)、[短反向频率](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/short_reverse_frequency.png)、[Recovery占比](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_item73_repair_final/recovery_share.png)
- [最终进程清点](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/final_item73_completion_process_check.json)

评分沿用冻结口径，仅描述开发批次。没有M2 A对照，不用于参数最优结论；跌倒UNKNOWN、物理/策略安全证据边界和收尾字段须单独解释。过程环境已关闭并核对无本次相关进程。

以下历史索引保留；其状态段落对应当时停止点，不代表当前结果。

# 历史状态快照（2026-10-16）

批次 `STOPPED / INCOMPLETE`，T1保持ACTIVE等待Director审阅。真实恢复入口的离线联测通过后，从计划第6项连续运行至第73项；前5条有效结果未重跑。新增68次实际场景启动，其中第6–72项67项有效；第73项启动MuJoCo后，在RL前因物理authority `STALE`发生`PREPARATION_FAILURE / INVALID`，随后PASSIVE、shutdown和closeout确认。按停止规则未重试第73项，计划第74–144项71项未启动。

- [最新执行报告](T1_M2_ROUND2_EXECUTION_REPORT.md)
- [动态计划/额度台账](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/m2_round2_plan.json)：144项中72项有效、1项启动后INVALID、71项未启动；累计实际场景启动74/145，未知0；4条确认无场景编排失败另计，不消耗场景额度。
- [运行结果表](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/m2_run_results_144.csv)、[attempt表](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/m2_attempt_results.csv)、[部分摘要](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/m2_partial_summary.json)
- [第73项原始run](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/runs/073_r2_m2d01_x12/result/B/rep_01/bd6502665c9a40ac832c7d971e19214f/)
- 部分评分产物：[144行逐项评分](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_scene_budget_stop/run_scores_144.csv)、[48行地图×阈值表](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_scene_budget_stop/map_threshold_summary_48.csv)、[8档部分汇总](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_scene_budget_stop/threshold_summary_8.csv)、[评分计数](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_scene_budget_stop/scoring_summary.json)。72条有分、72条UNKNOWN；没有完整重复单元，不构成阈值排名或选择。
- 部分图表：[终局构成](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_scene_budget_stop/terminal_composition.png)、[评分热图](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_scene_budget_stop/map_threshold_score_heatmap.png)、[分项评分](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_scene_budget_stop/score_components.png)、[到达时间](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_scene_budget_stop/arrival_time.png)、[短反向频率](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_scene_budget_stop/short_reverse_frequency.png)、[Recovery占比](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_scene_budget_stop/recovery_share.png)
- [同环境GLFW预检](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/glfw_preflight_scene_budget_resume.json)、[停止后进程清点](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/final_scene_budget_stop_process_check.json)

最终相关进程清单为空。第73项`shutdown_complete=true`、`closeout_complete=true`；GLFW探针窗口关闭。下方是此前各阶段的历史结果索引，原始失败与旧表图均保留。

# 最新续跑尝试停止（2026-10-16）

本次同环境GLFW预检通过，但实际批次入口在第6项启动前因dispatcher遗漏恢复授权参数而拒绝了已有工程失败项；没有启动场景、子进程或生成run_id。按失败停止规则没有重试。经保存本次独立wrapper失败后，台账为10次总attempt：6次实际场景启动、4条无场景失败、5项有效、139项未完成。原147上限保留，用户授权的148有效上限也保留；目前只剩138次额度，无法覆盖139项，因此等待Director/用户审阅，不再续跑。

修正后的dispatcher只通过了编译和定向离线门检查，未再次调用。原5项部分结果保留，不代表完整矩阵汇总。见[最新执行报告](T1_M2_ROUND2_EXECUTION_REPORT.md)、[台账](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/m2_round2_plan.json)、[最终进程清点](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/final_139_stop_process_check.json)。

## 历史快照：第5项补做后、最终139项续跑前

批次仍为 **STOPPED / INCOMPLETE**。原第5项 attempt_02 为 `VALID / ARRIVED`；attempt_01 原始 `PREPARATION_FAILURE / INVALID` 保留。第6项因启动器和恢复台账问题发生两次无场景失败，未进入仿真；第6–144项共139项场景运行未完成，其中第6项留有工程恢复记录、其余138项未开始。

- [最新执行报告](T1_M2_ROUND2_EXECUTION_REPORT.md)
- [更新后的144行计划结果](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/m2_run_results_144.csv)
- [attempt台账](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/m2_attempt_results.csv)：计划attempt行；单独的wrapper失败请以计划JSON中的`pre_scene_start_attempts`和`resume_preflight_failures`核对，不能只靠CSV行数代表总attempt。
- [第5项补做原始run](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/runs/005_r1_m2d06_x11/attempt_02/result/B/rep_01/a28a54aa81694e88b59d9be9d6a7ffab/)
- [第5项补做部分评分与图表](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_recovery_partial/)，5条有效分数（X=-0.11新补做分数88.579）、139条UNKNOWN；不用于参数排序。包括[终局构成图](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_recovery_partial/terminal_composition.png)、[评分热图](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_recovery_partial/map_threshold_score_heatmap.png)、[分项图](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_recovery_partial/score_components.png)、[到达时间图](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_recovery_partial/arrival_time.png)、[短反向频率图](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_recovery_partial/short_reverse_frequency.png)及[Recovery占比图](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_recovery_partial/recovery_share.png)。
- [第6项目录失败及进程清点记录](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/engineering_failure_process_inventory.json)与[恢复失败后进程清点](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/post_resume_failure_process_check.json)
- [同环境GLFW补预检](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/glfw_preflight_engineering_retry.json)、[最终进程清点](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/final_process_check.json)

原始第1–5项历史目录、此前部分评分目录和本索引下文保留；新结果没有覆盖旧失败数据。第5项原准备失败的具体字段根因仍UNKNOWN。原额度147中已计9、剩138，少于139个剩余场景项；不再自动续跑。

---

---

# 历史部分结果索引（首次执行快照；最新状态见上文）

批次状态：`STOPPED`，不是144项完成。计划144；实际五个MuJoCo/controller运行，其中四条`VALID / ARRIVED`、一条`PREPARATION_FAILURE / INVALID`；另有一次运行前的wrapper冻结时间戳失败（没有场景启动）。因此合计6次批次/启动尝试，139个计划项未运行。按停止规则未重试。

## 本批输出

- [执行报告](T1_M2_ROUND2_EXECUTION_REPORT.md)
- [运行前冻结矩阵与方法](FROZEN_MATRIX_AND_METHOD.md)
- [完整计划、来源冻结和逐run状态](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/m2_round2_plan.json)
- [144行运行结果表](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/m2_run_results_144.csv)
- [5条场景attempt表](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/m2_attempt_results.csv)
- [4条完整评分及140条UNKNOWN表](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_partial/run_scores_144.csv)
- [48行地图×阈值表](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_partial/map_threshold_summary_48.csv)
- [8档部分汇总](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_partial/threshold_summary_8.csv)
- [地图聚类bootstrap表](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_partial/map_cluster_bootstrap_ci.csv)
- [汇总评分计数](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_partial/scoring_summary.json)
- [五条已启动run的timeline索引](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/representative_timelines.csv)（各run目录内保留原始timeline.png）
- [终局构成图](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_partial/terminal_composition.png)、[评分热图](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_partial/map_threshold_score_heatmap.png)、[分项图](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_partial/score_components.png)、[到达时间图](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_partial/arrival_time.png)、[短反向频率图](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_partial/short_reverse_frequency.png)、[Recovery占比图](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/weighted_score_partial/recovery_share.png)
- [GLFW预检](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/glfw_preflight.json)、[运行前进程检查](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/preflight_process_inventory.json)、[运行后进程检查](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/postrun_process_check.json)、[存储记录](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/storage_summary.json)

原始run目录按顺序：

1. `runs/001_r1_m2d06_x13/result/B/rep_01/f0659a4b363340939f540370bf6307ff/`
2. `runs/002_r1_m2d06_x08/result/B/rep_01/c0e7bd8bb5994c9eb4ffd1efb9c79781/`
3. `runs/003_r1_m2d06_x09/result/B/rep_01/0c5748cb8ec34de7b335d9dcab47b486/`
4. `runs/004_r1_m2d06_x06/result/B/rep_01/ef9a62612b7549829a0e273e7c6069aa/`
5. `runs/005_r1_m2d06_x11/result/B/rep_01/f6217b89cba24b06beaa2e4ed799fe93/`（准备失败，Invalid）

每个已启动run的相应目录内有原始`runtime_record.jsonl`（若已进入RL）、`run_context.json`、`terminal.json`、`process_facts.json`、指标、时间线及控制台/ROS/MuJoCo日志；原始记录未更改。失败项没有策略帧，不能进入阈值导航比较。

## 局部观察，不作候选排序

四个可评分run都来自m2dev06、每档仅一次，均ARRIVED且任务安全覆盖字段为碰撞False、跌倒False；到达时间6.200、6.700、7.820、6.398仿真秒。逐run分数依次为89.768（X=-0.13）、86.337（-0.08）、84.943（-0.09）、87.447（-0.06）。这些是单地图、单次的描述值，不构成四档排序、评分区间或参数建议。

其余140项评分UNKNOWN：139项未运行，另1项准备失败无任务指标。Bootstrap中仅四个阈值各有一个地图簇，其他阈值无已运行地图，不能提供有意义的跨地图不确定性结论。M2没有A参考，本批不计算B−A差。
