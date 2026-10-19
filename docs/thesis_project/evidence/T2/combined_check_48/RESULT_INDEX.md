# T2 48项组合核验结果索引

**状态：T2 COMPLETE（Director有条件通过，2026-10-19）。** 本页索引的48项仍是开发阶段初步对照；通过范围、代表参数及限制见[T2任务](../../../tasks/T2.md)和[最终研究参数记录](../../../designs/T2/HOLD_TIME_SELECTION.md)。

批次：`20261019_1358_01_seed2026101901`。这是六张M2开发图上的初步功能对照，不是正式实验。

最新离线收尾复核与计数修正：[四条D组超时复核](../closeout_review/T2_CLOSEOUT_REVIEW.md)、[摘要更正记录](../closeout_review/REPORT_CORRECTIONS.md)。更正基于逐run表和terminal，不改原始run数据。

- [执行报告](T2_COMBINED_CHECK_48_EXECUTION_REPORT.md)
- [冻结方法](FROZEN_METHOD_48.md)、[48项矩阵](FROZEN_MATRIX_48.csv)
- [逐run结果及48项reset/实测cycle0初态](analysis/per_run_results.csv)
- [24格地图×组别结果](analysis/map_group_24.csv)
- [四组汇总](analysis/group_summary.csv)
- [主要对照及UNKNOWN边界](analysis/factor_contrasts.csv)
- [同地图/重复号共同到达时间明细](analysis/paired_arrival_time_comparisons.csv)
- [C/D逐episode保持审计](analysis/hold_episode_audit.csv)
- [RA风险事件与Recovery决策配对](analysis/risk_to_recovery_events.csv)
- [分析摘要](analysis/analysis_summary.json)
- 图表：[终局](analysis/charts/terminal_outcomes.png)、[切换](analysis/charts/switching_metrics.png)、[Recovery占用及仿真时限](analysis/charts/recovery_occupancy_and_timeouts.png)、[地图×组别终局](analysis/charts/map_group_terminals.png)、[保持episode覆盖](analysis/charts/hold_episode_coverage.png)、[固定样本时间线](analysis/charts/example_timeline_e03_round1.png)
- [批次原始记录与逐项run目录](../../../../../logs/02_T2_minimum_hold/combined_check_48/20261019_1358_01_seed2026101901/)

任务终局、记录有效性、站立观察、controller停止确认和进程关闭分别记录。VALID不等于安全成功；命令写出不表示MuJoCo已消费。留出集未运行。
