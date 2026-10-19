> **2026-10-19口径更新**：原碰撞差值“最不利”标签已纠正。H0.20保守差值−16.7pp、有利差值−19.4pp，候选不变；旧表图保留，见[独立重算说明](unknown_sensitivity_review_20261019/README.md)。

# T2 阶段3 180项A/C保持时间扫描结果索引

- 批次：`logs/t2/hold_scan_180/20261018_192507_t2scan180/`
- 执行报告：[T2_HOLD_SCAN_180_EXECUTION_REPORT.md](T2_HOLD_SCAN_180_EXECUTION_REPORT.md)
- 运行前冻结方法：[FROZEN_METHOD_180.md](FROZEN_METHOD_180.md)
- H=0.20边界检查：[H020_BOUNDARY_CHECK.txt](H020_BOUNDARY_CHECK.txt)
- 冻结计划/调度/配置/attempt台账：批次目录 `t2_hold_scan_180_plan.json`、`attempt_ledger.csv`、`progress.json`
- 逐run：[per_run_results.csv](per_run_results.csv)
- 18图×5档90格：[map_level_90.csv](map_level_90.csv)
- 五档总体：[level_summary_5.csv](level_summary_5.csv)
- 三层×五档：[layer_summary_15.csv](layer_summary_15.csv)
- 相对A差值（bootstrap）：[level_deltas_vs_A.csv](level_deltas_vs_A.csv)
- 逐图差值：[map_deltas_vs_A.csv](map_deltas_vs_A.csv)
- 选择性配对到达时间：[paired_arrival_time_diff.csv](paired_arrival_time_diff.csv)
- 碰撞上下文：[collision_context.csv](collision_context.csv)
- 保持episode：[hold_episodes.csv](hold_episodes.csv)
- 保持episode汇总：[hold_audit_summary.csv](hold_audit_summary.csv)
- Recovery驻留：[recovery_residence.csv](recovery_residence.csv)
- 候选H：[candidate_H.json](candidate_H.json)
- 自动汇总：[RESULTS_SUMMARY.md](RESULTS_SUMMARY.md)
- 图表：[charts/arrival_collision_vs_level.png](charts/arrival_collision_vs_level.png)、[charts/switching_recovery_vs_level.png](charts/switching_recovery_vs_level.png)、[charts/timeouts_vs_level.png](charts/timeouts_vs_level.png)、[charts/hold_residence.png](charts/hold_residence.png)、[charts/heatmap_arrival.png](charts/heatmap_arrival.png)、[charts/heatmap_collision.png](charts/heatmap_collision.png)

## 结果计数

| 项目 | 值 |
|---|---:|
| 计划项 | 180 |
| 有效任务结果 | 180 |
| ARRIVED | 142 |
| COLLISION_TERMINATION | 18 |
| DIAGNOSTIC_SIM_TIME_LIMIT | 19 |
| FALL_TERMINATION | 1 |
| WALL_CLOCK_GUARD_INCOMPLETE | 0 |
| 实际场景启动 | 181 |
| 工程补做（场景） | 1（第34项射线安全中止后一次补做；原记录保留） |
| 纯编排错误 | 0 |
| 未启动项 | 0 |

## 候选H

- 推荐代表值：**C H=0.20仿真秒**
- 合格档：仅H=0.20
- H=0.10：1次确认跌倒，不自动推荐
- H=0.30：1次跌倒UNKNOWN（d08/repeat1，candidate_active_at_terminal_boundary），证据不足，不自动推荐
- H=0.50：到达率61.1%（A 77.8%，−16.7pp），不满足任务容忍度
- H=0.20主要代价：Recovery占比22.6%（A 18.9%）、驻留中位数0.218s（A 0.020s）、仿真时限3次（A 0）、m06两重复均未到达、middle层到达−8.3pp、到达时间8.63s（A 7.40s）

## 关键限制

每图每档仅2次，是开发集预实验；bootstrap区间宽，不证明统计非劣效或全局最优。UNKNOWN不填False/0；base_link界内不等于整机足迹；碰撞邻接不作因果。站立观察159项完成、20项不适用、1项INCOMPLETE（d07/H0.20/repeat1，第34项补做）。批次约1.8GB，可用磁盘约51GB，最终相关进程扫描为空。T2保持ACTIVE，未启动48项组合、留出集、正式实验、训练或实机，不提交Git。
