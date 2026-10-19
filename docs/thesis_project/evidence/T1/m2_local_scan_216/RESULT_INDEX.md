# T1 新版M2六档退出阈值216项局部扫描结果索引

**审阅后状态：T1 COMPLETE（Director有条件通过，2026-10-18）。** 用户同意选择E=−0.05、X=−0.10为后续B/D研究代表参数；候选段[−0.11,−0.10]仅指已测离散档。代表值的代价与限制见[参数记录](../../../designs/T1/EXIT_THRESHOLD_SELECTION.md)。本轮不修改部署配置，当前没有已授权Active Task，不自动启动T2、留出验证或其他扫描。

非足端碰撞UNKNOWN继续保留；−0.12的地图等权已知样本口径为11.1%、最不利情景13.9%，碰撞终止终局5/36，三者须分开解释。原始/派生数据未在本次文档同步中改写。

- 批次：`logs/t1/m2_local_scan_216/20261018_162629_m2scan216/`
- 执行报告：[T1_M2_LOCAL_SCAN_216_EXECUTION_REPORT.md](T1_M2_LOCAL_SCAN_216_EXECUTION_REPORT.md)
- 运行前冻结方法：[FROZEN_METHOD_216.md](FROZEN_METHOD_216.md)
- 冻结计划、参数、调度、初态、attempt 台账：批次目录 `m2_local_scan_216_plan.json`、`attempt_ledger.csv`、`progress.json`
- 逐 run：[per_run_results.csv](per_run_results.csv)
- 108格地图×阈值：[map_threshold_108.csv](map_threshold_108.csv)
- 六档总体：[threshold_summary_6.csv](threshold_summary_6.csv)
- 分层：[layer_summary_18.csv](layer_summary_18.csv)
- 与参考差值（bootstrap）：[threshold_deltas_vs_ref.csv](threshold_deltas_vs_ref.csv)
- 逐图差值：[map_deltas_vs_ref.csv](map_deltas_vs_ref.csv)
- 选择性配对到达时间：[paired_arrival_time_diff.csv](paired_arrival_time_diff.csv)
- 碰撞上下文：[collision_context.csv](collision_context.csv)
- Recovery驻留：[recovery_residence.csv](recovery_residence.csv)
- 契约/显示核验：[contract_and_display_check.csv](contract_and_display_check.csv)
- 候选段：[candidate_segments.json](candidate_segments.json)
- 自动汇总：[RESULTS_SUMMARY.md](RESULTS_SUMMARY.md)
- 图表：[charts/arrival_collision_vs_X.png](charts/arrival_collision_vs_X.png)、[charts/switching_recovery_vs_X.png](charts/switching_recovery_vs_X.png)、[charts/timeouts_vs_X.png](charts/timeouts_vs_X.png)、[charts/heatmap_arrival.png](charts/heatmap_arrival.png)、[charts/heatmap_collision.png](charts/heatmap_collision.png)

## 结果计数

| 项目 | 值 |
|---|---:|
| 计划项 | 216 |
| 有效任务结果 | 216 |
| ARRIVED | 172 |
| COLLISION_TERMINATION | 33 |
| DIAGNOSTIC_SIM_TIME_LIMIT | 8 |
| FALL_TERMINATION | 3 |
| WALL_CLOCK_GUARD_INCOMPLETE | 0 |
| 确认场景启动 | 216 |
| 工程补做（场景） | 0 |
| 未启动项 | 0 |

## 候选段

- 候选段：X = [−0.11, −0.10]
- 代表值建议：X = −0.10
- 备选：X = −0.11
- −0.12 有1次确认跌倒，不进入自动推荐；−0.08/−0.09 也有跌倒且切换变差。
- 候选段只代表已采样的离散网格，不声称未测连续数值均已证明鲁棒。

## 关键限制

本批每图每档仅2次，是开发集局部扫描，不用于估计可靠成功率、统计非劣效或全局最优。5个百分点是工程筛选容忍度；bootstrap区间较宽。UNKNOWN不补成False；base_link界内不等于整机足迹界内；命令记录不等于MuJoCo消费确认。留出集未使用，未训练，未实机，不提交Git。
