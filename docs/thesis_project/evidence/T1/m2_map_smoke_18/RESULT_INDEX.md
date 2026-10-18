# T1 新版M2 18图试跑结果索引

- 批次：`logs/t1/m2_map_smoke_18/20261018_154940_m2v2_18/`
- 执行报告：[T1_M2_18_MAP_SMOKE_EXECUTION_REPORT.md](T1_M2_18_MAP_SMOKE_EXECUTION_REPORT.md)
- 计划/参数/初始状态/attempt 台账：批次目录 `m2_map_smoke_plan.json`、`attempt_ledger.csv`、`progress.json`
- 逐 run 指标：[run_results.csv](run_results.csv)
- 契约/显示核验：[contract_and_display_check.csv](contract_and_display_check.csv)
- 碰撞上下文：[collision_context.csv](collision_context.csv)
- 图表：[charts/terminal_outcomes.png](charts/terminal_outcomes.png)、[charts/task_metrics.png](charts/task_metrics.png)、[charts/switching_activity.png](charts/switching_activity.png)
- GLFW 预检：[glfw_preflight.json](glfw_preflight.json)

## 结果计数

| 项目 | 值 |
|---|---:|
| 计划项 | 18 |
| 有效任务结果 | 18 |
| ARRIVED | 14 |
| COLLISION_TERMINATION | 3 |
| DIAGNOSTIC_SIM_TIME_LIMIT | 1 |
| FALL_TERMINATION | 0 |
| 确认场景启动 | 18 |
| 工程补做（场景） | 0 |
| 未启动项 | 0 |

## 批次文件

| 文件 | 说明 |
|---|---|
| `m2_map_smoke_plan.json` | 冻结顺序、参数、每项 attempt、初始状态、编排事件、版本与最终分析 |
| `run_results.csv` | 18 项逐 run 任务/切换/安全/收尾指标 |
| `contract_and_display_check.csv` | 实际 B/E/X/保持、E-X 消费、2 秒观察、隐藏窗口、RViz 关闭核验 |
| `collision_context.csv` | d02/d07/d08 碰撞前策略帧与 Recovery 边沿上下文 |
| `RESULTS_SUMMARY.md` | runner 自动生成的一览表 |
| `runner.log` | 完整终端记录 |
| `runs/<序号>_<scene_id>/` | 每项的原始 run、日志、metrics、terminal、process facts 与每项派生分析 |

原始运行记录未改写；历史批次、旧地图成绩和旧报告保持原样。本批为非正式开发诊断，不用于成功率、阈值排名或正式性能结论。
