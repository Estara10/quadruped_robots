# T2 A/C保持功能试跑结果索引

- 批次：`logs/t2/hold_function_check/20261018_190259_t2hold/`
- 执行报告：[T2_HOLD_FUNCTION_CHECK_EXECUTION_REPORT.md](T2_HOLD_FUNCTION_CHECK_EXECUTION_REPORT.md)
- 冻结计划/配置/attempt台账：批次目录 `t2_hold_function_plan.json`、`attempt_ledger.csv`、`progress.json`
- 逐run结果：[run_results.csv](run_results.csv)
- 保持episode：[hold_episodes.csv](hold_episodes.csv)
- 保持时间线：[hold_timeline.md](hold_timeline.md)
- 语义检查：[semantic_checks.json](semantic_checks.json)
- 配置/显示核验：[config_and_display_check.csv](config_and_display_check.csv)
- 违规表：[hold_violations.json](hold_violations.json)（空）
- 图表：[charts/run_metrics.png](charts/run_metrics.png)、[charts/hold_residence.png](charts/hold_residence.png)
- GLFW预检：[glfw_preflight.json](glfw_preflight.json)

## 结果计数

| 项目 | 值 |
|---|---:|
| 计划项 | 4 |
| 有效任务结果 | 4 |
| ARRIVED | 4 |
| COLLISION/FALL/TIMEOUT | 0 |
| 确认场景启动 | 4 |
| 工程补做（场景） | 0 |
| 读取层分类修复（无重跑） | 1 |
| C 保持episode | 34 |
| 保持违规 | 0 |

## 功能结论

- A 实际无保持（`hold=false/H=0`），C 实际消费 H=0.10/0.30/0.50 仿真秒。
- 34个C episode：H前提前退出0、到期且RA<E未及时退出0、到期RA≥E继续Recovery 4、风险回升6且入口时刻均未重置、34个不同入口时刻、时钟不完整0、审计违规0。
- 四份 A/C overlay 均无 `exit_threshold`；实际退出阈值均为 E=−0.05。CLI 的 X=−0.10 与 A 的 H=0.30 仅为现有入口校验回退值，未写入配置。
- 本轮支持进入“180项扫描功能就绪”判断，但不选择最佳H、不比较参数优劣；180项、48项组合、留出集、训练和实机均未启动。

## 限制

每档仅一次、仅m02一张图；不能估计成功率或H趋势。隐藏GLFW仍依赖DISPLAY/OpenGL，不是真正headless；保持时间为仿真秒，实机hold未验证。原始记录和读取层修复记录均保留，不提交Git。
