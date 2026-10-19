# 手动 A 运行汇总

计划参数：{"entry_threshold_E": -0.05, "exit_threshold_X_BD": -0.08, "recovery_min_hold_H_CD_sim_s": 0.3}。
预实验准备批次不是正式实验。各组样本数、INVALID、未运行及合法终局单独计数；UNKNOWN 不按零处理。
任务主窗口从首个 rl_entered=1 且 rl_active=1 周期到真实终局（含终止周期）；历史0.10m位移窗口仅作辅助。有效连续时间、路径和速度不冒充完整经过时间。

| 指标 | A |
|---|---|
| 记录有效性 | INVALID |
| supervisor原始终局 | STARTUP_FAILURE |
| 结束原因 | UNKNOWN |
| 任务窗口碰撞判定 | UNKNOWN |
| 任务窗口跌倒判定 | UNKNOWN |
| 非足端碰撞 | UNKNOWN |
| 足端接触次数 | UNKNOWN |
| 足端撞击次数 | UNKNOWN |
| 终点距目标/m | UNKNOWN |
| 首个活跃策略周期至终局经过时间/s | UNKNOWN |
| 全部模式切换次数 | UNKNOWN |
| 具备有效仿真时间的切换次数 | UNKNOWN |
| 短时反向切换次数 | UNKNOWN |
| 短反向统计覆盖 | UNKNOWN |
| Agile 周期数 | UNKNOWN |
| Recovery 周期数 | UNKNOWN |
| 全记录策略频率/Hz | UNKNOWN |
| RL活跃起点至终局经过时间/s | UNKNOWN |
| RL活跃窗口有效连续时间/s | UNKNOWN |
| RL活跃窗口有效路径/m | UNKNOWN |
| RL活跃窗口有效平均速度/(m/s) | UNKNOWN |
| 任务窗口有效仿真时间覆盖率 | UNKNOWN |
| RL活跃窗口模式边沿数 | UNKNOWN |
| 配对风险→Recovery事件数 | UNKNOWN |
| 风险→模式决定延迟中位数/ns | UNKNOWN |
| 有效连续运动时间/s | UNKNOWN |
| 有效片段 Recovery 时间/s | UNKNOWN |
| 有效片段路径/m | UNKNOWN |
| 有效片段平均速度/(m/s) | UNKNOWN |
| 有效区间峰值路径速度/(m/s) | UNKNOWN |
| Recovery/有效连续运动时间占比 | UNKNOWN |
| Recovery保持审计状态 | UNKNOWN |
| 保持审计完整episode数 | 0 |
| 保持审计不完整episode数 | 0 |
| 保持规则违规数 | 0 |
| 实际启动增益轨迹 | MISSING |
| 实际增益记录事件数 | 0 |
| 到达后站立观察 | NOT_REACHED_OR_UNKNOWN |
| PASSIVE控制器转换确认 | UNKNOWN |
| 本次进程退出确认 | True |
| 收尾记录完整 | True |

每次运行的完整指标在 `batch_summary.json`；各 run 的原始记录、指标和 timeline.png 位于对应组/run_id 子目录。
ARRIVED 与正常站立收尾分开；任务窗安全 UNKNOWN 不按零处理。INVALID 和未运行状态另行保留。
