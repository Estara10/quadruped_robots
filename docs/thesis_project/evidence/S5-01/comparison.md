# S5-01 B/D 单次功能诊断对照

每组一次，仅为本次诊断观察；不能据此计算成功率、推断泛化或认定某组性能更好。场景均为 `scene_random_04.xml` / `random_04`（S2-08-r1），目标 `(7,0)`，到达半径 `0.5 m`；共同模型指纹 `8ac55fa1e09853f8a7a0b35e96b966b7d919154ae2b8eecd143625a6b8460694`。B：E=-0.05、X=-0.08、无保持。D：同 E/X，H=0.30 仿真秒。

两次使用同一初态流程，但记录没有保存起始关节实测状态向量；因此不能证明两次初态数值完全一致，也不是严格重放配对。

| 指标 | B | D | 口径 |
|---|---:|---:|---|
| run_id | `[local run ID omitted]` | `[local run ID omitted]` | 独立 session/run |
| 配置消费 | B / `hysteresis_switch` / hold=false | D / `combined_switch` / hold=true | controller 来源日志和首周期绑定 |
| 记录有效性与终局 | VALID / ARRIVED | VALID / ARRIVED | 终止策略快照完整 |
| 到达时距目标 | 0.4781 m | 0.4693 m | 目标半径 0.5 m |
| 起点到终止周期经过时间 | 9.340 s | 15.296 s | 终止 sim_time 减运动起点 sim_time |
| 有效连续间隔累计时间 | 9.234 s | 15.214 s | 不跨无效时钟/间隔补值 |
| 有效连续片段路径长度 | 7.960 m | 10.162 m | 只用有效连续片段 |
| 有效连续片段平均速度 | 0.862 m/s | 0.668 m/s | 路径长度/有效连续累计时间 |
| Agile / Recovery 有效时钟周期 | 333 / 132 | 478 / 286 | 运动窗口、时钟有效帧 |
| Agile→Recovery / Recovery→Agile | 35 / 34 | 16 / 16 | 模式边沿总数 |
| 全部模式边沿 / 有有效仿真时间 | 69 / 69 | 32 / 32 | 无效时钟不会删除边沿；本次边沿均有时间 |
| Recovery 有效连续驻留累计 | 2.682 s | 5.710 s | 仅有效连续间隔，缺口不插值 |
| RA 范围 | [-0.9993, 0.3126] | [-0.9989, 0.3292] | 运动窗口策略帧 |
| 0.5 s 短反向切换 | 64 | 24 | analyzer 指标定义的窗口统计 |
| 策略帧数 / 无效时钟帧 | 516 / 3 | 814 / 2 | 原始采集 |
| 整段策略频率 | 50.233 Hz | 50.159 Hz | 连续策略周期单调时间间隔统计 |
| 足端接触 / 足端撞击 | 0 / 0 | 0 / 0 | 未记录事件；不外推为全程无接触 |
| 非足端碰撞终止 | 未记录 | 未记录 | 两次均非碰撞终局；碰撞否定结论保留覆盖限制 |
| 跌倒 | false（按本次规则） | UNKNOWN | D 有未确认姿态候选 |
| 站立观察 / 进程清理 | 约 5 仿真秒 / 完成 | 约 5 仿真秒 / 完成 | `process_facts.json` 与 terminal |

## 产物位置

- B：`../../../../logs/manual_ab_compare/20261011_163046_c9f3e0/B/[local run ID omitted]/`
- D：`../../../../logs/manual_ab_compare/20261011_163046_c9f3e0/D/[local run ID omitted]/`
- 每组 `analysis/run_results.md`、`metrics.json`、`timeline.png`；原始 `runtime_record.jsonl`、`run_context.json`、`terminal.json`、`process_facts.json` 均保留。
