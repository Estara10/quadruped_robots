# S6-06 四组单次诊断结果对照

本表由本批 A/B/C/D 各自的生产分析结果汇总。它是一次/组的流程核验，不用于成功率、算法排名、性能优劣或参数选择。终局与记录有效性分开；任务指标从首个 `rl_entered=1 && rl_active=1` 的 cycle 0 起，到终止周期（含）计算。有效时间/路径/速度只使用有效连续区间，不插值。

## 终局、覆盖与运动指标

| 组 | 配置消费 | supervisor终局 | 记录有效性 | 终点距离/m | cycle 0→终局经过时间/s | 有效连续时间/s | 覆盖率 | 有效路径/m | 有效平均速度/m/s | 峰值区间速度/m/s | 碰撞窗口 | 跌倒窗口 | 足端接触/撞击 |
|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| A | A E=-0.05；source confirmed | ARRIVED | VALID | 0.494 | 9.980 | 9.476 | 0.949 | 7.567 | 0.799 | 2.803 | non-foot obstacle=False (complete) | False | 1 / 1 |
| B | B E=-0.05, X=-0.08；source confirmed | ARRIVED | VALID | 0.493 | 6.320 | 6.164 | 0.975 | 7.519 | 1.220 | 3.492 | non-foot obstacle=False (complete) | UNKNOWN | 0 / 0 |
| C | C E=-0.05, H=0.30 s；source confirmed | ARRIVED | VALID | 0.493 | 14.820 | 14.280 | 0.964 | 9.432 | 0.660 | 2.503 | non-foot obstacle=False (complete) | False | 0 / 0 |
| D | D E=-0.05, X=-0.08, H=0.30 s；source confirmed | ARRIVED | VALID | 0.500 | 15.440 | 15.380 | 0.996 | 10.226 | 0.665 | 2.648 | non-foot obstacle=False (complete) | False | 0 / 0 |

`ARRIVED` 表示终止周期距目标不超过 0.5 m。碰撞栏的 False 指本任务窗口没有非足端机器人—障碍终止碰撞；它不代表没有足端接触。A 组单独记录到 FL 足端与 `ppt_obstacle_15` 接触并满足足端撞击条件，故 `abs_reference_collision=true`。B 组任务窗口有未确认跌倒候选，因此跌倒结论为 UNKNOWN；其余组的运动窗口跌倒为 False，仍只限于各自声明窗口。

## 策略周期、切换、Recovery 和收尾

| 组 | policy帧数；周期缺口 | 无效时钟帧 | 实测频率 Hz；周期中位数/p95 ms | 全部模式边沿 / 有效仿真时间边沿 | Agile→Recovery / Recovery→Agile | 短反向切换数（0.5 s，覆盖） | 有效Recovery时间/占有效时间 | 保持审计 | 站立观察 |
|---|---:|---:|---:|---:|---:|---|---:|---|---|
| A | 495; 5 | 9 | 49.98; 19.96/30.21 | 134 / 130 | 67 / 67 | 125 (partial_or_unknown) | 2.812 s / 0.297 | not_observed | COMPLETE (5.000 sim s) |
| B | 315; 2 | 2 | 50.14; 20.02/25.98 | 14 / 14 | 7 / 7 | 10 (partial_or_unknown) | 1.492 s / 0.242 | not_observed | COMPLETE (5.008 sim s) |
| C | 740; 2 | 12 | 50.01; 19.97/25.92 | 34 / 34 | 17 / 17 | 29 (partial_or_unknown) | 5.046 s / 0.353 | incomplete | COMPLETE (5.004 sim s) |
| D | 771; 2 | 0 | 50.00; 19.99/25.53 | 34 / 34 | 17 / 17 | 28 (partial_or_unknown) | 5.286 s / 0.344 | pass | COMPLETE (5.000 sim s) |

短反向切换数使用分析器的 0.5 s 窗口；四组均为 `partial_or_unknown` 覆盖，数字仅是可观察部分计数。无效时钟不删除模式边沿，但不进入精确时间计算。A/B 无保持配置，保持记为 `not_observed`。C 的 17 个 episode 均满足已观察到的保持规则，但 6 个 episode 的连续时钟覆盖字段不完整，故聚合状态为 `incomplete`。D 的 17 个 episode 均有完整时钟覆盖且规则通过。

## 自动生成源及原始记录

自动批次汇总：[`comparison.md`](../../../../logs/manual_abcd_compare/20261013_101052_11b8e3/comparison.md)；机器结果：[`batch_summary.json`](../../../../logs/manual_abcd_compare/20261013_101052_11b8e3/batch_summary.json)。每组的 `run_context.json`、`runtime_record.jsonl`、`terminal.json`、`metrics.json`、`process_facts.json`、启动增益轨迹和分析器输出均保留在对应日志目录（每组 `rep_01/` 下唯一运行子目录）。原始记录未改写。

## 时间线

| 组 | 时间线 | 原始记录目录 |
|---|---|---|
| A | [timeline A](timelines/A.png) | [本组日志](../../../../logs/manual_abcd_compare/20261013_101052_11b8e3/A/rep_01/) |
| B | [timeline B](timelines/B.png) | [本组日志](../../../../logs/manual_abcd_compare/20261013_101052_11b8e3/B/rep_01/) |
| C | [timeline C](timelines/C.png) | [本组日志](../../../../logs/manual_abcd_compare/20261013_101052_11b8e3/C/rep_01/) |
| D | [timeline D](timelines/D.png) | [本组日志](../../../../logs/manual_abcd_compare/20261013_101052_11b8e3/D/rep_01/) |

每组正常到达后分别完成约 5 仿真秒站立观察。命令 6 为正常减速/站立收尾请求；日志确认 FIXEDSTAND、站立条件及保持观察完成。没有请求 PASSIVE，因此 PASSIVE 转换字段 UNKNOWN 不表示本次站立收尾失败。MuJoCo 命令实际消费仍未验证。
