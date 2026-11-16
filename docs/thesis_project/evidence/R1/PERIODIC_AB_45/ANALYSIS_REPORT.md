# R1 A/B双阈值 Periodic 比较（45项）

## 主要发现

本批按5轮冻结交错排程运行45项，得到43项有效任务结果；序号001因排程设置名更新后overlay路径仍引用旧设置名，未启动场景；序号002启动MuJoCo后，DDS在沙箱内无法枚举网络接口，在任务开始前退出。两项均记录为启动故障，未重试或替代。其余43项地图、实际E/X、保持关闭、几何射线、目标、到达半径0.5m、20/120秒任务上限和2秒到达后观察均由运行上下文/命令核对（43/43）；Periodic P=0.06另由controller日志核实。实际场景进程启动44次，无任务期系统安全中止或有效任务配置不符。有效任务终局共到达20、碰撞12、仿真限时11、跌倒0、墙钟保护0、安全中止0；安全完成TRUE/FALSE/UNKNOWN为20/23/2，UNKNOWN均为两次未进入有效任务窗的启动失败。

逐图看，d09窄档的B_E04_X07在4项有效运行中4次到达且无碰撞，A为4/5到达、1次碰撞；但B_E04_X07有1项启动失败，且同时改变E与X。m02中档B_E04_X07到达2/5（A为1/4），碰撞3/5（A为3/4），是局部改善线索；B_E05_X10在m02到达1/5、碰撞4/5。challenge27中A到达2/5，两档B各到达1/5、仿真限时4/5。B_E05_X10没有在三图形成一致改善。每格计划5次、有效任务结果4或5次；所有地图都是开发图，本批不支持普遍优越或等效结论。


## 9格地图×设置汇总

| map | setting | valid_task_n | startup_failure_n | safe_true_n | safe_false_n | safe_unknown_n | arrived_n | collision_termination_n | fall_termination_n | diagnostic_sim_time_limit_n | wall_clock_guard_incomplete_n | system_safety_abort_n | net_goal_progress_mean_m | end_distance_median | arrival_time_n | arrival_time_median_s | recovery_share_median | switch_count_median | periodic_updates_total | periodic_interval_run_median_s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| r1_d09_gap_narrow_v2.xml | A_REF | 5 | 0 | 4 | 1 | 0 | 4 | 1 | 0 | 0 | 0 | 0 | 6.441 | 0.480 | 4 | 5.600 | 0.122 | 4 | 40 | 0.060 |
| r1_d09_gap_narrow_v2.xml | B_E04_X07 | 4 | 1 | 4 | 0 | 1 | 4 | 0 | 0 | 0 | 0 | 0 | 7.060 | 0.480 | 4 | 5.521 | 0.126 | 3.000 | 34 | 0.061 |
| r1_d09_gap_narrow_v2.xml | B_E05_X10 | 5 | 0 | 4 | 1 | 0 | 4 | 1 | 0 | 0 | 0 | 0 | 6.474 | 0.485 | 4 | 5.810 | 0.149 | 4 | 53 | 0.060 |
| r1_m02_space_middle_v2.xml | A_REF | 4 | 1 | 1 | 3 | 1 | 1 | 3 | 0 | 0 | 0 | 0 | 5.981 | 1.903 | 1 | 6.300 | 0.205 | 69.000 | 13 | 0.078 |
| r1_m02_space_middle_v2.xml | B_E04_X07 | 5 | 0 | 2 | 3 | 0 | 2 | 3 | 0 | 0 | 0 | 0 | 6.208 | 1.822 | 2 | 6.040 | 0.181 | 46 | 23 | 0.080 |
| r1_m02_space_middle_v2.xml | B_E05_X10 | 5 | 0 | 1 | 4 | 0 | 1 | 4 | 0 | 0 | 0 | 0 | 5.859 | 1.951 | 1 | 6.460 | 0.245 | 38 | 64 | 0.062 |
| challenge27.xml | A_REF | 5 | 0 | 2 | 3 | 0 | 2 | 0 | 0 | 3 | 0 | 0 | 4.327 | 4.994 | 2 | 6.611 | 0.458 | 569 | 115 | 0.060 |
| challenge27.xml | B_E04_X07 | 5 | 0 | 1 | 4 | 0 | 1 | 0 | 0 | 4 | 0 | 0 | 3.394 | 5.027 | 1 | 5.320 | 0.451 | 560 | 162 | 0.060 |
| challenge27.xml | B_E05_X10 | 5 | 0 | 1 | 4 | 0 | 1 | 0 | 0 | 4 | 0 | 0 | 3.399 | 5.047 | 1 | 5.860 | 0.460 | 359 | 258 | 0.062 |

## 同期差异

`contrasts.csv`列出两档B−A及B_E05_X10−B_E04_X07的到达率差、Newcombe–Wilson 95%区间、碰撞/限时率差和平均净推进差。到达率区间很宽并跨0。B_E04_X07在d09和m02有局部积极线索，但在challenge27到达更少且推进均值较低；由于E和X同时改变，不能单独归因于滞回。B_E05_X10与A共用E，更接近增加滞回的对照；它在d09到达率相同，在m02碰撞更多，在challenge27限时更多，没有支持其跨图改善的证据。现有结果不足以据此选择B进入新布局验证；若后续仍要验证，应先审阅典型策略帧并预先指定单一候选和布局。


## Periodic更新与记录范围

controller日志确认Periodic P=0.06的有效任务 43/43；记录到type=periodic更新共762次，其中写入Recovery 762次。逐次中位间隔的中位数为0.062s（样本43项），优化耗时逐次中位数的中位数为1.167ms。切换次数使用metrics中的`mode_transitions`，每次切换是否完整可见`mode_transition_count_coverage`字段；有效运行间的切换统计见逐次CSV，仅用于行为描述，不以切换减少判定有效性。实际Periodic间隔受控制器策略周期离散影响；这些是本批描述值。


## 限制

任务状态分别报告ARRIVED、COLLISION_TERMINATION、FALL_TERMINATION、仿真限时、墙钟保护和系统安全中止。安全完成UNKNOWN保留在汇总内，并在逐次CSV给出分类原因；启动失败因没有有效任务窗而为UNKNOWN。到达时间只对到达样本报告。逐次CSV记录了每个有效运行首个Active策略帧的位置、朝向与机身速度；初态未严格重放。各格样本少，不能据此证明参数因果或迁移到新布局。旧Frozen结果不并入本批。没有清理历史记录或追加运行。
