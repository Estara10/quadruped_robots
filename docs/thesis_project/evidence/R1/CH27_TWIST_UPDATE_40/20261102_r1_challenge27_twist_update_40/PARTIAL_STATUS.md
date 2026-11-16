# Challenge27 A/C × Frozen/Periodic：暂停状态（2/40项）

本文件记录第一次因实时状态面板不可见而暂停时的快照；随后按用户要求调整并重建面板、从第3项继续，最终40/40项完成。当前最终结果以[完整报告](RESULTS_SUMMARY.md)为准。

本批使用可见MuJoCo、关闭RViz和帧缓冲截图。运行日志两项均记录`requested=visible visible=1`，但操作者反馈看不到实时状态面板；因此按现场观察要求暂停，未派发其余38项。GLFW窗口可见标记不等于控制状态面板可见。

## 已启动项

|配置|结果|安全完成|碰撞|跌倒|任务窗时长|路径长度|末端距目标|Recovery片段/更新|记录与清理|
|---|---|---|---|---|---:|---:|---:|---|---|
|A-Frozen|ARRIVED|True|False|False|5.262s|7.487m|0.490m|1片段；入口1、Periodic 0|VALID；shutdown/closeout True/True|
|C-Periodic|DIAGNOSTIC_SIM_TIME_LIMIT|False|False|UNKNOWN|20.020s|8.434m|5.053m|68片段；入口68、Periodic 93|VALID；shutdown/closeout True/True|

## Twist与周期成本（目前仅有A-Frozen 1次、C-Periodic 1次）

|模式|入口优化调用|Periodic优化调用|入口优化耗时中位数|Periodic优化耗时中位数|Recovery策略周期 >20ms|更新所在策略周期 >20ms|
|---|---:|---:|---:|---:|---:|---:|
|Frozen|1|0|7.483 ms|—|0/22 (0.0%)|0/1|
|Periodic|68|93|2.842 ms|3.261 ms|14/467 (3.0%)|12/161|

C-Periodic日志记录93次Periodic更新，覆盖68个Recovery片段；0/67个已退出片段在首个Periodic更新前退出，另有1个片段在仿真时限终止时仍处于Recovery。全部Periodic更新日志`written_to_recovery=1`，首次更新间隔为仿真时间0.06秒或更长（策略周期离散导致略晚）。这些只描述单次运行。

## 排程和原始记录

40项计划与实际区组顺序见`logs/09_R1_switch_constraints/challenge27_twist_update_40/20261102_r1_challenge27_twist_update_40/actual_schedule.csv`及`run_plan.json`；两项原始策略帧、context、终局、配置快照和启动日志均保留在批次目录。其余38项明确为NOT_STARTED。

无碰撞的两条结果不构成A/C或Frozen/Periodic优劣结论；本批不完整，不能外推到其他地图。
