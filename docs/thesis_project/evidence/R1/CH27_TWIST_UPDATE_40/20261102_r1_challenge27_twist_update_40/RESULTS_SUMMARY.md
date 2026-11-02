# Challenge27：A/C × Frozen/Periodic（40项）结果

## 设置与完成状态

共40个计划项、实际启动40次；10个区组每组四条件各一次，排程在`actual_schedule.csv`。地图为登记场景challenge27.xml；目标(7.5,0)m；E=−0.05；A无保持，C的H=0.10仿真秒；Periodic P=0.06仿真秒，Frozen显式配置。所有40项runtime config binding confirmed、VALID、shutdown和closeout完成。MuJoCo日志均标记GLFW visible=1，RViz关闭，未启用屏幕帧缓冲捕获。
起初因状态HUD未显示而在第2项后暂停；随后将HUD绘制移至场景右上角并放在普通UI/帮助层之后，重建MuJoCo查看器后从原排程第3项继续。第1、2项未重跑。面板修复只改绘制顺序和位置，不改状态帧读取、策略或控制逻辑；没有截图或录像。

## 每条件结果

安全完成按现有分析器给出T/F/U；时间限时不重跑。到达耗时只在到达样本中统计，任务推进/末端目标距离覆盖全部10次。

|配置|VALID|安全完成T/F/U|到达|碰撞终局|跌倒终局|仿真限时|墙钟限时|平均净目标推进|平均末端距离|到达耗时n|平均到达耗时|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|A-Frozen|10/10|3/7/0|3|4|0|3|0|4.60m|2.94m|3|8.73s|
|A-Periodic|10/10|6/4/0|6|1|0|3|0|5.61m|1.93m|6|8.08s|
|C-Frozen|10/10|2/8/0|2|3|0|5|0|4.66m|2.87m|2|11.14s|
|C-Periodic|10/10|2/8/0|2|1|1|6|0|3.94m|3.60m|2|5.73s|

## Periodic更新机会与计算成本

恢复片段以控制器日志中的entry更新为准。每项周期耗时采用权威RuntimeFrame字段`policy_cycle_duration_ns`，统计范围为`policy_state=RECOVERY`且可计时的策略帧；超20ms计为超周期。更新所在策略周期按日志policy_cycle与RuntimeFrame策略周期ID匹配。Twist优化耗时取日志中的steady-clock optimizer_ns，和周期总耗时分开。Frozen/Periodic分别合并A、C，另在更新机会表中分设置列出。

|配置|Recovery片段|入口优化|Periodic优化|发生过Periodic更新的片段|首次Periodic前退出/已退出片段|末端仍Recovery片段|Periodic写入Recovery|
|---|---:|---:|---:|---:|---:|---:|---:|
|A-Frozen|1057|1057|0|0|—|1|0/0|
|A-Periodic|697|697|360|139|558/696|1|360/360|
|C-Frozen|488|488|0|0|—|1|0/0|
|C-Periodic|491|491|678|491|0/488|3|678/678|

|模式|入口Twist优化耗时中位数/p95|Periodic优化耗时中位数/p95|Recovery策略周期 >20ms|更新所在策略周期 >20ms|
|---|---:|---:|---:|---:|
|Frozen|3.12/11.52ms|—|129/6631 (1.95%)|101/1545 (6.54%)|
|Periodic|3.06/11.30ms|3.30/12.30ms|203/5667 (3.58%)|178/2226 (8.00%)|

本批Periodic更新间隔观测为0.060–0.104仿真秒，1038/1038次都发生在deadline后首个仍处Recovery的策略周期，且1038/1038更新写入Recovery输入。最大比deadline晚0.044秒，对应该次下一策略周期在deadline后44ms到达；没有提前更新，也没有延后到更后面的Recovery周期。见`periodic_timing_audit.json`。成本只描述本次challenge27的小样本运行，不能作为普遍实时性结论。

## 行为观察与解释

逐次指标、区组、轨迹长度、目标距离及成本在`per_run_results.csv`和`final_analysis.json`。代表到达轨迹：第4项A-Periodic的机器人轨迹横向y约在−0.29至+0.92m之间变化，终点约(7.42,−0.28)m；第15项C-Periodic约在−0.31至+0.93m之间变化，终点约(7.60,−0.30)m。两条成功路线都先向正y侧绕行，再回到目标附近；这是两条个例，不代表稳定路线。一个C-Periodic碰撞样本（第5项）接触`ppt_obstacle_02`的FR小腿；碰撞前最后策略帧为AGILE，位置约(4.02,0.05)m。时间相邻不代表因果。未保存屏幕截图或录像。碰撞、跌倒与限时是本地图上的实际结果。本批只在challenge27这一张地图上做小规模检查，不据此宣布Periodic普遍有效，也不外推到其他地图。

## 记录入口

计划/实际排程：本目录上一级批次中的`run_plan.json`、`actual_schedule.csv`、`actual_scene_starts.csv`。每项配置快照、逐次runtime record、context、terminal和ROS/MuJoCo日志均保留在批次目录。原先2项暂停时的部分记录见`PARTIAL_STATUS.md`；最终完整逐次表见`per_run_results.csv`。
