# 400项结果索引

## 第一轮筛选后的用户决定

2026-10-16：后续候选范围缩小到X=−0.05至−0.13（含端点），保留现有−0.05（A参考）、−0.07、−0.09、−0.11、−0.13五档。另三档暂不继续，但下列全部8档、400项数据及评分保持原样，不删除或修改旧排名。本决定没有授权新实验，也未确定细选步长及重复次数。

## 用户确认权重的事后评分

按到达40%、无终止性碰撞30%、效率20%、切换稳定性10%计算，400条均参与；先逐run变换，再逐地图平均、地图等权汇总。该规则在400次运行结束后确定，属于事后辅助分析，未改动运行前冻结方法或原始数据。

- [完整评分报告、口径和排名](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/weighted_score_v1/REPORT.md)
- [分项得分图](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/weighted_score_v1/weighted_score_components.png)
- [逐地图得分热图](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/weighted_score_v1/map_threshold_weighted_scores.png)
- [相对A的分差与区间](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/weighted_score_v1/score_difference_vs_A.png)
- [8档评分CSV](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/weighted_score_v1/threshold_scores_8.csv)
- [400条逐run评分CSV](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/weighted_score_v1/per_run_scores_400.csv)

批次：[原始目录](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/)

## 表格与机器可读数据

- [冻结400项计划与attempt证据](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/expanded_400_plan.json)
- [逐run结果（400行）](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/expanded_run_results.csv)
- [逐attempt结果（401行）](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/expanded_attempt_results.csv)
- [地图×阈值结果（80行）](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/map_threshold_summary_80.csv)
- [八档汇总](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/threshold_summary_8.csv)
- [地图聚类bootstrap区间及B−A差值](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/map_cluster_bootstrap_ci.csv)
- [最终机器可读汇总](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/expanded_final_summary.json)
- [预定代表时间线清单](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/representative_timelines.json)
- [80/160/240/320/400项阶段快照](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/round_progress_400.json)
- [首次批次包装器退出码记录](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/batch_runner_exit.json)：首次400项运行后绘图代码报错退出码1；已仅修复派生分析并重建最终输出。原记录保留。

## 总览图

- [终局构成](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/terminal_composition.png)
- [按阈值到达比例热图](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/map_threshold_arrival_fraction.png)
- [到达样本耗时](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/elapsed_task_duration_s.png)
- [按有效运动时间归一化的短反向频率](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/short_reverse_per_valid_min.png)
- [Recovery有效运动时间占比](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/recovery_share_valid_motion_time.png)
- [任务窗口碰撞结果](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/collision_outcome_by_threshold.png)
- [任务窗口跌倒结果](../../../../../logs/t1/b_exit_validation_400/20261016_131943_26/fall_outcome_by_threshold.png)

## 单次运行资料

逐run的完整数据分别保留在批次`runs/<plan_item_id>/result/...`目录；第一次计划项的命令行失败及其修复attempt在同计划项`attempt_02/`和计划JSON中保留。没有将任何运行数据合并或删除。
