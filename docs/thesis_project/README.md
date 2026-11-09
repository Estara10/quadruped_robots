# 项目文档导航

更新：2026-11-09。R1 仍为唯一 ACTIVE 研究任务。仿真当前结果以 Periodic A/B 45 项为准；实机优先完成 ABS 在 Go2 的实现，近期工作为相机 IMU→固定机身转换及后续原生 A→B 基础验证。安装参数尚未测量，完整 ABS 实机控制未通过。

## 当前先读

- [当前状态](CURRENT_STATE.md)：最新结果、限制和下一项具体工作。
- [实机入口](real_go2/README.md)：Go2 EDU、原生运动、D435i/VINS 与控制边界。
- [机身转换与测量](../../scripts/go2_vins/BODY_FRAME.md)：base 原点、安装测量、状态接口及离线检查。
- [第二／三组分段审阅](real_go2/20261109_ROUND23_SEGMENTED_REVIEW.md)：净位移与停后变化；原方案保留备用。
- [在线外参 A/B 回放](real_go2/20261109_EXTRINSIC_AB_REPLAY.md)：稳定性改善不等于精度通过，现场根因仍未知。
- [最新仿真 45 项报告](evidence/R1/PERIODIC_AB_45/ANALYSIS_REPORT.md)：43 项有效、2 项启动故障，未证实跨图普遍收益。
- [A1 需求记录](real_go2/A1_PROJECT_REQUIREMENTS.md)：独立导航工程的原始需求；后续计划在该工程维护。
- [R1任务](tasks/R1.md)：已完成阶段、当前范围及完成条件。
- [R1项目](designs/R1/README.md)、[研究计划](designs/R1/PLAN.md)：已有产物、ABS阈值依据和下一步候选方案。
- [90项开发确认报告](evidence/R1/M2_ABC_repeat_90/分析报告.md)：d01/d03/m01的A/B/C各10次，安全完成分别A 25/30、B 23/30、C 25/30。

频繁切换是正常现象。比较切换约束对安全完成、到达、碰撞、超时、推进和耗时的作用，不以切换更少判断方法更好。

- [120项阈值位置比较报告](evidence/R1/M2_threshold_position_120/分析报告.md)：四设置总体到达20/30、20/30、17/30、22/30，未发现跨地图一致收益。
- [R1-6六图72项报告](evidence/R1/R1v2_threshold_72/分析报告.md)：六张R1 v2地图的分层开发比较。
- [R1-7 m02三图18项报告](evidence/R1/R1_m02_threshold_refine_18/分析报告.md)：两种新增双阈值设置，含六候选跨批次对照。

## 目标、方法与记录

- [项目目标](OVERVIEW.md)、[路线图](ROADMAP.md)、[实验计划](EXPERIMENT_PLAN.md)、[切换规则](SWITCH_CONTRACT.md)
- [日志导航](../../logs/README.md)：开发、正式批次及R1记录的实际位置。
- 周报与总体工作材料位于本地`weekly_reports/`，不提交Git或GitHub；按文件标注的截止日期阅读。

## 历史实验结果

- [原M1报告](evidence/S7-02/S7-02_EXECUTION_REPORT_20261023.md)、[结果索引](evidence/S7-02/RESULT_INDEX.md)、[原始记录清理限制](evidence/S7-02/DATA_RETENTION_20261023.md)
- [M3最终报告](evidence/S7-02/M3_ABCD_240_FINAL_REPORT_20261023.md)、[结果索引](evidence/S7-02/M3_ABCD_240_RESULT_INDEX.md)、[比较图](evidence/S7-02/M3_ABCD_COMPARISON.png)
- [T1任务](tasks/T1.md)、[X选择记录](designs/T1/EXIT_THRESHOLD_SELECTION.md)
- [T2任务](tasks/T2.md)、[H选择记录](designs/T2/HOLD_TIME_SELECTION.md)
- [S7-01准备任务](tasks/S7-01.md)、[历史S7-02](tasks/S7-02.md)

原研究代表参数为E=−0.05、B/D的X=−0.10、C/D的H=0.20仿真秒。新候选不覆盖这些历史设置。

## 地图

- [M1冻结地图](designs/M1/library_v1/README.md)：30张开发图、30张原留出图。
- [M2的18张开发图](designs/M2/selected_v2/README.md)：R1-4使用其中d01、d03、m01。
- [M3 v1.4的30张挑战图](designs/M3/README.md)：历史筛选集合，部分地图与M1同源，不是新独立留出集。

## 历史文件与复现入口

- [2026-10-23阶段说明](../../archive/markdown/20261030_cleanup/docs/thesis_project/PROJECT_STATUS_AND_EXPERIMENT_DECISIONS_20261023.md)：仅反映当日阶段。
- [原M1执行说明](designs/S7/FORMAL_RUN_GUIDE.md)、[M3执行说明](designs/S7/m3_challenge_240/RUN_GUIDE.md)：已完成批次的复现参考，不是再次派发指令。
- [2026-10-30整理归档](../../archive/markdown/20261030_cleanup/README.md)：三份旧文档的新位置与四份跳转页删除清单。
- [2026-10-23文档归档](../../archive/markdown/20261023_research_replan/README.md)、[2026-10-18历史归档](../../archive/markdown/20261018/README.md)

旧提示词和旧路径的小型入口保留以兼容已有引用。代码、模型、地图和原始记录不因本轮文档整理而搬动或删除。
