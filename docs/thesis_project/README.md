# 当前毕业设计文档

## 日常工作入口

- [当前任务S7-02](tasks/S7-02.md)：正式A/B/C/D仿真执行；ACTIVE / AWAITING_RUN_AUTHORIZATION，240项未授权、未启动。
- [已完成任务S7-01](tasks/S7-01.md)：方法冻结、最小运行器、开发补证与决策时钟重建准备；COMPLETE（Director有条件通过）。
- [T2最终状态与研究代表参数](tasks/T2.md)、[参数记录](designs/T2/HOLD_TIME_SELECTION.md)：T2 COMPLETE（Director有条件通过）。
- [正式方法复核稿](designs/S7/FORMAL_METHOD_REVIEW.md)、[方法冻结审阅摘要](designs/S7/METHOD_FREEZE_REVIEW.md)及[240项正式方案](designs/S7/FORMAL_EXPERIMENT_PLAN_DRAFT.md)：方法与矩阵已冻结，240项仍未获运行授权。
- [正式执行说明](designs/S7/FORMAL_RUN_GUIDE.md)：计划查看、环境核对、待授权启动、状态/恢复、分析、输出目录和停止原因。
- [运行准备报告](evidence/S7-01/runner_readiness/RUNNER_READINESS_REPORT.md)：最小运行器、断点恢复、三层汇总入口及21项定向离线联测；明确未做运行态验证。
- [A run cycle0时钟复核](evidence/S7-01/cycle0_clock_review/CYCLE0_CLOCK_REVIEW.md)：decision-time有效、publication-time stale；重建已统一接入正式分析，原始STALE保留；A/D核对完成；S7-01已COMPLETE，S7-02待运行授权。
- [m02 A/D开发补证执行报告](evidence/S7-01/dev_supplement/DEV_SUPPLEMENT_EXECUTION_REPORT.md)：A 原始发布态 `ARRIVED/VALID/UNKNOWN`、统一分析 `True`；D `ARRIVED/VALID/True`；2次启动、0替换；非正式数据，不影响240项授权。
- [240项冻结矩阵CSV](designs/S7/formal_matrix_240.csv)、[JSON](designs/S7/formal_matrix_240.json)及[生成脚本](designs/S7/generate_formal_matrix.py)：调度种子2026102001，仅控制计划顺序，不控制物理初态。
- [已完成216项扫描提示词](T1_M2_LOCAL_SCAN_216_EXECUTOR_PROMPT.md)，不构成再次运行授权。
- [已完成18图试跑提示词](T1_M2_18_MAP_SMOKE_PROMPT.md)与对应报告保留作本轮前置证据。
- [当前状态](CURRENT_STATE.md)：当前进度、限制和授权。
- [已完成任务T1](tasks/T1.md)：B组退出阈值开发选参，Director有条件通过。
- [项目总览](OVERVIEW.md)、[路线图](ROADMAP.md)、[实验计划](EXPERIMENT_PLAN.md)。
- [切换合同](SWITCH_CONTRACT.md)：A/B/C/D规则、阈值和计时语义。

## 当前地图与参数资料

- [已选代表退出阈值−0.10及限制](designs/T1/EXIT_THRESHOLD_SELECTION.md)。
- [216项审阅后执行报告](evidence/T1/m2_local_scan_216/T1_M2_LOCAL_SCAN_216_EXECUTION_REPORT.md)。
- [M2 easy/middle/difficult地图说明](designs/M2/selected_v2/README.md)、[18图总览](designs/M2/selected_v2/overview_18.png)。
- [M1冻结地图库](designs/M1/library_v1/README.md)：开发/留出边界保留。
- [三轮参数比较与咨询说明](T1_PARAMETER_COMPARISON_WEB_REVIEW.md)。
- [T1两档36图执行报告](evidence/T1/two_thresholds_36_maps/T1_TWO_THRESHOLDS_36_EXECUTION_REPORT.md)。其他执行报告仍在evidence下原位置。

## 历史归档

[2026-10-18归档索引](../../archive/markdown/20261018/README.md)保存75份已结束或被替代的任务、已执行提示词、旧P1治理、旧日报和早期仓库基线，含旧路径与当前位置对照。历史提示词不构成当前执行授权。

技术合同、论文笔记、训练/部署使用说明、地图规则及实验报告保留；部分历史日志按授权清理，范围见CURRENT_STATE及清理记录，不能将所有历史原始记录视为完整留存。
