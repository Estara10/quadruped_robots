# 项目文档导航

更新：2026-10-23。

## 现在先看什么

- [当前状态](CURRENT_STATE.md)：原M1与M3各完成240项，旧批次结果保留，R1为当前唯一ACTIVE项目。
- [R1项目](designs/R1/README.md)及[研究计划](designs/R1/PLAN.md)：先看为什么碰撞和超时，再决定改哪个参数；尚未执行。
- [项目现状说明](PROJECT_STATUS_AND_EXPERIMENT_DECISIONS_20261023.md)：已完成、未完成、结果与限制。
- [当前任务R1](tasks/R1.md)、[历史任务S7-02](tasks/S7-02.md)。

当前共识：频繁、反向切换是正常运行现象。我们研究双阈值和最小保持在这种情况下对任务表现的作用，不以切换更少判断方法更好，不要求先证明切换有害。

## 研究目标与主要结果

- [项目目标](OVERVIEW.md)、[路线图](ROADMAP.md)、[实验计划](EXPERIMENT_PLAN.md)、[切换规则](SWITCH_CONTRACT.md)
- [原M1报告](evidence/S7-02/S7-02_EXECUTION_REPORT_20261023.md)、[M1索引](evidence/S7-02/RESULT_INDEX.md)、[原始记录清理限制](evidence/S7-02/DATA_RETENTION_20261023.md)
- [M3最终报告](evidence/S7-02/M3_ABCD_240_FINAL_REPORT_20261023.md)、[M3索引](evidence/S7-02/M3_ABCD_240_RESULT_INDEX.md)、[四组比较图](evidence/S7-02/M3_ABCD_COMPARISON.png)

## 地图和代表参数

- [M1冻结地图](designs/M1/library_v1/README.md)：30张开发图、30张原留出图。
- [M2当前18张开发图](designs/M2/selected_v2/README.md)。
- [M3当前30张挑战图](designs/M3/README.md)：历史筛选集合，不是新的独立留出集。
- [T1已完成任务](tasks/T1.md)、[退出阈值选择](designs/T1/EXIT_THRESHOLD_SELECTION.md)：B/D使用X=−0.10。
- [T2已完成任务](tasks/T2.md)、[保持时间选择](designs/T2/HOLD_TIME_SELECTION.md)：C/D使用H=0.20仿真秒。
- [S7-01准备任务](tasks/S7-01.md)：已完成，不再是Active Task。

## 已完成批次的操作参考

- [原M1执行说明](designs/S7/FORMAL_RUN_GUIDE.md)
- [M3执行说明](designs/S7/m3_challenge_240/RUN_GUIDE.md)

批次已完成，旧命令只作复现参考，不要再次派发。新实验需按下一阶段研究安排确定。

## 历史资料

- [2026-10-23文档整理归档](../../archive/markdown/20261023_research_replan/README.md)：旧提示词、旧咨询材料、长篇状态及整理前任务快照。
- [2026-10-18历史归档](../../archive/markdown/20261018/README.md)：早期任务、旧P1治理及历史提示词。

实验报告、选参依据和原始数据保留。旧路径的小型入口用于兼容已有链接与脚本元数据，不代表当前任务或运行授权。
