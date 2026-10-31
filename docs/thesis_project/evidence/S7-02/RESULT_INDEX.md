# S7-02 正式结果索引

本索引保留原M1冻结测试集240项结果。当前M3 v1.4挑战批次的运行进度、暂停原因和数据位置见[M3批次索引](M3_ABCD_240_RESULT_INDEX.md)；两批结果分开报告。

> **数据可用性更新（2026-10-23）：**逐帧JSONL和ROS详细日志后来已清理；既有分析、台账、报告、图表、终局事实及部分gzip上下文/指标仍保留。现有结果可供复核，但不能完整重算时间序列。详见[数据精简说明](DATA_RETENTION_20261023.md)。

批次 `s7-abcd240-20261023` 于 2026-10-23 按冻结顺序完成原M1冻结测试集的240个计划项；总启动241次（包含S7-001首次GLFW基础设施失败和已批准的唯一替代）。分析器当时从原始记录重算，结果待Director审阅。S7-02仍ACTIVE。

- [通俗中文执行与结果报告](S7-02_EXECUTION_REPORT_20261023.md)
- [按组确认安全完成率图](S7-02_SAFE_COMPLETION_BY_GROUP.svg)
- [冻结分析摘要](../../../../logs/08_formal_experiment/S7_ABCD_240/s7-abcd240-20261023/analysis/formal_summary/SUMMARY.md)
- [计划项主结果（240 行）](../../../../logs/08_formal_experiment/S7_ABCD_240/s7-abcd240-20261023/analysis/formal_summary/primary_plan_items.csv)
- [全部实际启动可靠性账](../../../../logs/08_formal_experiment/S7_ABCD_240/s7-abcd240-20261023/analysis/formal_summary/all_actual_scene_start_counts.csv)
- [首次启动敏感性](../../../../logs/08_formal_experiment/S7_ABCD_240/s7-abcd240-20261023/analysis/formal_summary/first_scene_sensitivity.csv)
- [组间差值、地图簇区间及 UNKNOWN 情景界](../../../../logs/08_formal_experiment/S7_ABCD_240/s7-abcd240-20261023/analysis/formal_summary/comparisons.csv)
- [决策时钟重建敏感性](../../../../logs/08_formal_experiment/S7_ABCD_240/s7-abcd240-20261023/analysis/formal_summary/reconstruction_sensitivity.json)
- [完整机器可读汇总](../../../../logs/08_formal_experiment/S7_ABCD_240/s7-abcd240-20261023/analysis/formal_summary/formal_summary.json)
- [批次保留记录、attempt 与台账](../../../../logs/08_formal_experiment/S7_ABCD_240/s7-abcd240-20261023/)
- [先前暂停记录（保留历史）](EXECUTION_PAUSE_20261023.md)
