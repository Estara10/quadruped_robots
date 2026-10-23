# 本批数据精简说明（2026-10-23）

按用户要求保留必要记录，清理本批大体积文件。

- 保留：完整 analysis/、ledger/、批次配置与状态、执行报告、图表、terminal.json、process_facts.json、MuJoCo日志（含首次GLFW失败）。
- run_context.json、metrics.json、run_results.json 以无损 gzip 保存，文件名追加 .gz。
- 删除：runtime_record.jsonl、ros_launch.log、startup_gain_transition.jsonl，以及 overlay 中的模型和网格等资源副本；配置文件保留。项目源模型和源地图未改。
- 原正式结果不变：A 49/60、B 48/60、C 43/60（2 UNKNOWN）、D 47/60 确认安全完成。
- 原始逐帧数据已清理，不能再从原始帧完整重算或恢复运行；保留分析可用于查看既有结果。今后实验使用独立批次。
- 清理前 2095.5 MiB，清理后 231.6 MiB；释放约 1.82 GiB。

本轮未修改地图、未启动仿真、未提交Git。
