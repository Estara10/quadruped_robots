# 实验记录导航

2026-10-19按用途整理。优先打开下列六个编号目录；旧名字均为兼容符号链接，不是另一份数据。

## 六个主要目录

| 目录 | 内容 |
|---|---|
| [01_T1_exit_threshold](01_T1_exit_threshold/) | T1：退出阈值实验（含早期粗选） |
| [02_T2_minimum_hold](02_T2_minimum_hold/) | T2：保持时间功能检查、180项扫描、48项组合 |
| [03_map_validation](03_map_validation/) | 地图验证：M1地图库及地图试跑 |
| [04_B1_recording_validation](04_B1_recording_validation/) | B1：记录链、指标与收尾补强 |
| [05_manual_observations](05_manual_observations/) | 手动观察：场景观看、碰撞复核、单次运行 |
| [06_startup_and_control](06_startup_and_control/) | 启动与控制诊断：起立、下沉恢复、增益交接 |

## 常用实验入口

- [T1：400次扩大验证](01_T1_exit_threshold/b_exit_validation_400/)
- [T1：原M2第二轮144项](01_T1_exit_threshold/m2_exit_round2/)
- [T1：18图216项局部扫描](01_T1_exit_threshold/m2_local_scan_216/)
- [T1：已精简的两档36图历史结果](01_T1_exit_threshold/two_thresholds_36_maps/)
- [T2：4次功能试跑](02_T2_minimum_hold/hold_function_check/)
- [T2：180项保持扫描](02_T2_minimum_hold/hold_scan_180/)
- [T2：48项组合核验](02_T2_minimum_hold/combined_check_48/)

## 旧路径与兼容规则

| 原目录 | 新实体目录 |
|---|---|
| `logs/t1` | `logs/01_T1_exit_threshold` |
| `logs/t2` | `logs/02_T2_minimum_hold` |
| `logs/manual_m1_a` | `logs/03_map_validation/M1_A_library` |
| `logs/manual_a_map_survey` | `logs/03_map_validation/A_map_survey` |
| `logs/b1` | `logs/04_B1_recording_validation` |
| `logs/manual_collision_viewing` | `logs/05_manual_observations/collision_review` |
| `logs/manual_scene_runs` | `logs/05_manual_observations/scene_runs` |
| `logs/manual_single_run_01` | `logs/05_manual_observations/single_run_01` |
| `logs/manual_single_run_02` | `logs/05_manual_observations/single_run_02` |
| `logs/manual_s6_05` | `logs/06_startup_and_control/gain_transition_S6_05` |
| `logs/manual_startup_observation` | `logs/06_startup_and_control/startup_observation` |
| `logs/manual_startup_observation_4s` | `logs/06_startup_and_control/startup_observation_4s` |
| `logs/s6-03` | `logs/06_startup_and_control/preparation_S6_03` |
| `logs/s6-04` | `logs/06_startup_and_control/startup_recovery_S6_04` |
| `logs/s6_12` | `logs/01_T1_exit_threshold/legacy_coarse_S6_12` |

- 原始run_id、批次ID、JSONL、context、terminal及冻结计划内容没有修改。
- 旧路径通过符号链接继续可用；脚本无需批量改写，旧计划和报告中的绝对路径仍能定位。
- 请保留这些兼容链接；删除它们会使历史路径失效。符号链接几乎不占空间，不能把其目标再次计为一份数据。
- 如查看早期批次内使用相对路径的历史报告，可从其旧兼容入口打开；新总览优先使用本页及docs中的执行报告。
- 已按此前授权删除的manual_abcd_compare未恢复，目录内原始记录仍为已清理状态。
- 本次只整理目录，不删除实验数据，不运行仿真/测试，不提交Git。
