# 历史脚本归档（2026-11-03）

从 scripts/ 移出29个已完成的一次性工具、独立S7检查和历史批次脚本。未删除脚本；当前运行与依赖模块仍在原处。

## 使用边界

这里保留历史源码，不是当前运行入口。原脚本普遍依赖其位于 scripts/ 的位置来计算项目根目录和导入模块，不能直接从归档目录运行。需要复现时，按下表恢复到原路径，并一起恢复该批次依赖的归档模块；不覆盖当前同名文件。数据、模型和既有实验结果未移动。

历史文档中的运行命令保留当时写法；相关Markdown文件链接已改为归档位置。

## 文件清单

| 原位置 | 归档位置 | 类别 |
|---|---|---|
| `scripts/rename_m1_maps.py` | [one_off/rename_m1_maps.py](one_off/rename_m1_maps.py) | one_off |
| `scripts/generate_m1_stage2_samples.py` | [one_off/generate_m1_stage2_samples.py](one_off/generate_m1_stage2_samples.py) | one_off |
| `scripts/create_s6_10_relaxed_alternating_variant.py` | [one_off/create_s6_10_relaxed_alternating_variant.py](one_off/create_s6_10_relaxed_alternating_variant.py) | one_off |
| `scripts/create_s6_11_common_object_variants.py` | [one_off/create_s6_11_common_object_variants.py](one_off/create_s6_11_common_object_variants.py) | one_off |
| `scripts/create_stool_dev_scene.py` | [one_off/create_stool_dev_scene.py](one_off/create_stool_dev_scene.py) | one_off |
| `scripts/generate_project_report.py` | [one_off/generate_project_report.py](one_off/generate_project_report.py) | one_off |
| `scripts/reproduce_report.py` | [one_off/reproduce_report.py](one_off/reproduce_report.py) | one_off |
| `scripts/status_report.py` | [one_off/status_report.py](one_off/status_report.py) | one_off |
| `scripts/check_s7_attempt_accounting.py` | [legacy_checks/check_s7_attempt_accounting.py](legacy_checks/check_s7_attempt_accounting.py) | legacy_checks |
| `scripts/check_s7_cycle0_clock.py` | [legacy_checks/check_s7_cycle0_clock.py](legacy_checks/check_s7_cycle0_clock.py) | legacy_checks |
| `scripts/check_s7_dev_supplement.py` | [legacy_checks/check_s7_dev_supplement.py](legacy_checks/check_s7_dev_supplement.py) | legacy_checks |
| `scripts/check_s7_runner_readiness.py` | [legacy_checks/check_s7_runner_readiness.py](legacy_checks/check_s7_runner_readiness.py) | legacy_checks |
| `scripts/check_s7_safety_completion.py` | [legacy_checks/check_s7_safety_completion.py](legacy_checks/check_s7_safety_completion.py) | legacy_checks |
| `scripts/manual_s6_12_coarse.py` | [historical_batches/manual_s6_12_coarse.py](historical_batches/manual_s6_12_coarse.py) | historical_batches |
| `scripts/manual_t1_expanded_400.py` | [historical_batches/manual_t1_expanded_400.py](historical_batches/manual_t1_expanded_400.py) | historical_batches |
| `scripts/manual_t1_m2_round2.py` | [historical_batches/manual_t1_m2_round2.py](historical_batches/manual_t1_m2_round2.py) | historical_batches |
| `scripts/manual_t1_two_thresholds_36.py` | [historical_batches/manual_t1_two_thresholds_36.py](historical_batches/manual_t1_two_thresholds_36.py) | historical_batches |
| `scripts/manual_t1_m2_local_scan_216.py` | [historical_batches/manual_t1_m2_local_scan_216.py](historical_batches/manual_t1_m2_local_scan_216.py) | historical_batches |
| `scripts/manual_t2_hold_scan_180.py` | [historical_batches/manual_t2_hold_scan_180.py](historical_batches/manual_t2_hold_scan_180.py) | historical_batches |
| `scripts/analyze_t1_m2_local_scan_216.py` | [historical_batches/analyze_t1_m2_local_scan_216.py](historical_batches/analyze_t1_m2_local_scan_216.py) | historical_batches |
| `scripts/score_t1_expanded_400.py` | [historical_batches/score_t1_expanded_400.py](historical_batches/score_t1_expanded_400.py) | historical_batches |
| `scripts/score_t1_m2_round2.py` | [historical_batches/score_t1_m2_round2.py](historical_batches/score_t1_m2_round2.py) | historical_batches |
| `scripts/analyze_t2_hold_scan_180.py` | [historical_batches/analyze_t2_hold_scan_180.py](historical_batches/analyze_t2_hold_scan_180.py) | historical_batches |
| `scripts/analyze_t2_combined_check_48.py` | [historical_batches/analyze_t2_combined_check_48.py](historical_batches/analyze_t2_combined_check_48.py) | historical_batches |
| `scripts/test_s6_12_ray_repair_resume.py` | [historical_batches/test_s6_12_ray_repair_resume.py](historical_batches/test_s6_12_ray_repair_resume.py) | historical_batches |
| `scripts/test_t1_expanded_400.py` | [historical_batches/test_t1_expanded_400.py](historical_batches/test_t1_expanded_400.py) | historical_batches |
| `scripts/test_t1_m2_item73_repair.py` | [historical_batches/test_t1_m2_item73_repair.py](historical_batches/test_t1_m2_item73_repair.py) | historical_batches |
| `scripts/test_t1_m2_preparation_recovery.py` | [historical_batches/test_t1_m2_preparation_recovery.py](historical_batches/test_t1_m2_preparation_recovery.py) | historical_batches |
| `scripts/test_t1_m2_round2.py` | [historical_batches/test_t1_m2_round2.py](historical_batches/test_t1_m2_round2.py) | historical_batches |

## 保留的旧名称依赖

- `manual_t1_m2_smoke_18.py`、`manual_t2_hold_function_check.py`仍被R1批次脚本使用。
- `p1_10_common_start.py`仍被当前记录器导入。
- `s2_03_stop_publisher.py`、`s6_07_startup_protocol.py`仍参与运行和停止流程。
- 没有批量移动当前格式、记录、安全判定等模块的测试。

## 机械更新的文档链接

- `docs/thesis_project/evidence/S6-10/S6-10_EXECUTION_REPORT.md`
- `docs/thesis_project/evidence/S7-01/attempt_accounting_check/ATTEMPT_ACCOUNTING_REVIEW.md`
- `docs/thesis_project/evidence/T1/two_thresholds_36_maps/T1_TWO_THRESHOLDS_36_EXECUTION_REPORT.md`
