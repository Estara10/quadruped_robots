| run_id | record_validity | terminal_result | frame_count | rl_step_gaps | invalid_clock_frames | policy_frequency_hz_overall | mode_transitions | recovery_share_valid_motion_time | target_distance_final_m | mean_path_speed_mps |
|---|---|---|---|---|---|---|---|---|---|---|
| 22e64352d7c042708b96608a16a89221 | VALID | ARRIVED | 75 | 0 | 2 | 51.2443 | 0 | 0.0000 | 0.5107 | 0.8906 |
| cb47356b98104e40b258b81a8246db17 | VALID | ARRIVED | 75 | 0 | 1 | 51.7758 | 0 | 0.0000 | 0.4843 | 0.9331 |

终局边界核对：`cb47356b98104e40b258b81a8246db17` 的 ARRIVED 周期 74 已在 `runtime_record.jsonl` 中。`22e64352d7c042708b96608a16a89221` 的 `run_context.json` 直接记录 ARRIVED 于 session `20359911390477`、周期 75、`sim_time=24.338 s`、距离 `0.484716 m`，但采集文件最后只保存到周期 74、距离 `0.510698 m`。因此第二行的表格终点距离和路径速度截至最后已保存周期，终止周期的完整策略载荷缺失；该 run 的运动指标边界不完整，不能据此声称两次都满足合同的含终止周期指标要求。此注释不改写原始帧、context 或 terminal。
