| scene | seed | obstacle_count | terminal | goal_distance_final_m | motion_elapsed_start_to_terminal_s | motion_effective_valid_contiguous_time_s | valid_contiguous_segment_path_length_m | valid_contiguous_segment_mean_speed_mps | agile_valid_motion_cycles | recovery_valid_motion_cycles | mode_switches_motion_all_cycles | agile_to_recovery_motion_all_cycles | recovery_to_agile_motion_all_cycles | mode_switches_motion_with_valid_sim_time | mode_switches_motion_invalid_sim_time | short_reverse_switch_count_existing_metric | recovery_time_s_contiguous | ra_min_motion | ra_max_motion | foot_contact_count | foot_impact_count | nonfoot_collision | fall | policy_frequency_hz_full_capture | policy_interval_ms_median | policy_interval_ms_p95 | invalid_clock_frames_full_capture | policy_cycle_gaps | passive_controller_confirmed | process_shutdown_complete |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| scene_random_01.xml | 3101 | 15 | ARRIVED | 0.4772 | 3.3000 | 3.2600 | 6.4029 | 1.9641 | 165 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0.0000 | -0.9996 | -0.7139 | 0 | 0 | False | False | 49.9886 | 19.9728 | 24.5136 | 1 | 0 | True | True |
| scene_random_02.xml | 3102 | 17 | ARRIVED | 0.4799 | 3.4600 | 3.3800 | 6.3583 | 1.8811 | 172 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0.0000 | -0.9994 | -0.4535 | 0 | 0 | False | False | 50.0326 | 19.9936 | 28.0771 | 2 | 0 | True | True |
| scene_random_03.xml | 3103 | 18 | ARRIVED | 0.4908 | 3.4240 | 3.4240 | 6.6104 | 1.9306 | 172 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0.0000 | -0.9995 | -0.6924 | 0 | 0 | False | False | 49.9648 | 19.9866 | 26.0438 | 0 | 0 | True | True |
| scene_random_04.xml | 3104 | 19 | ARRIVED | 0.4874 | 11.7600 | 11.1080 | 7.7485 | 0.6976 | 407 | 167 | 136 | 68 | 68 | 128 | 8 | 117 | 3.3180 | -0.9984 | 0.2985 | 0 | 0 | False | UNKNOWN | 50.0126 | 19.9612 | 30.7328 | 15 | 0 | True | True |
| scene_random_05.xml | 3105 | 20 | ARRIVED | 0.4524 | 3.5300 | 3.2820 | 6.0780 | 1.8519 | 171 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0.0000 | -0.9994 | -0.5060 | 0 | 0 | False | False | 49.8107 | 20.0264 | 28.6985 | 6 | 0 | True | True |

时间与路径口径：`motion_elapsed_start_to_terminal_s` 是运动窗口起止边界仿真时间之差；
`motion_effective_valid_contiguous_time_s` 是分析器累计的有效连续仿真间隔，缺失/无效时钟处不桥接。
路径长度与平均速度仅覆盖有效连续片段，不能解释为完整起终点路径或全程平均速度。
模式变化按 `start_cycle..end_cycle`（含边界）统计，保留无效时间周期中已经记录的变化；
`mode_switches_motion_with_valid_sim_time` 只统计具有有效仿真时间的变化，不能用于补计无效区间的精确驻留时间。
`short_reverse_switch_count_existing_metric` 是分析器在有效时钟运动窗口中，对相邻且方向相反、间隔不超过 0.5 s 的转换对计数；
它不包含时间无效的转换，也不代表全部模式变化数。
