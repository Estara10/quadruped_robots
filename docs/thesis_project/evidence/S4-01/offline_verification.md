# S4-01 定向离线核验记录

日期：2026-10-11。仅覆盖本任务直接受影响路径。

- `quadruped_ros2_control_humble/build/rl_quadruped_controller/s2_01_switch_config`: 56 C++ parser/state-transition checks, 0 failures。
- `quadruped_ros2_control_humble/build/rl_quadruped_controller/p1_07_switching`: 292 A/B regression checks passed。
- `scripts/test_abs_rt_frame.py`: 29 tests passed，含 v2 exact-size compatibility 与 v3/C HUD cases。
- `scripts/test_run_record.py`: 54/54 passed。
- `scripts/test_s4_01_hold_analysis.py`: hold episode targeted cases passed。
- `scripts/test_s1_runtime_source.py`: source-binding checks passed。
- `scripts/test_s4_01_overlay_isolation.py`: actual `manual_ab_compare.prepare_overlay()` A/C copy isolation passed; source config through symlink remained byte-identical。
- `scripts/test_s4_01_diagnostic_config.py`: accepts equivalent `0.30`/`0.300`, rejects missing/non-finite/mismatched hold value。
- `scripts/test_s4_01_controller_graph.py`: one manager/controller passes; duplicate manager, duplicate controller, incomplete and empty direct graph outputs reject.
- Affected Python modules `py_compile` and `git diff --check` passed.
- Incremental controller target build passed. After discovery that the earlier A run loaded the old installed v2 library, `colcon build --packages-select rl_quadruped_controller --symlink-install` completed; installed library now matches the new build target by size and build timestamp.

The production `verify_config_consumption()` check accepted the actual C controller initialization log (group C, E=-0.05, effective exit=-0.05, hysteresis=false, hold=true, H=0.3, and expected model/config overlay paths). Regression test `test_current_c_initialization_log...` was added against that preserved log; `scripts/test_s1_runtime_source.py` passed. Evidence: `docs/thesis_project/evidence/S4-01/C_initialization_source_binding.json`.

The new A run's `/proc/<pid>/maps` directly records the expected installed controller library path. In the C attempt, the same library-mapping gate passed before the later duplicate-node uniqueness check, but the mapping was not persisted in that run context; therefore C's library mapping is inferred, not a separately preserved direct observation.

The former default daemon-backed node check reported duplicate `/controller_manager` and `/rl_quadruped_controller` names. Its preserved launch log has one `ros2_control_node` manager process and one controller activation, but no contemporaneous node-instance identity; historical cause (stale daemon entries versus graph discovery timing) remains unproven. The check was changed to use `ros2 node list --no-daemon --spin-time 2`, while retaining the default daemon response as diagnostic evidence. Exact duplicates remain a hard stop; process-group writer count and mapped library path are checked independently. In the authorized C run, direct and daemon queries each showed exactly one manager and one controller. The earlier `0.30` versus `0.3` startup failure remains under `logs/manual_ab_compare/20261011_145733_075bc8/C/` and is not relabeled.

The production C hold analyzer was run against the new preserved policy record: 19 entries and 19 exits; all observed exits met elapsed >=0.30 s and RA<-0.05; 22 expired high-risk cycles stayed in Recovery; five renewed risk events within Recovery retained the original hold start. Ten episodes have complete clock coverage and nine are incomplete because of cycle/clock gaps, so the aggregate audit remains `incomplete` with zero detected violations. This is runtime-record analysis, not a separate test harness.

The first post-run plotting invocation with `/usr/bin/python3` failed because its system Matplotlib extension is incompatible with installed NumPy 2.2.6. The same raw C run was analyzed successfully with the existing Anaconda Python (NumPy 2.1.3 / Matplotlib 3.10.0); the failure log remains preserved. No simulation was rerun.
