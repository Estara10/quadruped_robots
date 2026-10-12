# S5-01 配置来源、定向验证与构建证据

## 配置来源和运行绑定

共同 controller 安装前缀为 `quadruped_ros2_control_humble/install/rl_quadruped_controller`；overlay 的配置与模型为每次运行的独立实体副本。场景均为 `scene_random_04.xml`，两组 `scene_model_closure_sha256` 与 `runtime_model_fingerprint` 一致。

- B 实际 controller 日志 `../../../../logs/manual_ab_compare/20261011_163046_c9f3e0/B/[local run ID omitted]/ros_launch.log`：`candidate=hysteresis_switch group=B entry_threshold=-0.05 effective_exit_threshold=-0.08 hysteresis=true hold=false hold_s=0`。
- D 实际 controller 日志 `../../../../logs/manual_ab_compare/20261011_163046_c9f3e0/D/[local run ID omitted]/ros_launch.log`：`candidate=combined_switch group=D entry_threshold=-0.05 effective_exit_threshold=-0.08 hysteresis=true hold=true hold_s=0.3`。D policy frame 记录 `switching_mode=4`、X/H/保持起点/elapsed/status。
- 两组 `run_context.json` 均记录 `runtime_source_bound`、session、group、E/X、goal、config/model overlay 路径及模型/场景指纹；`runtime_record.jsonl` 是同 run 的周期证据。D 日志实际记录 hold-period exit 与 `RA<X`。

原始来源保存在 `logs/manual_ab_compare/20261011_163046_c9f3e0/`。

## 定向检查与构建

以下均为本轮定向命令/检查，不是全仓库套件：

- `s2_01_switch_config` C++ 目标：68 checks，0 failures；覆盖 D 配置合法/非法、E/X 边界、H 未到期/到期、时钟 veto、新会话重置、风险重复不重启等。
- `p1_07_switching` 既有回归目标：292 checks pass。
- `scripts/test_abs_rt_frame.py`：29 tests pass；D frame mode/hold 组合及错误组合。
- `scripts/test_abs_live_hud.py`：18 tests pass；D/E/X/H 显示。
- `scripts/test_s4_01_overlay_isolation.py`：D overlay 副本与配置隔离检查通过。
- `scripts/test_run_record.py -k d_hold_frame_payload_is_accepted`：1 passed。
- `scripts/test_s1_runtime_source.py -k d_group_binds_combined_threshold_and_hold_source_to_frame`：1 passed。
- `scripts/test_s4_01_hold_analysis.py`：包含 C/D 保持边界、风险和退出规则、缺周期/无效时钟及生产 `hold_status` 浮点边界样例，PASS。
- `python3 -m py_compile scripts/s1_analyze_run.py scripts/test_s4_01_hold_analysis.py`：通过。
- `colcon build --packages-select rl_quadruped_controller --cmake-args -DBUILD_TESTING=ON`：1 package finished；生成并安装 controller shared library 与 C++ 检查目标。

初次系统 Python 重分析遭遇系统 Matplotlib ABI 与 NumPy 2.2.6 不兼容；未安装或改动系统组件。改用任务既有绘图解释器 `configured plotting interpreter`，成功从相同的 B/D 原始记录重算指标、保持审计与 timeline。原始周期、终局和进程记录未编辑。
