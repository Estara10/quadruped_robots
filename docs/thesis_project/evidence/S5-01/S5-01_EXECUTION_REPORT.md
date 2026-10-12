# S5-01 执行报告：D 组组合机制与 B/D 功能诊断

> **最新状态：COMPLETE — Director 有条件通过（2026-10-11）。** 以下保留执行时的完整记录与历史事实；单次诊断不构成正式实验或总体性能比较。

## Director 审阅结论

通过范围：D 生产配置实际消费；滞回与最小保持组合生效；定向验证、构建及同代码 B/D 各一次功能诊断；到达后站立观察和环境清理。

保留限制：16 个 D Recovery episode 中 15 个覆盖完整且符合规则，1 个因周期 196 时钟 STALE 而不完整；整体保持审计仍为 `incomplete`，缺失周期未获验证。单次结果不能证明性能优劣或成功率；起始关节实测向量未保存；D 跌倒为 UNKNOWN，碰撞否定结论受覆盖范围限制；有效 D 实时面板截图未取得；参数尚未正式冻结；MuJoCo 命令消费等既有 UNKNOWN 保留。分析器浮点边界修正说明、历史失败及原始运行记录均保留。

## 完成范围

实现 D 组配置与组合状态机：`E=-0.05` 进入、`X=-0.08` 退出、`H=0.30` 仿真秒最小保持。D 在保持到期前留在 Recovery；到期后只有 `RA < X` 才退出；`RA == X` 继续 Recovery；Recovery 中风险再次升高不重启保持。C++ 状态机、配置解析、帧模式值、Python reader、运行记录、实时 HUD、来源匹配和分析审计均已接入 D；S4-01/A/B/C 规则保留。

研究参数仅为本任务诊断值，不是正式论文参数。D 使用既有 v3 帧格式和 mode=4，不增加帧布局版本。

## 定向验证与构建

- C++ 定向状态机/配置目标：`S5-01 config/switch checks: 68 checks, 0 failures`；既有 `p1_07_switching` 回归目标：292 项通过。
- Python 定向验证：帧 reader 29 项、HUD 18 项、overlay 隔离、D 配置/来源绑定、D hold 分析边界测试均通过；`run_record` D 帧接受检查通过。
- 本轮复核运行记录后发现分析器曾以 `elapsed + 1e-6 >= H` 判断到期，与生产 C++ 的 `elapsed >= H` 不一致。对分析器做了最小修正：按生产帧 `hold_status` 判断到期，并增加浮点边界“0.2999999999998 秒仍未到期”和“生产标记到期但 RA<X 却未退出”的定向样例。样例和 Python 编译检查通过。用 `configured plotting interpreter` 对原 B/D 数据重跑分析和时间线；系统 Python 的 Matplotlib 与 NumPy 2.2.6 ABI 不兼容，未更改环境或安装组件。
- `rl_quadruped_controller` 增量构建成功，含 `StateRL.cpp`、controller shared library 和定向 C++ 目标；安装产物位于 `quadruped_ros2_control_humble/install/rl_quadruped_controller/`。两次运行实际日志进一步显示 D 新配置、D frame mode 与保持字段已消费。

## B/D 单次运行

共同条件：`scene_random_04.xml` / `random_04`，同一场景闭包与模型指纹，目标 `(7,0)`、到达半径 `0.5 m`、运动仿真上限 `20 s`、到达后站立观察 `5 s`；独立 overlay 和 run/session。B、D 均由本轮同一 controller 安装、同一模型和同一初态流程运行。来源日志与 `run_context.json` 显示本次 controller config/model 位于各自 overlay。记录没有保存起始关节的实测状态向量，不能声称两次初态数值完全相同或属于严格配对重放。

| 指标 | B | D |
|---|---:|---:|
| run_id | `[local run ID omitted]` | `[local run ID omitted]` |
| 配置消费 | `group=B, E=-0.05, X=-0.08, hysteresis=true, hold=false` | `group=D, E=-0.05, X=-0.08, hysteresis=true, hold=true, H=0.30` |
| 记录 / 终局 | VALID / ARRIVED | VALID / ARRIVED |
| 到达距离目标 | 0.4781 m | 0.4693 m |
| 起点至终止周期经过时间 | 9.340 s | 15.296 s |
| 有效连续仿真间隔累计 | 9.234 s | 15.214 s |
| 有效连续片段路径 / 平均速度 | 7.960 m / 0.862 m/s | 10.162 m / 0.668 m/s |
| Agile / Recovery 有效时钟周期数 | 333 / 132 | 478 / 286 |
| 模式切换（总数；有有效 sim_time） | 69；69 | 32；32 |
| Recovery 有效连续驻留累计 | 2.682 s | 5.710 s |
| RA 范围（运动周期） | [-0.9993, 0.3126] | [-0.9989, 0.3292] |
| 0.5 秒窗口短反向切换数 | 64 | 24 |
| 无效时钟帧 / 策略周期频率 | 3 / 50.233 Hz | 2 / 50.159 Hz |
| 足端接触/撞击、非足端碰撞 | 0 / 0，未记录非足端终止碰撞 | 0 / 0，未记录非足端终止碰撞 |
| 运动期跌倒判断 | false（按本次分析规则） | UNKNOWN（有未确认姿态候选） |
| 站立观察与清理 | 5 秒观察完成；相关进程退出 | 5 秒观察完成；相关进程退出 |

频率由同 session 连续策略帧的单调时间间隔计算；上表为全段统计，约 50 Hz。无效时钟帧没有删除模式变化计数；两次所有已计模式边沿均有有效仿真时间。起终点经过时间与有效连续累计时间分开，路径/速度仅由有效连续片段计算。运行每组一次，以上差异不证明 D 优于 B，也不构成成功率或统计结论。

## D 保持与滞回审计

D 运动窗口记录 16 次 Agile→Recovery 和 16 次 Recovery→Agile。修正分析器并在同一原始数据上重算后，16 个 Recovery episode 中 15 个时间覆盖完整、1 个覆盖不完整；可判定的 episode 均满足保持和 `RA<X` 退出规则，共 0 项可判定规则违规。唯一不完整 episode 为 `enter_cycle=147` 到 `exit_cycle=197`：周期 196 的 sim clock 状态为 STALE（status=2），因此记录的 1.002 s 不能当作完整连续驻留。逐 episode 概要及周期边界见 [hold/hysteresis audit](hold_hysteresis_audit.md)。

直接帧证据例：周期 69 以 `sim_time=24.834 s` 进入；周期 84 的 `hold_elapsed=0.2999999999998 s`、`hold_status=active`、RA<X，因此继续 Recovery；周期 85 `hold_elapsed=0.320 s`、`hold_status=expired` 且 RA<X，因此退出。原审计器的 1 微秒容差曾把周期 84 误标为违规，已纠正分析口径；原始运行帧未改写。

## 原始记录、分析产物与截图

- 根目录：`logs/manual_ab_compare/20261011_163046_c9f3e0/`
- 配置消费、运行来源、定向命令与构建证据见 [verification.md](verification.md)。
- B 原始数据：`B/[local run ID omitted]/`；D 原始数据：`D/[local run ID omitted]/`。各目录保留 `runtime_record.jsonl`、`run_context.json`、`terminal.json`、`process_facts.json`、ROS/MuJoCo 日志、指标及 timeline。
- 更新后的逐组分析：`B/analysis/run_results.md`、`D/analysis/run_results.md`；五项指标并列对照见 [comparison.md](comparison.md)。
- D 运行期 X11 截图尝试因 `xwd -root` 报 `BadColor (invalid Colormap parameter)` 失败，留下的 `d_policy_live.xwd` 为 0 字节，**不是有效截图**。没有取得真实 D LIVE 面板截图。D 的 `standing_hold.ppm` 是到达后的站立画面，不作为 LIVE D 面板证据。

## 文件修改范围

- 生产切换/配置：`RASwitchingLogic.hpp`、`AbsSwitchConfig.hpp`、`StateRL.cpp`、`StateRL.h`。
- 周期帧、reader、记录、HUD、运行来源/overlay：`common/abs_rt_frame_contract.h`、`scripts/abs_rt_frame.py`、`scripts/run_record.py`、`scripts/abs_live_hud.py`、`scripts/s1_run_diagnostic.py`、`scripts/manual_ab_compare.py`。
- 分析及定向检查：`scripts/s1_analyze_run.py`、相应 D 组 config/switch/frame/HUD/record/source/overlay/hold 测试脚本。
- 文档：`docs/thesis_project/SWITCH_CONTRACT.md`、本报告、`comparison.md`、`hold_hysteresis_audit.md`、S5-01 任务记录和 `CURRENT_STATE.md`。

## 安全结果、限制与清理

两组均以 `ARRIVED` 结束，距离目标均小于 0.5 m；均完成 FIXEDSTAND 站立观察。运动分析未记录足端接触/撞击或非足端碰撞事件，但碰撞无事件覆盖不完整，不能概括为“全程无碰撞”。B 按诊断规则无运动期确认跌倒；D 有运动窗口未确认姿态候选，跌倒为 UNKNOWN。D 保持整体 audit 为 `incomplete`，不是完整证明所有保持区间。没有采到有效 D 运行期面板截图。MuJoCo 命令消费等既有 UNKNOWN 仍保留。

每次 run 的 `process_facts.json` 记有正常终止、站立观察完成、`shutdown_complete=true`、`closeout_complete=true`、无强制结束和无 closeout error。两次结束后沙箱外进程核对未发现本次 MuJoCo、controller/ROS launch、RViz、recorder、stop publisher 或辅助进程残留；只有核对命令自身匹配行。桌面/OpenGL 预检窗口也已关闭。

**执行结论：**D 配置、组合状态机和保持/滞回运行链已实际接入；本次 D 有 15 个覆盖完整的 episode 符合规则，另 1 个仍不完整。B/D 各一次均 VALID / ARRIVED，但不支持优劣或性能判断。Director 有条件通过后，S5-01 标记 COMPLETE；不表示参数冻结、性能结论或正式实验完成。
