# S6-06 运行前核对

日期：2026-10-13。

- 当前权威状态无其他 Active Task；S6-05 已为 Director 有条件通过。S6-06 是唯一 Active Task。
- controller 当前构建与安装库的 GNU build ID 相同（`8aeaf2c42ff21e4c9ec8cf6671732b090543e821`）；安装配置中 `startup_gain_transition_s=0.30`。每个运行还会由诊断器核对 controller 进程实际映射的 library。
- MuJoCo 可执行文件 `unitree_mujoco/simulate/build2/unitree_mujoco` 存在；每次进程启动时会核对 `/proc/<pid>/exe` 与预期构建路径。
- 安装 overlay 的 ABS、RA、Recovery 三个模型文件可读。对生产 overlay 生成函数执行了 A/B/C/D 四组定向检查：配置解析、E/X/H/目标字段和四个配置/模型文件均为独立普通文件，不与源安装文件共享 inode。
- 本次启动前未发现本项目 MuJoCo、controller、ROS launch 或 RViz 遗留进程。wrapper 与诊断器会在每组启动时再次执行唯一 controller/writer 检查。
- 普通沙箱中的隐藏 GLFW 初始化失败；按本轮授权在沙箱外使用当前桌面环境运行 16×16 隐藏窗口探针，X/GLFW/OpenGL context 成功，Mesa 硬件渲染 OpenGL 4.6，窗口已关闭。未改授权文件、系统权限或桌面设置。
- 系统 Python 的 Matplotlib 与 NumPy ABI 不兼容；已有绘图解释器可成功导入 Matplotlib。批次将通过临时 `ABS_PLOT_PYTHON` 选择该解释器，并把 Matplotlib 缓存放在 `/tmp`。没有安装或升级软件。
- `manual_ab_compare.py` 的最小修正：记录标记更新为 S6-06；启用现有实际增益写出追踪；运行表增加有效时间覆盖、保持审计、实际增益轨迹、站立/PASSIVE与清理字段；启动前验证配置及三模型 overlay 为独立副本。Python 编译及 A/B/C/D overlay 生产函数定向检查通过。

上述是启动前证据；实际运行是否加载正确来源、周期0是否完整持久化、记录/分析/收尾是否有效，以每组本身的运行记录为准。


## 本批实际运行复核

2026-10-13 按 A/B/C/D 顺序各运行一次。四组 runtime source validation 均为 `confirmed`，controller 库映射和 MuJoCo 实际可执行文件均匹配预期；实际 group/E/X/H/candidate、模型配置 overlay、writer/session 和场景身份在各自 run_context/launch 证据中核对。每组 recorder 在 RL 前可读写；cycle 0 完整策略帧和 companion 关联通过；终止帧为同周期完整 payload。四组 `startup_recovery_observation_s=0.0`，使用普通跌倒规则。

每组均 `VALID / ARRIVED`，达到后完成约 5 仿真秒站立观察。组间没有改代码或参数。C 的保持审计虽没有规则违规，但 6/17 episode 的连续时钟覆盖不完整；B 任务窗有未确认跌倒候选，跌倒结论 UNKNOWN。单次诊断不能推出性能优劣。

逐组收尾事实：MuJoCo 和 ROS launch 均退出码 0、未强制终止；prepared stop publisher 退出码 1；closeout errors 为空且 `shutdown_complete`/`closeout_complete` 均为 true。批次后进程扫描未见本任务相关残留。结果与边界见 [执行报告](S6-06_EXECUTION_REPORT.md)。
