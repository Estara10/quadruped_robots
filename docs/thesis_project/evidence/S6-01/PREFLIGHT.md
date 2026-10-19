# S6-01 运行前核对

日期：2026-10-12。所有场景运行均属于非正式流程核验。

## 定向补证：ROS 图发现、采样器和运行门槛

本节对应第二批 `logs/manual_abcd_compare/20261012_121911_451ec8/` 的 C/D 两次运行。首批 C 的 `INVALID` 原始记录和本文件下方的历史失败说明未更改。

### ROS graph 结果与来源

- 新诊断器先记录 daemon 结果，再仅以 fresh `ros2 node list --no-daemon --spin-time 2` 作为放行依据；若 direct 查询缺节点，则同环境间隔 0.5 s 最多重查 5 次，查询失败或同名重复立即停止。实际 C、D 都在 direct 首次尝试获得 manager/controller 各一个，分别约耗时 2.815 s、2.769 s，因此没有等待第二次重试；controller list 均是一条 `active` RL controller。
- C 的 daemon 和 direct 均各列一个节点。D 的 daemon 缓存各列两个同名 `/controller_manager`、`/rl_quadruped_controller`，但 fresh direct 各列一个；同组 launch 范围只有一个 `ros2_control_node`、一个 controller writer PID，运行帧 session/config 与该 PID 绑定。因此 D 的重复只见于 daemon 缓存视图，没有实际第二个活动 controller/writer 的证据。缓存没有用于放行。
- 每个 ROS 查询在 `run_context.json` 保存 start/end monotonic ns、持续时间、return code、输出、调用 PID/UID/executable/cmdline、selected 环境变量、ROS daemon process identity/RMW 映射及 direct/daemon 节点计数。历史首批 C 原日志没有这些字段，所以不能回溯判定它是具体何种等待或环境问题。此次有界 fresh 查询消除了当前放行歧义，但不能证明旧失败唯一原因。
- C/D graph 查询中 `ROS_DOMAIN_ID` 未设置，ROS 默认 domain 0；`ROS_LOCALHOST_ONLY=0`；`RMW_IMPLEMENTATION` 未设置，运行时 ROS daemon 命令行选择 `rmw_fastrtps_cpp`，launch controller `/proc/<pid>/maps` 也显示 `librmw_fastrtps_cpp.so`。fresh direct ROS CLI 的真实查询 PID 是 UID 1000 的 `/usr/bin/python3.10`，其 cmdline 为 `/usr/bin/python3 /opt/ros/humble/bin/ros2 node list ...`；`ros2` launcher 位于 `/opt/ros/humble/bin/ros2`。查询 shell 中 `python3` 是 Anaconda，只用于打印 shell 环境信息，不是实际 ROS CLI 解释器。
- 直接查询 Python PID 的短时 maps 样本只看见 `librmw_implementation.so`，没有捕获到选择后的插件映射；同环境 `/usr/bin/python3` 运行的额外 RMW identifier probe 返回 `rmw_fastrtps_cpp`。该 probe 在 sandbox 内提示 UDP socket `EPERM`，所以仅作为默认 RMW 选择的补充，实际图发现证据来自前述运行时查询、daemon 和 controller 映射。

### joint-state 解释器与初态边界

- C/D 在 PASSIVE/FIXEDSTAND 准备时，以 `/usr/bin/python3` 和 `/opt/ros/humble` 下 Python 3.10 `rclpy` 成功读到 12 个具名 Go2 joint position/velocity；完整数组、具名映射、header stamp、monotonic 时间和有限 ROS 环境字段保存在各组 `run_context.json`。Sampler 检查具名集合及数值有限性；失败会在 command 3 之前拒绝进入 RL。
- C 样本至命令发布 CLI 调用开始约 `147 ms`、至首个 active 策略帧 `1.935 s`；D 分别约 `141 ms`、`1.605 s`。运动位移锚点后的异步样本分别晚 `0.784 s`、`0.953 s`。这些均不是同步同周期初态，偏差已写入 `initial_state_and_sources.md`。
- A/B 首批的关节采样失败记录保留；不从 policy target 或 reset qpos 冒充实测关节状态。四组不能视为严格重放。

### 停止请求历史

- 首批 B 从正常 command 6 流程已到 `STANDING_HOLD`，但运动结束后 physics authority 由 LIVE 变为 MISSING，5 仿真秒 hold 未满足；之前 2.766 仿真秒观察里 controller 已达到站立确认且支撑增益保持。监督器因此使用 command 9 fallback。controller 日志确认切到 PASSIVE；该证据仅为 controller 状态，不是 MuJoCo 消费确认。B 原终局和历史记录均未改写，也未为此重跑 B。
- 第二批 C 站立观察完成，正常未发 command 9。D 正常 command 6 没获确认，随后 command 9 成功，controller 日志确认 PASSIVE；独立任务终局仍为 `ARRIVED`，站立收尾未完成。无运动期安全中止事件。D 原始 process cleanup event 中 `controller_state_before_stop=FIXEDSTAND` 是旧代码硬编码，和 controller 日志冲突；原始数据保留，以该 controller 日志记录 PASSIVE。日志事件生成逻辑已改为从停止证据读取状态。

### 实际构建/安装范围

本轮增补修改限于 Python supervisor、joint-state sampler、analyzer、comparison 工具和报告。没有变更 controller/C++ 模型、策略或研究运行参数，故无新的 controller 构建/安装。Python `py_compile` 与 ROS source-binding 定向检查通过。既有 `scripts/test_s1_05_safety.py` 运行失败于其 v4 binary fixture 长度断言（当前 production reader 已使用 v5）；未把这项旧 fixture 失败归因于本轮工具更改，也未改写该脚本或原始记录。

## 软件、配置与来源

- 控制器以 `colcon build --packages-select rl_quadruped_controller` 检查/构建；目标报告已是 up-to-date。A、B 和 C 的实际 launch 日志及进程映射指向同一安装库：`quadruped_ros2_control_humble/install/rl_quadruped_controller/lib/rl_quadruped_controller/librl_quadruped_controller.so`。相同 MuJoCo 可执行路径为 `unitree_mujoco/simulate/build2/unitree_mujoco`；A/B/C `process_facts.json` 记录的 `/proc/<pid>/exe` 与该路径相符。A、B runtime source binding 与 session/writer/controller PID 相符。C 在初始化日志中出现 group=C/H=0.30 及模型加载行，但没有 RL 策略帧，因此只证明初始化日志，不证明策略周期消费。
- 系统安装库大小为 2,221,976 字节，2026-10-11 16:27；MuJoCo executable 大小为 4,259,976 字节，2026-10-11 17:10。构建输出称 controller 目标已是 up-to-date。此处记录了实际加载路径，不声称不同 build/install 库字节完全相同。
- 四组 overlay 位于本批次 `logs/manual_abcd_compare/20261012_101309_772306/overlays/{A,B,C,D}`。overlay 根、配置文件和模型文件均为普通实体文件，不是符号链接；A/B/C/D 的 `abs/config.yaml` 因组参数不同而不同，`robot_control.yaml` 及 Agile、RA、Recovery 权重字节与 source description 对应文件相同。A/B/C 实际 launch 使用各自 overlay package_share；D overlay 已准备但没有启动消费。源码配置 SHA-256：`config/abs/config.yaml`=`5ba616527dbc214cefc2a83feb7c396e4d0f2b0d3ad4e54ed533bea029170b7d`，`config/robot_control.yaml`=`59c61ad4f29c2b37b3741236b35ce11c773fb6cfe16ed8d58b92757febf8bf7c`；运行前后 `abs/config.yaml` hash 一致。配置/权重没有写回 source。
- 每个实际运行都有独立 session、run_id 和 writer/controller 绑定；A/B 来源校验确认 `all_source_events_match`。A writer/controller PID [omitted]、session 1719084890764；B PID [omitted]、session 1776891406391。C 的启动流程因直接 graph 唯一性查询失败而停止。

## 显示预检

- 当前桌面 `the active DISPLAY value`、`XAUTHORITY=current desktop-session authorization file`；授权文件可读，desktop-session UID。默认 sandbox 内 X11 连接被拒；按已有授权进行沙箱外最小预检后，X 连接与 GLFW 隐藏窗口 OpenGL context 成功。
- GLFW 报告 Mesa hardware renderer、OpenGL `4.6 (Compatibility Profile) Mesa 23.x`。未修改桌面授权、驱动、系统服务或全局环境。
- A/B 运行面板来自实际 MuJoCo framebuffer 捕获，分辨率 1280×720。真实画面文件：[`A_live_framebuffer.png`](screenshots/A_live_framebuffer.png)、[`B_live_framebuffer.png`](screenshots/B_live_framebuffer.png)；实际 PPM 亦保存在同目录。到达后站立画面为 INVALID/无当前策略数据并隐藏旧 RA/模式值：[`A_standing_framebuffer.png`](screenshots/A_standing_framebuffer.png)、[`B_standing_framebuffer.png`](screenshots/B_standing_framebuffer.png)。画面清晰可读；截图对应的 run_id 见 [`group_results.md`](group_results.md)。
- 系统 Python 的 Matplotlib/Numpy ABI 不兼容，未用于生成图；调用链使用脚本既有可用绘图解释器。没有因为截图或渲染缺陷追加场景运行。

## 首批 C 停止规则触发（历史，之后已获新授权补证）

C run `[local run ID omitted]` 中，`ros2 node list --no-daemon --spin-time 2` 返回 `/controller_manager` 与 `/rl_quadruped_controller` 均未发现；daemon node list 同时列出两节点，`ros2 control list_controllers` 显示 joint_state_broadcaster、imu_sensor_broadcaster、rl_quadruped_controller 均 active。run wrapper 的“直接查询必须唯一”门槛因此失败，C 被保存为 `INVALID / STARTUP_FAILURE`，0 policy frames，并完成清理。证据不能判定是 graph discovery 延迟、查询差异或节点实例问题；不能仅凭该矛盾称已证明唯一 controller。controller manager 在受控 SIGINT 收尾时由 launch 日志记为 exit -2，批次 wrapper 的 run exit code 为 0，且 closeout complete、无进程残留。为遵守任务停止规则，D 未启动，也没有重跑 C。
