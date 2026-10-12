# S6-05 执行报告：四组共用的启动增益连续交接

**最新审阅状态：COMPLETE（Director 有条件通过，2026-10-12）。** 通过范围为四组共用的实际增益连续交接、0.30 控制运行秒到达策略目标、定向核验及构建、平地/场景04各一次普通 A 诊断和正常站立收尾。两次诊断均未启用低高度延期。

保留限制：准备期低高度事件、采样覆盖和初态差异仍在；单次结果不证明普遍改善或唯一因果；没有触发 Recovery，不能判断切换效果；接口写出不等于 MuJoCo 消费，实机未验证。关节目标及前一动作未修改；0.30 秒是工程增益交接时间，不是 Recovery 保持时间。以下历史执行细节、原始结果与限制保留。

## 结果摘要

- RL 入口先读取 12 个具名 command interface 当前实际 Kp/Kd，再以 `abs.startup_gain_transition_s: 0.30` 在控制器有效运行周期内线性过渡到策略目标。A/B/C/D 走同一逻辑；旧 `soft_start_steps` 作为冲突字段拒绝，不再叠加近零比例。停止/veto 路径仍优先。
- 两次运行的实际接口记录都显示 Kp 从 80 连续到 30、Kd 从 3 连续到 0.65：入口 `elapsed=0, alpha=0` 时仍写 80/3；每 0.005 秒 controller 周期推进；`elapsed=0.300 s, alpha=1` 时写到目标。该记录证明写入 controller command interface，不证明 MuJoCo 已消费。
- 平地与 `scene_random_04.xml` 各一次普通 A 诊断均 ARRIVED，并完成 5 仿真秒站立观察。两次都观察到准备阶段低高度 episode 已确认并后来关闭；RL 周期内保存快照没有 fall candidate/confirmed。场景04 RL 采样快照最低高度约 0.2346 m，低于平地约 0.2865 m。与 S6-04 的特殊观察 run（保存最低约 0.07598 m）相比，这两次读数看起来较轻，但初态、运行方式、覆盖和采样不同，无法据此认定普遍改善或增益交接是唯一原因。下沉风险没有被证明已消除。
- 两次运行没有 Recovery，因此不提供切换效果证据。

## 实现与来源证据

新增 `StartupGainTransition.hpp`，供生产状态类执行 12 通道有限值检查、入口捕获、线性插值、周期累计及精确终点；`StateRL::enter()` 在第一次 RL 写入前读取实际 loaned Kp/Kd，`StateRL::run()` 用 `period.seconds()` 推进，不以固定频率换算。`setCommand()` 写入每个关节的插值值，旧 `soft_start_step_` / `applied_gain_ratio_` 路径已移除。增益异常或无效周期进入既有安全拒绝路径。

配置位于 `quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml`，安装后的 overlay 副本为本次运行实际配置。运行日志显示 `startup_gain_transition_s=0.3`、共享 A/B/C/D、旧字段拒绝。两次 `run_context.json` 均记录 group A、E=-0.05、exit=-0.05、hysteresis=false、hold=false、goal=(7,0)、同一安装包下 Agile/RA/Recovery 模型路径及各自 controller PID/session；`ros_launch.log` 的 controller library mapping 指向本轮构建安装库。诊断器在启动前保留唯一 controller/writer、来源和采集器就绪门槛。

新增 controller-authored `[S6-05-GAIN]` JSON 事件，采集器提取为每次运行的 `startup_gain_transition.jsonl`，含 session、steady timestamp、joint names、entry/target/written Kp/Kd、elapsed、alpha 与 period。通道按 controller FR/FL/RR/RL 顺序逐项具名。采样同时记录旧 S6 trace；策略 q/动作链未修改。日志开关为 `--startup-gain-trace`，只控制诊断额外记录。

运行配置中的有效场景、目标和组别由 `runtime_config_observed`、scene binding、实际加载路径及 launch 日志核对。系统 GLFW 预检使用本机 `the active DISPLAY value` 与现有 XAUTHORITY；最小窗口检查得到 Mesa hardware renderer、OpenGL 4.6。仅使用本次命令的临时沙箱外桌面访问，没有修改系统授权、驱动或全局环境。

## 针对性验证与构建

- `s6_05_startup_gain_transition` 生产 helper 检查通过：12 通道入口连续、不同入口/目标、0/中间/到期/超期、非法时长/增益/period、数值相等配置写法、旧字段拒绝、新入口重置。
- `scripts/test_s6_05_gain_trace.py`：3 项解析检查通过；Python `py_compile` 通过。
- `s2_01_switch_config` 定向回归检查：68 checks、0 failures；A/B/C/D 原切换配置与 helper 路径未因增益过渡改变。
- 首次 controller 配置因未提供 LibTorch `Torch_DIR` 构建失败；使用本机已安装 LibTorch 路径重试后，`rl_quadruped_controller` 增量构建和安装通过。Go2 描述包也增量安装；随后确认源码及安装配置都为 0.30，旧配置键不再存在。
- 实际运行分别加载本轮安装的 `librl_quadruped_controller.so`。两个 trace 各有 62 行：entry 事件、60 次 0.005 秒 command update（elapsed 0 至 0.295 秒），以及 elapsed=0.300 秒的 transition-complete 写出。首周期 alpha=0 写 80/3，最后事件 alpha=1 写 30/0.65，所有 12 通道逐项一致。

## 两次普通诊断

共同设定：group A / `paper_faithful_switch`，E=-0.05、无滞回/保持，goal=(7,0)，0.5 m 到达半径，20 仿真秒/120 墙钟秒上限，正常跌倒/碰撞检查，不启用低高度延期；到达后观察站立 5 仿真秒。安装配置与模型位置见各自 `run_context.json`，无 overlay 回写源码/原安装目录的迹象。

| 场景 | Run ID / 原始记录 | 结果与运动记录 | 启动/跌倒及增益观察 | 收尾 |
|---|---|---|---|---|
| 平地 `scene_flat.xml` | `[local run ID omitted]`，run 目录（原始运行/环境明细保留本地，未纳入分享提交） | ARRIVED，终点距目标 0.4816 m，目标阈值 0.5 m；192 个策略步记录到 arrival；无 Recovery。策略记录 189 帧。 | capture 内有已确认且关闭的低高度 episode：sim 0.25 s 开始、0.55 s 确认、21.46 s 关闭，均早于首个有效策略帧约 23.96 s。策略帧关联物理快照中的 RL 区间最低保存高度约 0.2865 m；当前 candidate/confirmed 均未出现。接口增益入口 80/3，0.30 s 到 30/0.65。 | ARRIVED 后完成站立观察 5 仿真秒；进程事实 shutdown/closeout complete，相关进程事后核对无残留。 |
| 随机场景04 `scene_random_04.xml` / S2-08-r1 | `[local run ID omitted]`，run 目录（原始运行/环境明细保留本地，未纳入分享提交） | ARRIVED，终点距目标 0.4918 m；531 策略步到达记录，保存 530 帧；无 Recovery。 | 有已确认且关闭的准备期低高度 episode：sim 0.25 s 开始、0.55 s 确认、22.69 s 关闭，早于首个有效策略帧约 26.43 s。RL 区间最低保存高度约 0.2346 m；current candidate/confirmed 均未出现。接口增益入口 80/3，0.30 s 到 30/0.65。 | ARRIVED 后完成站立观察 5 仿真秒；进程事实 shutdown/closeout complete，相关进程事后核对无残留。 |

两组的 terminal `fall_events=true` 保留了准备期确认事件；这些 episode 在策略帧开始前已关闭。两组当前快照均无活动跌倒状态，不能把事件字段改写为 false，也不能将 preparation event 解释成 RL 期的 FALL_TERMINATION。碰撞字段只按各自 authority 声明的连续覆盖范围解释，不外推为全时域无碰撞。

## 启动下沉判断与剩余限制

此次测得的是“实际入口增益不再先降到近零，并按 0.30 controller 运行秒到达策略目标”。两次 run 都在策略期继续运动并到达；其策略期最低保存高度分别约 0.2865 m、0.2346 m，没有策略期 fall candidate/confirmed。相比 S6-04 特殊观察 run 的保存最低高度约 0.07598 m，观察结果较轻，但不是同初态、同观察模式或完整逐物理步高度对照，也只有各一 run。不能证明普遍改善、无触地、无跌倒或增益变化的单独因果；准备阶段真实低高度确认仍保留。

本轮不验证 MuJoCo 是否消费每次 command-interface Kp/Kd，不验证实机行为，也不做无观测开销对照。覆盖范围仍受记录器快照/运动窗口声明约束。关节目标仍按既有策略首批输出发生变化；本轮未做目标平滑。两次都未触发 Recovery，不能判断 A/B/C/D 切换效果。S6-04 的策略缺帧/STALE、模型动作顺序来源绑定、停止命令消费等历史限制不因本任务而消失。诊断时长 0.30 秒仅为工程交接时长，不是研究参数。

## 产物与环境

- 增益时序原始记录：每个 run 目录内 `startup_gain_transition.jsonl`。
- 配置、来源、初态、终局及清理：每个 run 的 `run_context.json`、`runtime_record.jsonl`、`terminal.json`、`process_facts.json`、`ros_launch.log` 和 `mujoco.log`。
- 代码配置改动：`StartupGainTransition.hpp`、`StateRL.h`、`StateRL.cpp`、controller `CMakeLists.txt`、Go2 ABS `config.yaml`、`scripts/s1_run_diagnostic.py`，以及新增的 `scripts/test_s6_05_gain_trace.py` 和 controller helper check。
- 两次运行后均核对本次 MuJoCo、controller/ROS、停止发布器及相关辅助进程退出；场景04结束时相关进程列表为空。临时 GLFW 探针已退出并删除。没有提交 Git。

结论：控制器实际增益交接已连续并由运行时接口日志直接支持；这两次普通诊断未在 RL 周期内出现确认跌倒且都到达，但 startup low-height 仍在准备期得到确认并保留。是否普遍减轻策略启动下沉尚不能定论。S6-05 已按 Director 有条件通过结论标记 COMPLETE；当前没有已授权的下一项 Active Task。
