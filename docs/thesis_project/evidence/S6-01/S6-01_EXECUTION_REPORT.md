# S6-01 执行报告：四组预实验流程与指标可用性核验

**最新状态：COMPLETE — Director 有条件通过（2026-10-12）。**

Director 接受的范围：四组流程可执行、实际配置与来源可核对、真实 LIVE 面板证据、具名关节采集可工作，以及指标定义和可用性评估。最新四组结果见[结果与初态对照](group_results.md)，指标状态见[指标可用性清单](METRIC_AVAILABILITY.md)，关节来源和样本时间见[初态与来源记录](initial_state_and_sources.md)。这不代表严格初态配对、正常收尾全部通过、安全指标完整可用、参数选择完成或性能结论成立。

## 执行结论

经过本次定向补证，A、B、C、D 各有一条有效 `VALID / ARRIVED` 流程运行；C 与 D 的 controller、writer、session、实际配置和模型来源均在运行中确认。首批 C 的 `INVALID / STARTUP_FAILURE` 与 D 未启动的历史仍完整保留，本次 C 是单独新运行，不覆盖失败记录。本批 A/B 与补证 C/D 属于两批流程核验，不构成严格初态配对；每组仅一次，不得据此比较性能、成功率或参数优劣，且全部排除正式实验统计。

C/D 运行前新增 ROS graph 查询时间与实际进程身份记录，使用同环境 fresh direct 查询，有界等待，并保留重复节点拒绝。D 的 daemon 缓存显示同名节点各两条，但 direct graph、controller list、launch 范围进程与 writer/session 均只证实一个活动 controller/writer；缓存不作为放行依据。C/D 在 command 3 前取得 `/usr/bin/python3` 的 12 个具名关节位置/速度样本，同时保存其与 RL 活跃帧、运动锚点的时间偏差，未冒充同周期初态。

C 完成 5 秒站立观察；D 虽 `ARRIVED` 且数据有效，但正常 command 6 未确认，command 9 后 controller 确认 PASSIVE，站立观察未完成。A 完成站立观察；B 的历史站立观察未完成，同样使用 command 9。运动期安全事件与终局后停止分别报告。无碰撞与 C 跌倒结论仍受覆盖限制。

当前指标同时保留旧的 `0.10 m` 位移起点，并新增首个 `rl_entered && rl_active` 策略周期起点。前者会排除策略活跃后的启动周期；新旧边界不混写。风险到模式决定的纳秒差只表示内部决策耗时，不是完整避障响应或动作生效延迟。

## 首批执行记录（历史，未被改判）

以下记录的是补证前的首批状态。A/B 有效；C 因旧的一次 direct/daemon 不一致检查留下 `INVALID / STARTUP_FAILURE`；按当时停止规则 D 没有启动。补证之后第二批 C/D 的状态见本报告顶部及后文。

首批的 A、B 分别一次 `VALID / ARRIVED`，并保存 LIVE/站立画面。首批 C 的直接 ROS graph 唯一性检查与 daemon graph/controller list 结果不一致，0 policy frame 后被保存为 INVALID；首批 D 未运行。这一首批结论不等于目前 C/D 补证仍未完成，也没有改写首批原始数据。

已完成定向面板核验、必要构建与来源核查；扩展记录了 MuJoCo executable 路径、reset qpos0、运动起点策略帧锚点和姿态样本、joint-state 采集状态。A/B 关节 position/velocity 没有成功样本，故初态证据仍不完整。

## 首批改动与核验（历史）

- `scripts/s1_analyze_run.py`：运动周期窗口由同 session 的运动起点和 terminal 周期界定，mode 边沿数不再因该帧 sim_time 无效而丢弃；连续时间、驻留和位姿差分仍仅使用有效连续时钟。增加周期内 Agile/Recovery 数、invalid clock/gap 数、完整经过时间、有效区间峰值路径速度、短反向覆盖状态及风险到同周期 A→R 决策延迟配对。
- `scripts/manual_ab_compare.py`：汇总表增加全部/有效时间切换数、短反向覆盖、模式周期数、策略频率、风险配对/延迟和路径/时间指标。
- `scripts/s1_run_diagnostic.py`：保存实际 MuJoCo `/proc/<pid>/exe` 路径、scene reset probe、运动起点周期/位姿锚点、碰撞 authority 姿态样本，并尝试异步捕获关节状态。新增真实 MuJoCo framebuffer 文件捕获。
- 新增 `scripts/capture_joint_state_once.py`，采集具名 `/joint_states` 的 position/velocity。
- `unitree_mujoco/simulate/test/s2_02_live_panel_test.cpp`：针对 A/B/C/D 显示、C/D 保持状态和策略停止后隐藏旧策略值增加定向案例。首次 fixture 因缺 A 组字段失败，补正后测试通过；`cmake --build build2 --target s2_02_live_panel_test unitree_mujoco -j2` 成功。
- Python 语法检查通过：`scripts/s1_run_diagnostic.py`、`scripts/capture_joint_state_once.py`、`scripts/s1_analyze_run.py`、`scripts/manual_ab_compare.py`。
- 系统 GLFW/OpenGL 预检成功：Mesa hardware renderer，OpenGL 4.6 Compatibility Mesa 23.x。A/B 的真实 1280×720 LIVE 及站立 framebuffer 图见 [截图目录](screenshots/)。

### 关节初态采集缺口

采集器第一次调用登录 shell 的 `python3`，落到 Anaconda Python 3.13，而 ROS Humble `rclpy` 扩展是 Python 3.10；错误为找不到 `_rclpy_pybind11`。A/B 的运行上下文保存了该失败，未以目标关节值代替。工具随后固定使用 `/usr/bin/python3` 并经 ROS 环境导入检查，但本批没有再运行采样，因此实际运动初始关节角/速度仍 UNKNOWN。准确 reset qpos0 与策略周期位姿锚点见 [初态与来源对照](group_results.md) 和 reset probe 输出（原始运行/环境明细保留本地，未纳入分享提交）。

## 首批四组结果（历史；后续 C/D 补证见本报告下方）

完整表格、组配置、初态、来源、事件和证据路径见 [group_results.md](group_results.md)。简表：

| 组 | 状态 | 终局 | 关键观察 |
|---|---|---|---|
| A | VALID | ARRIVED，距目标 0.4808 m | 322 个模式边沿；Recovery 6.164 s（有效时间 15.682 s 的 39.31%）；有 RL 足端接触/撞击；运动中存在未确认姿态候选。 |
| B | VALID | ARRIVED，距目标 0.4910 m | 102 个模式边沿；Recovery 3.268 s（有效时间 9.980 s 的 32.75%）；有 RL 足端接触/撞击；5 s 站立观察未完成，hard-stop 后 controller 确认 PASSIVE。 |
| C | INVALID（首批历史状态） | STARTUP_FAILURE，0 policy frame | ROS 直接 node query 与 daemon graph/controller list 不一致，唯一性 gate 停止；未进入 RL。 |
| D | NOT STARTED（首批历史状态） | UNKNOWN | C 的基础设施失败触发停止规则，未启动 D。 |

A/B 两次都是到达任务，但每组只有一次，初态并非严格相同，不据此推断成功率、性能排名或参数优劣。A 的 4 个无效时钟帧未抹除两个 mode 边沿，但相关时间边沿不可精确计时。A/B 各记录 RL 足端 `ppt_obstacle_15` contact 与 impact；non-foot authority 没报告终止碰撞，但运动窗否定覆盖不足，不能声称无碰撞。A 的姿态候选未确认；B 无窗口内记录的姿态事件，不构成正式跌倒率结论。

## 首批指标可用性判断（历史；最新评估见独立清单）

逐项计算定义、证据来源、覆盖条件及后续影响见 [METRIC_AVAILABILITY.md](METRIC_AVAILABILITY.md)。当前可依赖 A/B 有效终局、距离、运动起止周期、有效策略帧频率、mode 边沿和已观测足端事件。有效连续时间和有效片段路径/速度可作带覆盖说明的描述。A 的短反向与 Recovery 时间指标受无效时钟影响；碰撞否定、A 未跌倒否定、初态关节向量、C/D 运动指标不可用。风险响应值是状态决定事件的 monotonic 时间差，不是控制命令写出或 MuJoCo 消费响应。

## 首批环境收尾（历史）

A、B 正常关闭 MuJoCo 后关闭 ROS/controller 等任务进程。B 的普通停止确认未完成，记录了 hard-stop command 9；controller 日志确认状态切至 PASSIVE，之后 MuJoCo、ROS/controller 与 publisher 均退出。C 在启动失败后 wrapper 关闭本次 MuJoCo/launch。每个已启动 run 的 `process_facts.json` 为 `closeout_complete=true`，退出码 0；最后按进程名核对 `unitree_mujoco`、`ros2_control_node`、`rviz2`、项目 `ros2 launch`、采集和停止发布进程，没有发现本任务残留。截图预检窗口已退出。没有关闭无关会话。

## 首批报告当时的 Remaining UNKNOWN（历史状态，最新进展见下方补证）

1. **C 的节点查询分歧**：无法据现有证据决定是 ROS graph 发现延迟还是别的查询/实例问题。后续先修复并定向验证唯一 controller 检查，证明同一个 manager/controller 的直接 query 可重复确认，再由 Director 决定是否允许单次 C/D 补证；不在本任务中重跑。
2. **运动起点关节 position/velocity**：当前 A/B 无采样值。后续在 `/usr/bin/python3` + ROS 环境下单独验证一次 joint-state subscriber 后，再纳入新授权运行上下文。
3. **碰撞否定与跌倒否定**：A/B 不能因未出现 non-foot termination 就判无碰撞；A 有未确认姿态候选，B 的无事件不验证正式跌倒阈值。正式预实验前确认运动窗口的 authority 覆盖和跌倒参数适用性。
4. **控制命令消费**：B controller PASSIVE 确认不等于 MuJoCo 已消费命令；此限制不影响本次终局记录，但不可做物理停稳断言。
5. **实验外推**：一组一次、同一场景的流程核验不能估成功率、算法优劣、参数最优或正式指标方差。

## 首批证据索引（历史批次）

- [预检和停止原因](PREFLIGHT.md)
- [A/B/C/D 来源、初态、结果、周期/截图索引](group_results.md)
- [指标可用性清单](METRIC_AVAILABILITY.md)
- MuJoCo reset probe 原始文本（原始运行/环境明细保留本地，未纳入分享提交）
- 首批原始记录：`logs/manual_abcd_compare/20261012_101309_772306/`
- A/B raw session、周期记录、terminal、过程事实和图片均留在各自 run 子目录；首批 C 的 INVALID 原始记录保留，首批未创建 D run。

首批报告提交时的状态为 **ACTIVE / INCOMPLETE**；下方记录之后的补证，不覆写该历史结论。

## 定向修正与 C/D 补证（2026-10-12；最新记录）

### 修正与针对性检查

- `scripts/s1_run_diagnostic.py`：记录每次 ROS graph 查询的 monotonic 起止、进程身份、解释器/ROS 环境、节点计数与 daemon 身份。daemon 结果只作诊断；fresh direct 查询出现缺节点时同环境有界重查，重复和失败仍拒绝放行。保存实际 controller-manager/writer PID 和加载库来源。RL 前采样 `/joint_states`，必须取得 12 个具名 Go2 关节有限位置/速度，否则在 command 3 之前中止。首次 `rl_entered && rl_active` 策略帧作为新增分析边界。
- `scripts/capture_joint_state_once.py`：样本记录实际 `/usr/bin/python3`、rclpy 文件、ROS header 和 monotonic 时间、具名 position/velocity。
- `scripts/s1_analyze_run.py`：保留历史 `0.10 m` 位移窗口，并新增首个 active policy cycle 到 terminal 的经过时间、有效连续时间、有效路径、速度和模式边沿；风险到模式切换字段明确是内部决策时间。
- `scripts/manual_ab_compare.py`：C/D 新批 comparison 增加 RL-active 边界字段；已从保存的 C/D metrics 重生成 comparison 表，没有重跑运行。
- 停止清理路径统一 MuJoCo 先退出、之后 controller launch、最后停止发布器。清理状态字段读取真实停止确认而不是固定写 `FIXEDSTAND`。本轮没有改 controller/C++、模型或研究参数，无需新构建/安装。
- `/usr/bin/python3 -m py_compile` 对本轮 Python 工具通过；`scripts/test_s1_runtime_source.py` 通过；节点清单计数检查通过。尝试运行 `scripts/test_s1_05_safety.py` 时在旧 v4 binary fixture 的长度断言失败，因为当前生产 reader 已接受 v3/v4/v5 布局；该失败与本轮修改无关，未改 fixture 或历史数据。`git diff --check` 通过。

### ROS 图原因判断

首批 C 的运行记录没有保存查询环境和时间，因而无法追溯证明当时是 discovery 未完成、ROS 域/RMW 不同，还是重复实例。本次 C daemon/direct 均各列一个节点；本次 D daemon 缓存各列两个同名节点，但相同 overlay/domain 的 fresh direct 查询各列一个、controller list 仅一条 active controller、该 launch 进程组仅一个 controller manager/writer，且策略帧 source/session 绑定该 PID。可确认 D 本次重复来自 daemon 缓存视图，不是两个活动 controller/writer；旧 C 的精确根因仍 UNKNOWN。新规则保留重复拒绝，不依赖 daemon 缓存放行。

C/D 查询 shell 的 `ROS_DOMAIN_ID` 未设置（默认 0），`ROS_LOCALHOST_ONLY=0`，`RMW_IMPLEMENTATION` 未设置；ROS daemon 命令行与 controller 映射指向 `rmw_fastrtps_cpp`。`ros2` 脚本由 `/usr/bin/python3` shebang 启动，运行时 direct 查询 PID 的 executable 为 `/usr/bin/python3.10`；shell 里 Anaconda Python 仅打印元数据。实际 RMW 选择由 daemon/controller maps 和同环境 system-Python identifier probe 交叉支持；直接 CLI PID 的早期 maps 仅抓到 `librmw_implementation.so`，该进程映射本身未捕获具体插件，报告不夸大成更强的逐 PID 插件映射证据。

### C/D 有效诊断

第二批目录：`logs/manual_abcd_compare/20261012_121911_451ec8/`。

| 组 | run / terminal | 位移起点结果 | mode / hold / 时钟 | 站立收尾与安全 |
|---|---|---|---|---|
| C | `[local run ID omitted]` / `VALID / ARRIVED`，终距 `0.4937 m` | 经过 `12.000 s`；有效 `11.960 s`；路径 `9.003 m`；平均/峰值区间速度 `0.753/3.104 m/s` | 24 边沿，其中有效时间 24；Recovery 3.830 s / 有效时间 11.960 s = 32.02%；`H=0.30 s` 审计 12/12 完整、PASS；1 个 invalid clock，0 周期缺口；50.0004 Hz | 5 s 站立观察完成；无运动期外部安全事件；碰撞无否定结论，跌倒 UNKNOWN（存在未确认姿态候选）。 |
| D | `[local run ID omitted]` / `VALID / ARRIVED`，终距 `0.4996 m` | 经过 `11.018 s`；有效 `10.816 s`；路径 `8.232 m`；平均/峰值区间速度 `0.761/3.473 m/s` | 22 边沿且均有有效时间；Recovery 3.354 s / 有效时间 10.816 s = 31.01%；保持审计 11 episodes 中 9 完整、2 不完整，总状态 `incomplete`；5 invalid clocks，0 周期缺口；50.0144 Hz | 正常站立观察目标 5 s，physics authority 为 MISSING 前只观察 2.092 s，command 6 未确认；command 9 后 controller 确认 PASSIVE（发布开始到确认 `4.795 ms`）；无运动期安全事件。D 原始 cleanup state 字段错误写 `FIXEDSTAND`，controller hard-stop 原日志确认 PASSIVE；原始记录保留并在 [group_results.md](group_results.md) 说明。 |

两个运行的 live framebuffer 截图分别为 [C LIVE](screenshots/C_live_[local run ID omitted].png) 与 [D LIVE](screenshots/D_live_[local run ID omitted].png)。画面分别显示 C/D 组和保持状态；截图只为原始 MuJoCo framebuffer 无损转 PNG。未为 Recovery 截图或其他结果追加运行。

### 初态和收尾结果

C/D 的 pre-RL `/joint_states` 均有 12 个具名关节位置/速度值；各自完整数值见 [initial_state_and_sources.md](initial_state_and_sources.md) 和原 `run_context.json`。RL 活跃帧分别在采样后 `1.935 s`、`1.605 s`，运动锚点后的异步 joint sample 分别晚 `0.784 s`、`0.953 s`；A/B 初始实测关节数据仍缺，四组不是严格初态配对。C/D reset qpos 均为构造状态中的 base `(0,0,0.445 m)`、单位四元数、12 个关节零 qpos；qvel 未由 reset probe 提供。

四次有效流程数据有效且任务结果均 ARRIVED；A/C 正常站立观察完成，B/D 正常站立观察未完成。四次运动期均无安全中止事件。B/D 的 command 9 是终局后停止回退，不算运动期安全事件。C/D `process_facts.json` 均 `closeout_complete=true`、退出码 0，记录 MuJoCo 在 controller 之前退出，之后 ROS/controller 和 stop publisher 退出；运行后项目进程核对无残留。D controller 日志确认 PASSIVE，但不等于 MuJoCo 命令消费。

最新完整对照、初态、模式/RA/接触/跌倒覆盖、图表及原始证据位置见 [group_results.md](group_results.md)；指标定义和可靠性边界见 [METRIC_AVAILABILITY.md](METRIC_AVAILABILITY.md)。本报告最新状态为 **COMPLETE — Director 有条件通过**。以下保留限制仍成立；当前没有已授权的下一项 Active Task。

### Remaining UNKNOWN 与影响

1. 首批 C 的直接查询为何空，无法从当时未记录 PID/RMW/环境/查询时间的原始记录中精确重建；新门槛在补证中通过，但不回改旧 INVALID 根因。
2. A/B 没有实际 joint position/velocity 样本；C/D 有 RL 前及运动锚点后的异步样本，但存在 0.14–1.94 s 时间间隔，不能视为同周期完整初态。四组不是严格相同初态，限制后续配对比较。
3. B/D 未完成目标 5 s 站立观察。command 9 的 PASSIVE 确认是 controller 状态证据，不是 MuJoCo 命令消费或物理运动完全停止确认。
4. D 的 11 个 Recovery episode 中 2 个时间覆盖不完整；审计总体仍为 `incomplete`。C 有未确认姿态候选而跌倒 UNKNOWN；C/D 未记录碰撞事件不构成无碰撞否定证据。
5. 每组只有一条有效运行且批次初态不同，只支持流程可执行和日志可复核；参数预实验、成功率和性能对照尚未完成。风险到模式决定时间不能替代完整避障响应延迟。
