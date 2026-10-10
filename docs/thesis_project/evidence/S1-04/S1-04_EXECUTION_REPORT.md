# S1-04 Execution Report

**Latest status: COMPLETE — Director 有条件通过（2026-10-08）。** 通过范围为基础运行、记录、指标及收尾流程；不表示切换效果或正式实验条件全部验证完成。两次完整运行的验收对照见[验收对照](diagnostic_runs_terminal_capture_20261008/acceptance_comparison.md)。历史执行记录、INVALID 原始数据和所有 UNKNOWN 均保留。

## 系统 GLFW 恢复与唯一补跑（2026-10-08）

### 图形环境排查与探针

旧失败 `preflight.json` 记录的是普通执行环境视图：X socket 可达，但该视图中 `/dev/dri` 不可见，GLFW 不能创建 window。按用户授权申请并使用 `require_escalated` 后，只读 `/dev/dri` 检查可见 `card1/card2/renderD128/renderD129`。没有驱动文件缺失或系统服务异常的证据。

此前成功探针源文件未保存在仓库，因此本次用系统 GCC 重建一个 16×16 hidden GLFW C 探针。编译器 `/usr/bin/x86_64-linux-gnu-gcc-11`，GCC `11.4.0`；`pkg-config glfw3` 为 `3.3.6`。在原 `DISPLAY=:0` 和 `XAUTHORITY=/run/user/1000/gdm/Xauthority` 下，系统 C 探针 `glfwInit=1`、窗口创建成功。使用诊断脚本实际设置的临时 `LD_LIBRARY_PATH` 再执行同一探针也通过。窗口在探针结束前销毁，无全局变量、授权、权限、系统服务或组件改动。

成功 context 直接报告 vendor `Intel`、renderer `Mesa Intel(R) Graphics (RPL-P)`、OpenGL `4.6 Compatibility Profile Mesa 23.2.1-1ubuntu3.1~22.04.4`；没有开启软件渲染。`/proc/self/maps` 直接显示系统 `libglfw.so.3.3`、`libGL.so.1.7.0`、`libGLX_mesa.so.0.0.0`、`iris_dri.so`、`libLLVM-15.so.1` 和 `libstdc++.so.6.0.30`。MuJoCo executable 链接系统 GLFW、`/usr/lib/x86_64-linux-gnu/libstdc++.so.6` 和项目 MuJoCo 3.3.3 library。当前 PATH 含 Anaconda，helper 的 Python 为 `/home/lidio/anaconda3/bin/python3`；但运行时 `LD_LIBRARY_PATH` 未含 Anaconda，native probe/ MuJoCo 的 GL 与 libstdc++ 都解析到系统路径。先前 Python ctypes GLFW 调用失败而 C 探针成功；不能把 ctypes 失败当作系统驱动故障。此前成功记录只存 Mesa OpenGL 4.6、未存 renderer；本次同为系统 GLFW 3.3.6/Mesa 4.6、Intel 硬件渲染，逐字相同的 renderer 历史上无法核实。探针证据见 [system_glfw_preflight.json](diagnostic_runs_terminal_capture_20261008/system_glfw_preflight.json)。

### 补跑记录与边界

启动前项目仿真、RViz、controller manager 和采集进程均未发现。唯一新 run 为 `7089073a2a684328b6523efa8c06c384`，采集器先启动；scene `scene_obstacle.xml`、候选 `paper_faithful_switch`、世界目标 `(1.0,0.0) m`、0.5 m 到达半径、`entry=exit=−0.05`、同一 `/tmp/s1_04_overlay` 的 Agile/RA/Recovery 模型以及 `6 s / 50 s` 上限均由 run context/实际 controller 来源事件确认。controller/writer PID `203918` 与 session `22412744739286` 一致。

实际诊断命令：`rtk env DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority rtk python3 scripts/s1_run_diagnostic.py --output-root docs/thesis_project/evidence/S1-04/diagnostic_runs_terminal_capture_20261008 --overlay /tmp/s1_04_overlay --goal-x 1.0 --sim-limit-s 6 --wall-cap-s 50`。显示变量只作用于本次命令；按用户授权使用沙箱外执行，没有改全局环境。

运行记录保留 38 个 LIVE 策略帧，session 单一、cycle 38–75 连续且 38 帧仿真时钟有效。最后保存的 cycle 75/sequence 152 payload 包含 RA `−0.9972`、Agile 模式、动作来源 1、位姿和时钟；其到目标距离 `0.499772 m`，落在 0.5 m 范围内。**但是**后处理在 `scripts/s1_run_diagnostic.py` 访问不存在的 `recorder.recorder.error` 时抛出 `AttributeError`。此时 ROS/MuJoCo 清理已经发起，但最终 run context 更新与 `recorder.finalize()` 未执行：`terminal_frame_ref` 未持久化、runtime record 无 terminal line，且缺 `terminal.json`/`process_facts.json`。因此不能将其终局分类为 ARRIVED，也无法证明 terminal_ref 与 cycle 75 对应；该 run 为 `INVALID_INCOMPLETE_POSTRUN_CONTEXT`，没有重新运行。

修复了该收尾属性访问为 `FrameCapture.error` 后，`python3 scripts/test_s1_04_contract.py` 和相关脚本 `py_compile` 通过；未再次启动仿真。原始 run context 和 38 帧 JSONL 保持原样；异常记录见 [postrun_failure.json](diagnostic_runs_terminal_capture_20261008/7089073a2a684328b6523efa8c06c384/postrun_failure.json)。分析器生成 [metrics.json](diagnostic_runs_terminal_capture_20261008/7089073a2a684328b6523efa8c06c384/metrics.json) 与说明性 [timeline.png](diagnostic_runs_terminal_capture_20261008/7089073a2a684328b6523efa8c06c384/timeline.png)：记录因 `missing_terminal_line`、`terminal_not_at_record_boundary`、`terminal_not_unique` 标 INVALID，运动指标 UNKNOWN。raw cycle 描述性频率为 37 个相邻周期对、`50.3865 Hz`（中位数 `20.102 ms`、P95 `27.753 ms`、最小/最大 `6.982/28.653 ms`），不作为有效 run 或验收指标。38 个 LIVE 碰撞快照有 325 个 physics-step gap，不能声称无碰撞。日志检索未发现 `ABS-CONTRACT` veto 或 `[EMERGENCY]`；清理中的 `[HARD-STOP] command=1` 是停止请求，不是 MuJoCo 消费确认。

验收对照见 [acceptance_comparison.md](diagnostic_runs_terminal_capture_20261008/acceptance_comparison.md)：完整第一次 `cb47356b98104e40b258b81a8246db17` 仍是有效参考；本次补跑来源/配置正确且持久化 cycle 75，但没有终端记录，因此不能作为第二次完整 run。先前缺终止帧的 run 与 INVALID 数据未改写。终局、运动起点、路径、速度、碰撞完整覆盖、跌倒检测、关节顺序、策略停止确认和 MuJoCo 命令消费仍为 UNKNOWN。

收尾进程组日志显示 ROS/controller、`robot_state_publisher` 和 RViz 收到 SIGINT；事后 `/proc` 检查未发现 `unitree_mujoco`、`rviz2`、`ros2_control_node`、`controller_manager` 或诊断进程残留。由于属性异常，退出码和逐进程 `process_facts` 没有持久化，不能报告精确 exit codes。GLFW 探针窗口已销毁，探针进程已退出。

修改文件：`scripts/s1_run_diagnostic.py`（修正 capture_error 来源）、`scripts/s1_analyze_run.py`（INVALID 时间线原因文案）、`scripts/test_s1_04_contract.py`、本报告、`tasks/S1-04.md`、`CURRENT_STATE.md`；新增 system GLFW 探针和本次故障/结果证据。没有改模型、控制算法、动作映射、研究参数或原始 run 数据。S1-04 继续 `ACTIVE / INCOMPLETE`，等待 Director 审阅。

简单来说，沙箱外系统 GLFW/OpenGL 已恢复，并完成了唯一一次补跑；策略帧写到 cycle 75，但收尾代码故障让终局关联信息没保存，因此这次不能验收。第一次完整运行仍有效，但第二次完整运行还缺。项目环境和探针窗口均已关闭，没有相关进程遗留。

## 完整收尾生产路径验证与唯一补跑（2026-10-08）

### 实现与离线验证

将实际诊断 `finally` 路径使用的收尾操作提取为 `finalize_run_closeout()`：先停止本次拥有的 ROS/controller 与 MuJoCo 进程组，再写收尾前 context、调用真实 `FrameCapture.finish()`/`RunRecordRecorder.finalize()`、写 `terminal.json`、补全最终 context 和 process facts。清理或 artifact 错误被收集到 context/process facts 并尽力重写；分析器发现 `lifecycle.closeout_errors` 时拒绝将运行作为完整结果。每个 owned process 的停止异常单独保留，继续处理另一个进程。没有改变控制器、切换、模型或动作。

新增临时目录端到端离线样例，实际使用 `FrameCapture.observe()` 与 `finalize_run_closeout()` 写入多帧记录，随后通过生产分析器 `calculate()` 校验。检查确认终止引用与 raw payload 的 session/cycle/sequence/monotonic 一致；payload 含 RA、模式、动作、位姿和仿真时钟；JSONL 恰有一条 terminal line；context、`terminal.json`、process facts 均保存；分析器接受结果，运动终止周期计入、后续清理帧排除。另注入 run-context 写入错误，确认清理回调仍先执行、错误被持久化，分析器返回 `INVALID / INCOMPLETE_CLOSEOUT`。既有缺 terminal payload 的拒绝样例仍通过。

`python3 scripts/test_s1_04_contract.py` 和相关脚本 `py_compile` 通过。此验证不是只在测试中复制收尾逻辑；它调用本次运行所使用的生产收尾函数。

### 显示预检、运行来源和配置

默认工作区权限保持不变。按用户先前授权通过 `require_escalated` 临时执行本机显示访问：`xdpyinfo` 对 `DISPLAY=:0` 成功；系统 GCC 11.4 编译的 native GLFW 3.3.6 隐藏窗口探针成功，OpenGL vendor `Intel`、renderer `Mesa Intel(R) Graphics (RPL-P)`、版本 `4.6 Compatibility Profile Mesa 23.2.1-1ubuntu3.1~22.04.4`。仅本次命令设置 `DISPLAY`、既有 `XAUTHORITY` 和诊断 helper 的 `LD_LIBRARY_PATH`；没有启用软件渲染、修改全局变量或安装组件。临时 probe 窗口、源码和二进制已关闭/删除。预检记录：[retry2_preflight.json](diagnostic_runs_terminal_capture_20261008/retry2_preflight.json)。第一次完整 run 没记录 renderer，所以不推断其与本次相同。

启动前相关项目进程列表为空，overlay 中 Agile/RA/Recovery 三个模型与 config 均存在。记录器先启动。本次唯一诊断 run `7172581a85b6411db9c1122043ce279b` 使用 `scene_obstacle.xml`、`paper_faithful_switch`、世界目标 `(1.0,0.0) m`、0.5 m 到达半径、`entry=exit=-0.05`、`/tmp/s1_04_overlay` 模型，以及 6 s 仿真时限/50 s 墙钟保护。`run_context.preflight` 和 ROS 日志确认 package share/config、模型和唯一 controller 来源一致：controller/writer PID `212324`，PGID `212297`，session `23546368334100`。日志记录 `[PATH] goal=(1.00,0.00)`、`[S1-RUN-SOURCE] candidate=paper_faithful_switch`；周期帧记录两个阈值都为 `−0.0500000007`。

实际启动命令：`rtk env DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority rtk python3 scripts/s1_run_diagnostic.py --output-root docs/thesis_project/evidence/S1-04/diagnostic_runs_terminal_capture_20261008 --overlay /tmp/s1_04_overlay --goal-x 1.0 --sim-limit-s 6 --wall-cap-s 50`。临时显示变量仅用于本次诊断命令。

### 终局、记录完整性和指标

运行结果为 `ARRIVED`，定义为终止策略帧进入 0.5 m 半径：distance `0.498959 m`。运动从 cycle 48、`sim_time=25.968 s` 开始，终止于 cycle 74、`sim_time=26.488 s`，有效持续 `0.520 s`。terminal_ref 指向 session `23546368334100` / cycle 74 / sequence 150 / monotonic `23547871799396`；runtime record 有且仅有一条与其身份匹配的完整 frame payload 和一条 terminal line，`terminal.json`、最终 `run_context.json` 与 `process_facts.json` 均在，`closeout_complete=true` 且 `closeout_errors=[]`。记录器自身 `termination_reason=FRAMES_ENDED_RC0` 表示正常停止后的帧/进程结束；supervisor 的任务终局仍为 `ARRIVED`，两字段含义不同。

分析器结果为 `VALID / ARRIVED`，38 帧、0 周期缺口、0 无效时钟帧；全采集周期统计 37 个相邻周期对，约 `49.8321 Hz`（中位数 `20.065 ms`，P95 `21.720 ms`，最小/最大 `9.782/27.515 ms`）。运动指标仅覆盖 cycle 48–74，路径 `0.471739 m`、平均路径速度 `0.90719 m/s`、0 次切换、0 Recovery。清理帧排除在运动路径和时长外。指标、结果行与时间线：[metrics.json](diagnostic_runs_terminal_capture_20261008/7172581a85b6411db9c1122043ce279b/metrics.json)、[run_results.md](diagnostic_runs_terminal_capture_20261008/7172581a85b6411db9c1122043ce279b/run_results.md)、[timeline.png](diagnostic_runs_terminal_capture_20261008/7172581a85b6411db9c1122043ce279b/timeline.png)。

原完整 run `cb47356b98104e40b258b81a8246db17` 与本次运行的两次同配置对照已写入 [acceptance_comparison.md](diagnostic_runs_terminal_capture_20261008/acceptance_comparison.md)。两次均 `VALID / ARRIVED`，终止帧完整且可复算，参数、场景和模型路径一致；原 INVALID 运行记录保持原样。无安全 veto 不等于没有碰撞：两次碰撞快照都有 physics-step 缺口（641 与 326），所以碰撞事件/无碰撞结论 UNKNOWN。两次都没有 Recovery。

### 清理、保留事项和状态

诊断关闭了本次 ROS/controller 与 MuJoCo；process facts 显示启动器退出码 0、没有强制终止、`shutdown_complete=true`。清理日志记录了 PASSIVE 请求和后续 `[HARD-STOP] command=1`，策略 FSM 停止确认仍为 false；这不是 MuJoCo 消费确认。事后核对没有 `unitree_mujoco`、`ros2_control_node`、controller manager、RViz 或 robot_state_publisher 残留；GLFW 探针与临时文件也已移除。

仍为 UNKNOWN：碰撞覆盖是否足以确认碰撞事件、跌倒检测、artifact 的真实动作关节顺序、停止请求后策略停止的独立确认、MuJoCo 是否消费 Recovery/最终命令。历史 INVALID run 和前次收尾异常不改写；旧预检/renderer 缺项不推断为当前系统故障。S1-04 保持 `ACTIVE`，等 Director 最终审阅，不自行标记 COMPLETE，也不启动后续任务。

简单来说，这次先用真实生产收尾代码做了成功与故障注入验证，再完成一次有效短诊断。它与已有完整诊断组成两次同配置结果，终止帧、指标和图表现已闭环。碰撞、跌倒、关节顺序及命令消费仍待后续讨论；仿真、控制器、RViz、记录器和探针均已关闭。

## 终止帧采集修正与补跑预检（2026-10-08）

### 修正与定向离线检查

`scripts/s1_run_diagnostic.py` 现在由 supervisor 单写入者采集：每次主循环读出一致性 raw 策略快照后，先将同一份 raw 写入记录并核对其 session、sequence、rl_step、RA、模式、动作、位姿和时钟字段，然后才用该帧判定到达、超时、碰撞或帧内安全终局。已持久化的相同周期去重；session 变更保留原始证据并继续触发来源拒绝。采集停止前无需等待异步线程，也不重读共享内存。外部安全日志没有对应策略快照时，只保留事件和最后已保存帧，不能假称该帧就是事件触发帧。

`scripts/s1_analyze_run.py` 现按 session/cycle/source_sequence/monotonic identity 查找终止帧，并检查完整 RA、模式、动作、关节目标、位姿和时钟 payload。匹配帧缺失、字段缺失或写入失败时，结果标为 `INVALID / INCOMPLETE_TERMINAL_FRAME_CAPTURE`，运动指标置 UNKNOWN。运动指标仍限于运动起点至终止周期（含终止周期）；终止后的清理周期不计入。

`python3 scripts/test_s1_04_contract.py` 通过，覆盖 supervisor 赶上落后一周期的终止快照、重复周期去重、session 混流及周期倒序拒绝、写入失败后不完整记录拒绝、终止周期包含于路径/Recovery 指标、清理尾段排除；`py_compile` 通过。来源匹配器和射线时间修正未改变。

### 补跑显示预检

预检前 `/proc` 未发现 `unitree_mujoco`、`rviz2`、`ros2_control_node`、`controller_manager` 或 S1 诊断进程。当前 uid 为 1000，环境指向 `DISPLAY=:0` 和既有 `/run/user/1000/gdm/Xauthority`；获准执行下 `xdpyinfo` 连接成功。隐藏 16×16 GLFW 预检中 `glfwInit=1`，但 `glfwCreateWindow` 返回空；Mesa 日志报告无法加载 iris 和 swrast driver。复现诊断脚本的 `LD_LIBRARY_PATH` 后结果相同。虽 `/usr/lib/x86_64-linux-gnu/dri` 中可见对应文件，当前执行环境也没有 `/dev/dri`；证据只能说明本轮执行器不能建立该 OpenGL context，不能单凭此判定主机桌面或授权文件故障。

为遵守先完成 GLFW 预检再启动采集/仿真的顺序，本轮在预检处停止；没有启动 recorder、MuJoCo、ROS/controller、RViz，也没有补跑。因此没有新增 run_id、来源绑定、终局帧或指标结果；两次完整同配置运行条件仍未满足。没有改授权、权限、系统服务、桌面设置或安装组件。历史第一次完整 run 与第二次漏终止帧的原始记录和分析结果保持不变。

### 本轮状态与 UNKNOWN

- 终止帧同步采集修正与针对性离线样例：通过。
- 本轮运行验证：未执行；具体阻塞为隐藏 GLFW/OpenGL context 创建失败。没有把预检失败算作短诊断或运行失败。
- 两次验收对照仍为第一次 `cb47356b98104e40b258b81a8246db17` 完整，第二次 `22e64352d7c042708b96608a16a89221` 缺 cycle 75 payload；本轮未能补跑。
- 碰撞完整覆盖、跌倒检测、动作关节顺序、策略停止确认及 MuJoCo 命令消费继续 UNKNOWN。未声称无碰撞或命令已消费。
- 修改文件：`scripts/s1_run_diagnostic.py`、`scripts/s1_analyze_run.py`、`scripts/test_s1_04_contract.py`、本 run 合同、本报告、`tasks/S1-04.md`、`CURRENT_STATE.md`，以及新增预检记录。未改控制算法、模型、动作映射、研究参数或原始 run 记录。
- S1-04 保持 `ACTIVE / INCOMPLETE`，等待 Director 审阅。

简单来说，同一份触发终局的策略帧现在会先同步落盘，分析器也会拒绝缺失终止帧的指标。本轮没有实际补跑：X 连接正常，但 GLFW 无法创建 OpenGL 窗口；没有启动仿真，项目进程保持关闭。

## Director 审阅补证（2026-10-08）

### 根因及来源修正

上次混合记录现已关联到两个同时写入者：

- 仍运行的 S1-03 `ros2 launch` 进程组中有 `ros2_control_node` PID 138829，参数文件来自 `/tmp/s1_03_overlay/.../robot_control.yaml`。S1-03 日志第 434 行直接记录 `/mujoco_rt_frame` session `16478219458311`；这个 session 正是保留的 S1-04 INVALID run `121051ab9a1d474a820563ddde740856` 中 425 帧的 `paper_faithful_switch` 流。
- 该旧 S1-04 run 的 ROS launch 使用工作区 install 的 `robot_control.yaml`；日志记录了 7.0 m 目标，第二个 session `16478220701166` 的帧记录为 `stabilized_switch`、退出阈值 `−0.08`。共享帧 ABI 原来没有 writer PID，所以仅从旧原始记录无法识别具体进程。

已识别的 S1-03 进程组（PID 138797、138827、138829；PGID 138670）收到 SIGINT，且确认全部退出。未改写原始 S1-04 INVALID 帧、ROS 或 MuJoCo 日志。直接进程证据及跨任务 session 对照保存在 [source_preflight_and_orphan_cleanup.json](diagnostic_runs_director_retry_20261008/source_preflight_and_orphan_cleanup.json)。

启动 helper 原先在加载工作区后把 `AMENT_PREFIX_PATH` 重设为只有 overlay，导致 controller package 无法找到。该错误直接保存在 run `0f4568bffd1d4c678d3100fd9d616c26`（`Package 'rl_quadruped_controller' not found`）。现在 helper 先 source ROS 和工作区，再在保留工作区路径列表的前提下把 overlay 放到首位；package prefix 检查和实际 launch 使用相同 shell 顺序。新增启动前检查是否已有 controller manager，并保存本次 manager 进程信息。

### 最小修改及定向检查

- 控制器新增日志，记录实际 package share、配置文件、Agile/RA 模型路径、候选、进入阈值、目标和 writer PID；共享帧初始化日志把 writer PID 与 session ID 写在一起。这些只增加诊断可见性，没有改变策略、模型、动作、阈值、频率或研究参数。
- 记录器在收尾前关闭运动采样窗口。run context/terminal 将主要终局与后续清理事件分开保存。分析器只在运动起点至配对终止周期内计算运动指标；Recovery 时长在第一个 Agile 周期闭合。没有运动区间时运动指标为 UNKNOWN。全采集周期质量单独注明范围。
- 离线针对性样例覆盖：周期缺口不桥接、Recovery 区间计至首个 Agile 周期、排除终局后的清理帧、保留但不以清理安全事件改写运动终局、缺少运动边界时保留 UNKNOWN。`python3 scripts/test_s1_04_contract.py` 通过；受影响 controller package 构建成功，Python 编译检查通过。

### 短诊断结果

安全 veto 前完成了一次配置正确的短诊断：[run be99392ce9444c1cbe8b17eabe095f7e](diagnostic_runs_director_retry_20261008/be99392ce9444c1cbe8b17eabe095f7e)。配置为 `scene_obstacle.xml`、目标 `(1.0, 0.0) m`、`paper_faithful_switch`、进入/退出阈值均为 `−0.05`，使用 `s1_04_overlay`。同一 launch shell 将 `go2_description` 解析到 `/tmp/s1_04_overlay`，将 `rl_quadruped_controller` 解析到工作区 install。ROS 日志显示唯一 manager PID 173825 使用 overlay 的 `robot_control.yaml`；`[S1-RUN-SOURCE]` 和 `[REC]` 日志直接记录 Agile、RA、Recovery 模型路径。策略 session `18128296032633` 与 `writer_pid=173825` 一致；路径日志中的目标为 `(1.00,0.00)`。

记录器只取得一个有效策略帧：`sim_time=22.800 s`、周期 0，状态已为 `FAULTED`、`RA=−1.0`、候选为 paper，阈值为 `−0.05/−0.05`。ROS 日志记录 `[ABS-CONTRACT] ... ray_reason=monotonic_clock_order` 和 PASSIVE 安全命令，之后控制器记录 `Switched from rl to passive`。终局为 `SYSTEM_SAFETY_ABORT`。没有运动起点、Recovery 命令或可用的频率、切换、驻留、距离及速度结果。随后代码审阅确认读取时序存在一个可导致该判断的漏洞（读取快照前采样当前单调时间）；但这没有证明该漏洞是此历史 veto 的唯一原因。安全检查没有关闭。

本 run 启动前取得了绑定本次 capture 的 `LIVE` 碰撞快照；策略期只有一个碰撞快照样本，因此碰撞覆盖不完整，不能据此声称无碰撞。直接的终局证据是 controller safety log、FAULTED runtime frame 和 `rl→passive` FSM 日志；后两者表明策略状态进入 PASSIVE，不表明 MuJoCo 已消费最终命令。

**直接证实：** 上一轮 S1-03 overlay 进程仍在运行，PID 138829 的参数路径和 session `16478219458311` 分别由进程快照与 S1-03 ROS 日志证实，并与旧 INVALID 流的 paper session 精确相同。旧 S1-04 launch 日志另显示 workspace install 参数路径；同一记录里的另一候选帧流为 stabilized。新的 run 有唯一 manager PID/session，且路径、候选、阈值与目标日志相符。

**代码与记录链推断：** 两个不同 controller 进程同时写无 owner lock 的同名 `/mujoco_rt_frame`，足以解释两组周期交错；旧帧协议没有 PID，旧 workspace 流的 PID 归属无法从原始帧单独恢复。新日志把配置来源、writer PID 和 session 绑定起来，启动时拒绝已有 manager，运行中遇到来源漂移会保留帧并标 INVALID。

**仍为 UNKNOWN：** 上述历史 `monotonic_clock_order` 是否由已发现的读取时序漏洞单独导致，以及修正后控制器能否在可用显示环境下正常推进；正常策略周期频率；没有运动起点时的运动和切换指标；非安全终局后 command 1 的请求/确认及未确认时的输出持续时间；跌倒检测、模型动作关节顺序、MuJoCo 命令消费。

前述启动失败和 preflight 格式化器错误作为单独尝试保存，不计入诊断 run。最终结果表为 [run_results.md](diagnostic_runs_director_retry_20261008/run_results.md)，该目录也包含说明 timeline。每次尝试后均核对本次启动的 MuJoCo、RViz、ROS 和 controller 进程，确认没有遗留；S1-03 旧进程组是在识别来源后单独关闭的。

### 更新后的完成条件

| S1-04 条件 | 当前结果 |
|---|---|
| 确认混合 writer，并让来源/配置可见 | 由 S1-03 精确 session 日志匹配确认根因；现在记录 writer PID 和实际模型路径。 |
| 绑定 launch package/config 与实际候选/阈值/目标 | 一次清洁预检及一次实际运行日志/帧证实 overlay、工作区 controller package、paper 候选、`−0.05` 阈值和 1.0 m 目标。 |
| 一个主要终局、清理分段及 PASSIVE 确认边界 | 合同和记录/分析行为已修订且离线测试通过；本次由策略 safety veto 终止，并有 FSM `rl→passive` 直接日志。非安全终局后 command 1 收尾尚未运行验证。 |
| 仅用运动阶段计算切换、Recovery、距离和速度 | 边界规则已修订且离线样例通过；本次没有运动起点，因此这些结果均 UNKNOWN。 |
| 两次相同设置、可信终局且可计算结果的短 run | **未满足。** 唯一符合配置的 run 在 safety veto 时停止，没有尝试第二次。 |
| 保留证据并关闭所启进程 | 原 INVALID 数据保留；新启动失败和安全终局记录已保存；本次启动的进程确认退出。 |

### 本轮文件更新

- 控制器最小路径/PID 观测：`quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp`。
- 来源预检、终局记录及停止检查：`scripts/s1_run_diagnostic.py`、`scripts/run_record.py`。
- 运动/周期指标及针对性样例：`scripts/s1_analyze_run.py`、`scripts/test_s1_04_contract.py`。
- 合同、任务状态、当前状态及本报告：`S1-04_RUN_CONTRACT.md`、`tasks/S1-04.md`、`CURRENT_STATE.md` 和本文件。
- 新增证据：`diagnostic_runs_director_retry_20261008/` 下的 orphan writer 来源对照、启动失败记录、安全中止原始记录、逐 run 指标、结果表和 timeline。既有 INVALID run 的帧、ROS、MuJoCo 原始记录未改写。
- overlay 中的诊断配置位于 `/tmp/s1_04_overlay`；工作区运行 YAML、模型和控制频率未修改。

仍为 UNKNOWN：首个周期触发 `monotonic_clock_order` 的具体原因；该配置下的正常策略频率和运动行为；运动、切换及 Recovery 指标；非安全终局的 command-1 停止确认及终局后输出时长；模型动作关节顺序（正式实验前风险）；跌倒检测和 MuJoCo 是否消费命令。具体配置混用根因已不再是 UNKNOWN。

简单来说，上次混流是 S1-03 遗留控制器和新控制器同时写共享帧；遗留进程已关闭。修正后的启动确实使用了请求的 paper 候选、1 m 目标和 overlay 模型，但控制器在第一个策略周期就安全停止。现在仍缺可用运动结果和第二次重复运行，因此 S1-04 继续 ACTIVE / INCOMPLETE。

The following sections retain the original S1-04 baseline findings and implementation history; where they say the mixed source was unresolved or that no later run existed, this supplement supersedes those statements.

## What was implemented

- [S1-04 run contract](S1-04_RUN_CONTRACT.md) defines run identity, preparation versus motion, world-frame target/distance units, motion-start interval, terminal criteria/precedence, collision versus ground contact, metric time bases, diagnostic limits, and closeout.
- [s1_run_diagnostic.py](../../../../scripts/s1_run_diagnostic.py) starts the record before MuJoCo, requires a capture-bound LIVE collision source before controller/motion startup, records the lifecycle and process groups, and has bounded cleanup. For non-safety terminal causes it now requests command 1/PASSIVE before ROS teardown; this closeout change was made after the invalid attempt and has not been runtime-verified.
- [run_record.py](../../../../scripts/run_record.py) now validates collision snapshots against a fresh host monotonic timestamp taken after reading the collision shared memory. This addresses snapshots whose simulator timestamp advances after the frame poll timestamp; this adjustment was not runtime-revalidated after the invalid attempt.
- [s1_analyze_run.py](../../../../scripts/s1_analyze_run.py) computes policy gaps/periods, mode edges, Recovery intervals/valid-time share, path length/speed, a result table and timeline. It refuses to aggregate an invalid record and emits an explanatory timeline instead.
- [test_s1_04_contract.py](../../../../scripts/test_s1_04_contract.py) covers external safety precedence and that cycle gaps are not bridged into dwell time.

## Ray 时间校验修正补证（2026-10-08）

### 修正及针对性验证

`StateRL::updateRay2d()` 原来先读取 `now_ns`，再读取射线时间戳和 seqlock 快照。若生产者在两步之间发布新帧，时间戳会合法地晚于旧 `now_ns`，从而误报 `monotonic_clock_order`。现在只有在序列一致性检查通过、11 个值和头部/时间戳构成同一稳定快照后，才读取 `validation_now_ns`；年龄计算、未来时间戳判定、超时和帧有效性使用这一同一值。安全检查保持开启，未放宽超时。

验证代码位于 [abs_ray2d_validation.h](../../../../common/abs_ray2d_validation.h)，并由控制器生产路径调用。新增 [test_abs_ray2d_validation.cpp](../../../../scripts/test_abs_ray2d_validation.cpp) 直接编译并测试同一判定函数：读取期间发布的新时间戳在快照后校验时合法通过；时间戳仍晚于校验时间时拒绝；过期、零时间戳、非有限射线及不一致/奇数序列拒绝。编译和运行此最小测试通过。`python3 scripts/test_s1_04_contract.py` 也通过。`rl_quadruped_controller` 的 colcon 增量构建成功。

安全日志现分别写出 `ray_stamp_ns`（生产者时间戳）、`ray_validation_now_ns`（实际校验时钟）与 `detection_ns`（安全 veto 检测时刻），避免把三者当成同一时刻。

这一代码漏洞已确认，但仍不能证明它是前一 run 出现 `monotonic_clock_order` 的唯一原因；该因果关系保留 UNKNOWN。

### 本轮短诊断与收尾

受限执行器中的启动失败 run [4a22ae0ee95b4ec1b7e3849e0481bacf](diagnostic_runs_ray_clock_fix_20261008/4a22ae0ee95b4ec1b7e3849e0481bacf/) 保留为 `STARTUP_FAILURE`，MuJoCo 报 `could not initialize GLFW`，0 策略帧。随后只读 `xdpyinfo` 连接 `DISPLAY=:0` 成功；使用现有 GLFW 3.3.6 编译的 16×16 隐藏窗口预检也成功，得到 Mesa OpenGL 4.6。两项是在获准访问本机桌面的执行中完成，未改授权、权限或系统设置；据此只能区分出受限执行器无法完成原启动，不能推断系统桌面有故障。

获准桌面访问后唯一实际短诊断为 [run 6ff88d266e174f71a74b8f2a64f1c651](diagnostic_runs_ray_clock_fix_20261008/6ff88d266e174f71a74b8f2a64f1c651/)。启动前无遗留 controller manager，采集器先记录 `recorder_ready`；目标 `(1.0,0.0) m`、`paper_faithful_switch`、overlay `/tmp/s1_04_overlay`、场景 `scene_obstacle.xml` 和碰撞源均被记录。实际 `[S1-RUN-SOURCE]` 行给出 `candidate=paper_faithful_switch`、entry `−0.05`、目标 `(1.000,0.000)` 以及 Agile/RA overlay 模型；独立 `[REC]` 行记录 overlay Recovery 模型。frame session `19444219130639` 与 writer PID `184497` 日志匹配，帧候选为 paper、进入/退出阈值均为 `−0.05`。

运行器仍将此 run 记为 `INVALID_RUNTIME_SOURCE`。只读对照定位到校验器格式假设不匹配：实际来源行使用 `candidate=paper_faithful_switch`，校验器却查找 `switching_mode=paper_faithful_switch`；校验器还要求 Recovery 路径出现在同一来源行，而实际 Recovery 路径由独立 `[REC]` 日志提供。没有证据表明本 run 实际加载了错误配置，但运行器判为无效的数据不得升格为有效 S1-04 结果。结果表的 `record_validity=VALID` 仅表示帧记录结构/连续性检查通过；`terminal_result=INVALID_RUNTIME_SOURCE` 是本次运行的最终分类，因此整次 run 不作为有效 S1-04 结果。本次保留 37 个记录帧；结果表计算的全采集周期频率为 `52.9045 Hz`（36 个相邻周期对、0 个策略步缺口、1 个无效时钟帧），仅作该 INVALID 记录的原始诊断统计，不作为有效运行频率。没有运动起点，切换、Recovery、距离和路径速度均 UNKNOWN。未观察到安全 veto；终局是 `INVALID_RUNTIME_SOURCE`，不是 MuJoCo 已消费命令的证据。

校验器发现来源绑定失败后，本轮按停止规则没有启动第二次场景。两次尝试的表格与逐 run 时间线见 [run_results.md](diagnostic_runs_ray_clock_fix_20261008/run_results.md)。ROS launch 与 MuJoCo 收到 supervisor 清理并以退出码 0 结束；后续 `unitree_mujoco`、`rviz2` 和 `ros2_control_node` 进程检查均无匹配运行进程。

### 更新后的状态与保留事项

- 射线时间戳读取顺序修正、共享生产/测试判定、针对性离线检查及增量构建：完成。
- 两次相同设置且具有可信终局和可计算结果的运行：未完成。一个实际 run 因来源校验器格式不匹配而标 INVALID，按规则未重复运行。
- 上次 safety veto 是否单由读取时序漏洞导致、校验器修正后的有效 run、有效策略频率和运动指标、MuJoCo 是否消费命令、跌倒检测、artifact 真实关节顺序：仍 UNKNOWN。INVALID run 的 52.9045 Hz 不作为有效频率结论。
- S1-04 继续 `ACTIVE / INCOMPLETE`，等待 Director 审阅；本轮未启动后续任务。

## 来源匹配器修正与同配置诊断（2026-10-08）

### 实现与离线回归

新增 [s1_runtime_source.py](../../../../scripts/s1_runtime_source.py)，由 [s1_run_diagnostic.py](../../../../scripts/s1_run_diagnostic.py) 调用。来源检查不再要求多个日志事件共处一行：它以当前 `ros_launch.log` 内的 ROS launch 节点标签为边界，关联 `[S1-RUN-SOURCE]`、`[RtFrame]` 和独立 `[REC]`/`[PATH]` 事件；来源事件 PID 必须等于本次 launch 唯一 manager PID，writer PID、writer session 必须分别匹配该 PID 和第一条策略帧的 session。还核对 manager 实际启动命令中的 overlay `robot_control.yaml`，以及来源日志中的完整 `package_share`、ABS config、Agile/RA 绝对模型路径、candidate、entry 阈值；exit 阈值由同周期帧核验。独立 Recovery 模型加载路径及 `[PATH]` 运行目标必须由同一 ROS launch 节点记录并逐字对应预期完整路径/坐标。

日志证据不足返回 `pending`，运行器最多等待 3 s 并将待确认状态写入 run context；缺证超时、明确值不符、额外 controller/source/writer 或 session 漂移将导致 `INVALID_RUNTIME_SOURCE` 并停止该 run。采集期间仍保留原始帧；只有来源确认且 session 未变，才进入有效运行判断。运行器预检同时拒绝已有 manager、要求本次 launch 组内恰有一个 manager，并在启动前检查三个模型文件均存在。

`python3 scripts/test_s1_runtime_source.py` 以 2026-10-08 保存的真实 `ros_launch.log` 和 runtime frame 回归，确认能解析 candidate、跨行 Recovery 模型记录、实际目标、launch config、完整模型路径和 writer/session；并覆盖候选、来源目标、PATH 目标、完整模型路径、writer PID/session、多 controller 来源及缺少 source/Recovery 证据。明确不符返回 conflict，缺事件返回 pending。当前解析器还对本轮两个实际 launch 日志重新验证为 confirmed。`python3 scripts/test_s1_04_contract.py` 与 Python 编译检查通过。上次的 37 帧 INVALID_RUNTIME_SOURCE 原始数据没有改写；其原因已定位为旧校验器字符串/行布局假设错误。

### 两次相同设置运行

两次运行均使用 `scene_obstacle.xml`、目标 `(1.0, 0.0) m`、`paper_faithful_switch`、`entry=exit=−0.05`、`/tmp/s1_04_overlay`、相同三个 deployment model 和 `6 s` 仿真运动上限 / `50 s` 墙钟保护。每次启动前无相关旧进程，采集器先就绪，预检确认本次唯一 manager；两次 run_id 和 session 独立。

| Run | 来源与终局 | 周期/运动结果 | 限制 |
|---|---|---|---|
| `cb47356b98104e40b258b81a8246db17` | `VALID / ARRIVED`；PID/writer `189998`，session `20302804686619`；75 帧；运动终局 cycle 74，`sim_time=24.524 s`，目标距离 `0.4843 m`。 | 周期缺口 0，1 个无效时钟帧；全采集相邻有效周期频率 `51.7758 Hz`，周期中位数 `19.988 ms`，P95 `26.186 ms`，最小/最大 `1.939/31.286 ms`。运动起点 cycle 49，`24.026 s`（区间 `24.006–24.026 s`）；有效连续运动时长 `0.460 s`，0 次模式转换、0 Recovery、Recovery 占比 `0%`；路径长 `0.4292 m`，平均路径速度 `0.9331 m/s`。 | 75 个碰撞快照均为 LIVE，但物理步不连续（641 个 gap），不能据此声称无碰撞。 |
| `22e64352d7c042708b96608a16a89221` | `VALID / ARRIVED`；PID/writer `192046`，session `20359911390477`；75 帧；run context 终局 cycle 75，`sim_time=24.338 s`，到达距离 `0.484716 m`。 | 记录周期缺口 0、2 个无效时钟帧；全采集频率 `51.2443 Hz`，周期中位数 `19.887 ms`，P95 `28.074 ms`，最小/最大 `6.337/36.591 ms`。运动起点 cycle 48，`23.796 s`；有效连续运动时长 `0.478 s`，记录的模式转换 0 次、Recovery 0； | **终端策略帧缺失：** runtime record 最后为 cycle 74，距离 `0.510698 m`，没有持久化 context 引用的 cycle 75 payload。因此周期 75 的 RA/模式/动作无法复核，路径与速度只到 cycle 74；本 run 的运动指标未覆盖合同要求的终止周期。表中 `record_validity=VALID` 指已保存帧结构和周期连续，不能代表终端边界采集完整。 |

两次 `terminal.json` 的运动期 `external_safety_events` 均为空，没有安全 veto。两次到达终局后，清理阶段都记录 command 1/PASSIVE 请求和随后 `[HARD-STOP]`；该事件留在 `cleanup_events`，没有覆盖先前 ARRIVED。停止请求后的策略停止确认未取得（FSM transition confirmation false）；它不等于 MuJoCo 已消费命令。两次 ROS launch 和 MuJoCo 均以退出码 0 收尾，事后没有 `unitree_mujoco`、`rviz2`、`ros2_control_node` 或 controller manager 遗留。

两次原始日志、runtime record、context、terminal、metrics 和时间线保存在 [diagnostic_runs_source_matcher_20261008](diagnostic_runs_source_matcher_20261008/)。汇总表为 [run_results.md](diagnostic_runs_source_matcher_20261008/run_results.md)；其附注解释第二次运行终止帧缺失。原始记录未改写。

### 状态、剩余 UNKNOWN 与文件

- 来源匹配器误判已修复；旧真实 ROS 日志离线回归通过，两次新 run 的同一当前解析器复核均为 confirmed。
- 本轮做了两次同配置启动，两次来源与到达终局均可信；第一次满足运动终止边界采集，第二次虽有可信 ARRIVED 事件和距离，但终止周期 payload 未写入记录。因此“两次均具有完整、可计算的含终局周期结果”仍未满足。本轮按采集缺口停止，没有第三次运行。
- 未出现运动期 safety veto；两次 cleanup 都有被分段记录的硬停止事件。第二次周期 75 的 RA/模式/动作、其终止周期切换计数和包含最后周期的路径速度为 UNKNOWN。
- 两次均无 Recovery；碰撞快照未覆盖连续物理步，不能下无碰撞结论。跌倒检测、停止请求后的策略停止确认和 MuJoCo 命令消费继续 UNKNOWN；部署模型关节顺序仍 UNKNOWN。
- 修改文件：`scripts/s1_runtime_source.py`（新解析器）、`scripts/s1_run_diagnostic.py`（实际来源匹配、唯一 manager/模型预检、3 s pending）、`scripts/test_s1_runtime_source.py`（新定向回归）、本报告、`tasks/S1-04.md`、`CURRENT_STATE.md`、本轮结果表附注。没有改控制算法、模型、动作映射或研究参数。
- S1-04 保持 `ACTIVE / INCOMPLETE`，等待 Director 审阅；不启动后续任务。

简单来说，来源校验现在能识别真实日志里分开的模型加载记录，两次同配置诊断也都确认了正确配置并观察到到达。但第二次没有把终止策略周期保存下来，所以两次完整指标仍未闭环。仿真、RViz、控制器和采集进程都已关闭。

No model, controller algorithm, action, observation, switching logic, control frequency, repository runtime config, training, formal experiment, or hardware path was changed. A temporary config copy under `/tmp/s1_04_overlay` requested `paper_faithful_switch` and a 1.0 m goal; the attempt below proved that overlay was not actually selected by the launch environment.

## Direct observations from the single attempt

Evidence directory: [diagnostic_runs_20261008/121051ab9a1d474a820563ddde740856](diagnostic_runs_20261008/121051ab9a1d474a820563ddde740856). The original `runtime_record.jsonl`, ROS/MuJoCo logs, process facts, and start/cleanup timestamps are preserved. `run_context.json` now distinguishes intended settings from actual log evidence.

- The run wrote 832 LIVE policy-frame snapshots from **two interleaved session IDs**: `16478219458311` (425 frames; observed rl_step 0–447) and `16478220701166` (407 frames; observed rl_step 0–418). Strict record validation rejects cross-session and non-increasing cycle order. The two streams also show `switching_mode=1` and `switching_mode=0`, so no single candidate can be assigned to the full record.
- There were 822 clock-valid policy frames. Collision snapshots were LIVE on 823 frames and INVALID on 9. The invalid rows had collision timestamps later than their frame-record timestamps by 11,768 ns to 3,660,651 ns. This is consistent with a read-order race; the new post-read timestamp check is intended to avoid it, but needs one targeted future run to verify.
- The context planned goal X=1.0 m and paper-faithful switching. The ROS launch log instead shows `[PATH] ... goal=(7.00,0.00)`, and loads the Recovery artifact from the workspace install tree. This is direct evidence the temporary package overlay was not selected in that run. The Recovery path is logged as `/home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/go2_description/share/go2_description/config/rec/policy.pt`; source lookup plus the same resolved package share and logged filenames imply Agile `/home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/go2_description/share/go2_description/config/abs/policy.pt` and RA `.../config/abs/ra_value.pt`. Runtime frames contain both candidate identifiers: both use entry `-0.05`, while mode 0 has exit `-0.08` and mode 1 exit `-0.05`. Because the streams are interleaved, they cannot be combined as one candidate result.
- The supervisor recorded `DIAGNOSTIC_SIM_TIME_LIMIT` as the movement end. A later frame at monotonic time `16487181527679` (`session_id=16478219458311`, `rl_step=447`, `sim_time=30.286 s`) had `policy_state=FAULTED`. It occurred after the supervisor movement-end timestamp `16486642145450` and before cleanup completed at `16487321167659`. The recorder terminal therefore reports `SAFETY_FAULT`. This conflict means the observed run end and policy cessation were not yet represented by a single credible terminal.
- The run is **not** a valid A-group result, success/failure rate sample, collision-free sample, or usable frequency measurement. The per-session raw adjacent cycle intervals had medians of 19.401 ms (402 adjacent pairs) and 18.298 ms (394 adjacent pairs), but the two-session/candidate conflict prevents reporting one actual run policy frequency. The summary row deliberately marks derived metrics UNKNOWN.
- The current attempt observed Recovery frames, but mode transitions, dwell time, motion duration, and speed are not reported as run metrics because the source record failed validity. The record's terminal collision-free coverage is UNKNOWN; command consumption by MuJoCo and authoritative fall detection remain UNKNOWN.

The table and explanatory timeline are [run_results.md](diagnostic_runs_20261008/run_results.md) and [timeline.png](diagnostic_runs_20261008/121051ab9a1d474a820563ddde740856/timeline.png). The row is `INVALID / INCOMPLETE_UNTRUSTED_RECORD`; metrics are not filled from a mixed session stream.

## Display, cleanup, and minimum checks

- Before the attempt, `xdpyinfo` passed for `DISPLAY=:0` and existing `XAUTHORITY=/run/user/1000/gdm/Xauthority`. The existing hidden GLFW preflight created a 16×16 context with Mesa OpenGL 4.6. During the run, RViz logged OpenGL 4.6. No authorization file, permissions, display service, or installed component was changed.
- The ROS launch process group exited with code 0 on SIGINT; the MuJoCo process group also exited with code 0. A post-run process check found no `unitree_mujoco`, `rviz2`, `ros2_control_node`, `robot_state_publisher`, or S1 recorder process. Motion end and cleanup completion are separate context timestamps.
- The read-only package-resolution check after correcting the launch environment returned `/tmp/s1_04_overlay`. That verifies package lookup in the corrected shell only; the corrected launch was not run.
- `python3 scripts/test_s1_04_contract.py`: PASS (safety terminal priority, cycle-gap handling, Recovery dwell). `python3 scripts/test_run_record.py`: 54/54 PASS. Python compilation checks passed for the modified recorder and S1 scripts. No build, training, formal run, or second simulation was performed.

## Files changed

- Runtime-adjacent diagnostic/recording tools: `scripts/run_record.py`, `scripts/s1_run_diagnostic.py`, `scripts/s1_analyze_run.py`, `scripts/test_s1_04_contract.py`.
- S1-04 documents: `S1-04_RUN_CONTRACT.md`, this report, `tasks/S1-04.md`, and `CURRENT_STATE.md`.
- New evidence: the one run directory, invalid-status result table, and explanatory timeline under `diagnostic_runs_20261008/`. Raw policy, clock/collision snapshots and ROS/MuJoCo logs were not rewritten; only the run-context note was amended to distinguish planned from observed configuration.
- Temporary runtime overlay/config is under `/tmp/s1_04_overlay`; workspace runtime YAML was not changed by this task.

## 上轮首次尝试的完成条件快照（已由上方补证更新）

| S1-04 condition | Result |
|---|---|
| Run ID, evidence directory, session, preparation/motion boundaries | Partial implementation; the only record contained two interleaved sessions, so association is not credible. |
| Run/terminal definitions and external safety source | Contract written; external safety-event field tested offline. Runtime termination conflict remains. |
| Collision versus ground contact | Contract uses bound obstacle contacts; one attempt had 9 invalid snapshots and no valid full-run collision conclusion. |
| Base metrics and cycle integrity | Analyzer and gap/dwell samples checked offline; no trustworthy run metrics due record invalidity. |
| Timeline and result table | Generated an invalid-status summary row and an explanatory non-data timeline; no valid RA/mode trajectory. |
| Simple scene and short target, preferred paper candidate | 当时 workspace config/7 m 目标实际加载；这一结论只适用于该次旧 run。 |
| Two identical short runs with computable results | **Not met.** 此表当时只有一次 INVALID 尝试；Director 补证后的实际结果见上方。 |
| Closeout and no unrelated work | Attempted MuJoCo, RViz, ROS, and recorder were closed; no formal experiment, model, training, or hardware activity. |

## 当前 Remaining UNKNOWN

- `monotonic_clock_order` veto 的具体触发原因；不关闭安全检查的前提下，本候选正常运行时的 policy 频率和运动行为。
- 没有运动起点后的切换、Recovery、到达、距离及速度指标；两次相同设置且可计算结果的运行条件未满足。
- 非安全终局下 command 1/PASSIVE 的停止确认和未确认时的策略输出持续区间。
- 跌倒检测、Recovery 命令是否被 MuJoCo 消费、Agile/Recovery artifact 的真实动作关节顺序。

配置混用的根因已经确认，不再列为 UNKNOWN。下一步由 Director 决定如何处理安全 veto；本报告不自行继续场景运行或启动下一任务。

简单来说，混流来源和错误 launch 环境已经修正并有直接证据；修正后的唯一诊断在第一策略周期触发安全停止，未取得运动结果。S1-04 仍为 ACTIVE / INCOMPLETE。

## Director 最终审阅结论（2026-10-08）

Director 对 S1-04 有条件通过，任务状态为 `COMPLETE`。通过范围仅为基础运行、记录、指标和收尾流程；不等于切换效果或正式实验条件全部通过。两次完整诊断对照见[验收对照](diagnostic_runs_terminal_capture_20261008/acceptance_comparison.md)。本报告中的历史执行记录与 INVALID 原始数据未更改。

保留限制：两次诊断均未出现 Recovery，不能用于判断切换效果；碰撞覆盖不完整，不能声称无碰撞；跌倒检测、模型动作关节顺序、策略停止确认及 MuJoCo 命令消费仍为 UNKNOWN；原完整运行没有记录 renderer，因此不能声称两次 renderer 已证实一致。S1-04 的条件通过不将 S1 整体标记为完成。
