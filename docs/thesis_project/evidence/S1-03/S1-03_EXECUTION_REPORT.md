# S1-03 Execution Report

**审阅后状态：COMPLETE（Director 有条件通过，2026-10-08）。**执行器提交时的 INCOMPLETE 结论及失败尝试保留。已接受的范围是正常连续运行的时钟、策略事件和命令来源记录；暂停后策略恢复及同名共享时钟重连仍未验证，不得视为已支持。暂停或安全中止后的 run 不直接续接，恢复流程另行安排。S1-03 已关闭。

**Director 对证据解释的限定：**日志中的 DDS timeout 来自关节位置连续不变检测，不等于直接观测到 DDS 通信故障。按键时间记录未给出完整发送区间；时钟变化早于记录时间，既不能确认也不能否定按键作用。下文各轮“INCOMPLETE / 等待审阅”是执行时的历史结论，以本段最终审阅状态为准。未修改原始采集记录。

## 摘要

控制器现读取已有 `/mujoco_sim_clock`，并在 runtime frame v2 记录策略周期的时钟样本、有效状态、来源序号、来源单调时间、时钟段及年龄；同一帧还记录 RA、阈值、转换前后模式、切换候选、动作来源和转移原因。Recovery 首个控制接口写出与来源 session/策略周期直接关联，并标记 episode。未修改候选条件、阈值、保持计数或动作数值。

控制器及直接记录消费者已构建/离线核验。旧沙箱内启动失败由 socket `EPERM` 导致；沙箱外复查连接到现有 X.Org 桌面，系统编译器下 GLFW/OpenGL 预检通过。既有短诊断加载了三模型并记录策略周期、仿真时间、切换和 Recovery 首次接口写出；较早的单独尝试观察到暂停时钟停更，但未发恢复键。本轮成对按键尝试的时钟边界未配对，且 DDS timeout 后策略帧未恢复；控制器时钟重映射也未运行验证，因此任务保持 INCOMPLETE。

## 时间语义

- 来源是 `unitree_mujoco/simulate/src/main.cc` 已有的 `abs_sim_clock::SimClockWriter`，每次 `mj_step()` 后发布 `mjData.time`（秒）、源序号及 `steady_clock` 单调时间。S1-03 复用该链，不改模拟器调度。
- `StateRL::runModel()` 在策略周期开始读取最近的完整快照；`writeRtFrame()` 在帧发布时重算样本年龄。样本不是策略决策瞬间的精确仿真时间。帧保留 `sim_time_s`、源序号/时间戳、时钟段和年龄；关联区间为源样本时间到策略帧发布单调时间，无插值。
- 状态包括 UNAVAILABLE、FRESH、STALE、RESET、STATIONARY。只有 FRESH/STATIONARY 且样本距策略周期起点不超过 100 ms 时有效；帧发布时如年龄超过 100 ms，则标为 STALE。100 ms 是新鲜度判据，不是保持时间。
- 早于当前 controller session 起点或时间戳在未来的源快照不能作为有效时间。控制器局部 `sim_clock_segment_id` 在新 session 首个有效样本、源序号回退、源单调时间回退或 `sim_time` 回退时增加。RESET 样本保留源字段并置无效。
- 若暂停期间没有新物理时钟样本，年龄超过 100 ms 后变为 STALE。当前发布协议无法区分仿真暂停与时钟发布器失效，二者为 UNKNOWN。本轮新增运行中有策略周期被记为 STALE，之后时钟样本又递增；但时钟停止/重启样本未能与暂停/恢复按键边界配对，且策略进入安全停止，不能据此认定控制器已正常恢复。

## 周期字段与事件链

`common/abs_rt_frame_contract.h` 的 v2 固定 ABI 为 536 字节，Python 解析器和记录器同步。旧 v1 不会被静默按 v2 接受。

每个策略帧包含 session、`rl_step`/policy cycle、帧单调时间、仿真时钟关联、同周期 RA、进入/退出阈值、候选、`mode_before`/`mode_after`、动作来源、转移原因、风险条件和模式转换标记。风险评估时间在 RA 推理后采集；只有连续有效周期内假→真才记 `risk_condition_entered` 时间，首周期/缺口保留 UNKNOWN；模式转换时间在状态机完成转换决定处采集。事件时间均为 `steady_clock`。

`setCommand()` 在实际写控制接口的位置写出 q/dq/kp/kd/tau。它在同一互斥保护区复制命令和来源元数据；`[ABS-POLICY-CMD]` 含写出序号/时间、来源 session/策略周期、模式、动作来源、原因和 Recovery episode。每个 Recovery episode 的首个 Recovery 来源写出标为 `recovery_command_issued`；`ABS_POLICY_EVENT_TRACE=1` 时重复写出为 `command_write`，仍带来源周期。该事件只证明接口写出，不证明 MuJoCo 已消费/应用。

## 验证与证据

- `scripts/test_abs_rt_frame.py`：26/26 通过（v2 ABI、时钟失效样本、模式/事件字段）。
- `scripts/test_run_record.py`：54/54 通过。
- `scripts/test_formal_runtime_binding.py`：12/12 通过。
- `scripts/test_formal_rt_frame_recorder.py`：12/12 通过。
- `scripts/test_p1_08_harness.py`：96 项检查通过，其中包含 harness 预检失败分支；不是仿真运行证据。
- 相关 Python 文件 `py_compile` 通过；`collect_runtime.py --check` 通过，报告帧大小 536 字节、时钟大小 40 字节。启动前 [collector_ready.json](diagnostic_20261008/collector_ready.json) 和 [capture_start.json](diagnostic_20261008/capture_start.json) 保存了预检结果。
- `rl_quadruped_controller` 构建通过：`colcon build --packages-select rl_quadruped_controller --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch`。最终构建 stderr 为空，日志位于 `quadruped_ros2_control_humble/log/latest_build/rl_quadruped_controller/`。
- 先前沙箱内仿真尝试在控制器启动前退出；[旧启动日志](diagnostic_20261008/mujoco.log) 报 `ERROR: could not initialize GLFW`。本次沙箱外预检确认这是执行器对 X socket 的 `EPERM`，不是宿主显示故障。先前目录无运行策略帧；本轮成功运行的证据见下方 `diagnostic_display_retry_20261008/`。

诊断将 `paper_faithful_switch` 仅设在 `/tmp/s1_03_overlay` 下的临时配置副本；本轮 controller log 证实三模型均已加载，运行配置仍不是正式 A 组结果。工作区运行配置未改动。

## 修改文件

- `common/abs_rt_frame_contract.h`：frame v2 ABI 和时间/事件字段。
- `quadruped_ros2_control_humble/controllers/rl_quadruped_controller/include/rl_quadruped_controller/FSM/RASwitchingLogic.hpp`：候选感知的实际退出阈值选择。
- `quadruped_ros2_control_humble/controllers/rl_quadruped_controller/include/rl_quadruped_controller/FSM/StateRL.h`、`src/FSM/StateRL.cpp`：时钟读取、生命周期/年龄状态、周期元数据及命令写出来源记录。
- `scripts/abs_rt_frame.py`、`scripts/run_record.py`：同步 v2 布局、验证及逐周期 JSONL 字段。
- `scripts/test_abs_rt_frame.py`、`scripts/test_run_record.py`、`scripts/test_formal_runtime_binding.py`、`scripts/test_formal_rt_frame_recorder.py`、`scripts/test_p1_08_harness.py`：同步 ABI 夹具/结构检查。
- `docs/thesis_project/evidence/S1-02/short_diag_20261007/collect_runtime.py`：更新直接采集消费者以读取 v2 并写入已有 per-run record；其历史诊断产物未重写。
- 本报告、`tasks/S1-03.md` 引用及 `CURRENT_STATE.md` 状态。
- `display_preflight_retry_20261008.json`：最新 DISPLAY/Xauthority/OpenGL 启动预检及停止决定。

新增证据文件：`display_glfw_preflight_retry_20261008.json` 记录沙箱外显示/GLFW预检；`diagnostic_display_retry_20261008/` 保存本轮逐周期、时钟、ROS/MuJoCo 日志及运行上下文。

## 直接证实、代码推断与 UNKNOWN

**直接证实：** C++ frame ABI 声明 536 字节；控制器源码构建与采集器预检通过；沙箱外 `:0`/X.Org 和原 Xauthority 可用，系统编译器下可建 GLFW/OpenGL 4.6 context。既有成功短诊断加载三模型并采得 1,796 个连续策略周期：1,792 帧时钟有效；4 个无效帧均为 `STALE (sim_clock_status=2)`，位于周期 0、73、626、1329；没有 RESET 帧。首帧 `ra_value=-0.386445...` 且 `risk_condition_met=false`，不能称首帧风险条件已成立。既有运行观察到 3 次 Agile→Recovery→Agile；Recovery 首个控制接口写出分别来自周期 133、136、162。

本轮另有 69 个同 session 策略帧（周期 0–68）：49 个有效 FRESH、20 个无效 STALE（周期 49–68）。Space 请求前基线曾确认周期和时钟推进；但第一帧 STALE（周期 49，单调时间 8883230765003）比 `pause_sent` 记录早约 65 ms，且 source clock `sequence=18640, sim_time=18.638s, monotonic_ns=8883115882271` 的最后样本比该按键记录早约 180 ms。因此时钟停更及 STALE 分类已经在按键记录之前开始。暂停观察点仍读到该快照；随后首个新时钟样本 `sequence=18642, sim_time=18.640s, monotonic_ns=8884594437265` 出现在 `resume_sent` 之前约 178 ms。因此按键与时钟停止/重新推进没有形成可归因的成对边界，不能把它们写成已验证的暂停/恢复响应。ROS 日志随后直接记录 `[EMERGENCY] DDS timeout detected (100 steps frozen)! Forcing PASSIVE!`；恢复键发出后 3 秒观察到 clock 到 `sequence=21828, sim_time=21.826s`，但没有新策略帧。安全检查未关闭，也未重跑。证据位于 `diagnostic_pause_resume_final_20261008/`。

本次 69 帧的 `safety_faulted` 均为 false，runtime terminal 汇总也写 `safety_fault_seen=false`；DDS 强制 PASSIVE 发生在最后策略帧之后，所以安全停止结论依据 ROS 控制器日志，而不是该帧字段。采集器在计划的 3 秒窗口之后仍记录到时钟推进至 `sim_time=65.904s`，而策略帧数保持 69；该尾段保留在原始采集文件中，不用于暂停/恢复归因。wrapper 未产生 `cleanup complete` 标记，采集器停止后的进程核对未发现 MuJoCo、ROS 控制器或采集器仍在运行；仿真进程的精确停止时间 UNKNOWN。

**代码推断：** 同一策略帧中的 RA、候选、前后模式、动作来源及原因来自 `runModel()` 本周期；命令与来源周期由 `setCommand()` 在锁内一起复制。逐周期帧和 ROS `[ABS-POLICY-CMD]` 日志已在运行中观察到一致的来源周期。`recovery_command_issued` 只说明控制接口写出，不证明 MuJoCo 消费。

**仍为 UNKNOWN：** 本轮按键输入与时钟停更/恢复样本的因果边界不清，无法确认 Space 是否触发了这两次时钟变化；DDS timeout 安全停止后没有策略周期，故策略是否能在同一 session 恢复为有效 FRESH 未验证。当前协议不能区分仿真暂停与时钟发布器失效。控制器重新映射同名重建 shm 的运行时路径仍未实测。命令是否被 MuJoCo 消费未知。既有成功运行首周期 `risk_condition_met=false`，风险精确起点仍 UNKNOWN；该帧 sim_time 无效。S1-02 保留的 Agile/Recovery artifact 真实动作顺序 UNKNOWN 未重新探测。

## 完成条件与下一步

| S1-03 条件 | 判断 |
|---|---|
| 控制器读取 MuJoCo 时间并记录来源、有效性、年龄/生命周期 | 1,796 帧有逐帧 sim_time/来源/有效性/年龄；仿真重启时原始时钟序号与 sim_time 回退有记录。暂停恢复、控制器 remap 未实测 |
| 策略帧同周期包含 RA、前后模式、动作来源和原因 | 1,796 个连续周期已直接采集 |
| 模式转换与 Recovery 首次接口写出独立记录并关联来源周期 | 3 次 Recovery 首次接口写出对应来源周期 133/136/162；未证明 MuJoCo 已消费 |
| 暂停、恢复和重启最小验证 | 原始时钟重启回退只证明发布端重启边界；本轮记录到 STALE 周期、DDS timeout 强制 PASSIVE、恢复键和其后仿真时钟推进，但按键与停更/推进边界未配对，且策略恢复被安全停止阻断；该条件未完成 |
| 切换/动作行为未被观测改动改变，组件构建和运行 | 一次 paper 候选诊断实测阈值与切换；本次策略周期整体约 50.14 Hz。与 S1-02 基线中位数接近，但存在宽抖动，不能归因或排除观测开销 |
| 不把命令写出说成 MuJoCo 消费、不提前实现时间保持/正式组别 | 符合 |

Remaining UNKNOWN 的最小后续办法由 Director 决定。本轮没有再运行场景；若后续授权重验，应在一轮有明确时钟样本边界和确认窗口状态的短诊断中验证策略恢复，并保留安全机制。控制器读取同名重建共享时钟对象仍需单独运行时验证。无需重新探测模型形状或比较场景。

通俗说明：显示环境可用，既有短诊断记录到了时钟、切换和 Recovery 命令来源；原始日志中的四个坏时钟样本其实都是 STALE，首帧风险条件也没有成立。本轮按键尝试中策略时钟变旧并触发 DDS 安全停止；后来仿真时钟继续走了，但策略没有恢复，因此仍不能说暂停后控制器能正常恢复。

## Director 审阅补证（2026-10-08）

### 本轮修正

- 增加 `abs_switching::effectiveExitThreshold()`：paper 候选返回进入阈值 `E`，stabilized 候选返回原配置的滞回阈值 `E−0.03`。`runModel()` 用同一个返回值填 `cycle_trace_.exit_threshold` 并传给 `stepSwitching()`，避免日志与实际状态机参数分离。阈值数值和候选逻辑没有改变。
- Recovery episode 序号现在只由策略线程维护；进入决策只增加待发布序号，不提前更改控制线程读取的 `CommandTrace`。策略发布时在一个 `command_mutex_` 临界区内同时写 q/dq/kp/kd/tau、来源 session、策略周期、模式、动作来源、原因和 episode ID；`setCommand()` 在同一把锁内复制命令与这份来源快照。首次 issued 标记推迟到控制接口的关节写调用完成之后；因此没有实际写出的 episode 不产生 issued 日志。
- `enter()` 现在在同一锁内初始化待写命令。控制器每个策略周期重新打开共享内存名并比较当前对象与映射 fd 的 `st_dev/st_ino`；发现对象更换后重新映射、清空旧样本状态、递增本地时钟段，并将新对象首个可读样本标为 RESET（无效），后续新样本再按新段判断。对象尚未到 40 字节时不映射，避免初始化竞态。

### 针对性检查

- 临时 C++ 检查程序调用实际 `effectiveExitThreshold()` 与 `stepSwitching()`：paper 候选记录/退出使用 `E`；stabilized 保留 `E−0.03` 和原保持/滞回行为。断言全部通过。
- 既有切换逻辑直接核验 `p1_07_switching`：292 项通过。
- POSIX 临时 shm 探针执行 unlink 后同名重建，旧 fd 仍打开时新旧 `st_dev/st_ino` 不同；检测条件可识别该对象更换。该探针不等于 StateRL 的运行时重连验证。
- 更新后的 `rl_quadruped_controller` 增量构建成功，最终 stderr 为空。此前通过的 v2 frame/run-record 离线测试和构建结果保留；本轮重验只针对阈值/切换，重编译受影响控制器。
- 命令快照一致性通过源码路径核对与构建：策略值/元数据一起提交，控制线程一起读取，接口写完才更新 issued 去重状态。没有并发压力或运行时命令轨迹可供验证。

### 先前沙箱内预检记录（本轮提权前）

本轮没有启动采集器或再次运行 MuJoCo。环境预检已保存于 [display_preflight_retry_20261008.json](display_preflight_retry_20261008.json)：`DISPLAY=:0`，X socket 存在，但 `xdpyinfo -display :0` 仍无法打开；`XAUTHORITY` 文件存在，`xauth list` 报 `error in locking authority file`；GLFW、GL、GLX、EGL 运行库存在，`glxinfo`、`Xvfb`、`xvfb-run` 不存在。此前唯一最小 MuJoCo 启动记录 [mujoco.log](diagnostic_20261008/mujoco.log) 报 `could not initialize GLFW`。按任务要求保存预检后停止启动尝试，没有重复运行场景。

#### 上一轮执行器内定向会话与权限补证

补证记录：[display_preflight_session_20261008.json](display_preflight_session_20261008.json)。执行器用户为 `lidio`（UID 1000），与 Xauthority 文件和 socket 的属主一致；环境变量为 `DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority`、`XDG_SESSION_TYPE=x11`。执行器 PID 命名空间只看到沙箱进程，`loginctl` 访问 session bus 返回 `Operation not permitted`，因此不能检查宿主 Xorg 会话；`:0` 是否对应当前活跃桌面仍为 UNKNOWN。

`/tmp/.X11-unix/X0` 路径存在且权限为 0777，但从执行器对 AF_UNIX socket 做最小连接探针得到 `PermissionError: [Errno 1] Operation not permitted`。Xauthority 文件属主为 `lidio:lidio`、模式 0700 且可读；父目录模式 0711、执行器不可写。原路径 `xauth list` 因无法创建锁而失败；将文件临时复制到 `/tmp` 后 `xauth list` 成功（未输出 cookie），说明锁错误不能据此判为文件损坏。未删除锁文件、未改授权文件或权限、未使用 `xhost`。使用临时副本时 `xdpyinfo -display :0` 仍失败；执行器在 X 握手前已拒绝 socket 连接，所以这不是对宿主 X server 状态的验证。

GLFW、OpenGL、GLX、EGL 运行库存在；`glxinfo` 未安装。由于 X socket 连接预检未通过，没有做 GLFW 窗口预检，也没有启动采集器或重试 MuJoCo。此前唯一最小 MuJoCo 启动记录 [mujoco.log](diagnostic_20261008/mujoco.log) 报 `could not initialize GLFW`。当前证实的阻塞是执行器沙箱缺少宿主 X socket/session bus 访问权限；不能据此声称宿主显示不可用或 Xauthority 损坏。最小解法是在获准访问当前桌面 socket 和会话授权的执行环境内重跑预检；当前证据不支持安装 Xvfb 或修改系统服务。按要求停止重复启动。

### 时钟重连支持边界与影响

代码路径现在覆盖控制器再次进入 session、同一映射内的 `sim_time`/序号回退，以及 `/mujoco_sim_clock` 被删除并同名重建的 inode 更换。前两项由每 session 清零状态和单调时间/源序号/`sim_time` 检查处理；同名重建由逐周期重开名字并比较 inode 处理。unlink 到新对象出现之间仍会读旧映射，但样本随年龄变 STALE；新对象出现后会 remap，新段首样本置 RESET。**这些 StateRL 运行行为本轮没有仿真验证**；只证实 POSIX inode 探针与代码编译，不能写成重连已实测通过。

S1-02 之前测得约 50.14 Hz 是本次修改前的历史基线。本轮没有可运行策略周期，故本次实际 policy 频率及新增 `shm_open`/`fstat` 观测开销是否造成明显调度影响均为 UNKNOWN；没有把基线误写成新测量。

### 更新后的结论

以上是本轮沙箱外补证前的结论，现由下面的运行补证更新。

## 沙箱外显示访问与短诊断补证（2026-10-08）

### 显示/GLFW结果

本次 escalation 已生效：仍以 UID 1000、`DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority` 运行；X socket connect 成功，`xdpyinfo` 证实宿主 X.Org 1.21.1.4、1920×1080。未改授权文件、权限或系统服务。

Anaconda Python 的 ctypes GLFW 探针 `glfwInit` 成功但窗口创建失败。随后确认不是缺少 Mesa 文件：`iris_dri.so` 存在，但 Anaconda 的 `libstdc++.so.6` 不含系统 `libLLVM-15.so.1` 需要的 `GLIBCXX_3.4.30`。使用 `/usr/bin/g++` 在 `/tmp` 编译小型探针并临时将系统库目录置前后，隐藏 16×16 GLFW 窗口创建成功，OpenGL 4.6 context 可用；MuJoCo 二进制的 `ldd` 也指向系统 `libstdc++.so.6`。完整记录见 [display_glfw_preflight_retry_20261008.json](display_glfw_preflight_retry_20261008.json)。

第一次启动 MuJoCo 后，ROS launch 因临时 `go2_description` overlay 缺少 `xacro/robot.xacro` 失败；对应日志为 `diagnostic_display_retry_20261008/ros_launch_overlay_failure.log`。该次没有控制器。将原安装包资源目录只读链接到 `/tmp/s1_03_overlay`，保留原 `abs/rec` 候选配置后，第二次启动成功。第一次仿真只用于启动路径检查；不作为策略运行结果。

### 本次运行设置与模型

- 场景：`unitree_mujoco/scene_obstacle.xml`；控制命令沿用短诊断 stand-up 序列及向前 `lx=1.0`。
- 实际配置：`/tmp/s1_03_overlay/share/go2_description/config/abs/config.yaml`，`switching_mode=paper_faithful_switch`。非正式基础诊断，不是正式 A 组结果。
- 加载的 Agile、RA、Recovery artifact 分别为 `/tmp/s1_03_overlay/share/go2_description/config/abs/policy.pt`、`.../abs/ra_value.pt`、`.../rec/policy.pt`。ROS 日志直接确认 `policy.pt`、`ra_value.pt`、Recovery `policy.pt` 已加载；前两个完整路径由运行配置路径和加载代码共同确定。
- 每周期采集器在 MuJoCo 启动前已 `--check` 就绪，并以 1 ms 请求采样；切换证据位于 [diagnostic_display_retry_20261008](diagnostic_display_retry_20261008/)，暂停证据位于 [diagnostic_pause_resume_20261008](diagnostic_pause_resume_20261008/)。两目录含 `policy_frames.jsonl`、`sim_clock.jsonl`、`runtime_record.jsonl`、ROS/MuJoCo 日志及运行上下文。

### 直接观察到的运行事实

- 成功运行记录 1,796 个 `policy_cycle_id` 连续递增的周期（0–1795）。1,792 帧有效 FRESH；4 个无效帧均为 STALE（状态 2），周期 0、73、626、1329；没有 RESET 帧。首帧 `ra_value≈-0.386445`，`risk_condition_met=false`，首帧风险起点仍 UNKNOWN，且 sim_time 无效。其余有效帧的时钟年龄中位数 2.435 ms、最大 38.164 ms。
- 切换记录的 entry/exit threshold 均为 `-0.05`，符合 paper 候选；观察到 Recovery 进入周期 133/136/162，RA 分别 `-0.02824/-0.03606/-0.01645`，以及 Agile 返回周期 134/146/174，RA 分别 `-0.09752/-0.16393/-0.06371`。日志对应三次 Agile→Recovery→Agile。
- ROS `[ABS-POLICY-CMD] event=recovery_command_issued` 记录 episode 1/2/3 的首个 Recovery 接口写出，来源周期依次是 133/136/162；与策略帧模式转换周期逐个相符。这只证明控制接口写出，不证明 MuJoCo 已消费。没有 `monotonic_clock_order` veto；策略帧及 terminal record 未报告 safety fault。
- 同一个 collector 连续记录两次模拟器启动。第一段时钟到 `sequence=35146, sim_time=35.144s`；模拟器进程重启后记录到 `sequence=146, sim_time=0.144s`，直接可见发布端序号和仿真时间回退。这只证明发布端重启边界；1,796 个策略帧中没有 RESET 状态，不能写成控制器已观察 RESET 或已验证同名 shm 重连。
- 较早的定向暂停尝试记录于 `diagnostic_pause_resume_20261008/`：时钟和策略帧停止，自动清理前未发出恢复键，作为历史记录保留。本轮新尝试见 `diagnostic_pause_resume_final_20261008/`：目标窗口 PID 已核对，事件记录保存单调时间。周期 49–68 共 20 帧被标为 STALE。最后一个冻结时钟样本早于 pause 记录；第一个新递增样本又早于 resume 记录，因而不能把两次按键与停更/推进建立可靠因果配对。ROS 日志明确记录 DDS timeout 安全停止并强制 PASSIVE。resume 记录后 3 秒 sim_time 已到 21.826 s，但策略帧仍为 69 个；没有控制器策略恢复证据。安全检查保持开启。
- 以相邻策略帧的 `monotonic_ns` 计算每个 `dt_i=t[i+1]-t[i]`，并按 `1e9/dt_i` 得到点频率：1,795 个周期间隔的中位数 20.005 ms、P95 27.600 ms，最短 2.655 ms、最长 58.587 ms。相应整体频率 50.14 Hz，中位点频率 49.99 Hz；逐间隔频率 P05/P95 为 36.23/86.49 Hz。控制器启动日志报告 update rate 200 Hz，但逐帧测得的策略周期约 50 Hz；报告以策略周期记录为准。与 S1-02 的约 50.14 Hz 基线中位数接近，但抖动明显，单次运行无法判定抖动是否由新增记录造成，也没有无观测对照。
- 超时停止时 wrapper 输出 `cleanup complete`；之后进程核对未见 MuJoCo、ROS controller 或 collector 运行。安全 veto 未关闭。`runtime_record.jsonl` 的 terminal 汇总仍将碰撞覆盖和总仿真时长标为 UNKNOWN；它不替代逐策略帧中的有效时钟关联。

### 本轮结论与剩余项

沙箱限制已被实际验证并绕过；`DISPLAY=:0` 对应的宿主 X.Org 会话可用。既有逐周期时钟、paper 候选阈值、三次模式转换和 Recovery 首次接口写出证据保留。Director 指出的原始记录误读已纠正：四个无效样本均为 STALE，首帧 `risk_condition_met=false`；重启回退只证明发布端边界。本轮暂停/恢复尝试留下 STALE 帧与 DDS timeout 安全停止证据；虽观察到后续 sim_time 增长，但控制器没有恢复策略帧，且按键边界未能配对。同名 shm 控制器重连仍是“代码已实现，运行未验证”；单次频率记录不用于归因或排除观测开销。因此 S1-03 继续为 **ACTIVE / INCOMPLETE**，等待 Director 审阅；没有启动下一任务。

### 本轮修订文件与证据

- 本报告：纠正既有周期状态/首帧风险/发布端重启解释，并追加暂停/恢复尝试的直接结果、限制和安全 veto 证据。
- `tasks/S1-03.md`、`CURRENT_STATE.md`：同步准确状态及 UNKNOWN；S1-03 仍为唯一 ACTIVE Task。
- `diagnostic_pause_resume_final_20261008/pause_resume_context.json`：将先前的 success 字样更正为 INCOMPLETE，记录时钟边界配对失败和 DDS safety veto，并说明延长采集尾段与最终进程核对。
- 同目录中的 `policy_frames.jsonl`、`sim_clock.jsonl`、`runtime_record.jsonl`、`ros_launch.log`、`mujoco.log`、`pause_resume_events.jsonl`、`capture_start.json`、`capture_end.json` 作为原始记录保留，未改写。
