# S2-02 执行报告：MuJoCo 实时状态面板

**最新状态：COMPLETE — Director 有条件通过（2026-10-10）。** 通过范围：MuJoCo 只读实时面板已实现；实际 LIVE 截图清晰、布局可读；显示模式、Body XY speed、RA、实际阈值、仿真时间和数据状态；非运行及无效/过期状态不显示旧策略值；独立终端 HUD 保留且控制逻辑未改变。

保留限制：目标距离为 N/A；Body XY speed 与倾斜时的世界水平速度不同；未进入 RL、停止、过期状态只有离线检查、没有真实截图；本次 PASSIVE 后策略停止确认未获证；MuJoCo 命令消费未验证。面板通过不代表 A 组避障性能通过。下文保留最初的 INCOMPLETE 和失败记录，均为当时状态，不覆盖本次审阅结论。

## 本轮定向修订与显示补证（2026-10-10）

### 显示语义和布局修正

- 速度标签改为 **Body XY speed**。它是实时帧 `lin_vel[0:2]` 的范数；该向量来自完整姿态旋转后的机身坐标速度，因此不是世界水平速度。机器人有 roll/pitch 时，机身 XY 分量范数与世界 XY 投影可能不同。本轮不扩大帧协议或控制链。
- LIVE 且 RL 正在运行时才显示速度、Agile/Recovery、RA、实际进入/退出阈值和仿真时间。新鲜帧但未进入 RL 显示 `NOT IN RL`；`rl_entered=1, rl_active=0` 显示 `POLICY STOPPED`；安全故障或 fault 状态显示 `FAULTED`。这三类状态不展示旧策略数值。MISSING、INVALID、STALE 继续只显示状态和适用时的帧年龄。
- 视觉检查的实图中，面板位于 MuJoCo 中央场景视口左上角，文字清楚可读；左右控制菜单在视口之外，没有遮挡。面板向下避开实时倍率提示；帮助文字启用时面板隐藏，避免与帮助内容争用同一左上角。
- 当前帧没有可信目标坐标，目标距离仍显示 N/A。未在窗口运行中观察到 Recovery；Recovery 只由离线状态检查覆盖。

### 离线核对与构建

- `s2_02_live_panel_test` 调用生产 `classify()` / `render()`，覆盖 LIVE、STALE、MISSING、INVALID、未进入 RL、策略停止、FAULTED、Agile 和 Recovery，并确认非运行状态不显示策略值。测试目标显式启用断言（`-UNDEBUG`）。定向 CTest 通过 1/1。
- 复用此前真实运行 `4fff5053b6ed486a8a0a8dc74ff30c88` 的 cycle 38 JSON 载荷，按实时帧 v2 布局重序列化为 536 字节。Python `RuntimeFrame` reader 判为 LIVE，C++ 生产 classifier/render 离线探针接受该帧。此验证核对了记录载荷的兼容性；历史文件保存的是解析后的字段，不含当时原始共享内存字节。
- MuJoCo 增量构建成功：`unitree_mujoco`、`s2_02_live_panel_test`。脚本 Python 编译检查通过。控制器、模型、阈值、动作和安全规则均未修改。

### 实际窗口和短显示运行

- 显示预检：既有 `DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority` 下 X11 连接通过；临时系统 GLFW 16×16 隐藏窗口创建和 OpenGL context 通过。预检 renderer 为 `Intel / Mesa Intel(R) Graphics (RPL-P) / OpenGL 4.6 Compatibility Profile, Mesa 23.2.1-1ubuntu3.1~22.04.4`；MuJoCo 诊断进程本身未单独写出 renderer 字符串。`glxinfo` 未安装；没有为此安装组件，使用短小的系统编译器/GLFW 探针验证。权限仅用于本机桌面和此次短运行，没有修改 X 授权、系统权限、驱动或全局环境。
- 截图通过 MuJoCo 自己的 `mjr_readPixels` 从实际渲染 framebuffer 读取 1280×720 RGB 像素，保存原始 PPM，再无损转换为 PNG；不是合成或重绘画面。实图：[live_panel.png](diagnostics_20261010/display_verification/live_panel.png)，原始像素：[live_panel.ppm](diagnostics_20261010/display_verification/live_panel.ppm)。画面显示 LIVE、Agile、Body XY speed、RA/进入/退出阈值、目标距离 N/A、Sim time 和帧年龄。面板可读且没有与菜单或倍率提示重叠。截图中的 frame age 为 3 ms；截图没有把 session/cycle ID 烧入图像，因此像素帧与运行记录某一特定策略周期之间的精确配对仍 UNKNOWN。来源绑定由本次 controller/session 运行证据另行确认。
- 一次短显示核验 run：`99ae35aff52d4cfc82d1d38d743934c4`，场景 `scene_obstacle.xml`，A=`paper_faithful_switch`，配置 `group=A, E=-0.05, exit=-0.05, hysteresis=false, hold=false`，目标 `(7,0) m`，运动观察上限 0.05 仿真秒、墙钟保护 35 秒。配置、三个模型绝对路径、唯一 controller/writer 和 session 由来源匹配器确认为一致；controller loaned 足接触接口顺序为 `FR, FL, RR, RL`。16 个连续记录帧覆盖 cycle 39–54、sequence 80–110、session `18692744512216`；terminal cycle 54 payload 已持久化，终局为 `DIAGNOSTIC_SIM_TIME_LIMIT`。运动期间无安全 veto；这不证明无碰撞，也不是避障性能结果。
- 收尾发出 PASSIVE 请求，日志随后出现由该命令触发的 `[HARD-STOP] ... forcing PASSIVE`；FSM 策略停止确认字段为 false（观察到请求后仍有 250 ms 输出）。supervisor 记录 MuJoCo 和 ROS launch/controller 均以 exit code 0 退出、无强杀、无 closeout error。该停止确认限制保留，不能写作策略已确认停止或 MuJoCo 消费确认。
- 当前显示运行取得了 LIVE 画面；未进入 RL、停止、STALE 状态的真实窗口截图未保存，本轮以生产函数离线用例覆盖这些状态，没有再启动运行。

### 本轮证据位置

- [run_context.json](diagnostics_20261010/display_verification/99ae35aff52d4cfc82d1d38d743934c4/run_context.json)
- [runtime_record.jsonl](diagnostics_20261010/display_verification/99ae35aff52d4cfc82d1d38d743934c4/runtime_record.jsonl)
- [terminal.json](diagnostics_20261010/display_verification/99ae35aff52d4cfc82d1d38d743934c4/terminal.json)
- [process_facts.json](diagnostics_20261010/display_verification/99ae35aff52d4cfc82d1d38d743934c4/process_facts.json)
- [MuJoCo log](diagnostics_20261010/display_verification/99ae35aff52d4cfc82d1d38d743934c4/mujoco.log)；[ROS launch log](diagnostics_20261010/display_verification/99ae35aff52d4cfc82d1d38d743934c4/ros_launch.log)
- [显示预检结果](graphics_preflight_20261010.json)

下文首次执行部分保留了当时的原始状态与失败记录；后续截图和修正以本补充节为准。

## 首次执行记录（由上方本轮补证更新）

本节及其后首次执行记录保留 2026-10-10 最初提交时的事实。其截图缺失结论仅指当时尝试；本轮真实截图与语义修订见上方补充节。

## 实现

- 在 MuJoCo 自带的 `mjr_overlay` 绘制路径增加左上角简洁面板，位于原实时倍率提示下方。独立终端 HUD `scripts/abs_live_hud.py` 未修改。
- 渲染线程每帧打开当前 `/mujoco_rt_frame` 共享对象，最多尝试三次 seqlock 快照；映射只在该次读取期间存在，不缓存旧 session 的映射。布局长度、magic/version、偶数稳定 sequence、AUTHORITATIVE_RUNTIME 来源、帧一致性、有限值和单调时间新鲜度按实时帧合同拒绝无效数据。MISSING、INVALID、STALE 状态只显示状态文案，不保留旧 RA、速度或模式值；STALE 同时显示帧年龄，超时为 500 ms。controller 正常析构会先使共享帧失效；controller 异常停止后仍有效但停止更新的帧会按年龄转为 STALE。
- RL 已进入且正在运行时显示 `hypot(lin_vel[0], lin_vel[1])` 的 **Body XY speed**、Agile/Recovery、帧中实际 entry/exit 阈值及 RA、仿真时间、帧年龄。`StateRL::runModel()` 将世界线速度旋到机身坐标后写入帧；机身倾斜时该值不等于世界水平投影，因此标签不称为世界水平速度。
- 新鲜帧但 RL 未进入、策略停止或 FAULTED 时只显示对应状态，不显示速度、RA、模式相关旧策略值或仿真策略数据。MISSING、INVALID、STALE 同样不显示旧策略值。
- 当前实时帧不含可靠目标坐标。面板明确显示 `Goal distance: N/A (goal absent from runtime frame)`，不从命令速度或画面位置猜目标距离。仿真时间仅在帧标记 sim clock 有效时显示，否则为 N/A；它来自 MuJoCo 仿真秒，不是墙钟。
- 标签使用简短英文，避免引入额外字体依赖。

## 针对性核验与构建

- 离线检查调用生产 `abs_mujoco_live_panel::classify()` / `render()`，覆盖 LIVE 字段格式与速度模长、MISSING 不残留旧字段、超过阈值的 STALE 不显示旧字段、错误帧长、错误 version、非授权 source、非有限 RA、FAULTED 状态、Recovery 模式和新 session 快照。`s2_02_live_panel_test` 通过（1/1）。
- MuJoCo 增量构建通过：`cmake --build unitree_mujoco/simulate/build2 --target unitree_mujoco s2_02_live_panel_test -j2`。没有修改控制器、切换条件、配置、模型、动作或安全检查。
- 短诊断运行时已使用本轮首次构建、具备面板和基础实时帧分类的 MuJoCo 二进制。随后为让状态/标志校验与 Python reader 完整一致，收紧了 classifier 并重新构建、重跑离线检查；这个最终 classifier 未再次接入运行帧验证，因为任务只授权一次场景运行。
- 屏幕截图获取尝试：CUA 桌面 inventory 返回 `apps=[]`；环境没有 `wmctrl` / `xdotool`；`xwd -root` 在当前 X server 返回 `BadColor (invalid Colormap parameter)` 并产生空文件，空临时文件已清理。没有用合成画面替代真实截图。

## 唯一一次短诊断

- Run ID：`4fff5053b6ed486a8a0a8dc74ff30c88`；场景 `scene_obstacle.xml`；A 组 `paper_faithful_switch`；配置 `group=A, entry=-0.05, effective exit=-0.05, hysteresis=false, hold=false`；目标 `(7.0, 0.0) m`；运动仿真上限 6 s、墙钟保护 50 s。模型路径与来源证据位于本次 `run_context.json`、`ros_launch.log`。
- 新构建的 MuJoCo 进程实际通过现有 GLFW 环境启动，日志包含 MuJoCo 3.3.3 场景加载，未见 GLFW/OpenGL 启动错误。renderer 本次没有被记录。此事实证明程序运行到场景加载，但由于没有可保存的屏幕截图，不能证明各标签在屏幕中的具体位置、遮挡和可读性。
- 实际 controller 来源行确认 package/config/model 路径、唯一 manager PID `78745`、A 组与目标；接触接口门禁观察到 `FR, FL, RR, RL`。记录有 159 个 LIVE 策略帧、同一 session `17837954878104`，策略周期范围覆盖到 cycle 197，RA、模式、entry/exit 阈值、世界位姿和有效 sim time 都在真实记录载荷中。代表帧：cycle 38，Agile，RA `-0.3049`，阈值 `-0.05/-0.05`，水平速度 `0.3949 m/s`，sim time `23.154 s`；terminal cycle 197，Recovery，RA `0.0898`，水平速度 `0.3874 m/s`，sim time `26.334 s`。这些数值由记录载荷计算，不能替代对窗口像素的直接视觉核验。
- run 以运动期障碍碰撞终止，terminal frame 已保存；策略停止确认日志为真。收尾记录 ROS launch 和 MuJoCo 均 exit code 0，无强杀、无 closeout error。此诊断仅检查显示数据链，不用于 A 切换效果或避障性能结论。

原始记录与收尾证据：

- [run_context.json](diagnostics_20261010/4fff5053b6ed486a8a0a8dc74ff30c88/run_context.json)
- [runtime_record.jsonl](diagnostics_20261010/4fff5053b6ed486a8a0a8dc74ff30c88/runtime_record.jsonl)
- [terminal.json](diagnostics_20261010/4fff5053b6ed486a8a0a8dc74ff30c88/terminal.json)
- [process_facts.json](diagnostics_20261010/4fff5053b6ed486a8a0a8dc74ff30c88/process_facts.json)
- [MuJoCo log](diagnostics_20261010/4fff5053b6ed486a8a0a8dc74ff30c88/mujoco.log)
- [ROS launch log](diagnostics_20261010/4fff5053b6ed486a8a0a8dc74ff30c88/ros_launch.log)

## 首次执行时的剩余缺口与清理（历史）

- **未完成项：**真实窗口截图与屏幕视觉核验缺失，因此看不到本次实际 LIVE 面板画面，也无法确认面板不遮挡窗口原有提示。桌面 inventory 与 XWD 失败证据如上。按任务一次运行上限，没有再启动场景。
- **字段边界：**目标距离在当前 RT frame 没有权威来源，显示 N/A；真实 renderer 未记录。MuJoCo 是否消费控制器写出的 Recovery 命令仍不在本任务的证明范围内。
- **清理：**监督器记录本次 MuJoCo 与 ROS launch/controller 正常退出，退出码均为 0；之后定向进程核对没有 `unitree_mujoco`、controller manager、`rviz2` 遗留。未删除持久共享内存对象，未影响其他会话。

首次提交时的结论为 INCOMPLETE：当时尚未取得截图。后续定向修订、实际截图及 Director 审阅结论见报告顶部；该历史状态保留，不代表当前状态。
