# Go2 原生交互运动工具

现场逐步操作与完整终端命令见 [现场命令](FIELD_COMMANDS.md)。首次只做最小速度、最短时长的直走与停止验证。

独立于论文策略控制的 Go2 原生运动工具。用于：

- 保留上一版短时直走，作为首次现场停止验证入口；
- 通过终端 `goto x y` 到初始坐标系中的固定点；
- 通过 `yaw deg` 转到相对初始朝向的角度；
- 运动过程中随时 `status`、`stop`、`quit` 或按 `Ctrl+C`。

不接入 D435i、VINS、雷达、SLAM、路径规划或自动避障；不验证论文控制器或策略。本轮只做离线修改与验证，未连接或操作机器人。

## 范围

程序只做：

- 使用 `unitree::robot::go2::SportClient::Move(vx, vy, vyaw)` 发送速度指令；
- 使用 `SportClient::StopMove()` 主动停止；
- 订阅 `rt/sportmodestate`，使用原生位置、线速度和航向估计；
- 读写终端命令和 CSV 记录。

程序不做：

- 不创建 `LowCmd` 发布器，不发布低层控制包；
- 不调用 `ReleaseMode`、`SelectMode`、`MotionSwitcher` 或 `SwitchJoystick`；
- 不关闭、释放或切换原生运动服务，不禁用原生遥控响应；
- 不启动论文控制器、策略推理、相机、VINS、雷达或定位程序；
- 不自动站立、趴伏、起立或切换步态；
- 不采集 `LowState`；
- 不做路径规划、障碍物避让、目标队列或无限重试。

现场由用户先用原装遥控器站稳并保持遥控器在手。程序默认不运动，必须先 `origin` 确认起点，再输入运动命令。

## 已核实的本机基础

- SDK2 安装在 `/home/lidio/Libraries/unitree_sdk2`，包含 `unitree/robot/go2/sport/sport_client.hpp` 和 `libunitree_sdk2.a`。
- `Move(vx, vy, vyaw)` 对应 API ID `1008`，参数以 JSON `{x, y, z}` 发送；`StopMove()` 对应 API ID `1003`。
- `rt/sportmodestate` 的消息类型为 `unitree_go::msg::dds_::SportModeState_`。本工具读取 `position()`、`velocity()` 和 `imu_state().rpy()[2]`。
- 本地 `go2w_sport_client.cpp` 注释写明：调用一次 `Move` 后机器人约 1 秒后停下；循环调用可持续运动；1 秒内没有新的 `Move` 会自行停下。Go2 示例则用周期线程持续调用。
- 本工具沿用 **10 Hz（每 100 ms）重发**，运动命令在任务期间持续发送，任务结束立即停止重发。

以下尚未在本台 Go2 上实测，不能当作已验证事实：

- Go2 本机固件是否同样是“1 秒内无新 `Move` 自动停下”；
- `vx/vy/vyaw` 的实际正方向、单位与响应。接口命名和常见宇树约定是机体系前向/横向/转向速度，预期 `m/s`、`m/s`、`rad/s`；本工具不据此保证实际方向和距离；
- `SportModeState.position()`、`velocity()`、`imu_state().rpy()[2]` 的坐标系、单位和正方向。程序原样记录，不换轴、不缩放；
- 原生 yaw 的符号。本工具按右手系、逆时针/向左为正的常见约定处理；首次 `yaw` 必须低速小角度核对，若方向相反立即停止并排查；
- 原生位置是否以米为单位、是否与 yaw 处在同一坐标系。`goto` 的尺度依赖这个假设。

原生位置是估计反馈，不是独立测量真值。到点后仍须用地面标记和卷尺核对实际误差。

### Move与StopMove返回值不同

2026-11-08进一步核对本机实际链接的SDK：Move使用无回包发送路径（noReply=true），返回0只表示SDK发送成功，不证明机器人服务接受或执行了行走。StopMove使用等待服务响应的路径，其返回码具有不同含义。不能把CSV中Move的api_return=0当作机器人执行成功；必须结合实际动作和状态反馈。0.20 m/s、6秒的现场尝试仍被用户观察为只前倾，当前未通过连续行走验证，不继续只靠加速/加时排查。

## 初始坐标系

收到有效原生状态后，用户确认机器人静止，再输入 `origin`：

- 起点位置为本地原点 `(0, 0)`；
- 初始前方为 `+X`；
- 初始左侧为 `+Y`；
- 初始朝向为 `0°`；
- 向左转为正角度；
- 位置单位米，角度单位度。

因此 `goto 1 0` 表示“起始位置前方 1 米”的固定点。`origin` 记录当时的原生位置 `(x0,y0)` 和原生 yaw `yaw0`，之后：

- `goto` 目标在任务开始时一次性转换为原生定位坐标系中的固定目标，不会每个周期重新设成“当前前方 1 米”；
- 每个控制周期用当前原生位置和当前原生 yaw，把固定目标的位置误差旋转到当前机体系，生成 `vx/vy`；
- `yaw` 目标是初始朝向加指定角度，转换为固定原生 yaw；每个周期用当前 yaw 计算带 ±180° 回绕的角误差；
- `goto` 只处理位置，不控制朝向；`yaw` 只处理朝向，不平移；
- 本轮不支持运行中重设原点。需要重新 `origin` 时，`quit` 后重新启动。

`±180°` 统一按回绕后的 `+π`（左转）处理。`yaw 190` 会规范化为相对初始朝向的 `-170°`，按最短角度转过去；`yaw 270` 等价于 `yaw -90`。

本工具内部按 `position` 为米、原生 yaw 为弧度、左转为正、`position`/yaw 同一右手坐标系进行目标转换和误差计算。以上单位、符号和同系假设尚未在本台 Go2 上实测；首次 `goto`/`yaw` 必须小范围核对，方向或尺度不符立即停止。

## 终端命令

普通命令按 Enter 执行；`Ctrl+C` 不需要 Enter。

| 命令 | 含义 |
| --- | --- |
| `status` | 显示原生位置、线速度、航向、状态时龄、当前任务、本地位置/朝向和 `stop_failed` 状态 |
| `origin` | 用户确认机器人静止后，记录初始位置和航向；本轮只允许一次 |
| `straight` | 保留上一版短时直走，使用 `--vx` 和 `--duration-s` |
| `goto x y` | 到初始坐标系中的固定点，单位米 |
| `yaw deg` | 转到相对初始朝向的角度，单位度 |
| `stop` | 终止当前任务并调用 `StopMove`；空闲时也执行；失败后可重复 `stop`，但不解除本会话失败状态 |
| `quit` | 先执行停止，再退出；本会话出现过停止失败时退出码为 3 |
| `help` | 显示命令帮助 |

运动期间终端仍继续读取：

- 允许 `status`、`stop`、`quit`；
- 新的 `goto`、`yaw`、`straight` 会提示先 `stop`，不建立任务队列；
- `origin` 在任务中会被拒绝；
- `Ctrl+C` 立即中断任务，停止并退出。

`origin` 和 `straight` 是为满足“人工确认起点”和“保留上一版入口”增加的命令；原命令表中的 `goto/yaw/stop/quit/status` 均保留。`straight` 沿当前机体系前方运动，不使用初始坐标系；`goto/yaw` 才使用初始坐标系。

### StopMove 失败后的会话状态

`StopMove()` 返回非零或抛出异常时，工具进入并保持“停止失败”状态：

- 立即提示用户用遥控器或现场方式确认机器人停止，并保持安全距离。
- 本会话拒绝 `straight`、`goto`、`yaw`，也拒绝 `origin`、`help` 和无法识别命令；只允许 `status`、`stop`、`quit`。
- 用户可再次输入 `stop`；工具不会自动重试，也不会自动恢复运动。
- 即使后续 `stop` 返回 0，本会话仍保持停止失败状态，运动命令仍被拒绝。要重新运动，必须 `quit` 后重新启动，并先现场确认机器人已停止。
- `quit`/`Ctrl+C` 仍会尝试最后一次 `StopMove`；只要本会话进入过停止失败状态，退出码为 3，即使最后一次 `StopMove` 返回 0。最后一次返回码仍记录在 CSV 和终端。
- 该状态表示停止请求失败，不代表机器人一定在运动；后续 `stop` 返回 0 也不证明机器人实际停住。现场确认优先。

## 控制参数

所有参数集中在 `native_motion_core.hpp`。以下是首次试验暂定值，不是已验证的机器人安全保证。

### 直走 `straight`

| 项目 | 值 | 说明 |
| --- | --- | --- |
| `vx` 默认 | `0.10 m/s` | `--vx` 覆盖 |
| `vx` 范围 | `[0.05, 0.50] m/s` | 超出直接拒绝，不启动 |
| `duration` 默认 | `2.0 s` | `--duration-s` 覆盖 |
| `duration` 范围 | `[1.0, 30.0] s` | 超出直接拒绝，不启动 |
| `vy`, `vyaw` | 固定 `0` | `straight` 仅前进 |

`0.50 m/s` 和 `30 s` 只是输入上限；本轮现场首项固定为 `0.20 m/s`、`6 s`，不自动执行更高速度、更长时长或连续任务。

### 通用和 `goto` / `yaw`

| 项目 | 值 | 状态 |
| --- | --- | --- |
| `Move` 重发周期 | `0.10 s`（10 Hz） | 沿用上一版 |
| 状态时龄期限 | `0.30 s` | 超过则终止任务并尝试停止 |
| 启动等待状态期限 | `10.0 s` | 超时不发送运动指令 |
| SDK 单次调用超时 | `1.0 s` | `SetTimeout(1.0f)` |
| 终端命令轮询周期 | `0.05 s` | 空闲时检查命令 |
| `goto` 最大目标距离 | `3.0 m` | 首次试验安全上限 |
| `goto` 位置容差 | `0.05 m` | 到达判据，不代表实际精度；此前10厘米容差的现场记录保持原样 |
| `goto` 位置增益 | `0.8 1/s` | 比例速度，接近目标减速 |
| `goto` 最大平移速度 | 默认 `0.30 m/s` | `--goto-max-speed`，范围 `0.05–0.50` |
| `goto` 接近目标速度下限 | 默认 `0.30 m/s` | `--goto-approach-speed`，范围 `0.05–max`；首轮防失速候选，不是硬件最低速度 |
| `goto` 最长运行时间 | `30.0 s` | 超时停止 |
| `yaw` 朝向容差 | `5°` | 到达判据 |
| `yaw` 朝向增益 | `1.5 1/s` | 比例角速度 |
| `yaw` 最大角速度 | `0.50 rad/s` | 上限 |
| `yaw` 最长运行时间 | `15.0 s` | 超时停止 |

`goto` 每周期按固定目标的位置误差计算机体系 `vx/vy`；速度随距离减小，但接近目标时不低于 `--goto-approach-speed`，以避免速度降入不能连续迈步的区间。0.30 m/s 是首轮已实测能走的候选，不是已验证的硬件最低速度；approach 下限也按同一候选先设，后续可单独降低验证。没有路径规划、障碍避让、自动重试或目标队列。原生位置噪声或机器人最小步速可能导致超时或停不到容差内，这属于现场参数验证内容。`ARRIVED` 只表示原生估计进入容差，不表示实际位置或角度达到该精度；现场仍须用地面标记、卷尺或量角器核对。

`straight` 的 0.50 m/s 上限只适用于 `straight`；`goto` 速度由 `--goto-max-speed`/`--goto-approach-speed` 独立配置，`yaw` 参数未变。

## 构建与离线自检

```bash
cmake -S /home/lidio/quadruped_robots/scripts/go2_native_motion \
  -B /tmp/go2_native_motion_build \
  -DCMAKE_PREFIX_PATH=/home/lidio/Libraries/unitree_sdk2 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/go2_native_motion_build --parallel 2
```

自检不初始化 DDS，不连接机器人：

```bash
/tmp/go2_native_motion_build/go2_native_motion_selftest
```

本轮自检覆盖新增或实质改变的部分：

- 非零初始航向下的 `goto` 目标转换、当前位置转换和机体系误差转换；
- 航向跨 ±180° 的回绕；
- 运动过程中接收 `stop`，且 `stop_request` 之后不再出现 `cmd` 行；
- `straight` 到时停止和 `stop` 中途停止；
- `goto` 到达、`goto` 超时后的停止；
- `goto` 的 `--goto-max-speed`/`--goto-approach-speed` 边界，以及接近目标时速度不低于 approach 下限；
- `yaw` 跨 ±180° 的转向方向；
- 任务中新的 `goto/yaw` 被拒绝，不排队；
- 未 `origin` 禁止运动、`origin` 只允许一次；
- `Move` 返回错误、状态超期、`Ctrl+C`、运动中 `quit`、启动超时；
- `StopMove` 返回非零或抛异常后保持失败状态：继续输入 `goto/yaw/straight` 被拒绝且不产生 `Move`；允许再次 `stop`，但不会自动重试；后续 `stop` 返回 0 也不恢复运动；退出码为 3；
- 命令解析和 `--vx/--duration-s` 上限（0.50 m/s、30 s 可接受；超出拒绝）。

自检使用替代 `Backend`、模拟时钟和脚本化终端命令，不创建 `rt/lowcmd` 发布器，不初始化真实 DDS。如需证明自检不依赖 SDK/DDS，可在配置时加 `-DCMAKE_DISABLE_FIND_PACKAGE_unitree_sdk2=ON`，此时只构建 `go2_native_motion_selftest`。

检查源码没有低层发包或模式切换调用：

```bash
grep -nE "ChannelPublisher|CreateSendChannel|rt/lowcmd" \
  /home/lidio/quadruped_robots/scripts/go2_native_motion/*.cpp \
  /home/lidio/quadruped_robots/scripts/go2_native_motion/*.hpp
grep -nE "ReleaseMode|SelectMode|SwitchJoystick|StandUp|StandDown|Damp\(|SwitchGait|MotionSwitcher" \
  /home/lidio/quadruped_robots/scripts/go2_native_motion/*.cpp \
  /home/lidio/quadruped_robots/scripts/go2_native_motion/*.hpp
```

预期无输出。实际 SDK 调用只有 `client_.SetTimeout()`、`client_.Init()`、`client_.Move()` 和 `client_.StopMove()`。

本轮离线结果：干净构建通过；自检全部 PASS；不依赖 SDK/DDS 的构建也通过；未初始化 DDS、未连接机器人。

## StopMove 返回 -1 的只读查询

同一构建目录还生成独立目标 `go2_native_readonly_query`，用于下一次现场区分原生模式、服务状态和 Sport API 版本问题。它订阅 `rt/sportmodestate`，并发送 `CheckMode`、`ServiceList`、`GetServerApiVersion` 查询请求；因此不是“只订阅、完全不发包”。它不调用运动、停止、站立、趴伏、阻尼、步态、模式切换、`ServiceSwitch`、`SwitchJoystick`、`SetSilent` 或 `SetReportFreq`，退出时不发送停止或恢复指令。

```bash
env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/home/lidio/Libraries/unitree_sdk2/lib \
  /tmp/go2_native_motion_build/go2_native_readonly_query \
  --domain 0 --interface enp7s0
```

查询有有限等待；失败或空值打印 `NOT OBTAINED`，不补默认值。完整现场命令、输出示例和退出说明见 [FIELD_COMMANDS.md 第 8 节](FIELD_COMMANDS.md)。本轮只构建，未运行该入口。

本次 StopMove 返回 -1 的离线排查与证据见 [StopMove=-1 排查报告](../../docs/thesis_project/real_go2/20261108_STOPMOVE_RETURN_MINUS1_INVESTIGATION.md)。

## 现场运行

先用实际网络与 DDS 配置替换参数。2026-11-04 本机实际通信使用 domain `0` 和网卡 `enp7s0`，运行前仍应确认网卡仍是当前 Go2 链路。

### 启动命令：0.20 m/s、6 秒直走观察

此前 0.20 m/s、3 秒已经能前进，但实际约 10 cm，尚未确认连续行走。本次保持 0.20 m/s，把一次观察延长到 6 秒，用于区分起步时间不足与持续行走异常。只做一次，不自动执行更高速度、更长时长或连续任务。

```bash
env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/home/lidio/Libraries/unitree_sdk2/lib \
  /tmp/go2_native_motion_build/go2_native_motion \
  --domain 0 --interface enp7s0 \
  --vx 0.20 --duration-s 6.0 \
  --goto-max-speed 0.30 --goto-approach-speed 0.30 \
  --csv /home/lidio/quadruped_robots/logs/go2_native_motion/$(date -u +%Y%m%d_%H%M%S)/native_motion.csv
```

省略 `--csv` 时默认写到当前目录的 `logs/go2_native_motion/<UTC时间戳>/native_motion.csv`。程序会创建父目录，并在终端再次打印最终路径；启动时会明确显示本次 `straight` 的 vx 和 duration。

### 第一次现场顺序

1. 确认空旷区域、地面状况、线缆余量。
2. 用户用原装遥控器把 Go2 站稳，保持遥控器在手。
3. **先确认停止入口**：在电脑发送运动前，用用户已确认的遥控停止方式验证机器人能响应。不要用本项目的“按键 9”代替；本工具不提供独立急停。
4. 运行上面的 0.20 m/s、6 秒命令。
5. 等待终端显示有效原生状态，确认终端打印的本次 vx=0.200 m/s、duration=6.000 s。
6. 确认机器人静止后输入 `origin`，记录起点。
7. 可选但建议：机器人静止时输入 `stop`，确认终端能收到电脑侧 `StopMove` 返回；该命令不发送 `Move`。若出现 `StopMove 失败`，立即按停止失败流程处理，不要继续 `straight/goto/yaw`。
8. 输入一次 `straight`，观察起步后是否连续迈步、到时是否实际停住，并用地面标记/卷尺测量实际位移；理论指令距离为 `0.20 × 6 = 1.2 m`，不代表实际距离。
9. 若仍只有一次迈步、持续前倾或姿态异常，立即停止试验，不自动继续增加速度、时长或连续任务。
10. 用户确认实际停住后输入 `quit` 退出。
11. 只有本次直走和停止结果审阅通过后，才安排 `goto`、`yaw` 或更高速度/更长时长。

终端示例：

```text
本次 straight 参数: vx=0.200 m/s, duration=6.000 s（vy=0, vyaw=0；10 Hz 重发）
goto 参数: vmax=0.300 m/s, approach=0.300 m/s（approach 为首轮防失速候选，不是已验证的硬件最低速度）
...
命令> origin
起点已记录: native x=... y=... yaw_raw=...
...
命令> straight
任务开始: straight vx=0.200 duration=6.000
...
任务结束: straight COMPLETED
停止流程: reason=COMPLETED StopMove返回=0
命令> quit
```

### 短距离 `goto` 与 `yaw`

短距离 goto 首试用 0.5 米，并显式使用 `--goto-max-speed 0.30 --goto-approach-speed 0.30`（也是当前默认值）。重新 `origin` 后输入：

```text
命令> goto 0.5 0
任务开始: goto local=(0.500,0.000) native=(...) vmax=0.300 approach=0.300
...
任务结束: goto ARRIVED
```

`approach=0.30` 是接近目标时的防失速下限候选，不是已验证的硬件最低速度；0.30 也只是当前已实测能走的候选最大速度。观察是否连续迈步、到时是否实际停住、是否明显过冲，并用地面标记/卷尺测量实际位移。只做这一次，不自动提高速度、降低下限或连续追加 goto。完整独立启动命令见 [FIELD_COMMANDS.md 第7节](FIELD_COMMANDS.md)；`--vx/--duration-s` 只影响 `straight`，不会被 `goto` 使用。

再用 1 米目标验证固定坐标系：

```text
命令> goto 1 0
任务开始: goto local=(1.000,0.000) native=(...) vmax=0.300 approach=0.300
...
任务结束: goto ARRIVED
```

到点判据来自原生位置估计，不代表实际到点。用地面标记和卷尺核对实际误差，并用 `status` 查看本地位置。

再用小角度验证转向方向：

```text
命令> yaw 30
任务开始: yaw local_deg=30.000 native_yaw_raw=...
...
任务结束: yaw ARRIVED
```

短距离 `goto 0.5 0` 首试通过、确认连续迈步和停止正常后，才考虑 `yaw 90`、更远 `goto`。建议在独立、已审阅的会话中再先 `goto 1 0` 验证前向，用 `goto 0 1` 验证左侧 Y 方向，再用 `yaw 30` 验证左转。任一方向与预期不符，立即停止，不要扩大目标。

任何异常都先停止并回到直走/停止验证，不要直接扩大目标。

若终端出现 `StopMove 失败`：立即用遥控器或现场方式停止机器人，保持安全距离；本会话只允许 `status`、`stop`、`quit`，不要继续输入 `goto/yaw/straight`；可再次 `stop` 尝试，但工具不会自动重试或恢复运动；确认机器人停止后 `quit`。如需继续，重新启动并重新 `origin`。

## 停止、终端和遥控

- **停止顺序**：任何停止触发先终止当前任务并清空任务状态，后续循环不会再次发送 `Move`；然后调用一次 `StopMove()`，记录返回码或异常；再回到空闲。正常停止后只有用户重新输入运动命令才会再次运动；若 `StopMove` 失败，本会话不再接受运动命令。
- **正常到达 / `straight` 到时 / 超时**：先结束任务，再 `StopMove`，回到空闲。
- **`stop`**：终止当前任务并调用 `StopMove`；空闲时也执行停止调用并记录。
- **`quit`**：先执行停止流程，再退出；本会话有停止失败时退出码 3。
- **`Ctrl+C`**：中断任务，执行停止流程并退出；状态就绪后即使空闲也会尝试 `StopMove`；本会话有停止失败时退出码 3。
- **状态超期 / `Move` 错误/异常**：进入同一停止流程，回到空闲；不会自动重试。若随后的 `StopMove` 失败，进入停止失败状态。
- **`StopMove` 失败**：进入会话内保持的停止失败状态；立即提示现场停止机器人；拒绝运动命令，只允许 `status`、`stop`、`quit`；用户可再次 `stop`，但工具不自动重试。即使后续 `StopMove` 返回 0，本会话仍不恢复运动；退出码为 3。

终端读取在独立线程中；主循环每 0.05 s 空闲轮询、任务期间每 0.1 s 控制周期检查命令。SDK 单次调用超时 1.0 s，因此一次阻塞的 `Move` 最多让停止命令等待约 1 秒加一个控制周期，不会无限等待。若 SDK 请求本身超过超时仍不返回，停止也会被拖住；这是外部接口风险，程序不伪造已停止。

本工具不关闭原生运动服务，不调用 `SwitchJoystick`，不禁用原生遥控响应。遥控停止仍由原生服务处理。高层遥控和电脑 `Move` 指令可能冲突，本工具不能宣称遥控必然具有停止优先权。运动窗口内使用遥控停止时，请同时输入 `stop` 或按 `Ctrl+C` 终止电脑重发；这条电脑路径不是独立急停，不能替代现场人员、支撑和遥控器。

出现任何异常时，用户优先用遥控器停止机器人并确保现场安全，之后再处理程序、CSV 或后续采集。

电脑掉电、`kill -9`、强制关机、操作系统崩溃或网线断开时，本程序的清理代码不能保证执行。这些情况下不能依赖程序调用 `StopMove`。

程序退出、`StopMove` 返回成功、速度指令为零，都不证明机器人已经实际停住。必须由用户在现场确认。

## CSV 记录

默认路径见上文。列定义：

| 列 | 含义 |
| --- | --- |
| `wall_time_utc` | 本机系统时间，UTC，毫秒 |
| `mono_s` | 本机单调时钟，会话开始后的秒数 |
| `record_type` | `cmd` 或 `event` |
| `task` | `none`、`straight`、`goto`、`yaw` |
| `state_seq`, `state_age_s` | 原生状态接收序号和该样本的本机接收时龄 |
| `origin_native_x/y/yaw` | `origin` 记录的固定原点 |
| `target_local_x/y`, `target_local_yaw_deg` | 初始坐标系中的固定目标 |
| `target_native_x/y/yaw` | 任务开始时转换出的原生固定目标 |
| `local_pos_x/y`, `local_heading_deg` | 当前原生状态在初始坐标系中的位置和朝向 |
| `cmd_vx_mps`, `cmd_vy_mps`, `cmd_vyaw_radps` | 本次发送的速度指令 |
| `native_pos_x/y/z` | 原生 `position()` 原样值 |
| `native_vel_x/y/z` | 原生 `velocity()` 原样值 |
| `native_yaw_raw` | 原生 `imu_state().rpy()[2]` 原样值 |
| `api_return` | `Move`/`StopMove` 的 SDK 返回码 |
| `event` | `state_ready`、`terminal`、`terminal_rejected`、`origin_set`、`status`、`task_start`、`task_end`、`stop_request`、`stop_result`、`end` |
| `detail` | 原始终端命令、任务结束原因、异常或停止失败信息 |

`task_end` 的 `detail` 使用 `ARRIVED`、`COMPLETED`、`TIMEOUT`、`STOPPED`、`INTERRUPTED`、`STATE_STALE`、`MOVE_API_ERROR`、`QUIT`。`stop_request` 之后不再写 `cmd` 行；`stop_result` 的原生字段是 `StopMove` 返回后的快照，仍不是实际停住的证明。

停止失败后，`terminal_rejected` 和 `status` 的 `detail` 可能是 `stop_failed`；退出时 `end` 的 `detail` 可能是 `QUIT_STOP_FAILED` 或 `INTERRUPTED_STOP_FAILED`，表示本会话的停止失败状态在退出时仍未解除。`stop_result` 记录每次 `StopMove` 的返回码；后续 `stop` 返回 0 也不会解除本会话的失败状态。

位置、速度和航向都是原生估计反馈，不是独立测量真值；坐标系和单位仍待本机核实。

## 退出码

| 退出码 | 含义 |
| --- | --- |
| `0` | `quit` 或 `Ctrl+C` 结束，本会话没有停止失败，且最后一次 `StopMove` 返回 0 |
| `1` | 启动阶段没有有效状态或收到中断；未进入运动 |
| `2` | 参数错误、DDS 初始化失败或程序运行异常（本会话没有停止失败） |
| `3` | 本会话进入过停止失败状态（会话内保持，即使后续 `StopMove` 返回 0），或最后一次 `StopMove` 返回非零/抛出异常 |

任务中的到达、超时、`stop`、`Move` 错误和状态超期不会直接结束进程；它们进入停止流程后回到空闲，等待用户下一条命令。停止失败状态表示停止请求失败，不等于机器人一定在运动；退出码和 `StopMove` 返回码都不代表机器人已实际停住。
