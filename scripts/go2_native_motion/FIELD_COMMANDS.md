# Go2 原生终端控制：现场命令

> **当前续接（2026-11-08）：**0.20 m/s、6秒的直走已经执行，用户仍观察为只前倾；之后用户报告0.30 m/s可正常行走。下文0.20/6直走步骤保留为历史操作，不再作为当前首项重复。当前下一项为第7节的独立goto候选：vmax=approach=0.30 m/s，单次 `goto 0.5 0`。须先确认此前0.30直走能实际停稳。两个速度参数相等时为恒定速度至进入0.05米位置容差后停止，不是逐渐减速；若明显越过目标、反向追赶或姿态异常，用户立即stop/遥控停止，不任其反复找点。当前未证明实际到点精度。

> **容差更新：**用户要求将goto位置容差改为5厘米（0.05 m）。重新启动本次构建的程序，终端应显示 `goto 位置容差=0.050 m`。速度仍沿用0.30 m/s候选，历史10厘米容差试验不改写。移动到新场地后，在静止状态重新origin；停稳后先读取status和测量，再趴伏或搬动。若越过目标后反向追赶或反复调整，立即stop，本次不放任持续纠偏。5厘米是原生估计判据，不是实际精度保证。

本文件供用户在电脑本地终端逐步操作。首次只做 `straight` 短时直走与停止，不运行论文控制器或低层控制。没有外接传感器要求。

命令不需要 RTK 或 sudo。不要在 `unitree@ubuntu` 的 SSH 终端中运行；本工具和 SDK 位于电脑 `/home/lidio/` 下。

## 1. 现场准备

- Go2 原生运动服务保持运行，电脑通过网线连接机器人。
- 场地空旷，线缆有余量，机器人与周围人员保持距离。
- 用户使用原装遥控器站稳机器人，遥控器拿在手中，先确认已核对的原生停止操作能够响应。
- 站立后还须由用户按熟悉的遥控操作确认已进入原生可行走状态，摇杆归中；不能把“已站立”自动等同于“已解除遥控锁定”。2026-11-08先前StopMove返回-1，用户确认此前锁定；进入可行走状态后的同工具静止stop返回0，记录见 `logs/go2_native_motion/20261108_044903/native_motion.csv`。该结果不代表运动后的实际停止已验证。
- 不同时运行其他向机器人发送运动指令的程序。
- 以下命令逐步输入；不要一次粘贴整套交互命令，以免动作连续开始。

## 2. 电脑本地检查网卡

```bash
cd /home/lidio/quadruped_robots
ip -br addr show enp7s0
ip route get 192.168.123.18
```

历史配置是 `enp7s0`、电脑地址 `192.168.123.99/24`、SDK DDS domain `0`。确认当前仍使用该机器人网口；路由查询应显示 `dev enp7s0`。这一步只确认网络配置，不证明 SDK 通信或停止可用。若配置不符，先核对，不照抄启动参数。

## 3. 构建（已有最新产物则跳过）

执行器已报告构建和21项离线自检通过；若产物仍在且源码未变，不必重复构建、自检。`/tmp` 产物丢失或源码更新时，再执行：

```bash
cmake -S /home/lidio/quadruped_robots/scripts/go2_native_motion \
  -B /tmp/go2_native_motion_build \
  -DCMAKE_PREFIX_PATH=/home/lidio/Libraries/unitree_sdk2 \
  -DCMAKE_BUILD_TYPE=Release
```

上一步成功后：

```bash
cmake --build /tmp/go2_native_motion_build --parallel 2
```

构建成功后做离线自检，不初始化 DDS、不连接机器人：

```bash
/tmp/go2_native_motion_build/go2_native_motion_selftest
```

任一步失败就停止，不继续现场启动。

## 4. 历史直走启动：0.20 m/s、6秒（已执行，不重复）

此前 0.20 m/s、3 秒已经能前进，但实际约 10 cm，尚未确认连续行走。本次保持 0.20 m/s，把一次观察延长到 6 秒，用于区分起步时间不足与持续行走异常。只做一次，不自动执行更高速度、更长时长或连续任务。

允许范围是 `vx 0.05–0.50 m/s`、`duration 1–30 s`；本次只用 `0.20 m/s / 6.0 s`。

在电脑本地终端执行：

```bash
cd /home/lidio/quadruped_robots
motion_log_dir="/home/lidio/quadruped_robots/logs/go2_native_motion/$(date -u +%Y%m%d_%H%M%S)"
env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/home/lidio/Libraries/unitree_sdk2/lib \
  /tmp/go2_native_motion_build/go2_native_motion \
  --domain 0 --interface enp7s0 \
  --vx 0.20 --duration-s 6.0 \
  --goto-max-speed 0.30 --goto-approach-speed 0.30 \
  --csv "$motion_log_dir/native_motion.csv"
```

启动会建立原生 SDK 通信、订阅状态，但不会自动运动。程序自动创建日志目录并打印 CSV 路径；状态就绪后会打印本次 `straight` 的 vx=0.200 m/s、duration=6.000 s，以及 `goto` 的 `vmax=0.300 m/s`、`approach=0.300 m/s`。等待显示状态就绪与 `命令>`；无状态或启动报错时，不执行后续运动操作。

**`--vx` 和 `--duration-s` 只配置 `straight`，不会限制 `goto` 和 `yaw`。**

## 5. 在程序的“命令>”中逐步操作

以下不是 Bash 命令，必须在工具运行后的交互提示处输入，每条按 Enter。不要整段粘贴。

### 查看状态

```text
status
```

检查状态在更新，且没有停止失败提示。原生反馈仍是估计值。

### 用户确认机器人静止后记录起点

```text
origin
```

程序记录一次初始位置和朝向，不会运动。本会话不能再次记录原点。

### 静止时检查电脑停止调用

```text
stop
```

应看到 `StopMove返回=0`；这只能确认接口返回，用户仍需确认机器人状态。若停止失败，先现场停止机器人，不继续运动；本会话即使再次 stop 成功，也不再接受运动命令。

### 首次直走观察

确认现场准备完成后输入：

```text
straight
```

工具沿当前机体前方，以 `0.20 m/s` 持续发送 6 秒指令，然后停止重发并调用 `StopMove`。终端应显示本次 `vx=0.200 m/s`、`duration=6.000 s`。理论指令距离为 `0.20 × 6 = 1.2 m`，**不代表实际距离**。

观察：

- 起步后是否连续迈步，而不是只有一次迈步；
- 到时是否实际停住，以及之后是否保持停止；
- 用地面标记和卷尺测量实际位移，与理论 1.2 m 分开记录。

只做这一次。若仍只有一次迈步、持续前倾或姿态异常，立即停止试验，不自动继续增加速度、时长或连续任务。没有明显位移也不等于程序故障，不自动加大参数。

### 停止与退出

运动中需要停止，在同一终端输入并按 Enter：

```text
stop
```

也可以按键盘 **Ctrl+C**，尝试停止并退出；不要输入字符 `Ctrl+C`。

正常任务结束、用户确认实际停住后，输入：

```text
quit
```

普通停止先终止任务、停止发送 Move，再调用 StopMove。SDK 调用可能阻塞：当前调用超时设置为1秒，停止并非即时完成，若 SDK 不按超时返回则可能更久。

发生危险时，用户优先使用已核对的原生遥控停止操作，同时尽快 stop 或 Ctrl+C，终止电脑重发。不要把松开遥控摇杆当成取消电脑命令。遥控与电脑指令优先级尚未在本机验证；本工具不提供独立急停，不用拔线或强杀进程代替停止。

## 6. 退出后查看结果

程序退出、重新出现 Bash 提示符后，先执行以下命令，保留刚才的退出码：

```bash
motion_exit_code=$?
printf '程序退出码：%s\nCSV：%s\n' "$motion_exit_code" "$motion_log_dir/native_motion.csv"
```

- `0`：正常退出，且本会话没有停止请求失败；不证明机器人实际停住。
- `1`：启动阶段未等到状态或收到中断，未进入运动。
- `2`：参数、初始化或运行异常。
- `3`：本会话出现过停止请求失败，或最终停止请求失败。

检查本工具进程：

```bash
pgrep -af '[g]o2_native_motion'
```

没有输出表示未发现名称匹配的进程；有输出时先确认来源，不盲目结束其他会话。用户确认机器人实际停住后，再处理日志或残留进程。

首次完成条件：起步后连续迈步、方向正确、姿态平稳、到时实际停住且未自行再次运动；实际位移已用地面标记/卷尺测量并记录，CSV保留。接口返回、CSV中的完成事件和进程退出都不能代替现场观察。

## 7. 后续验证入口（首次不执行）

先审阅首次结果，之后再安排运动中 stop、goto、yaw 的验证；本节只说明命令，不是要求现在连续执行。

### 验证运动中 stop

本次 6 秒窗口足够在运动中输入 `stop`；但本次首要观察是完整 6 秒的连续行走与到时停止，不应为了再测一次而自动追加任务。若本次因异常使用 `stop` 提前终止，记录原因；若要单独验证运动中 stop，由主助手另行安排会话，仍使用已审阅参数，不自动提高速度或时长。

本段为此前直走操作记录。0.20 m/s / 6秒已执行，当前不重复；后续按本页顶部续接和下方独立goto步骤进行。

### 短距离 goto

另一次已安排的验证中，使用独立会话（不输入 `straight`，不会自动运动）：

```bash
cd /home/lidio/quadruped_robots
goto_log_dir="/home/lidio/quadruped_robots/logs/go2_native_motion/$(date -u +%Y%m%d_%H%M%S)_goto"
env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/home/lidio/Libraries/unitree_sdk2/lib \
  /tmp/go2_native_motion_build/go2_native_motion \
  --domain 0 --interface enp7s0 \
  --goto-max-speed 0.30 --goto-approach-speed 0.30 \
  --csv "$goto_log_dir/native_motion.csv"
```

`--vx/--duration-s` 使用默认值，只影响 `straight`，不会被 goto 使用。确认状态并重新 `origin` 后，先做一次短距离 goto：

```text
命令> goto 0.5 0
任务开始: goto local=(0.500,0.000) native=(...) vmax=0.300 approach=0.300
...
任务结束: goto ARRIVED
```

`approach=0.30` 是接近目标时的防失速下限候选，不是已验证的硬件最低速度；0.30 也只是当前已实测能走的候选最大速度。观察是否连续迈步、到时是否实际停住、是否明显过冲，并用地面标记/卷尺测量实际位移。只做这一次，不自动提高速度、降低下限或连续追加 goto。

### 到点与转向

在短距离 goto 结果审阅后，按已安排的范围逐项测试：

| 命令 | 含义 |
| --- | --- |
| `goto 0.5 0` | 短距离前向首试；到初始原点前方0.5米的固定点 |
| `goto 1 0` | 到初始原点前方1米的固定点 |
| `goto 0 1` | 到初始原点左侧1米的固定点 |
| `yaw 30` | 转到相对初始朝向左转30°的方向 |
| `status` | 查看当前本地位置、朝向和任务 |
| `stop` | 取消任务并尝试停止 |
| `quit` | 停止后退出 |

这些目标都相对本会话的初始原点，不是相对上一任务终点。例如先 goto 1 0 再 goto 0 1，第二段是斜向运动，不能当作纯左移试验；需要独立方向验证时，退出重启并在静止状态重新 origin。

当前独立限制：

- `goto` 最大平移速度默认 `0.30 m/s`，可用 `--goto-max-speed` 配置（范围 `0.05–0.50 m/s`）；接近目标速度不低于 `--goto-approach-speed`（默认 `0.30 m/s`，范围 `0.05–max`）。`approach` 是首轮防失速候选，不是已验证的硬件最低速度。目标距初始原点不超过3米，最长30秒，位置容差0.05米；3米不是累计路程或每段运动距离上限。`straight` 的 0.50 m/s 上限只适用于 `straight`。
- `yaw` 最大角速度 `0.50 rad/s`（约28.6°/秒），最长15秒，朝向容差5°，采用最短角度转向。
- `goto` 不控制朝向，`yaw` 不主动控制位置；没有自动避障。
- `ARRIVED` 只表示原生估计进入容差。使用地面标记、卷尺核对实际距离，不将原生估计当作独立真值。

方向、速度、姿态、状态或线缆有任何异常，立即停止，不扩大目标。

## 8. 本次 StopMove 返回 -1 后的只读查询

### 8.1 已有事实

2026-11-08 三次会话日志 `logs/go2_native_motion/20261108_032016/native_motion.csv`、`20261108_033359/native_motion.csv` 和 `20261108_044308/native_motion.csv` 中都没有 `cmd` 行；静止 stop 和退出时的 StopMove 均返回 -1，结束事件为 QUIT_STOP_FAILED。第三次会话没有 `origin`、没有任务，直接 `quit` 仍返回 -1，说明不是先前运动/任务残留。运动工具已确认退出，当前不继续 straight/goto/yaw。

两次会话之间的5秒只读观察结果：LowState约500 Hz，SportModeState约498–499 Hz，后者最大接收间隔约13.6 ms；mode=6、error_code=0，原生线速度接近零。观察器正常退出。后右髋部温度原始字段由72升至73、后左髋部为81，其他电机约27–33。尚无本机温度保护阈值依据，不能把温度认定为StopMove=-1的原因；mode=6也尚未取得适用于本机固件的官方语义依据，不用SDK示例中的测试选项编号解释它。

**下一步不是再运行只订阅状态的观察器，而是运行独立的只读查询入口。**该入口会订阅状态，并发送 `CheckMode`、`ServiceList`、`GetServerApiVersion` 查询请求；它不是“只订阅、完全不发包”。

### 8.2 构建查询入口

查询目标与运动工具同一构建目录。若 `/tmp/go2_native_motion_build/go2_native_readonly_query` 不存在，先执行第3节的 cmake 配置与构建；源码更新后也重新构建。不要运行运动工具。

```bash
ls -l /tmp/go2_native_motion_build/go2_native_readonly_query
```

### 8.3 现场最小只读命令

机器人保持静止，用户手持遥控器；不要在运动工具中执行 straight/goto/yaw。只在电脑本地终端执行：

```bash
cd /home/lidio/quadruped_robots
query_log_dir="/home/lidio/quadruped_robots/logs/go2_native_motion/$(date -u +%Y%m%d_%H%M%S)_readonly_query"
mkdir -p "$query_log_dir"
timeout --signal=INT --kill-after=2s 20s \
  env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/home/lidio/Libraries/unitree_sdk2/lib \
  /tmp/go2_native_motion_build/go2_native_readonly_query \
  --domain 0 --interface enp7s0 \
  2>&1 | tee "$query_log_dir/readonly_query.txt"
query_exit=${PIPESTATUS[0]}
printf 'query exit: %s\nlog: %s\n' "$query_exit" "$query_log_dir/readonly_query.txt"
```

该命令只做以下事情：

- 初始化 DDS；
- 订阅 `rt/sportmodestate`，最多等待3秒取得状态；
- 发送 `CheckMode`（`motion_switcher`，API 1001）、`ServiceList`（`robot_state`，API 1003）、`GetServerApiVersion`（Sport 通用 API 1）三个查询请求；
- 关闭订阅并释放 DDS，退出。

不调用 `Move`、`StopMove`、站立、趴伏、阻尼、步态、模式切换、`ServiceSwitch`、`SwitchJoystick`、`SetSilent` 或 `SetReportFreq`；退出、异常和析构中不发送停止或恢复指令。查询请求本身会发包，不能称为“完全只订阅”。

构造并 `Init` 各客户端只做本地 API 版本设置和 API 注册，不发送运动、停止或模式切换请求；随后只调用上面的查询方法。

等待时间：`CheckMode` 和 `ServiceList` 各设置 1 秒；`GetServerApiVersion` 是 SDK 现有方法，内部使用 1 秒并对发送/超时错误重试一次；本入口没有新增重试或超时。外部 `timeout 20s` 只是总时限。

### 8.4 预期输出与解释

保留终端和 `readonly_query.txt`。输出大致为：

```text
state before queries: seq=... mode=... error_code=... age_s=... pos=(...) vel=(...) yaw_raw=...
query check_mode: rc=... form="..." name="..."
query service_list: rc=... count=...
  service[0] name="..." status=... protect=...
  ...
query sport_api: local="1.0.0.1" server="..."
state after queries: seq=... mode=... error_code=... age_s=...
queries complete; no motion or mode-changing request sent; no stop/restore on exit
```

解释边界：

- `check_mode` 的 `form`/`name` 是机器人返回的原生模式；`rc` 非零时只记录返回码，不补默认模式，不据此猜测。
- `service_list` 的 `name/status/protect` 是机器人返回的服务状态；`rc` 非零或空列表时按实际记录，不补默认服务。
- `sport_api` 的 `local` 是本机 SDK 声明版本，`server` 是机器人返回版本；为空时显示 `NOT OBTAINED`，不补默认版本。
- `state` 的 `mode`、`error_code`、`age_s` 是原始状态和本机接收时龄；`mode=6` 仍按原始值记录，不用测试编号或社区说法当作官方定义。
- `NOT OBTAINED` / 空值表示没取得，不是正常值，也不是失败原因。

下一次查询要区分的候选：

- `CheckMode` 显示原生模式不允许运动/停止命令，或 `ServiceList` 显示运动服务未运行/受保护；
- Sport 服务端 API 版本与本机声明的 `1.0.0.1` 不一致；
- 服务列表中没有预期的原生运动服务，或服务状态异常；
- 以上都正常但 StopMove 仍返回 -1：现有本地证据无法解释，下一步查官方资料/支持渠道，不用 Move 或模式切换试探。

### 8.5 退出与清理

- 程序正常完成且未被中断时返回0；各个查询自身的 `rc` 仍可能是非零，必须以输出为准。
- DDS 初始化失败或运行中被中断返回1；参数错误返回2；外部 `timeout` 返回124属于限时终止，需记录，不代表机器人状态。
- 程序关闭订阅并释放 DDS；不发送停止/恢复指令，也不承担停止机器人的作用。
- 查询结束后核对进程：
  ```bash
  pgrep -af '[g]o2_native_readonly_query'
  ```
  没有输出表示未发现名称匹配的残留进程。
