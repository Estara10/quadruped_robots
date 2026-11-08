# Go2 StopMove 返回 -1 的离线排查与只读查询准备

日期：2026-11-08。仅做离线日志、源码、SDK 头和链接库核对，以及构建准备；未连接机器人、未初始化实机 DDS、未运行运动工具或查询入口。

## 1. 结论摘要

- 三次现场会话中 `StopMove()` 返回 `-1`，都没有发送过 `Move`。第三次会话没有记录 `origin`，打开工具后直接 `quit`，仍然返回 `-1`；这说明失败与是否先运动、是否已有任务无关，至少不是由先前的 `Move` 或目标控制留下的状态导致。
- 复核链接库调用链后可以确认：`-1` 不是客户端捕获异常，不是 SDK 的发送失败（3102）、请求超时（3104）、API 未注册（3103）或响应 API 不匹配（3105），而是机器人服务返回的 `ResponseStatus.code = -1`。
- 当前没有足够证据确定服务为什么返回 `-1`。本地 SDK 没有为 Sport 服务定义状态码 `-1` 的含义；本机日志也无法区分“当前模式不接受停止”“服务状态/保护”“服务端 API 实现差异”等候选。
- 已准备独立的只读查询入口 `go2_native_readonly_query`，用于下一次现场区分主要候选；本轮只构建，未运行。
- 未修改论文控制器、仿真、模型或唯一 ACTIVE 任务；未提交 Git。

## 2. 已证实的事实与证据

### 2.1 原始日志

| 会话 | 文件 | 事件 | 时间差 | 结果 |
| --- | --- | --- | --- | --- |
| 1 | `logs/go2_native_motion/20261108_032016/native_motion.csv` | 静止 `stop` | `stop_request` 78.789848 → `stop_result` 78.790569，约 0.72 ms | `api_return=-1`，`StopMove returned -1` |
| 1 | 同上 | `quit` 前最后停止 | 178.831057 → 178.831655，约 0.60 ms | `-1` |
| 2 | `logs/go2_native_motion/20261108_033359/native_motion.csv` | 静止 `stop` | 4.459040 → 4.459581，约 0.54 ms | `-1` |
| 2 | 同上 | `quit` 前最后停止 | 19.745382 → 19.746081，约 0.70 ms | `-1` |
| 3 | `logs/go2_native_motion/20261108_044308/native_motion.csv` | 无 `origin`、无任务，直接 `quit` | 4.718681 → 4.719459，约 0.78 ms | `-1` |

三个 CSV 均无 `cmd` 行；结束事件均为 `QUIT_STOP_FAILED`。状态订阅在同期持续更新（例如 session 1 的 `state_seq` 从 25 增到 39305，session 2 从 25 增到 9854，session 3 从 25 增到 2358），说明 `rt/sportmodestate` 接收正常。session 3 没有 `origin`、没有任务、没有 `Move`，仍返回 `-1`；这进一步说明失败不是由先前的运动/任务状态触发，而是与当前原生服务/模式/接口状态有关。

### 2.2 当前源码的返回路径

`scripts/go2_native_motion/main.cpp` 的 `SportBackend::stopMove()`：

```cpp
try {
  const int32_t code = client_.StopMove();
  setLastError(code == 0 ? std::string()
                         : "StopMove returned " + std::to_string(code));
  return code;
} catch (const std::exception& error) {
  setLastError(error.what());
  return -1;
} catch (...) {
  setLastError("unknown exception in StopMove");
  return -1;
}
```

日志中的 `detail` 是 `StopMove returned -1`，与“返回值 `-1`”路径一致；若是捕获异常，`detail` 应为异常文本，或为空后由核心写成 `StopMove returned nonzero`。因此本次不是捕获异常路径。

### 2.3 SDK 头文件与链接库

- 实际使用 `/home/lidio/Libraries/unitree_sdk2`；运动工具链接静态库 `libunitree_sdk2.a`，运行时从同一目录加载 `libddsc.so.0` 和 `libddscxx.so.0`。
- `SportClient` 构造函数使用服务名 `sport`，`Init()` 设置 API 版本 `1.0.0.1` 并注册 API 1003（`StopMove`）；`StopMove()` 以空参数调用 API 1003。
- `SportClient::Move` 是 API 1008；本次日志无 `cmd` 行，未调用。
- 本机 SDK 内部错误码：
  - `3102`：请求发送失败；
  - `3103`：API 未注册；
  - `3104`：客户端 API 超时；
  - `3105`：响应 API 不匹配。
- 反汇编 `ClientBase::Call`（链接后的 `/tmp/go2_native_motion_build/go2_native_motion`）：
  - `SendRequest` 返回空 → 3102；
  - `GetResponse` 返回空 → 3104；
  - 响应 API ID 不匹配 → 3105；
  - 否则返回 `response.header().status().code()`。
- `Client::CheckApi` 在 API 未注册时返回 3103；本次返回的是 `-1`，且 API 1003 已由 `SportClient::Init()` 注册，因此也不是未注册路径。
- `StopMove` 调用在约 0.5–0.7 ms 内返回，远短于 SDK 的 1 s 客户端超时。

**可以确认：** 客户端收到了匹配 API 1003 的响应，服务层返回了状态码 `-1`。`-1` 不能解释为客户端超时、发送失败、API 未注册、响应不匹配或捕获异常。

### 2.4 状态订阅能证明什么

- LowState / SportModeState 高频接收只证明 DDS 状态发布与订阅链路正常。
- 本次 StopMove 请求能够得到匹配响应，进一步证明 `sport` 请求/响应链路可达；失败发生在机器人服务的应用层。
- 状态接收正常不证明原生运动服务处于可接受停止命令的状态，也不证明服务列表、API 版本或模式兼容。

### 2.5 官方资料与社区资料

- 本地 SDK 头文件、官方 `go2_sport_client.cpp` 示例和官方 `unitree_sdk2_python` 的 SportClient 示例是直接 SDK 证据：`StopMove` 发空参数、只返回状态码；示例本身没有模式检查或 StopMove 前置条件。
- 已尝试访问宇树官方文档中心的 SportClient、Motion Switcher、RobotStateClient 页面；页面被站点安全策略拦截或只返回标题，未取得可引用的正文。不能据此补写官方限制。
- 社区资料中有人把 `SportModeState.mode=6` 解释为 jointLock/关节锁定并提示过热；这是社区观察，不是官方定义，不能直接证明本机 Go2 EDU 行为。另有不同型号的 `FSM_STATE_STR` 枚举索引 6 是别的内容，不能混用。
- SDK 示例里的 `normal_stand`、`velocity_move` 等是示例测试选项编号，不是 `SportModeState.mode` 的映射；本报告不据此解释 `mode=6`。
- 未找到官方或本地 SDK 对 Sport 服务状态码 `-1` 的直接定义。

## 3. 尚未确定的原因

服务返回 `-1` 的具体原因仍未确定。候选包括：

1. 当前原生模式/状态不接受 Sport 停止命令；`CheckMode` 的 form/name 与 `SportModeState.mode` 可提供直接信息。
2. 原生运动服务未运行、未列出或处于保护状态；`ServiceList` 的 name/status/protect 可提供直接信息。
3. 服务端 Sport API 版本/实现与本机 SDK 声明的 `1.0.0.1` 不一致；`GetServerApiVersion` 可提供直接信息。
4. 固件/服务端特有拒绝，本地 SDK 和当前日志无法解释；若前三项都正常，下一步应查官方资料或支持渠道。

第三次会话没有 `origin`、没有任务、没有 `Move`，仅 `quit` 仍返回 `-1`；因此“先前运动/任务残留”这一原因被进一步排除，候选 1–3 更值得优先。尤其是如果 StopMove 只在某个运动/控制模式下有效，当前静止或非运动模式下的调用可能被服务拒绝；但这仍是候选，不是已证实结论。

不能把 `SportModeState.error_code=0` 当作控制接口正常；它不覆盖 Sport RPC 服务状态。也不能把髋部温度原始值直接归因为 StopMove=-1；没有温度阈值或保护触发证据。

## 4. 本轮修改

- 新增 `scripts/go2_native_motion/readonly_query.cpp`：独立只读查询入口。
- 在 `scripts/go2_native_motion/CMakeLists.txt` 增加目标 `go2_native_readonly_query`，仅链接现有 SDK。
- 更新 `scripts/go2_native_motion/FIELD_COMMANDS.md` 第 8 节，给出下一次现场最小只读命令、预期输出和退出说明。
- 未修改运动控制的发送逻辑；停止失败锁存仍保留。
- 未修改共享控制器、仿真配置、模型或唯一 ACTIVE 任务；未安装包；未提交或推送 Git。

查询入口做：

- 订阅 `rt/sportmodestate`，最多等待 3 s；
- 构造 `MotionSwitcherClient`、`RobotStateClient`、`SportClient` 并调用各自的 `Init()`；这些 `Init()` 只做本地 API 版本设置和 API 注册，不发送运动、停止或模式切换请求；
- 调用 `MotionSwitcherClient::CheckMode`（`motion_switcher` API 1001，查询），客户端等待 1 s、无新增重试；
- 调用 `RobotStateClient::ServiceList`（`robot_state` API 1003，查询），客户端等待 1 s、无新增重试；
- 打印本机 `SportClient::GetApiVersion()`，调用 `SportClient::GetServerApiVersion()`（Sport 内部 API 1，查询）；后者是 SDK 现有实现，内部使用 1 s 并对 3102/3104 重试一次，本入口没有新增重试或超时；
- 打印查询前后的 `SportModeState` mode、error_code、接收时龄、位置/速度/yaw 原始值；
- 关闭订阅并释放 DDS；退出、异常和析构中不发送停止或恢复指令。

查询入口**不调用**：`Move`、`StopMove`、站立、趴伏、阻尼、步态、`ReleaseMode`、`SelectMode`、`ServiceSwitch`、`SwitchJoystick`、`SetSilent`、`SetReportFreq`。它不包含“查询失败就切换模式”的回退。它发送查询 RPC，因此不能称为“只订阅、完全不发包”。

## 5. 离线验证

- 干净构建通过；新目标 `go2_native_readonly_query` 构建成功。
- 运动工具自检仍 **20 项全部 PASS**，停止失败锁存未被诊断改动放行。
- 查询源码检查无运动、停止、站立、步态、模式切换、服务切换、静默或报告频率调用，也无 `ChannelPublisher`/`LowCmd`。
- 查询入口 `--help` 在 DDS 初始化前退出，未连接设备。
- 未运行带有效 domain/interface 的查询入口；本轮没有初始化实机 DDS。
- 当前无相关进程残留。

## 6. 下一次现场最小只读命令

完整命令、输出示例和退出说明见 [`scripts/go2_native_motion/FIELD_COMMANDS.md` 第 8 节](../../../scripts/go2_native_motion/FIELD_COMMANDS.md)。核心命令为：

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

预期获得：

- `check_mode` 的返回码、form、name；
- `service_list` 的返回码和服务 name/status/protect；
- `sport_api` 的本机版本与机器人返回版本；
- 查询前后的 `state` mode、error_code、age_s。

任一查询失败或为空时显示 `NOT OBTAINED`，不补默认值。输出中的 `rc` 和原始字段是下一次判断的依据：若 `CheckMode`/`ServiceList`/API 版本都正常而 StopMove 仍为 -1，则现有本地证据无法解释，转查官方资料/支持，不用 Move 或模式切换试探。

退出码：`0` 为查询程序完成且未被中断（单项查询仍可能非零）；`1` 为 DDS 初始化失败或运行中被中断；`2` 为参数错误；`124` 为外部 `timeout` 限时终止。程序退出不承担停止机器人的作用。

## 7. 参考与访问限制

- 本机 SDK 头文件：`/home/lidio/Libraries/unitree_sdk2/include/unitree/robot/go2/sport/sport_api.hpp`、`sport_client.hpp`、`/home/lidio/Libraries/unitree_sdk2/include/unitree/robot/client/client.hpp`、`client_base.hpp`、`/home/lidio/Libraries/unitree_sdk2/include/unitree/robot/internal/internal_error.hpp`。
- 官方 C++ 示例：[`unitree_sdk2/example/go2/go2_sport_client.cpp`](https://raw.githubusercontent.com/unitreerobotics/unitree_sdk2/main/example/go2/go2_sport_client.cpp)；本机仓库内也有一份同源示例。
- 官方 Python SportClient：[`unitree_sdk2_python/go2/sport/sport_client.py`](https://raw.githubusercontent.com/unitreerobotics/unitree_sdk2_python/master/unitree_sdk2py/go2/sport/sport_client.py)。
- 官方 RobotStateClient 页面 [support.unitree.com/home/zh/developer/RobotStateClient](https://support.unitree.com/home/zh/developer/RobotStateClient)、SportClient 页面和 Motion Switcher 页面已尝试访问；返回站点安全拦截或仅标题，未取得可引用的正文。报告没有用这些页面的内容补写限制。
- 社区二手摘要（如 CSDN 开发日志）提到 Go2 `mode=6` 可能为 jointLock/关节锁定；未取得官方定义，也未在本机验证。本报告不把它作为本机 Go2 EDU 的事实，只作为需要下一次查询区分的候选背景。
- 没有找到官方或本机 SDK 对 Sport 服务 `ResponseStatus.code=-1` 的直接定义；不把 -1 解释为超时、权限或网络原因。

## 8. 后续现场只读查询结果（2026-11-08）

用户开机后亲自在终端执行查询，日志保存在 `logs/go2_native_motion/20261108_042731_readonly_query/readonly_query.txt`，程序退出码0。主助手未代发查询或控制请求。

- 查询前后 `mode=7`、`error_code=0`、消息接收时龄约1 ms，位置z约0.049、原生线速度近零；不据此为mode=7补写未经核实的模式定义。
- `CheckMode`：`rc=0, form="0", name="ai"`。
- `ServiceList`：`rc=0, count=28`。关键原始项：`ai_sport(status=0, protect=0)`、`sport_mode(status=1, protect=0)`、`advanced_sport(status=1, protect=0)`、`obstacles_avoid(status=1, protect=0)`、`motion_switcher(status=0, protect=0)`。
- `SportClient` 客户端和服务端报告的API版本均为 `1.0.0.1`。没有取得机器人整机系统/固件版本，不能把这个API版本当成固件版本。
- 查询器正常关闭；随后进程核对未发现运动工具、查询器或状态观察器残留。

官方 [Go2示例的queryServiceName函数](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2/go2_stand_example.cpp)将四足form="0"的name="ai"对应为ai_sport，将normal对应为sport_mode。本次因此确认当前被选择的是ai模式。这里仅引用其映射，不执行这个含低层发包和模式释放的示例。

结论：当前运动模式已取得，简单的API版本字符串不一致候选不成立；服务查询与Sport版本查询可达。API版本相同不证明1003在当前ai模式、姿态和本机固件下可用，当前仍未确定StopMove=-1的具体原因。不能仅凭服务status数字判断需要开启sport_mode，更不能自行同时开启多个运动服务。下一步应结合本机整机系统版本，核实ai模式下SportClient/StopMove的支持与适用状态；未取得依据前不改成零速度Move、不切换模式试探。

## 9. 进入原生可行走状态后StopMove成功（后续现场结果）

用户确认前一次空闲退出会话一直保持趴伏；随后按约定用遥控器站立并进入原生可行走状态，再运行原工具。`logs/go2_native_motion/20261108_044903/native_motion.csv` 及用户终端输出记录静止stop返回0。用户说明此前使用遥控器锁定了机器人不运动，本次解除相关锁定。没有为此修改工具、切换原生服务或绕过失败保护。

这项结果支持停止接口接受情况与原生锁定/运行状态有关，应把“用户遥控确认原生可行走状态、摇杆归中”纳入现场操作前提。不能由此推导所有姿态下StopMove均成功，也没有得到各数值mode的官方映射。当前只验证静止状态的停止请求成功，尚未验证真实运动后的物理停止或遥控优先级。

后续不继续追查固件版本或重复查询，先按已有方案做0.05 m/s、1秒的单次straight，观察方向、姿态及到时实际停住。若仍出现请求失败，保留原停止失败锁存，不把-1改为成功。

## 10. 短时前进请求成功，但用户报告不能连续迈步

后续主助手只读检查已有CSV：

- `20261108_044903`：0.05 m/s、1秒，10条Move，随后StopMove返回0；用户报告刚要走即到时。
- `20261108_045314`：实际参数为0.10 m/s、2秒，20条Move，随后StopMove返回0。
- `20261108_045405`：实际参数为0.10 m/s、3秒，30条Move，随后StopMove返回0。

以上Move返回值均为0，三条速度分量分别为各次设定的正前向速度、0、0；命令行记录的状态最大接收时龄约2.1–3.1 ms。日志均有quit结束事件。接口返回与用户观察必须分开：用户报告不能连续迈步、只向前倾斜，不能把这些会话记为基础行走通过。

现有结果不支持继续只靠延长运动时间或加大速度排查。下一项最小对照是：恢复安全稳定姿态后，由用户确认同一当前原生状态下遥控器是否能正常走一两步并停止；若遥控同样只倾斜，则结束运动尝试，查当前状态/现场和设备；若遥控正常，优先比较原生遥控与SDK指令的运行状态和接收路径。A到B坐标转换未在straight中使用，暂不调goto参数或增加定位传感器。

## 11. 6秒仍不能行走；补正Move返回值的证据含义

用户确认遥控器正常。后续按原计划延长时长：`logs/go2_native_motion/20261108_052021/native_motion.csv` 记录两次straight，每次0.20 m/s、6秒，各60条Move，共120条。Move均返回0，命令行记录的最大状态接收时龄约3.0 ms，任务完成后的StopMove均返回0。第一次任务开始/结束的本地位置约(-0.0008,-0.0003)→(0.0318,0.0039)，第二次约(0.0034,-0.0010)→(0.0388,0.0016)，单位沿用原生估计假设；不是独立真实位移。用户报告仍只前倾、不运动。主助手检查时未发现运动工具进程，不能将此结果写成行走成功。

主助手进一步只读核对当前实际运动二进制：`SportClient::Move(float,float,float)`用JsonizeVec3构造参数，调用API1008的`Client::Call(apiId, parameter)`无输出参数重载；对应`ClientBase::Call`设置`noReply=true`，只调用`ClientStub::Send`，发送成功即返回0，不调用GetResponse。StopMove使用带data输出参数的重载，等待匹配响应后返回服务状态码。

因此，Move返回0只证明SDK发送路径报告成功，不能等同于机器人应用层接受，更不能证明已执行连续行走。此前对Move返回0的解释若被理解为机器人确认执行，应以本节修正为准；对StopMove=-1来自匹配服务响应的原结论不变。

单纯延长启动时间的解释已不足以说明当前现象，不继续扩大速度或时长。下一项应离线准备能够区分“服务接受/拒绝”和“接受但未进入行走状态”的最小诊断：核实同一API1008是否支持带回包查询式诊断调用，配合记录mode/gait_type；若没有依据支持请求回包，不把改调用重载当作已修复，也不做现场试探。本轮主助手没有改控制代码或发送机器人指令。
