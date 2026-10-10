# S2-07 执行报告

**最新状态：COMPLETE（Director 有条件通过）。** 通过范围为：到达后停止运动、确认站稳并连续保持约 5 仿真秒；正常路径不自动趴下或卸力；运动指标排除收尾；先关闭 MuJoCo，再关闭 controller 等进程。证据为下文两次仿真 ARRIVED 诊断及各自约 5 仿真秒的稳定站立观察。本结论不扩展为实机安全或完整避障性能通过。

本版实现、验证及结果见文末“新版执行结果（2026-10-10）”。后面的旧趴下执行记录用于保留历史，不是本版验收目标。

## 历史收尾合同与判据（旧趴下方案，已被新版取代）

运动终局的同一策略快照先封口。仅当终局是 ARRIVED 或诊断仿真时限，且终局帧有 LIVE 碰撞/物理快照、无安全故障、无未分类接触、无跌倒候选/确认、无机器人—障碍接触时，监督器才发送正常收尾请求。其他终局沿用原 PASSIVE 安全停止，不请求站立或趴下。

正常流程及仿真诊断判据如下。所有时长和阈值仅用于本次仿真诊断，不是实机安全界限或研究参数。

| 阶段 | 请求/动作 | 实际确认条件 | 上限/失败处理 |
|---|---|---|---|
| DECELERATING | 策略机身速度命令在 600 ms 内线性收至 0，仍由 Agile 保持支撑 | 控制器里程计 XY 速度 ≤0.08 m/s、IMU 角速度 ≤0.15 rad/s、roll/pitch 各 ≤0.35 rad、足端支撑数 ≥2，连续 ≥400 ms | 5 s；未确认转 PASSIVE，不进入站立/趴下 |
| STANDING | 转入现有 FIXEDSTAND；关节目标从实测起始位置连续插值，kp/kd 从现有命令值连续爬升 | 12 关节最大位置误差 ≤0.12 rad、最大关节速度 ≤0.10 rad/s、里程计 XY 速度 ≤0.08 m/s、IMU 角速度 ≤0.15 rad/s、roll/pitch 各 ≤0.30 rad、机身高度 ≥0.25 m、至少 3 足支撑，连续 ≥400 ms | 10 s；失败时保持当前阶段，不自动趴下；监督器硬停收尾 |
| LOWERING | 仅在站立已确认后转入现有 FIXEDDOWN；从实测关节位置平滑插值，增益连续 | 12 关节最大位置误差 ≤0.12 rad、最大关节速度 ≤0.10 rad/s、水平速度 ≤0.08 m/s、角速度 ≤0.15 rad/s、roll/pitch 各 ≤0.35 rad、机身高度 ≤0.29 m、至少 2 足支撑，连续 ≥400 ms | 10 s；失败时不确认趴下、不正常卸力，监督器硬停 |
| RELEASED | DOWN 确认后进入 PASSIVE | 下一 controller 周期读回全部 kp、kd、tau 命令均为 0 | 读回失败则不报告卸力确认；MuJoCo 命令消费仍 UNKNOWN |

命令 1/9 在 controller 更新最前端抢占并转 PASSIVE；异常、无效物理源、传感器不健康或阶段失败不得继续站立/趴下。若正常收尾未到 RELEASED，监督器会记录原因并发送 command 9，随后只在收到 controller PASSIVE 确认时报告该层已停止。固定等待时间不会被写成运动状态确认。

## 历史执行证据（旧趴下方案）

本节在定向验证及运行后追加。运动终局和收尾观察分别保存；下方任何空缺仍为 UNKNOWN，不以命令请求代替状态确认。

### 针对性检查与构建

- `preflight/offline_checks.json` 记录生产 C++ 判据 helper、supervisor 正常收尾门槛、运动终局/清理尾段分析路径及 Python 语法检查结果。硬停优先分支位于 controller `update()` 的 estimator/FSM 调用之前；离线 `evaluate()` 检查确认硬停返回 ABORT，不把请求误作阶段确认。
- 首次仅构建控制器包通过；随后因发现 `BaseFixedStand` 属于直接受影响的 `controller_common`，改为增量构建 `controller_common` 与 `rl_quadruped_controller`，两包通过。仅有原有 CMake 版本兼容性弃用警告。
- 旧 S1-04 完整收尾测试最初因调用未提供新增必需的场景 authority 参数而失败；将其离线调用适配为 `scene_obstacle.xml` 权威对象后，生产收尾与指标检查通过。没有改动该历史测试的原始数据。

### 显示预检与运行记录

显示检查结果在 [`preflight/display_preflight.json`](preflight/display_preflight.json)：默认沙箱连接 `:0` 失败；按现有授权临时在沙箱外使用 `DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority` 检查后，X11、GLFW 初始化、窗口创建通过，OpenGL renderer 为 Mesa Intel(R) Graphics (RPL-P)，OpenGL 4.6。未修改系统、驱动、授权或桌面配置。

平地尝试保留在 [`diagnostic_runs_20261010/scene_flat/`](diagnostic_runs_20261010/scene_flat/)：

1. `371cde10a7e9498f99bf54686852d4c7`：启动命令在初始 FSM 过渡时序中未进入 RL；0 策略帧，终局 `WALL_CLOCK_GUARD_INCOMPLETE`，指标为 INVALID。收尾中 controller command 1→PASSIVE 得到确认，`shutdown_complete=true`、exit code 0。旧 supervisor 在写完结果后发生空值访问异常；记录与 `terminal.json` 已在异常前落盘，原始记录保留。
2. `5e1555d9223a4f7a83078d41af0fa755`：同样 0 策略帧并以 `WALL_CLOCK_GUARD_INCOMPLETE` 结束，指标 INVALID；PASSIVE 与清理确认完成。逐周期启动日志显示切换日志早于新状态 `enter()`，入口会清除 command 3。启动器后续增加了等待真实 FIXEDSTAND 运行日志再请求 RL 的检查。
3. `4e4ee785e6de4b2c9bfc6fd4f4e543cf`：修正后的启动通过，session `37940492120169`，实际加载 A / `paper_faithful_switch`，阈值 entry/exit 均 `-0.05`，目标 `(7,0) m`，平地 scene ID `flat`，模型位于本次 overlay 下的 Agile、RA、Recovery 三路径。实测 loaned contact 顺序 `FR, FL, RR, RL`，record 218 帧，终局 `ARRIVED`，终点距离 `0.497545 m`。分析为 VALID，运动窗口含周期 46–217、无周期缺口、6 个无效时钟帧、策略频率约 `50.7507 Hz`；运动有效累计时间 `3.276 s`、路径长 `6.2498 m`、平均路径速度 `1.9077 m/s`。详见对应 `run_context.json`、`runtime_record.jsonl`、`terminal.json`、`process_facts.json` 与 [`analysis/run_results.json`](diagnostic_runs_20261010/scene_flat/analysis/run_results.json)。运动指标止于 ARRIVED 策略终止周期，正常收尾采样与清理没有加入运动窗口。

### 正常收尾实测与停止点

第三次平地运行终局 gate 为 eligible。controller 日志直接记录：

- `DECELERATED`：里程计 XY speed `0.00068 m/s`、IMU angular speed `0.00893 rad/s`、四足支撑，连续稳定 400 ms；随后切入 FIXEDSTAND，kp/kd 从现存命令值连续过渡。
- 本次减速请求日志记录初始策略机身命令 `vx=vy=wz=0`，所以它直接证明连续 ramp 路径运行、并在之后观测到速度稳定；不能单独证明非零命令时 ramp 的减速效果。
- `STANDING_CONFIRMED`：连续 400 ms 满足判据；里程计 XY speed `0.00140 m/s`、IMU angular speed `0.00222 rad/s`、base z `0.36458 m`、roll `0.00860 rad`、pitch `0.02242 rad`、max q error `0.11247 rad`、max qdot `0.01487 rad/s`、四足支撑。controller 随后请求进入 FIXEDDOWN。
- 趴下阶段期间，scene-bound PhysicsLoop authority 触发了 supervisor 的保护条件：`fall_confirmed` 或机器人—障碍接触数大于 0。supervisor 立即停止正常趴下流程，发送 command 9，controller 日志确认 `ABORTED` 后进入 PASSIVE。没有 `DOWN_CONFIRMED` 或正常 `RELEASED` 事件，所以本次**没有证据证明已确认趴下后才卸力**。这次 PASSIVE 是安全硬停兜底，不计作正常卸力完成。
- 触发收尾 veto 的那一帧在当时实现中先检查再追加 observation，未被持久化；触发前已保存的最后样本高度降至 `0.17955 m`，该样本 `fall_confirmed=false`、障碍接触数 0。因触发样本未保存，触发时究竟是跌倒确认还是机器人—障碍接触，及触发帧的精确几何/姿态仍 UNKNOWN。监督器现已调整为先保存当前 physics 样本，再分别记录跌倒或机器人—障碍接触的拒绝原因；此观测修正未重新运行验证。
- `process_facts.json` 确认 ROS launch、MuJoCo、预备发布器均以 exit code 0 清理结束、无强制终止、`shutdown_complete=true`；controller 对 command 9 的 PASSIVE 转换有直接确认。RViz 未由本脚本启动。MuJoCo 是否消费 PASSIVE/零增益仍 UNKNOWN。

当前限制：两次有效策略启动前的无效尝试均保留；本次只完成平地有效运行，随机场景04没有运行。controller 增益连续性有生产代码的接口读回和插值依据，但没有每周期 kp/kd 数值时间序列；下降保护触发的精确类别/触发快照没有在旧记录中保存。没有 `DOWN_CONFIRMED`/`RELEASED`，故用户关心的“趴下后才卸力”仍未验证。模型真实动作语义、MuJoCo 命令消费、实机安全界限及任务外长期稳定性不在本轮证据内。

按旧任务停止条件，下降阶段触发安全 gate 后未运行 scene_random_04；不为完成趴下或取得更好结果重跑。该事实保留为历史。新版不再要求完成趴下或卸力。

## 新版执行结果（2026-10-10）

### 新版收尾合同

健康 ARRIVED 后，终止策略快照先保存并封口。正常流程为 `DECELERATING → STANDING → STANDING_HOLD`：策略机身速度命令在 600 ms 内按线性比例收至 0；经过连续 400 ms 速度、角速度、姿态和足端支撑判据后进入 FIXEDSTAND；从实测关节位置插值至固定站立，同时延续 kp/kd。站立确认要求关节最大位置误差 ≤0.12 rad、最大关节速度 ≤0.10 rad/s、XY 速度 ≤0.08 m/s、角速度 ≤0.15 rad/s、姿态各轴 ≤0.30 rad、机身高度 ≥0.25 m、至少 3 个 controller 足端力接口显示支撑，连续 400 ms。确认后每个 controller 周期继续验证上述站立条件和非零 kp/kd；条件失效、反馈无效或急停会终止正常保持并走安全兜底。

站立观察默认 5 仿真秒、范围 0–30 秒；另设独立 1–60 秒墙钟保护，默认 30 秒。supervisor 以连续 LIVE、覆盖完整的 MuJoCo 物理 authority 样本计时，不用 sleep 代替状态证据；仿真暂停或时间倒退会使本次保持失败，墙钟上限防止无限等待。运动记录在 ARRIVED 周期停止。完成保持后，仍在 FIXEDSTAND 时先停止并确认本次 MuJoCo 进程退出，再关闭 ROS/controller 与停止发布器；健康路径不发 command 1/9、不进入 FIXEDDOWN、不转 PASSIVE。命令 1/9 的 controller 急停分支仍优先于正常状态机。

真实 MuJoCo framebuffer 会在保持窗口显示 `ARRIVED - HOLDING STAND` 提示；策略帧被正常退出失效后面板显示 INVALID/No current policy data，不复用旧策略数值。实机不能沿用“先关仿真再关控制器”的生命周期处理。

### 定向验证与构建

- `preflight/offline_checks.json` 的新版条目汇总 C++ helper 编译/运行、Python 定向检查、增量构建和两次诊断统计。
- 5 项定向 pytest 通过：站立物理样本覆盖/姿态/支撑校验、仿真时间到期与回退拒绝、安全事件优先级、终止周期包含且收尾尾段排除、收尾写入错误仍完成进程清理。Python 编译检查通过。C++ helper 检查了 600 ms 非零命令比例 1.0→0.5→0、支撑增益有效性、超时、无效反馈及急停抢占。
- `controller_common`、`rl_quadruped_controller` 和 `unitree_mujoco` 增量构建通过。对 `StateRL` 的 600 ms ramp 仅将原有线性比例计算提取为生产 helper；公式不变。该 helper 的新增验证发生在两次运行之后，仿真中观察到的初始速度命令均为零，因此运行数据本身不能证明非零命令 ramp 的实测效果。
- 系统 Python 的 matplotlib 与其 NumPy 版本不兼容，未安装或改动组件；改用已有 Anaconda Python 3.13 / Matplotlib 3.10.0 运行同一生产分析器，生成两次 metrics 和时间线。

### 两次运行和站立观察

运行前完成现有桌面 `DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority` 的 X11/GLFW 预检：X11、16×16 GLFW 窗口、Intel Mesa OpenGL 4.6 均通过；未改系统权限、驱动或桌面设置。每次运行实际来源日志确认 group A、candidate `paper_faithful_switch`、entry/exit 均为 -0.05、hysteresis=false、hold=false、目标 (7,0)，模型来自本次 `go2_description` overlay。场景来源、策略 session、分析结果和收尾数据按 run 分目录保存。

| 场景 / run | 运动终局 / 终点距离 | 有效运动时间 / 路径 / 平均速度 | 策略记录 | 站立保持 | 清理 |
|---|---|---|---|---|---|
| flat / `da53e788be7044868df34758895ec61f` | ARRIVED / 0.499997 m | 3.380 s / 6.411 m / 1.897 m/s | 208 帧；约 50.041 Hz；1 个无效时钟帧；无周期缺口；0 次模式变化 | 5.002 s；418 个保持窗口物理样本；最低 base z 0.3225 m、四足地面支撑、无 fall/障碍接触样本 | MuJoCo 先退出 0，ROS launch/controller 退出 0；停止发布器退出 1；无强制结束，shutdown_complete=true |
| random_04 / `19b4b20771ec4b3baed9ef02c23b23f1` | ARRIVED / 0.477546 m | 9.366 s / 7.739 m / 0.826 m/s | 552 帧；约 50.209 Hz；17 个无效时钟帧；1 个策略周期缺口；115 次已记录模式变化 | 5.004 s；372 个保持窗口物理样本；最低 base z 0.3213 m、四足地面支撑、无 fall/障碍接触样本 | MuJoCo 先退出 0，ROS launch/controller 退出 0；停止发布器退出 1；无强制结束，shutdown_complete=true |

两次 `STANDING_CONFIRMED` controller 样本分别测得 XY speed 0.00330/0.00128 m/s、角速度 0.00194/0.00284 rad/s、四足支撑、max kp=80、max kd=3、最大关节位置误差 0.10964/0.11294 rad、最大关节速度 0.01893/0.01403 rad/s；之后 stage 保持 FIXEDSTAND，日志没有 FAILED、LOWERING、RELEASE_PENDING、RELEASED 或硬停转换。各次保持期的 physics sample 序号与 sim_time 单调推进，覆盖标志有效。监督器没有发送正常 PASSIVE；MuJoCo 停止前没有 controller stop 请求。MuJoCo 命令消费不适用/未作执行确认。站立保持只证明本次有限仿真窗口，不是长期或实机稳定性保证。

### 证据位置与保留边界

- 共用参数：[run_invocations.json](diagnostic_runs_20261010_standing/run_invocations.json)；两次结果：[run_results.md](diagnostic_runs_20261010_standing/analysis/run_results.md)、[run_results.json](diagnostic_runs_20261010_standing/analysis/run_results.json)。每个 run 目录内保存原始 `runtime_record.jsonl`、`terminal.json`、`run_context.json`、`process_facts.json`、`metrics.json` 和 `timeline.png`。站立窗口摘要为两个 `standing_observation_summary.json`。
- random_04 的实际画面：[standing_hold.png](diagnostic_runs_20261010_standing/scene_random_04/19b4b20771ec4b3baed9ef02c23b23f1/standing_hold.png)（由 MuJoCo `mjr_readPixels` 直接保存的 framebuffer PPM 无损转换为 PNG）；画面可见机器人、提示文字和 INVALID/No current policy data。flat 的首次输出路径为相对路径，没有保存画面；此问题已在脚本中改为绝对路径并由 random_04 截图验证。遵守两次场景运行上限，未为补平地截图启动第三次运行。平地站立时间序列和实测状态完整保留。
- flat 原始 `run_context.json` 中首版 `standing_observation.sample_count=743` 包含减速和过渡阶段，不是单独保持窗口数；原文件保持不变。随 run 保存的 `standing_observation_summary.json` 依据其仿真时间起止筛选出保持窗口内的 418 个样本。random_04 运行使用修正后的采样字段，保持样本数为 372。
- flat 和 random_04 的碰撞分析均没有获得可用于全运动窗口“无碰撞”结论的完整覆盖边界；terminal 汇总的 fall history 也不能按全 run 外推到保持窗口。本报告只说保持窗口采样中未见跌倒候选/确认或机器人障碍接触，不声称整段运行无碰撞或无跌倒。random_04 有 17 个无效策略时钟帧和 1 个周期缺口，时间类指标按分析器有效区间解释。其 115 次模式变化只记为本次周期结果，不作为切换性能结论。
- 之前两次零策略帧 INVALID 启动数据以及原趴下方案进入保护停止的历史数据均未改写；新结果不替代旧结果。动作关节语义、策略停止确认、MuJoCo 命令消费和实机运行适用性等既有 UNKNOWN 继续保留。

两次诊断均达到 0.5 m 目标半径并连续通过 5 仿真秒站立保持，健康路径没有进入趴下或卸力。Director 有条件通过 S2-07 并将其标记为 **COMPLETE**。历史限制、停止发布器退出码 1、平地截图缺失及旧方案失败记录均保留；当前没有已授权的下一项 Active Task。
