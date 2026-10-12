# S6-04 执行报告

**最新状态：COMPLETE（Director 有条件通过，2026-10-12）。** 通过范围为显式仿真启动观察、实际增益与动作记录、恢复判定修正，以及本次低高度事件随后恢复的证据。原始 `terminal.json` 中的 `STARTUP_OBSERVATION_UNRECOVERED_LIMIT` 保留；生产 helper 离线重评为已恢复，并在观察上限正常结束。reader 采样跳步不等于 producer 物理覆盖断裂。

限制：本次发生过低高度确认及触地，不能称为无跌倒或安全成功；单次恢复不证明普遍安全；策略缺帧和 STALE 保留；增益爬升与下沉时间重叠，不证明唯一因果；平滑交接尚未实现；普通运行规则未修改，不能视为已解决普通任务早停；MuJoCo 具体命令消费仍为 UNKNOWN。原始 terminal、JSON、历史失败及派生重评均保留。本轮文档同步未运行测试或仿真，未改代码、配置或原始数据。

## 最新定向修复：reader 采样间隔不再重置恢复驻留（2026-10-12）

完整的原始核对、派生时间线及增益对应见[本次恢复重评](S6-04_RECOVERY_REASSESSMENT_3F6F.md)。

- **生产 helper 修正：**`advance_startup_observation()` 现在要求同 run 的 physics authority 为 LIVE、v5、身份稳定、publisher `physics_coverage_complete=true`、`event_latches_complete=true`，且 collision/fall history 无溢出。它核对 fall_history 数量与实际条目数、episode 时间顺序和闭合状态，并检查 unknown-contact 与 invalid-posture 累计及其首次/末次发生时刻。reader snapshot 的 `physics_step` 跳跃只登记为未被 reader 单独读取的步数，不再重置恢复驻留。
- **恢复判定：**发生启动低高度后，恢复驻留起点取同 run 完整 fall history 中相关 episode 的 producer 关闭时刻。若区间出现新 fall episode，起点更新到该 episode 关闭时刻；出现未关闭 episode、无效姿态、未知接触、历史溢出、source 变化、真实 publisher 覆盖断裂、时间/身份倒退或历史缺失/不一致时，均不能确认恢复。证据不足在观察到期时分类为 `STARTUP_OBSERVATION_RECOVERY_UNKNOWN_LIMIT`，不冒称“未恢复”。正常姿态端点仍须满足高度、倾角及 candidate/confirmed 条件。
- **指定 run 的原始证据：**run `[local run ID omitted]` 的原监控器重复重置驻留549次；最终同 run authority 有连续物理覆盖、完整事件锁存、2/2 fall history且无溢出、`invalid_posture_steps=0`、`unknown_contact_steps=0`。启动窗口内低高度 episode 于30.532 s开始、30.834 s确认、31.112 s关闭。原始末快照时间34.328 s，base Z约0.30657 m，candidate/confirmed均为false。
- **同 helper 离线重算：**读取原始 runtime 中策略帧关联的51个真实 physics snapshot，累计 reader 未单独看到495个物理步。因 producer 的连续覆盖与累计记录完整，这些读取间隔不构成物理生产覆盖缺失。周期52关联的 snapshot 在31.418 s、episode关闭后0.306 s，base Z约0.26759 m，姿态有效、candidate/confirmed=false，故生产 helper 确认 `RECOVERED_AFTER_LOW_HEIGHT_EVENT`。继续重放至原始最终 authority snapshot（34.328 s）时，生产 helper 给出派生终局 `STARTUP_OBSERVATION_LIMIT` 且 `recovered=true`；原观察4.002仿真秒已完整结束。这只表示该次启动低高度随后恢复至本诊断姿态条件，不等于无跌倒、无接触或安全成功。
- **增益/高度关系：**controller trace 中实际写到命令接口的 Kp/Kd 从近零上升。低高度首次出现约30.542 s时约4.32/0.0936；最低保存高度约0.07598 m（30.736 s）时约9.12/0.1976；episode关闭31.112 s附近约18.24/0.3952；31.418 s恢复确认附近约25.44/0.5512；step250约31.854 s达到30/0.65。数据表明下沉、回升与软启动重叠，但不能证明增益斜坡是原因；接口写出不证明 MuJoCo 消费。
- **验证：**`scripts/test_s6_04_startup_observation.py` 12/12 通过；覆盖稀疏 reader + 完整累计覆盖可恢复、区间新异常推迟恢复起点、无效姿态、历史溢出、真实覆盖缺口、来源变化和历史不完整时拒绝/UNKNOWN。`py_compile` 通过。本轮未启动仿真，原始数据保持不变。

## 最新定向修复：手动观察时钟与恢复后续看（2026-10-12）

### 复核的原始记录

用户指定的 run_context.json（原始运行/环境明细保留本地，未纳入分享提交） 确认：运行 `[local run ID omitted]` 的首个持久化策略周期为 cycle 0，周期1、2缺失；cycle 10 的 controller `sim_clock_status=STALE`，`sim_time_s=null`，旧逻辑因此置 `STARTUP_OBSERVATION_INVALID`。该终局没有确认跌倒。与其关联的同 run MuJoCo PhysicsLoop 快照为 `LIVE`、version 5、`physics_coverage_complete=true`，sim_time=25.800 s、physics_step=12900、unknown contacts=0、invalid posture steps=0。故障在于监督器把策略时钟质量错误升级成观察安全来源失败；物理记录不支持“已确认跌倒”。原始 run_context、runtime record 和终局均未修改。

### 修正内容

- `scripts/s1_run_diagnostic.py` 的显式 `--startup-recovery-observation-s` 模式现在独立轮询、校验同一 capture/run、模型与场景的 MuJoCo v5 authority。唯一的观察时钟、4秒 deadline 和恢复驻留使用该物理快照 `sim_time`；策略帧时钟无效、共享帧漏周期、策略帧暂时缺失只累计为策略记录质量问题，不会单独结束观看，也不会把物理时间填入策略帧或策略指标。
- 物理快照缺失/过期/无效、事件覆盖不完整、时间或步号倒退、source/run identity 改变、接触分类不确定、无效姿态、倾斜/翻倒、障碍碰撞及 ROS/controller veto 仍会结束观察。观察起点沿用与 RL entry 关联的同 run 保守物理时钟锚点；计时不因策略缺帧、恢复或安全事件重置。墙钟上限只负责防止仿真暂停时无限等待，不冒充仿真观察时长。
- 低高度 tracker 和全部物理累计事件保持原样。低高度事件满足原诊断定义恢复后记录事件与 MuJoCo 恢复时刻，但继续观察至指定物理时间上限；恢复后新确认跌倒立即按普通规则终止。确认低高度但观察到期仍未恢复则以跌倒终止；不称“安全成功”。
- context 与 runtime terminal 均记录停止原因、末次物理来源状态、物理样本/采样缺口、policy 缺失状态观察数、cycle 缺口数、策略时钟无效帧数（并单列 STALE 数）、是否确认低高度/是否恢复、恢复仿真时刻、实际物理观察时长及清理结果。policy 时间统计保留原有效性，缺失值不插值。该行为只在显式手动仿真观察模式生效，普通默认仍关闭。

### 定向验证及运行边界

- `scripts/test_s6_04_startup_observation.py`：10/10 项通过，覆盖 policy clock stale + physics LIVE、策略缺帧、物理 authority missing/stale/invalid、物理时钟倒退、run/source 变化、4 秒物理时限、恢复后继续观察、恢复后再次跌倒终止、碰撞优先停止和常规终局顺序。
- `scripts/test_run_record.py`：56/56 项通过；确认普通终端记录未带手动观察字段时原记录行为保持不变，显式模式的观察摘要以可选字段传到 terminal。
- `py_compile` 对诊断器、record writer 与定向检查脚本通过。
- 本轮没有启动仿真、模型、controller、ROS、采集器或图形窗口；未做实际恢复观察。因而本轮只证实逻辑和序列化路径，不证实 MuJoCo 下真实恢复状态。已有 run 的策略数据缺口继续限制逐周期分析。

### 用户可复制的 4 秒手动观察命令

```bash
rtk proxy /usr/bin/python3 scripts/s1_run_diagnostic.py \
  --output-root logs/manual_startup_observation_4s \
  --overlay <workspace-root>/quadruped_ros2_control_humble/install/go2_description \
  --group A --entry-threshold -0.05 \
  --scene scene_random_04.xml --goal-x 7 \
  --sim-limit-s 20 --wall-cap-s 120 \
  --standing-observation-sim-s 0 \
  --startup-recovery-observation-s 4 \
  --display :0 --xauthority current desktop-session authorization file \
  --classification 'S6-04 manual startup descent and recovery viewing only'
```

运行前沿用脚本的现有来源/记录器/显示预检，不通过则停止。运行后请保留新 run 目录并核对 `startup_recovery_observation`、`terminal.json`、`run_context.json` 和本次进程清理事实；本轮没有实际执行该命令。

## 原方案 B 的先前运行结果（历史记录保留）

完成了诊断专用记录和监督器修改、定向离线验证、必要构建，并按授权进行一次 A 组启动观察。普通运行默认仍不启用低高度延期；生产跌倒 tracker 没有放宽，仍累计 episode 和接触；倾斜、碰撞、来源/时钟异常及既有 controller 安全停止仍可立即终止。

本次场景运行 `[local run ID omitted]` 使用 `scene_random_04.xml` / `random_04`（S2-08-r1），group A、E=-0.05、有效退出阈值=-0.05、无滞回/保持、目标 `(7,0)`。sim-limit=20 s、wall-cap=120 s；显式启动观察上限为1.0仿真秒。实际 controller 来源日志确认候选、组别、阈值、目标、模型路径和唯一 writer/session；加载库路径匹配本轮工作区安装库，MuJoCo 使用 `simulate/build2/unitree_mujoco`。

准备期稳定站立与具名关节样本均取得。controller 的 `StateRL::enter` 日志与运行 session 对应；cycle 0 帧实际写入，后续 trace 记录了 cycle 0–3。pre-entry 权威时钟样本为 sim_time=27.640 s，RL 转换单调时间为 `21981735062715 ns`；保守窗口锚点与转换相隔约26.8 ms墙钟，终止期限设为锚点后1.0仿真秒。

supervisor 只持久化到两个 runtime policy frame：cycle 0 / source sequence 2 与 cycle 3 / source sequence 8；cycle 1、2 不在同一共享帧记录中。监督器因此以 `STARTUP_OBSERVATION_INVALID` 停止，没有用 controller trace 补造缺失的策略帧。controller trace 虽保存了 cycle 0–3，但没有每周期完整物理快照，不能替代缺失的 RA/模式/时钟/接触快照。

两个已保存周期的物理快照显示 base 高度约 `0.314 m → 0.300 m`，roll/pitch 均接近水平，四足地面支撑数为4，未见当前确认中的跌倒。观察只覆盖到约 sim_time=27.746 s（约0.106仿真秒的保守锚点区间），远未达到1.0秒上限或连续0.30秒正常恢复条件。结果不能说明“未恢复”，也不能说明“已经恢复”。

终局记录中的 `fall_events=true` 来自完整 PhysicsLoop 累计历史：对应 episode 在 sim_time 0.25 s 开始、0.55 s确认、23.81 s关闭，早于本次 RL 起点27.686 s，是本次准备期历史事件，不是本次任务窗口的跌倒。准备历史保留，未改写为运动期事件。记录到的两个运动策略周期未见障碍碰撞；因 runtime 周期缺失，不据此作完整窗口的无碰撞结论。

## 原运行的交接与增益证据（历史记录保留）

可复核的逐 controller 更新数据保存在 controller_startup_trace.jsonl（原始运行/环境明细保留本地，未纳入分享提交），原始共享策略帧、来源、终局和进程事实分别保存在同一目录的 `runtime_record.jsonl`、`ros_launch.log`、`terminal.json`、`run_context.json` 和 `process_facts.json`。

- trace 记录了实际测得的12关节 q/dq、关节目标、速度目标、前一动作、策略原始/裁剪动作、关节观测、姿态四元数、角速度、足接触观测、模式/动作来源，以及写入 ros2_control command interface 的 Kp/Kd/tau。cycle 0 previous action 为零；动作数组按模型输出与 controller 顺序分别记录。
- 实际增益不是用配置值代替：软启动 step 1–8 的 ratio 为0.004–0.032，对应首关节接口 Kp `0.12→0.96`、Kd `0.0026→0.0208`；cycle 0 的首条策略周期记录为 step 4，Kp=0.48、Kd=0.0104。cycle 1、2、3 首条记录分别为 step 8/12/16，Kp=0.96/1.44/1.92，Kd=0.0208/0.0312/0.0416。它们证明 controller 向 command interface 写出的值，不证明 MuJoCo 已消费。
- 本次 trace 中足接口序列化字段只写出重复的 `foot_force` 前缀。launch 日志中的 `[FOOT-LOANED]` 直接记录了实际顺序 `foot_force/FR, foot_force/FL, foot_force/RR, foot_force/RL`。本轮运行后已将 trace 序列化修正为包含 interface 名称，并重新构建安装；该修正没有第二次运行验证。接触快照有四足地面支撑/力，但本次无障碍接触事件，不能报告具名碰撞部位。
- policy-cycle trace 的 monotonic 采样间隔为约4.32 ms、1.86 ms、14.68 ms；它只有四个策略周期样本，且 runtime 共享帧漏掉两周期。没有无记录开销基线，不能归因或排除日志对调度的影响。

## 原方案 B 实施与验证（历史记录保留）

- `common/abs_safety_rules.h` 只增加当前/最近跌倒 episode 的原因位（低高度、倾斜）；未改原阈值、持续时间或 tracker 确认/关闭规则。v5 的既有 `reserved_flags` 传递此原因，布局大小不变；reader 与 recorder 输出该字段。
- `scripts/s1_run_diagnostic.py` 增加默认关闭的 `--startup-recovery-observation-s`，上限1.0秒并限制为本次 A / E=-0.05 / `scene_random_04.xml` / goal-x=7.0。仅当确认当前活动 episode 原因严格为低高度、姿态与来源有效且物理覆盖完整时，才延后 supervisor 的 FALL 终止；不改 tracker。连续周期/时钟/射线/接触覆盖失效立即判 INVALID。
- `StateRL` 的 trace 只由本次 `ABS_S6_04_STARTUP_TRACE=1` 启用，保存实际命令接口值和交接状态。trace 事后解析为独立 JSONL。trace 接口名称字段在本次运行后修正并增量重建。
- 针对性 Python 用例4项通过，覆盖低高度单因准入、混合/倾斜与数据无效拒绝、正常恢复姿态、硬停止优先级及 trace schema。`s1_05_safety_rules_test` 增加跌倒原因位断言，构建并运行通过。MuJoCo authority 与可执行文件构建成功；ROS `colcon build --packages-up-to rl_quadruped_controller --symlink-install` 三包构建成功。Python 编译检查通过。
- OpenGL 预检在默认沙箱中 GLFW 初始化失败；按已有授权以临时沙箱外调用、当前 `the active DISPLAY value` 和 `XAUTHORITY=current desktop-session authorization file` 通过隐藏窗口检查。renderer 为 Mesa hardware renderer，OpenGL 4.6 Compatibility Profile Mesa 23.x。没有修改桌面或系统授权。

## 原运行收尾与限制（历史记录保留）

观察失效后发布 command 1；controller 在发布开始后约2.202 ms确认进入 PASSIVE。该确认只到 controller 状态层，不代表 MuJoCo 命令消费。清理顺序先关闭 MuJoCo，再关闭 ROS/controller；run context 记录 `shutdown_complete=true`，相关进程事后列表为空。stop publisher 退出码1，未残留。探针窗口已销毁。

本轮没有改变策略动作、RA、切换规则、研究参数或普通运行跌倒门槛；没有实施平滑交接，也没有训练、正式实验、实机或 Git 提交。剩余问题是策略共享帧采集没有保存 cycle 1、2，因而无法判定启动低高度事件的恢复；posture/contact 不能与每个 trace 周期完整一一配对。需要 Director 审阅后再决定是否授权针对记录器逐周期交接的定向修正；本轮不追加运行。
