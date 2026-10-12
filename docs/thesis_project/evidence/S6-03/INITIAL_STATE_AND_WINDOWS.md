# S6-03 初态、首周期与任务窗口证据

本文件区分实现/离线检查与实际场景证据。首批 A 曾在 controller/MuJoCo/recorder 启动前发生配置预检 INVALID，原始记录保留。之后用户重新授权的一次 A 场景运行启动了 recorder、MuJoCo 和 controller，但固定站立准备时检测到持续确认跌倒，RL 被拒绝；仍无首策略周期或同周期实测初态。下列规则不能误读为已由本次运行完整验证。

## 三类初态

1. **构造/reset 状态**：由受支持 scene 的 MuJoCo 初始化探针取得 qpos/base pose。reset qvel 当前探针未输出，不能用训练默认关节姿态代替实测。
2. **RL 前准备状态**：经过统一 PASSIVE/FIXEDSTAND 稳定门槛后，从 ROS `/joint_states` 采集按名称排列的 12 关节 q/dq；记录 monotonic 采样时间及到 RL 请求/首 active 周期的差，不宣称同周期。
3. **首个 active policy 周期**：controller 的 `[ABS-INITIAL-STATE]` companion 记录与 supervisor 同步持久化 JSONL 首帧必须在 `(session_id, cycle_id/rl_step, frame_monotonic_ns)` 上精确匹配；否则首周期初态状态为 UNKNOWN 并阻止把该组视为准备就绪。

Companion 输出 joint order、q/dq、IMU quaternion 原值、angular velocity、策略机身线速度、世界 XY/yaw、session/cycle/时钟 identity。当前 C++ 字段把姿态顺序标为 `framework-native`；其底层分量顺序与参考系没有找到直接、充分证据，不能据此算姿态相似度。ROS joint sample 的 header/source 与 monotonic 采样时间仍需按实际记录解释。

## 任务时间与安全边界

- primary task start = 首个持久化 `rl_entered=1 && rl_active=1` 周期；保留 cycle identity。该周期仿真时间无效时，时间指标 UNKNOWN，不把后续首个有效时间当作新的起点。
- supervisor 从该策略周期开始执行到达、碰撞/跌倒/安全终局和仿真时限判断。运行时使用任务开始时的有效 sim time 筛选累计 PhysicsLoop episode；若该时钟无效，使用最后 pre-RL LIVE boundary physics step 保守判断。分析器还要求时间边界及覆盖完整，不能仅凭运行时 fail-safe 机制报告“无碰撞/无跌倒”。
- analyzer 主窗口为首 active 周期至真实 terminal 周期（含 terminal payload）。同一策略周期边沿即使仿真时钟无效仍计数；有效连续时间/位移只累加连续、时钟有效的相邻间隔。清理、到达后站立观察和 terminal 后事件不计入运动指标，但继续保存在记录中。
- 旧 `>=0.10 m` 位移边界保留在 `legacy_displacement_window_metrics`。旧运行及历史报告均未回写。

## 首批直接证据及 UNKNOWN

A run `[local run ID omitted]` 的 run context（原始运行/环境明细保留本地，未纳入分享提交） 记录 scene filename/ID/fingerprint、目标和计划阈值，但 `frames_written=0`、first active state 为 `strategy_capture_not_started`、joint state pending、controller/MuJoCo/publisher 均未启动。不能声称该首批实际场景/模型/controller 来源加载，也没有任何 task-start 安全结论。

针对性离线用例调用生产 `extract_first_active_policy_state()`：当前要求完整 `rl_step=0` payload，companion/record 的 session、cycle、monotonic identity 必须精确配对；后续周期即使身份彼此一致也返回 UNKNOWN。批次工具现对本轮实际 A/D overlay 调用生产 validator 并通过数值字段检查；只有本轮 A controller 日志证明了配置实际消费。首批 A 的阈值字符串拒绝事故及修复见 [执行报告](S6-03_EXECUTION_REPORT.md)。

## 本轮 A 运行及初态边界（2026-10-12）

本轮 A `[local run ID omitted]` 的 run context（原始运行/环境明细保留本地，未纳入分享提交） 证明 recorder meta 在 RL 前已 flush/readback、运行场景 random_04 authority LIVE、A 配置实际消费、唯一活动 controller/writer 和 stop publisher 已绑定。固定站立检查发现跌倒 episode 从 sim 0.25 s/step 125 开始、0.55 s/step 275 确认，并在 sim 26.128 s/step 13064 仍持续；故拒绝进入 RL。具名 `/joint_states` q/dq 和最终 RL 前 safety-boundary 采样没有执行。reset probe 记录 qpos/base state，但 reset qvel 未知，且它不能替代同周期初态。

该运行没有策略周期、`rl_step=0` payload 或 companion；任务窗口不存在，不能给出任务期碰撞/跌倒否定结论。supervisor 原始终局为 PREPARATION_FAILURE，记录有效性为 INVALID/no_frame_lines。详细批次计划、自动表、分析产物及清理事实链接见 [S6-03 最新补证报告](S6-03_EXECUTION_REPORT.md)。D 按失败停止规则未启动。

## 后续准备专用观察（2026-10-12）

用户单独授权的一次准备观察 run `[local run ID omitted]` 在同一 random_04、A overlay 下使用 `--prepare-only`；实际没有策略请求、command 3、策略帧或任务窗口。其 run context（原始运行/环境明细保留本地，未纳入分享提交） 保存固定站立期间不同物理 sequence 的姿态、支撑、tracker、episode 与控制器状态；派生 CSV（原始运行/环境明细保留本地，未纳入分享提交） 为便于阅读的同一批采样。

此次观察到当前 confirmed fall episode 在 base_link 高度恢复越过原 0.22 m 门槛后关闭，历史确认事件没有清除。其后 sim 26.716–27.222 s 满足原连续稳定门槛。说明生产 tracker 能识别恢复，修正后的准备函数不会在第一次确认事件处提前结束。它不能证明 cycle-0 初态或策略任务安全，也不是重复启动可靠性验证；最后实测固定站立 phase=0.684，controller 内部 stand ramp 完成时刻未测得。

离线 analyzer/绘图在历史 D 的临时副本上成功生成结果文件；它把已有确认跌倒 episode 与首 active 起点交叠识别为安全冲突，并因缺起点之前 boundary snapshot 将碰撞判为 UNKNOWN。复核 JSON 在 historical_task_window_recheck.json（原始运行/环境明细保留本地，未纳入分享提交）。它不是新场景运行证据，不覆盖历史 S6-01 保存的指标/报告。
