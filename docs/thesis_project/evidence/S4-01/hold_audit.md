# S4-01 A/C 保持审计

## 最新 C 运行

对应 `logs/manual_ab_compare/20261011_151659_d4a7f6/C/[local run ID omitted]/`。全部 884 个策略帧均为 v3、group=C、hold enabled、H=0.30 s；仿真时钟有 16 个无效帧和 1 个策略周期缺口。

| 检查 | 直接记录 | 判断范围 |
|---|---|---|
| Agile→Recovery 起算 | 19 次进入，进入周期记录 `hold_enter_sim_time_s`，elapsed 从 0 开始。 | 运行中观察到。 |
| H 未满且 RA<E | 219 个有效 elapsed 周期中，RA<E 时 `mode_after` 仍为 Recovery。 | 未观察到提前退出；无效时钟周期不用于此计数。 |
| H 到期且 RA<E | 19 次实际退出均在观测 elapsed >=0.300 s，且当周期 RA<-0.05；边沿周期仿真时间有效。 | 观察到按退出条件切回 Agile。退出 elapsed 范围 0.300–0.660 s；较长值与缺周期/无效时间覆盖有关，不能逐帧补插。 |
| H 到期但 RA>=E | 22 个 elapsed>=0.300 s 且 RA>=E 的周期继续处于 Recovery。 | 运行中观察到到期后高风险继续保持。 |
| Recovery 中风险再次升高 | 有 5 次 `risk_condition_entered`；每次的 `hold_enter_sim_time_s` 与该 episode 最初进入 Recovery 的起点完全相同。 | 本次可见事件未重启计时。 |
| episode 覆盖 | 19 个 episode 均记录退出和 `hold_rule=PASS`；10 个时钟覆盖完整，9 个因缺帧/无效时钟覆盖不完整。分析总状态 `incomplete`，无违规项。 | 不将整体审计写成全周期完整 PASS；不跨缺口插值。 |

生产状态机的离线检查还覆盖恰好到期、无效/回退时钟和新会话重置；本节运行结论只来自上述本次 C 原始周期记录。没有此次数据覆盖的细节仍由离线证据支持，不能替代运行证据。

## 同代码 A/C 对照来源

- 新 A：`[local run ID omitted]`，v3，VALID / COLLISION_TERMINATION，E=-0.05，无保持。451 帧，137 次模式边沿；Recovery 有效片段 3.430 s，但 episode 覆盖不完整。
- 新 C：`[local run ID omitted]`，v3，VALID / ARRIVED；到达后站立 5.006 仿真秒。两组使用同一安装控制器 Build ID、模型和 `scene_random_04.xml` / S2-08-r1；C 仅增加 H=0.30 s。

A/C 指标对照见 `logs/manual_ab_compare/20261011_151659_d4a7f6/comparison.md`。本轮首次 C 启动失败 `[local run ID omitted]` 仍保存在先前运行目录，不改写；它只有初始化配置日志、0 策略帧，不属于保持审计样本。

## 环境与限制

C 的实际 MuJoCo framebuffer 保存为原始 `standing_hold.ppm`，同目录 `standing_hold.png` 是该原始图像的无重绘格式转换。画面显示 `ARRIVED - HOLDING STAND`，面板状态为 `INVALID / No current policy data`，因为策略已经停止；没有取得实时 C 策略面板截图。清理顺序为 MuJoCo 退出码 0 后 ROS/controller 退出码 0；stop publisher 退出码 1，未强杀，整体 shutdown/closeout 均确认完成。运行后未发现本次相关项目进程残留。
