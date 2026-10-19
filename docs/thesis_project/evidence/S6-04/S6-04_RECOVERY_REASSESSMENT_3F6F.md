# S6-04 手动观察 run 恢复判定重评

**用途：**对既有 run `[local run ID omitted]` 离线重算恢复结论。原始 `run_context.json`、`runtime_record.jsonl`、`terminal.json` 与 controller trace 均未修改；本文件是依据修正后生产 helper 得到的派生判断。没有重跑仿真。

原始数据目录：`logs/manual_startup_observation_4s/[local run ID omitted]/`（原始运行/环境明细保留本地，未纳入分享提交）。离线重放调用 `scripts/s1_run_diagnostic.py::new_startup_observation_state()` 与 `advance_startup_observation()`，输入只取该目录已保存策略帧所附的 MuJoCo physics snapshot；没有插值或伪造策略帧。

## 判定

- **修正后的结果：**低高度事件后已满足本诊断定义的恢复条件，最迟在已保存 physics snapshot 的 `sim_time=31.418 s` 确认。该 snapshot 的 `base_height=0.26759 m`、roll/pitch有效且在范围内、`fall_candidate=false`、`fall_confirmed=false`。对应低高度 fall episode 在31.112 s关闭；两时刻相差0.306 s，大于要求的0.30仿真秒。
- **修正后的观察终局：**用同一生产 helper 继续重放到原 run 最终权威快照 `sim_time=34.328 s`，没有再出现硬停止；4.002仿真秒观察上限到达时，派生终局应为 `STARTUP_OBSERVATION_LIMIT`，且 `recovered=true`。因此原始 `STARTUP_OBSERVATION_UNRECOVERED_LIMIT` 被纠正为“已恢复后正常结束观察窗”。
- **生产覆盖证据：**最终同 run v5 snapshot 为 `physics_coverage_complete=true`、`event_latches_complete=true`；fall history 计数为2、保存2项、无溢出。第一项是早于本次 RL 的历史 episode（0.25–28.014 s）；本次 episode 为30.532 s开始、30.834 s确认、31.112 s关闭。`invalid_posture_steps=0`、`unknown_contact_steps=0`、`unknown_contacts=0`，且 run 内没有新的碰撞或跌倒 episode。最终 `base_height=0.30657 m`，candidate/confirmed均为false。
- **reader 与 producer 的区别：**旧 run_context 在4秒观察内记下549次“physics sample gap”恢复计时重置；该数字反映 supervisor 没有逐个读取每个物理步。重放至恢复确认时处理了51个实际持久化物理快照，累计跨过495个未被 reader 单独看到的 physics step；重放到观察末端共处理197个 snapshot，跨过1804个未被 reader 单独看到的 step。同 run authority 的 sticky `physics_coverage_complete=true` 和完整累计事件历史说明这些不是 producer 覆盖断裂。故旧 `STARTUP_OBSERVATION_UNRECOVERED_LIMIT` 是监控器把 reader 采样间隔当成物理覆盖缺口造成的错误判定。
- **限制：**策略周期记录仍有2个周期缺失并有1个 STALE 策略时钟帧；恢复结论只使用同 run MuJoCo 物理 authority 的累计覆盖和实际保存快照。它说明这次低高度 episode 随后恢复到诊断姿态条件，不表示无跌倒、无接触、安全成功或普遍可恢复。

## 下沉、恢复与实际写出增益的时间关系

高度来自 runtime policy frame 中的真实 physics snapshot；Kp/Kd来自 controller trace 实际写入 ros2_control command interface 的值，以最近的真实 trace 时间点作对应（未插值）。

| 物理仿真时间 | 观测 | 相邻 controller trace 的软启动 step / Kp / Kd |
|---:|---|---|
| 30.428 s | cycle 0关联快照，base Z=0.3050 m | step 8，0.96 / 0.0208 |
| 30.542 s | 低高度首次可见，base Z=0.2080 m；episode开始 | step 36，4.32 / 0.0936 |
| 30.736 s | 本次最低已保存高度0.0760 m | step 76，9.12 / 0.1976 |
| 30.840 s | fall episode确认，base Z=0.0877 m，已经开始上升 | step 96，11.52 / 0.2496 |
| 31.112 s | fall history记录episode关闭 | 最近trace约step 152，18.24 / 0.3952 |
| 31.124 s | 首个越过0.22 m的后续保存快照：Z=0.2262 m | 最近记录为step 152 @31.114 s，18.24 / 0.3952（相差10 ms；不插值） |
| 31.418 s | 关闭后0.306 s，Z=0.2676 m；离线确认恢复 | step 212，25.44 / 0.5512 |
| 31.854 s | trace达到软启动step 250 | 30.0 / 0.6500 |
| 34.328 s | 观察末端，Z=0.3066 m；candidate/confirmed=false | 已到step 250，30.0 / 0.6500 |

这组时间对应显示下沉和回升发生时，接口增益仍从近零向配置目标爬升；它不能单独说明增益斜坡造成了下沉，也不能证明写出的命令已被 MuJoCo 消费。高度记录是稀疏策略帧关联快照，episode 开始/确认/关闭时间则来自 PhysicsLoop 累计 fall history。

## 离线核验

`scripts/test_s6_04_startup_observation.py` 的12项定向检查通过。检查覆盖：reader 稀疏而 producer 累计覆盖完整时可确认恢复；恢复区间有新 episode 时从最新关闭事件重起驻留；无效姿态、历史溢出、publisher 覆盖断裂、source变化拒绝恢复；累计历史不完整时返回证据不完整，而非“未恢复”。本轮没有运行仿真、改变安全阈值或修改原始记录。
