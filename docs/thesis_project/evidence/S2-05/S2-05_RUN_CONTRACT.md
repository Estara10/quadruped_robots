# S2-05 单次稀疏场景诊断合同

状态：本任务的非正式验证合同；不是正式性能实验参数冻结。

## 接触分类

- 足端只认模型中精确命名为 FL、FR、RL、RR 的 group-3 sphere collision geom。当前 Go2 MJCF 将这些 geom 挂在 FL_calf、FR_calf、RL_calf、RR_calf body 下；geom 身份来自模型名称、类型和 group，事件同时保存实际 owning body。这个 MJCF body 归属关系不改变四个碰撞球作为足端的项目定义。
- 未命名机器人碰撞 geom 通过 geom_bodyid 记录所属 body 名称。不能用窗口外观推断部位。
- v5 current_collision 和 collision_history 只表示非足端机器人—障碍碰撞；这类碰撞继续终止。v3/v4 的相同字段维持旧“任意机器人—障碍碰撞”含义。
- 足端—障碍接触都记录为 foot_obstacle_contact，不终止。对同一具名足端每个物理步，将已知外部接触的力（含地面和障碍）先按 mjContact.frame 的行向量从接触坐标转到世界坐标，并按足端处于 geom1/geom2 翻转受力方向；自接触不加进外力合力。
- 只有同一物理步存在足端—障碍接触、相关外部接触力完整且有限时，才计算 Fxy=hypot(Fx,Fy)、abs(Fz) 及 T=2*abs(Fz)+10 N。Fxy>T 是足端撞击事件，仍不终止；等于阈值不算撞击。缺力、方向无效、非有限数或接触分类未知时，撞击记 UNKNOWN。
- 足端接触与撞击历史以 foot geom/obstacle geom 身份和物理步为事件键；保存首末 step/time、force-known、撞击触发的 Fxy、abs(Fz) 及阈值。全局 contact/impact physics-step duration 只在连续物理覆盖上累加；缺口将 publisher coverage 置为 incomplete，不跨缺口估算。

## 记录与分析

新的 C++ snapshot 使用 v5、固定 14,472 字节；v5 扩展从 byte offset 5,760 开始。v3/3,864 字节和 v4/5,760 字节布局不变，Python reader 按“版本+长度”严格配对，错配、截断和未知版本拒绝。历史 v3/v4 不具备力与足端事件，不能由旧记录推断无足端接触或无足端撞击；旧碰撞字段仍按原语义分析。

每次分析分别输出：

- navigation_terminal_result：安全中止、非足端碰撞、跌倒、到达或诊断时限；
- nonfoot_collision_failure；
- foot_contact_count 和 foot_contact_duration_s；
- foot_impact_count 和 foot_impact_duration_s；
- arrived_with_foot_contact、arrived_with_foot_impact；
- abs_reference_collision：非足端碰撞或足端撞击为 true。

所有运动统计只用观察到的运动起点到终止周期（含）范围；清理尾段排除。记录缺失、时钟/physics coverage 不完整、事件历史溢出或关键力未知时，相关否定结论保持 UNKNOWN。时间线用不同标记显示足端接触、足端撞击和非足端终止碰撞。

## 本任务唯一运行

- scene：scene_ppt_sparse.xml / ppt_sparse，四个绑定障碍；
- A 组：paper_faithful_switch，沿用当前模型、阈值、控制频率与安全检查；
- 目标 (7.0, 0.0) m，到达半径 0.5 m；
- 最大运动时间 10 MuJoCo 仿真秒，墙钟保护 120 秒；
- 不暂停、不重置、不热更新，不因未出现某类接触重跑；
- PASSIVE 请求和 controller 确认使用 S2-03 已有路径。控制接口确认不等于 MuJoCo 已消费命令。

证据采用独立 run_id、真实配置与 writer/session、逐周期快照、terminal 和 closeout。该运行是非正式诊断，不解释为成功率、性能或跨场景结论。
