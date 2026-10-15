# S6-12 场地边界 reader 时序定向验证

## 范围

本补证只验证 `scripts/abs_base_bounds.py::read_snapshot()` 与 `scripts/s1_run_diagnostic.py` 的 pre-RL 场地边界准入helper。没有启动MuJoCo、GLFW、ROS、策略或批次续跑；没有修改仿真publisher、研究参数、安全判据、地图或原始运行数据。

## 修正

默认生产调用现在先读取完整且seqlock一致的共享快照，再调用单调时钟取得`validation_monotonic_ns`。future判断和年龄/过期判断共用该校验时刻。显式`now_ns`仍保留给确定性离线调用，定义为调用方提供的校验时刻；生产调用省略它，避免传入读取开始前的时间。snapshot返回数据附带校验时刻及`age_ns`。

pre-RL gate使用生产helper检查reader结果，必须同时满足状态LIVE、`bounds_valid=1`和`physics_coverage_complete=1`。不满足时继续拒绝RL。`command 3`仍位于这一失败分支之后；本补证不改变命令路径或准入标准。

## 原始失败的证据边界

原始attempt：`logs/s6_12/b_exit_coarse/20261015_192706_3/runs/002_m1ssd01_x17_r1/result/B/rep_01/33a9ddddc59a44e3a41e358e95b3704f/`。

- RL准入所用base-bounds快照：scene `m1ssd01`，physics step 3129、sim_time约6.258 s、sequence 6260；capture与模型指纹对应本次run，bounds有效且publisher coverage complete。
- 原reader结果：`INVALID / future_timestamp`，`age_ns=-163750`。
- context的`observed_monotonic_ns=37634655105467`属于独立物理/碰撞快照读取；base-bounds快照timestamp为`37634656408048`。历史context没有保存base-bounds reader自己的validation time，因此不能用前者替代reader校验时刻，也无法直接证明发布操作落在竞态窗口内。
- terminal中的`STALE / snapshot_expired`、age `854600025 ns`来自稍后的采集覆盖检查，物理step 3136、sim_time约6.272 s。它说明覆盖不完整，不是RL准入拒绝的直接快照。

源码确认reader在读前取时，允许并发生产更新被误判为未来；这与负年龄现象一致，但因原始记录缺少该reader校验时刻，不能宣称已从运行记录证明这是唯一根因。原run_context、terminal、metrics和日志均保持不变；该运行仍为`PREPARATION_FAILURE / INVALID`。

## 离线核验结果

执行：`rtk proxy /usr/bin/python3 scripts/test_b1_base_bounds_reader.py`

该检查写入临时文件，并调用生产reader及实际准入helper，结果PASS：

1. 通过受控I/O时序，在reader原本会取得旧时刻后、payload读取前发布一个更新快照；reader返回LIVE，且校验时刻在快照时间戳之后。
2. 明确注入真正早于snapshot timestamp的校验时间，仍为`INVALID / future_timestamp`。
3. age恰好500 ms仍LIVE；大于500 ms转`STALE`。原时限未变。
4. 场景身份错配、未知version、错误长度和奇数/不稳定seqlock仍拒绝。
5. 显式`now_ns`注入的年龄可复现，并返回被使用的validation time。
6. reader的LIVE有效完整快照通过边界gate；非LIVE、bounds无效或coverage不完整均被gate拒绝。生产主流程在该失败处抛出准备失败，command 3只会在该gate通过后继续。

受影响Python文件编译检查：`rtk proxy /usr/bin/python3 -m py_compile scripts/abs_base_bounds.py scripts/s1_run_diagnostic.py scripts/test_b1_base_bounds_reader.py`，PASS。

未运行publisher→reader MuJoCo握手；本轮只授权离线验证。修正后的pre-RL快照在真实运行中的LIVE结果仍UNKNOWN，需Director另行决定是否补做计划项2。
