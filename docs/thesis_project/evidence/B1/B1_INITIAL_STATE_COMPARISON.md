# B1 实测初态与组间差异

每次均保存reset构造状态及cycle 0首个活跃策略周期companion。各组使用同一准备流程，不构成严格重放。reset探针只保存qpos，不提供reset关节速度；cycle 0的q/dq、机身速度及IMU四元数来自该run的真实周期companion，与首个持久化策略帧精确按session/cycle/monotonic身份配对。异步ROS具名关节样本和运动起点后测状态单独保留，不冒充cycle 0。

## 最新独立D收尾核验初态（不并入既有组间比较）

本次run为独立的收尾流程补证，不与旧D或其他组构成严格配对。reset probe记录base位姿[0,0,0.445]m、四元数[wxyz]=[1,0,0,0]；reset关节速度为UNKNOWN。首个活跃策略周期cycle0在sim_time=6.526s有效取得，session/cycle/monotonic身份为`[30612695857800, 0, 30612751885186]`，12维FR、FL、RR、RL具名关节q/dq齐全，且同周期策略帧已持久化。

| 运行 | cycle0身份 `[session,cycle,monotonic_ns]` | sim_time | 位置XY / yaw | q/dq | run_context |
|---|---|---:|---|---|---|
| [dev03 D收尾核验](../../../../logs/b1/closeout_runtime_verification/dev03_D_20261015/D/rep_01/6a0e402de1754d9a828704976d8ee32f/run_context.json) | `[30612695857800,0,30612751885186]` | 6.526s，有效 | `[-0.038547, 0.000012]m / -0.000070rad` | 12/12 | 本次独立补证 |

| 运行 | reset机身位姿 xyz m / 四元数 wxyz | cycle0保存/身份 | q/dq通道数 | reset dq | 实测首活跃周期文件 |
|---|---|---|---:|---|---|
| flat_A_01 | [0, 0, 0.445] / [1, 0, 0, 0] | 0 / `[20276013569964, 0, 20276078306231]` | 12/12 | UNKNOWN | [run_context](../../../../logs/b1/runs/flat_A_01/A/rep_01/be491dadee3f4f229e3f6ddc8c1034d0/run_context.json) |
| flat_A_02 | [0, 0, 0.445] / [1, 0, 0, 0] | 0 / `[20320669384149, 0, 20320739876624]` | 12/12 | UNKNOWN | [run_context](../../../../logs/b1/runs/flat_A_02/A/rep_01/e0d59210618a41c98138380c04480322/run_context.json) |
| m1ssd01_A | [0, 0, 0.445] / [1, 0, 0, 0] | 0 / `[20378384213098, 0, 20378455981732]` | 12/12 | UNKNOWN | [run_context](../../../../logs/b1/runs/m1ssd01_A/A/rep_01/2788e030fbdb496cbb6d77d58f0a5ddb/run_context.json) |
| m1ssd01_B | [0, 0, 0.445] / [1, 0, 0, 0] | 0 / `[20437164554575, 0, 20437229536293]` | 12/12 | UNKNOWN | [run_context](../../../../logs/b1/runs/m1ssd01_B/B/rep_01/923705d16d3b4e63b66f8e6ef3a95702/run_context.json) |
| m1ssd01_C | [0, 0, 0.445] / [1, 0, 0, 0] | 0 / `[20491549874003, 0, 20491623321964]` | 12/12 | UNKNOWN | [run_context](../../../../logs/b1/runs/m1ssd01_C/C/rep_01/69ddda58e6bd4d8ca9d4ed23e556a84b/run_context.json) |
| m1ssd01_D | [0, 0, 0.445] / [1, 0, 0, 0] | 0 / `[20566925581983, 0, 20566991560841]` | 12/12 | UNKNOWN | [run_context](../../../../logs/b1/runs/m1ssd01_D/D/rep_01/b735497bb66345619c599e254b108254/run_context.json) |
| m1dod01_D | [0, 0, 0.445] / [1, 0, 0, 0] | 0 / `[20627484135586, 0, 20627554976410]` | 12/12 | UNKNOWN | [run_context](../../../../logs/b1/runs/m1dod01_D/D/rep_01/0f148c54215e409fbbcacb23955510de/run_context.json) |
| m1dod01_C | [0, 0, 0.445] / [1, 0, 0, 0] | 0 / `[20682564898390, 0, 20682619670512]` | 12/12 | UNKNOWN | [run_context](../../../../logs/b1/runs/m1dod01_C/C/rep_01/a0c8d5a30cbb46718ccd1acb7cb7f994/run_context.json) |
| dev25_A_single_supplement | [0, 0, 0.445] / [1, 0, 0, 0] | 0 / `[26364708805493, 0, 26364776247825]` | 12/12 | UNKNOWN | [run_context](../../../../logs/b1/runs/m1dod01_A/attempt_01/A/rep_01/a79d150a1e544bc98c75f4db38345b94/run_context.json) |

该补证cycle 0 companion的12维q/dq顺序为FR、FL、RR、RL，每腿hip/thigh/calf；sim_time=6.860 s且有效，与同session的首个持久化策略帧精确匹配。异步`/joint_states`采样在RL step 19运动起点策略帧之后取得；保留其单独monotonic时间，不称为cycle 0或同周期实测值。此补证不纳入原八条组间初态差异统计。

## 周期0实测状态差异（同图，逐组相对首列基准）

以下为cycle0 companion实测向量的最大绝对分量差；描述差异，不作人为容差通过/失败，也不按结果挑选运行。

| 场景 | 对照 | max abs(Δq) rad | max abs(Δdq) rad/s | max abs(Δbody linear velocity) m/s | max abs(Δbody angular velocity) rad/s | max abs(ΔIMU quaternion) |
|---|---|---:|---:|---:|---:|---:|
| flat | A→A | 0.00041198 | 0.00768811 | 0.000958025 | 0.000726268 | 3.06545e-06 |
| m1ssd01 | A→B | 0.00167382 | 0.0786863 | 0.0114354 | 0.00425396 | 8.8552e-05 |
| m1ssd01 | A→C | 5.77439e-05 | 0.000162094 | 2.42441e-05 | 6.30542e-05 | 5.06614e-06 |
| m1ssd01 | A→D | 0.0017271 | 0.121493 | 0.0181191 | 0.00480997 | 0.000175873 |
| m1dod01 | D→C | 0.00179171 | 0.0470186 | 0.00751733 | 0.00184076 | 9.26782e-05 |

reset基准均为controller reset probe记录的初态，不能代表cycle0时已观测的真实姿态；不同运行cycle0状态存在实际变化。周期身份、q/dq完整性和逐run详细值可从链接的原始run_context核验。

## Director重审后的补充B尝试（不并入主批对照）

两次B运行的真实cycle 0 companion均已保存，具有同session/cycle/monotonic身份匹配及12维具名q/dq。它们不是严格重放；两条runtime记录都因后续帧顺序错误判为INVALID，故不纳入组间指标比较。

| 运行 | cycle 0身份 `[session, cycle, monotonic_ns]` | sim_time有效 | world XY位置 / yaw | q/dq | 来源 |
|---|---|---|---|---|---|
| [B首次尝试](../../../../logs/b1/runs/m1dod01_B/B/rep_01/6edf02f841a64b508c25235aad9deba2/run_context.json) | `[23138294148070, 0, 23138355717985]` | 是，7.108 s | `[-0.038919, 0.000014] m` / `-0.000077 rad` | 12/12 | StateRL实测companion，同周期runtime帧已持久化 |
| [B唯一重试](../../../../logs/b1/runs/m1dod01_B_retry_01/B/rep_01/8fa9e43785f34f41b73aa26af7b5ea34/run_context.json) | `[23258341867747, 0, 23258414189662]` | 是，7.050 s | `[-0.038894, 0.000014] m` / `-0.000077 rad` | 12/12 | StateRL实测companion，同周期runtime帧已持久化 |

初态采集链在两次无效运行中直接可用；初态成功采集不抵消后续帧失序，也不把运行转为VALID。


## dev03 C/D 新补证初态（不与旧run合并作严格配对）

两次reset probe均保存相同标称reset姿态；cycle0来自各run实际StateRL companion，含具名12维q/dq、机身线/角速度、IMU四元数、位姿及有效sim_time，并与同session首帧配对。reset q/dq速度字段仍UNKNOWN；异步具名`/joint_states`采样与cycle0不是同一时刻。

| 运行 | cycle0身份 `[session,cycle,monotonic_ns]` | sim_time | q/dq | 位置XY/yaw | 与另一组cycle0差值 | run_context |
|---|---|---:|---|---|---|---|
| C | `[28502028731893,0,28502115734923]` | 6.362s，有效 | 12/12 | 见companion | 基准 | [../../../../logs/b1/cd_runtime_verification/dev03_C_D_20261015_170521/C/rep_01/e1243039761e43ddaa909f431bb5011c/run_context.json](../../../../logs/b1/cd_runtime_verification/dev03_C_D_20261015_170521/C/rep_01/e1243039761e43ddaa909f431bb5011c/run_context.json) |
| D | `[28530263446873,0,28530329665089]` | 6.712s，有效 | 12/12 | 见companion | max abs Δq=0.002760rad；max abs Δdq=0.106660rad/s | [run_context](../../../../logs/b1/cd_runtime_verification/dev03_C_D_20261015_170521/D/rep_01/e8cae530962645ca8f0d25b0359e0345/run_context.json) |

准备流程相同，但cycle0实测状态有上述差异，不能称严格重放或完全配对。D的后续策略记录INVALID不抹去已持久化且边界匹配的cycle0 companion。
