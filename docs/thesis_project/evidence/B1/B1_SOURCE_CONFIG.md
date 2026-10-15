# B1 运行来源与配置核对

下表逐run摘自原始`run_context.json`和预检记录，不靠命令行推断。各运行实际加载的controller共享库、三个模型artifact、独立配置overlay、group/E/X/H/goal及碰撞场景身份均有来源绑定。完整PID、session、绝对路径和证据细节由链接的原始run_context保留。

| 运行 | 场景身份 | 组别配置实际观察 | 来源/控制器库/场景物理绑定 | 独立overlay与run证据 |
|---|---|---|---|---|---|
| flat_A_01 | scene_flat.xml / `flat` | A: E=-0.05000000074505806, X=-0.05000000074505806, H=0.0;滞回=False,保持=False;goal=[7.5, 0.0] | confirmed / confirmed / LIVE | overlay独立；[run_context](../../../../logs/b1/runs/flat_A_01/A/rep_01/be491dadee3f4f229e3f6ddc8c1034d0/run_context.json) |
| flat_A_02 | scene_flat.xml / `flat` | A: E=-0.05000000074505806, X=-0.05000000074505806, H=0.0;滞回=False,保持=False;goal=[7.5, 0.0] | confirmed / confirmed / LIVE | overlay独立；[run_context](../../../../logs/b1/runs/flat_A_02/A/rep_01/e0d59210618a41c98138380c04480322/run_context.json) |
| m1ssd01_A | dev03.xml / `m1ssd01` | A: E=-0.05000000074505806, X=-0.05000000074505806, H=0.0;滞回=False,保持=False;goal=[7.5, 0.0] | confirmed / confirmed / LIVE | overlay独立；[run_context](../../../../logs/b1/runs/m1ssd01_A/A/rep_01/2788e030fbdb496cbb6d77d58f0a5ddb/run_context.json) |
| m1ssd01_B | dev03.xml / `m1ssd01` | B: E=-0.05000000074505806, X=-0.07999999821186066, H=0.0;滞回=True,保持=False;goal=[7.5, 0.0] | confirmed / confirmed / LIVE | overlay独立；[run_context](../../../../logs/b1/runs/m1ssd01_B/B/rep_01/923705d16d3b4e63b66f8e6ef3a95702/run_context.json) |
| m1ssd01_C | dev03.xml / `m1ssd01` | C: E=-0.05000000074505806, X=-0.05000000074505806, H=0.3;滞回=False,保持=True;goal=[7.5, 0.0] | confirmed / confirmed / LIVE | overlay独立；[run_context](../../../../logs/b1/runs/m1ssd01_C/C/rep_01/69ddda58e6bd4d8ca9d4ed23e556a84b/run_context.json) |
| m1ssd01_D | dev03.xml / `m1ssd01` | D: E=-0.05000000074505806, X=-0.07999999821186066, H=0.3;滞回=True,保持=True;goal=[7.5, 0.0] | confirmed / confirmed / LIVE | overlay独立；[run_context](../../../../logs/b1/runs/m1ssd01_D/D/rep_01/b735497bb66345619c599e254b108254/run_context.json) |
| m1dod01_D | dev25.xml / `m1dod01` | D: E=-0.05000000074505806, X=-0.07999999821186066, H=0.3;滞回=True,保持=True;goal=[7.5, 0.0] | confirmed / confirmed / LIVE | overlay独立；[run_context](../../../../logs/b1/runs/m1dod01_D/D/rep_01/0f148c54215e409fbbcacb23955510de/run_context.json) |
| m1dod01_C | dev25.xml / `m1dod01` | C: E=-0.05000000074505806, X=-0.05000000074505806, H=0.3;滞回=False,保持=True;goal=[7.5, 0.0] | confirmed / confirmed / LIVE | overlay独立；[run_context](../../../../logs/b1/runs/m1dod01_C/C/rep_01/a0c8d5a30cbb46718ccd1acb7cb7f994/run_context.json) |

所有配置值来自controller来源核对及`runtime_config_observed`，不是只检查shell查包路径。A/C有效退出值由E推导，B/D使用X=-0.08；C/D H=0.30仿真秒。所有配置均为本次运行独立副本。场景ID和碰撞reader物理authority在每次运行均匹配，并为LIVE。控制器PID在同次来源事件中与writer PID一致。

## 原10项批次中未运行的计划组别（不含后续补证）

原10项批次中的密集图`m1dod01` B、A未运行，没有对应controller PID、overlay、writer或session；后续另获准的A补证见下表，不能与原批次拼接。

## 唯一dev25 A组运行补证（原主批之外）

| 运行 | 场景 | 实际组别/阈值/目标 | 来源与构建 | 证据 |
|---|---|---|---|---|
| [A 单次补证](../../../../logs/b1/runs/m1dod01_A/attempt_01/A/rep_01/a79d150a1e544bc98c75f4db38345b94/run_context.json) | `dev25.xml / m1dod01`，authority LIVE | A；E=-0.050000000745、有效退出阈值同E；hysteresis=false、hold=false、H=0；goal=[7.5, 0.0] | controller PID=448050、writer PID=448050；controller共享库为当前workspace install路径；MuJoCo PID=447921，实际执行`unitree_mujoco/simulate/build2/unitree_mujoco`。三个模型均从本次独立overlay加载；runtime source validation=`confirmed` | session=26364708805493；独立overlay、配置、模型绝对路径及场景模型指纹见run_context |

来源校验由run_context内的启动日志、controller配置消费日志及本次session交叉确认；不以shell查包路径代替实际消费证据。

## 最新：dev03 D 收尾顺序真实核验（2026-10-15）

| run | 实际消费 | controller / writer / session | 场景与模型来源 | 证据 |
|---|---|---|---|---|
| [一次D收尾核验](../../../../logs/b1/closeout_runtime_verification/dev03_D_20261015/D/rep_01/6a0e402de1754d9a828704976d8ee32f/run_context.json) | `combined_switch`，D；E=-0.05、X=-0.08、H=0.30s；hysteresis=true、hold=true；goal=[7.5,0] | PID=492425；writer PID=492425；session=30612695857800 | `dev03.xml / m1ssd01`；controller、Agile/RA/Recovery模型由本次独立D overlay加载；场景authority LIVE | run_context `runtime_source_validation=confirmed`；[批次计划](../../../../logs/b1/closeout_runtime_verification/dev03_D_20261015/batch_plan.json)、[ROS日志](../../../../logs/b1/closeout_runtime_verification/dev03_D_20261015/D/rep_01/6a0e402de1754d9a828704976d8ee32f/ros_launch.log) |

cycle 0伴随实测状态sim_time=6.526s，有效；reset构造位姿为[0,0,0.445]m、单位四元数，reset速度仍UNKNOWN。完整关节初态见[初态对照表](B1_INITIAL_STATE_COMPARISON.md)。

## Director重审后的补充B尝试（均INVALID）

以下来源来自各自run_context，不与主表八条合并。两次均确认B配置、dev25/m1dod01、7.5 m目标、独立overlay、controller/writer PID与session绑定一致；记录有效性失败发生在runtime帧写入顺序。

| 运行 | controller / writer PID | session | 实际组别与参数 | 来源证据 |
|---|---:|---:|---|---|
| [B首次尝试](../../../../logs/b1/runs/m1dod01_B/B/rep_01/6edf02f841a64b508c25235aad9deba2/) | 424162 / 424162 | 23138294148070 | B；E=-0.05，X约-0.08，H=0；目标[7.5, 0.0] | runtime source confirmed；该run独立overlay与模型绝对路径见run_context |
| [B唯一重试](../../../../logs/b1/runs/m1dod01_B_retry_01/B/rep_01/8fa9e43785f34f41b73aa26af7b5ea34/) | 426647 / 426647 | 23258341867747 | B；E=-0.05，X约-0.08，H=0；目标[7.5, 0.0] | runtime source confirmed；该run独立overlay与模型绝对路径见run_context |

两次都保存了有效的cycle 0及完整12关节q/dq companion；详见[初态对照](B1_INITIAL_STATE_COMPARISON.md)。实际终局为ARRIVED，但runtime因周期时间、rl_step及source sequence不递增而INVALID，故不纳入有效指标。


## dev03 C/D 单次补证来源（2026-10-15）

| 运行 | 组别/实际消费 | controller=writer PID / session | controller配置及模型 | scene/authority |
|---|---|---|---|---|
| [C](../../../../logs/b1/cd_runtime_verification/dev03_C_D_20261015_170521/C/rep_01/e1243039761e43ddaa909f431bb5011c/run_context.json) | candidate=`minimum_hold_switch`; E=-0.05; 有效X=E; H=0.30s; hysteresis=false; hold=true; goal=[7.5,0] | 469068 / 28502028731893 | 本批独立overlay C；Agile、RA、Recovery均从该overlay加载；runtime_source_validation=confirmed | `dev03.xml / m1ssd01`；terminal safety snapshot LIVE |
| [D](../../../../logs/b1/cd_runtime_verification/dev03_C_D_20261015_170521/D/rep_01/e8cae530962645ca8f0d25b0359e0345/run_context.json) | candidate=`combined_switch`; E=-0.05; X=-0.08; H=0.30s; hysteresis=true; hold=true; goal=[7.5,0] | 473582 / 28530263446873 | 本批独立overlay D；Agile、RA、Recovery均从该overlay加载；runtime_source_validation=confirmed | `dev03.xml / m1ssd01`；terminal healthy gate LIVE/eligible |

同一控制器构建、场景、模型和准备流程；cycle0各自有真实12维q/dq companion及source/session身份。两次独立初态不构成严格重放。D的整条runtime记录为INVALID，来源确认与记录完整性分开。
