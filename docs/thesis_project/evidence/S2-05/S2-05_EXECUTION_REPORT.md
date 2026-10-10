# S2-05 执行报告

**状态：COMPLETE — Director 有条件通过。** 本报告只覆盖足端接触/撞击与非足端终止碰撞的区分，不表示 A 组性能通过或正式实验完成。

## 结果概览

- 足端轻微障碍接触现在单独记录，不触发终止；足端撞击按 Fxy > 2*abs(Fz)+10 N 记录，也不触发终止。机身、大腿、小腿等非足端与已绑定障碍接触仍触发终止。
- 新 snapshot 为 v5 / 14,472 bytes；v3/3,864 与 v4/5,760 的布局和历史字段语义保留。正常 Python reader、记录器和分析器已通过真实 C++ 发布快照核验。
- 唯一一次稀疏场景短诊断为 VALID / COLLISION_TERMINATION：实际观察到 RL 足端接触和足端撞击；终止来自同一障碍上的非足端碰撞。记录、分析表、时间线及 PASSIVE/清理证据均已生成。

## 接触规则与实际归属

足端身份严格按当前 Go2 MuJoCo 模型中的 FL、FR、RL、RR 四个具名 group-3 sphere collision geom 识别。定向模型探针直接核对其 owning body 分别是 FL_calf、FR_calf、RL_calf、RR_calf。这个归属来自 MJCF 的 geom/body 关系；虽然名称表示足端球，模型实际把 geom 放在 calf body 下。本次不会把 body 归属改写成一个未存在的 *_foot 所有关系。

无名机器人碰撞 geom 的 body 身份由 geom_bodyid 在发布时解析并随非足端碰撞 episode 保存。不能从画面猜测。示例见探针 nonfoot_identity geom=<unnamed> body=FL_thigh，以及真实运行中无名 geom 46 → body 12 RL_calf。

MuJoCo mj_contactForce 给出 contact frame 中的 6D wrench。实现用 mjContact.frame 的三行轴向量将力转到世界坐标；足端在 geom1 时将作用力反号，在 geom2 时保留符号。同一具名足端每物理步把已知外部接触力（包括地面与障碍）相加，自接触不计入外力。只有该步存在足端—障碍接触时才判断撞击。方向/接触未知、约束力不可用、非有限值或覆盖缺口均不能作为轻微接触/无撞击处理。MuJoCo 口径是依据 ABS 条件构造的力学对应定义，并不声称与 Isaac Gym 的 rigid-body net contact force 数值完全等价。MuJoCo API 背景见 [MuJoCo contact force 文档](https://mujoco.readthedocs.io/en/3.2.0/programming/simulation.html)。

## 实现和定向核验

- common/abs_collision_contract.h 将新字段追加在 v4 的 5,760-byte 前缀后，新增 v5。offsetof(v5)=5760、C++ sizeof=14472 与 Python struct size 一致。v5 的 current_collision 和 collision_history 现在只代表非足端障碍终止事件；原始 robot_obstacle_contacts 仍统计足端与非足端的合计。
- unitree_mujoco/simulate/src/foot_contact_policy.h 与 obstacle_collision_authority.h 识别四个具名足端、换算/聚合世界力、应用严格 > 边界、记录足端/障碍身份和撞击触发力值；非足端 history 另外保留 body ID/name。
- scripts/abs_collision.py 只接受明确的 version/size 配对，解析足端事件；v3/v4 保持旧碰撞语义。scripts/run_record.py 序列化并严格校验 v5 拆分计数、力向量、足端/非足端事件和 body 身份。scripts/s1_run_diagnostic.py 将终止事件明确命名为 non-foot collision 并记录 body。scripts/s1_analyze_run.py 在运动边界内生成单次结果指标与三类事件时间线；清理尾段不进入运动指标。使用说明补充在 evidence/S2-04/SINGLE_RUN_GUIDE.md，本次口径见 [S2-05_RUN_CONTRACT.md](S2-05_RUN_CONTRACT.md)。
- 必要增量构建通过：unitree_mujoco 与 s2_05_contact_probe。
- C++ 定向检查通过：严格阈值低于/等于/高于测试；足端 geom1/geom2 方向反号测试；多个外力相加及 NaN/Inf/UNKNOWN；超阈值 impact 分类与足端非终止判定；四个具名球体和无名 thigh/body 回退。真实 sparse MuJoCo model probe 观察到 4 个足—地面 contact sample，mj_contactForce 经转换得到 RR 世界力 (-36.4923, 6.08441, 99.5727) N；地面 contact 不算障碍，也未触发终止。probe 对实际 contact 几何身份作定向替换来测试 producer 分类分支，因此该部分是路径核验，不冒充实际足端撞障碍运行。
- 跨语言检查由生产 C++ ObstacleCollisionAuthority 快照经过正常 Python reader、collision_snapshot_payload、strict payload validator 和 motion_safety。读到 v5/14,472-byte LIVE；足端障碍接触 current_collision=false，非足端障碍接触 current_collision=true，body fallback 正确。1-byte 截断、版本/长度错配拒绝；v3/3,864 与 v4/5,760 结构解码通过定向布局 fixture；留存 S1-05 v4 JSON 帧也由记录验证器接受。实际 v3 历史二进制未找到，因此报告不把 fixture 写成历史 run 验证。详细结果：[C++ 探针日志](offline_probe/cpp_probe.log)、[Python 跨语言结果](offline_probe/python_validation.json)。

## 唯一一次稀疏诊断

桌面预检使用既有 DISPLAY=:0 与 /run/user/1000/gdm/Xauthority。默认沙箱连接 X socket 返回 EPERM；经用户既有授权的沙箱外 X11 查询和系统 GLFW 隐藏窗口探针均 PASS，renderer 为 Intel / Mesa Intel(R) Graphics (RPL-P)。未更改桌面权限、授权、驱动、系统服务或全局环境。预检记录：[display_preflight.json](display_preflight.json)。

真实运行配置由 controller 启动日志直接证明：scene_ppt_sparse.xml / ppt_sparse、paper_faithful_switch、group A、entry/effective exit -0.05、hysteresis=false、hold=false、目标 (7,0)；运动上限 10 MuJoCo 仿真秒、墙钟保护 120 秒，实际由碰撞提前结束。实际 config 为 /home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/go2_description/share/go2_description/config/abs/config.yaml；Agile、RA、Recovery 模型分别为 /home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/go2_description/share/go2_description/config/abs/policy.pt、/home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/go2_description/share/go2_description/config/abs/ra_value.pt、/home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/go2_description/share/go2_description/config/rec/policy.pt。controller PID 及 policy-frame writer PID 均为 147462，策略 session 26545441952957。MuJoCo authority 日志绑定 ppt_sparse、4 个障碍及本场景 runtime model fingerprint。具体来源保存在 run 的 ros_launch.log、mujoco.log 和 run_context.json。

运行 ID 为 bf84dbe4946b43b29a30200dc24d6ec5。运动终止周期为 session 26545441952957 / cycle 184，persisted policy sequence 370；目标距离 2.1471 m，因此没有到达。记录 149 policy frames、1 个无效 sim-clock frame、0 周期缺口；测得实际 policy frequency 49.9956 Hz，有 2 次模式转换。记录 VALID、运动覆盖完整。

运动终局和事件分开列示：

| 项目 | 本次值 |
|---|---|
| navigation_terminal_result | COLLISION_TERMINATION |
| nonfoot_collision_failure | true |
| foot_contact_count | 1 |
| foot_impact_count | 1 |
| arrived_with_foot_contact / impact | false / false |
| abs_reference_collision | true |

非足端终止事件为无名 geom 46 / body 12 RL_calf 与 obstacle geom 3 ppt_obstacle_03 接触，physics step 14816–14819。足端事件为具名 geom 48 RL 与同一障碍接触，physics step 14815–14819；模型 MJCF 对应 owning body 是 RL_calf。其触发值为 Fxy=216.8427 N、abs(Fz)=35.0042 N、T=80.0084 N，严格大于条件成立。足端接触和足端撞击都写入记录，但实际终止原因仍是非足端碰撞。两个事件同时发生，所以本次实际运行不能单独估计“只有足端撞击”时的终止行为；这个分支另由生产路径定向探针、阈值检查核对。

两个足端 episode 都在终止观察处右删失，物理 snapshot 时间覆盖到 29.638 s，而终止策略周期时间是 29.636 s。完整采集累计足端接触和撞击各覆盖 5 physics steps / 约 0.010 s；因事件跨过运动窗口边界，运动窗口 duration 明确为 UNKNOWN，不把越界部分计入。可复核输出：[metrics.json](diagnostic_runs_20261010/bf84dbe4946b43b29a30200dc24d6ec5/metrics.json)、[结果表](diagnostic_runs_20261010/analysis/run_results.md)、[时间线](diagnostic_runs_20261010/bf84dbe4946b43b29a30200dc24d6ec5/timeline.png)、[terminal.json](diagnostic_runs_20261010/bf84dbe4946b43b29a30200dc24d6ec5/terminal.json)。

终局后 PASSIVE command=1 请求及 controller HARD-STOP-CONFIRMED state=PASSIVE 均有证据；process exit code 0、shutdown_complete=true、closeout_complete=true、closeout errors 为空。之后核对本任务相关的 MuJoCo、controller manager、RViz、stop publisher 和 probe 进程列表为空；本任务自己的临时 probe shared-memory 名称已清理。PASSIVE/controller 确认不等于 MuJoCo 已消费命令。

## 保留限制

- 唯一策略 run 实际遇到足端撞击和非足端碰撞，没有 ARRIVED，也没有独立的轻微足端障碍接触实例；离线和 controlled producer probe 覆盖了足端接触不终止及严格公式边界，不扩展为稳定性/成功率结论。
- 足端 motion-window 接触/撞击 duration 因 event/snapshot 与终止策略时间相差 2 ms 且 episode 未闭合而 UNKNOWN；全采集 observation 为约 0.010 s/5 physics steps，不能冒充运动窗口时长。
- 无效接触力、unknown geom、physics gap、事件 history overflow 会令相关否定结论 UNKNOWN；容量内历史无溢出。真实 v3 二进制历史 run 未找到。
- 相对 Isaac Gym 的力值等价、其他场景/姿态下 force coverage、正式性能和统计仍未验证；控制器关节顺序来源限制保持既有 CURRENT_STATE 记录。命令被 MuJoCo 消费、实机足力接口语义、长期稳定性仍 UNKNOWN。
- 分析器初次使用系统 Python 时因 Matplotlib/NumPy ABI 不兼容失败。未重跑仿真；按使用说明使用临时 PYTHONNOUSERSITE/Matplotlib cache 环境重新分析，成功生成 metrics、结果表和图像。脚本执行后相关分析进程正常退出。

## Director 审阅结论（2026-10-10）

有条件通过。已接受的能力是三类接触事件的生产分类、v5 记录与分析链、历史版本读取边界，以及真实运行中足端撞击被记录但终局由非足端碰撞触发的证据。单独轻微足端接触、成功率、其他场景表现、长期稳定性和 MuJoCo 命令消费未由本任务证明，继续作为后续限制保留。
