# S6-10 执行报告：交替绕行开发地图宽松变体与停滞核验

**最新状态：COMPLETE — Director 有条件通过。** 通过范围包括澄清原 C 组准备失败、创建宽松开发变体、修复运行端指纹元数据、完成生产 publisher→reader LIVE 握手，以及变体上 A、C 各一次有效到达并完成站立观察。原始 A 失败中的 MuJoCo 物理快照推进到 step 3578、sim_time 7.156 s，但因模型指纹不匹配而为 INVALID，控制器/RL 被正确阻止；原始记录保留。67个场景只更新运行端指纹元数据，XML、几何、布局和开发/留出划分未变。A 的安全结论仍 UNKNOWN，C 保持审计不完整；各一次运行不用于性能比较。

## 原图 A 与近期 B/C/D 记录

原图为 `scene_random_s6_09_dense_alternating_bypass_dev_01.xml`，内部 ID `s609_da_d01`。

较早的原图 A 运行目录为 [`20261013_183325_350946`](../../../../logs/manual_abcd_compare/20261013_183325_350946/)。原始分析为 `VALID / COLLISION_TERMINATION`；在后退过程中 `RL_thigh` 与 `ppt_obstacle_01` 接触，仿真时间约11.218–11.236 s。该单次记录不是性能结论。

近期 B/C/D 批次为 [`20261013_185042_ea1e78`](../../../../logs/manual_abcd_compare/20261013_185042_ea1e78/)：

| 组别 | 配置和来源 | 结果 | 对“卡死”的证据判断 |
|---|---|---|---|
| B | `group=B`、E=-0.05、X=-0.08；独立 overlay；scene ID `s609_da_d01` | `VALID / COLLISION_TERMINATION`，在RL后碰撞终止 | 已进入策略；MuJoCo持续推进。分析记录为 `RL_calf` 接触 `ppt_obstacle_01`。 |
| C | `group=C`、E=-0.05、H=0.30仿真秒；独立 overlay；来源日志确认配置 | `PREPARATION_FAILURE / INVALID`，0条策略帧、没有RL session | 仿真进程并未停摆：MuJoCo物理步推进至12533，仿真时间推进至24.474 s。20 s站立准备墙钟上限到达时，连续站立准入仍未确认。它不是C策略运行后机器人停滞；策略根本没有启动。 |
| D | 计划为E=-0.05、X=-0.08、H=0.30；同批次 | `NOT_STARTED` | C准备失败后按既有批次停止规则未运行。 |

C的准备观察记录显示：早期高度约0.206 m，fall candidate/confirmed成立，并有一个从0.25 s开始、0.55 s确认的episode；该episode在4.576 s关闭。最后一次准备观察时sim_time约24.474 s、高度约0.325 m、pitch约0.0154 rad、roll接近0、四足接触，当前candidate/confirmed为假。controller日志为 `FIXEDSTAND`，`STAND-READY` gate曾通过，但准备结果仍是 `not_stable`，原因字段为 `invalid physics time, incomplete coverage, or unknown contact`。这支持“物理仿真推进、控制器仍在起立准备、准入条件未被确认”。聚合原因无法区分具体是哪项时间/覆盖/authority条件阻断站立准入，仍为 UNKNOWN。原始 C 记录未更改。

因此，已有 C 数据不能验证 H=0.30，也不能证明策略或仿真卡死。它证明的是准备阶段在20 s墙钟上限内未获得站稳确认并以 `PREPARATION_FAILURE` 收尾。

## 新增宽松开发变体

- 来源是上述 S6-09 dense alternating-bypass 开发图。新内部 ID：`s610_dar_d01`；场景 XML：[scene_random_s6_10_dense_alternating_relaxed_dev_01.xml](../../../../unitree_mujoco/unitree_robots/go2/scene_random_s6_10_dense_alternating_relaxed_dev_01.xml)。专用构建脚本：[create_s6_10_relaxed_alternating_variant.py](../../../../scripts/create_s6_10_relaxed_alternating_variant.py)，布局：[variant_layout.json](../../designs/S6-10/variant_layout.json)，[俯视预览](../../designs/S6-10/s6_10_relaxed_alternating_dev01.png)。
- 原图、原60张库、留出集和旧运行记录均未改动。新变体只单独加入共享场景目录，不计入原60张库、留出或正式统计。
- 原图几何要点与变体：

| 项目 | 原图 | 变体 |
|---|---:|---:|
| 长墙沿行进方向全厚度 | 0.50 m | 0.35 m（减少30%） |
| 三个开口宽度 | 1.95 m | 2.35 m |
| 闸口中心 x | 1.30、3.50、5.70 m | 1.20、3.50、5.80 m |
| 相邻闸口墙面之间的纵向净距 | 1.70 m | 1.95 m（各增加0.25 m） |
| 障碍最近对净距 | 0.1678 m | 0.2428 m（obstacle_04 / obstacle_07） |
| 参考路线最小障碍净距 | 0.7564 m | 0.9187 m |
| 参考路线长度 | 13.427 m | 12.548 m |
| 障碍投影占地比例 | 13.77% | 9.10% |

占地比例落入 S6-09 中密度档，因此该图明确标记为“由密集图派生的宽松变体”，不代表密集档样本。起点、目标、平地、障碍高度、Go2模型、物理参数、控制规则和安全门保持不变。起终点最近障碍边缘的 x 向净空由约1.050 m变为1.025 m，仍满足0.95 m下限。

三处开口中心仍为左(+1.2 m)、右(-1.2 m)、左(+1.2 m)，起终点直线仍被 `ppt_obstacle_01`、`ppt_obstacle_04`、`ppt_obstacle_05` 阻挡。几何检查使用实际旋转箱体/圆柱轮廓：所有障碍在边界内、端点净空通过、无障碍重叠且最小障碍间距至少0.10 m；0.05 m网格连通检查通过，参考路线连续轮廓净距为0.9187 m。既有0.65 m机器人包络加0.10 m工程余量（要求净距0.75 m）没有缩小。参考路线只用于几何检查和展示。

## 场景目录、构建与显示预检

- 新ID `s610_dar_d01` 为唯一短ASCII身份，适配32字节场景身份字段。生产 `abs_scene` 目录读取77个条目；原图仍解析为 `s609_da_d01`，变体解析为 `s610_dar_d01`、7个障碍，XML root/closure身份匹配目录。
- 变体XML由 MuJoCo Python 3.2.3 生产加载/模型指纹路径加载。ray writer和碰撞 authority 仍由共享 catalog/`selectObstacleGeoms()` 选择同一登记障碍集合。`unitree_mujoco/simulate/build2/unitree_mujoco` 主程序针对新catalog成功构建。运行期ray/碰撞快照是否LIVE尚未在本变体上观测。
- 沙箱外最小 GLFW 预检用 `DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority`，隐藏16×16窗口通过；renderer为 Intel / `Mesa Intel(R) Graphics (RPL-P)`，OpenGL 4.6 Compatibility / Mesa 23.2.1。探针窗口关闭，临时探针文件清理。

## 变体 A 首次诊断（上一轮）及当时停止点

上一轮按既定运行入口尝试A一次：A、E=-0.05、目标(7,0)、0.5 m到达半径、20 s仿真上限、120 s墙钟上限、到达后5 s站立观察；模型、启动流程和安全规则沿用。

- run `200dc6ff5b6c476b82d22b6331cf2793`，原始记录位于[`A运行目录`](../../../../logs/manual_abcd_compare/20261013_190515_e5723b/A/rep_01/200dc6ff5b6c476b82d22b6331cf2793/)。A overlay生产参数预检通过，E=-0.05，目标(7,0)，overlay为独立文件树。
- 终局为 `STARTUP_FAILURE`，record validity 为 INVALID。`mujoco.log` 记录 `ERROR: could not initialize GLFW`；MuJoCo退出码1。authority没有进入LIVE，策略帧为0；ROS launch、controller和ROS查询/发布会话均未启动。
- 独立沙箱外GLFW预检通过，但这次实际仿真启动仍失败。日志未提供更细 errno；实际运行执行上下文的桌面访问限制是可能原因，精确拒绝原因仍UNKNOWN。
- 按“基础设施失败后停止后续运行”规则，C未启动；没有第二次尝试，也没有用无效启动评价地图或A/C行为。

## 结论和清理（截至首次 A 尝试时；后续补证见下）

当前证据足以判断原C“卡死”说法不准确：那次仿真仍推进，但卡在进入RL之前的站立准备失败，因而没有测试C保持逻辑。变体几何、XML、生产目录身份解析和MuJoCo主程序构建已核验；变体上的真实A/C运行链、authority发布/reader LIVE和保持审计还没有证据。新变体可能更宽松，但本次没有策略运行可以验证这一点。

上一轮首次A收尾记录确认MuJoCo退出（exit code 1，未强制）；ROS launch和持久查询会话未启动；`shutdown_complete`、`closeout_complete`为真。该次运行后未发现相关项目进程残留。它与本轮的预检及第二次A尝试分开记录，原始数据均保留。

## 定向补证：桌面访问及第二次 A 尝试（2026-10-13；历史故障结论已由下方修正）

### GLFW/OpenGL 与执行环境

- 上次失败 run 的 `mujoco.log` 原文是 `ERROR: could not initialize GLFW`，配置记录 `DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority`。它只证明当时 GLFW 初始化失败，没有直接 errno。
- 本轮当前 shell 同样使用 `DISPLAY=:0` 和上述 XAUTHORITY；授权文件对当前用户可读，X socket 存在。默认沙箱中的系统 GLFW 隐藏窗口探针返回 `glfwInit failed`。`unitree_mujoco` 实际解析到系统 `/lib/x86_64-linux-gnu/libglfw.so.3`、`libX11.so.6` 和 `libstdc++.so.6`；Mesa DRI 驱动文件位于 `/usr/lib/x86_64-linux-gnu/dri/`，但默认沙箱内 `/dev/dri` 不可见。沙箱外同一探针成功，renderer 为 Intel / `Mesa Intel(R) Graphics (RPL-P)`，OpenGL 4.6 Compatibility / Mesa 23.2.1，与既有预检一致。沙箱限制是默认探针失败与沙箱外成功的证据所支持的原因；上一次 MuJoCo 失败的具体系统调用原因仍 UNKNOWN。
- 按已有授权临时申请的沙箱外执行获准并完成了探针及本次单次场景尝试。只使用现有 DISPLAY、XAUTHORITY 和系统库/驱动；没有更改 X 授权、权限、驱动、安装或全局环境。探针已关闭，临时源文件和可执行文件已删除。

### 唯一补充 A 运行

批次为 [`20261013_191201_c75f10`](../../../../logs/manual_abcd_compare/20261013_191201_c75f10/)，run ID `d1f349bb02764dae98bfc310232b88c3`，原始记录位于 [`A run`](../../../../logs/manual_abcd_compare/20261013_191201_c75f10/A/rep_01/d1f349bb02764dae98bfc310232b88c3/)。实际参数仍为 A、E=-0.05、目标(7,0)、到达半径0.5 m、运动仿真上限20 s、墙钟上限120 s、到达后站立观察5 s；诊断器和记录器在 MuJoCo 启动前就绪，使用独立 overlay。分类为非正式 S6-10 补证。

- 运行日志包含 MuJoCo 3.3.3、关节/传感器清单和 `[Ray2D] Source: geometric scene_id=s610_dar_d01`，说明进程已越过此前的 GLFW 初始化失败并加载目标场景。没有新的 A 策略帧或 controller 初始化。
- supervisor 的等待错误为 `MuJoCo current-run physics authority did not become LIVE and advance before timeout; no motion started`，`mujoco_authority_live_monotonic_ns` 为空、策略 `frames_written=0`，终局为 `STARTUP_FAILURE`。Director 对原记录的复核及 `run_context` 最后一次启动观察证明物理确实持续推进到 step 3578、sim_time 7.155999999999434 s；状态是 INVALID，而不是缺少物理数据。控制器/RL 未启动，因此没有策略周期、站立准入或机器人运动证据。报告旧版“物理时间推进未获证实”结论已撤回。
- supervisor 在有界准备失败后收尾；`run_context` 的 `process_groups_stopped` 记录 MuJoCo exit code 0、未强制，ROS launch 和 ROS 查询会话均未启动；`shutdown_complete` 与 `closeout_complete` 为 true。诊断入口汇总中的进程退出码字段为 null，保留该字段差异，不将它改写为信号层退出码。
- 基础设施失败后未启动 C；没有第二次 A，也没有改地图或重跑。该 A 记录保留为 INVALID。

### 场景适用性结论与最终清理（截至第二次 A 尝试；后续补证更新了结论）

宽松变体的几何检查只说明参考通行包络存在连续通路。旧图 A/B 的单次碰撞与旧图 C 的预备失败都不能代表新变体表现；在身份修复前的新 A 未进入可信物理 LIVE，也未进入 RL。修复后的 A/C 各一次运行提供了变体上的实际策略运动证据，但不足以证明它能减少卡滞/碰撞或适合作为正式切换机制研究场景。按任务边界，不继续迭代该布局。

补证结束后核对本次 MuJoCo、ROS/controller、RViz、采集器和辅助进程，无本次残留；用户其他会话未处理。原图、原60张地图、所有留出图及两次启动失败原始记录均未改写。**当时的提交前状态（历史）：**S6-10 保持 ACTIVE，等待 Director 审阅；其后修复补证及最新审阅结论见本报告末尾。

## 定向修复：指纹拒绝诊断与 A/C 补证（2026-10-13）

### 根因定位及运行端版本对照

- 原始失败 run `d1f349bb02764dae98bfc310232b88c3` 的物理观察最后值为 step 3578、sim_time 7.155999999999434 s、status `INVALID`；旧 run 没有持久化原始共享 snapshot bytes，因此不能声称从该历史文件直接重读出了每个原始字段值。
- 本任务以当时登记值和同一 C++ 生产发布器/生产 Python reader 做了有界复现。旧登记 fingerprint 为 `88021a444ce8a218810c017cc29ceafc3a2799d81df066fab909fec1cc9e3b82`；MuJoCo 3.3.3 C++ 发布器实际写出 `a94d4d5b11db0920557978a17275c611183bc93c03140979000233f1e9c4e783`。authority 的 `invalid_reason=5`（`kInvalidModelFingerprint`），Python reader 首个拒绝项为 `runtime_model_fingerprint`，实际值为该 C++ 指纹、期望值为旧登记指纹。安全校验没有放宽。
- 生成工具使用 MuJoCo Python 3.2.3，运行端主程序使用 MuJoCo 3.3.3。两者加载同一份宽松变体 XML，编译模型的 `nbody=18`、`ngeom=64`，几何类型、body/group、碰撞位等离散字段和具名对象一致；99个 geom 位置分量、132个四元数分量及99个尺寸分量有浮点差异（本次比较最大四元数分量差约3.5e-6），因此逐位浮点序列化后的 SHA-256 不同。现有证据支持“两个编译版本的派生模型数值不同导致本次精确指纹不匹配”；不足以把所有差异归为某一单独的 MuJoCo 内部原因。
- 以实际运行端 MuJoCo 3.3.3 和生产 `abs_collision_model::compute` 对目录所有 77 个场景重新核验：已有10个基础场景的 fingerprint 原本一致；6个 S6-08、60个 S6-09 和1个 S6-10 生成场景共67个登记 fingerprint 与运行端不同。仅更正这67个场景目录的 `model_fingerprint` 身份元数据。没有修改 XML、障碍几何、布局、种子、地图分层或留出集成员关系；原始失败日志保留。
- 增加的启动诊断在既有分类判断为 INVALID 时记录首个拒绝字段及 actual/expected；`_valid` 的接受条件、比较方式和安全门不变。
- 修改文件为 `scripts/abs_collision.py`（拒绝字段诊断）、`scripts/s1_run_diagnostic.py`（启动等待记录诊断值）、`common/abs_scene_catalog.def`（运行端指纹身份元数据）。

### 生产链核验、构建与运行

- C++ MuJoCo 主程序 `unitree_mujoco/simulate/build2/unitree_mujoco` 增量构建成功。C++ 生产 authority 对宽松变体发布3个物理步；正常 Python reader 接收 version 5、scene ID `s610_dar_d01`、新登记 fingerprint、`authoritative=1`、`invalid_reason=0`，并将 step 3 / sim_time 0.006 s 判为 LIVE。独立临时共享内存和 publisher 正常退出并删除。
- 显示预检在原 `DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority` 下通过；renderer 为 Intel / `Mesa Intel(R) Graphics (RPL-P)`，OpenGL 4.6 Compatibility / Mesa 23.2.1。未更改桌面授权、驱动或系统设置。
- 修复后批次为 [`20261013_192635_6b7696`](../../../../logs/manual_abcd_compare/20261013_192635_6b7696/)，同一场景 `scene_random_s6_10_dense_alternating_relaxed_dev_01.xml`、目标(7,0)、到达半径0.5 m、运动上限20仿真秒/120墙钟秒、到达后站立观察5仿真秒。A、C 共用本次构建、模型、场景、初态准备和安全链，各仅运行一次：

| 组别 | run ID / 记录有效性 | 终局与距离 | 策略与切换 | 安全/保持覆盖 | 收尾 |
|---|---|---|---|---|---|
| A，E=-0.05 | `74b284f0f1844df9b83cf9676ad899b4` / VALID | ARRIVED，0.4758 m | 484帧；约49.996 Hz；38条模式边沿，其中35条有有效仿真时间；19次 Agile→Recovery、19次 Recovery→Agile；Recovery 144周期、有效区间累计2.710 s | 16个策略时钟帧无效、3个策略周期缺口；运动安全覆盖不完整，碰撞/跌倒为 UNKNOWN；未观察到保持审计 | 5 s站立观察 COMPLETE；MuJoCo、ROS launch、同会话查询/命令节点均以exit 0退出 |
| C，E=-0.05、H=0.30仿真秒 | `8fc1b61f1ed9403d91b40d74ad826999` / VALID | ARRIVED，0.4987 m | 351帧；约50.294 Hz；8条模式边沿均有有效仿真时间；4次进入及4次退出 Recovery；Recovery 62周期、有效区间累计1.186 s | 10个策略时钟帧无效、3个策略周期缺口；声明运动窗口的碰撞/跌倒覆盖完整且均未记录事件；保持审计4个 episode 中3个完整、1个不完整、未发现违规 | 5 s站立观察 COMPLETE；MuJoCo、ROS launch、同会话查询/命令节点均以exit 0退出 |

两组都通过站立准入、RL配置来源核验和任务记录；策略命令“已发布/控制器转换”不等于 MuJoCo 已消费。A 的运动安全窗口缺少起点前物理快照，所以碰撞与跌倒否定结论仍为 UNKNOWN；C 结果只适用于其声明且覆盖完整的运动窗口。每组一次 ARRIVED 不证明地图整体可达、切换效果或性能优劣；C 保持审计不完整，不能把“未发现违规”扩大成所有周期均已验证。此次新运行给变体带来真实策略运动证据，但不足以断言其适合正式切换机制实验。

从这两次有限诊断看，变体上确实发生了真实 Agile/Recovery 切换并到达目标，初步支持它可用于后续功能诊断；但 A 的安全覆盖不完整、C 有一个保持 episode 不完整，且每组仅一次，因此其作为正式切换机制对照场景的适用性仍未确认。

### 清理和停止点

A、C 每次均按合同先关闭 MuJoCo，再关闭 controller/ROS；记录显示相关进程正常退出、没有强制终止，停止发布/查询会话已收到 EOF 并关闭。结束后按进程名核对未发现本次 MuJoCo、ROS、RViz、采集器或辅助进程残留；唯一临时 authority 共享内存也已移除。GLFW 探针和本轮 `/tmp` 临时 C++ 文件/可执行文件已清理。不再迭代布局、不运行参数比较。

## Director 审阅结论（2026-10-13）

**S6-10 COMPLETE（Director 有条件通过）。** 通过范围为：原 C 组准备失败的原因边界澄清；宽松开发变体；运行端指纹问题修复；生产 publisher→reader 握手；以及变体上 A、C 各一次有效 ARRIVED 和5秒站立观察。

限制继续有效：A 组碰撞/跌倒安全结论为 UNKNOWN；C 保持审计4个 episode 中3个完整、1个不完整，聚合审计仍为 incomplete；A/C 各一次且初态不同，不能支持性能或成功率比较。67个场景仅更新指纹元数据，XML、几何、布局及开发/留出划分均未改变。今后生成场景时，指纹应按实际运行端的计算方式生成，避免登记值不兼容；本轮不扩展实现。原复杂地图和历史失败记录均保留，本布局不再继续迭代。运行环境已按记录清理并核对无本次相关残留。
