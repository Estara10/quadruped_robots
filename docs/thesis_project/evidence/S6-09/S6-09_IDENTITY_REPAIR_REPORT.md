# S6-09 身份字段修复与运行补证

**最新状态：COMPLETE（Director 有条件通过，身份修复补证）。** 审阅通过范围为唯一短内部 ID、地图几何保持不变、MuJoCo 主程序构建，以及一次稀疏开发图 A 组有效运行。原地图库的有条件通过范围仍保留。本补证处理新地图实际启动时发现的内部场景身份过长问题；几何、模型、阈值、安全门与等待上限均未改变，只对指定稀疏开发图 A 组补跑一次。

## 原因与最小修复

失败运行 `a42f90e1a8f844969d84a9c255997946` 的 run context 和 MuJoCo 日志证明加载了正确的新 XML、scene root/closure 与模型指纹。旧内部 ID `random_s6_09_sparse_single_side_bypass_dev_01` 长于 `common/abs_collision_contract.h` 的 `kScenarioIdBytes=32`。`ObstacleCollisionAuthority::copyField()` 对超容量字符串不写入，而 authority 初始化没有将该长度列为绑定失败；因此 authority 的快照场景身份为空/无效，Python reader 无法与期望 binding 匹配，physics authority 没有进入 LIVE，运行以 `STARTUP_FAILURE` 结束，未开始运动。第二次历史启动失败也保留，路径见下方历史记录。

采用短内部 ID，不改共享二进制版本或字段布局：

| 字段 | 约定 |
|---|---|
| ID | `s609_<密度码><结构码>_<划分码><序号>`，ASCII 字节数必须小于 32，以留出 NUL 终止符 |
| 密度码 | `s/m/d` = sparse/medium/dense |
| 结构码 | `b/a/o/f/i` = single-side/alternating/offset-gates/fork/irregular |
| 划分码 | `d/h` = development/held-out；序号为 `01/02` |
| 示例 | `s609_sb_d01` = sparse、单侧绕行、开发图01 |

XML 文件名和 PNG 文件名仍使用原完整描述性 slug；布局数据、共享场景目录及运行身份改用短 ID。C++ authority 现在要求环境传入 ID 在 `kScenarioIdBytes` 内可完整 NUL 终止，超长 ID 会使绑定无效，不再静默发布空字段。未扩展协议，无需修改 Python reader。

## 定向核验

- 60/60 内部 ID 唯一、ASCII、最长 11 字节；与完整生产目录中的 ID 无冲突。描述性 XML/PNG 文件名仍各自唯一且保持不变。
- 迁移前后按文件名逐图比较了 60 张布局：障碍、路线、密度、结构和生成种子均完全相同。无地图几何变化。
- `scripts/generate_s6_09_scenes.py --generate` 和 `--check` 使用 `s6-09-mapgen/1.1` 完成；输出为 60 张地图、30/30 开发/留出、60/60 生产目录 binding。`scripts/test_s6_09_map_library.py` 通过 60 图几何/结构回归及 ID 长度/唯一性检查。记录见 `identity_layout_check.txt`、`identity_geometry_regression.txt`、`identity_generator_check.txt`。
- MuJoCo 主程序 `unitree_mujoco/simulate/build2/unitree_mujoco` 增量构建成功；未改 controller、模型或共享布局。构建记录见 `mujoco_identity_repair_build.txt`。
- 当前显示预检通过：沙箱外只读连接 `DISPLAY=:0`、既有 XAUTHORITY；系统 GLFW 3.3.6 隐藏窗口创建/销毁成功，OpenGL vendor Intel、renderer `Mesa Intel(R) Graphics (RPL-P)`、4.6 Compatibility / Mesa 23.2.1。未改系统或桌面设置，探针临时文件已清理。记录见 `display_glfw_preflight_identity_repair.json`。

## 唯一 A 组补跑

- 地图：`scene_random_s6_09_sparse_single_side_bypass_dev_01.xml`，内部 ID `s609_sb_d01`；保留失败前同一独立 A overlay。
- 条件：A、E=-0.05、目标 `(7,0)`、到达半径0.5 m、运动上限20仿真秒、墙钟120秒、到达后站立观察5仿真秒；沿用相同模型、启动增益交接和安全规则。
- run：`24304f75bb974ebeb85452cb854f6f00`，记录目录 [`日志与记录`](../../../../logs/05_manual_observations/scene_runs/s6_09_identity_repair_sparse_dev01_A/24304f75bb974ebeb85452cb854f6f00/)。
- 生产发布/读取：C++ 日志 `[ABS-SCENE-AUTHORITY]` 和 `[Ray2D]` 均打印 `scene_id=s609_sb_d01`，8 个障碍与 root/closure/model fingerprint 与所选 XML binding 一致。记录器得到373条策略 `frame`，373/373 `status=LIVE`；首条为同一 session 的 `rl_step=0, rl_entered=1, rl_active=1`，末条 `rl_step=378`。嵌入的 authority 快照身份同为 `s609_sb_d01`、`authoritative=true`、`status=LIVE`。生产 reader/analyzer 接受完整身份，run `record_validity=VALID`；这构成一次实际 MuJoCo publisher → 正常 reader 的端到端运行证据。
- 终局：`VALID / ARRIVED`，终止帧距目标0.49982 m。任务窗口从仿真10.038 s至17.598 s；记录373策略帧，整体策略频率约50.01 Hz，含6个策略周期缺口和6个无效策略时钟帧。模式边沿62次。运动区间碰撞/跌倒覆盖由分析器判定完整；该结论只限任务窗口。
- 安全边界：安全快照保留了一个在策略任务起点前已开始并关闭的低高度确认 episode（仿真0.25–8.368 s）；真实任务起点为10.038 s，故它不落入本次运动窗口。不得据此称全采集期无跌倒。
- 收尾：到达后完成5仿真秒站立观察；发送正常关闭命令6。run controller/MuJoCo/ROS 会话进程退出码0，未强制，closeout errors为空。脚本报告本次 ROS、MuJoCo 和查询/命令会话均退出；定向进程核对未发现 `unitree_mujoco`、`controller_manager` 或 `rviz2` 残留。隐藏 GLFW 探针临时文件已删除。

分析输出为 `analysis/run_results.md`、`analysis/run_results.json`、run 内 `metrics.json` 与 `timeline.png`。第一次尝试用 `/usr/bin/python3` 生成时间线时，系统 Matplotlib 与该环境 NumPy ABI 不兼容；未改环境，改用已有 Anaconda Python 3.10 / Matplotlib 3.10 完成同一生产分析器处理。策略原始记录与终局未改写。

## 历史失败与限制

以下两次 `STARTUP_FAILURE` 原始目录未改写，均因本次身份字段容量问题导致 physics authority 未进入 LIVE，未开始运动：

- `logs/manual_abcd_compare/20261013_180936_7fae9e/A/rep_01/a42f90e1a8f844969d84a9c255997946/`
- `logs/manual_abcd_compare/20261013_180345_9db542/A/rep_01/3f7d2a3f363a4f63b1b397abef2d5bdc/`

新补跑证明该稀疏开发图在当前构建下的实际身份发布、reader 接受、记录及到达后收尾链路工作；单次运行不能证明其他 59 张逐张实际策略启动，也不构成策略性能结论。保留策略缺周期/无效时钟、任务窗口边界、无物理围墙导致可能外绕及包络未步态验证等地图限制。后续开发集试跑应记录轨迹与越出声明区域情况，并事先确定越界终止规则；留出集不得用于参数选择或按表现删改。

身份修复补证获 Director 有条件通过，S6-09 标记 COMPLETE。周期缺口、无效策略时钟、任务起点前已关闭的低高度事件和单次运行限制保留；一次 ARRIVED 不证明性能。两次历史 `STARTUP_FAILURE` 原始记录未改写；留出集未用于策略测试。本轮未启动参数比较、批量开发集试跑、训练或正式实验，未提交 Git。
