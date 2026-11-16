# 脚本使用导航

当前常用脚本和公共模块仍放在原位置。2026-11-03已将29个历史脚本移到[归档目录](../archive/scripts/20261103_cleanup/README.md)，没有删除实验数据或模型。

## 实机与独立定位（2026-11-09）

当前先测相机安装变换并核对固定机身位姿／速度，为原生 A→B 和 ABS 共用。硬件消息可读、程序构建通过与实机精度分别判断。

| 用途 | 入口 | 当前范围 |
|---|---|---|
| Go2 状态观察／Estimator 观察 | [go2_readonly_state/README.md](go2_readonly_state/README.md) | 只订阅；动态 Estimator 速度未通过 |
| 终端原生运动／只读服务查询 | [go2_native_motion/README.md](go2_native_motion/README.md)／[现场命令](go2_native_motion/FIELD_COMMANDS.md) | Move／StopMove 与 LowCmd 分开；goto 仍用原生位置 |
| D435i／VINS 数据链 | [go2_vins/README.md](go2_vins/README.md)／[跨机说明](go2_vins/CYCLONE_FIELD_CHECK.md) | USB2 480×270 双 IR＋相机 IMU；固定版本与补丁另行构建 |
| 相机 IMU→Go2 base | [go2_vins/BODY_FRAME.md](go2_vins/BODY_FRAME.md)／[转换程序](go2_vins/go2_body_odometry.py) | 7 项离线检查通过；真实安装未测、在线未验证、不发运动命令 |
| D435i 深度→ABS 11束离线适配 | [realsense_depth_to_abs_rays.py](realsense_depth_to_abs_rays.py) | 单张 `.npy`；显式单位/scale/模型；不访问相机、DDS或共享内存；缺测像素拒绝预测 |
| D435i 60Hz深度→ABS射线只读观察 | [go2_depth_ray/README.md](go2_depth_ray/README.md)／[观察器](go2_depth_ray_observer.py) | 最新待处理帧槽；有限运行；记录频率、覆盖、时序、未知比例与11束诊断输出；不写协议或控制数据 |
| 实机实际进展 | [实机文档](../docs/thesis_project/real_go2/README.md) | 当前下一项与历史现场记录 |

原生 A→B 使用宇树运动服务；完整 ABS 则运行自己的策略与 LowCmd。现有 VINS／转换程序尚未接入两者；不因看到脚本就启动实机控制。A1 工程独立规划，当前库不覆盖其文件。

## 日常观看与演示

| 用途 | 入口 | 说明 |
|---|---|---|
| 三张地图录屏演示 | [manual_map_view.py](manual_map_view.py) | 当前默认e02、m02、m05；启动时选择A/B/C；每图回车开始 |
| ABCD单次手动观看 | [Lidio_test.py](Lidio_test.py) | 每次选择A/B/C/D和已登记地图；完整可视仿真；文件顶部开关决定是否保存本次记录 |
| 自选地图观看 | [manual_map_view.py](manual_map_view.py) | 显式加`--interactive`，按提示输入地图；默认不询问地图名 |
| 实时终端数据 | [abs_live_hud.py](abs_live_hud.py) | 只读监视模式、RA、射线、速度和数据时龄，不独立判断碰撞终局 |
| 手动组别比较 | [manual_ab_compare.py](manual_ab_compare.py) | 已有运行入口；启动前确认组别、参数及所用overlay |

在项目根目录运行：

```bash
# 三图演示，交互选择A/B/C
rtk proxy /usr/bin/python3 scripts/manual_map_view.py

# 单次手动观看：选择A/B/C/D，再输入已登记地图名
rtk proxy /usr/bin/python3 scripts/Lidio_test.py

# 直接指定B组
rtk proxy /usr/bin/python3 scripts/manual_map_view.py --demo --group B

# 自选地图观看
rtk proxy /usr/bin/python3 scripts/manual_map_view.py --interactive

# 预测射线演示：使用现存微调模型和CUDA，无几何回退
rtk proxy /usr/bin/python3 scripts/manual_map_view.py --demo --ray-source prediction

# 另开终端查看实时数据
rtk proxy /usr/bin/python3 scripts/abs_live_hud.py --interval 0.2
```

演示设置集中在`manual_map_view.py`顶部：`DEMO_MAPS`、`ENTRY_THRESHOLD`、`EXIT_THRESHOLD`、`HOLD_S`。A不使用退出阈值或保持时间，B只使用退出阈值，C只使用保持时间。演示入口显式选择Periodic；不代表所有历史批次入口或部署默认配置均已改成Periodic。可见窗口需要有效桌面会话。

`Lidio_test.py`适合逐图观看：默认E=−0.05、X=−0.07、H=0.02秒、Periodic P=0.06秒、几何射线、目标(7.5,0)m和20/120秒时限，到达后观察站立2秒。结果保存开关为文件顶部的`SAVE_RUN_RECORDS`，默认关闭；开启后每次运行及对应日志写入`logs/manual_visual_abcd/`。正常终局后可继续选择下一次运行；系统安全中止或启动异常会停止后续派发。此入口运行完整诊断流程，但属于手动观看，不作为正式实验批次。

## 当前运行与分析链

| 用途 | 文件或目录 |
|---|---|
| 实际仿真运行、终止与清理 | [s1_run_diagnostic.py](s1_run_diagnostic.py) |
| 逐帧记录 | [run_record.py](run_record.py) |
| 单次结果分析 | [s1_analyze_run.py](s1_analyze_run.py) |
| 场景和实时数据读取 | [abs_scene.py](abs_scene.py)、[abs_rt_frame.py](abs_rt_frame.py) |
| 物理事件与任务安全分类 | [abs_collision.py](abs_collision.py)、[abs_base_bounds.py](abs_base_bounds.py)、[safety_task_completion.py](safety_task_completion.py) |
| 预测射线 | [ray_predictor.py](ray_predictor.py)、[compare_ray_pred.py](compare_ray_pred.py) |
| 实机深度单帧离线转换 | [realsense_depth_to_abs_rays.py](realsense_depth_to_abs_rays.py) | 只读文件输入；旧 [realsense_ray_predictor.py](realsense_ray_predictor.py) 会启动相机并写裸共享内存，不用于当前协议 |
| 已完成12项射线批次入口 | [manual_r1_ray_pred_12.py](manual_r1_ray_pred_12.py)，保留复核用途，不作为日常演示入口 |
| 实机首次只读连接 | [go2_readonly_state/](go2_readonly_state/README.md)，不发送电机命令；首次通信已验证，接管待验证 |
| 实机状态曲线与前置相机 | [go2_live_view.py](go2_live_view.py)，只读窗口；运行说明见[观察器文档](go2_readonly_state/README.md#曲线与前置相机窗口) |

`manual_r1_*`和`analyze_r1_*`保留现有路径，用于R1历史记录与结果复核；它们不是可随意重复启动的普通演示脚本。训练、模型导出、相机和地图工具也保留，存在脚本不代表已验证或获得运行授权。

## 为什么还保留一些旧文件名

- R1的A/C运行器仍导入`manual_t1_m2_smoke_18.py`和`manual_t2_hold_function_check.py`。
- 当前记录器仍导入`p1_10_common_start.py`及相关公共代码。
- `s2_03_stop_publisher.py`和`s6_07_startup_protocol.py`仍参与现有运行和停止流程。
- 其余公共模块及当前测试保留原路径，避免为了目录整理修改控制和分析实现。

不要按T1、P1、S6或`test_`前缀批量删除文件。历史脚本的清单、迁移位置和恢复方式见[归档说明](../archive/scripts/20261103_cleanup/README.md)。
