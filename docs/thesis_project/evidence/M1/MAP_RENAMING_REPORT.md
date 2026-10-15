# M1 地图短名称与旧测试场景清理

2026-10-15用户授权统一命名`dev01–30`、`holdout01–30`并删除以前测试地图。

## 执行内容

60张XML和对应PNG已统一短名，清单和当前总览同步；保留内部M1 ID、XML原字节、种子、几何和开发/留出划分。用户面对的名称是dev/holdout，历史内部ID继续用于原90次记录关联。逐图新旧对应在`designs/M1/library_v1/name_mapping.json`。

从活动Go2场景目录精确删除183个旧`scene*.xml`（含改名前60张M1 XML、先前随机/PPT/家具和测试场景）。删除前压缩到`archive/maps_before_short_names_20261015.zip`，验证ZIP完整性，并逐文件比对备份字节。保留基础`scene.xml`、`scene_flat.xml`、`scene_obstacle.xml`、机器人模型和共享资源。活动注册表只保留两张登记基础图及60张当前M1图，历史注册表在ZIP中。旧设计文档与原始运行数据保留，不修改历史结论。

生成器今后使用相同短名，不恢复长文件名，并保留已登记的运行端指纹；手动对照脚本默认改为dev01.xml、目标7.5米，避免默认选择已删除场景。改名前的日志批次清单仍绑定原布局文件，不能强行跨名称版本恢复。

## 核验

- 60张XML改名前后逐字节相等，运行端模型指纹不变；仅文件路径导致closure登记更新。
- `verify_m1_library.py`通过60张的确定性几何、数量、30/30划分和生产Python身份解析。
- 实际MuJoCo主程序及生产身份探针构建通过。
- 60张C++生产publisher→Python reader握手结果见[记录](renamed_maps_handshake_60.json)。握手不是机器人策略运行。
- Python编译及Git差异空白检查通过。没有运行模型推理、ROS或机器人运动，没有提交Git。

## 使用和限制

[当前地图说明](../../designs/M1/library_v1/README.md)提供新命令、编号规则及备份位置。当前几何冻结不变，原M1安全、越界和时钟覆盖限制保留；本次改名不补足旧数据，也不产生新的性能证据。

本次只执行构建、离线加载/握手与图片重绘。探针退出和临时共享内存清理以握手JSON为准，未启动MuJoCo GUI、controller、ROS或RViz。
