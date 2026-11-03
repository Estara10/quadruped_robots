# 当前冻结地图：M1_01–M1_60

> 当前说明（2026-10-23）：原留出30图已完成正式240项比较，结果见[报告](../../../evidence/S7-02/S7-02_EXECUTION_REPORT_20261023.md)。其后部分来源被选入M3挑战集，二者不作为独立新环境合并。地图冻结不变，原留出结果不用于再次选参。

几何版本：`m1-mapset/1.0`；名称修订：`dev-holdout/1.0`（2026-10-15）。

- 仿真XML：`unitree_mujoco/unitree_robots/go2/M1_01.xml`–`M1_30.xml`对应dev01–dev30；`M1_31.xml`–`M1_60.xml`对应holdout01–holdout30。
- M1预览图仍使用原dev/holdout文件名；[开发图总览](overview_development_30.png)、[留出图总览](overview_held_out_30.png)。
- [索引](scene_index.csv)记录名称、类别、密度、种子和目标；[名称对照](../../MAP_FILENAME_CROSSWALK.md)。
- 编号01–10稀疏、11–20中等、21–30密集；每档依次为单侧绕行、缓和S型、错位通道、局部绕行、前向错列，每类两张。
- 地图XML内容、内部M1身份、几何、种子和开发/留出划分未改变；内部旧ID为历史90次记录提供稳定关联，当前运行使用M1编号；旧文件名仍可作为输入。
- 文件路径改变导致资源closure身份重新计算，运行端模型指纹保持不变；不把改名描述成重新训练、重新实验或几何版本变化。

手动运行一张开发图（当前目标为7.5米）：

```bash
rtk proxy /usr/bin/python3 -u scripts/manual_ab_compare.py \
  --groups A --repeats 1 --scene M1_01.xml --goal-x 7.5 \
  --entry-threshold -0.05 \
  --plot-python /home/lidio/anaconda3/bin/python3 \
  --display :0 --xauthority /run/user/1000/gdm/Xauthority \
  --classification "manual renamed M1 development map; not a formal experiment"
```

将`M1_01.xml`换成其他M1编号即可；旧dev/holdout文件名仍兼容。留出图仍不得用于选参数。

旧测试场景已从活动Go2场景目录及场景注册表移除。保留`scene.xml`、`scene_flat.xml`、`scene_obstacle.xml`作为默认启动/基础诊断依赖；保留`go2.xml`、assets和地形纹理等共享资源。

删除前备份：`archive/maps_before_short_names_20261015.zip`，含旧测试XML、改名前完整场景注册表和M1布局/预览快照；[清理清单](cleanup_inventory.json)。历史日志和文档未改写；旧dev/holdout文件名仍可输入并解析到对应M1编号。

原90次批次计划绑定改名前布局文件SHA，不能用新布局强行恢复旧计划；不要修改历史批次的hash或名称来绕过校验。新批次应显式使用新的独立输出目录。
