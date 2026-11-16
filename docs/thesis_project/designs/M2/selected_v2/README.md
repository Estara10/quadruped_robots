# M2：easy / middle / difficult 开发地图集

2026-10-18用户确认按难度重命名，当前版本m2-selected-development/2.1。
当前文件为 **M2_20–M2_22.xml、M2_23–M2_28.xml、M2_29–M2_37.xml**，共18张。旧e/m/d文件名仍兼容。XML位于`unitree_mujoco/unitree_robots/go2/`。

[18图总览](overview_18.png) · [逐图清单](MAP_INDEX.md) · [CSV索引](scene_index.csv) · [布局](layouts.json) · [旧名对照](../../MAP_FILENAME_CROSSWALK.md)

| 当前文件 | 重命名前文件名 | 源地图 | 难度 |
|---|---|---|---|
| [M2_20.xml](e01.png) | e01.xml | M1 M1_01.xml | easy |
| [M2_21.xml](e02.png) | e02.xml | M1 M1_12.xml | easy |
| [M2_22.xml](e03.png) | e03.xml | M1 M1_22.xml | easy |
| [M2_23.xml](m01.png) | m01.xml | M2 M2_04.xml | middle |
| [M2_24.xml](m02.png) | m02.xml | M2 M2_06.xml | middle |
| [M2_25.xml](m03.png) | m03.xml | M1 M1_13.xml | middle |
| [M2_26.xml](m04.png) | m04.xml | M1 M1_17.xml | middle |
| [M2_27.xml](m05.png) | m05.xml | M1 M1_26.xml | middle |
| [M2_28.xml](m06.png) | m06.xml | M1 M1_29.xml | middle |
| [M2_29.xml](d01.png) | d01.xml | M2 M2_03.xml | difficult |
| [M2_30.xml](d02.png) | d02.xml | M2 M2_05.xml | difficult |
| [M2_31.xml](d03.png) | d03.xml | M2 M2_02.xml | difficult |
| [M2_32.xml](d04.png) | d04.xml | M1 M1_03.xml | difficult |
| [M2_33.xml](d05.png) | d05.xml | M1 M1_04.xml | difficult |
| [M2_34.xml](d06.png) | d06.xml | M1 M1_08.xml | difficult |
| [M2_35.xml](d07.png) | d07.xml | M1 M1_11.xml | difficult |
| [M2_36.xml](d08.png) | d08.xml | M1 M1_19.xml | difficult |
| [M2_37.xml](d09.png) | d09.xml | M1 M1_24.xml | difficult |

## 分类与使用边界

- 用户指定原M2 03、05、02为高难度，对应d01、d02、d03。其他六张difficult沿用已同意的M1高风险候选。
- m01为修改后的原M2 dev04 r5，归middle开发标签，已参与T1/T2运行；历史旧几何成绩仍不能套用到新几何。
- easy 3张、middle 6张、difficult 9张是开发选图标签，不是经统计确认的真实风险或严格难度排序。
- 旧M1、原M2 XML及旧编号保留历史；当前实验仅使用这18个新名，不把别名与源地图当独立样本。本18图开发集的原选择未使用留出集；后续原M1留出正式批次及M3完成情况见[当前状态](../../../CURRENT_STATE.md)。
- 旧18图选择资料和旧名预览已归入[重命名前历史目录](history/before_easy_middle_difficult_20261018/)。旧实验/批次不改写。
- m01的01与08障碍间距约0.514m，低于原0.60m候选规则；参考通路净距约0.764m。精确用户修改保留，不以此保证策略可达。

## 生产接入

18张完成生产C++ MuJoCo 3.3.3静态加载、障碍集合及指纹登记，MuJoCo主程序构建完成。新内部ID为m2v2_e01等，避免与原m2d01历史身份冲突。
重命名仅改变文件/顶层模型名称、显示名与身份，障碍几何不动。生成器`scripts/prepare_m2_selected_maps.py`已使用新命名；未来重新生成或编辑需同步身份并构建。

手动查看示例（只说明入口，此次未运行）：

新运行默认到达后站立观察2仿真秒；可用`--standing-observation-sim-s`显式设置。已冻结的历史批次仍按其原计划时长记录，不修改旧数据。

```bash
rtk proxy /usr/bin/python3 -u scripts/manual_ab_compare.py \
  --groups B --repeats 1 --scene M2_29.xml \
  --entry-threshold -0.05 --exit-threshold -0.07 --goal-x 7.5 \
  --sim-limit-s 20 --wall-cap-s 120 \
  --plot-python /home/lidio/anaconda3/bin/python3 \
  --display :0 --xauthority /run/user/1000/gdm/Xauthority \
  --classification "manual M2 renamed map viewing; not a formal experiment"
```

本轮未启动物理步进、机器人策略、ROS或GUI，未运行216次参数扫描或测试套件，未提交Git。
