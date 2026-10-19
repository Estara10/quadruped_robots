# M2：easy / middle / difficult 开发地图集

2026-10-18用户确认按难度重命名，当前版本m2-selected-development/2.1。
当前文件为 **e01–e03.xml、m01–m06.xml、d01–d09.xml**，共18张。XML位于`unitree_mujoco/unitree_robots/go2/`。

[18图总览](overview_18.png) · [逐图清单](MAP_INDEX.md) · [CSV索引](scene_index.csv) · [布局](layouts.json)

| 当前文件 | 之前M2名称 | 源地图 | 难度 |
|---|---|---|---|
| [e01.xml](e01.png) | m2dev17.xml | M1 dev01.xml | easy |
| [e02.xml](e02.png) | m2dev18.xml | M1 dev12.xml | easy |
| [e03.xml](e03.png) | m2dev19.xml | M1 dev22.xml | easy |
| [m01.xml](m01.png) | m2dev04.xml | M2 m2dev04.xml | middle |
| [m02.xml](m02.png) | m2dev06.xml | M2 m2dev06.xml | middle |
| [m03.xml](m03.png) | m2dev13.xml | M1 dev13.xml | middle |
| [m04.xml](m04.png) | m2dev14.xml | M1 dev17.xml | middle |
| [m05.xml](m05.png) | m2dev15.xml | M1 dev26.xml | middle |
| [m06.xml](m06.png) | m2dev16.xml | M1 dev29.xml | middle |
| [d01.xml](d01.png) | m2dev03.xml | M2 m2dev03.xml | difficult |
| [d02.xml](d02.png) | m2dev05.xml | M2 m2dev05.xml | difficult |
| [d03.xml](d03.png) | m2dev02.xml | M2 m2dev02.xml | difficult |
| [d04.xml](d04.png) | m2dev07.xml | M1 dev03.xml | difficult |
| [d05.xml](d05.png) | m2dev08.xml | M1 dev04.xml | difficult |
| [d06.xml](d06.png) | m2dev09.xml | M1 dev08.xml | difficult |
| [d07.xml](d07.png) | m2dev10.xml | M1 dev11.xml | difficult |
| [d08.xml](d08.png) | m2dev11.xml | M1 dev19.xml | difficult |
| [d09.xml](d09.png) | m2dev12.xml | M1 dev24.xml | difficult |

## 分类与使用边界

- 用户指定原M2 03、05、02为高难度，对应d01、d02、d03。其他六张difficult沿用已同意的M1高风险候选。
- m01为修改后的原M2 dev04 r5，暂归middle，风险/可达性仍待实测，不能套用旧几何成绩。
- easy 3张、middle 6张、difficult 9张是开发选图标签，不是经统计确认的真实风险或严格难度排序。
- 旧M1、原M2 XML及旧编号保留历史；当前实验仅使用这18个新名，不把别名与源地图当独立样本。没有使用留出集。
- 旧18图选择资料和旧名预览已归入[重命名前历史目录](history/before_easy_middle_difficult_20261018/)。旧实验/批次不改写。
- m01的01与08障碍间距约0.514m，低于原0.60m候选规则；参考通路净距约0.764m。精确用户修改保留，不以此保证策略可达。

## 生产接入

18张完成生产C++ MuJoCo 3.3.3静态加载、障碍集合及指纹登记，MuJoCo主程序构建完成。新内部ID为m2v2_e01等，避免与原m2d01历史身份冲突。
重命名仅改变文件/顶层模型名称、显示名与身份，障碍几何不动。生成器`scripts/prepare_m2_selected_maps.py`已使用新命名；未来重新生成或编辑需同步身份并构建。

手动查看示例（只说明入口，此次未运行）：

新运行默认到达后站立观察2仿真秒；可用`--standing-observation-sim-s`显式设置。已冻结的历史批次仍按其原计划时长记录，不修改旧数据。

```bash
rtk proxy /usr/bin/python3 -u scripts/manual_ab_compare.py \
  --groups B --repeats 1 --scene d01.xml \
  --entry-threshold -0.05 --exit-threshold -0.07 --goal-x 7.5 \
  --sim-limit-s 20 --wall-cap-s 120 \
  --plot-python /home/lidio/anaconda3/bin/python3 \
  --display :0 --xauthority /run/user/1000/gdm/Xauthority \
  --classification "manual M2 renamed map viewing; not a formal experiment"
```

本轮未启动物理步进、机器人策略、ROS或GUI，未运行216次参数扫描或测试套件，未提交Git。
