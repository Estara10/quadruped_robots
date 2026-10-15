# 静态凳子开发样例

- 场景：`scene_stool_dev_01.xml`；内部ID：`stool_dev_01`。
- 独立开发样例，不计入60张地图，不进入留出集。
- 两张固定凳子，中心位于(2.3,0)、(4.8,-0.65)米，第二张绕竖轴旋转18°。
- 每张座面0.60×0.60米、厚0.06米、总高0.52米；四腿截面0.07×0.07米、高0.46米。座面下缘与腿上端均为0.46米。
- 10个具名世界静态碰撞box，没有自由关节，不会被撞动。碰撞记录可区分座面和各腿；现有障碍计数10表示构件数，不是10张凳子。
- 射线沿用当前2D投影：座面和腿都投影到XY平面，不是带高度的深度感知。座面投影保守覆盖腿间空隙，机器人应绕开整个凳子；真实碰撞没有额外填满下方空间。此观测与物理形状差异是明确限制，不用于钻凳实验或家具感知真实性结论。
- 起点(0,0)、目标(7,0)，参考路线从上侧绕行。按0.65米保守包络加0.10米余量检查，实际轮廓连续线段最小净距0.80米。路线只用于检查，不输入策略，未证明策略可达。

[俯视图](top_view.png) · [斜视几何示意图](stools_oblique.png) · [布局](layout.json)

生成：`rtk proxy /usr/bin/python3 scripts/create_stool_dev_scene.py`。首次登记指纹必须使用当前运行端C++生产算法计算；生成器不会用Python版MuJoCo的编译模型指纹覆盖已登记运行端值。

手动A组运行（本次制作未执行）：

```bash
rtk proxy /usr/bin/python3 -u scripts/manual_ab_compare.py \
  --groups A --repeats 1 --scene scene_stool_dev_01.xml \
  --entry-threshold -0.05 \
  --plot-python /home/lidio/anaconda3/bin/python3 \
  --display :0 --xauthority /run/user/1000/gdm/Xauthority \
  --classification "manual A fixed stool development sample; not a formal experiment"
```
