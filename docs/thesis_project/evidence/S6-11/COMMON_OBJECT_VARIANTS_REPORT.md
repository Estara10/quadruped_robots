# S6-11 常见物体开发场景变体

**当前任务状态：SUPERSEDED — 未完成地图工作已转入 M1。** 本报告记录S6-11追加四张开发变体的交付结果；该追加当时只完成几何与生产身份核验，未运行机器人策略、策略模型推理或参数实验。四图不属于现有60张地图，不改变开发/留出清单，也不改动来源地图和历史运行记录。

## 变体总览

| 新内部 ID / 场景文件 | 来源开发图 | 常见物体替换 | 场景物体数 | 碰撞构件数 | 实测 XY 投影占地比例 | 包络检查最小净距 | 余量后剩余通道宽度 |
|---|---|---|---:|---:|---:|---:|---:|
| `s611_obj_sl_d01` / `scene_s6_11_sparse_local_objects_dev_01.xml` | `s611_sl_d01` | 四脚凳、小桌 | 15（2件常见物体+13件简单障碍） | 23 | 6.782% | 0.760 m | 0.021 m |
| `s611_obj_ml_d01` / `scene_s6_11_medium_local_objects_dev_01.xml` | `s611_ml_d01` | 四脚凳、办公桌、垃圾桶 | 19（3+16） | 27 | 9.596% | 0.759 m | 0.019 m |
| `s611_obj_sq_d01` / `scene_s6_11_sparse_offset_objects_dev_01.xml` | `s611_sq_d01` | 花盆、垃圾桶、路桩 | 13（3+10） | 13 | 5.329% | 0.802 m | 0.105 m |
| `s611_obj_mq_d01` / `scene_s6_11_medium_offset_objects_dev_01.xml` | `s611_mq_d01` | 小桌、花盆、路桩 | 20（3+17） | 24 | 9.169% | 0.775 m | 0.049 m |

后两图的密度名称继承自来源图；家具代理替换后，1 cm XY 投影估算分别为5.329%和9.169%，低于原稀疏6.0%及中等9.5%的下界。报告按实测占地比例说明，不把继承名称当作仍满足原分档，也未为凑比例压缩通道或增加障碍。

## 图片

- [四张俯视总览](../../designs/S6-11/common_object_variants/common_object_variants_overview.png)
- [稀疏局部绕行俯视图](../../designs/S6-11/common_object_variants/s611_obj_sl_d01_top.png) · [三维几何示意](../../designs/S6-11/common_object_variants/s611_obj_sl_d01_oblique_schematic.png)
- [中等局部绕行俯视图](../../designs/S6-11/common_object_variants/s611_obj_ml_d01_top.png) · [三维几何示意](../../designs/S6-11/common_object_variants/s611_obj_ml_d01_oblique_schematic.png)
- [稀疏前向错列俯视图](../../designs/S6-11/common_object_variants/s611_obj_sq_d01_top.png) · [三维几何示意](../../designs/S6-11/common_object_variants/s611_obj_sq_d01_oblique_schematic.png)
- [中等前向错列俯视图](../../designs/S6-11/common_object_variants/s611_obj_mq_d01_top.png) · [三维几何示意](../../designs/S6-11/common_object_variants/s611_obj_mq_d01_oblique_schematic.png)

俯视图为布局数据渲染，斜视图为 Pillow 绘制的静态几何示意图，均不是 MuJoCo 窗口截图。浅青带表示0.65 m Go2平面包络加0.10 m工程余量；橙色直线为起点至目标的几何参照，青色折线为检查路线，均不作为策略输入。场地范围沿用 x=[-0.5,7.5] m、y=[-3,3] m，起点(0,0) m、目标(7,0) m。

## 对象与替换尺寸

所有尺寸以米计。物体在原简单障碍中心就地替换；局部绕行两张参考路线只作最小横向调整，分别为+0.025 m及+0.075 m，以通过同一净空检查。前向错列两张路线未调整。

| 场景 | 来源障碍 → 常见物体 | 中心 (m) | 碰撞构件与几何 |
|---|---|---|---|
| sparse local | `ppt_obstacle_01` → 四脚凳 | (2.098462, 0) | 座面0.52×0.52×0.055；总高0.50；4腿0.06×0.06截面，共5个box |
| sparse local | `ppt_obstacle_02` → 小桌 | (4.998462, 0) | 桌面0.48×0.42×0.05；总高0.48；4腿0.045×0.045截面，共5个box |
| medium local | `ppt_obstacle_01` → 四脚凳 | (1.635112, 0) | 同上，5个box |
| medium local | `ppt_obstacle_02` → 办公桌 | (3.485112, 0) | 桌面0.58×0.50×0.06；总高0.72；4腿0.05×0.05截面，共5个box |
| medium local | `ppt_obstacle_03` → 垃圾桶 | (5.335112, 0) | 直径0.36、高0.48的圆柱代理，1个geom |
| sparse offset | `ppt_obstacle_01` → 花盆 | (1.635058, -0.68) | 直径0.40、高0.44的圆柱代理，1个geom |
| sparse offset | `ppt_obstacle_02` → 垃圾桶 | (3.480213, 0.76) | 直径0.36、高0.48的圆柱代理，1个geom |
| sparse offset | `ppt_obstacle_03` → 路桩 | (5.418476, -0.68) | 直径0.24、高0.65的圆柱代理，1个geom |
| medium offset | `ppt_obstacle_01` → 小桌 | (1.606829, -0.68) | 桌面0.48×0.42×0.05；总高0.48；4腿0.045×0.045截面，共5个box |
| medium offset | `ppt_obstacle_02` → 花盆 | (3.463399, 0.76) | 直径0.40、高0.44的圆柱代理，1个geom |
| medium offset | `ppt_obstacle_03` → 路桩 | (5.371083, -0.76) | 直径0.24、高0.65的圆柱代理，1个geom |

完整尺寸、生成版本、来源种子、逐构件名称/位置/高度、替换对应、路径点及检查值保存在[布局总表](../../designs/S6-11/common_object_variants/layouts_common_objects.json)和各图独立 `*_layout.json`。物体数与碰撞构件数分开计数；例如一张凳子是1件物体、5个geom，不是5张凳子。`ppt_obstacle_objNN_<part>`名包含对象和部位，均为独立唯一碰撞名。桌面/座面和腿相接属于同一物体内部设计；不同物体投影之间未发现非预期重叠。

桌椅是固定在世界坐标的box，无活动关节；垃圾桶、花盆和路桩用竖直圆柱代理，省略桶沿、盆的锥度、路桩底座及装饰。所有构件保留独立碰撞几何，桌面/座面下方没有额外填充体。

## 射线、碰撞与几何核验

- 未修改射线算法、策略输入、模型、物理参数或安全/切换规则。生产射线继续将全部被选碰撞geom的XY投影作为二维障碍；桌面/座面投影覆盖其下方区域，策略不获得钻桌/钻凳的高度信息。该二维代理与实机深度相机或射线预测模型并不等价，不是实机感知验证。
- 物理碰撞仍按座面、腿或圆柱代理的具名geom分别产生；没有为了匹配二维射线而填满桌椅底部。真实接触行为尚未通过策略运行观察。
- 代码身份链定向核对：`scripts/s1_run_diagnostic.py` 用 `--scene` 调用 `abs_scene.resolve_scene()`，并将该目录的 scene binding 写入运行上下文、用于碰撞reader期望绑定；`scripts/s1_analyze_run.py` 比较记录中的 `scene_binding` 与运行设置的文件名、ID、障碍数和指纹。射线 bridge 与碰撞 authority 共用 C++ `selectObstacleGeoms`。本轮未启动策略，因此验证到场景解析、构建及publisher→reader authority身份，不声称产生了新策略运行记录或完成运行级分析闭环。
- 四图以真实旋转box/cylinder XY轮廓检查地图边界、起终点0.95 m净空、不同物体间重叠和参考路线连续净空。参考路线至少有0.75 m几何净距，即0.65 m包络加0.10 m余量；最窄处余量后剩余宽度见上表。路线只核验几何，不证明策略可达。
- 局部绕行两图的直线障碍仍由被替换后的实际物体投影构件阻挡；前向错列两图保留来源图原有中心线状态。障碍群之外仍可绕行；地图边界不是实体墙，未来运行需记录实际轨迹及越界情况。

## 身份、构建和 publisher→reader 握手

四个新增唯一ID分别为 `s611_obj_sl_d01`、`s611_obj_ml_d01`、`s611_obj_sq_d01`、`s611_obj_mq_d01`，ASCII均小于32字节。描述性XML文件名保留完整类型和密度。四项已追加到共享场景目录 `common/abs_scene_catalog.def`；正常 `abs_scene.resolve_scene()` 对文件名解析出的内部ID和障碍数与布局一致，因此可通过现有 `--scene` 场景选择入口选取。

模型指纹由运行端 C++ `abs_collision_model::compute` 在 MuJoCo 3.3.3 上计算，不使用 Python MuJoCo 版本替代。实际构建命令：

```bash
rtk proxy cmake --build unitree_mujoco/simulate/build2 \
  --target unitree_mujoco s6_11_scene_identity_probe -j2
```

四个 XML 均通过该实际C++模型加载。生产 `ObstacleCollisionAuthority` 与射线bridge共同调用 `abs_scene::selectObstacleGeoms`；`kNamedPptObstacles` 会选择所有 `ppt_obstacle_` 静态世界碰撞构件。探针检查地面/机器人未被选中，登记数量和C++实际选择数量一致。随后生产C++ publisher经生产 Python `scripts/abs_collision.py` reader进行了各一次有界握手，4/4为 `LIVE`：

| ID | 生产者登记/reader实际障碍数 | reader | MuJoCo | 探针退出/共享内存 |
|---|---:|---|---|---|
| `s611_obj_sl_d01` | 23 | LIVE | C++ 3.3.3 | exit 0 / 已删除 |
| `s611_obj_ml_d01` | 27 | LIVE | C++ 3.3.3 | exit 0 / 已删除 |
| `s611_obj_sq_d01` | 13 | LIVE | C++ 3.3.3 | exit 0 / 已删除 |
| `s611_obj_mq_d01` | 24 | LIVE | C++ 3.3.3 | exit 0 / 已删除 |

逐次原始握手结果见[JSON记录](common_object_variants_handshake.json)，其中含sequence、physics_step和运行指纹。XML具名障碍集合与布局清单逐项相等；四个catalog身份解析、指纹及障碍计数通过生产Python目录读取。

复现与核验命令：

```bash
rtk proxy /usr/bin/python3 scripts/create_s6_11_common_object_variants.py
rtk proxy /usr/bin/python3 scripts/test_s6_11_scene_handshake.py \
  --probe unitree_mujoco/simulate/build2/s6_11_scene_identity_probe \
  --layouts docs/thesis_project/designs/S6-11/common_object_variants/layouts_common_objects.json \
  --output docs/thesis_project/evidence/S6-11/common_object_variants_handshake.json
```

生成器二次运行后18个布局/XML/PNG输出的SHA-256逐项一致，确认确定性。机器人/策略运行命令未执行；待用户审图后再选具体变体。

## 修改与清理

新增生成器 `scripts/create_s6_11_common_object_variants.py`；新增四张Go2 XML、四份单图布局、布局总表、4张俯视图、4张斜视几何示意及总览图；`common/abs_scene_catalog.def` 追加4项；新增本报告和握手证据；S6-11任务与当前状态补入本次范围。原60张布局、地图和划分未改，来源地图的XML、ID、预览及运行证据未改。

实际运行的是短时 C++身份探针和 Python reader握手，不是仿真GUI或机器人运行。四个探针均exit 0并各自删除唯一临时共享内存对象；生成和图片渲染在当前进程内完成、进程正常结束。未启动MuJoCo GUI、controller、ROS、RViz、采集器或策略，也没有遗留本轮渲染/辅助进程。

## 限制与审阅点

- 这四张是独立开发变体，不能计入60张清单或留出集。原图保留。
- 两张offset变体的实际投影占地比例略低于原密度带下界；需要用户确认是否接受作为“来源稀疏/中等”的物体展示变体，或后续另行授权修订密度规则。
- 保守Go2包络/余量是已有几何工程约定，未做步态实测；贴边可通只表示所用几何模型下有路线。
- 斜视图是静态示意；圆柱均为简化代理。二维投影会挡住桌椅下方，仿真投影可见性不说明实机可见性。
- 未运行机器人，因此策略是否会绕行、是否碰撞、局部避障是否有效均 UNKNOWN；参考路线不是策略轨迹或导航输入。

**原停止点（历史）：**把四张总览和单图交给用户审查；当时S6-11等待用户决定是否手动运行开发图。随后未完成地图工作转入M1，本报告不表示S6-11仍为Active。
