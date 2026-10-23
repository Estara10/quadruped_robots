# M3：挑战测试地图集（30张）

## 手动观看

在仓库目录运行 `rtk proxy /usr/bin/python3 scripts/manual_map_view.py`，随后输入1–30选择M3地图，也可输入其他地图文件名；输入q退出。默认A组、可见视窗、不截图、不生成事后分析图。每次收尾确认后删除临时数据；收尾未确认时停止并保留临时资料。参数集中在脚本顶部，以后修改手动测试设置使用这一个入口。

版本：`s7-challenge-test/1.0`。后续测试入口为 `challenge01.xml`–`challenge30.xml`；原M1 holdout01–30及第一轮结果不改写。

[30张总览](overview_30.png)

## 存放位置

- 本目录与M1、M2并列，保存30张预览、总览、布局和索引。
- 可运行XML继续放在 `unitree_mujoco/unitree_robots/go2/challenge01.xml`–`challenge30.xml`，保持现有MuJoCo相对资源路径与手动命令有效。
- M3是对现有挑战集的目录命名；场景ID、地图几何及登记版本不变。

## 规则与完成情况

- 全部6张middle、9张difficult开发模板，每模板2个新变体；不使用easy模板。
- 障碍中心x扰动±0.22m、y扰动±0.18m，尺寸乘0.92–1.08，箱体朝向扰动±0.12rad；使用固定新种子。
- 保留短障碍与局部绕行；物体间距至少0.50m，参考路线净距至少0.75m，不缩小0.65m包络+0.10m余量。
- 每图直线至少有一个阻挡物；参考线不输入策略。起点(0,0)、目标(7.5,0)，边界x[-0.5,8]、y[-3,3]，非实体墙。
- 生产MuJoCo 3.3.3已静态加载30张并完成场景登记，主程序构建成功；未运行策略。

## 使用及限制

- XML目录：`unitree_mujoco/unitree_robots/go2/`。诊断器已接入新版清单及边界元数据。
- 本集合是第一轮结果之后制定的后续挑战测试集，不能替换第一轮实验结果。
- 30张属于15个相关模板家族；不能当作30个完全独立模板或全新分布的泛化验证。后续统计应考虑模板家族。
- 所有中等/困难模板均纳入，不按A/B/C/D胜负筛选。高风险是设计意图，实际难度、策略可达性与组间效果尚未测定。
- 原240项冻结矩阵仍对应旧holdout地图，不得直接resume为新版。新版矩阵尚未生成，本轮未启动新实验。

## 地图索引

| 预览 | 开发模板 | 标签 | 障碍数 | 路线净距(m) |
|---|---|---|---:|---:|
| [challenge01.xml](challenge01.png) | m01.xml | middle | 9 | 0.785 |
| [challenge02.xml](challenge02.png) | m01.xml | middle | 9 | 0.837 |
| [challenge03.xml](challenge03.png) | m02.xml | middle | 9 | 0.947 |
| [challenge04.xml](challenge04.png) | m02.xml | middle | 9 | 0.961 |
| [challenge05.xml](challenge05.png) | m03.xml | middle | 6 | 0.762 |
| [challenge06.xml](challenge06.png) | m03.xml | middle | 6 | 0.791 |
| [challenge07.xml](challenge07.png) | m04.xml | middle | 6 | 0.751 |
| [challenge08.xml](challenge08.png) | m04.xml | middle | 6 | 0.767 |
| [challenge09.xml](challenge09.png) | m05.xml | middle | 11 | 0.797 |
| [challenge10.xml](challenge10.png) | m05.xml | middle | 11 | 0.804 |
| [challenge11.xml](challenge11.png) | m06.xml | middle | 10 | 0.766 |
| [challenge12.xml](challenge12.png) | m06.xml | middle | 10 | 0.770 |
| [challenge13.xml](challenge13.png) | d01.xml | difficult | 10 | 0.804 |
| [challenge14.xml](challenge14.png) | d01.xml | difficult | 10 | 0.762 |
| [challenge15.xml](challenge15.png) | d02.xml | difficult | 10 | 0.816 |
| [challenge16.xml](challenge16.png) | d02.xml | difficult | 10 | 0.796 |
| [challenge17.xml](challenge17.png) | d03.xml | difficult | 9 | 0.833 |
| [challenge18.xml](challenge18.png) | d03.xml | difficult | 9 | 0.826 |
| [challenge19.xml](challenge19.png) | d04.xml | difficult | 4 | 0.844 |
| [challenge20.xml](challenge20.png) | d04.xml | difficult | 4 | 0.926 |
| [challenge21.xml](challenge21.png) | d05.xml | difficult | 4 | 0.768 |
| [challenge22.xml](challenge22.png) | d05.xml | difficult | 4 | 0.784 |
| [challenge23.xml](challenge23.png) | d06.xml | difficult | 4 | 0.828 |
| [challenge24.xml](challenge24.png) | d06.xml | difficult | 4 | 0.812 |
| [challenge25.xml](challenge25.png) | d07.xml | difficult | 7 | 0.805 |
| [challenge26.xml](challenge26.png) | d07.xml | difficult | 7 | 0.785 |
| [challenge27.xml](challenge27.png) | d08.xml | difficult | 7 | 0.788 |
| [challenge28.xml](challenge28.png) | d08.xml | difficult | 7 | 0.763 |
| [challenge29.xml](challenge29.png) | d09.xml | difficult | 10 | 0.765 |
| [challenge30.xml](challenge30.png) | d09.xml | difficult | 10 | 0.862 |
