# 当前D435i安装候选下的射线适配数据生成（离线准备）

## 结论

旧生成器有两个实质问题：相机一直硬编码为旧的102°视场和`[0,0,0.27]m`局部位置；它还调用`compute_geometric_ray_log()`时漏传了当前必需的场景文件名，因而按当前源码无法完整生成数据。本轮给生成器增加显式安装位置、俯角、原生深度分辨率和内参参数，并修好场景名传递。未给参数时仍走旧的160×90、102°、旧相机位置默认路径。

已用单面墙和偏左箱体完成一个合成几何检查：机身0°射线标签为2.00m，和墙前表面相对于既有射线起点的几何距离一致；+18°/+27°射线分别命中偏左箱体，图像中的箱体也在左半边。深度渲染中心采样为1.6498m，与相机候选点到墙面的预期光轴深度1.6490m相差约0.8mm。这个检查覆盖代码路径、单位、左右方向和标签参考定义；不验证实际相机外参、姿态，也不验证模型精度。

## 输入几何与采样

本次显式候选参数：

| 项目 | 值 | 依据与限制 |
|---|---:|---|
| base_link到相机壳体中心 | 前0.305、左0、上0.140m | 用户重装尺寸；人工误差约1cm。代码暂把壳体中心当作光学中心代理，真实壳体到光学中心结构偏移仍未知。 |
| 相机向下俯角 | 4° | 人工估计3–5°；本候选按相对base_link +x前向、水平面的角度解释。角度参考未独立实测。 |
| 站立base高度 | 0.36m | 用于检查场景；记录给出的站立范围为0.35–0.36m。 |
| 深度图与K | 480×270；fx=fy=244.134，ppx=245.186，ppy=134.22 | 由任务提供；尚需确认这些K对应最终训练/实机使用的深度图及校正状态。 |
| 模型输入 | 160×90 | 保持当前模型合同。 |

高度分辨率下，垂直fovy按`2 atan(H/(2fy))`计算约57.883°；水平视场按`2 atan(W/(2fx))`计算约89.022°，不是硬编码102°。MuJoCo `Renderer`只提供以相机内参建模的针孔渲染；新路径用`fx/fy`设定焦距，并在更新场景后按`ppx/ppy`设置离轴frustum，避免将主点偏差伪装成图像拉伸。

送入模型前复用`downsample_depth_centers()`：每个3×3像素足迹取中心像素，不插值、不缩放。对480×270输入，模型索引`(80,45)`取源图`(u=241,v=136)`。等效160×90内参为：`fx'=fy'=81.378`、`cx'=(ppx−1)/3=81.3953`、`cy'=(ppy−1)/3=44.4067`。主点不恰好落在模型图像几何中心，已保留其位置。

MuJoCo RGB/深度相机沿相机光轴观察；候选无横滚，按body坐标`+x前、+y左、+z上`放置，向下俯角通过相机视线方向设置。已知侧箱位于机身`+y`侧，投影到图像左侧；它使标签顺序中的正角度`+18°/+27°`束距离变短。这直接核对了图像左右与当前标签顺序，没有翻转11束。

标签仍完全由场景几何独立计算，使用ABS消费者合同：[消费者](../../../unitree_mujoco/simulate/src/unitree_sdk2_bridge.h)固定11束从−45°到+45°、每束9°，机身水平平面，射线起点base_link前向−0.05m/横向0m，距离截断0.1–6m；[策略输入契约](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/AbsObservationContract.cpp)按11项消费，保存训练标签为米制。相机视场约±44.5°，11束标签覆盖±45°，最外侧束略超出相机可视范围，不能从图像补造其观测；后续必须保留标签定义并评估此边缘盲区，或另行审阅是否需要训练/接口设计变更，本轮没有改角度。

## 代码改动

- [generate_mujoco_ray_dataset.py](../../../scripts/generate_mujoco_ray_dataset.py)：新增成组显式参数`--camera-position-m`、`--camera-pitch-down-deg`、`--depth-intrinsics WIDTH HEIGHT FX FY PPX PPY`。完整提供时按原生尺寸渲染、设置内参和离轴主点，再按已用中心采样器转成160×90。只给部分参数会报错。未传这些参数时仍采用旧默认。
- [compare_ray_pred.py](../../../scripts/compare_ray_pred.py)：加入可选相机渲染配置；缺省分支保留旧相机参数。该脚本的`compute_geometric_ray_log`需要`scene_filename`，生成器现传入实际场景名；标签射线定义和算法没有改。
- 输出manifest继续保留旧字段，并额外记录source分辨率、候选几何、内参、采样方式和主点处理。新候选将相机壳体中心作为光学中心代理，manifest明确记录结构偏移未知。

## 最小已知几何检查

使用MuJoCo 3.2.3现有`abs`环境及EGL无窗口渲染，在合成场景只生成一个姿态：base高度0.36m、零yaw；墙前表面x=1.95m、半宽3m，侧箱几何已知。模型图为90×160；标签中0°=2.000m，±9°与墙面相交约2.025m，+18°/+27°侧箱束约1.157/1.235m。侧箱近距离深度集中于图像左侧（模型像素质心x≈36.2，小于中心80）；中心采样深度1.6498m，对照光轴平面交点预期1.6490m。另以同一场景生成一帧不带新参数的legacy模式样本，manifest仍为160×90、102°和`[0,0,0.27]m`，确认旧默认几何保持。

这证明程序能从当前候选几何生成成像输入和独立机身射线标签，且单位、顺序及采样路径一致。它没有涉及策略或模型推理，也没有把侧箱渲染图当成现实标定。

![安装候选几何检查深度图](../../../logs/real_go2/20261113_mounted_camera_ray_data_preparation/geometry_check/geometry_projection_check.png)

- [合成场景](../../../logs/real_go2/20261113_mounted_camera_ray_data_preparation/known_wall_fixture.xml)
- [生成样本](../../../logs/real_go2/20261113_mounted_camera_ray_data_preparation/geometry_check/scene00_pose00000.npy)
- [独立标签](../../../logs/real_go2/20261113_mounted_camera_ray_data_preparation/geometry_check/label.pkl)
- [检查数值与假设](../../../logs/real_go2/20261113_mounted_camera_ray_data_preparation/geometry_check/geometry_check.json)
- [legacy默认检查manifest](../../../logs/real_go2/20261113_mounted_camera_ray_data_preparation/legacy_default_check/manifest.json)

## 可复用命令

以下命令复现这个单姿态合成检查（需要新输出名；不能覆盖已有检查目录）：

```bash
rtk proxy env MUJOCO_GL=egl /home/lidio/anaconda3/envs/abs/bin/python \
  scripts/generate_mujoco_ray_dataset.py \
  --scenes /home/lidio/quadruped_robots/logs/real_go2/20261113_mounted_camera_ray_data_preparation/known_wall_fixture.xml \
  --output-root logs/real_go2/20261113_mounted_camera_ray_data_preparation \
  --dataset-name geometry_check_reproduction \
  --x-values 0 --y-values 0 --yaw-deg-values 0 --base-z 0.36 \
  --min-target-clearance 0 \
  --camera-position-m 0.305 0 0.140 \
  --camera-pitch-down-deg 4 \
  --depth-intrinsics 480 270 244.134 244.134 245.186 134.22
```

后续有限候选生成时，将`--scenes`指向选定的Go2障碍XML，并显式给出安装参数；应按场景/位置/yaw块留出验证集，不能把相邻姿态随机拆到训练和验证两边。训练前保存最终安装测量、所选XML、命令及manifest。

## 训练前仍缺少什么

1. **实际光学外参：**当前位置来自壳体中心，壳体至深度光学中心的平移/转角未知；相机俯角的测量参考、最终base高度及roll/yaw也应在固定安装后确认。当前候选不能登记为生产外参。
2. **K对应的图像类型：**需确认fx/fy/ppx/ppy与将训练、推理的原始或rectified深度图完全对应，采集分辨率也应冻结。
3. **真实空洞处理：**本轮MuJoCo生成的是完整几何深度，没有真实D435i空洞或mode 1纹理。需先有最终安装条件下具代表性的有效性掩码，并决定训练与部署共用的补洞/残余未知策略；不能让训练把未知当作6m空旷，也未在本轮实现或验证空洞增强。
4. **相机视场边缘：**89.0°相机略窄于固定±45°标签。外侧标签信息不可观测。要先量化留出几何中受影响的束，再决定训练采样覆盖策略；不能改标签角度伪装对齐。
5. **真实独立评价：**趴伏柜前图和约196cm人工近似测量（红十字测点，2026-11-13确认，起点未标定）不是站立训练标签。后续应使用站立最终安装、已测障碍位置和明确射线起点生成少量独立逐束评价记录。

完成以上准备后，可用medium作候选ray-only起点、原权重保留、soft-safety留作对照；数据按场景/位姿区块分割，并评估每束MAE/RMSE/偏差、危险距离高估及过度低估。本轮没有训练，也没有证明当前模型可以直接复用或进入控制。

## 收尾

使用现有Conda `abs`环境（MuJoCo 3.2.3），未安装依赖。默认无显示后端首次渲染失败；切换现有EGL后端，单样本生成成功。ROS、DDS、相机、Go2、控制器、策略和真实共享内存均未访问。新增/修改仅限本生成器、几何比较脚本和本报告/检查产物；唯一ACTIVE任务R1和其历史实验、共享仿真配置、控制器、模型均未改。生成命令退出后，本轮Python/MuJoCo进程已退出；不留后台渲染进程。
