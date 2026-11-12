# 新 D435i 深度空洞处理候选离线评估

## 目的与边界

只用替换机已保存的9张原始深度帧，评估严格拒绝与“零值替换6 m”诊断基线。计划中的第三路为 librealsense 2.51.1 `hole_filling_filter` mode 1，再把残余零值替换6 m。电脑实际有 ROS Humble 包提供的 librealsense 2.58.4 运行库和SDK工具，但不是要求的2.51.1；`pyrealsense2` 不存在。按任务要求没有安装软件，也没有用2.58.4替代或用近似算法冒充，因此该滤波对照本轮未运行。

没有连接或操作相机、Orin、Go2；没有启动 ROS/DDS、VINS、控制器或仿真；没有训练模型、写共享内存或修改生产适配器、模型、配置及协议。全部模型输出仅为诊断值，不适用于控制。

## 数据身份与设置

| 数据组 | 相机序列号 | 文件与格式 | 分辨率／比例尺 | 已有采集记录 |
|---|---|---|---|---|
| cabinet_replacement | 238322070866 | 原始 `.u16`，每帧259200字节，与480×270×2相符；Z16 uint16 | 480×270，`frames.csv` 的 `scale_m=0.001` m/单位 | Orin USB2；librealsense 2.51.1；深度-only，请求60 Hz；投射器开、自动曝光开、laser power=150 |
| position1 | 238322070866 | 同上 | 同上 | 同一替换机；深度-only，请求60 Hz；投射器开、自动曝光开、laser power=150 |
| position2 | 238322070866 | 同上 | 同上 | 同一替换机；深度-only，请求60 Hz；投射器开、自动曝光开、laser power=150 |

序列号、驱动版本及名义设置由对应的 `comparison.json` 和实机报告记录；单帧文件本身不包含完整设备/profile元数据。逐帧曝光和增益未保存，60 Hz是请求值而非这三组记录的持续帧率测量。位置1、2没有独立目标距离。三组目录均为新替换机，未混入旧相机帧。

`.u16` 文件没有自描述的字节序标记；分析按 Orin 小端 `uint16` 读取。9帧计算得到的零值比例和中央40×40非零中位深度均与各自 `frames.csv` 相符，因此此次解释与记录统计一致。

## 处理与结果

现有严格入口先将 Z16 零值作为无效像素，再映射到160×90；每个输出格对应的源像素足迹只要有一个无效值，整帧就被拒绝。[`realsense_depth_to_abs_rays.py`](../../../scripts/realsense_depth_to_abs_rays.py#L50) 中的零值有效性判断在深度裁剪前完成，足迹判定见同文件第81–105行。

| 数据组 | 原始零值均值 | 160×90严格足迹未知均值 | 严格路径拒绝 |
|---|---:|---:|---:|
| cabinet_replacement（3帧） | 4.69% | 6.01% | 3/3 |
| position1（3帧） | 5.57% | 8.12% | 3/3 |
| position2（3帧） | 11.50% | 16.99% | 3/3 |

诊断基线把原始零值替换为6 m，再按当前160×90中心采样、0.1–6 m裁剪、`log2(m)`、三通道复制流程推理。[`direct6_per_ray.csv`](../../../logs/real_go2/20261112_d435i_hole_candidate_review/direct6_per_ray.csv) 保存9帧×11束的逐束距离；[`summary.json`](../../../logs/real_go2/20261112_d435i_hole_candidate_review/summary.json) 保存逐帧原始零值数、严格未知格数和模型输出。被替换像素仍在统计中标为原始未知；模型实际输入没有掩码，因此6 m占位在网络看来与真实6 m相同。

这些输出只说明在指定输入构造下当前权重能生成11个有限数值。它们没有独立距离真值，不能评价准确性、安全性，也不能据卷尺或结果“看起来合理”选择填充值。

### mode 1滤波候选状态

电脑环境检查发现 `ros-humble-librealsense2` 提供的共享库及工具版本为2.58.4；没有2.51.1运行库，Python也不能导入 `pyrealsense2`。因此9帧没有进行 SDK mode 1 滤波；滤波填入数量、滤波后残余零值、与直接6 m基线的逐束差值均为**未测**，不是0。历史旧相机同帧滤波结果不用于替代本次新相机对照。

## 对模型和ABS接口的含义

当前权重的模型输入为160×90深度、裁剪0.1–6 m、取 `log2` 并复制为3通道；接口没有有效性掩码。本机 D435i 水平视场约88.9°，现有模型入口配置102°（[`ray_predictor.py`](../../../scripts/ray_predictor.py#L13-L20)），角度覆盖尚未对齐。即使模式1减少未知像素，也不能据此确认输入分布或11束方向与模型训练合同一致。

ABS共享内存协议只有11个 `float32` 射线值及 magic、version、sequence、时间戳字段，没有逐束有效性位图（[`abs_ray2d_shm_contract.h`](../../../common/abs_ray2d_shm_contract.h#L12-L23)）。控制器校验整帧结构、时龄与数值有限性，11项都有限就作为11束观测消费（[`AbsObservationContract.cpp`](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/AbsObservationContract.cpp#L44-L50)、[`StateRL.cpp`](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp#L1064-L1098)）。运行记录中的 `ray_valid` 是整帧状态（[`StateRL.cpp`](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp#L1303-L1306)），不代表每一束可单独标未知。模型固定输入仍无有效性通道；已有训练审查也未找到真实D435i空洞与掩码训练依据（[未知深度输入审查](20261112_UNKNOWN_DEPTH_FILL_MODEL_SENSITIVITY.md)）。因而要处理真实未知，至少需要让“逐束有效性”贯穿模型输入/训练验证及消费者决策；单在预处理里填一个有限数，不能表达这个像素来自补值。

## 结论与下一步

- 新替换机九帧全部被当前严格入口拒绝；位置2的严格未知比例最高，平均16.99%。
- 6 m路径只提供诊断基线，不能解释为空旷真值。mode 1对新相机数据的未知减少量和11束输出影响本轮无法量化，因为本机没有准确版本滤波运行时。
- **不建议把任一填值或本轮6 m射线接入 ABS 控制。**下一步若要评估唯一候选，应在可运行既有 librealsense 2.51.1 的离线环境，对这同一批9张新机原始帧逐张执行 mode 1，记录原始有效掩码、滤波新增像素、残余零值、直接6 m与滤波后射线逐束差。仍需另行解决D435i与模型视场合同及接口/训练对未知状态的表达；本轮不放行机器人运动。

## 复现

在仓库根目录运行（只做原始严格统计和直接6 m模型诊断；不运行滤波）：

```bash
rtk proxy /usr/bin/python3 scripts/analyze_d435i_new_camera_hole_candidates.py
```

模型：`archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt`。脚本使用CPU，输出目录若已存在会拒绝覆盖。运行时 Torch 2.0.1+cu118 对 NumPy 2.2.6 打出已知 NumPy ABI 警告；本脚本通过只读 byte buffer 构造输入张量，9帧均完成推理并检查输出为有限值。该警告记录为环境限制，不等于射线正确性验证。
