# D435i 深度到 ABS 11 束射线：离线适配准备

日期：2026-11-10。只读代码和已关闭记录，新增单张深度文件离线入口；未启动相机、ROS、DDS、VINS、MuJoCo或Go2控制器，没有访问或写入真实共享内存。当前VINS现场命令显式关闭 `enable_depth`；已有深度图来自独立旧检查，不是双IR/IMU/VINS同时运行的证据。

## 数据合同

| 环节 | 已有合同与证据 | 实机适配边界 |
|---|---|---|
| 深度编码／单位 | 历史 ROS 记录为 `16UC1`／Z16，正值是设备深度单位。实时 RealSense SDK 的 `depth_sensor.get_depth_scale()` 才能给出米/单位；旧 `device_enumeration.txt` 的 `Depth Units` 行只显示选项默认值 `0.001`，不是本次保存帧的实际读取记录。 | 新入口要求显式 `--unit z16 --z16-scale-m <实测scale>`；米制数组用 `--unit m`。不会从默认值猜比例。 |
| 分辨率／预处理 | 微调集由 [generate_mujoco_ray_dataset.py](../../../scripts/generate_mujoco_ray_dataset.py) 以 160×90 原生渲染生成；[finetune_mujoco_ray_pred.py](../../../scripts/finetune_mujoco_ray_pred.py) 裁剪米制图到 `[0.1,6]` 后取 `log2`。推理入口 [ray_predictor.py](../../../scripts/ray_predictor.py) 把单通道复制为3通道。 | 新入口只接受 16:9 图像并采样成 160×90，不做隐式裁剪/拉伸。各目标采样块有任一无效像素则拒绝整帧。采样值取像素中心；原训练数据没有这一重采样步骤，因此需由现场静态图验证。 |
| 无效深度 | 原微调集为 MuJoCo 渲染图，没有建立 D435i 零值／NaN／Inf 的训练处理合同。旧 RealSense 入口先 `clip` 再检测零，导致零被改成 0.1 m；之后的 `<=0` 判断再也抓不到它。 | 零、负数、NaN、Inf 在裁剪前标成未知；只要采样覆盖仍有未知，不运行模型、不生成射线帧。大于6m及正值小于0.1m按已有训练裁剪合同限幅；这不把零值伪装为远处无障碍。 |
| 相机几何 | D435i 历史设备枚举含 480×270 Depth `fx=fy=244.1043`、`cx=240.5143`、`cy=132.6388`，系数为零，列示FOV约89.03°×57.89°；该枚举不是当前USB2下实际运行 profile。另有640×480保存帧及其CameraInfo（`16UC1`、`plumb_bob`、D全零），但未记录其米制scale。模型训练相机为 [compare_ray_pred.py](../../../scripts/compare_ray_pred.py) 的160×90、HFOV 102°，MuJoCo输入为原生渲染。 | 模型训练视场与历史D435i深度视场不一致。新入口不使用内参做点云/几何变换，也不对齐彩色图、不从VINS在线外参推机器人安装位姿。安装角度、相机到Go2 `base` 的位置／旋转仍待测；预测方向只可按训练的名义相机视角解释。 |
| 模型输出 | 当前候选权重显式指定为 `archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt`，TorchScript、输入 `(1,3,90,160)`、输出 `(1,11)`。网络回归11个距离，不是从深度图抽11列。 | 角度按训练代码为 `−45° + 9°·i`（i=0…10），训练参考的水平射线原点为 base平面 `x=-0.05m, y=0`，范围 `[0.1,6]m`。载荷为 `log2(m)`。真实相机视场／安装参考点未匹配，因此束的物理朝向尚未验证。 |
| 控制器／共享内存 | [abs_ray2d_shm_contract.h](../../../common/abs_ray2d_shm_contract.h)：magic `0x415253594143544f`、version 2、11个float及 `FrameHeader(magic,version,sequence,monotonic_ns)`；偶数sequence为稳定帧。控制器 [StateRL.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp) 经一致快照读取，检查header、序号、有限值和配置时龄（默认200ms），载荷按 `log2(m)`直接进入观测。 | 新入口只写离线 `.frame` 文件（32字节头＋44字节载荷）和JSON说明，不调用 `shm_open`、`mmap` 或 `/dev/shm`。文件时间戳是本次推理完成时间，不是图像采集时间；离线文件不能复制到控制器共享内存。 |

### 视场与射线含义

ABS训练及当前控制器使用的11束方向来自模型标签定义：`-45,-36,...,0,...,+36,+45` 度；不能因输出有11项就等同于11列像素。训练标签是简化的水平平面障碍距离，训练相机名义安装为 Go2 base 上方0.27m、朝前；D435i当前安装外参尚未测量。D435i历史480×270 Depth HFOV约89°，训练输入HFOV为102°，存在约13°的视图差。此次不裁切、拉伸视场或改射线定义；真实画面中的方向和距离需用静态已知目标检查。

USB2流组合也不能靠单路profile列表推断。Intel D400 2024资料的USB2表列出Depth+左Y8及D435i IMU组合，右红外标为校准用途；同一资料的USB3同时流表也说明Depth、IR左、IR右常规三流并发仅支持专用校准格式。与此同时，本项目在USB2下曾实际记录480×270双IR+IMU。项目记录没有Depth，因此没有证明“再加Depth”可在本机固件／驱动／端口下同时工作。首次观察必须由一个相机pipeline实际解析并记录联合profile；没有可用联合profile就停止，不开第二个pipeline争抢设备，也不把左右IR切换流程说成联合通过。参考：[Intel D400 Series Datasheet，USB2格式及同时流表](https://www.intelrealsense.com/wp-content/uploads/2024/10/Intel-RealSense-D400-Series-Datasheet-October-2024.pdf)；[本项目机载记录](20261106_ONBOARD_REALSENSE_BRINGUP.md)。

## 旧入口不兼容处

[realsense_ray_predictor.py](../../../scripts/realsense_ray_predictor.py) 是会独占相机的旧在线脚本，本轮没有运行或修改它。它固定打开640×480@30，与当前USB2的480×270@15候选不同；默认模型路径不存在时会静默切到另一个权重；输出直接覆盖 `/mujoco_ray2d` 的44字节裸数组，缺少当前版本化头、序号和时龄；用整步长抽样后从左上角截取，不是明确的16:9映射；深度零值在clip时变成0.1m；旧HFOV注释还把深度视场写成待确认的69°。其彩色对齐设置发生在pipeline启动后，不能据此认为已运行彩色对齐。

这些是代码层不兼容证据；它们不证明模型对真实深度图的预测是否准确。

## 新离线入口

[realsense_depth_to_abs_rays.py](../../../scripts/realsense_depth_to_abs_rays.py) 只读取一个二维 `.npy`，不导入 RealSense SDK、ROS或DDS，不连接设备。它复用 [ray_predictor.py](../../../scripts/ray_predictor.py) 的输入尺寸、范围和header布局，并显式给出输入单位和现存TorchScript模型，没有权重fallback。无效输入会写 `PREFIX.json`（`prediction_valid=false`、射线为空）并以退出码2结束，不生成 `.frame`。有效输入写出协议字段兼容的单帧文件与JSON；JSON分别留出采集时间、电脑接收时间和推理时间，离线文件的前两者为null。程序是单帧，不刷新旧图的时间戳，也没有在线输入时龄检查。

例：Z16 scale必须来自该次设备／驱动的实际读取值，不使用下方示例常数代替实测：

```bash
/usr/bin/python3 scripts/realsense_depth_to_abs_rays.py \
  --input /path/to/saved_depth.npy --unit z16 --z16-scale-m "$DEPTH_SCALE_M" \
  --model /home/lidio/quadruped_robots/archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt \
  --output-prefix /path/to/new_output/ray_frame
```

若文件已经是米制浮点深度，使用 `--unit m` 且不传 `--z16-scale-m`。输入4:3图像会明确拒绝，因为本轮没有证据选择裁剪或拉伸；480×270是低负担候选尺寸，但其Depth+双IR+IMU联合profile尚未在当前USB2链路验证。

## 离线验证

- `--help` 正常，脚本中无相机、DDS、ROS节点、控制器或共享内存调用。
- 合成480×270米制2m和Z16 2000×显式0.001 m/unit输入均生成160×90×3的 log2 张量，范围为1（log2 2m）；模型输出有限的11项。合成单位只用于数值检查，不是本机深度scale依据，也不说明预测正确。
- 本机两次合成CPU推理报告约76.1ms（模型调用及输出转换）。一次同机C++解析器接续读取v2帧成功，时间龄约191ms；其中包含独立进程启动开销，不代表实时吞吐或稳定延迟，但说明离线文件很快会变旧，不能据此喂控制器。
- 合成零、NaN、Inf输入保留无效计数，退出码2，未生成 `.frame`。
- 对有效合成输出，临时C++检查程序使用 [abs_ray2d_validation.h](../../../common/abs_ray2d_validation.h) 解析帧头与11个float，确认magic/version、稳定序号和载荷格式；检查时让验证时钟等于文件时间戳，只验证序列化，不验证真实时龄。文件约76字节。没有启动控制器或访问真实共享内存。
- 已保存的 `20261105_145010_d435i_scene_check/depth_raw.npy` 是480×640 `uint16`，非零比例65.88%，但未记录实际scale；它是4:3，故不能用来声称当前模型输入已在真实帧上验证。`device_enumeration.txt`中的0.001只是Depth Units默认值，不足以补足该scale。
- 本机Torch 2.0.1+cu118与NumPy 2.2.6存在NumPy 1.x/2.x桥接警告，CUDA不可用。入口改用CPU单线程和字节缓冲传Tensor；本次单帧推理成功但仍打印依赖警告。没有安装、升级或修改包。

## 下一次静态相机观察

只做一次静态、无控制观察，Go2保持静止；先确认当前USB链路仍为USB2和设备固件。一个相机节点独占设备，**不并行开Viewer、SDK pipeline或第二个相机节点**。

1. 用相机驱动在唯一pipeline中查询并尝试解析Depth Z16、双IR、gyro和accel的联合profile；480×270@15只作为低负担候选，且该三图像流并发有官方校准格式限制，不能预先当作当前USB2下可用。记录实际打开的所有profile、depth scale、Depth CameraInfo内参/畸变/`frame_id`、图像编码、header时间、接收时间及深度无效比例。若联合profile可用且VINS现有可执行文件可直接使用，可作为同一ROS图内的只读订阅者观察静止输出；不得为此覆盖A→B或VINS构建目录。若完整联合组合无法解析，本轮现场步骤就以失败证据结束，不换高分辨率或另开pipeline；将“Depth与双IR/IMU共存”列为未通过。
2. 若联合profile打开，在同一pipeline中记录深度及既有双IR/IMU频率和最大间隔；用静止场景中的中心及左右已知方向目标、可测量距离检查深度与射线方向。建议约30秒有限观察。机器人不运动，不接LowCmd。保存少量原始深度帧及每帧scale元数据，停相机后再运行本入口。
3. 每张离线帧记录无效比例和模型推理耗时；与实测距离/方向逐项对照，不定义未经依据的精度通过阈值。脚本没有在线射线输出频率或输入数据时龄，不能用于控制器，也不能说明200ms共享内存时龄检查已满足。在线输入来源、采集/接收时间校验、持续输出频率仍需单独实现与验证；若现场需要同步测输出频率/帧龄，本轮工具无法提供该能力。

若新增Depth后图像/IMU明显降频、设备报错、VINS静态输出失效、无效深度占比过高或现场人员要求停止，先停止相机观察并保存记录。本入口没有停止机器人或控制器的能力。本轮没有执行该现场任务。

## 修改范围与结论

新增单图离线适配器；更新脚本导航及实机文档索引。未修改旧在线脚本、模型、策略观测、控制器、仿真设置、唯一ACTIVE任务或A→B/VINS配置与运行目录。合成检查证明输入单位转换、尺寸、无效值拒绝、模型输出维度和离线协议序列化路径可工作；不能证明真实D435i深度scale、真实模型精度、视场/安装匹配、实时频率/时龄、联合流兼容或避障安全。
