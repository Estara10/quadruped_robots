# 三朝向深度缺失与启动慢帧离线诊断

## 结论

三次观察的深度零值比例随画面明显变化，但接收、电脑端处理和射线输出保持约60Hz。最有依据的判断是：**缺失已经存在于相机发布的深度图内容中，且与场景/视角下的双目测深条件有关；现有资料不能把主因进一步定为曝光、表面纹理、遮挡或测距范围中的某一个。** 不是已知的电脑推理积压或网络接收丢帧。

这三次不是受控朝向实验：朝向、所见场景及相机位姿没有测量或固定，因此只能称为三组不同画面的记录，不能声称“旋转角度造成零值增加”。前两组没有像素图，不对其场景作推断。

## 深度缺失证据

| 组别 | 稳定段全图零值均值 | 中央40×40零值均值 | 收图／射线输出 | 接收后至完成中位数 |
|---|---:|---:|---:|---:|
| 1 | 7.181% | 0.00256% | 59.847／59.889 Hz | 12.85 ms |
| 2 | 13.320% | 0.66718% | 59.795／59.912 Hz | 13.20 ms |
| 3 | 20.957% | 13.67488% | 59.794／59.846 Hz | 13.04 ms |

三组稳定段均无处理错误；PC UDP接收错误、缓存溢出及 `Frame resources` 错误均为0，稳定段没有新增待处理帧覆盖。稳定统计排除了各次起始5秒，但全程记录及启动帧仍保留。它们支持“帧到达后电脑跟得上”，不测量相机内部匹配过程，也不证明相机到PC的端到端时龄。

![三朝向稳定段零值比例和首帧推理时间](../../../logs/real_go2/20261112_depth_gap_startup_analysis/three_heading_depth_and_startup.png)

第三组唯一保存的深度/IR样本全图20.459%为零，中央40×40为14.25%；约80.43%的零像素属于触边连通区域，最大三个缺失连通块为9982、6345、4424像素。缺失既有成片区域，也沿物体轮廓/画面边缘分布，并非只是一两个孤立噪点。样本IR画面有强亮背景，左IR精确255值占约4.02%；深度图中部分大块空洞在视觉上与亮背景/远景处于同一大区域，另有空洞出现在物体边缘和下方区域。

**这不是逐像素对应证据。** 同header只说明三帧时间戳一致；样本的 `frame_id` 分别是 `camera_depth_optical_frame` 与 `camera_infra1_optical_frame`/`camera_infra2_optical_frame`，记录没有保存这组流的CameraInfo/注册映射，也没有执行对齐。因此不能由这张拼图计算“亮像素对应无效深度”的相关性，更不能认定全部空洞来自曝光。现有图只给出场景级线索。

第三组另有16570个深度值大于6m（占全图约12.79%），以及398个值为65535。当前观察器的 `source_valid` 只排除原始值0；因此这些非零像素仍会进入模型输入，随后按适配器合同裁剪至6m。这样处理会把所有超过6m的值（包括65535）压成同一个最大输入值，并非把它们识别为未知。该比例不是零值比例，也不代表这些远距数值准确。

RealSense ROS 4.51.1代码将深度帧映射成 `16UC1`，`fix_depth_scale()` 按SDK深度单位转成毫米；当SDK单位为1mm时直接保留原16位样本。发布代码没有对65535设专门无效语义；本次数据所用的观察器也将65535视为非零有效输入。因而**目前不能把65535称为官方无效码或可信远距测量**，它的设备端来源仍未知。SDK Z16用设备深度单位解释像素值，SDK文档说明无法测得的像素通常记为0。ROS端也能通过正的`clip_distance`把超限像素改写成0，但本次启动脚本没有指定它，4.51.1默认值为负数，且源码仅在其大于0时裁剪；所以当前更支持零值在SDK测深输出阶段已形成。仍未保存运行时参数快照，故不将驱动配置影响绝对排除。来源见[ROS 4.51.1深度缩放与发布实现](https://github.com/IntelRealSense/realsense-ros/blob/4.51.1/realsense2_camera/src/base_realsense_node.cpp#L170-L206)、[同版本图像消息发布实现](https://github.com/IntelRealSense/realsense-ros/blob/4.51.1/realsense2_camera/src/base_realsense_node.cpp#L967-L1035)、[同版本裁剪调用路径](https://github.com/IntelRealSense/realsense-ros/blob/4.51.1/realsense2_camera/src/base_realsense_node.cpp#L208-L229)、[4.51.1 clip_distance默认值](https://github.com/IntelRealSense/realsense-ros/blob/4.51.1/realsense2_camera/launch/rs_launch.py#L41-L45)、[librealsense v2.51.1帧距离实现](https://github.com/IntelRealSense/librealsense/blob/v2.51.1/src/frame.cpp)、[librealsense投影与Z16说明](https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0)。4.51.1的README注明配套librealsense v2.51.1并支持Foxy，见[固定版本README](https://github.com/IntelRealSense/realsense-ros/tree/4.51.1)。

### 目前能弱化和不能分开的解释

- **可弱化：**电脑处理不及、持续用户态队列积压、PC UDP接收缓存溢出。三组约60Hz稳定输出、接收错误为0，稳定段覆盖数不增加。它们解释不了深度消息内部的大块零像素。
- **有支持但不能细分：**画面/视角相关的立体匹配条件。不同组的零值比例不同；第三组图像中同时可见亮背景、远处区域、边缘和大块无效区。
- **仍未确认：**高亮或曝光/增益、低纹理/重复纹理、遮挡/视差、目标距离与视场边缘各自贡献。朝向与场景同时变化；IR和深度未注册；没有曝光/增益逐帧记录、深度视差或第二视角参考。

## 首帧约200ms

三组首个实际帧的模型调用分别为201.345、196.399、192.622ms；之后模型调用中位数约11.73、12.05、11.87ms。主线程中的零张量预热分别约67.6、73.5、73.7ms，而首个真实推理慢约16.7ms的60Hz帧周期的12倍；这足以解释启动阶段帧覆盖与首个输出延迟，但不说明慢在哪里。

我用现有模型和第三组保存深度做了一次无ROS离线对照：模型加载后主线程零输入预热69.55ms；同形状实际输入在主线程调用36.69ms，在新工作线程首次调用17.24ms、第二次12.29ms。该对照没有复现约200ms，也不支持把原因确定为“工作线程第一次/JIT一定很慢”。它只说明这一次本机调用中，首次线程推理并未产生现场量级延迟。CPU调度/频率、启动时系统负载或其他未记录初始化因素仍可能参与，但当前没有证据可以选定其中之一。**本轮不改预处理或推理路径，不丢弃首帧，也保留启动耗时和覆盖统计。**

## SIGINT清理修正

现场记录中曾出现SIGINT后 `ExternalShutdownException`。原观察器只捕获 `KeyboardInterrupt`；若rclpy在 `spin_once()` 中因ROS context关闭抛出前者，清理虽由`finally`执行，异常仍会以栈追踪形式逸出。本轮增加了仅针对 `rclpy.executors.ExternalShutdownException` 的关闭分支，状态记为`ros_context_shutdown`，随后仍走原有销毁订阅、关闭最新帧槽、等待工作线程、销毁节点和写出结果的`finally`路径。推理、时间窗口及数据字段不变。

离线用`py_compile`及代码路径检查验证了语法和清理结构；因本轮禁止初始化ROS/DDS，没有运行信号触发的在线验证。因此仍需下次实际使用时确认该退出状态及退出码，不把静态检查称为运行验证。

## 唯一推荐的下一次现场对照

做**一次静态、固定相机位姿和固定场景的曝光对照**，不换朝向、不移动相机、不改分辨率/帧率/投射器或DDS：先按当前自动曝光设置记录一段短样本，再只改变深度传感器曝光条件为一个由设备支持范围和当前读数确定的手动值，记录同样长度。每段同步保存同header的Depth、左右IR、CameraInfo和实际曝光/增益/自动曝光状态；检查深度与IR像素注册关系后，比较同一区域的零值比例、IR饱和值比例及空洞位置。

若设备/当前驱动不能明确读出或设置曝光，停止这项对照，不猜参数。曝光改变后空洞随亮区/饱和比例有重复变化，才支持曝光是主要贡献；若空洞基本不变，则降低“仅曝光”解释的优先级，仍不能自动证明纹理或遮挡是根因。机器人保持静止，不接控制；遇设备/profile不符、数据停收或现场要求停止时结束并保存记录。

## 复现与文件

三组输入目录：

- `logs/real_go2/20261112_024050_depth_ray60_readonly/`
- `logs/real_go2/20261112_025107_depth_ray60_heading2/`
- `logs/real_go2/20261112_025955_depth_ray60_heading3/`

使用`/usr/bin/python3`和系统Matplotlib离线生成摘要与图表：

```bash
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/go2_depth_diag_mpl \
  /usr/bin/python3 logs/real_go2/20261112_depth_gap_startup_analysis/analyze.py
```

新产物在`logs/real_go2/20261112_depth_gap_startup_analysis/`：`summary.json`、`three_heading_depth_and_startup.png`及一次性分析脚本。原始场景记录及图像未改。未修改模型、共享控制器、仿真或R1，唯一ACTIVE任务保持原状。
