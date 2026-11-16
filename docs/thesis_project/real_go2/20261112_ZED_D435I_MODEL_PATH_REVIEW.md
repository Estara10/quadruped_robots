# ZED原作者路径与当前D435i模型差异核对

本轮只读查阅论文、官方代码、厂商说明和本地模型清单，未访问设备、运行推理、训练或控制。

## 直接结论

当前约2m柜前画面产生约0.8m的正前方模型输出，不能解释成D435i直接测深只有0.8m，也不能主要归因于是否填6m。对当前3对样本，补洞处理使正前方预测由0.831m变为0.779m，仅约5.2cm；两种处理都产生很近的预测。相机中心深度与模型机身几何射线定义不同，故不把1.2m差值当正式误差，但它足以要求审查感知适配。

## 原作者到底做了什么

ABS论文VII节：用仿真深度与机身水平方向几何射线成对监督训练ResNet-18，使用多种障碍、水平翻转、随机擦除、高斯模糊和高斯噪声，部署补洞，输入160×90。附录给出的训练数据规模参考为250k带标签图像。它不是从深度图固定取11个点。

原作者ZED发布代码使用ZED Mini、PERFORMANCE深度模式、米制、最大6m、SDK fill mode；取160×90深度，将残余Inf替换6m，log2后复制3通道，模型输出再2的幂还原距离。代码里的权重路径为占位符，没有指明本项目当前权重。

本地原始训练脚本和当前推理对log2/三通道/exp2的基本约定一致，未发现简单漏取指数或毫米当米的依据。此代码核对不替代完整几何或模型验证。

## 当前最关键差异：权重与已知性能

当前实机诊断加载的是`archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt`，属于本项目MuJoCo微调模型。其manifest中的best_metrics为MAE=0.5426m、RMSE=1.0544m，并记录false_danger=4387、false_safe=158（这些是该验证集上的帧×束计数，非比例）。不能把这份模型当作已验证的精密实机测距模型。

中间medium模型的manifest指向本项目20260528的go2_depth模型作为基模；该基模文件仍在。后续应在同一批保存帧、相同预处理下比较微调前与后的输出，先区分模型退化与相机画面域偏移，不立即重新训练或更换相机。加权训练设置偏重近障/危险低估代价，但本轮未证明它导致当前近距离预测偏置。

## 相机几何仍需匹配，但不是已经确认的根因

本项目训练记录为102°水平视场，实机由内参计算约89°，且当前画面来自趴伏姿态。高度、俯角和地面比例会改变图像分布。不能仅以89°与102°不同就认定大偏差根因：Stereolabs官方区分镜头最大视场与校正图像有效视场；ZED Mini典型HD720校正水平视场也小于102°，实际应读每台相机SDK内参。原作者运行时内参未包含在发布脚本中，不能假定其实际有效视场恰为102°。

论文还记录暗走廊出现射线网络漏检引发碰撞，原作者系统也有感知局限。因此不能由论文成功演示推出ZED在任意输入和模型下都可靠。

## 来源

- [ABS论文VII、VIII-C、IX与附录](https://arxiv.org/html/2401.17583v3)。
- [原作者发布代码](https://github.com/LeCAR-Lab/ABS/blob/main/deployment/src/abs_src/publisher_depthimg_linvel.py)。
- [原作者训练相机配置](https://github.com/LeCAR-Lab/ABS/blob/main/training/legged_gym/legged_gym/envs/go1/go1_pos_config.py)。
- [ZED最大视场与校正图像实际视场](https://support.stereolabs.com/articles/8809264540-what-is-the-camera-focal-length-and-field-of-view)。
- [本地当前权重清单](../../../archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/manifest.json)。

本轮没有启动进程或新增设备状态，无进程需要清理；未修改模型、数据、算法或配置，未提交Git。
