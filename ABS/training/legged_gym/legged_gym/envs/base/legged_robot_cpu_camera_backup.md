# legged_robot_cpu_camera_backup.py

## 用途

原始 `legged_robot.py` 的备份。

## 修改原因

原始代码使用 `enable_tensors=True` + `get_camera_image_gpu_tensor` 获取深度图，依赖 CUDA-Vulkan 互操作。这在以下 GPU 上会段错误：
- NVIDIA A800（纯计算卡，无显示输出，Vulkan 支持不完整）
- RTX 4060（Isaac Gym Preview 4 的 Vulkan 版本与较新驱动不兼容）

错误信息：
```
cudaImportExternalMemory failed on rgbImage buffer with error 999
段错误 (核心已转储)
```

## 修改内容（在 `legged_robot.py` 中）

1. `camera_props.enable_tensors = False` — 关闭 GPU tensor 模式
2. 相机循环中 `self.camera_tensors.append(None)` — 不预分配 GPU tensor
3. `dump_depth()` — 改为 CPU 路径：`render_all_camera_sensors` → `get_camera_image` → `np.array` → `torch.tensor`
4. `render_cameras()` — 简化为直接调用 `dump_depth()`

CPU 取图路径不影响数据质量，仅增加微小的 GPU→CPU 拷贝开销。
