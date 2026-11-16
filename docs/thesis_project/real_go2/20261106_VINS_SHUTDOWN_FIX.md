# 2026-11-06 VINS停止异常离线修复

## 问题与根因

2026-11-05相机-only静止运行在收到SIGINT后记录到`terminate called without an active exception`、退出码-6。检查固定上游提交`72023bc5d2abb14faa4c02a5c040d84f996b587a`后确认：`main()`启动`sync_process`线程，而该循环原为`while(1)`；`executor.spin()`因SIGINT返回后，主函数直接返回，joinable的`std::thread`析构会调用`std::terminate`。此外，`Estimator::setParameter()`启动的`processThread`也原为无停止条件的循环；只修同步线程会让Estimator析构时等待不能退出的线程。

离线有序退出进一步暴露了Publisher生命周期问题：`visualization.cpp`中的全局Publisher比`main()`的局部Node活得更久。线程join后进程进入静态析构时，Node/participant已先销毁，FastDDS报Publisher清理错误并SIGSEGV。GDB回溯落在`DomainParticipant::set_listener()`、`rmw_destroy_node()`和全局`pub_odometry`析构链。

## 最小修复

更新了[`0001-vins-humble-cpu-and-odometry.patch`](../../../scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch)，从原固定提交重新应用可复现。估计、输入配对和输出数据合同不变：

1. SIGINT使ROS executor返回后，通知同步线程停止并join。
2. `Estimator::shutdown()`设置原子停止标志；Estimator主循环与等待IMU的循环检查该标志，随后join后台处理线程。析构函数也调用同一清理函数，已join时不重复join。
3. 两个线程结束后调用`clearPub()`释放全部全局Publisher，此时局部Node仍存活；之后再按C++作用域顺序销毁订阅、executor和Node。

没有detach线程、吞异常、强制kill、改变退出码或修改算法/相机参数。

## 离线验证

- 固定上游提交上的补丁通过`git apply --check`、应用后`git diff --check`。
- 使用ROS 2 Humble、系统Ceres 2.0.0，在`/tmp/go2_vins_build`隔离构建`camera_models`与`vins`成功；最终修改后的增量构建摘要为`2 packages finished`。既有上游弃用API/VLA警告仍存在，不影响构建。
- 使用隔离ROS域42、`ROS_LOCALHOST_ONLY=1`和临时配置启动VINS；没有相机、图像或IMU发布者。确认完成配置/订阅初始化并在无输入等待阶段运行2秒后发SIGINT，进程在15秒内退出，实际退出码**0**；日志没有`terminate called`、Publisher清理错误或FastDDS销毁错误。后台Estimator线程启动并成功join。
- 测试配置、输出和ROS日志在`/tmp/go2_vins_shutdown_check_20261106/`；本轮测试进程已退出。

第一次检查中，默认ROS日志目录不可写，进程在logger初始化时退出；修正日志目录后，曾在等待提示一出现就发SIGINT。源码显示该提示早于订阅创建，因此这次不计为等待阶段停止验证。订阅初始化完成后的GDB回溯定位到上述全局Publisher生命周期问题；修复后才完成退出码0的检查。

## 证据边界与下一步

本次只验证无传感器输入的等待阶段，包括同步线程和Estimator空闲处理线程的退出；没有相机数据包、没有VIO初始化，也没有覆盖Estimator正在处理图像/IMU时收到SIGINT。因此不能声称初始化后或持续输出中的停止已验证。

下一项只推荐主助手在下一次相机-only检查中，于VINS稳定初始化并持续输出后按Ctrl+C一次，记录VINS退出码、日志是否有terminate/FastDDS清理错误及进程是否退出；随后再停止相机。Go2保持关机，现场不移动相机或机器人以完成这项停止检查。

**后续（2026-11-06）：**该初始化后SIGINT检查已执行。VINS在稳定输出约8秒后收到一次SIGINT，约0.164秒退出，退出码0，无terminate、SIGSEGV或FastDDS清理错误。见[本次结果](../../../logs/real_go2/20261106_102325_vins_initialized_shutdown_check/RESULT.md)。不再重复同类停止检查。
