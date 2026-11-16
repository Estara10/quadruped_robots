# 离线分析与收尾记录

- 场景运行前的初次受限执行环境 GLFW 预检失败，窗口未创建、未启动场景；随后按已授权环境重启，预检通过并完成全部120个计划场景。`runner_status.json`保留首次预检错误文本，同时记录最终`preflight_status=PASS`、`COMPLETE`和120次启动。
- 汇总绘图使用本批运行时指定的Anaconda Python。系统Python自带Matplotlib与NumPy 2.2.6不兼容，第一次仅生成图表前失败；未影响运行数据。终版分析脚本SHA-256：`36d75eb44f039432a0f9dc3bdbbb439b48fbd50c1839b062747fc011b002d496`。
- 首版分析器误读了不存在的`world_position_xy_m`字段；核对原始`runtime_record.jsonl`后按实际`world_pose=[x,y,yaw]`修正读取并重生成代表轨迹、路线侧和报告。运行终局及其安全分类未作变更。
- 汇总完成后删除了120份逐次`analysis`目录和240个重复单次汇总文件、两份配置临时overlay及本次Matplotlib缓存。完整帧记录、context、terminal、实际配置、关键ROS日志、安全事件、原始终局、运行计划和聚合分析均保留。
