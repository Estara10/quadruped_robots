# 2026-11-04 遥控停止链修复与只读核对

## 任务与结果

用户确认遥控器背面铭牌原文“Go2 EDU 12679 / DATE 20240108”，并提供正面照片。照片可确认是双手双摇杆遥控器，贴纸将L2+B标为“阻尼模式（软急停）”；铭牌编号用途及具体产品型号未进一步推断，DATE不当作机器人固件版本。

用户授权补齐停止映射、实机启动链和只读按键核对。代码/离线检查完成；实机已捕获L2+B按下、停止请求映射及松开。当前唯一ACTIVE任务与仿真策略参数未改。本轮未运行ROS控制器、模型或LowCmd；未调用ReleaseMode/SelectMode。

## 最小修改

- 遥控输入新增L2+B→command=9，优先于START和其他动作组合；SELECT→1及左方向+X→9保留为停止路径。每包先清零输出，停止时不携带旧摇杆值。
- 新增read_only参数：为true时根本不创建/control_input发布器，只打印接收与解码结果。
- 新增stop_only参数：为true时只解码停止命令，其他命令与全部输出轴置零。实机launch固定使用stop_only=true，未开放站立、RL启动和摇杆运动；其网络/domain与硬件使用同一组参数，并声明遥控包运行依赖。
- 独立joystick.launch.py要求显式网卡/domain，默认read_only=true、stop_only=true、state_source=lowstate；不再硬编码旧网卡enp46s0。
- 先在rt/wirelesscontroller监听120秒，未收到该类型消息，正常SIGINT退出。未把“没有数据”当作遥控可用，也未据此断言所有固件都没有该话题。
- 对照宇树官方实机部署代码及本机SDK REMOTE_DATA_RX结构，增加显式state_source=lowstate，读取LowState.wireless_remote[40]，转换后复用同一停止解码。实机launch已指定该来源。原节点默认state_source=wirelesscontroller和原full模式保留，兼容既有仿真/其他入口；没有自动切换来源或自动放宽规则。

## 离线检查

遥控节点构建通过；停止优先级、旧轴清零、SELECT/左方向+X保留、stop_only屏蔽START与轴、松开中性、full模式旧映射保留，以及read_only不创建发布器均通过直接编译生产回调的替代ROS/SDK检查。另用明确字节序列验证LowState keys[2:4]=0x0220和四轴偏移4/8/12/20，经同一解码得到command9且不发布。测试未建立DDS。

实机与独立launch通过离线构造检查：网卡confirmed_nic、domain=37能传给遥控节点，bool/int类型与两种只读/发布配置正确，state_source均为lowstate；没有实际启动launch。第一次离线launch检查因沙箱不能写~/.ros/log而中断，设置ROS_LOG_DIR到/tmp后通过，无机器人运行。构建后安装到本机现有工作空间。

## 实机证据

网卡enp7s0、电脑192.168.123.99/24、domain=0。仅运行遥控输入节点read_only=true、stop_only=true、state_source=lowstate；ROS_LOCALHOST_ONLY=1，ROS图保持本机，SDK单独订阅机器人网卡。LowState路径持续收到数据；该接收数/频率属于LowState，不是遥控器无线链路健康证明。

实际按键变化如下：

| 状态 | keys | L2 | B | mapped_command | publish |
|---|---|---:|---:|---:|---:|
| 未按 | 0x0000 | 0 | 0 | 0 | 0 |
| 先按L2 | 0x0020 | 1 | 0 | 0 | 0 |
| L2+B | 0x0220 | 1 | 1 | 9 | 0 |
| 松开B | 0x0020 | 1 | 0 | 0 | 0 |
| 全部松开 | 0x0000 | 0 | 0 | 0 | 0 |

组合键在本机ROS日志时间1793771653.9959645被记录，随后1793771654.3154967记录全部松开。用户明确回复“已按并松开，机器人保持趴伏”。全段92个日志快照均publish=0，末次日志累计接收LowState 44026条；日志中的axes是原始轴值，输出轴清零由离线回调检查证明，未在实机发布。

满足本步完成条件：真实遥控按键可从明确来源读取，按下映射为既有全局停止请求9，松开回到中性；本次不证明控制器实际执行了停止，更不证明电机响应或遥控器重新接管。保持趴伏也不能作为阻尼/卸力动态效果的证明。

## 边界与下一步

- 进入控制前仍需在可靠支撑条件下核实控制权释放、停止效果和恢复原生ai模式及遥控操作。下一项不直接站立。
- command9在当前RL控制器走PASSIVE、零增益/力矩；并非重现原生L2+B阻尼参数，也不是独立断电急停。ROS节点/电脑/网络正常是此路径工作的必要条件。LowState继续更新不能证明遥控器仍连接，不声称具备无线失联停止保障。
- 新查到的宇树官方MuJoCo说明明确提示：实机内置运动服务关闭后SportModeState不可读，仿真则保留。当前硬件依赖该位置/速度来源，不能在缺失时填零、放宽期限或未经验证自动换估计器来运行RL。该官方说明加强了既有定位/速度缺口；本台ai固件的实际释放后行为仍待受控核实。本轮没有关闭运动服务。
- 已核对来源后向本次read_only监听进程发送SIGINT，进程退出码0；原独立话题监听也正常退出。未遗留本轮控制或监听进程。

## 记录与依据

[实际按键日志](../../../logs/real_go2/20261104_remote_stop/readonly_lowstate_keys.txt)、[初始独立话题无消息日志](../../../logs/real_go2/20261104_remote_stop/readonly_keys.txt)、[按键变化汇总](../../../logs/real_go2/20261104_remote_stop/summary.json)、[用户遥控器照片](../../../logs/real_go2/20261104_remote_stop/remote_front.png)。

协议依据：[宇树官方实机遥控解码](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/common/remote_controller.py)，以及本机SDK include/unitree/dds_wrapper/common/unitree_joystick.hpp。状态来源限制依据：[宇树官方MuJoCo说明](https://github.com/unitreerobotics/unitree_mujoco/blob/main/readme.md#supported-unitree-sdk2-messages)。
