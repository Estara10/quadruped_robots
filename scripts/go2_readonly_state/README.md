# Go2只读状态观察器

独立订阅Unitree SDK2的`rt/lowstate`和`rt/sportmodestate`，不启动ROS controller或硬件插件，不创建LowCmd发布器，不调用运动控制/控制权接口，也不加载策略模型。程序必须显式给出DDS domain和网卡名。

## 构建

本机已安装SDK2到`/home/lidio/Libraries/unitree_sdk2`，可运行：

```bash
cmake -S scripts/go2_readonly_state -B /tmp/go2_readonly_state_build \
  -DCMAKE_PREFIX_PATH=/home/lidio/Libraries/unitree_sdk2 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/go2_readonly_state_build --parallel 2
```

其他机器将`CMAKE_PREFIX_PATH`改为该机器已有的Unitree SDK2安装前缀，不需要安装ROS或模型推理依赖。

## 本机无实机检查

```bash
/tmp/go2_readonly_state_build/go2_readonly_state --domain 0 --interface lo
```

当前没有Go2或仿真消息发布到loopback时，两路都显示“尚未收到数据”，值字段不显示占位零。按Ctrl+C退出。loopback不支持组播时SDK会提示关闭组播，这是本机无数据检查的预期情况。

## 将来连接Go2

先根据实际网络与DDS配置填写，不要照搬旧手册中的接口名或假设domain：

```bash
REAL_NETWORK_INTERFACE="<已确认的网卡名>"
DDS_DOMAIN="<已确认的domain编号>"
/tmp/go2_readonly_state_build/go2_readonly_state \
  --domain "$DDS_DOMAIN" --interface "$REAL_NETWORK_INTERFACE"
```

该程序不释放原生运动控制权，因此SportModeState在释放控制权后的更新情况仍需其他经审阅的受控步骤验证；本程序本身不能完成该项验证。

## 输出解释

- LowState与SportModeState分开计数、计频、计最近接收时龄。计时来自本机`steady_clock`接收回调，不代表机器人内部传感器采样频率。
- 接收频率是启动后首条至最近一条消息的平均频率`(n−1)/(t_last−t_first)`；少于两条时显示样本不足。若消息停更，频率保留已观测平均值，最近接收时龄会持续增大。
- 数值展示为各topic最近一次收到的样本，旁边总显示该样本接收时龄，不伪装成新数据。
- 关节顺序为FR、FL、RR、RL，每腿hip/thigh/calf，与当前Go2硬件映射一致。q/dq显示接口预期rad/rad/s并提示待实机核实。
- IMU四元数按SDK/现有硬件映射的w,x,y,z显示；角速度保留SDK原始值，单位待实机核实。足端力显示SDK原始值，不擅自换算成牛顿。
- SportModeState的position/velocity原样显示；坐标系、单位及其实际定位含义待实机核实。
- 日志文件默认关闭；本版本只刷新终端，不写文件。
