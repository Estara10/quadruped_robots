# Go2 真机部署手册

> 状态: 准备中 | 最后更新: 2026-06-05

## 硬件连接

1. **网线**: PC 网口 ↔ Go2 网口（Go2 默认 IP: `192.168.123.161`）
2. **PC IP**: 设置同子网，如 `192.168.123.100/24`
3. **开机**: Go2 上电，等待 DDS 就绪（约 30s）

## 网络验证

```bash
# 检查 Go2 是否在线
ping 192.168.123.161

# 查看本机 IP
ip addr show enp46s0
```

## 启动步骤

**步骤 1**: 确认 Go2 已开机且 DDS 就绪

**步骤 2**: 启动 RL 控制器（真机模式，不启动 rviz）:
```bash
source /opt/ros/humble/setup.bash
source ~/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash
export LD_LIBRARY_PATH=/home/lidio/Libraries/unitree_sdk2/lib:/home/lidio/Libraries/libtorch-cpu-2.0.1/lib:$LD_LIBRARY_PATH
ros2 launch rl_quadruped_controller real_go2.launch.py
```

**步骤 3**: 键盘遥控:
```bash
source /opt/ros/humble/setup.bash
source ~/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash
ros2 run keyboard_input keyboard_input
```

**操作顺序**:
- 按 `2` → 进入 FixedDown（趴下）
- 按 `2` → 进入 FixedStand（站立）
- 按 `W` ~20 次 → ly=1.0（全速前进）
- 按 `3` → 进入 RL 模式
- 继续按 `W` 维持前进

## 安全特性

| 特性 | 状态 | 说明 |
|------|------|------|
| DDS 超时检测 | ✅ | >200ms 关节位置冻结 → 强制 PASSIVE + FATAL 日志 |
| 遥控器 B 键急停 | ❌ | 订阅 `/rt/wirelesscontroller` |
| 电机温度监控 | ❌ | 温度 > 80°C → 强制 PASSIVE |
| Soft start | ✅ | 进入 RL 时 Kp/Kd 从 0 渐进到目标值 (~0.5s) |
| 扭矩限制 | ✅ | 已配置 (33.5 Nm) |

## 故障应急

1. **机器人异常动作**: 立即按 `1`（PASSIVE）→ 电机断电
2. **DDS 断连**: 自动检测（200ms 位置冻结）→ 强制 PASSIVE
3. **启动冲击防护**: Soft start 自动渐进 Kp/Kd，无需手动干预

## 网络配置参考

| 参数 | 真机 | 仿真 |
|------|------|------|
| DDS domain | 0 | 1 |
| 网络接口 | enp46s0 (有线) / wlan0 (WiFi) | lo |
| DDS 协议 | UDP multicast | UDP multicast |
