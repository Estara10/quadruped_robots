# Hardware Unitree Mujoco

ROS2 hardware interface for Unitree Go2 LowCmd/LowState over `unitree_sdk2` DDS.

Despite the historical package name, this interface is used for both:

- MuJoCo simulation (`domain=1`, `network_interface=lo`)
- Real Go2 low-level tests (`domain=0`, `network_interface=enp7s0`)

---

## Interfaces

Command interfaces per joint:

- `position`
- `velocity`
- `effort`
- `kp`
- `kd`

State interfaces per joint:

- `position`
- `velocity`
- `effort`

Sensors:

- IMU: orientation, angular velocity, linear acceleration
- Foot force: FR, FL, RR, RL
- Odometer: position(x,y,z), velocity(x,y,z)

---

## Joint / Motor Order

Controller joint order is fixed:

```text
FR_hip, FR_thigh, FR_calf,
FL_hip, FL_thigh, FL_calf,
RR_hip, RR_thigh, RR_calf,
RL_hip, RL_thigh, RL_calf
```

`HardwareUnitree` now uses an explicit mapping:

```cpp
motor_index_map_ = {0,1,2, 3,4,5, 6,7,8, 9,10,11};
```

This means:

```text
controller[0] FR_hip_joint   -> Unitree motor[0]
controller[1] FR_thigh_joint -> Unitree motor[1]
...
controller[11] RL_calf_joint -> Unitree motor[11]
```

Startup prints `[MOTOR-MAP]` lines. If real Go2 mapping proves different, change `motor_index_map_` rather than reordering YAML or policy data.

---

## Stop Sentinel / PASSIVE

Unitree low-level stop uses sentinel values:

```cpp
PosStopF = 2.146E+9f;
VelStopF = 16000.0f;
```

PASSIVE command semantics:

```text
kp=0, kd=0, tau=0
-> q=PosStopF, dq=VelStopF
```

`write()` starts each cycle by setting all 20 motor slots to stop sentinel, then overwrites the 12 controlled joints. This avoids stale commands on unused motor slots.

---

## Real Go2 sport_mode Release

In real mode (`network_interface != lo`), the hardware interface initializes `MotionSwitcherClient` after `ChannelFactory::Init()` and releases native motion service:

```text
sport_mode / ai_sport / advanced_sport -> ReleaseMode()
```

This prevents Go2 native controller from fighting ROS2 LowCmd.

Expected log:

```text
Motion service 'sport_mode' is active; releasing before LowCmd control
Motion service is already deactivated
```

If release fails or service remains active, stop and do not press keyboard commands.

---

## Network Modes

Simulation mode in `ros2_control.xacro`:

```xml
<!--<param name="domain">0</param>-->
<!--<param name="network_interface">enp7s0</param>-->
```

Real Go2 mode:

```xml
<param name="domain">0</param>
<param name="network_interface">enp7s0</param>
```

Rebuild `go2_description` after switching.

---

## Build

```bash
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
colcon build --packages-select hardware_unitree_mujoco --symlink-install
```
