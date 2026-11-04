# Hardware Unitree Mujoco

ROS2 hardware interface for Unitree Go2 LowCmd/LowState over `unitree_sdk2` DDS.

Despite the historical package name, this interface is used for both:

- MuJoCo simulation (`domain=1`, `network_interface=lo`)
- Real Go2 low-level tests (explicitly confirmed DDS domain and network interface)

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

## Control ownership and shutdown (2026-11-04)

**Actual Go2 result:** restoring native `ai` caused an unintended stand during the separate prone handoff diagnostic. Do not treat `SelectMode("ai")` as a motionless cleanup/remote-return step. The real hardware control/automatic restore path remains NO-GO pending a revised recovery/support plan; this result is not standing-policy validation. See [actual record](../../../docs/thesis_project/real_go2/20261104_MODE_HANDOFF_RESULT.md).

- `on_init()` initializes DDS and state subscriptions. It does not create a LowCmd writer or release native motion control. This full plugin is still not the first-connection entry; use `scripts/go2_readonly_state`.
- Hardware `on_activate()` queries and remembers the original native mode. It releases that mode and confirms that no native motion mode remains before creating the LowCmd writer. Query/release failure rejects activation. Activation clears old gains/torques.
- Hardware `on_deactivate()`, `on_shutdown()` and destruction send a final stop sentinel, close the writer, then restore the original mode and query confirmation. A mutex excludes concurrent `write()`. Failure is logged explicitly; lifecycle callbacks return ERROR. A closed writer cannot send subsequent commands.
- If no native mode was active at entry, shutdown does not invent a replacement mode. Manual remote recovery must be arranged onsite.
- Deactivating only the RL controller clears commands to PASSIVE, but does not deactivate the hardware or restore native control. PASSIVE/command 1 or 9 unloads; it does not hold the robot upright or return control to the remote.
- Native mode query confirmation is not confirmation that the physical remote works. Abrupt process death or computer/network loss cannot be covered by host shutdown callbacks.

## Network configuration

`ros2_control.xacro` accepts `network_interface` and `domain` arguments, defaulting to `lo` and `1` for existing simulation launches. `real_go2.launch.py` requires both arguments and passes them into the hardware URDF parameters; no xacro editing is needed. This launch activates hardware and the controller; it is not a read-only entry. Supply only values confirmed for the actual robot network.

Real hardware readiness and offline checks: [Go2 preparation notes](../../../docs/thesis_project/real_go2/README.md).

## Build

```bash
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
colcon build --packages-select hardware_unitree_mujoco --symlink-install
```
