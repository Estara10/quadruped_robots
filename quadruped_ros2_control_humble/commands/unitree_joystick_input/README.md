# Unitree Joystick Input Node

This node receives Unitree SDK2 remote state and, when explicitly enabled, publishes ROS `control_input_msgs/Inputs` on `/control_input`. It never publishes LowCmd or switches native motion services.

## Go2 EDU commissioning (2026-11-04)

`joystick.launch.py` requires explicit `network_interface` and `domain`, and defaults to `read_only=true`, `stop_only=true`, `state_source=lowstate`. For the connection verified on this computer:

```bash
ros2 launch unitree_joystick_input joystick.launch.py network_interface:=enp7s0 domain:=0
```

- `read_only=true`: no `/control_input` publisher is created. Log raw keys, decoded command and raw axes only. This is the input check entry, not a controller launch.
- `stop_only=true`: L2+B maps to command 9, SELECT maps to 1, left direction+X remains 9. Stops have priority over START and movement combinations; all output axes are zero. Other buttons/axes produce only neutral command 0. Stop commands are continuously asserted while held.
- `state_source=lowstate`: decode `LowState.wireless_remote[40]` using the SDK `REMOTE_DATA_RX` layout, then use the same command mapping. This source was verified on the actual robot. LowState reception rate is not the wireless radio rate or proof of remote link health.
- `state_source=wirelesscontroller`: subscribe `rt/wirelesscontroller` explicitly. This retains the existing simulation/topic path; this robot produced no messages on that path during the initial 120-second check. There is no automatic source fallback.
- `read_only=false` enables publishing `/control_input`; use only in a separately prepared control test. Full input mapping additionally requires `stop_only=false`. The node's legacy defaults remain false for compatibility; the launch files set commissioning behavior explicitly.

Real `real_go2.launch.py` starts this node with the same interface/domain as hardware, `read_only=false`, `stop_only=true`, `state_source=lowstate`. It also activates hardware/controller and must not be used for a read-only test. Starting the input node does not itself prove that remote messages are arriving.

For the RL controller, commands 1/9 request PASSIVE (zero gains/torque), not native damping gains or power removal. Physical stopping and native remote takeover after low-level control remain to be tested with support. This host/ROS path is not an independent hardware emergency stop and does not detect radio loss from continuing LowState packets.

Actual check: [remote stop record](../../../docs/thesis_project/real_go2/20261104_REMOTE_STOP.md).

## Existing full-input usage

The following controller mappings apply only with `read_only=false, stop_only=false`; they are not current real-robot commissioning instructions.

Tested environment:
* Ubuntu 24.04
  * ROS2 Jazzy

### Build Command
```bash
cd ~/ros2_ws
colcon build --packages-up-to unitree_joystick_input --symlink-install
```

### Launch Command
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_joystick_input joystick.launch.py network_interface:=<NIC> domain:=<DOMAIN>
```

## 1. Use Instructions for Controllers

### 1.1 Unitree Guide Controller
* Passive Mode: select
* Fixed Down: start
* Fixed Stand: start
    * Free Stand: right + X
    * Trot: right + Y
    * SwingTest: right + B
    * Balance: right + A

### 1.2 OCS2 Quadruped Controller
* Passive Mode: select
* OCS2 Mode: start
  * Stance: start 
  * trot: right + X
  * standing trot: right + Y
  * flying_trot: right + B

### 1.3 RL Quadruped Controller
* Passive Mode: select
* Global stop: L2+B or left direction+X -> command 9
* Fixed Down: start
* Fixed Stand: start
  * RL Mode: right direction + B -> command 3
