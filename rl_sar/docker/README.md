# Docker Support for rl_sar

Docker environment for simulation and deployment with ROS2 Humble.

## Prerequisites

Before building, initialize git submodules:
```bash
git submodule update --init --recursive
```

## Quick Start

```bash
# Enable X11 forwarding (on host)
xhost +local:docker

# Build and run (from repo root or docker/ directory)
cd docker
docker compose up -d

# To use software rendering (Mesa llvmpipe) instead of GPU:
# docker compose --profile software up -d rl_sar_software

# Check which renderer is used:
# docker exec -it rl_sar glxinfo | grep "OpenGL renderer"
#   GPU mode:      NVIDIA GeForce RTX xxxx
#   Software mode: llvmpipe (LLVM xx.x, 256 bits)

# Rebuild after code changes, if only change policies not codes, no need to rebuild
docker compose up -d --build

# in the docker folder
docker compose exec rl_sar bash
# or anywhere
docker exec -it rl_sar bash
```

## Usage

### MuJoCo Simulation

```bash
./cmake_build/bin/rl_sim_mujoco <ROBOT> <SCENE>
# Example:
./cmake_build/bin/rl_sim_mujoco g1 scene_29dof
./cmake_build/bin/rl_sim_mujoco go2 scene
```

### Gazebo Simulation

```bash
# Terminal 1: Launch Gazebo
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=<ROBOT>

# Example:
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=g1

# Terminal 2: Run controller
source install/setup.bash
ros2 run rl_sar rl_sim

# Terminal 3: control via ros topic
# Press N to enter navigation mode
source install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10
# Note, current impl won't stop if the message is not received for a while, press N again to exit navigation mode

```

### Real Robot

```bash
# Go2/Go2W
./cmake_build/bin/rl_real_go2 <NETWORK_INTERFACE> [wheel]

# G1 (29dofs)
./cmake_build/bin/rl_real_g1 <NETWORK_INTERFACE>

# A1
./cmake_build/bin/rl_real_a1
```

## Controls

| Gamepad | Keyboard | Description |
|---------|----------|-------------|
| A | Num0 | Stand up (move to default pose) |
| B | Num9 | Return to initial pose |
| RB+DPadUp | Num1 | Basic locomotion |
| LY/LX | W/S, A/D | Move forward/backward, left/right |
| RX | Q/E | Yaw rotation |
| RB+Y | R | Reset simulation (Gazebo) |

## Files

| File | Description |
|------|-------------|
| `Dockerfile` | ROS2 Humble + Gazebo + MuJoCo + Real Robot |
| `docker-compose.yml` | X11 + gamepad + network support |
| `entrypoint.sh` | Sources ROS2 setup |

## Build Manually

```bash
# From repo root
docker build -f docker/Dockerfile -t rl_sar:humble .
```

## Notes

- Gazebo models (sun, ground_plane) are automatically downloaded during build
- Policy files are mounted read-only from `../policy`
- Container runs in privileged mode with host network for robot communication
- Gamepad devices are passed through via `/dev/input`
