#!/bin/bash
# ============================================================
# ABS Simulation Launch Script
# Starts MuJoCo + ROS2 Controller + Keyboard Input in order
# Press Ctrl+C to stop all processes
# ============================================================
set -e

# Config
MUJOCO_DIR="${HOME}/quadruped_robots/unitree_mujoco"
MUJOCO_BIN="${MUJOCO_DIR}/simulate/build2/unitree_mujoco"
ROS2_WS="${HOME}/quadruped_robots/quadruped_ros2_control_humble"
UNITREE_SDK2_LIB="${HOME}/Libraries/unitree_sdk2/lib"
LIBTORCH_LIB="${HOME}/Libraries/libtorch-cpu-2.0.1/lib"

# Environment
export LD_LIBRARY_PATH="${UNITREE_SDK2_LIB}:${LIBTORCH_LIB}:${LD_LIBRARY_PATH}"
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

cleanup() {
    echo -e "\n${YELLOW}[Shutdown] Stopping all processes...${NC}"
    kill %1 %2 2>/dev/null
    wait 2>/dev/null
    echo -e "${GREEN}[Shutdown] All processes stopped.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM

# --- Step 1: MuJoCo Simulator ---
echo -e "${GREEN}[1/3] Starting MuJoCo simulator...${NC}"
cd "${MUJOCO_DIR}"
${MUJOCO_BIN} &
MUJOCO_PID=$!
sleep 3

if ! kill -0 ${MUJOCO_PID} 2>/dev/null; then
    echo -e "${RED}[ERROR] MuJoCo failed to start!${NC}"
    exit 1
fi
echo -e "${GREEN}  -> MuJoCo running (PID ${MUJOCO_PID})${NC}"

# --- Step 2: ROS2 Controller ---
echo -e "${GREEN}[2/3] Starting ROS2 Controller...${NC}"
cd "${ROS2_WS}"
ros2 launch rl_quadruped_controller mujoco.launch.py &
ROS2_PID=$!
sleep 8

# Wait for controller activation
echo -n "  -> Waiting for controller..."
for i in $(seq 1 20); do
    if ros2 control list_controllers 2>/dev/null | grep -q "rl_quadruped_controller.*active"; then
        echo -e "\n${GREEN}  -> Controller ready${NC}"
        break
    fi
    sleep 1
    echo -n "."
done

# --- Step 3: Ready ---
echo -e "${GREEN}[3/3] Ready!${NC}"
echo -e "${YELLOW}  Start keyboard in another terminal:${NC}"
echo -e "${YELLOW}    source /opt/ros/humble/setup.bash${NC}"
echo -e "${YELLOW}    source ${ROS2_WS}/install/setup.bash${NC}"
echo -e "${YELLOW}    ros2 run keyboard_input keyboard_input${NC}"
echo -e "${YELLOW}  Controls: 2=FixedStand, 3=RL, 4=RL_REC, W/S=forward/back${NC}"
echo -e "${YELLOW}  Press Ctrl+C to stop all simulation processes${NC}"

# Wait for MuJoCo (user will Ctrl+C to stop)
wait ${MUJOCO_PID} 2>/dev/null
cleanup
