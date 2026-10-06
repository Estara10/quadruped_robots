#!/bin/bash
# ============================================================
# ABS Obstacle Simulation Launch Script
# Starts MuJoCo (random box+cylinder obstacles) + ROS2 Controller → Auto-enter RL
# Press Ctrl+C to stop all processes
# ============================================================
set -e

# Config
MUJOCO_DIR="${HOME}/quadruped_robots/unitree_mujoco"
MUJOCO_BIN="${MUJOCO_DIR}/simulate/build2/unitree_mujoco"
# Default scene: scene_obstacle.xml (random box + cylinder obstacles)
# Override with: MUJOCO_SCENE=scene_test3.xml ./scripts/launch_abs_obstacle.sh
MUJOCO_SCENE="${MUJOCO_SCENE:-scene_obstacle.xml}"
ROS2_WS="${HOME}/quadruped_robots/quadruped_ros2_control_humble"
UNITREE_SDK2_LIB="${HOME}/Libraries/unitree_sdk2/lib"
LIBTORCH_LIB="${HOME}/Libraries/libtorch-cpu-2.0.1/lib"

# Auto-enter RL mode (set to "false" to disable)
AUTO_RL="${AUTO_RL:-true}"

# Environment
export LD_LIBRARY_PATH="${UNITREE_SDK2_LIB}:${LIBTORCH_LIB}:${LD_LIBRARY_PATH}"
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

cleanup() {
    echo -e "\n${YELLOW}[Shutdown] Stopping all processes...${NC}"
    kill %1 %2 2>/dev/null
    wait 2>/dev/null
    echo -e "${GREEN}[Shutdown] All processes stopped.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM

# -------------------------------------------------------
# Step 1: MuJoCo Simulator (obstacle scene)
# -------------------------------------------------------
echo -e "${GREEN}[1/3] Starting MuJoCo simulator (scene: ${MUJOCO_SCENE})...${NC}"
cd "${MUJOCO_DIR}"
${MUJOCO_BIN} -s "${MUJOCO_SCENE}" &
MUJOCO_PID=$!
sleep 3

if ! kill -0 ${MUJOCO_PID} 2>/dev/null; then
    echo -e "${RED}[ERROR] MuJoCo failed to start!${NC}"
    exit 1
fi
echo -e "${GREEN}  -> MuJoCo running (PID ${MUJOCO_PID})${NC}"

# -------------------------------------------------------
# Step 2: ROS2 Controller
# -------------------------------------------------------
echo -e "${GREEN}[2/3] Starting ROS2 Controller...${NC}"
cd "${ROS2_WS}"
ros2 launch rl_quadruped_controller mujoco.launch.py &
ROS2_PID=$!
sleep 8

# Wait for controller activation (Humble spawner may report FATAL due to race condition)
echo -n "  -> Waiting for controller..."
CONTROLLER_READY=false
for i in $(seq 1 30); do
    if ros2 control list_controllers 2>/dev/null | grep -q "rl_quadruped_controller"; then
        echo -e "\n${GREEN}  -> Controller loaded${NC}"
        CONTROLLER_READY=true
        break
    fi
    sleep 1
    echo -n "."
done

if [ "$CONTROLLER_READY" = false ]; then
    echo -e "\n${YELLOW}  -> Controller may still be starting (continuing anyway)${NC}"
fi

# -------------------------------------------------------
# Step 3: Auto-enter RL Mode
# -------------------------------------------------------
if [ "$AUTO_RL" = "true" ]; then
    echo -e "${BLUE}[3/3] Auto-entering RL mode...${NC}"

    # Helper: publish control_input command
    pub_cmd() {
        local cmd=$1 lx=${2:-0.0} ly=${3:-0.0} rx=${4:-0.0} ry=${5:-0.0}
        ros2 topic pub --once /control_input control_input_msgs/msg/Inputs \
            "{command: $cmd, lx: $lx, ly: $ly, rx: $rx, ry: $ry}" 2>/dev/null
    }

    # --- Phase A: PASSIVE → FIXEDDOWN → FIXEDSTAND ---
    pub_cmd 2
    sleep 0.5
    pub_cmd 2
    echo -e "  -> FIXEDDOWN → FIXEDSTAND (waiting for crouch+stand...)"

    # Wait for full FIXEDDOWN→FIXEDSTAND transition (~3.6s for both)
    sleep 4

    # --- Phase B: FIXEDSTAND → RL ---
    pub_cmd 3 0.0 1.0
    sleep 0.5
    pub_cmd 3 0.0 1.0
    echo -e "  -> FIXEDSTAND → RL (waiting for stand completion...)"

    sleep 3

    echo -e "${GREEN}  -> RL mode activated! (ly=1.0 forward)${NC}"

    echo -e "${YELLOW}  Controls: W/S=forward/back, A/D=strafe, J/L=turn, 4=Recovery, Space=stop${NC}"
    echo -e "${YELLOW}  Start keyboard in another terminal to manually control:${NC}"
    echo -e "${YELLOW}    ros2 run keyboard_input keyboard_input${NC}"
else
    echo -e "${YELLOW}[3/3] Auto-RL disabled (AUTO_RL=false). Start keyboard manually:${NC}"
    echo -e "${YELLOW}    ros2 run keyboard_input keyboard_input${NC}"
    echo -e "${YELLOW}  Controls: 2=FIXEDSTAND, 3=RL, 4=RL_REC, W/S=forward/back${NC}"
fi

echo -e "${YELLOW}  Press Ctrl+C to stop all simulation processes${NC}"

# Keep running until user Ctrl+C
wait ${MUJOCO_PID} 2>/dev/null
cleanup
