#!/bin/bash
# ============================================================
# ABS Real-Robot Go2 Launch Script
# Starts ZED Ray-Pred writer + ROS2 Controller → manual RL entry
# Press Ctrl+C to stop all processes
#
# Before first use:
#   1. Edit ros2_control.xacro: uncomment network_interface line
#      and set to Go2's network interface (e.g., enp46s0, eth0, wlan0)
#   2. Rebuild: colcon build --packages-select go2_description --symlink-install
#   3. Verify ZED camera is connected
#
# Usage:
#   ./scripts/launch_abs_real.sh
#   # Then in another terminal:
#   ros2 run keyboard_input keyboard_input
#   # Press: 2 → 2 → 3 (stand up → enter RL)
#   # W to go forward slowly
#   # B/command=1 for emergency stop
# ============================================================
set -e

ROOT_DIR="${HOME}/quadruped_robots"
ROS2_WS="${ROOT_DIR}/quadruped_ros2_control_humble"
UNITREE_SDK2_LIB="${HOME}/Libraries/unitree_sdk2/lib"
LIBTORCH_LIB="${HOME}/Libraries/libtorch-cpu-2.0.1/lib"

# Ray-Pred model (uses ZED-trained weights — no domain mismatch)
RAY_PRED_MODEL="${RAY_PRED_MODEL:-${ROOT_DIR}/logs/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt}"

# Environment
export LD_LIBRARY_PATH="${UNITREE_SDK2_LIB}:${LIBTORCH_LIB}:${LD_LIBRARY_PATH}"
export RAY_PRED_MODEL
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
    echo -e "${GREEN}[Shutdown] Done.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# -------------------------------------------------------
# Step 1: Check ZED and start Ray-Pred writer
# -------------------------------------------------------
echo -e "${GREEN}[1/2] Starting ZED Ray-Pred writer...${NC}"
echo -e "${YELLOW}  Model: ${RAY_PRED_MODEL}${NC}"
cd "${ROOT_DIR}"

if command -v conda &>/dev/null && [ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]; then
    source "${HOME}/anaconda3/etc/profile.d/conda.sh"
    conda activate abs 2>/dev/null || true
fi

python3 scripts/zed_ray_predictor.py &
ZED_PID=$!
sleep 3

if ! kill -0 "${ZED_PID}" 2>/dev/null; then
    echo -e "${RED}[ERROR] ZED Ray-Pred writer failed to start! Check camera connection.${NC}"
    exit 1
fi
echo -e "${GREEN}  -> ZED Ray-Pred running (PID ${ZED_PID})${NC}"

# -------------------------------------------------------
# Step 2: ROS2 Controller
# -------------------------------------------------------
echo -e "${GREEN}[2/2] Starting ROS2 Controller...${NC}"
cd "${ROS2_WS}"
ros2 launch rl_quadruped_controller mujoco.launch.py &
ROS2_PID=$!
sleep 8

echo -n "  -> Waiting for controller..."
CONTROLLER_READY=false
for i in $(seq 1 20); do
    if ros2 control list_controllers 2>/dev/null | grep -q "rl_quadruped_controller.*active"; then
        echo -e "\n${GREEN}  -> Controller ready${NC}"
        CONTROLLER_READY=true
        break
    fi
    sleep 1
    echo -n "."
done

if [ "${CONTROLLER_READY}" = false ]; then
    echo -e "\n${YELLOW}  -> Controller may still be starting${NC}"
fi

echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  REAL ROBOT CONTROL READY${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "${YELLOW}  Start keyboard in another terminal:${NC}"
echo -e "${YELLOW}    source /opt/ros/humble/setup.bash${NC}"
echo -e "${YELLOW}    source ${ROS2_WS}/install/setup.bash${NC}"
echo -e "${YELLOW}    ros2 run keyboard_input keyboard_input${NC}"
echo ""
echo -e "${YELLOW}  Control sequence:${NC}"
echo -e "${YELLOW}    2 → 2 → 3  (stand up, enter RL)${NC}"
echo -e "${YELLOW}    W/S: forward/back (go SLOW first!)${NC}"
echo -e "${YELLOW}    Space: stop and stand${NC}"
echo -e "${YELLOW}    B / command=1: EMERGENCY STOP${NC}"
echo ""
echo -e "${RED}  SAFETY: First test — keep speed LOW (< 0.3 m/s)${NC}"
echo -e "${RED}  Be ready to press emergency stop at any time${NC}"
echo ""

wait "${ROS2_PID}" 2>/dev/null
cleanup
