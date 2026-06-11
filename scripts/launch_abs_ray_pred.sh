#!/bin/bash
# ============================================================
# ABS Ray-Pred Simulation Launch Script
# Starts MuJoCo obstacle scene in external ray mode + Ray-Pred writer + ROS2 Controller → Auto-enter RL
# Default goal is configured in abs/config.yaml: goal_x=7.0, goal_y=0.0, resample_goal_on_arrival=false.
# Press Ctrl+C to stop all processes
# ============================================================
set -e

# Config
ROOT_DIR="${HOME}/quadruped_robots"
MUJOCO_DIR="${ROOT_DIR}/unitree_mujoco"
MUJOCO_BIN="${MUJOCO_DIR}/simulate/build2/unitree_mujoco"
ROS2_WS="${ROOT_DIR}/quadruped_ros2_control_humble"
UNITREE_SDK2_LIB="${HOME}/Libraries/unitree_sdk2/lib"
LIBTORCH_LIB="${HOME}/Libraries/libtorch-cpu-2.0.1/lib"
RAY_PRED_SCRIPT="${ROOT_DIR}/scripts/ray_predictor.py"

# Defaults: obstacle scene + soft-safety Ray-Pred model
MUJOCO_SCENE="${MUJOCO_SCENE:-scene_obstacle.xml}"
MUJOCO_RAY_SOURCE="${MUJOCO_RAY_SOURCE:-ray_pred}"
RAY_PRED_MODEL="${RAY_PRED_MODEL:-${ROOT_DIR}/logs/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt}"
MUJOCO_GL="${MUJOCO_GL:-egl}"
AUTO_RL="${AUTO_RL:-true}"

# Environment
export LD_LIBRARY_PATH="${UNITREE_SDK2_LIB}:${LIBTORCH_LIB}:${LD_LIBRARY_PATH}"
export MUJOCO_SCENE
export MUJOCO_RAY_SOURCE
export RAY_PRED_MODEL
export MUJOCO_GL
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"
if [ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]; then
    source "${HOME}/anaconda3/etc/profile.d/conda.sh"
    conda activate abs
fi

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

MUJOCO_PID=""
RAY_PRED_PID=""
ROS2_PID=""

cleanup() {
    echo -e "\n${YELLOW}[Shutdown] Stopping all processes...${NC}"
    for pid in ${ROS2_PID} ${RAY_PRED_PID} ${MUJOCO_PID}; do
        if [ -n "${pid}" ]; then
            kill "${pid}" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
    echo -e "${GREEN}[Shutdown] All processes stopped.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# -------------------------------------------------------
# Step 1: MuJoCo Simulator (external Ray-Pred source)
# -------------------------------------------------------
echo -e "${GREEN}[1/4] Starting MuJoCo simulator (scene: ${MUJOCO_SCENE}, ray_source: ${MUJOCO_RAY_SOURCE})...${NC}"
cd "${MUJOCO_DIR}"
MUJOCO_RAY_SOURCE="${MUJOCO_RAY_SOURCE}" ${MUJOCO_BIN} -s "${MUJOCO_SCENE}" &
MUJOCO_PID=$!
sleep 3

if ! kill -0 "${MUJOCO_PID}" 2>/dev/null; then
    echo -e "${RED}[ERROR] MuJoCo failed to start!${NC}"
    exit 1
fi
echo -e "${GREEN}  -> MuJoCo running (PID ${MUJOCO_PID})${NC}"

# -------------------------------------------------------
# Step 2: Ray-Pred writer
# -------------------------------------------------------
echo -e "${GREEN}[2/4] Starting Ray-Pred writer...${NC}"
echo -e "${YELLOW}  -> Model: ${RAY_PRED_MODEL}${NC}"
cd "${ROOT_DIR}"
python3 "${RAY_PRED_SCRIPT}" &
RAY_PRED_PID=$!
sleep 3

if ! kill -0 "${RAY_PRED_PID}" 2>/dev/null; then
    echo -e "${RED}[ERROR] Ray-Pred writer failed to start!${NC}"
    exit 1
fi
echo -e "${GREEN}  -> Ray-Pred writer running (PID ${RAY_PRED_PID})${NC}"

# -------------------------------------------------------
# Step 3: ROS2 Controller
# -------------------------------------------------------
echo -e "${GREEN}[3/4] Starting ROS2 Controller...${NC}"
cd "${ROS2_WS}"
ros2 launch rl_quadruped_controller mujoco.launch.py &
ROS2_PID=$!
sleep 8

# Wait for controller activation
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
    echo -e "\n${YELLOW}  -> Controller may still be starting (continuing anyway)${NC}"
fi

# -------------------------------------------------------
# Step 4: Auto-enter RL Mode
# -------------------------------------------------------
if [ "${AUTO_RL}" = "true" ]; then
    echo -e "${BLUE}[4/4] Auto-entering RL mode...${NC}"

    pub_cmd() {
        local cmd=$1 lx=${2:-0.0} ly=${3:-0.0} rx=${4:-0.0} ry=${5:-0.0}
        ros2 topic pub --once /control_input control_input_msgs/msg/Inputs \
            "{command: $cmd, lx: $lx, ly: $ly, rx: $rx, ry: $ry}" 2>/dev/null
    }

    pub_cmd 2
    sleep 0.5
    pub_cmd 2
    echo -e "  -> FIXEDDOWN → FIXEDSTAND (waiting for crouch+stand...)"
    sleep 4

    pub_cmd 3 0.0 1.0
    sleep 0.5
    pub_cmd 3 0.0 1.0
    echo -e "  -> FIXEDSTAND → RL (waiting for stand completion...)"
    sleep 3

    echo -e "${GREEN}  -> RL mode activated! Goal is abs/config.yaml goal_x=7.0, goal_y=0.0; arrival stops by default.${NC}"
else
    echo -e "${YELLOW}[4/4] Auto-RL disabled (AUTO_RL=false).${NC}"
fi

echo -e "${YELLOW}  Press Ctrl+C to stop all simulation processes${NC}"
wait "${MUJOCO_PID}" 2>/dev/null
cleanup
