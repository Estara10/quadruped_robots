#!/bin/bash
# ============================================================
# Multi-Scene ABS Test — cycles through test scenes sequentially
# Each scene: 30s autonomous navigation + obstacle avoidance
# Logs saved to /tmp/abs_test_<scene>.log
# ============================================================
set -e

MUJOCO_DIR="${HOME}/quadruped_robots/unitree_mujoco"
MUJOCO_BIN="${MUJOCO_DIR}/simulate/build2/unitree_mujoco"
ROS2_WS="${HOME}/quadruped_robots/quadruped_ros2_control_humble"
UNITREE_SDK2_LIB="${HOME}/Libraries/unitree_sdk2/lib"
LIBTORCH_LIB="${HOME}/Libraries/libtorch-cpu-2.0.1/lib"
TEST_DURATION=${TEST_DURATION:-25}  # seconds per scene (enough for 1-2 goals)
SCENES=("scene_test1.xml" "scene_test2.xml" "scene_test3.xml" "scene_test4.xml" "scene_test5.xml" "scene_terrain.xml")

export LD_LIBRARY_PATH="${UNITREE_SDK2_LIB}:${LIBTORCH_LIB}:${LD_LIBRARY_PATH}"
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'

cleanup_all() {
    echo -e "\n${YELLOW}[Cleanup] Killing all processes...${NC}"
    kill %1 %2 2>/dev/null
    wait 2>/dev/null
    sleep 1
}

pub_cmd() {
    local cmd=$1 lx=${2:-0.0} ly=${3:-0.0} rx=${4:-0.0} ry=${5:-0.0}
    ros2 topic pub --once /control_input control_input_msgs/msg/Inputs \
        "{command: $cmd, lx: $lx, ly: $ly, rx: $rx, ry: $ry}" 2>/dev/null
}

run_scene() {
    local scene=$1
    local name="${scene%.xml}"
    local logfile="/tmp/abs_test_${name}.log"

    echo ""
    echo -e "${BLUE}========================================================${NC}"
    echo -e "${BLUE}  Scene: ${scene}${NC}"
    echo -e "${BLUE}========================================================${NC}"

    # --- MuJoCo ---
    echo -e "${GREEN}[1/2] Starting MuJoCo (${scene})...${NC}"
    cd "${MUJOCO_DIR}"
    ${MUJOCO_BIN} -s "${scene}" > "${logfile}" 2>&1 &
    MUJOCO_PID=$!
    sleep 3
    if ! kill -0 ${MUJOCO_PID} 2>/dev/null; then
        echo -e "${RED}[ERROR] MuJoCo failed to start!${NC}"
        return 1
    fi

    # --- ROS2 Controller ---
    echo -e "${GREEN}[2/2] Starting ROS2 Controller...${NC}"
    cd "${ROS2_WS}"
    ros2 launch rl_quadruped_controller mujoco.launch.py >> "${logfile}" 2>&1 &
    ROS2_PID=$!
    sleep 8

    # --- Auto-enter RL ---
    echo -e "${YELLOW}[Auto] Entering RL mode...${NC}"
    pub_cmd 2; sleep 0.5
    pub_cmd 2; sleep 4
    pub_cmd 3 0.0 1.0; sleep 0.5
    pub_cmd 3 0.0 1.0; sleep 3

    echo -e "${GREEN}  RL active! Running ${TEST_DURATION}s...${NC}"
    echo -e "${YELLOW}  Watch MuJoCo window for robot behavior${NC}"

    # Run for test duration, logging key events
    local elapsed=0
    while [ $elapsed -lt $TEST_DURATION ]; do
        sleep 5
        elapsed=$((elapsed + 5))
        # Extract key log lines
        grep -E '\[RA-REC\] (ENTER|EXIT)|\[GOAL\]|\[TWIST-GD\]|\[ARRIVED\]|\[GOAL-RESAMPLE\]' "${logfile}" 2>/dev/null | \
            tail -5 > "/tmp/abs_${name}_events.txt"
        echo -ne "\r  ${elapsed}/${TEST_DURATION}s  "
    done
    echo ""

    # Collect stats
    local rec_entries=$(grep -c "ENTER recovery" "${logfile}" 2>/dev/null || echo 0)
    local rec_exits=$(grep -c "EXIT recovery" "${logfile}" 2>/dev/null || echo 0)
    local arrived=$(grep -c "\[ARRIVED\]" "${logfile}" 2>/dev/null || echo 0)
    local resamples=$(grep -c "GOAL-RESAMPLE" "${logfile}" 2>/dev/null || echo 0)

    echo -e "${GREEN}  Results: recovery_entries=${rec_entries} arrived=${arrived} resamples=${resamples}${NC}"

    # Cleanup
    cleanup_all
    sleep 2
}

# --- Main ---
trap 'cleanup_all; exit 0' SIGINT SIGTERM

echo -e "${BLUE}ABS Multi-Scene Test${NC}"
echo -e "Scenes: ${SCENES[*]}"
echo -e "Duration per scene: ${TEST_DURATION}s"
echo -e "Press Ctrl+C to skip current scene\n"

for scene in "${SCENES[@]}"; do
    if [ -f "${MUJOCO_DIR}/unitree_robots/go2/${scene}" ]; then
        run_scene "${scene}"
    else
        echo -e "${YELLOW}  Scene ${scene} not found, skipping${NC}"
    fi
done

echo -e "\n${GREEN}All scenes complete!${NC}"
echo -e "Full logs: /tmp/abs_test_*.log"
echo -e "Event summaries: /tmp/abs_*_events.txt"
