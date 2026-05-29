#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Source local workspace if built with ROS
if [ -f /rl_sar/install/setup.bash ]; then
    source /rl_sar/install/setup.bash
fi

exec "$@"
