# rl_sar Docker Image
# ROS2 Humble + Gazebo + MuJoCo + Real Robot Support
#
# Build from repo root:
#   docker build -f docker/Dockerfile -t rl_sar:humble .
#
# Or use docker compose:
#   cd docker && docker compose up
#
FROM ros:humble-ros-base-jammy

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# System dependencies (from README.md)
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    g++ \
    build-essential \
    libyaml-cpp-dev \
    libeigen3-dev \
    libboost-all-dev \
    libspdlog-dev \
    libfmt-dev \
    libtbb-dev \
    liblcm-dev \
    # For downloads
    curl \
    wget \
    unzip \
    # For GUI (MuJoCo visualization)
    libgl1-mesa-glx \
    libglfw3 \
    libglfw3-dev \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    libxi6 \
    # Mesa/EGL for GPU rendering
    libgl1-mesa-dri \
    libegl1 \
    libegl1-mesa \
    libglvnd-dev \
    mesa-utils \
    # Gamepad support
    joystick \
    jstest-gtk \
    # Utilities
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Create NVIDIA EGL ICD config for GPU rendering
RUN mkdir -p /usr/share/glvnd/egl_vendor.d && \
    printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' \
    > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# NVIDIA container environment
ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all

# ROS2 packages for robot control
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-teleop-twist-keyboard \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-control-toolbox \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-joy \
    # For parameter_blackboard used in gazebo.launch.py
    ros-humble-demo-nodes-cpp \
    && rm -rf /var/lib/apt/lists/*

# Gazebo simulation packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# Download Gazebo models (sun, ground_plane) using sparse checkout
RUN mkdir -p /root/.gazebo/models && \
    cd /root/.gazebo/models && \
    git clone --depth 1 --filter=blob:none --sparse \
        https://github.com/osrf/gazebo_models.git . && \
    git sparse-checkout set sun ground_plane

# Create workspace
WORKDIR /rl_sar

# Copy local source (requires: git submodule update --init --recursive)
COPY . .
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Build ROS2 workspace (for Gazebo simulation)
# Must source ROS2 setup first for colcon to find ament_cmake
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && ./build.sh

# Build MuJoCo (for MuJoCo simulation + real robot executables)
RUN ./build.sh -mj

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
