# Use ROS2 Kilted core for ARM64 (Raspberry Pi)
FROM --platform=linux/arm64 ros:kilted-ros-core

# Install essential build tools and ROS 2 utilities
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    python3-vcstool \
    python3-rosdep \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update || true

# Create workspace
WORKDIR /root/ldlidar_ws
RUN mkdir -p src

# Clone ldlidar_ros2 repo with submodules
RUN git clone https://github.com/ldrobotSensorTeam/ldlidar_ros2.git src/ldlidar_ros2 \
    && cd src/ldlidar_ros2 \
    && git submodule update --init --recursive

# Patch pthread include in log_module.cpp
RUN sed -i '1i#include <pthread.h>' src/ldlidar_ros2/sdk/src/log_module.cpp

# Install runtime dependencies needed by ldlidar_ros2
RUN apt-get update && apt-get install -y \
    libxcb-xinerama0 \
    libxcb-xinerama0-dev \
    libxkbcommon-x11-0 \
    mesa-utils \
    ros-kilted-rmw-cyclonedds-cpp \
    ros-kilted-cyclonedds \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-kilted-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace
WORKDIR /root/ldlidar_ws
RUN /bin/bash -lc "source /opt/ros/kilted/setup.bash && colcon build --symlink-install"

# Copy entrypoint script
COPY src/lidar_data/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
