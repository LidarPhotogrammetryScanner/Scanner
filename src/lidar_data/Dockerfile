# Use ROS2 Kilted core for ARM64 (Raspberry Pi)
FROM --platform=linux/arm64 ros:kilted-ros-core

ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------
# ROS 2 Middleware configuration
# -----------------------------
# Force Fast DDS (NO CycloneDDS)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml

# -----------------------------
# Install build + runtime deps
# -----------------------------
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    build-essential \
    cmake \
    git \
    libxcb-xinerama0 \
    libxcb-xinerama0-dev \
    libxkbcommon-x11-0 \
    mesa-utils \
    ros-kilted-rmw-fastrtps-cpp \
    ros-kilted-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Initialize rosdep (safe in Docker)
# -----------------------------
RUN rosdep init || true && rosdep update || true

# -----------------------------
# Create workspace
# -----------------------------
WORKDIR /root/ldlidar_ws
RUN mkdir -p src

# -----------------------------
# Clone LDLiDAR ROS2 repo
# -----------------------------
RUN git clone https://github.com/ldrobotSensorTeam/ldlidar_ros2.git src/ldlidar_ros2 \
    && cd src/ldlidar_ros2 \
    && git submodule update --init --recursive

# -----------------------------
# Patch pthread include
# -----------------------------
RUN sed -i '1i#include <pthread.h>' src/ldlidar_ros2/sdk/src/log_module.cpp

# -----------------------------
# Build workspace
# -----------------------------
WORKDIR /root/ldlidar_ws
RUN /bin/bash -lc "\
    source /opt/ros/kilted/setup.bash && \
    colcon build --symlink-install \
"

# -----------------------------
# Fast DDS profile (disable SHM)
# -----------------------------
RUN mkdir -p /root/.ros
COPY config/fastdds.xml /root/.ros/fastdds.xml


# -----------------------------
# Entrypoint
# -----------------------------
COPY src/lidar_data/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
