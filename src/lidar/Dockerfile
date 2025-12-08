# Use the same base as your ROS2 image
FROM osrf/ros:kilted-desktop

# Install dependencies for building
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace
WORKDIR /root/ldlidar_ws
RUN mkdir -p src

# Clone the ldlidar_ros2 repo
RUN git clone https://github.com/ldrobotSensorTeam/ldlidar_ros2.git src/ldlidar_ros2 \
    && cd src/ldlidar_ros2 \
    && git submodule update --init --recursive

# Patch pthread include
RUN sed -i '1i#include <pthread.h>' src/ldlidar_ros2/sdk/src/log_module.cpp

RUN apt-get update && apt-get install -y \
    libxcb-xinerama0 \
    libxcb-xinerama0-dev \
    libxkbcommon-x11-0 \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-kilted-rmw-cyclonedds-cpp \
    ros-kilted-cyclonedds \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace
WORKDIR /root/ldlidar_ws
RUN /bin/bash -lc "source /opt/ros/kilted/setup.bash && colcon build --symlink-install"

# Source the workspace in entrypoint
COPY /src/lidar/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
