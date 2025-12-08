# Base image with ROS 2 Kilted desktop
FROM osrf/ros:kilted-desktop

# Install essential ROS 2 build tools and git
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Optional: network libraries, useful for GUI or DDS
RUN apt-get update && apt-get install -y \
    libxcb-xinerama0 \
    libxcb-xinerama0-dev \
    libxkbcommon-x11-0 \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Optional: CycloneDDS for ROS 2 communication
RUN apt-get update && apt-get install -y \
    ros-kilted-rmw-cyclonedds-cpp \
    ros-kilted-cyclonedds \
    && rm -rf /var/lib/apt/lists/*

# Create a generic workspace
WORKDIR /root/ros2_ws
RUN mkdir -p src

# Build the workspace (empty workspace is fine)
WORKDIR /root/ros2_ws
RUN /bin/bash -lc "source /opt/ros/kilted/setup.bash && colcon build --symlink-install || true"

# Copy entrypoint script and set entrypoint
COPY /src/servo/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]
