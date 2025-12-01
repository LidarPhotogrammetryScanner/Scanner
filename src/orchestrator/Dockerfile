# Minimal generic ROS 2 image
FROM osrf/ros:kilted-desktop

# Install essential ROS 2 build tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    libxcb-xinerama0 \
    libxcb-xinerama0-dev \
    libxkbcommon-x11-0 \
    mesa-utils \
    ros-kilted-rmw-cyclonedds-cpp \
    ros-kilted-cyclonedds \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /root/ros2_ws
RUN mkdir -p src

# Copy Python service and entrypoint
COPY ./run_service.py /root/ros2_ws/src/run_service.py
COPY ./entrypoint.sh /entrypoint.sh

# Build workspace (empty workspace is okay)
RUN /bin/bash -lc "source /opt/ros/kilted/setup.bash && colcon build --symlink-install || true"

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command keeps container alive
CMD ["bash"]
