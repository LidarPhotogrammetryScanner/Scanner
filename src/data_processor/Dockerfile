FROM osrf/ros:kilted-desktop

# Install essential build tools
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

# Copy service and entrypoint
COPY domain /root/ros2_ws/domain
COPY scanner_pkg /root/ros2_ws/src/scanner_pkg
COPY config /root/ros2_ws/config
COPY src/data_processor /root/ros2_ws/src/data_processor
COPY src/data_processor/entrypoint.sh /entrypoint.sh

# Fix Python imports
ENV PYTHONPATH=/root/ros2_ws:$PYTHONPATH

# Build workspace
RUN /bin/bash -lc "source /opt/ros/kilted/setup.bash && colcon build --symlink-install  || true"

# Source workspace in entrypoint to fix Python imports
ENV PYTHONPATH=/root/ros2_ws/install:$PYTHONPATH

# Entrypoint and default CMD
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "data_processor_service", "data_processor_service"]
