FROM osrf/ros:kilted-desktop

# Install essential tools
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

# Copy orchestrator node and entrypoint
#COPY /src/orchestrator/orchestrator_service.py /root/ros2_ws/src/orchestrator_service.py
COPY domain /root/ros2_ws/domain
COPY config /root/ros2_ws/config
COPY /src/orchestrator /root/ros2_ws/src/orchestrator
COPY /src/orchestrator/entrypoint.sh /entrypoint.sh

# Fix Python imports
ENV PYTHONPATH=/root/ros2_ws:$PYTHONPATH

# Build workspace
RUN /bin/bash -lc "source /opt/ros/kilted/setup.bash && colcon build --symlink-install || true"

# Entrypoint and default CMD
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "orchestrator_service", "orchestrator_service"]
