FROM osrf/ros:kilted-desktop

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

WORKDIR /root/ros2_ws
RUN mkdir -p src

COPY domain /root/ros2_ws/domain
COPY scanner_pkg /root/ros2_ws/src/scanner_pkg
COPY config /root/ros2_ws/config
COPY src/orchestrator /root/ros2_ws/src/orchestrator
COPY src/orchestrator/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Fix Python imports
ENV PYTHONPATH=/root/ros2_ws:$PYTHONPATH

RUN /bin/bash -lc "source /opt/ros/kilted/setup.bash && colcon build --symlink-install"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "orchestrator_service", "orchestrator_service"]
