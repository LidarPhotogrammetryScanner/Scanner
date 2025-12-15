FROM --platform=linux/arm64 ros:kilted-ros-core

# -----------------------------
# ROS 2 Middleware configuration
# -----------------------------
# Force Fast DDS (NO CycloneDDS)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml

# Install essential build tools and Python GPIO library
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    git \
    libxcb-xinerama0 \
    libxcb-xinerama0-dev \
    libxkbcommon-x11-0 \
    mesa-utils \
    ros-kilted-rmw-cyclonedds-cpp \
    ros-kilted-cyclonedds \
    python3-pip \
    python3-rpi.gpio \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y python3-pydantic

# Install RpiMotorLib
RUN pip3 install --break-system-packages RpiMotorLib

# Create workspace
WORKDIR /root/ros2_ws
RUN mkdir -p src

# Copy service and entrypoint
COPY domain /root/ros2_ws/domain
COPY scanner_pkg /root/ros2_ws/src/scanner_pkg
COPY config /root/ros2_ws/config
COPY src/step_motor /root/ros2_ws/src/step_motor
COPY src/step_motor/entrypoint.sh /entrypoint.sh

# -----------------------------
# Fast DDS profile (disable SHM)
# -----------------------------
RUN mkdir -p /root/.ros
COPY config/fastdds.xml /root/.ros/fastdds.xml

RUN chmod +x /entrypoint.sh

# Fix Python imports
ENV PYTHONPATH=/root/ros2_ws:$PYTHONPATH

# Build workspace
RUN /bin/bash -lc "source /opt/ros/kilted/setup.bash && colcon build --symlink-install || true"

# Source workspace in entrypoint to fix Python imports
ENV PYTHONPATH=/root/ros2_ws/install:$PYTHONPATH

# Entrypoint and default CMD
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "step_motor_service", "step_motor_service"]
