#!/bin/bash
set -e

# -----------------------------
# Source ROS 2
# -----------------------------
source /opt/ros/kilted/setup.bash

if [ -f /root/ldlidar_ws/install/local_setup.bash ]; then
    source /root/ldlidar_ws/install/local_setup.bash
fi

# -----------------------------
# DDS / ROS2 safety cleanup
# -----------------------------
rm -rf /root/.ros/log/* || true
rm -rf /dev/shm/* || true

export ROS_DOMAIN_ID=0

# -----------------------------
# Explicit middleware safety
# -----------------------------
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml

# Helpful for debugging (optional)
export RCUTILS_LOGGING_BUFFERED_STREAM=1

# -----------------------------
# Execute command
# -----------------------------
exec "$@"
