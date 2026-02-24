#!/bin/bash
# ros_entrypoint.sh for Step motor node

# Exit on error
set -e

# Source ROS 2 Kilted setup
source /opt/ros/kilted/setup.bash

# Source the workspace (so scanner_pkg and domain modules are found)
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID=0

# -----------------------------
# Explicit middleware safety
# -----------------------------
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml

# Run your Python script directly
python3 /root/ros2_ws/src/step_motor/step_motor_service.py

# Execute any command passed to the container
exec "$@"
