#!/bin/bash
# ros_entrypoint.sh for LiDAR node

# Exit on error
set -e

# Source ROS 2 Kilted setup
source /opt/ros/kilted/setup.bash

# Source the workspace (so the ldlidar_ros2 package is found)
if [ -f /root/ldlidar_ws/install/setup.bash ]; then
    source /root/ldlidar_ws/install/setup.bash
fi

# Execute any command passed to the container
exec "$@"