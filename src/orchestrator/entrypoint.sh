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

#export ROS_DISCOVERY_SERVER=discovery_server:11811

export ROS_DOMAIN_ID=0

# Run your Python script directly
python3 /root/ros2_ws/src/orchestrator_service.py

# Execute any command passed to the container
exec "$@"