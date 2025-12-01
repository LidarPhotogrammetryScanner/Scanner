#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/kilted/setup.bash

# Source workspace if it exists
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi

# Execute CMD (default is data_service node)
exec "$@"
