#!/bin/bash
# Generic ROS 2 entrypoint

set -e

# Source ROS 2
source /opt/ros/kilted/setup.bash

# Source workspace if it exists
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi

# Run Python service immediately
python3 /root/ros2_ws/src/run_service.py

# Continue with default command
exec "$@"
