#!/bin/bash
# ros_entrypoint.sh for Orchestrator node

# Exit on error
set -e

# Source ROS 2 Kilted setup
source /opt/ros/kilted/setup.bash

# Source the workspace (so scanner_pkg and domain modules are found)
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID=0

# Run your Python script directly
python3 /root/ros2_ws/src/orchestrator/orchestrator_service.py

# Execute any command passed to the container
exec "$@"