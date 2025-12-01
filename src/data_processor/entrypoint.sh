#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/kilted/setup.bash

# Source workspace if it exists
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi


#export ROS_DISCOVERY_SERVER=discovery_server:11811

export ROS_DOMAIN_ID=0

# Run your Python script directly
python3 /root/ros2_ws/src/data_service.py


# Execute CMD (default is data_service node)
exec "$@"
