#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/kilted/setup.bash
# Source our workspace
source /root/ldlidar_ws/install/local_setup.bash

exec "$@"
