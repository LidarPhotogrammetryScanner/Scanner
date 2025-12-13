#!/bin/bash
set -e

# Source ROS2
source /opt/ros/kilted/setup.bash

# Source workspace if exists
if [ -f /root/ldlidar_ws/install/local_setup.bash ]; then
    source /root/ldlidar_ws/install/local_setup.bash
fi

# --- CLEAN STALE ROS2 / DDS STATE ---
# Remove old ROS2 logs
rm -rf /root/.ros/log/*

# Remove stale CycloneDDS shared memory
rm -rf /dev/shm/*

# --- END CLEANUP ---

# Execute the passed command (launch your nodes)
exec "$@"
