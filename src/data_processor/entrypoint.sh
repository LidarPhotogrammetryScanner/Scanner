#!/bin/bash
# ros_entrypoint.sh for data node

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

# Ensure /output exists and is writable
OUTPUT_DIR="/output"
mkdir -p "$OUTPUT_DIR"
chmod 777 "$OUTPUT_DIR"

echo "Output directory: $OUTPUT_DIR"

# Run your Python script directly
python3 /root/ros2_ws/src/data_processor/data_service.py

# Execute any command passed to the container
exec "$@"
