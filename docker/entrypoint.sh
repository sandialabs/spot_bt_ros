#!/bin/bash
# Entrypoint for ROS Spot Docker containers

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source the spot_ros2 workspace if built
if [ -f /spot_ws/install/setup.bash ]
then
  source /spot_ws/install/local_setup.bash
  echo "Sourced spot_ros2 workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"