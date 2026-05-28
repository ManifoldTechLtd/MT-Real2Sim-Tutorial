#!/bin/bash
set -e

# Source ROS
source /opt/ros/noetic/setup.bash

# Source workspace (if built)
if [ -f /catkin_ws/devel/setup.bash ]; then
    source /catkin_ws/devel/setup.bash
fi

# Execute the passed command
exec "$@"
