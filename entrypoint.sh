#!/bin/bash
# entrypoint.sh

# Source ROS2 installation
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Start a bash shell if no command is provided
if [ $# -eq 0 ]; then
    exec bash
else
    exec "$@"
fi

