#!/bin/bash

# Allow Docker container to use your X server
xhost +local:root

# Run the ROS2 Humble container
docker run -dit  \
  --network host \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume ~/ros2_humble_docker/ros2_ws:/ros2_ws \
  --volume ~/.ssh:/root/.ssh \
  --name ros2_humble_gui \
  ros2_humble_dev
