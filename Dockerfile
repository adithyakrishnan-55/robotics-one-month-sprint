# Use official ROS2 Humble desktop image (Ubuntu 22.04 inside container)
FROM osrf/ros:humble-desktop

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install tools for workspace management
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Set workspace directory
WORKDIR /ros2_ws
RUN mkdir -p /ros2_s/src

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

