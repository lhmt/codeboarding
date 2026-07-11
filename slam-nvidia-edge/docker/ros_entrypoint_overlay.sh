#!/bin/bash
# Sources ROS + the workspace overlay, then execs the container command.
set -e
source /opt/ros/jazzy/setup.bash
source /workspace/slam-nvidia-edge/ros2_ws/install/setup.bash
exec "$@"
