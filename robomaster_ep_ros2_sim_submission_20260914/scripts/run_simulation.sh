#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source /home/nvidia/gz_ros2_control_ws/install/setup.bash
source install/setup.bash
set -u
exec ros2 launch robomaster_ep_gazebo gazebo_moveit.launch.py "$@"
