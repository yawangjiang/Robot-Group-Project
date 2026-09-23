#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source /home/nvidia/gz_ros2_control_ws/install/setup.bash
set -u
colcon build --symlink-install --event-handlers console_direct+
