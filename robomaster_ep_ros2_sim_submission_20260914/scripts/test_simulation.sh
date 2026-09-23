#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source /home/nvidia/gz_ros2_control_ws/install/setup.bash
source install/setup.bash
set -u
mkdir -p log
robot_urdf="$(mktemp /tmp/robomaster_ep_test.XXXXXX.urdf)"
launch_log="$(mktemp /tmp/robomaster_ep_launch.XXXXXX.log)"
sim_pid=""
cleanup() {
  if [[ -n "$sim_pid" ]] && kill -0 "$sim_pid" 2>/dev/null; then
    kill -INT -- "-$sim_pid" 2>/dev/null || true
    for _ in {1..30}; do kill -0 "$sim_pid" 2>/dev/null || break; sleep 0.2; done
    kill -TERM -- "-$sim_pid" 2>/dev/null || true
  fi
  rm -f "$robot_urdf"
}
trap cleanup EXIT INT TERM
xacro src/robomaster_ep_description/urdf/robomaster_ep.urdf.xacro \
  use_gazebo:=false initial_positions_file:="$PWD/src/robomaster_ep_description/config/initial_positions.yaml" \
  controllers_file:="$PWD/src/robomaster_ep_gazebo/config/ros2_controllers.yaml" > "$robot_urdf"
check_urdf "$robot_urdf"
python3 -m compileall -q src/robomaster_ep_description/launch src/robomaster_ep_moveit_config/launch \
  src/robomaster_ep_gazebo/launch src/robomaster_ep_demos/robomaster_ep_demos
setsid ros2 launch robomaster_ep_gazebo gazebo_moveit.launch.py headless:=true >"$launch_log" 2>&1 &
sim_pid=$!
timeout 40 ros2 run robomaster_ep_demos verify_simulation
timeout 180 ros2 run robomaster_ep_demos pick_place --ros-args -p cycles:=2
if grep -Ei 'nan|segmentation fault|process has died|PICK_PLACE_FAIL' "$launch_log"; then
  echo "Fatal pattern found in launch log: $launch_log" >&2
  exit 1
fi
echo "ALL_TESTS_PASS"
