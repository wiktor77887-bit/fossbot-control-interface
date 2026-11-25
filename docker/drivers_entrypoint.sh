#!/bin/bash
# Sources ROS and whichever driver workspace the base image has.
set -e
source /opt/ros/jazzy/setup.bash
for ws in /fossbot_ros2/ws_fossbot /ws; do   # simulator, bridge
  if [ -f "$ws/install/setup.bash" ]; then
    source "$ws/install/setup.bash"
  fi
done
# Qt (Gazebo, bridge dashboard) warns on every start without a private runtime dir.
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
mkdir -p -m 700 "$XDG_RUNTIME_DIR"
exec "$@"
