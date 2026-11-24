#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
# Qt (matplotlib backend) warns on every start without a private runtime dir.
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
mkdir -p -m 700 "$XDG_RUNTIME_DIR"
exec "$@"
