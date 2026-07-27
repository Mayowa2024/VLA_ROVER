#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/rover_vla_ws"

# ROS-generated setup scripts may read optional variables before defining them.
set +u
source "/opt/ros/${ROS_DISTRO:?ROS_DISTRO is not set}/setup.bash"
set -u
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
echo
echo "Built successfully."
echo "Run: source \"$WS/install/setup.bash\""
