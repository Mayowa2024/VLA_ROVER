#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/rover_vla_ws"

# ROS-generated setup scripts may read optional variables before defining them.
set +u
source "/opt/ros/${ROS_DISTRO:?ROS_DISTRO is not set}/setup.bash"
source "$WS/install/setup.bash"
set -u

INSTRUCTION="${1:-Drive to the blue cone}"

ros2 launch rover_vla_core software_only.launch.py \
  instruction:="$INSTRUCTION"
