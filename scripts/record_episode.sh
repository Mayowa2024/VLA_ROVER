#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 4 ]]; then
  echo 'Usage:'
  echo '  record_episode.sh EPISODE_ID "INSTRUCTION" IMAGE_TOPIC ACTION_TOPIC [ODOM_TOPIC] [STATE_SOURCE]'
  echo
  echo 'Example without encoders:'
  echo '  ./record_episode.sh 0 "Drive to the blue cone" /image_raw /teleop/cmd_vel /odom previous_action'
  echo
  echo 'Example with encoder odometry:'
  echo '  ./record_episode.sh 1 "Drive to the red cone" /image_raw /teleop/cmd_vel /odom odom'
  exit 1
fi

EPISODE_ID="$1"
INSTRUCTION="$2"
IMAGE_TOPIC="$3"
ACTION_TOPIC="$4"
ODOM_TOPIC="${5:-/odom}"
STATE_SOURCE="${6:-odom}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/rover_vla_ws"

# ROS-generated setup scripts may read optional variables before defining them.
# Keep strict unset-variable checks for this script, but not while sourcing ROS.
set +u
source "/opt/ros/${ROS_DISTRO:?ROS_DISTRO is not set}/setup.bash"
source "$WS/install/setup.bash"
set -u

cleanup() {
  [[ -n "${INSTRUCTION_PID:-}" ]] && kill "$INSTRUCTION_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run rover_vla_core instruction_publisher --ros-args \
  -p instruction:="$INSTRUCTION" &
INSTRUCTION_PID=$!

sleep 1

ros2 run rover_vla_core frame_recorder --ros-args \
  -p episode_id:="$EPISODE_ID" \
  -p image_topic:="$IMAGE_TOPIC" \
  -p action_topic:="$ACTION_TOPIC" \
  -p odom_topic:="$ODOM_TOPIC" \
  -p state_source:="$STATE_SOURCE" \
  -p output_root:="$HOME/rover_vla_data/raw"
