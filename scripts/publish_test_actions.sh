#!/usr/bin/env bash
set -euo pipefail

TOPIC="${1:-/vla/raw_cmd_vel}"

echo "Publishing a safe software-only command sequence on $TOPIC."
echo "No physical hardware should be connected to the mock stack."

ros2 topic pub --once "$TOPIC" geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.20}}"
sleep 1
ros2 topic pub --once "$TOPIC" geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: -0.20}}"
sleep 1
ros2 topic pub --once "$TOPIC" geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
