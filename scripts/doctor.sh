#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/codex_env.sh"

echo "ros2=$(command -v ros2)"
echo "colcon=$(command -v colcon)"
echo "ROS_DISTRO=${ROS_DISTRO:-}"
echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-}"
