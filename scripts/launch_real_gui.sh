#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$SCRIPT_DIR/codex_env.sh"

LEFT_IP="${DUAL_CRX_LEFT_IP:-192.168.2.100}"
RIGHT_IP="${DUAL_CRX_RIGHT_IP:-192.168.1.100}"
PLACEMENT_FILE="${DUAL_CRX_PLACEMENT_FILE:-$WORKSPACE_DIR/src/dual_crx_description/config/robot_placement_physical.yaml}"
LAUNCH_RVIZ=false

usage() {
  echo "Usage: $0 [--left-ip ADDRESS] [--right-ip ADDRESS] [--placement-file PATH] [--launch-rviz]"
  echo
  echo "Starts both physical arms, MoveIt, and the unified control server for GUI use."
  echo "Physical GUI startup always begins in connection-only mode."
  echo "Use the GUI's per-arm Enable Motion button after the live state is clear."
}

EXTRA_ARGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --left-ip)
      LEFT_IP="${2:?--left-ip requires an address}"
      shift 2
      ;;
    --right-ip)
      RIGHT_IP="${2:?--right-ip requires an address}"
      shift 2
      ;;
    --placement-file)
      PLACEMENT_FILE="${2:?--placement-file requires a path}"
      shift 2
      ;;
    --launch-rviz)
      LAUNCH_RVIZ=true
      shift
      ;;
    --)
      shift
      EXTRA_ARGS=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

cd "$WORKSPACE_DIR"

if [ ! -f "$PLACEMENT_FILE" ]; then
  echo "Physical placement file not found: $PLACEMENT_FILE" >&2
  echo "Collect, solve, and activate it with scripts/table_calibration_tool.py first." >&2
  exit 2
fi

echo "Starting GUI physical stack in CONNECTION-ONLY mode."

exec ros2 launch dual_crx_bringup dual_crx.launch.py \
  use_mock:=false \
  robot_placement_file:="$PLACEMENT_FILE" \
  require_valid_placement:=true \
  launch_rviz:="$LAUNCH_RVIZ" \
  launch_control_server:=true \
  allow_physical_control:=true \
  left_robot_ip:="$LEFT_IP" \
  right_robot_ip:="$RIGHT_IP" \
  left_motion_control:=0 \
  right_motion_control:=0 \
  "${EXTRA_ARGS[@]}"
