#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$SCRIPT_DIR/codex_env.sh"

ARM=""
LEFT_IP="${DUAL_CRX_LEFT_IP:-192.168.2.100}"
RIGHT_IP="${DUAL_CRX_RIGHT_IP:-192.168.1.100}"
PLACEMENT_FILE="${DUAL_CRX_PLACEMENT_FILE:-$WORKSPACE_DIR/src/dual_crx_description/config/robot_placement_physical.yaml}"
MOTION_CONTROL=0
CONFIRMED=0

usage() {
  echo "Usage: $0 --arm left|right|both [--left-ip ADDRESS] [--right-ip ADDRESS]"
  echo "          [--placement-file PATH]"
  echo "          [--enable-motion --yes-i-understand] [-- ROS_LAUNCH_ARGUMENTS...]"
  echo
  echo "Default mode is connection-only (motion_control=0)."
}

EXTRA_ARGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --arm)
      ARM="${2:?--arm requires left, right, or both}"
      shift 2
      ;;
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
    --enable-motion)
      MOTION_CONTROL=1
      shift
      ;;
    --yes-i-understand)
      CONFIRMED=1
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

case "$ARM" in
  left|right|both) ;;
  *)
    echo "--arm left, --arm right, or --arm both is required" >&2
    exit 2
    ;;
esac

if [ "$MOTION_CONTROL" -eq 1 ] && [ "$CONFIRMED" -ne 1 ]; then
  echo "Refusing to request robot motion authority without --yes-i-understand." >&2
  echo "First run connection-only, verify robot status and complete the safety checklist." >&2
  exit 2
fi

cd "$WORKSPACE_DIR"

if [ "$MOTION_CONTROL" -eq 0 ]; then
  echo "Starting $ARM in CONNECTION-ONLY mode (motion_control=0)."
else
  echo "Starting $ARM with MOTION AUTHORITY requested (motion_control=1)."
fi

if [ "$ARM" = "both" ]; then
  if [ ! -f "$PLACEMENT_FILE" ]; then
    echo "Physical placement file not found: $PLACEMENT_FILE" >&2
    echo "Calibrate and activate it before a combined physical launch." >&2
    exit 2
  fi
  exec ros2 launch dual_crx_bringup dual_crx.launch.py \
    use_mock:=false \
    robot_placement_file:="$PLACEMENT_FILE" \
    require_valid_placement:=true \
    left_robot_ip:="$LEFT_IP" \
    right_robot_ip:="$RIGHT_IP" \
    left_motion_control:="$MOTION_CONTROL" \
    right_motion_control:="$MOTION_CONTROL" \
    "${EXTRA_ARGS[@]}"
fi

if [ "$ARM" = "left" ]; then
  ROBOT_IP="$LEFT_IP"
  ARM_PREFIX="left_"
  ARM_NAMESPACE="left_arm"
  ARM_CHILD_LINK="left_ee_mount"
  ARM_Y="0.3"
else
  ROBOT_IP="$RIGHT_IP"
  ARM_PREFIX="right_"
  ARM_NAMESPACE="right_arm"
  ARM_CHILD_LINK="right_ee_mount"
  ARM_Y="-0.3"
fi

exec ros2 launch fanuc_hardware_interface fanuc_physical_control.launch.py \
  robot_ip:="$ROBOT_IP" \
  robot_model:=crx5ia \
  robot_series:=crx \
  gpio_config_path:=config/example_gpio_config_small.yaml \
  motion_control:="$MOTION_CONTROL" \
  prefix:="$ARM_PREFIX" \
  namespace:="$ARM_NAMESPACE" \
  child_link:="$ARM_CHILD_LINK" \
  origin_y:="$ARM_Y" \
  launch_rviz:=false \
  "${EXTRA_ARGS[@]}"
