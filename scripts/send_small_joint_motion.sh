#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/codex_env.sh"

ARM="right"
JOINT="J6"
DELTA_RAD="0.02"
DURATION="3.0"
CONFIRMED=0
ALLOW_LARGE_DELTA=0

usage() {
  cat <<'EOF'
Usage: send_small_joint_motion.sh [--arm left|right|both]
                                 [--joint J1|...|J6|left_J1|...|right_J6]
                                 [--delta RADIANS | --delta-deg DEGREES]
                                 [--duration SECONDS]
                                 [--allow-large-delta]
                                 --yes-i-understand

Sends a small single-joint trajectory using the selected arm's live joint state
as the start point. With --arm both, the same joint index moves on both arms.

Defaults:
  --arm right
  --joint J6
  --delta 0.02
  --duration 3.0

Conservative limit:
  Without --allow-large-delta, |delta| must be <= 0.05 rad (~2.86 deg).
  With    --allow-large-delta, |delta| must be <= 0.174533 rad (10 deg).
EOF
}

normalize_joint_suffix() {
  case "$1" in
    J1|left_J1|right_J1) printf 'J1\n' ;;
    J2|left_J2|right_J2) printf 'J2\n' ;;
    J3|left_J3|right_J3) printf 'J3\n' ;;
    J4|left_J4|right_J4) printf 'J4\n' ;;
    J5|left_J5|right_J5) printf 'J5\n' ;;
    J6|left_J6|right_J6) printf 'J6\n' ;;
    *) return 1 ;;
  esac
}

active_arms() {
  case "$1" in
    left) printf 'left\n' ;;
    right) printf 'right\n' ;;
    both) printf 'left\nright\n' ;;
    *) return 1 ;;
  esac
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --arm)
      ARM="${2:?--arm requires left, right, or both}"
      shift 2
      ;;
    --joint)
      JOINT="${2:?--joint requires a value}"
      shift 2
      ;;
    --delta)
      DELTA_RAD="${2:?--delta requires a value}"
      shift 2
      ;;
    --delta-deg)
      DELTA_RAD="$(
        python3 - <<'PY' "$2"
import math
import sys
print(repr(float(sys.argv[1]) * math.pi / 180.0))
PY
      )"
      shift 2
      ;;
    --duration)
      DURATION="${2:?--duration requires a value}"
      shift 2
      ;;
    --allow-large-delta)
      ALLOW_LARGE_DELTA=1
      shift
      ;;
    --yes-i-understand)
      CONFIRMED=1
      shift
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
    echo "Unsupported arm: $ARM" >&2
    exit 2
    ;;
esac

JOINT_SUFFIX="$(normalize_joint_suffix "$JOINT" || true)"
if [ -z "$JOINT_SUFFIX" ]; then
  echo "Unsupported joint: $JOINT" >&2
  exit 2
fi

if [ "$CONFIRMED" -ne 1 ]; then
  echo "Refusing to send a physical motion command without --yes-i-understand." >&2
  exit 2
fi

declare -A PAYLOAD_BY_ARM

while IFS= read -r arm_name; do
  [ -n "$arm_name" ] || continue
  namespace="/${arm_name}_arm"
  prefix="${arm_name}_"
  resolved_joint="${prefix}${JOINT_SUFFIX}"
  STATUS_MSG="$(ros2 topic echo --once ${namespace}/fanuc_gpio_controller/robot_status fanuc_msgs/msg/RobotStatus)"
  JOINT_STATE_MSG="$(ros2 topic echo --once ${namespace}/joint_states sensor_msgs/msg/JointState)"

  export STATUS_MSG JOINT_STATE_MSG DELTA_RAD DURATION ALLOW_LARGE_DELTA ARM_NAME="$arm_name" RESOLVED_JOINT="$resolved_joint"

  PAYLOAD_BY_ARM["$arm_name"]="$(
python3 - <<'PY'
import ast
import math
import os
import sys

status_msg = os.environ["STATUS_MSG"]
joint_state_msg = os.environ["JOINT_STATE_MSG"]
delta = float(os.environ["DELTA_RAD"])
duration = float(os.environ["DURATION"])
allow_large_delta = os.environ["ALLOW_LARGE_DELTA"] == "1"
resolved_joint = os.environ["RESOLVED_JOINT"]
arm_name = os.environ["ARM_NAME"]

ordered_joints = [f"{arm_name}_J{i}" for i in range(1, 7)]
max_delta = math.pi / 18.0 if allow_large_delta else 0.05

if abs(delta) > max_delta:
    if allow_large_delta:
        print("Refusing delta magnitude above 10 deg for this script.", file=sys.stderr)
    else:
        print("Refusing delta magnitude above 0.05 rad for this script without --allow-large-delta.", file=sys.stderr)
    sys.exit(2)
if duration < 1.0:
    print("Refusing duration below 1.0 s for this script.", file=sys.stderr)
    sys.exit(2)
if resolved_joint not in ordered_joints:
    print(f"Joint {resolved_joint} is not valid for arm {arm_name}.", file=sys.stderr)
    sys.exit(2)

status = {}
for raw_line in status_msg.splitlines():
    line = raw_line.strip()
    if ":" not in line:
        continue
    key, value = [part.strip() for part in line.split(":", 1)]
    status[key] = value

required_status = {
    "in_error": "false",
    "tp_enabled": "false",
    "e_stopped": "false",
    "motion_possible": "true",
}

for key, expected in required_status.items():
    actual = status.get(key)
    if actual != expected:
        print(f"Robot status check failed for {arm_name}: {key}={actual!r}, expected {expected!r}", file=sys.stderr)
        sys.exit(2)

names = None
positions = None
current_section = None

for raw_line in joint_state_msg.splitlines():
    stripped = raw_line.strip()
    if stripped == "name:":
        current_section = "name"
        continue
    if stripped == "position:":
        current_section = "position"
        continue
    if stripped.endswith(":") and stripped not in {"name:", "position:"}:
        current_section = None
        continue
    if current_section == "name" and stripped.startswith("- "):
        names = names or []
        names.append(stripped[2:].strip())
    elif current_section == "position" and stripped.startswith("- "):
        positions = positions or []
        positions.append(float(ast.literal_eval(stripped[2:].strip())))

if not names or not positions:
    print(f"Failed to parse /{arm_name}_arm/joint_states", file=sys.stderr)
    sys.exit(2)
if len(names) != len(positions):
    print("Joint state name/position length mismatch", file=sys.stderr)
    sys.exit(2)

state = dict(zip(names, positions))
missing = [name for name in ordered_joints if name not in state]
if missing:
    print(f"Missing joints in current state for {arm_name}: {missing}", file=sys.stderr)
    sys.exit(2)

target = [state[name] for name in ordered_joints]
target[ordered_joints.index(resolved_joint)] += delta

goal = (
    "{"
    " trajectory: {"
    f" joint_names: [{', '.join(ordered_joints)}],"
    " points: ["
    f"{{positions: [{', '.join(f'{value:.12f}' for value in target)}], time_from_start: {{sec: {duration:.3f}}}}}"
    " ]"
    " }"
    "}"
)

print(goal)
PY
  )"
done < <(active_arms "$ARM")

echo "Sending small trajectory:"
echo "  arm=${ARM}"
echo "  joint=${JOINT_SUFFIX}"
echo "  delta=${DELTA_RAD} rad"
echo "  duration=${DURATION} s"

pids=()
for arm_name in "${!PAYLOAD_BY_ARM[@]}"; do
  ros2 action send_goal \
    "/${arm_name}_arm/joint_trajectory_controller/follow_joint_trajectory" \
    control_msgs/action/FollowJointTrajectory \
    "${PAYLOAD_BY_ARM[$arm_name]}" &
  pids+=("$!")
done

status=0
for pid in "${pids[@]}"; do
  if ! wait "$pid"; then
    status=1
  fi
done

exit "$status"
