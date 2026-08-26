#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/codex_env.sh"

JOINT="right_J6"
DELTA_RAD="0.02"
DURATION="3.0"
CONFIRMED=0
ALLOW_LARGE_DELTA=0

usage() {
  cat <<'EOF'
Usage: send_small_right_motion.sh [--joint right_J1|right_J2|right_J3|right_J4|right_J5|right_J6]
                                 [--delta RADIANS | --delta-deg DEGREES]
                                 [--duration SECONDS]
                                 [--allow-large-delta]
                                 --yes-i-understand

Sends a small single-joint trajectory to /right_arm/joint_trajectory_controller
using the arm's current joint state as the start point.

Defaults:
  --joint right_J6
  --delta 0.02
  --duration 3.0

Conservative limit:
  Without --allow-large-delta, |delta| must be <= 0.05 rad (~2.86 deg).
  With    --allow-large-delta, |delta| must be <= 0.174533 rad (10 deg).
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
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

case "$JOINT" in
  right_J1|right_J2|right_J3|right_J4|right_J5|right_J6) ;;
  *)
    echo "Unsupported joint: $JOINT" >&2
    exit 2
    ;;
esac

if [ "$CONFIRMED" -ne 1 ]; then
  echo "Refusing to send a physical motion command without --yes-i-understand." >&2
  exit 2
fi

STATUS_MSG="$(ros2 topic echo --once /right_arm/fanuc_gpio_controller/robot_status fanuc_msgs/msg/RobotStatus)"
JOINT_STATE_MSG="$(ros2 topic echo --once /right_arm/joint_states sensor_msgs/msg/JointState)"

export STATUS_MSG JOINT_STATE_MSG JOINT DELTA_RAD DURATION ALLOW_LARGE_DELTA

GOAL_PAYLOAD="$(
python3 - <<'PY'
import ast
import math
import os
import sys

status_msg = os.environ["STATUS_MSG"]
joint_state_msg = os.environ["JOINT_STATE_MSG"]
joint = os.environ["JOINT"]
delta = float(os.environ["DELTA_RAD"])
duration = float(os.environ["DURATION"])
allow_large_delta = os.environ["ALLOW_LARGE_DELTA"] == "1"

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
        print(f"Robot status check failed: {key}={actual!r}, expected {expected!r}", file=sys.stderr)
        sys.exit(2)

names = None
positions = None
current_section = None

for raw_line in joint_state_msg.splitlines():
    line = raw_line.rstrip()
    stripped = line.strip()
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
    print("Failed to parse /right_arm/joint_states", file=sys.stderr)
    sys.exit(2)

if len(names) != len(positions):
    print("Joint state name/position length mismatch", file=sys.stderr)
    sys.exit(2)

state = dict(zip(names, positions))
ordered_joints = ["right_J1", "right_J2", "right_J3", "right_J4", "right_J5", "right_J6"]

missing = [name for name in ordered_joints if name not in state]
if missing:
    print(f"Missing joints in current state: {missing}", file=sys.stderr)
    sys.exit(2)

target = [state[name] for name in ordered_joints]
target[ordered_joints.index(joint)] += delta

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

echo "Sending right-arm trajectory:"
echo "  joint=${JOINT}"
echo "  delta=${DELTA_RAD} rad"
echo "  duration=${DURATION} s"

ros2 action send_goal \
  /right_arm/joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "$GOAL_PAYLOAD"
