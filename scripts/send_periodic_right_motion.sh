#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/codex_env.sh"

JOINT="right_J6"
AMPLITUDE_RAD="0.02"
PERIOD="4.0"
CYCLES="3"
SAMPLES_PER_CYCLE="12"
CONFIRMED=0
ALLOW_LARGE_AMPLITUDE=0

usage() {
  cat <<'EOF'
Usage: send_periodic_right_motion.sh [--joint right_J1|right_J2|right_J3|right_J4|right_J5|right_J6]
                                    [--amplitude RADIANS | --amplitude-deg DEGREES]
                                    [--period SECONDS]
                                    [--cycles COUNT]
                                    [--samples-per-cycle COUNT]
                                    [--allow-large-amplitude]
                                    --yes-i-understand

Sends a bounded periodic single-joint trajectory to
/right_arm/joint_trajectory_controller using the arm's current joint state as
the oscillation center.

Defaults:
  --joint right_J6
  --amplitude 0.02
  --period 4.0
  --cycles 3
  --samples-per-cycle 12

Conservative limit:
  Without --allow-large-amplitude, amplitude must be <= 0.05 rad (~2.86 deg).
  With    --allow-large-amplitude, amplitude must be <= 0.174533 rad (10 deg).
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --joint)
      JOINT="${2:?--joint requires a value}"
      shift 2
      ;;
    --amplitude)
      AMPLITUDE_RAD="${2:?--amplitude requires a value}"
      shift 2
      ;;
    --amplitude-deg)
      AMPLITUDE_RAD="$(
        python3 - <<'PY' "$2"
import math
import sys
print(repr(float(sys.argv[1]) * math.pi / 180.0))
PY
      )"
      shift 2
      ;;
    --period)
      PERIOD="${2:?--period requires a value}"
      shift 2
      ;;
    --cycles)
      CYCLES="${2:?--cycles requires a value}"
      shift 2
      ;;
    --samples-per-cycle)
      SAMPLES_PER_CYCLE="${2:?--samples-per-cycle requires a value}"
      shift 2
      ;;
    --allow-large-amplitude)
      ALLOW_LARGE_AMPLITUDE=1
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

export STATUS_MSG JOINT_STATE_MSG JOINT AMPLITUDE_RAD PERIOD CYCLES SAMPLES_PER_CYCLE ALLOW_LARGE_AMPLITUDE

GOAL_PAYLOAD="$(
python3 - <<'PY'
import ast
import math
import os
import sys

status_msg = os.environ["STATUS_MSG"]
joint_state_msg = os.environ["JOINT_STATE_MSG"]
joint = os.environ["JOINT"]
amplitude = float(os.environ["AMPLITUDE_RAD"])
period = float(os.environ["PERIOD"])
cycles = int(os.environ["CYCLES"])
samples_per_cycle = int(os.environ["SAMPLES_PER_CYCLE"])
allow_large_amplitude = os.environ["ALLOW_LARGE_AMPLITUDE"] == "1"

max_amplitude = math.pi / 18.0 if allow_large_amplitude else 0.05
if amplitude <= 0.0:
    print("Amplitude must be positive.", file=sys.stderr)
    sys.exit(2)
if amplitude > max_amplitude:
    if allow_large_amplitude:
        print("Refusing amplitude above 10 deg for this script.", file=sys.stderr)
    else:
        print("Refusing amplitude above 0.05 rad for this script without --allow-large-amplitude.", file=sys.stderr)
    sys.exit(2)
if period < 2.0:
    print("Refusing period below 2.0 s for this script.", file=sys.stderr)
    sys.exit(2)
if cycles < 1 or cycles > 20:
    print("Cycles must be between 1 and 20.", file=sys.stderr)
    sys.exit(2)
if samples_per_cycle < 4 or samples_per_cycle > 100:
    print("samples-per-cycle must be between 4 and 100.", file=sys.stderr)
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

base = [state[name] for name in ordered_joints]
joint_index = ordered_joints.index(joint)
points = []
total_samples = cycles * samples_per_cycle

for sample_index in range(total_samples + 1):
    t = sample_index * period / samples_per_cycle
    phase = 2.0 * math.pi * t / period
    target = list(base)
    target[joint_index] += amplitude * math.sin(phase)
    sec = int(t)
    nanosec = int(round((t - sec) * 1_000_000_000))
    if nanosec == 1_000_000_000:
        sec += 1
        nanosec = 0
    points.append(
        "{positions: [%s], time_from_start: {sec: %d, nanosec: %d}}"
        % (", ".join(f"{value:.12f}" for value in target), sec, nanosec)
    )

goal = (
    "{"
    " trajectory: {"
    f" joint_names: [{', '.join(ordered_joints)}],"
    f" points: [{', '.join(points)}]"
    " }"
    "}"
)

print(goal)
PY
)"

echo "Sending right-arm periodic trajectory:"
echo "  joint=${JOINT}"
echo "  amplitude=${AMPLITUDE_RAD} rad"
echo "  period=${PERIOD} s"
echo "  cycles=${CYCLES}"
echo "  samples_per_cycle=${SAMPLES_PER_CYCLE}"

ros2 action send_goal \
  /right_arm/joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "$GOAL_PAYLOAD"
