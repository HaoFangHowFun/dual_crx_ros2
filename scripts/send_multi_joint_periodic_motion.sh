#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/codex_env.sh"

ARM="right"
PERIOD="4.0"
CYCLES="3"
SAMPLES_PER_CYCLE="12"
CONFIRMED=0
ALLOW_LARGE_AMPLITUDE=0
JOINT_MOTIONS=()

usage() {
  cat <<'EOF'
Usage: send_multi_joint_periodic_motion.sh \
         [--arm left|right|both] \
         --joint-motion JOINT:AMPLITUDE_RAD[:PHASE_DEG] \
         [--joint-motion JOINT:AMPLITUDE_RAD[:PHASE_DEG] ...] \
         [--joint-motion-deg JOINT:AMPLITUDE_DEG[:PHASE_DEG] ...] \
         [--period SECONDS] \
         [--cycles COUNT] \
         [--samples-per-cycle COUNT] \
         [--allow-large-amplitude] \
         --yes-i-understand

Builds bounded periodic trajectories around the live joint state.

Joint naming rules:
  With --arm left or --arm right:
    JOINT may be J1..J6 or fully qualified like left_J3 / right_J3.
  With --arm both:
    J1..J6 applies the same motion to both arms.
    left_J1..left_J6 or right_J1..right_J6 applies only to that arm.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --arm)
      ARM="${2:?--arm requires left, right, or both}"
      shift 2
      ;;
    --joint-motion)
      JOINT_MOTIONS+=("${2:?--joint-motion requires JOINT:AMPLITUDE[:PHASE_DEG]}")
      shift 2
      ;;
    --joint-motion-deg)
      JOINT_MOTIONS+=("$(
        python3 - <<'PY' "${2:?--joint-motion-deg requires JOINT:AMPLITUDE_DEG[:PHASE_DEG]}"
import math
import sys

parts = sys.argv[1].split(":")
if len(parts) not in (2, 3):
    raise SystemExit("Expected JOINT:AMPLITUDE_DEG[:PHASE_DEG]")
joint = parts[0]
amplitude_rad = float(parts[1]) * math.pi / 180.0
phase_deg = parts[2] if len(parts) == 3 else "0"
print(f"{joint}:{amplitude_rad!r}:{phase_deg}")
PY
      )")
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

case "$ARM" in
  left|right|both) ;;
  *)
    echo "Unsupported arm: $ARM" >&2
    exit 2
    ;;
esac

if [ "${#JOINT_MOTIONS[@]}" -eq 0 ]; then
  echo "At least one --joint-motion or --joint-motion-deg entry is required." >&2
  exit 2
fi

if [ "$CONFIRMED" -ne 1 ]; then
  echo "Refusing to send a physical motion command without --yes-i-understand." >&2
  exit 2
fi

declare -A STATUS_MSG_BY_ARM
declare -A JOINT_STATE_MSG_BY_ARM
declare -A PAYLOAD_BY_ARM

for arm_name in left right; do
  if [ "$ARM" != "both" ] && [ "$ARM" != "$arm_name" ]; then
    continue
  fi
  namespace="/${arm_name}_arm"
  STATUS_MSG_BY_ARM["$arm_name"]="$(ros2 topic echo --once ${namespace}/fanuc_gpio_controller/robot_status fanuc_msgs/msg/RobotStatus)"
  JOINT_STATE_MSG_BY_ARM["$arm_name"]="$(ros2 topic echo --once ${namespace}/joint_states sensor_msgs/msg/JointState)"
done

JOINT_MOTIONS_JOINED="$(printf '%s\n' "${JOINT_MOTIONS[@]}")"

for arm_name in left right; do
  if [ "$ARM" != "both" ] && [ "$ARM" != "$arm_name" ]; then
    continue
  fi

  export STATUS_MSG="${STATUS_MSG_BY_ARM[$arm_name]}"
  export JOINT_STATE_MSG="${JOINT_STATE_MSG_BY_ARM[$arm_name]}"
  export TARGET_ARM="$arm_name" ARM_SCOPE="$ARM" JOINT_MOTIONS_JOINED PERIOD CYCLES SAMPLES_PER_CYCLE ALLOW_LARGE_AMPLITUDE

  payload="$(
python3 - <<'PY'
import ast
import math
import os
import sys

status_msg = os.environ["STATUS_MSG"]
joint_state_msg = os.environ["JOINT_STATE_MSG"]
target_arm = os.environ["TARGET_ARM"]
arm_scope = os.environ["ARM_SCOPE"]
joint_motions_raw = [line.strip() for line in os.environ["JOINT_MOTIONS_JOINED"].splitlines() if line.strip()]
period = float(os.environ["PERIOD"])
cycles = int(os.environ["CYCLES"])
samples_per_cycle = int(os.environ["SAMPLES_PER_CYCLE"])
allow_large_amplitude = os.environ["ALLOW_LARGE_AMPLITUDE"] == "1"

ordered_joints = [f"{target_arm}_J{i}" for i in range(1, 7)]
max_amplitude = math.pi / 18.0 if allow_large_amplitude else 0.05

if period < 2.0:
    print("Refusing period below 2.0 s for this script.", file=sys.stderr)
    sys.exit(2)
if cycles < 1 or cycles > 20:
    print("Cycles must be between 1 and 20.", file=sys.stderr)
    sys.exit(2)
if samples_per_cycle < 4 or samples_per_cycle > 100:
    print("samples-per-cycle must be between 4 and 100.", file=sys.stderr)
    sys.exit(2)

motions = {}
for raw in joint_motions_raw:
    parts = raw.split(":")
    if len(parts) not in (2, 3):
        print(f"Invalid joint motion entry: {raw!r}", file=sys.stderr)
        sys.exit(2)

    joint_token = parts[0]
    amplitude = float(parts[1])
    phase_deg = float(parts[2]) if len(parts) == 3 else 0.0

    if amplitude <= 0.0:
        print(f"Amplitude must be positive for joint {joint_token}.", file=sys.stderr)
        sys.exit(2)
    if amplitude > max_amplitude:
        if allow_large_amplitude:
            print(f"Refusing amplitude above 10 deg for joint {joint_token}.", file=sys.stderr)
        else:
            print(f"Refusing amplitude above 0.05 rad for joint {joint_token} without --allow-large-amplitude.", file=sys.stderr)
        sys.exit(2)

    if joint_token.startswith("left_") or joint_token.startswith("right_"):
        joint_arm, suffix = joint_token.split("_", 1)
        suffix = suffix.strip()
        if suffix not in {f"J{i}" for i in range(1, 7)}:
            print(f"Unsupported joint: {joint_token}", file=sys.stderr)
            sys.exit(2)
        if arm_scope != "both" and joint_arm != target_arm:
            print(f"Joint {joint_token} does not match --arm {arm_scope}.", file=sys.stderr)
            sys.exit(2)
        if joint_arm != target_arm:
            continue
        resolved_joint = f"{joint_arm}_{suffix}"
    else:
        suffix = joint_token.strip()
        if suffix not in {f"J{i}" for i in range(1, 7)}:
            print(f"Unsupported joint: {joint_token}", file=sys.stderr)
            sys.exit(2)
        resolved_joint = f"{target_arm}_{suffix}"

    if resolved_joint in motions:
        print(f"Duplicate joint motion entry for {resolved_joint}.", file=sys.stderr)
        sys.exit(2)
    motions[resolved_joint] = {
        "amplitude": amplitude,
        "phase_rad": math.radians(phase_deg),
    }

if not motions:
    print("")
    raise SystemExit(0)

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
        print(f"Robot status check failed for {target_arm}: {key}={actual!r}, expected {expected!r}", file=sys.stderr)
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
    print(f"Failed to parse /{target_arm}_arm/joint_states", file=sys.stderr)
    sys.exit(2)
if len(names) != len(positions):
    print("Joint state name/position length mismatch", file=sys.stderr)
    sys.exit(2)

state = dict(zip(names, positions))
missing = [name for name in ordered_joints if name not in state]
if missing:
    print(f"Missing joints in current state for {target_arm}: {missing}", file=sys.stderr)
    sys.exit(2)

base = [state[name] for name in ordered_joints]
points = []
total_samples = cycles * samples_per_cycle

points.append(
    "{positions: [%s], time_from_start: {sec: 0, nanosec: 0}}"
    % ", ".join(f"{value:.12f}" for value in base)
)

for sample_index in range(1, total_samples + 1):
    t = sample_index * period / samples_per_cycle
    phase = 2.0 * math.pi * t / period
    target = list(base)
    for joint, motion in motions.items():
        joint_index = ordered_joints.index(joint)
        target[joint_index] += motion["amplitude"] * math.sin(phase + motion["phase_rad"])
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

  if [ -n "$payload" ]; then
    PAYLOAD_BY_ARM["$arm_name"]="$payload"
  fi
done

if [ "${#PAYLOAD_BY_ARM[@]}" -eq 0 ]; then
  echo "No arm received any resolved joint motions from the given arguments." >&2
  exit 2
fi

echo "Sending multi-joint periodic trajectory:"
echo "  arm=${ARM}"
printf '  %s\n' "${JOINT_MOTIONS[@]}"
echo "  period=${PERIOD} s"
echo "  cycles=${CYCLES}"
echo "  samples_per_cycle=${SAMPLES_PER_CYCLE}"

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
