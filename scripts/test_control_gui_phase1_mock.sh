#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ARTIFACT_DIR="${PHASE1_ARTIFACT_DIR:-$(mktemp -d /tmp/dual_crx_phase1_XXXXXX)}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + $$ % 100))}"
export ROS_DOMAIN_ID
export ROS_HOME="$ARTIFACT_DIR/ros_home"
export ROS_LOG_DIR="$ARTIFACT_DIR/ros_logs"
export QT_QPA_PLATFORM=offscreen
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
LAUNCH_PID=""

cleanup() {
  local status=$?
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  if [[ $status -eq 0 ]]; then
    echo "PASS Dual CRX control/GUI Phase 1 mock workflow"
  else
    echo "FAIL Dual CRX control/GUI Phase 1 mock workflow (status=$status)"
  fi
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  echo "Artifacts: $ARTIFACT_DIR"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

set +u
source "$SCRIPT_DIR/codex_env.sh"
set -u
cd "$WORKSPACE_DIR"

echo "[1/5] Build packages"
colcon build --symlink-install --packages-up-to dual_crx_bringup dual_crx_gui \
  --cmake-args -DBUILD_TESTING=ON 2>&1 | tee "$ARTIFACT_DIR/build.log"
set +u
source "$WORKSPACE_DIR/install/setup.bash"
set -u

echo "[2/5] Unit tests"
colcon test --packages-select dual_crx_control --event-handlers console_direct+ \
  2>&1 | tee "$ARTIFACT_DIR/unit.log"
colcon test-result --verbose 2>&1 | tee "$ARTIFACT_DIR/test_results.log"
python3 -m pytest -q test/test_table_calibration_*.py \
  2>&1 | tee "$ARTIFACT_DIR/table_calibration_unit.log"

echo "[3/5] Headless GUI smoke"
python3 -m pytest -q src/dual_crx_gui/test/test_gui_smoke.py \
  2>&1 | tee "$ARTIFACT_DIR/gui_smoke.log"

echo "[4/5] Start isolated mock (RViz disabled)"
setsid ros2 launch dual_crx_bringup dual_crx.launch.py use_mock:=true launch_rviz:=false \
  >"$ARTIFACT_DIR/mock_launch.log" 2>&1 &
LAUNCH_PID=$!

echo "[5/5] Live public API integration"
python3 scripts/phase1_mock_integration.py 2>&1 | tee "$ARTIFACT_DIR/integration.log"
