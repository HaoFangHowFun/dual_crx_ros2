#!/usr/bin/env bash

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/codex_env.sh"

LEFT_IP="${DUAL_CRX_LEFT_IP:-192.168.2.100}"
RIGHT_IP="${DUAL_CRX_RIGHT_IP:-192.168.1.100}"

usage() {
  echo "Usage: $0 [--left-ip ADDRESS] [--right-ip ADDRESS]"
}

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

failures=0
warnings=0

pass() { echo "PASS: $*"; }
warn() { echo "WARN: $*"; warnings=$((warnings + 1)); }
fail() { echo "FAIL: $*"; failures=$((failures + 1)); }

echo "Dual CRX physical-host preflight"
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
echo "left=$LEFT_IP right=$RIGHT_IP"

case "${ROS_DISTRO:-}" in
  humble|jazzy) pass "supported ROS distribution detected" ;;
  *) fail "expected ROS_DISTRO=humble or jazzy" ;;
esac

for package_name in fanuc_hardware_interface fanuc_controllers fanuc_crx_description dual_crx_bringup; do
  if ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
    pass "ROS package available: $package_name"
  else
    fail "ROS package missing: $package_name"
  fi
done

if [ "$LEFT_IP" = "$RIGHT_IP" ]; then
  fail "left and right robot IP addresses are identical"
fi

route_device() {
  ip route get "$1" 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "dev") {print $(i+1); exit}}'
}

left_device="$(route_device "$LEFT_IP")"
right_device="$(route_device "$RIGHT_IP")"

if [ -n "$left_device" ]; then
  pass "route to left robot uses $left_device"
else
  fail "no route to left robot $LEFT_IP"
fi

if [ -n "$right_device" ]; then
  pass "route to right robot uses $right_device"
else
  fail "no route to right robot $RIGHT_IP"
fi

if [ -n "$left_device" ] && [ "$left_device" = "$right_device" ]; then
  warn "both robots route through $left_device; dedicated NICs should normally be different"
fi

for robot_entry in "left:$LEFT_IP" "right:$RIGHT_IP"; do
  robot_name="${robot_entry%%:*}"
  robot_ip="${robot_entry#*:}"
  if ping -c 2 -W 1 "$robot_ip" >/dev/null 2>&1; then
    pass "$robot_name robot responds to ping ($robot_ip)"
  else
    fail "$robot_name robot does not respond to ping ($robot_ip)"
  fi
done

if [[ "$(uname -r)" == *rt* ]] || [[ "$(uname -r)" == *realtime* ]]; then
  pass "real-time kernel name detected: $(uname -r)"
else
  warn "PREEMPT_RT kernel not detected; acceptable for initial 500 Hz tests, monitor timing faults"
fi

echo
echo "Manual controller checks required on BOTH teach pendants:"
echo "  [ ] R-30iB Mini Plus software V9.40P/77 or later"
echo "  [ ] J519 + R912, or S636, installed"
echo "  [ ] payload/tool and DCS configuration verified"
echo "  [ ] E-stop and reduced-speed test procedure agreed"
echo "  [ ] AUTO, alarms cleared, TP disabled, TP programs aborted before motion authority"
echo
echo "Result: failures=$failures warnings=$warnings"

if [ "$failures" -ne 0 ]; then
  exit 1
fi
