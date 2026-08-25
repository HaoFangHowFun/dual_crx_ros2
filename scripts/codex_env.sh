#!/usr/bin/env bash

set -euo pipefail

_source_if_exists() {
  local path="$1"
  if [ -f "$path" ]; then
    local had_nounset=0
    case $- in
      *u*) had_nounset=1 ;;
    esac
    set +u
    # shellcheck disable=SC1090
    source "$path"
    if [ "$had_nounset" -eq 1 ]; then
      set -u
    fi
  fi
}

_prepend_path_if_dir() {
  local path="$1"
  if [ -d "$path" ]; then
    case ":${PATH:-}:" in
      *":$path:"*) ;;
      *) export PATH="$path:${PATH:-}" ;;
    esac
  fi
}

export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"

# Try the selected ROS distribution, then the supported project defaults.
if [ -n "${ROS_DISTRO:-}" ]; then
  _prepend_path_if_dir "/opt/ros/$ROS_DISTRO/bin"
fi
_prepend_path_if_dir "/opt/ros/jazzy/bin"
_prepend_path_if_dir "/opt/ros/humble/bin"
_prepend_path_if_dir "$HOME/.local/bin"

# Source ROS and overlay workspaces if they exist. Prefer an already-selected
# distribution, otherwise Jazzy on the development laptop and Humble on the
# Ubuntu 22.04 deployment PC.
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  _source_if_exists "/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
  _source_if_exists "/opt/ros/jazzy/setup.bash"
else
  _source_if_exists "/opt/ros/humble/setup.bash"
fi
_source_if_exists "$HOME/ws_fanuc/install/setup.bash"
_source_if_exists "$HOME/dual_crx_ros2/install/setup.bash"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "codex_env.sh: ros2 not found in PATH after sourcing known setup files." >&2
  return 1 2>/dev/null || exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "codex_env.sh: colcon not found in PATH after sourcing known setup files." >&2
  return 1 2>/dev/null || exit 1
fi
