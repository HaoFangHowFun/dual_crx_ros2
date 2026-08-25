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

# Resolve the ROS installation root. Support explicit overrides first, then
# search a few common nonstandard locations before falling back to /opt/ros.
_resolve_ros_root() {
  local distro="${ROS_DISTRO:-humble}"
  local candidate=""

  for candidate in \
    "${ROS_ROOT_HINT:-}" \
    "${ROS_ROOT:-}" \
    "/opt/ros/$distro" \
    "$HOME/ros/$distro" \
    "$HOME/ros2/$distro" \
    "$HOME/ros2_${distro}" \
    "/usr/local/ros/$distro"
  do
    if [ -n "$candidate" ] && [ -f "$candidate/setup.bash" ]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  return 1
}

ROS_INSTALL_ROOT="$(_resolve_ros_root || true)"
if [ -n "$ROS_INSTALL_ROOT" ]; then
  _prepend_path_if_dir "$ROS_INSTALL_ROOT/bin"
fi
_prepend_path_if_dir "$HOME/.local/bin"

# Source ROS and overlay workspaces if they exist. Prefer an explicit or
# discovered installation path, otherwise default this project to Humble.
if [ -n "$ROS_INSTALL_ROOT" ]; then
  _source_if_exists "$ROS_INSTALL_ROOT/setup.bash"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
  _source_if_exists "/opt/ros/humble/setup.bash"
fi
_source_if_exists "$HOME/ws_fanuc/install/setup.bash"
_source_if_exists "$HOME/dual_crx_ros2/install/setup.bash"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "codex_env.sh: ros2 not found in PATH after sourcing known setup files." >&2
  echo "codex_env.sh: set ROS_ROOT_HINT to your Humble install prefix, for example /opt/ros/humble." >&2
  return 1 2>/dev/null || exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "codex_env.sh: colcon not found in PATH after sourcing known setup files." >&2
  return 1 2>/dev/null || exit 1
fi
