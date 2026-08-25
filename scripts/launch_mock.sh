#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$SCRIPT_DIR/codex_env.sh"

cd "$WORKSPACE_DIR"
ros2 launch dual_crx_bringup dual_crx.launch.py use_mock:=true "$@"
