#!/usr/bin/env bash
set -euo pipefail

# Usage: run-in-ros.sh [--ws <workspace_root>] <command> [args...]
# Sources the host ROS 2 environment and the workspace install (if present),
# then executes the provided command.

WS_ROOT=""
if [[ "${1:-}" == "--ws" && -n "${2:-}" ]]; then
  WS_ROOT="$2"; shift 2
fi

# Determine ROS 2 codename from Ubuntu release
ROS2_NAME=""
case "$(lsb_release -cs 2>/dev/null || echo unknown)" in
  focal) ROS2_NAME=foxy;;
  jammy) ROS2_NAME=humble;;
  noble) ROS2_NAME=jazzy;;
  *)     ROS2_NAME="";;
esac

# Source system ROS if available (prefer detected codename)
if [[ -n "$ROS2_NAME" && -f "/opt/ros/$ROS2_NAME/setup.bash" ]]; then
  source "/opt/ros/$ROS2_NAME/setup.bash"
else
  for d in jazzy humble iron foxy rolling; do
    if [[ -f "/opt/ros/$d/setup.bash" ]]; then
      source "/opt/ros/$d/setup.bash"; break
    fi
  done
fi

# Resolve workspace root: CLI > env > PWD (if looks like a ws)
if [[ -z "$WS_ROOT" ]]; then
  if [[ -f "$HOME/.cache/ros-hacks/ros_ws_selected" ]]; then
    WS_ROOT=$(cat "$HOME/.cache/ros-hacks/ros_ws_selected")
  elif [[ -d "$(pwd)/src" ]]; then
    WS_ROOT="$(pwd)"
  fi
fi

# Source local install if present (workspace or package-local .colcon)
if [[ -n "${WS_ROOT}" && -f "${WS_ROOT}/install/setup.bash" ]]; then
  source "${WS_ROOT}/install/setup.bash"
elif [[ -f "${PWD}/.colcon/install/setup.bash" ]]; then
  source "${PWD}/.colcon/install/setup.bash"
fi

# Execute the command
if [[ $# -eq 0 ]]; then
  echo "Usage: $0 [--ws <workspace_root>] <command> [args...]" >&2
  exit 2
fi
exec "$@"
