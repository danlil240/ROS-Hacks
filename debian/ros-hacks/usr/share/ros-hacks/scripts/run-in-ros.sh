#!/usr/bin/env bash
set -eu
set -o pipefail 2>/dev/null || true
[[ -n ${ZSH_VERSION:-} ]] && setopt pipefail

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
if [[ -n "$ROS2_NAME" ]]; then
  if [[ -n ${ZSH_VERSION:-} && -f "/opt/ros/$ROS2_NAME/setup.zsh" ]]; then
    source "/opt/ros/$ROS2_NAME/setup.zsh"
  elif [[ -f "/opt/ros/$ROS2_NAME/setup.bash" ]]; then
    source "/opt/ros/$ROS2_NAME/setup.bash"
  fi
else
  for d in jazzy humble iron foxy rolling; do
    if [[ -n ${ZSH_VERSION:-} && -f "/opt/ros/$d/setup.zsh" ]]; then
      source "/opt/ros/$d/setup.zsh"; break
    elif [[ -f "/opt/ros/$d/setup.bash" ]]; then
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
if [[ -n "${WS_ROOT}" ]]; then
  if [[ -n ${ZSH_VERSION:-} && -f "${WS_ROOT}/install/setup.zsh" ]]; then
    source "${WS_ROOT}/install/setup.zsh"
  elif [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    source "${WS_ROOT}/install/setup.bash"
  fi
elif [[ -d "${PWD}/.colcon/install" ]]; then
  if [[ -n ${ZSH_VERSION:-} && -f "${PWD}/.colcon/install/setup.zsh" ]]; then
    source "${PWD}/.colcon/install/setup.zsh"
  elif [[ -f "${PWD}/.colcon/install/setup.bash" ]]; then
    source "${PWD}/.colcon/install/setup.bash"
  fi
fi

# Execute the command
if [[ $# -eq 0 ]]; then
  echo "Usage: $0 [--ws <workspace_root>] <command> [args...]" >&2
  exit 2
fi
exec "$@"
