#!/usr/bin/env bash
# Detect ROS 2 distribution from Ubuntu release codename or installed ROS.
# Sets and exports ROS2_NAME. Safe to source multiple times.

if [[ -n "${ROS2_NAME:-}" ]]; then
  return 0 2>/dev/null || true
fi

case "$(lsb_release -cs 2>/dev/null || echo unknown)" in
  focal) ROS2_NAME=foxy;;
  jammy) ROS2_NAME=humble;;
  noble) ROS2_NAME=jazzy;;
  *)     ROS2_NAME="";;
esac

if [[ -z "$ROS2_NAME" ]]; then
  for d in jazzy humble iron foxy rolling; do
    if [[ -f "/opt/ros/$d/setup.bash" || -f "/opt/ros/$d/setup.zsh" ]]; then
      ROS2_NAME=$d
      break
    fi
  done
fi

ROS2_NAME=${ROS2_NAME:-jazzy}
export ROS2_NAME
