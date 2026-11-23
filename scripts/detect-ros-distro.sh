#!/usr/bin/env bash
# Detect ROS 2 distribution from Ubuntu release codename or installed ROS.
# Sets and exports ROS2_NAME. Safe to source multiple times.

if [[ -n "${ROS2_NAME:-}" ]]; then
  return 0 2>/dev/null || true
fi

# lsb_release is not always installed (minimal images). /etc/os-release is.
_rh_codename="$(lsb_release -cs 2>/dev/null || true)"
if [[ -z "$_rh_codename" ]]; then
  _rh_codename="$(
    # shellcheck disable=SC1091
    . /etc/os-release 2>/dev/null && printf '%s' "${VERSION_CODENAME:-}"
  )"
fi

case "${_rh_codename:-unknown}" in
  focal)    ROS2_NAME=foxy;;
  jammy)    ROS2_NAME=humble;;
  noble)    ROS2_NAME=jazzy;;
  resolute) ROS2_NAME=lyrical;; # Ubuntu 26.04, ROS 2 Lyrical Luth (LTS)
  *)        ROS2_NAME="";;
esac
unset _rh_codename

if [[ -z "$ROS2_NAME" ]]; then
  for d in lyrical jazzy kilted humble iron foxy rolling; do
    if [[ -f "/opt/ros/$d/setup.bash" || -f "/opt/ros/$d/setup.zsh" ]]; then
      ROS2_NAME=$d
      break
    fi
  done
fi

ROS2_NAME=${ROS2_NAME:-jazzy}
export ROS2_NAME
