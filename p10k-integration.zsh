#!/usr/bin/env zsh

# ==========================================================
# ROS-Hacks Powerlevel10k Integration
# ==========================================================

# Source ROS-Hacks functions if not already loaded
if [[ -z "${ROSHACKS_LOADED:-}" ]]; then
  # Try to find ROS-Hacks directory
  local roshacks_dir=""
  if [[ -n "${ROSHACKS_DIR:-}" ]]; then
    roshacks_dir="$ROSHACKS_DIR"
  elif [[ -f "$HOME/.ros-hacks/ROS-Hacks.zsh" ]]; then
    roshacks_dir="$HOME/.ros-hacks"
  elif [[ -f "/usr/share/ros-hacks/ROS-Hacks.zsh" ]]; then
    roshacks_dir="/usr/share/ros-hacks"
  elif [[ -f "${(%):-%x}" ]]; then
    # If this file is being sourced, try to find ROS-Hacks relative to it
    roshacks_dir="$(cd -- "$(dirname -- "${(%):-%x}")" >/dev/null 2>&1 && pwd)"
  fi
  
  if [[ -n "$roshacks_dir" && -f "$roshacks_dir/functions.sh" ]]; then
    source "$roshacks_dir/functions.sh"
  fi
fi

# ROS-Hacks workspace segment for p10k
function prompt_ros_workspace() {
  if typeset -f get_current_ws_name >/dev/null 2>&1; then
    local ws_name=$(get_current_ws_name)
    if [[ "$ws_name" != "none" && -n "$ws_name" ]]; then
      local domain_id=""
      if typeset -f get_ros_domain_id >/dev/null 2>&1; then
        if [[ -f "$HOME/.cache/ros-hacks/ros_domain_id" ]]; then
          domain_id=$(cat "$HOME/.cache/ros-hacks/ros_domain_id" 2>/dev/null || echo "0")
        else
          domain_id="${ROS_DOMAIN_ID:-0}"
        fi
      fi
      
      if [[ -n "$domain_id" && "$domain_id" != "0" ]]; then
        p10k segment -s ros_workspace  -f ${POWERLEVEL9K_ROS_WORKSPACE_FOREGROUND:-green} -t "[$ws_name:$domain_id]"
      else
        p10k segment -s ros_workspace  -f ${POWERLEVEL9K_ROS_WORKSPACE_FOREGROUND:-green} -t "[$ws_name]"
      fi
    fi
  fi
}


# Update function to refresh prompt when workspace/domain changes
function _ros_hacks_refresh_p10k_prompt() {
  if typeset -f _p9k_reset_prompt >/dev/null 2>&1; then
    _p9k_reset_prompt 1
  fi
}

# Hook into ROS-Hacks functions to refresh prompt
if typeset -f set_current_ws >/dev/null; then
  # Rename original function
  functions[orig_set_current_ws]=$functions[set_current_ws]

  # Define wrapper
  set_current_ws() {
    orig_set_current_ws "$@"
    _ros_hacks_refresh_p10k_prompt
  }
fi


if typeset -f set_ros_domain_id >/dev/null; then
  # Rename original function
  functions[orig_set_ros_domain_id]=$functions[set_ros_domain_id]

  # Define wrapper
  set_ros_domain_id() {
    orig_set_ros_domain_id "$@"
    _ros_hacks_refresh_p10k_prompt
  }
fi
