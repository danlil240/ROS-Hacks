#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - Main Entry Point
# ==========================================================

# Define script directory
ROSHACKS_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Read version from VERSION file
if [[ -f "${ROSHACKS_DIR}/VERSION" ]]; then
    VERSION=$(cat "${ROSHACKS_DIR}/VERSION")
else
    VERSION="unknown"
fi

# Prevent duplicate sourcing
if [[ -n "${ROSHACKS_LOADED}" ]]; then
    return 0
fi

# Source configuration files
if [[ -f "${ROSHACKS_DIR}/aliases.sh" ]]; then
    source "${ROSHACKS_DIR}/aliases.sh"
else
    echo "[ROS-Hacks] Warning: Could not find aliases.sh file."
fi

if [[ -f "${ROSHACKS_DIR}/functions.sh" ]]; then
    source "${ROSHACKS_DIR}/functions.sh"
else
    echo "[ROS-Hacks] Error: Could not find functions.sh file."
    return 1
fi

# Update prompt with ROS2 workspace and domain info
PS1=' \[\e[1;32m\]\u\[\033[00m\] \[\e[32m\]$(get_current_ws_name):$ROS_DOMAIN_ID\[\033[00m\] \[\033[03;94m\]\w\[\033[00m\]\[\033[38;5;51m\]$(__git_ps1)\[\033[00m\]:\n$ '

# Load the current workspace
get_current_ws
if [[ -n "$curr_ws" && -d "$curr_ws" ]]; then
    source_ws "$curr_ws"
fi

# Get domain ID
get_ros_domain_id
if [[ -n "$domain_id" ]]; then
    export ROS_DOMAIN_ID="$domain_id"
fi


# Mark as loaded to prevent duplicate sourcing
export ROSHACKS_LOADED=1
