#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - Core Configuration
# ==========================================================

# Define cache directory for ROS-Hacks
ROSHACKS_CACHE_DIR=${ROSHACKS_CACHE_DIR:-"$HOME/.cache/ros-hacks"}
# Ensure cache directory exists
mkdir -p "${ROSHACKS_CACHE_DIR}"

# Ensure required environment variables are set
WS_FILE=${WS_FILE:-"${ROSHACKS_CACHE_DIR}/current_workspace"}
ROS_DOMAIN_ID_FILE=${ROS_DOMAIN_ID_FILE:-"${ROSHACKS_CACHE_DIR}/domain_id"}
QUICK_COMMAND_FILE=${QUICK_COMMAND_FILE:-"${ROSHACKS_CACHE_DIR}/.quick_command"}

# Determine ROS2 distribution based on Ubuntu version
if [[ $(lsb_release -cs) == 'focal' ]]; then
    export ROS2_NAME='foxy'
elif [[ $(lsb_release -cs) == 'jammy' ]]; then
    export ROS2_NAME='humble'
elif [[ $(lsb_release -cs) == 'noble' ]]; then
    export ROS2_NAME='iron'
else
    # Default to latest stable release if version can't be determined
    export ROS2_NAME='humble'
fi

# Define colors for output
NC='\033[0m'
GREEN_TXT='\e[0;32m'
GREEN_TXT2='\e[32m'
DARK_GREEN_TXT='\e[2;32m'
WHITE_TXT='\e[1;37m'
RED_TXT='\e[31m'
DIM_RED_TXT='\e[2;31m'
LIGHT_BLUE_TXT='\e[96m'
BLUE_TXT='\e[34m'
DIM_BLUE_TXT='\e[2;34m'
DARK_GREY_TXT='\e[90m'
YELLOW_TXT='\e[93m'
BOLDRED="\033[1m\033[31m"     # Bold Red
BOLDGREEN="\033[1m\033[32m"   # Bold Green
BOLDYELLOW="\033[1m\033[33m"  # Bold Yellow
BOLDBLUE="\033[1m\033[34m"    # Bold Blue
BOLDMAGENTA="\033[1m\033[35m" # Bold Magenta
BOLDCYAN="\033[1m\033[36m"    # Bold Cyan
BOLDWHITE="\033[1m\033[37m"   # Bold White

# Backward compatibility - migrate old files if they exist and cache files don't
if [[ -f "$HOME/.ros_ws_selected" && ! -f "${WS_FILE}" ]]; then
    cp "$HOME/.ros_ws_selected" "${WS_FILE}"
fi

if [[ -f "$HOME/.ros_domain_id" && ! -f "${ROS_DOMAIN_ID_FILE}" ]]; then
    cp "$HOME/.ros_domain_id" "${ROS_DOMAIN_ID_FILE}"
fi
