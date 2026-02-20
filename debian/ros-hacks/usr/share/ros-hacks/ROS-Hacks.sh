#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - Main Entry Point
# ==========================================================


# Define script directory (bash-specific)
ROSHACKS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
ROSHACKS_LIB_DIR="${ROSHACKS_DIR}/lib"
ROSHACKS_COMPLETIONS_DIR="${ROSHACKS_DIR}/completions"

# Read version from VERSION file
if [[ -f "${ROSHACKS_DIR}/VERSION" ]]; then
    VERSION=$(cat "${ROSHACKS_DIR}/VERSION")
else
    VERSION="unknown"
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

# Source optional modules if present
if [[ -d "${ROSHACKS_LIB_DIR}" ]]; then
    for _rh_mod in "${ROSHACKS_LIB_DIR}"/*.sh; do
        [[ -f "${_rh_mod}" ]] && source "${_rh_mod}"
    done
    unset _rh_mod
fi

# Load completions when interactive bash and completions exist
if [[ $- == *i* ]] && [[ -n "${BASH_VERSION:-}" ]] && [[ -d "${ROSHACKS_COMPLETIONS_DIR}" ]]; then
    if [[ -f "${ROSHACKS_COMPLETIONS_DIR}/ros-hacks-completions.bash" ]]; then
        source "${ROSHACKS_COMPLETIONS_DIR}/ros-hacks-completions.bash"
    fi
fi

# Ensure __git_ps1 is available; if not, try to source it or noop
if ! type -t __git_ps1 >/dev/null 2>&1; then
    if [[ -f "/usr/lib/git-core/git-sh-prompt" ]]; then
        source "/usr/lib/git-core/git-sh-prompt"
    else
        # define a noop to avoid prompt errors
        __git_ps1() { :; }
    fi
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
export COLCON_HOME=$HOME/.colcon
export COLCON_DEFAULTS_FILE=$COLCON_HOME/defaults.yaml
# Mark as loaded to prevent duplicate sourcing
export ROSHACKS_LOADED=1
