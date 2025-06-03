#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - Main Entry Point
# ==========================================================

# Define script directory (works with symlinks)
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do
    DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
    SOURCE="$(readlink "$SOURCE")"
    [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
SCRIPT_DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
ROSHACKS_DIR="$(dirname "$SCRIPT_DIR")"

# Read version from VERSION file
if [[ -f "${ROSHACKS_DIR}/VERSION" ]]; then
    VERSION=$(cat "${ROSHACKS_DIR}/VERSION")
else
    VERSION="unknown"
fi

# Source core configuration
if [[ -f "${ROSHACKS_DIR}/src/core/config.sh" ]]; then
    source "${ROSHACKS_DIR}/src/core/config.sh"
else
    echo "[ROS-Hacks] Error: Could not find core configuration file."
    return 1
fi

# Source all modules
for module_dir in "${ROSHACKS_DIR}/src"/*; do
    if [[ -d "${module_dir}" ]]; then
        for script in "${module_dir}"/*.sh; do
            if [[ -f "${script}" && "${script}" != "${ROSHACKS_DIR}/src/core/config.sh" ]]; then
                source "${script}"
            fi
        done
    fi
done

# Source aliases
if [[ -f "${ROSHACKS_DIR}/config/aliases.sh" ]]; then
    source "${ROSHACKS_DIR}/config/aliases.sh"
else
    echo "[ROS-Hacks] Warning: Could not find aliases file."
fi

# Check dependencies if needed
if [[ -f "${ROSHACKS_DIR}/install/check_dependencies.sh" ]]; then
    source "${ROSHACKS_DIR}/install/check_dependencies.sh"
    # Verify dependencies on first load
    if [[ -z "${ROSHACKS_DEPENDENCIES_CHECKED}" ]]; then
        check_dependencies
        export ROSHACKS_DEPENDENCIES_CHECKED=1
    fi
else
    echo "[ROS-Hacks] Warning: Could not find check_dependencies.sh file."
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

# Configure colcon
export COLCON_HOME=$HOME/.colcon
export COLCON_DEFAULTS_FILE=$COLCON_HOME/defaults.yaml

# Mark as loaded to prevent duplicate sourcing
export ROSHACKS_LOADED=1

echo "[ROS-Hacks] Loaded v${VERSION}"
