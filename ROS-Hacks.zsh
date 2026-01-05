#!/usr/bin/env zsh

setopt PROMPT_SUBST

# Define script directory (zsh-specific)
ROSHACKS_DIR="$(cd -- "$(dirname -- "${(%):-%x}")" >/dev/null 2>&1 && pwd)"
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

# Zsh-specific completions (bash completions are loaded by ROS-Hacks.sh)
if [[ $- == *i* ]] && [[ -d "${ROSHACKS_COMPLETIONS_DIR}" ]]; then
    # Load any zsh-specific completions if they exist
    for _rh_comp in "${ROSHACKS_COMPLETIONS_DIR}"/*.zsh; do
        [[ -f "${_rh_comp}" ]] && source "${_rh_comp}"
    done
    unset _rh_comp
fi

# Load zsh-specific keybindings
if [[ -f "${ROSHACKS_DIR}/zsh-keybindings.zsh" ]]; then
  source "${ROSHACKS_DIR}/zsh-keybindings.zsh"
fi

# Load p10k integration if available
if [[ -f "${ROSHACKS_DIR}/p10k-integration.zsh" ]]; then
  source "${ROSHACKS_DIR}/p10k-integration.zsh"
fi

# Ensure __git_ps1 is available; if not, try to source it or noop
if ! typeset -f __git_ps1 >/dev/null 2>&1; then
    if [[ -f "/usr/lib/git-core/git-sh-prompt" ]]; then
        source "/usr/lib/git-core/git-sh-prompt"
    else
        # define a noop to avoid prompt errors
        __git_ps1() { :; }
    fi
fi

# Zsh-specific prompt setup
PROMPT=$'%F{green}%n%f %F{green}$(get_current_ws_name):$ROS_DOMAIN_ID%f %F{blue}%~%f%F{cyan}$(__git_ps1)%f:\n%(?.%F{green}➜.%F{red}➜)%f '

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

# Set colcon environment
export COLCON_HOME=$HOME/.colcon
export COLCON_DEFAULTS_FILE=$COLCON_HOME/defaults.yaml

# Mark as loaded to prevent duplicate sourcing
export ROSHACKS_LOADED=1
