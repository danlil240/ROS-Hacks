#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - Dependency Checker
# ==========================================================

# Define script directory
ROSHACKS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"

# Function to check if a command is available
# Usage: command_exists <command>
function command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to verify dependencies are installed
# Usage: verify_dependencies
function verify_dependencies() {
    local missing_deps=()
    
    # Check for fzf (required for interactive selection)
    if ! command_exists fzf; then
        missing_deps+=("fzf")
    fi
    
    # Check for tmux (required for some commands)
    if ! command_exists tmux; then
        missing_deps+=("tmux")
    fi
    
    # If there are missing dependencies, print a message
    if [[ ${#missing_deps[@]} -gt 0 ]]; then
        printf "${YELLOW_TXT}Some dependencies are missing and required for full functionality:${NC}\n"
        for dep in "${missing_deps[@]}"; do
            printf "  - ${dep}\n"
        done
        printf "${BLUE_TXT}Run the setup.sh script to install missing dependencies.${NC}\n"
        return 1
    fi
    
    return 0
}

# Export functions
export -f command_exists
export -f verify_dependencies

# Run dependency check if this script is being sourced directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    # Source colors from functions.sh if available
    if [[ -f "${ROSHACKS_DIR}/functions.sh" ]]; then
        source "${ROSHACKS_DIR}/functions.sh"
    else
        # Define minimal color variables if functions.sh is not available
        NC='\033[0m'
        GREEN_TXT='\e[0;32m'
        YELLOW_TXT='\e[93m'
        RED_TXT='\e[31m'
        BLUE_TXT='\e[34m'
    fi
    
    verify_dependencies
fi
