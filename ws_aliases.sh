#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - Workspace Alias Manager
# ==========================================================

# Define script directory
ROSHACKS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
ROSHACKS_CACHE_DIR="${HOME}/.cache/ros-hacks"

# Source required functions
if [[ -f "${ROSHACKS_DIR}/functions.sh" ]]; then
    source "${ROSHACKS_DIR}/functions.sh"
else
    echo "[ROS-Hacks] Error: Could not find functions.sh file."
    return 1
fi

# Ensure cache directory exists
mkdir -p "${ROSHACKS_CACHE_DIR}"

# Function to find all alias files in a workspace
# Usage: find_ws_alias_files [workspace_path]
function find_ws_alias_files() {
    local ws_path="${1:-$curr_ws}"
    
    if [[ -z "$ws_path" ]]; then
        get_current_ws
        ws_path="$curr_ws"
    fi
    
    if [[ -z "$ws_path" || ! -d "$ws_path" ]]; then
        printf "${RED_TXT}No workspace selected or invalid workspace path.${NC}\n"
        return 1
    fi
    
    # Look for files with "alias" in the name in the src directory
    find "${ws_path}/src" -type f -name "*alias*" 2>/dev/null | sort
}

# Function to select and source an alias file using fzf
# Usage: source_ws_alias_file
function source_ws_alias_file() {
    local ws_path="${1:-$curr_ws}"
    
    if [[ -z "$ws_path" ]]; then
        get_current_ws
        ws_path="$curr_ws"
    fi
    
    if [[ -z "$ws_path" || ! -d "$ws_path" ]]; then
        printf "${RED_TXT}No workspace selected or invalid workspace path.${NC}\n"
        return 1
    fi
    
    # Find alias files
    local alias_files=($(find_ws_alias_files "$ws_path"))
    
    if [[ ${#alias_files[@]} -eq 0 ]]; then
        printf "${YELLOW_TXT}No alias files found in workspace: ${ws_path}/src${NC}\n"
        return 1
    fi
    
    # Use fzf to select an alias file
    local selected_file
    selected_file=$(printf "%s\n" "${alias_files[@]}" | fzf --prompt="Select alias file to source: " --height=40% --layout=reverse --border)
    
    if [[ -z "$selected_file" ]]; then
        printf "${YELLOW_TXT}No file selected.${NC}\n"
        return 1
    fi
    
    # Source the selected file
    if [[ -f "$selected_file" ]]; then
        printf "${GREEN_TXT}Sourcing alias file: ${WHITE_TXT}${selected_file}${NC}\n"
        source "$selected_file"
        
        # Save to history in cache
        echo "$selected_file" > "${ROSHACKS_CACHE_DIR}/last_sourced_alias"
        return 0
    else
        printf "${RED_TXT}File not found: ${selected_file}${NC}\n"
        return 1
    fi
}

# Function to display the last sourced alias file
# Usage: print_last_sourced_alias
function print_last_sourced_alias() {
    local last_file="${ROSHACKS_CACHE_DIR}/last_sourced_alias"
    
    if [[ -f "$last_file" ]]; then
        local alias_file=$(cat "$last_file")
        printf "${GREEN_TXT}Last sourced alias file: ${WHITE_TXT}${alias_file}${NC}\n"
    else
        printf "${YELLOW_TXT}No alias file has been sourced yet.${NC}\n"
    fi
}

# Export functions
export -f find_ws_alias_files
export -f source_ws_alias_file
export -f print_last_sourced_alias
