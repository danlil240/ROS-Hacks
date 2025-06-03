#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - Workspace Memory Management
# ==========================================================

# Define script directory and cache directory
ROSHACKS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
ROSHACKS_CACHE_DIR="${HOME}/.cache/ros-hacks"

# Ensure cache directory exists
mkdir -p "${ROSHACKS_CACHE_DIR}"

# Source required functions
if [[ -f "${ROSHACKS_DIR}/functions.sh" ]]; then
    source "${ROSHACKS_DIR}/functions.sh"
else
    echo "[ROS-Hacks] Error: Could not find functions.sh file."
    return 1
fi

# Function to save a workspace-specific memory
# Usage: save_ws_memory <key> <value> [workspace_path]
function save_ws_memory() {
    local key="$1"
    local value="$2"
    local ws_path="${3:-$curr_ws}"
    
    if [[ -z "$ws_path" ]]; then
        get_current_ws
        ws_path="$curr_ws"
    fi
    
    if [[ -z "$ws_path" || ! -d "$ws_path" ]]; then
        printf "${RED_TXT}No workspace selected or invalid workspace path.${NC}\n"
        return 1
    fi
    
    # Extract workspace name from path
    local ws_name=$(basename "$ws_path")
    
    # Create workspace-specific directory
    local ws_cache_dir="${ROSHACKS_CACHE_DIR}/${ws_name}"
    mkdir -p "$ws_cache_dir"
    
    # Save memory
    echo "$value" > "${ws_cache_dir}/${key}"
    printf "${GREEN_TXT}Saved memory '${WHITE_TXT}${key}${GREEN_TXT}' for workspace ${WHITE_TXT}${ws_name}${NC}\n"
    return 0
}

# Function to get a workspace-specific memory
# Usage: get_ws_memory <key> [workspace_path]
function get_ws_memory() {
    local key="$1"
    local ws_path="${2:-$curr_ws}"
    
    if [[ -z "$ws_path" ]]; then
        get_current_ws
        ws_path="$curr_ws"
    fi
    
    if [[ -z "$ws_path" || ! -d "$ws_path" ]]; then
        printf "${RED_TXT}No workspace selected or invalid workspace path.${NC}\n"
        return 1
    fi
    
    # Extract workspace name from path
    local ws_name=$(basename "$ws_path")
    
    # Check if memory exists
    local memory_file="${ROSHACKS_CACHE_DIR}/${ws_name}/${key}"
    if [[ -f "$memory_file" ]]; then
        cat "$memory_file"
        return 0
    else
        printf "${YELLOW_TXT}Memory '${key}' not found for workspace ${ws_name}.${NC}\n" >&2
        return 1
    fi
}

# Function to list all memories for a workspace
# Usage: list_ws_memories [workspace_path]
function list_ws_memories() {
    local ws_path="${1:-$curr_ws}"
    
    if [[ -z "$ws_path" ]]; then
        get_current_ws
        ws_path="$curr_ws"
    fi
    
    if [[ -z "$ws_path" || ! -d "$ws_path" ]]; then
        printf "${RED_TXT}No workspace selected or invalid workspace path.${NC}\n"
        return 1
    fi
    
    # Extract workspace name from path
    local ws_name=$(basename "$ws_path")
    
    # Check if workspace has any memories
    local ws_cache_dir="${ROSHACKS_CACHE_DIR}/${ws_name}"
    if [[ ! -d "$ws_cache_dir" || -z "$(ls -A "$ws_cache_dir" 2>/dev/null)" ]]; then
        printf "${YELLOW_TXT}No memories found for workspace ${ws_name}.${NC}\n"
        return 1
    fi
    
    printf "${GREEN_TXT}Memories for workspace ${WHITE_TXT}${ws_name}${GREEN_TXT}:${NC}\n"
    for memory_file in "$ws_cache_dir"/*; do
        if [[ -f "$memory_file" ]]; then
            local memory_key=$(basename "$memory_file")
            local memory_value=$(cat "$memory_file")
            printf "${WHITE_TXT}${memory_key}${NC}: ${memory_value}\n"
        fi
    done
    return 0
}

# Function to delete a workspace-specific memory
# Usage: delete_ws_memory <key> [workspace_path]
function delete_ws_memory() {
    local key="$1"
    local ws_path="${2:-$curr_ws}"
    
    if [[ -z "$ws_path" ]]; then
        get_current_ws
        ws_path="$curr_ws"
    fi
    
    if [[ -z "$ws_path" || ! -d "$ws_path" ]]; then
        printf "${RED_TXT}No workspace selected or invalid workspace path.${NC}\n"
        return 1
    fi
    
    # Extract workspace name from path
    local ws_name=$(basename "$ws_path")
    
    # Check if memory exists
    local memory_file="${ROSHACKS_CACHE_DIR}/${ws_name}/${key}"
    if [[ -f "$memory_file" ]]; then
        rm "$memory_file"
        printf "${GREEN_TXT}Deleted memory '${WHITE_TXT}${key}${GREEN_TXT}' from workspace ${WHITE_TXT}${ws_name}${NC}\n"
        return 0
    else
        printf "${YELLOW_TXT}Memory '${key}' not found for workspace ${ws_name}.${NC}\n"
        return 1
    fi
}

# Function to save the current workspace to a memory file
# Usage: remember_curr_ws
function remember_curr_ws() {
    get_current_ws
    if [[ -n "$curr_ws" && -d "$curr_ws" ]]; then
        echo "$curr_ws" > "${ROSHACKS_CACHE_DIR}/curr_ws"
        printf "${GREEN_TXT}Remembered current workspace: ${WHITE_TXT}${curr_ws}${NC}\n"
        return 0
    else
        printf "${YELLOW_TXT}No workspace selected or invalid workspace.${NC}\n"
        return 1
    fi
}

# Function to restore the remembered workspace
# Usage: restore_curr_ws
function restore_curr_ws() {
    local ws_file="${ROSHACKS_CACHE_DIR}/curr_ws"
    if [[ -f "$ws_file" ]]; then
        local remembered_ws=$(cat "$ws_file")
        if [[ -d "$remembered_ws" ]]; then
            set_current_ws "$remembered_ws"
            source_ws "$remembered_ws"
            printf "${GREEN_TXT}Restored workspace: ${WHITE_TXT}${remembered_ws}${NC}\n"
            return 0
        else
            printf "${YELLOW_TXT}Remembered workspace no longer exists: ${remembered_ws}${NC}\n"
            rm "$ws_file"
            return 1
        fi
    else
        printf "${YELLOW_TXT}No workspace has been remembered.${NC}\n"
        return 1
    fi
}

# Export functions
export -f save_ws_memory
export -f get_ws_memory
export -f list_ws_memories
export -f delete_ws_memory
export -f remember_curr_ws
export -f restore_curr_ws
