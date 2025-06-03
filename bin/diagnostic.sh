#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - Diagnostic Script
# ==========================================================

# Get script directory - handle symlinks properly
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do
    DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
    SOURCE="$(readlink "$SOURCE")"
    [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
SCRIPT_DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
ROSHACKS_DIR="$(dirname "$SCRIPT_DIR")"

# Source core configuration for colors
if [[ -f "${ROSHACKS_DIR}/src/core/config.sh" ]]; then
    source "${ROSHACKS_DIR}/src/core/config.sh"
else
    # Define minimal colors if config not found
    NC='\033[0m'
    GREEN_TXT='\e[0;32m'
    RED_TXT='\e[31m'
    YELLOW_TXT='\e[93m'
    BLUE_TXT='\e[34m'
    LIGHT_BLUE_TXT='\e[96m'
    WHITE_TXT='\e[1;37m'
    
    # Define cache directory
    ROSHACKS_CACHE_DIR="${HOME}/.cache/ros-hacks"
    mkdir -p "${ROSHACKS_CACHE_DIR}"
fi

# Check if a command exists
check_command() {
    local cmd=$1
    local msg=$2
    if command -v $cmd &> /dev/null; then
        printf "${GREEN_TXT}✓ ${msg} (${cmd})${NC}\n"
        return 0
    else
        printf "${RED_TXT}✗ ${msg} (${cmd})${NC}\n"
        return 1
    fi
}

# Check if a directory exists
check_directory() {
    local dir=$1
    local msg=$2
    if [[ -d "$dir" ]]; then
        printf "${GREEN_TXT}✓ ${msg} (${dir})${NC}\n"
        return 0
    else
        printf "${RED_TXT}✗ ${msg} (${dir})${NC}\n"
        return 1
    fi
}

# Check if a file exists
check_file() {
    local file=$1
    local msg=$2
    if [[ -f "$file" ]]; then
        printf "${GREEN_TXT}✓ ${msg} (${file})${NC}\n"
        return 0
    else
        printf "${RED_TXT}✗ ${msg} (${file})${NC}\n"
        return 1
    fi
}

# Check ROS2 environment
check_ros2_env() {
    printf "\n${BLUE_TXT}Checking ROS2 Environment...${NC}\n"
    
    # Check if ROS_DISTRO is set
    if [[ -n "$ROS_DISTRO" ]]; then
        printf "${GREEN_TXT}✓ ROS_DISTRO is set to ${ROS_DISTRO}${NC}\n"
    else
        printf "${RED_TXT}✗ ROS_DISTRO is not set${NC}\n"
    fi
    
    # Check if ROS_DOMAIN_ID is set
    if [[ -n "$ROS_DOMAIN_ID" ]]; then
        printf "${GREEN_TXT}✓ ROS_DOMAIN_ID is set to ${ROS_DOMAIN_ID}${NC}\n"
    else
        printf "${YELLOW_TXT}! ROS_DOMAIN_ID is not set${NC}\n"
    fi
    
    # Check if ROS2 is installed
    check_command "ros2" "ROS2 command is available"
    
    # Check if colcon is installed
    check_command "colcon" "Colcon build tool is available"
}

# Check ROS-Hacks environment
check_roshacks_env() {
    printf "\n${BLUE_TXT}Checking ROS-Hacks Environment...${NC}\n"
    
    # Check if ROSHACKS_LOADED is set
    if [[ -n "$ROSHACKS_LOADED" ]]; then
        printf "${GREEN_TXT}✓ ROS-Hacks is loaded${NC}\n"
    else
        printf "${RED_TXT}✗ ROS-Hacks is not loaded${NC}\n"
    fi
    
    # Check cache directory
    check_directory "$ROSHACKS_CACHE_DIR" "ROS-Hacks cache directory"
    
    # Check current workspace file
    check_file "$WS_FILE" "Current workspace file"
    
    # Check domain ID file
    check_file "$ROS_DOMAIN_ID_FILE" "Domain ID file"
    
    # Check current workspace
    get_current_ws
    if [[ -n "$curr_ws" ]]; then
        if [[ -d "$curr_ws" ]]; then
            printf "${GREEN_TXT}✓ Current workspace is set to ${curr_ws}${NC}\n"
            
            # Check if it's a valid ROS workspace
            if [[ -d "$curr_ws/src" ]]; then
                printf "${GREEN_TXT}✓ Workspace has src directory${NC}\n"
            else
                printf "${YELLOW_TXT}! Workspace does not have src directory${NC}\n"
            fi
            
            # Check build/install directories
            if [[ -d "$curr_ws/build" ]]; then
                printf "${GREEN_TXT}✓ Workspace has build directory${NC}\n"
            else
                printf "${YELLOW_TXT}! Workspace does not have build directory${NC}\n"
            fi
            
            if [[ -d "$curr_ws/install" ]]; then
                printf "${GREEN_TXT}✓ Workspace has install directory${NC}\n"
            else
                printf "${YELLOW_TXT}! Workspace does not have install directory${NC}\n"
            fi
        else
            printf "${RED_TXT}✗ Current workspace ${curr_ws} does not exist${NC}\n"
        fi
    else
        printf "${YELLOW_TXT}! No workspace is currently selected${NC}\n"
    fi
}

# Check ROS-Hacks installation
check_roshacks_install() {
    printf "\n${BLUE_TXT}Checking ROS-Hacks Installation...${NC}\n"
    
    # Check if ROS-Hacks directory exists
    check_directory "$ROSHACKS_DIR" "ROS-Hacks directory"
    
    # Check for core components
    check_directory "${ROSHACKS_DIR}/src" "Source directory"
    check_directory "${ROSHACKS_DIR}/bin" "Binary directory"
    check_directory "${ROSHACKS_DIR}/config" "Configuration directory"
    
    # Check for important scripts
    check_file "${ROSHACKS_DIR}/bin/ros-hacks.sh" "Main script"
    check_file "${ROSHACKS_DIR}/config/aliases.sh" "Aliases script"
    
    # Check for inputrc
    check_file "${ROSHACKS_DIR}/config/inputrc" "Input configuration"
    
    # Check if bashrc has ROS-Hacks entries
    if grep -q "ROS-HACKS entries" "$HOME/.bashrc"; then
        printf "${GREEN_TXT}✓ ROS-Hacks is configured in ~/.bashrc${NC}\n"
    else
        printf "${RED_TXT}✗ ROS-Hacks is not configured in ~/.bashrc${NC}\n"
    fi
    
    # Check if inputrc is linked
    if [[ -L "$HOME/.inputrc" ]]; then
        target=$(readlink "$HOME/.inputrc")
        if [[ "$target" == *"ros-hacks"* || "$target" == *"ROS-Hacks"* ]]; then
            printf "${GREEN_TXT}✓ inputrc is linked to ROS-Hacks${NC}\n"
        else
            printf "${YELLOW_TXT}! inputrc is linked to ${target}${NC}\n"
        fi
    else
        printf "${YELLOW_TXT}! inputrc is not linked${NC}\n"
    fi
}

# Print system information
print_system_info() {
    printf "\n${BLUE_TXT}System Information:${NC}\n"
    
    # OS information
    if [[ -f /etc/os-release ]]; then
        source /etc/os-release
        printf "${WHITE_TXT}OS: ${NAME} ${VERSION}${NC}\n"
    else
        printf "${WHITE_TXT}OS: $(uname -s) $(uname -r)${NC}\n"
    fi
    
    # Hardware information
    printf "${WHITE_TXT}CPU: $(grep "model name" /proc/cpuinfo | head -n1 | cut -d: -f2 | sed 's/^[ \t]*//')${NC}\n"
    printf "${WHITE_TXT}Memory: $(free -h | grep Mem | awk '{print $2}')${NC}\n"
    
    # User information
    printf "${WHITE_TXT}User: $(whoami)${NC}\n"
    printf "${WHITE_TXT}Hostname: $(hostname)${NC}\n"
    
    # Shell information
    printf "${WHITE_TXT}Shell: $SHELL ($BASH_VERSION)${NC}\n"
}

# Print ROS2 packages in current workspace
print_ros2_packages() {
    get_current_ws
    
    if [[ -z "$curr_ws" || ! -d "$curr_ws" ]]; then
        printf "\n${YELLOW_TXT}No valid workspace selected.${NC}\n"
        return
    fi
    
    printf "\n${BLUE_TXT}ROS2 Packages in Current Workspace:${NC}\n"
    
    # Check if ros2 command is available
    if ! command -v ros2 &> /dev/null; then
        printf "${RED_TXT}ros2 command not available${NC}\n"
        return
    fi
    
    # Get list of packages
    local packages=$(ros2 pkg list 2>/dev/null)
    
    if [[ -z "$packages" ]]; then
        printf "${YELLOW_TXT}No ROS2 packages found in current workspace${NC}\n"
    else
        printf "${WHITE_TXT}Found $(echo "$packages" | wc -l) packages:${NC}\n"
        echo "$packages" | sort | while read pkg; do
            printf "  ${GREEN_TXT}${pkg}${NC}\n"
        done
    fi
}

# Main function
main() {
    printf "${LIGHT_BLUE_TXT}===============================${NC}\n"
    printf "${LIGHT_BLUE_TXT}ROS-Hacks Diagnostic Tool${NC}\n"
    printf "${LIGHT_BLUE_TXT}===============================${NC}\n\n"
    
    print_system_info
    check_roshacks_install
    check_roshacks_env
    check_ros2_env
    print_ros2_packages
    
    printf "\n${GREEN_TXT}Diagnostic completed.${NC}\n"
}

# Run the main function
main "$@"
