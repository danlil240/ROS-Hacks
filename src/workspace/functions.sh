#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - Workspace Management Functions
# ==========================================================

# Prompts the user to create a new ROS2 workspace
# Usage: prompt_new_ws
function prompt_new_ws() {
    printf "${LIGHT_BLUE_TXT}Creating new ROS2 workspace${NC}\n"
    printf "${LIGHT_BLUE_TXT}Please enter the workspace name, will create ~/####_ws/src:${NC}\n"
    read -p ":  " name
    if [[ -z "${name}" ]]; then
        printf "${RED_TXT}No workspace name provided, aborting.${NC}\n"
        return 1
    fi
    createWS "$name"
}

# Creates a new ROS2 workspace with the specified name
# Usage: createWS <workspace_name>
function createWS() {
    local name=${1:-""}
    if [[ -z "${name}" ]]; then
        printf "${RED_TXT}ROS2 workspace name not specified.${NC}\n"
        return 1
    fi

    # Check if workspace already exists
    if [[ -d "~/${name}_ws" ]]; then
        printf "${YELLOW_TXT}Workspace ~/${name}_ws already exists.${NC}\n"
        read -p "Do you want to use it anyway? (y/n): " choice
        case "$choice" in
        y | Y) printf "${YELLOW_TXT}Using existing workspace.${NC}\n" ;;
        *)
            printf "${RED_TXT}Aborting workspace creation.${NC}\n"
            return 1
            ;;
        esac
    else
        # Create workspace directory structure
        mkdir -p ~/${name}_ws/src
        if [[ $? -ne 0 ]]; then
            printf "${RED_TXT}Failed to create workspace directory.${NC}\n"
            return 1
        fi
    fi

    # Set and initialize workspace
    set_current_ws "${HOME}/${name}_ws"
    get_current_ws
    cd "$curr_ws" || {
        printf "${RED_TXT}Failed to change to workspace directory.${NC}\n"
        return 1
    }

    # Initialize as ROS2 workspace
    printf "${BLUE_TXT}Initializing ROS2 workspace...${NC}\n"
    source /opt/ros/${ROS2_NAME}/setup.bash
    colcon build --symlink-install
    local build_status=$?
    source_ws "${curr_ws}"

    if [[ $build_status -eq 0 ]]; then
        printf "${GREEN_TXT}ROS2 workspace created successfully at ${WHITE_TXT}${curr_ws}${NC}\n"
    else
        printf "${YELLOW_TXT}ROS2 workspace initialized with warnings at ${WHITE_TXT}${curr_ws}${NC}\n"
    fi
}

# Allows user to select a workspace from a list of available workspaces
# Usage: select_ws
function select_ws() {
    find_ws
    ws_count=${#ws_list[@]}
    if [[ $ws_count -eq 0 ]]; then
        printf "${YELLOW_TXT}No ROS2 workspaces found.${NC}\n"
        read -p "Would you like to create a new workspace? (y/n): " choice
        if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
            prompt_new_ws
        fi
        return
    fi

    print_ws
    ask_for_num "Select workspace" $ws_count
    set_current_ws "${ws_list[$((selection - 1))]}"
}

# Gets the current ROS2 workspace path
# Usage: get_current_ws
# Sets global variable: curr_ws
function get_current_ws() {
    if [[ -f "${WS_FILE}" ]]; then
        curr_ws=$(cat "${WS_FILE}")
    else
        curr_ws=""
    fi
}

# Gets the current ROS2 workspace name (not full path)
# Usage: get_current_ws_name
# Returns: Workspace name or "none" if no workspace is selected
function get_current_ws_name() {
    get_current_ws
    if [[ -z "$curr_ws" ]]; then
        echo "none"
    else
        basename "$curr_ws" | sed 's/_ws//'
    fi
}

# Sets the current ROS2 workspace
# Usage: set_current_ws <workspace_path>
function set_current_ws() {
    local ws=${1:-""}
    if [[ -z "${ws}" ]]; then
        printf "${RED_TXT}ROS2 workspace path not specified.${NC}\n"
        return 1
    fi

    # Verify the workspace exists
    if [[ ! -d "${ws}" ]]; then
        printf "${RED_TXT}Workspace directory ${ws} does not exist.${NC}\n"
        return 1
    fi

    # Verify it's a valid ROS2 workspace
    if [[ ! -d "${ws}/src" ]]; then
        printf "${YELLOW_TXT}Warning: ${ws} may not be a valid ROS2 workspace (missing src directory).${NC}\n"
        read -p "Set as workspace anyway? (y/n): " choice
        if [[ "$choice" != "y" && "$choice" != "Y" ]]; then
            printf "${RED_TXT}Aborting workspace selection.${NC}\n"
            return 1
        fi
    fi

    # Save the selected workspace
    echo "${ws}" >"${WS_FILE}"
    printf "${GREEN_TXT}Current workspace set to: ${WHITE_TXT}${ws}${NC}\n"

    # Source the workspace
    source_ws "${ws}"
}

# Determine the ROS version of a workspace
# Usage: determine_ws_ros_version <workspace_path>
function determine_ws_ros_version() {
    local ws=${1:-""}
    if [[ -z "${ws}" ]]; then
        printf "${RED_TXT}ROS2 workspace path not specified.${NC}\n"
        return 1
    fi

    # Check for setup.sh files to determine ROS version
    if [[ -f "${ws}/install/setup.bash" ]]; then
        echo "ROS2"
    elif [[ -f "${ws}/devel/setup.bash" ]]; then
        echo "ROS1"
    else
        echo "unknown"
    fi
}

# Sources a ROS2 workspace setup file
# Usage: source_ws <workspace_path>
function source_ws() {
    local ws=${1:-""}
    if [[ -z "${ws}" ]]; then
        printf "${RED_TXT}ROS2 workspace path not specified.${NC}\n"
        return 1
    fi

    # Check if workspace exists
    if [[ ! -d "${ws}" ]]; then
        printf "${RED_TXT}Workspace directory ${ws} does not exist.${NC}\n"
        return 1
    fi

    # Determine which ROS version
    local ros_version=$(determine_ws_ros_version "${ws}")

    # Source the appropriate setup file
    if [[ "${ros_version}" == "ROS2" && -f "${ws}/install/setup.bash" ]]; then
        source "${ws}/install/setup.bash"
        printf "${GREEN_TXT}Sourced ROS2 workspace: ${WHITE_TXT}${ws}${NC}\n"
    elif [[ "${ros_version}" == "ROS1" && -f "${ws}/devel/setup.bash" ]]; then
        source "${ws}/devel/setup.bash"
        printf "${GREEN_TXT}Sourced ROS1 workspace: ${WHITE_TXT}${ws}${NC}\n"
    else
        printf "${RED_TXT}No valid setup.bash found in ${ws}.${NC}\n"
        return 1
    fi
}

# Rebuilds the current workspace
# Usage: rebuild_curr_ws
function rebuild_curr_ws() {
    get_current_ws
    if [[ -z "$curr_ws" ]]; then
        printf "${RED_TXT}No workspace is currently selected.${NC}\n"
        return 1
    fi

    # Change to workspace directory
    cd "$curr_ws" || {
        printf "${RED_TXT}Failed to change to workspace directory.${NC}\n"
        return 1
    }

    colcon build --symlink-install

    # Source the workspace after building
    if [[ $? -eq 0 ]]; then
        printf "${GREEN_TXT}Build completed successfully.${NC}\n"
        source_ws "$curr_ws"
    else
        printf "${RED_TXT}Build failed.${NC}\n"
        printf "${YELLOW_TXT}Showing errors:${NC}\n"
        show_colcon_errors
        return 1
    fi
}

# Find ROS2 workspaces in home directory
# Usage: find_ws
# Populates global variable: ws_list
function find_ws() {
    ws_list=()
    local potential_dirs=$(find $HOME -maxdepth 1 -type d -name "*_ws" 2>/dev/null)

    for dir in $potential_dirs; do
        if [[ -d "$dir/src" ]]; then
            ws_list+=("$dir")
        fi
    done
}

# Prints a list of discovered workspaces
# Usage: print_ws
function print_ws() {
    find_ws
    local ws_count=${#ws_list[@]}

    printf "${BLUE_TXT}Available ROS2 workspaces:${NC}\n"

    if [[ $ws_count -eq 0 ]]; then
        printf "${YELLOW_TXT}No workspaces found.${NC}\n"
        return
    fi

    get_current_ws

    for i in "${!ws_list[@]}"; do
        local ws="${ws_list[$i]}"
        local ws_name=$(basename "$ws")
        local index=$((i + 1))

        # Determine if this is the current workspace
        if [[ "$ws" == "$curr_ws" ]]; then
            printf "  ${WHITE_TXT}${index})${NC} ${GREEN_TXT}${ws_name}${NC} [CURRENT]\n"
        else
            printf "  ${WHITE_TXT}${index})${NC} ${ws_name}\n"
        fi

        # Show packages in the workspace
        if [[ -d "$ws/src" ]]; then
            local packages=$(find "$ws/src" -maxdepth 2 -name "package.xml" | wc -l)
            printf "     ${DARK_GREY_TXT}Contains ${packages} package(s)${NC}\n"
        fi

        # Show ROS version
        local ros_version=$(determine_ws_ros_version "$ws")
        printf "     ${DARK_GREY_TXT}Type: ${ros_version}${NC}\n"
    done
}

# Ask for a number selection from a range
# Usage: ask_for_num <prompt> <max_value>
function ask_for_num() {
    local prompt=${1:-"Select an option"}
    local max=${2:-1}

    if [[ $max -lt 1 ]]; then
        printf "${RED_TXT}No options available.${NC}\n"
        return 1
    fi

    selection=""
    while [[ -z "$selection" ]]; do
        read -p "${prompt} (1-${max}): " selection

        # Check if input is a number and in range
        if ! [[ "$selection" =~ ^[0-9]+$ ]]; then
            printf "${RED_TXT}Please enter a valid number.${NC}\n"
            selection=""
        elif [[ "$selection" -lt 1 || "$selection" -gt $max ]]; then
            printf "${RED_TXT}Please enter a number between 1 and ${max}.${NC}\n"
            selection=""
        fi
    done

    return 0
}

# Clean a ROS2 workspace (remove build, install, log directories)
# Usage: clean_ros2_ws <workspace_path>
function clean_ros2_ws() {
    ws=${1:-""}
    if [[ -z "${ws}" ]]; then
        printf "${RED_TXT}ROS2 workspace path not specified.${NC}\n"
        return 1
    fi

    printf "${YELLOW_TXT}Clearing the workspace: ${ws} ${NC}\n"
    find ${ws}/log -mindepth 1 -maxdepth 1 -type d -print0 | xargs -0 rm -R >/dev/null 2>&1
    find ${ws}/install -mindepth 1 -maxdepth 1 -type d -print0 | xargs -0 rm -R >/dev/null 2>&1
    find ${ws}/build -mindepth 1 -maxdepth 1 -type d -print0 | xargs -0 rm -R >/dev/null 2>&1
    printf "${GREEN_TXT}Workspace cleaned successfully.${NC}\n"

    return 0
}

# Change to a ROS2 package directory
# Usage: ros2cd <package_name>
function ros2cd() {
    pkgname=${1:-""}
    if [[ -z "${pkgname}" ]]; then
        printf "${RED_TXT}ROS2 package name not specified.${NC}\n"
        return 1
    fi

    get_current_ws
    cd $(ros2 pkg prefix $pkgname)
    return 0
}
