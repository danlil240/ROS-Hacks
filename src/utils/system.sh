#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - System Utility Functions
# ==========================================================

# Unset ROS environment variables for a clean environment
# Usage: unROS
function unROS() {
    # Get all variables containing 'ROS'
    vars=$(env | egrep -i ROS | column -t -s '=' | sed -E 's/ .*//g')

    # For everyone do:
    for v in $vars; do
        # Get the value
        if [[ $v == *"PWD"* ]]; then
            continue
        fi
        str=$(printenv $v)
        # Divide into array separated by colon
        arrIN=(${str//:/ })
        # If variable name contains 'ROS' unset it
        if [[ $v == *"ROS"* ]]; then
            unset $v
            continue
        else # Otherwise evaluate the fields
            # For every field check:
            for i in "${arrIN[@]}"; do
                # If contains 'ros' or '_ws/' - skip the field
                if [[ $i == *"ros"* ]]; then
                    continue
                fi
                if [[ $i =~ "_ws/" ]]; then
                    continue
                fi
                # Save the current state of temporary variable
                old=$(printenv ${v}_tmp)
                # Check if tmp variable exists and if not - set it
                if [[ -z "$old" ]]; then
                    old="$(printenv ${v})"
                fi
                # If not - add the field to temporary variable
                if [[ -z "$(echo $old | grep -e ${i})" ]]; then
                    export ${v}_tmp=${old}:${i}
                fi
            done
            # Export the new value
            tmp=$(printenv ${v}_tmp)
            if [[ -n "$tmp" && "${tmp:0:1}" == ":" ]]; then
                export ${v}_tmp=${tmp:1}
            fi
            export $v=$(printenv ${v}_tmp | sed -e 's/^://')
            tmp_trim=$(echo $(printenv ${v}) | sed -e 's/^://')
            export $v="$tmp_trim"
        fi
    done
    printf "${GREEN_TXT}Unset ROS environment variables.${NC}\n"
}

# Fix JetBrains IDE shortcuts for ROS2 compatibility
# Usage: fixJB
function fixJB() {
    CLION_FILE=~/.local/share/applications/jetbrains-clion.desktop
    PYCHARM_FILE=~/.local/share/applications/jetbrains-pycharm.desktop
    PYCHARM_CE_FILE=~/.local/share/applications/jetbrains-pycharm-ce.desktop

    if [ -f "$CLION_FILE" ]; then
        printf "${GREEN_TXT}Patching CLion shortcut for ROS2 compatibility.${NC}\n"
        sed -i -e 's/Exec="/Exec=bash -i -c "/g' $CLION_FILE
        sed -i -e 's/Name=CLion/Name=ROS2 CLion/g' $CLION_FILE
    else
        printf "${YELLOW_TXT}CLion not found in ~/.local/share/applications${NC}\n"
    fi

    if [ -f "$PYCHARM_FILE" ]; then
        printf "${GREEN_TXT}Patching PyCharm Professional shortcut for ROS2 compatibility.${NC}\n"
        sed -i -e 's/Exec="/Exec=bash -i -c "/g' $PYCHARM_FILE
        sed -i -e 's/Name=PyCharm Professional/Name=ROS2 PyCharm Professional/g' $PYCHARM_FILE
    else
        printf "${YELLOW_TXT}PyCharm Professional not found in ~/.local/share/applications${NC}\n"
    fi

    if [ -f "$PYCHARM_CE_FILE" ]; then
        printf "${GREEN_TXT}Patching PyCharm CE shortcut for ROS2 compatibility.${NC}\n"
        sed -i -e 's/Exec="/Exec=bash -i -c "/g' $PYCHARM_CE_FILE
        sed -i -e 's/Name=PyCharm /Name=ROS2 PyCharm /g' $PYCHARM_CE_FILE
    else
        printf "${YELLOW_TXT}PyCharm CE not found in ~/.local/share/applications${NC}\n"
    fi
}

# Interactive setup for workspaces and domain ID
# Usage: ask_for_ws_and_domain
function ask_for_ws_and_domain() {
    # Ask if user wants to select a workspace
    read -p "Would you like to select a ROS2 workspace? (y/n): " choice
    if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
        # Check if we should create a new workspace
        read -p "Create a new workspace? (y/n): " new_choice
        if [[ "$new_choice" == "y" || "$new_choice" == "Y" ]]; then
            prompt_new_ws
        else
            select_ws
        fi
    fi

    # Ask if user wants to set ROS domain ID
    read -p "Would you like to set ROS_DOMAIN_ID? (y/n): " choice
    if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
        # Show current domain ID
        get_ros_domain_id
        printf "${BLUE_TXT}Current ROS_DOMAIN_ID: ${WHITE_TXT}${domain_id}${NC}\n"
        
        # Ask for new domain ID
        read -p "Enter new ROS_DOMAIN_ID (0-255): " new_id
        
        # Validate input
        if ! [[ "$new_id" =~ ^[0-9]+$ ]]; then
            printf "${RED_TXT}Invalid domain ID. Please enter a number.${NC}\n"
        elif [[ $new_id -lt 0 || $new_id -gt 255 ]]; then
            printf "${RED_TXT}Domain ID must be between 0 and 255.${NC}\n"
        else
            set_ros_domain_id "$new_id"
        fi
    fi
}

# Get the version stamp of a workspace
# Usage: get_version_stamp <workspace_path>
function get_version_stamp() {
    local ws=${1:-""}
    if [[ -z "${ws}" ]]; then
        printf "${RED_TXT}ROS2 workspace path not specified.${NC}\n"
        return 1
    fi

    # Check if the workspace exists
    if [[ ! -d "${ws}" ]]; then
        printf "${RED_TXT}Workspace directory ${ws} does not exist.${NC}\n"
        return 1
    fi

    # Generate a version stamp based on current date/time
    local timestamp=$(date "+%Y%m%d-%H%M%S")
    local hostname=$(hostname | sed 's/\..*$//')
    local user=$(whoami)

    # Get the git branch if this is a git repository
    local git_branch=""
    if cd "${ws}" && git rev-parse --git-dir > /dev/null 2>&1; then
        git_branch=$(git symbolic-ref --short HEAD 2>/dev/null || echo "detached")
        git_branch="-${git_branch}"
    fi

    # Create the version stamp
    echo "${timestamp}-${user}@${hostname}${git_branch}"
}

# Build a release of the workspace
# Usage: build_release
function build_release() {
    get_current_ws
    if [[ -z "$curr_ws" ]]; then
        printf "${RED_TXT}No workspace is currently selected.${NC}\n"
        return 1
    fi

    # Get version stamp
    local version=$(get_version_stamp "$curr_ws")

    # Ask for confirmation
    printf "${BLUE_TXT}Building release of workspace: ${WHITE_TXT}${curr_ws}${NC}\n"
    printf "${BLUE_TXT}Version stamp: ${WHITE_TXT}${version}${NC}\n"
    read -p "Proceed with build? (y/n): " choice
    
    if [[ "$choice" != "y" && "$choice" != "Y" ]]; then
        printf "${YELLOW_TXT}Build cancelled.${NC}\n"
        return 0
    fi

    # Clean the workspace first
    printf "${BLUE_TXT}Cleaning workspace...${NC}\n"
    clean_ros2_ws "$curr_ws"

    # Change to workspace directory
    cd "$curr_ws" || {
        printf "${RED_TXT}Failed to change to workspace directory.${NC}\n"
        return 1
    }

    # Build in release mode
    printf "${BLUE_TXT}Building workspace in Release mode...${NC}\n"
    colcon build --symlink-install --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release
    
    if [[ $? -ne 0 ]]; then
        printf "${RED_TXT}Build failed.${NC}\n"
        return 1
    fi

    # Create archive directory if it doesn't exist
    local archive_dir="${curr_ws}/archive"
    if [[ ! -d "$archive_dir" ]]; then
        mkdir -p "$archive_dir"
    fi

    # Create archive of install directory
    printf "${BLUE_TXT}Creating archive...${NC}\n"
    local archive_file="${archive_dir}/install-${version}.tar.gz"
    tar -czf "$archive_file" -C "$curr_ws" install
    
    if [[ $? -eq 0 ]]; then
        printf "${GREEN_TXT}Archive created: ${WHITE_TXT}${archive_file}${NC}\n"
    else
        printf "${RED_TXT}Failed to create archive.${NC}\n"
        return 1
    fi

    # Source the workspace
    source_ws "$curr_ws"
}
