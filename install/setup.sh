#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks Setup Script
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

# Read version from VERSION file
if [[ -f "${ROSHACKS_DIR}/VERSION" ]]; then
    VERSION=$(cat "${ROSHACKS_DIR}/VERSION")
else
    VERSION="unknown"
fi

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

# Source dependency checker
if [[ -f "${ROSHACKS_DIR}/install/check_dependencies.sh" ]]; then
    source "${ROSHACKS_DIR}/install/check_dependencies.sh"
else
    echo "Error: Could not find dependency checker script."
    exit 1
fi

# Configure .bashrc
function configure_bashrc() {
    printf "${BLUE_TXT}Configuring your bash environment...${NC}\n"

    # Remove any existing ROS-Hacks entries
    if grep -q "ROS-HACKS entries" "$HOME/.bashrc"; then
        printf "${YELLOW_TXT}Found existing ROS-Hacks configuration in ~/.bashrc${NC}\n"
        printf "${BLUE_TXT}Updating configuration...${NC}\n"
        sed -i '/## ROS-HACKS entries ##/,/## ROS-HACKS END ##/d' "$HOME/.bashrc"
    fi

    # Add new entries
    printf "${BLUE_TXT}Adding ROS-Hacks to your ~/.bashrc${NC}\n"
    cat <<EOF >>"$HOME/.bashrc"

## ROS-HACKS entries ##
if [[ -f "/usr/share/ros-hacks/bin/ros-hacks.sh" ]]; then
    source "/usr/share/ros-hacks/bin/ros-hacks.sh"
elif [[ -f "$HOME/.ros-hacks/bin/ros-hacks.sh" ]]; then
    source "$HOME/.ros-hacks/bin/ros-hacks.sh"
elif [[ -f "$ROSHACKS_DIR/bin/ros-hacks.sh" ]]; then
    source "$ROSHACKS_DIR/bin/ros-hacks.sh"
fi
## ROS-HACKS END ##
EOF
}

# Configure inputrc
function configure_inputrc() {
    printf "${BLUE_TXT}Configuring keyboard shortcuts...${NC}\n"

    # Backup existing inputrc if it exists and is not a symlink
    if [[ -f "$HOME/.inputrc" && ! -L "$HOME/.inputrc" ]]; then
        printf "${YELLOW_TXT}Backing up existing ~/.inputrc to ~/.inputrc.bak${NC}\n"
        cp "$HOME/.inputrc" "$HOME/.inputrc.bak"
    fi

    # Check if symlink exists and where it points to
    if [[ -L "$HOME/.inputrc" ]]; then
        current_target=$(readlink "$HOME/.inputrc")
        if [[ "$current_target" != "${ROSHACKS_DIR}/config/inputrc" ]]; then
            printf "${YELLOW_TXT}Replacing existing symlink from ~/.inputrc to ${current_target} with our symlink${NC}\n"
        fi
    fi

    # Create symbolic link (will replace existing symlink if it exists)
    printf "${BLUE_TXT}Linking ROS-Hacks inputrc file...${NC}\n"
    ln -sf "${ROSHACKS_DIR}/config/inputrc" "$HOME/.inputrc"

    # Only run bind if we're in an interactive shell
    if [[ $- == *i* ]]; then
        bind -f $HOME/.inputrc 2>/dev/null || true
    fi
}

# Create initial config files
function create_initial_configs() {
    printf "${BLUE_TXT}Creating initial configuration files...${NC}\n"

    # Create domain ID file if it doesn't exist
    if [[ ! -f "${ROSHACKS_CACHE_DIR}/domain_id" ]]; then
        echo "0" >"${ROSHACKS_CACHE_DIR}/domain_id"
        printf "${GREEN_TXT}Created ROS domain ID file with default value (0)${NC}\n"
    fi

    # For backward compatibility
    if [[ -f "$HOME/.ros_domain_id" && ! -f "${ROSHACKS_CACHE_DIR}/domain_id" ]]; then
        printf "${BLUE_TXT}Migrating domain ID file to cache directory...${NC} "
        cp "$HOME/.ros_domain_id" "${ROSHACKS_CACHE_DIR}/domain_id"
        printf "${GREEN_TXT}Done${NC}\n"
    fi

    # Migrate workspace file if it exists
    if [[ -f "$HOME/.ros_ws_selected" && ! -f "${ROSHACKS_CACHE_DIR}/current_workspace" ]]; then
        printf "${BLUE_TXT}Migrating workspace file to cache directory...${NC} "
        cp "$HOME/.ros_ws_selected" "${ROSHACKS_CACHE_DIR}/current_workspace"
        printf "${GREEN_TXT}Done${NC}\n"
    fi

    # Create .colcon directory if it doesn't exist and plant defaults.yaml
    if [[ ! -d "$HOME/.colcon" ]]; then
        mkdir -p "$HOME/.colcon"
    fi

    # Copy defaults.yaml to .colcon directory
    if [ -f "$ROSHACKS_DIR/config/defaults.yaml" ]; then
        cp "$ROSHACKS_DIR/config/defaults.yaml" "$HOME/.colcon/"
    elif [ -f "/usr/share/ros-hacks/config/defaults.yaml" ]; then
        cp "/usr/share/ros-hacks/config/defaults.yaml" "$HOME/.colcon/"
    else
        printf "${YELLOW_TXT}Warning: Could not find defaults.yaml${NC}\n"
    fi
    printf "${GREEN_TXT}Planted defaults.yaml in ~/.colcon directory${NC}\n"

    # Copy inputrc file to config directory if it doesn't exist
    if [[ ! -f "${ROSHACKS_DIR}/config/inputrc" && -f "${ROSHACKS_DIR}/inputrc" ]]; then
        cp "${ROSHACKS_DIR}/inputrc" "${ROSHACKS_DIR}/config/inputrc"
        printf "${GREEN_TXT}Copied inputrc to config directory${NC}\n"
    fi
}

# Main setup function
function main() {
    printf "${LIGHT_BLUE_TXT}===============================${NC}\n"
    printf "${LIGHT_BLUE_TXT}Setting up ROS-Hacks v${VERSION}${NC}\n"
    printf "${LIGHT_BLUE_TXT}===============================${NC}\n\n"

    # Run setup steps
    check_dependencies
    configure_bashrc
    configure_inputrc
    create_initial_configs

    printf "\n${GREEN_TXT}ROS-Hacks installed successfully!${NC}\n"
    printf "${BLUE_TXT}To complete installation, please run:${NC}\n"
    printf "${WHITE_TXT}    source ~/.bashrc${NC}\n"
    printf "\n${BLUE_TXT}Keyboard shortcuts and functions are now available.${NC}\n"
    printf "${BLUE_TXT}Use ${WHITE_TXT}wsselect${BLUE_TXT} to select a ROS workspace.${NC}\n"
    printf "${BLUE_TXT}Use ${WHITE_TXT}wsnew${BLUE_TXT} to create a new ROS workspace.${NC}\n"
}

# Run the main setup if executed directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main
fi
