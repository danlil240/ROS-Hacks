#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks Setup Script
# ==========================================================

# Get script directory - handle symlinks properly
if [ -d "/usr/share/ros-hacks" ]; then
    # When installed via apt
    SCRIPT_DIR="/usr/share/ros-hacks"
else
    # When run from cloned repository
    # Follow symlinks to get the real script path
    SOURCE="${BASH_SOURCE[0]}"
    while [ -h "$SOURCE" ]; do
        DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
        SOURCE="$(readlink "$SOURCE")"
        [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
    done
    SCRIPT_DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
fi

# Read version from VERSION file
if [[ -f "${SCRIPT_DIR}/VERSION" ]]; then
    VERSION=$(cat "${SCRIPT_DIR}/VERSION")
else
    VERSION="unknown"
fi

# Set error handling
set -e

# Define colors for output
NC='\033[0m'
GREEN_TXT='\e[0;32m'
RED_TXT='\e[31m'
YELLOW_TXT='\e[93m'
BLUE_TXT='\e[34m'

# Define cache directory for ROS-Hacks
ROSHACKS_CACHE_DIR="${HOME}/.cache/ros-hacks"
# Ensure cache directory exists
mkdir -p "${ROSHACKS_CACHE_DIR}"
LIGHT_BLUE_TXT='\e[96m'
WHITE_TXT='\e[1;37m'

# Check for required dependencies
check_dependencies() {
    printf "${BLUE_TXT}Checking for required dependencies...${NC}\n"

    # Check for tmux
    if ! command -v tmux &>/dev/null; then
        printf "${YELLOW_TXT}tmux is not installed. It's recommended for quick commands.${NC}\n"
        read -p "Do you want to install tmux? (y/n): " choice
        if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
            sudo apt update && sudo apt install -y tmux
            printf "${GREEN_TXT}tmux installed successfully.${NC}\n"
        else
            printf "${YELLOW_TXT}Continuing without tmux. Some functionality will be limited.${NC}\n"
        fi
    else
        printf "${GREEN_TXT}✓ tmux is installed${NC}\n"
    fi

    # Check for fzf
    # Get the real user's home directory, even if running with sudo
    USER_HOME=$(getent passwd $(logname) | cut -d: -f6)

    if [ ! -f "$USER_HOME/.fzf/bin/fzf" ]; then
        printf "${YELLOW_TXT}fzf is not installed. It's recommended for quick commands.${NC}\n"
        read -p "Do you want to install fzf? (y/n): " choice
        if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
            git clone --depth 1 https://github.com/junegunn/fzf.git "$USER_HOME/.fzf"
            "$USER_HOME"/.fzf/install
            printf "${GREEN_TXT}fzf installed successfully.${NC}\n"
        else
            printf "${YELLOW_TXT}Continuing without fzf. Some functionality will be limited.${NC}\n"
        fi
    else
        printf "${GREEN_TXT}✓ fzf is installed${NC}\n"
    fi
}

# Configure .bashrc
configure_bashrc() {
    printf "${BLUE_TXT}Configuring your bash environment...${NC}\n"

    # Remove any existing ROS-Hacks entries
    if grep -q "ROS-HACKS entries" "$HOME/.bashrc"; then
        printf "${YELLOW_TXT}Found existing ROS-Hacks configuration in ~/.bashrc${NC}\n"
        printf "${BLUE_TXT}Updating configuration...${NC}\n"
        sed -i '/## ROS-HACKS entries ##/,/## ROS-HACKS END ##/d' "$HOME/.bashrc"
    fi

    # Add new entries
    printf "${BLUE_TXT}Adding ROS-Hacks to your ~/.bashrc${NC}\n"
    cat <<'EOF' >>"$HOME/.bashrc"

## ROS-HACKS entries ##
if [[ -f "/usr/share/ros-hacks/ROS-Hacks.sh" ]]; then
    source "/usr/share/ros-hacks/ROS-Hacks.sh"
elif [[ -f "$HOME/.ROS-Hacks/ROS-Hacks.sh" ]]; then
    source "$HOME/.ROS-Hacks/ROS-Hacks.sh"
fi
## ROS-HACKS END ##
EOF
}

# Configure inputrc
configure_inputrc() {
    printf "${BLUE_TXT}Configuring keyboard shortcuts...${NC}\n"

    # Backup existing inputrc if it exists and is not a symlink
    if [[ -f "$HOME/.inputrc" && ! -L "$HOME/.inputrc" ]]; then
        printf "${YELLOW_TXT}Backing up existing ~/.inputrc to ~/.inputrc.bak${NC}\n"
        cp "$HOME/.inputrc" "$HOME/.inputrc.bak"
    fi

    # Check if symlink exists and where it points to
    if [[ -L "$HOME/.inputrc" ]]; then
        current_target=$(readlink "$HOME/.inputrc")
        if [[ "$current_target" != "${SCRIPT_DIR}/inputrc" ]]; then
            printf "${YELLOW_TXT}Replacing existing symlink from ~/.inputrc to ${current_target} with our symlink${NC}\n"
        fi
    fi

    # Create symbolic link (will replace existing symlink if it exists)
    printf "${BLUE_TXT}Linking ROS2-Hacks inputrc file...${NC}\n"
    ln -sf "${SCRIPT_DIR}/inputrc" "$HOME/.inputrc"

    # Only run bind if we're in an interactive shell
    if [[ $- == *i* ]]; then
        bind -f $HOME/.inputrc 2>/dev/null || true
    fi
}

# Create initial config files
create_initial_configs() {
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
    if [ -f "$SCRIPT_DIR/defaults.yaml" ]; then
        cp "$SCRIPT_DIR/defaults.yaml" "$HOME/.colcon/"
    elif [ -f "/usr/share/ros-hacks/defaults.yaml" ]; then
        cp "/usr/share/ros-hacks/defaults.yaml" "$HOME/.colcon/"
    else
        printf "${YELLOW_TXT}Warning: Could not find defaults.yaml${NC}\n"
    fi
    printf "${GREEN_TXT}Planted defaults.yaml in ~/.colcon directory${NC}\n"
}

# Main setup function
main() {
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
    printf "${BLUE_TXT}Use ${WHITE_TXT}F3${BLUE_TXT} to select a ROS workspace.${NC}\n"
    printf "${BLUE_TXT}Use ${WHITE_TXT}Shift+F3${BLUE_TXT} to create a new ROS workspace.${NC}\n"
}

# Run the main setup
main
