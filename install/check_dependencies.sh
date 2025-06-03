#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - Dependency Checking
# ==========================================================

# Check for required dependencies
function check_dependencies() {
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
