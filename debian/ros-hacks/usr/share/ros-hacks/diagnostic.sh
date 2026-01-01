#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks Diagnostics Tool
# ==========================================================

# Get script directory
SCRIPT_DIR="$(cd -P "$(dirname "$(readlink -f "$0")")" >/dev/null 2>&1 && pwd)"

# Read version from VERSION file
if [[ -f "${SCRIPT_DIR}/VERSION" ]]; then
    VERSION=$(cat "${SCRIPT_DIR}/VERSION")
else
    VERSION="unknown"
fi

# Define colors for output
NC='\033[0m'
GREEN_TXT='\e[0;32m'
RED_TXT='\e[31m'
YELLOW_TXT='\e[93m'
BLUE_TXT='\e[34m'
LIGHT_BLUE_TXT='\e[96m'
WHITE_TXT='\e[1;37m'

# Print header
function print_header() {
    printf "\n${LIGHT_BLUE_TXT}===============================${NC}\n"
    printf "${LIGHT_BLUE_TXT}ROS-Hacks Diagnostics Tool${NC}\n"
    printf "${LIGHT_BLUE_TXT}===============================${NC}\n\n"
}

# Check zsh configuration
function check_zsh_config() {
    printf "${BLUE_TXT}Checking zsh configuration...${NC}\n"

    if [[ -f "$HOME/.zshrc" ]] && grep -q "ROS-HACKS entries" "$HOME/.zshrc"; then
        printf "  ${GREEN_TXT}✓ ROS-Hacks entries found in .zshrc${NC}\n"
    else
        printf "  ${YELLOW_TXT}⚠ ROS-Hacks entries not found in .zshrc${NC}\n"
        printf "    ${YELLOW_TXT}Suggestion: Run setup.sh again or manually add source line${NC}\n"
    fi
}

# Check if file exists
function check_file() {
    local file="$1"
    local desc="$2"

    printf "${BLUE_TXT}Checking ${desc}...${NC} "
    if [[ -f "$file" ]]; then
        printf "${GREEN_TXT}✓ Found${NC}\n"
        return 0
    else
        printf "${RED_TXT}✗ Not found${NC}\n"
        return 1
    fi
}

# Check if directory exists
function check_dir() {
    local dir="$1"
    local desc="$2"

    printf "${BLUE_TXT}Checking ${desc}...${NC} "
    if [[ -d "$dir" ]]; then
        printf "${GREEN_TXT}✓ Found${NC}\n"
        return 0
    else
        printf "${RED_TXT}✗ Not found${NC}\n"
        return 1
    fi
}

# Check ROS workspace
function check_workspace() {
    local ws_file="$HOME/.cache/ros-hacks/ros_ws_selected"

    printf "${BLUE_TXT}Checking ROS workspace configuration...${NC}\n"
    if [[ -f "$ws_file" ]]; then
        local ws=$(cat "$ws_file" 2>/dev/null || echo "")
        if [[ -z "$ws" ]]; then
            printf "  ${YELLOW_TXT}⚠ No workspace currently selected${NC}\n"
        elif [[ ! -d "$ws" ]]; then
            printf "  ${RED_TXT}✗ Selected workspace doesn't exist: ${WHITE_TXT}${ws}${NC}\n"
        elif [[ ! -d "$ws/install" || ( ! -f "$ws/install/setup.bash" && ! -f "$ws/install/setup.zsh" ) ]]; then
            printf "  ${YELLOW_TXT}⚠ Selected workspace may not be properly built${NC}\n"
            printf "    ${BLUE_TXT}Workspace path:${NC} ${ws}\n"
        else
            printf "  ${GREEN_TXT}✓ Workspace is valid:${NC} ${ws}\n"
        fi
    else
        printf "  ${YELLOW_TXT}⚠ No workspace file found. You'll need to create or select a workspace.${NC}\n"
    fi
}

# Check ROS domain ID
function check_domain_id() {
    local domain_file="$HOME/.cache/ros-hacks/ros_domain_id"

    printf "${BLUE_TXT}Checking ROS domain ID configuration...${NC}\n"
    if [[ -f "$domain_file" ]]; then
        local domain=$(cat "$domain_file" 2>/dev/null || echo "")
        if [[ -z "$domain" ]]; then
            printf "  ${YELLOW_TXT}⚠ Domain ID file exists but is empty${NC}\n"
        else
            printf "  ${GREEN_TXT}✓ Domain ID is set to:${NC} ${domain}\n"
        fi
    else
        printf "  ${YELLOW_TXT}⚠ No domain ID file found. Default (0) will be used.${NC}\n"
    fi

    # Check if ROS_DOMAIN_ID environment variable is set
    if [[ -n "$ROS_DOMAIN_ID" ]]; then
        printf "  ${GREEN_TXT}✓ ROS_DOMAIN_ID environment variable set to:${NC} ${ROS_DOMAIN_ID}\n"
    else
        printf "  ${YELLOW_TXT}⚠ ROS_DOMAIN_ID environment variable not set${NC}\n"
    fi
}

# Check bash configuration
function check_bash_config() {
    printf "${BLUE_TXT}Checking bash configuration...${NC}\n"

    if grep -q "ROS-HACKS entries" "$HOME/.bashrc"; then
        printf "  ${GREEN_TXT}✓ ROS-Hacks entries found in .bashrc${NC}\n"
    else
        printf "  ${RED_TXT}✗ ROS-Hacks entries not found in .bashrc${NC}\n"
        printf "    ${YELLOW_TXT}Suggestion: Run setup.sh again or manually add source line${NC}\n"
    fi
}

# Check tmux installation
function check_dependencies() {
    printf "${BLUE_TXT}Checking dependencies...${NC}\n"

    # Check for tmux
    if command -v tmux &>/dev/null; then
        printf "  ${GREEN_TXT}✓ tmux is installed${NC}\n"
    else
        printf "  ${RED_TXT}✗ tmux is not installed${NC}\n"
        printf "    ${YELLOW_TXT}Suggestion: Install with 'sudo apt install tmux'${NC}\n"
    fi
}

# Fix common problems
function fix_problems() {
    printf "\n${BLUE_TXT}Attempting to fix common problems...${NC}\n"

    # Fix .bashrc if needed
    if ! grep -q "ROS-HACKS entries" "$HOME/.bashrc"; then
        printf "  ${BLUE_TXT}Adding ROS-Hacks to .bashrc...${NC} "
        cat <<'EOF' >>"$HOME/.bashrc"

## ROS-HACKS entries ##
if [[ -f "$HOME/.ROS-Hacks/ROS-Hacks.sh" ]]; then
    source "$HOME/.ROS-Hacks/ROS-Hacks.sh"
fi
## ROS-HACKS END ##
EOF
        printf "${GREEN_TXT}Done${NC}\n"
    fi

    # Fix .zshrc if needed
    if [[ -f "$HOME/.zshrc" ]] && ! grep -q "ROS-HACKS entries" "$HOME/.zshrc"; then
        printf "  ${BLUE_TXT}Adding ROS-Hacks to .zshrc...${NC} "
        cat <<'EOF' >>"$HOME/.zshrc"

## ROS-HACKS entries ##
if [[ -f "/usr/share/ros-hacks/ROS-Hacks.zsh" ]]; then
    source "/usr/share/ros-hacks/ROS-Hacks.zsh"
elif [[ -f "$HOME/.ROS-Hacks/ROS-Hacks.zsh" ]]; then
    source "$HOME/.ROS-Hacks/ROS-Hacks.zsh"
fi
## ROS-HACKS END ##
EOF
        printf "${GREEN_TXT}Done${NC}\n"
    fi

    # Create domain ID file if needed
    if [[ ! -f "$HOME/.cache/ros-hacks/ros_domain_id" ]]; then
        printf "  ${BLUE_TXT}Creating default domain ID file...${NC} "
        echo "0" >"$HOME/.cache/ros-hacks/ros_domain_id"
        printf "${GREEN_TXT}Done${NC}\n"
    fi

    # Fix keyboard shortcuts if needed
    if [[ ! -L "$HOME/.inputrc" ]]; then
        printf "  ${BLUE_TXT}Fixing keyboard shortcuts...${NC} "
        SCRIPT_DIR="$(cd -P "$(dirname "$(readlink -f "$0")")" >/dev/null 2>&1 && pwd)"
        if [[ -f "$HOME/.inputrc" && ! -L "$HOME/.inputrc" ]]; then
            cp "$HOME/.inputrc" "$HOME/.inputrc.bak"
        fi
        ln -sf "${SCRIPT_DIR}/inputrc" "$HOME/.inputrc"
        printf "${GREEN_TXT}Done${NC}\n"
        if command -v bind >/dev/null 2>&1; then
            bind -f "$HOME/.inputrc" 2>/dev/null || true
        fi
    fi

    printf "\n${GREEN_TXT}Fixes applied. Please run 'source ~/.bashrc' (and 'source ~/.zshrc' if applicable) to apply changes.${NC}\n"
}

# Run all checks
function run_diagnostics() {
    print_header

    local script_dir="$(cd -P "$(dirname "$(readlink -f "$0")")" >/dev/null 2>&1 && pwd)"

    # Check core files
    check_file "${script_dir}/ROS-Hacks.sh" "main script"
    check_file "${script_dir}/aliases.sh" "aliases file"
    check_file "${script_dir}/functions.sh" "functions file"
    check_file "${script_dir}/inputrc" "keyboard shortcuts file"

    printf "\n"
    check_workspace
    printf "\n"
    check_domain_id
    printf "\n"
    check_bash_config
    printf "\n"
    check_zsh_config
    printf "\n"
    check_dependencies

    # Ask about fixing problems
    printf "\n${BLUE_TXT}Would you like to attempt to fix common problems? (y/n):${NC} "
    read -r choice
    if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
        fix_problems
    fi

    printf "\n${BLUE_TXT}Diagnostics completed.${NC}\n"
}

# Run the diagnostics
run_diagnostics
