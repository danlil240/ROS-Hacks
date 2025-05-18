#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks Updater Script
# ==========================================================

# Get script directory
DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Read version from VERSION file
if [[ -f "${DIR}/VERSION" ]]; then
    VERSION=$(cat "${DIR}/VERSION")
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

# Get the script directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

function update_ros_hacks() {
    printf "${BLUE_TXT}Updating ROS-Hacks from repository...${NC}\n"
    
    # Check if the directory is a git repository
    if [[ ! -d "$DIR/.git" ]]; then
        printf "${RED_TXT}Error: Directory is not a git repository.${NC}\n"
        printf "${YELLOW_TXT}Please reinstall ROS-Hacks from the official repository.${NC}\n"
        exit 1
    fi
    
    # Create backup
    BACKUP_DIR="$DIR/backups/$(date +"%Y%m%d_%H%M%S")"
    printf "${BLUE_TXT}Creating backup in ${BACKUP_DIR}...${NC}\n"
    mkdir -p "$BACKUP_DIR"
    cp -r "$DIR"/* "$BACKUP_DIR" 2>/dev/null || true
    
    # Save current git hash for version tracking
    OLD_VERSION=$(git -C "$DIR" rev-parse HEAD)
    
    # Stash any local changes
    printf "${BLUE_TXT}Saving local changes...${NC}\n"
    HAS_CHANGES=$(git -C "$DIR" status --porcelain | wc -l)
    if [[ $HAS_CHANGES -gt 0 ]]; then
        git -C "$DIR" stash
        STASHED=true
    else
        STASHED=false
    fi
    
    # Pull latest changes
    printf "${BLUE_TXT}Pulling latest updates...${NC}\n"
    if ! git -C "$DIR" pull; then
        printf "${RED_TXT}Error: Failed to pull changes from repository.${NC}\n"
        printf "${YELLOW_TXT}Attempting to restore from stash if applicable...${NC}\n"
        if [[ $STASHED == true ]]; then
            git -C "$DIR" stash pop
        fi
        exit 1
    fi
    
    # Apply stashed changes if any
    if [[ $STASHED == true ]]; then
        printf "${BLUE_TXT}Applying your local customizations...${NC}\n"
        if ! git -C "$DIR" stash pop; then
            printf "${YELLOW_TXT}Warning: There were conflicts applying your local changes.${NC}\n"
            printf "${YELLOW_TXT}Please resolve them manually. Your changes are still in the stash.${NC}\n"
        fi
    fi
    
    # Get new version hash
    NEW_VERSION=$(git -C "$DIR" rev-parse HEAD)
    
    # Save update information
    printf "${BLUE_TXT}Recording update information...${NC}\n"
    echo "Updated: $(date)" > "$DIR/LastUpdated"
    echo "Previous version: $OLD_VERSION" >> "$DIR/LastUpdated"
    echo "Current version: $NEW_VERSION" >> "$DIR/LastUpdated"
    
    # Check if version changed
    if [[ "$OLD_VERSION" == "$NEW_VERSION" ]]; then
        printf "${GREEN_TXT}ROS-Hacks is already up to date.${NC}\n"
    else
        printf "${GREEN_TXT}ROS-Hacks updated successfully!${NC}\n"
        printf "${BLUE_TXT}To apply changes, please run: source ~/.bashrc${NC}\n"
    fi
}

# Change to script directory
cd "$DIR"

# Run update function
update_ros_hacks

# Exit successfully
exit 0