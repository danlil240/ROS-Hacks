#!/bin/bash

# Setup a personal APT repository for ROS-Hacks
# This script helps you set up a GitHub Pages based APT repository,
# build the package, push to GitHub, and provide installation instructions

set -e

# Define colors for output
NC='\033[0m'
GREEN='\e[0;32m'
YELLOW='\e[93m'
BLUE='\e[34m'
RED='\e[31m'

# Check if GPG is installed
if ! command -v gpg &>/dev/null; then
    echo -e "${YELLOW}GPG is not installed. Installing...${NC}"
    sudo apt update && sudo apt install -y gnupg
fi

# Check if required packages are installed
if ! command -v dpkg-scanpackages &>/dev/null; then
    echo -e "${YELLOW}dpkg-dev is not installed. Installing...${NC}"
    sudo apt update && sudo apt install -y dpkg-dev
fi

# Setup repository directory
REPO_DIR="$HOME/ros-hacks"
KEY_NAME="ros-hacks-key"
SOURCE_DIR="$HOME/ros-hacks"

echo -e "${BLUE}Setting up APT repository in $REPO_DIR${NC}"
# Clean up any existing directory that should be a file
if [ -d "$REPO_DIR/dists/stable/Release.gpg" ]; then
    rm -rf "$REPO_DIR/dists/stable/Release.gpg"
fi
if [ -d "$REPO_DIR/dists/stable/InRelease" ]; then
    rm -rf "$REPO_DIR/dists/stable/InRelease"
fi
mkdir -p "$REPO_DIR"/{pool/main,dists/stable/main/binary-amd64}

# Generate GPG key if needed
if ! gpg --list-keys | grep -q "$KEY_NAME"; then
    echo -e "${BLUE}Generating GPG key for signing packages...${NC}"

    # Create key configuration file
    cat >/tmp/gpg-key-gen.conf <<EOF
Key-Type: RSA
Key-Length: 4096
Name-Real: ROS-Hacks APT Repository
Name-Email: your-email@example.com
Expire-Date: 0
%no-protection
%commit
EOF

    # Generate key
    gpg --batch --gen-key /tmp/gpg-key-gen.conf
    rm /tmp/gpg-key-gen.conf

    # Export public key
    gpg --armor --export "ROS-Hacks APT Repository" >"$REPO_DIR/ros-hacks.key"
    echo -e "${GREEN}GPG key generated and exported to $REPO_DIR/ros-hacks.key${NC}"
fi

# Function to update the repository
update_repo() {
    echo -e "${BLUE}Updating repository index...${NC}"

    # Create Packages file
    cd "$REPO_DIR"
    dpkg-scanpackages --multiversion pool/ >dists/stable/main/binary-amd64/Packages
    gzip -k -f dists/stable/main/binary-amd64/Packages

    # Create Release file
    cat >dists/stable/Release <<EOF
Origin: ROS-Hacks Repository
Label: ROS-Hacks
Suite: stable
Codename: stable
Architectures: amd64
Components: main
Description: ROS-Hacks APT Repository
Date: $(date -R)
EOF

    # Sign Release file
    rm -f dists/stable/Release.gpg dists/stable/InRelease
    gpg --default-key "ROS-Hacks APT Repository" -abs -o dists/stable/Release.gpg dists/stable/Release
    gpg --default-key "ROS-Hacks APT Repository" --clearsign -o dists/stable/InRelease dists/stable/Release

    echo -e "${GREEN}Repository index updated and signed${NC}"
}

# Add packages to repo
add_package() {
    local PACKAGE=$1

    if [ ! -f "$PACKAGE" ]; then
        echo -e "${YELLOW}Package file not found: $PACKAGE${NC}"
        return 1
    fi

    echo -e "${BLUE}Adding package to repository: $(basename $PACKAGE)${NC}"
    cp "$PACKAGE" "$REPO_DIR/pool/main/"
    update_repo
    echo -e "${GREEN}Package added successfully${NC}"
}

# Create instructions file
create_instructions() {
    cat >"$REPO_DIR/README.md" <<EOF
# ROS-Hacks APT Repository

This is a personal APT repository for the ROS-Hacks package.

## Adding this repository to your system

1. Add the GPG key:
\`\`\`bash
wget -qO - https://your-github-username.github.io/ros-hacks-repo/ros-hacks.key | sudo apt-key add -
\`\`\`

2. Add the repository to your sources:
\`\`\`bash
echo "deb https://your-github-username.github.io/ros-hacks-repo stable main" | sudo tee /etc/apt/sources.list.d/ros-hacks.list
\`\`\`

3. Update and install:
\`\`\`bash
sudo apt update
sudo apt install ros-hacks
\`\`\`

## Publishing Instructions

1. Build the package:
\`\`\`bash
cd ros-hacks
dpkg-buildpackage -us -uc -b
\`\`\`

2. Add the built package to the repository:
\`\`\`bash
scripts/setup-apt-repo.sh add ../ros-hacks_*.deb
\`\`\`

3. Push the repository to GitHub:
\`\`\`bash
cd ~/ros-hacks-repo
git add .
git commit -m "Update repository"
git push
\`\`\`
EOF

    echo -e "${GREEN}Instructions created in $REPO_DIR/README.md${NC}"
}

# Build the package
build_package() {
    echo -e "${BLUE}Building ROS-Hacks package...${NC}"

    # Install required build dependencies
    echo -e "${BLUE}Installing build dependencies...${NC}"
    sudo apt-get update
    sudo apt-get install -y debhelper debhelper-compat build-essential dh-make

    cd "$SOURCE_DIR"
    dpkg-buildpackage -us -uc -b -d
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Package built successfully${NC}"
        # Find the most recent .deb file
        DEB_FILE=$(find "$HOME" -maxdepth 1 -name "ros-hacks_*.deb" -type f -printf "%T@ %p\n" | sort -n | tail -1 | cut -d' ' -f2-)
        if [ -n "$DEB_FILE" ]; then
            echo -e "${GREEN}Found package: $DEB_FILE${NC}"
            add_package "$DEB_FILE"
        else
            echo -e "${RED}Could not find built .deb package${NC}"
            exit 1
        fi
    else
        echo -e "${RED}Package build failed${NC}"
        exit 1
    fi
}

# Setup GitHub repository
setup_github() {
    echo -e "${BLUE}Setting up GitHub repository...${NC}"

    local GITHUB_USER
    echo -e "${YELLOW}Enter your GitHub username:${NC}"
    read GITHUB_USER

    if [ -z "$GITHUB_USER" ]; then
        echo -e "${RED}GitHub username cannot be empty${NC}"
        return 1
    fi

    # Update README with correct GitHub username
    sed -i "s/your-github-username/$GITHUB_USER/g" "$REPO_DIR/README.md"

    # Initialize git repo if needed
    cd "$REPO_DIR"
    if [ ! -d ".git" ]; then
        git init
        git add .
        git commit -m "Initial repository setup"
        echo -e "${GREEN}Local git repository initialized${NC}"
    else
        git add .
        git commit -m "Update repository"
    fi

    # Check if remote exists
    if git remote | grep -q "origin"; then
        git remote set-url origin "https://github.com/$GITHUB_USER/ROS-Hacks.git"
    else
        git remote add origin "https://github.com/$GITHUB_USER/ROS-Hacks.git"
    fi

    echo -e "${YELLOW}Remote 'origin' configured to: https://github.com/$GITHUB_USER/ROS-Hacks.git${NC}"

    # Get the current branch name
    BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD)

    # Ask if user wants to push
    echo -e "${YELLOW}Do you want to push to GitHub now? (y/n)${NC}"
    read PUSH_NOW

    if [ "$PUSH_NOW" = "y" ] || [ "$PUSH_NOW" = "Y" ]; then
        echo -e "${BLUE}Pushing to GitHub...${NC}"
        git push -u origin $BRANCH_NAME
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}Successfully pushed to GitHub${NC}"
            echo -e "${YELLOW}Remember to enable GitHub Pages in repository settings:${NC}"
            echo -e "${YELLOW}1. Go to https://github.com/$GITHUB_USER/ROS-Hacks/settings/pages${NC}"
            echo -e "${YELLOW}2. Set source to 'main' branch and '/' folder${NC}"
            echo -e "${YELLOW}3. Click Save${NC}"
        else
            echo -e "${RED}Failed to push to GitHub. You can push manually:${NC}"
            echo -e "cd $REPO_DIR"
            echo -e "git push -u origin main"
        fi
    else
        echo -e "${YELLOW}You can push manually with:${NC}"
        echo -e "cd $REPO_DIR"
        echo -e "git push -u origin main"
    fi

    echo -e "${YELLOW}After pushing, enable GitHub Pages in repository settings (main branch, / folder)${NC}"
}

# Show installation instructions
show_instructions() {
    local GITHUB_USER=$(grep -o 'https://[^/]*/\([^/]*\)' "$REPO_DIR/README.md" | sed 's|https://||g' | head -1)

    echo -e "\n${GREEN}=== INSTALLATION INSTRUCTIONS FOR TARGET MACHINE ===${NC}"
    echo -e "${BLUE}Run these commands on the target machine:${NC}"
    echo -e "wget -qO - https://$GITHUB_USER.github.io/ROS-Hacks/ros-hacks.key | sudo apt-key add -"
    echo -e "echo \"deb https://$GITHUB_USER.github.io/ROS-Hacks stable main\" | sudo tee /etc/apt/sources.list.d/ros-hacks.list"
    echo -e "sudo apt update"
    echo -e "sudo apt install ros-hacks"
}

# Main logic
if [ "$1" = "add" ] && [ -n "$2" ]; then
    add_package "$2"
elif [ "$1" = "update" ]; then
    update_repo
elif [ "$1" = "all" ]; then
    echo -e "${BLUE}Running complete setup process...${NC}"
    update_repo
    create_instructions
    build_package
    setup_github
    show_instructions
elif [ "$1" = "github" ]; then
    setup_github
    show_instructions
elif [ "$1" = "build" ]; then
    build_package
else
    echo -e "${BLUE}Initial repository setup...${NC}"
    update_repo
    create_instructions

    echo -e "${GREEN}Repository setup complete!${NC}"
    echo -e "${YELLOW}Next steps:${NC}"
    echo -e "1. Run '$(basename $0) build' to build the package"
    echo -e "2. Run '$(basename $0) github' to setup and push to GitHub"
    echo -e "   OR"
    echo -e "3. Run '$(basename $0) all' to do everything in one step"
fi
