#!/bin/bash

# Setup a personal APT repository for ROS-Hacks
# This script helps you set up a GitHub Pages based APT repository

set -e

# Define colors for output
NC='\033[0m'
GREEN='\e[0;32m'
YELLOW='\e[93m'
BLUE='\e[34m'

# Check if GPG is installed
if ! command -v gpg &> /dev/null; then
    echo -e "${YELLOW}GPG is not installed. Installing...${NC}"
    sudo apt update && sudo apt install -y gnupg
fi

# Check if required packages are installed
if ! command -v dpkg-scanpackages &> /dev/null; then
    echo -e "${YELLOW}dpkg-dev is not installed. Installing...${NC}"
    sudo apt update && sudo apt install -y dpkg-dev
fi

# Setup repository directory
REPO_DIR="$HOME/ros-hacks-apt-repo"
KEY_NAME="ros-hacks-apt-key"

echo -e "${BLUE}Setting up APT repository in $REPO_DIR${NC}"
mkdir -p "$REPO_DIR"/{pool/main,dists/stable/{main/binary-amd64,Release.gpg}}

# Generate GPG key if needed
if ! gpg --list-keys | grep -q "$KEY_NAME"; then
    echo -e "${BLUE}Generating GPG key for signing packages...${NC}"
    
    # Create key configuration file
    cat > /tmp/gpg-key-gen.conf << EOF
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
    gpg --armor --export "ROS-Hacks APT Repository" > "$REPO_DIR/ros-hacks-apt.key"
    echo -e "${GREEN}GPG key generated and exported to $REPO_DIR/ros-hacks-apt.key${NC}"
fi

# Function to update the repository
update_repo() {
    echo -e "${BLUE}Updating repository index...${NC}"
    
    # Create Packages file
    cd "$REPO_DIR"
    dpkg-scanpackages --multiversion pool/ > dists/stable/main/binary-amd64/Packages
    gzip -k -f dists/stable/main/binary-amd64/Packages
    
    # Create Release file
    cat > dists/stable/Release << EOF
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
    cat > "$REPO_DIR/README.md" << EOF
# ROS-Hacks APT Repository

This is a personal APT repository for the ROS-Hacks package.

## Adding this repository to your system

1. Add the GPG key:
\`\`\`bash
wget -qO - https://your-github-username.github.io/ros-hacks-apt-repo/ros-hacks-apt.key | sudo apt-key add -
\`\`\`

2. Add the repository to your sources:
\`\`\`bash
echo "deb https://your-github-username.github.io/ros-hacks-apt-repo stable main" | sudo tee /etc/apt/sources.list.d/ros-hacks.list
\`\`\`

3. Update and install:
\`\`\`bash
sudo apt update
sudo apt install ros-hacks
\`\`\`

## Publishing Instructions

1. Build the package:
\`\`\`bash
cd ros-hacks-apt
dpkg-buildpackage -us -uc -b
\`\`\`

2. Add the built package to the repository:
\`\`\`bash
scripts/setup-apt-repo.sh add ../ros-hacks_*.deb
\`\`\`

3. Push the repository to GitHub:
\`\`\`bash
cd ~/ros-hacks-apt-repo
git add .
git commit -m "Update repository"
git push
\`\`\`
EOF
    
    echo -e "${GREEN}Instructions created in $REPO_DIR/README.md${NC}"
}

# Main logic
if [ "$1" = "add" ] && [ -n "$2" ]; then
    add_package "$2"
elif [ "$1" = "update" ]; then
    update_repo
else
    echo -e "${BLUE}Initial repository setup...${NC}"
    update_repo
    create_instructions
    
    echo -e "${GREEN}Repository setup complete!${NC}"
    echo -e "${YELLOW}Next steps:${NC}"
    echo -e "1. Initialize the repo directory as a git repository"
    echo -e "2. Create a GitHub repository named 'ros-hacks-apt-repo'"
    echo -e "3. Enable GitHub Pages for the repository (set to root)"
    echo -e "4. Push your local repository to GitHub"
    echo -e "5. Follow the instructions in $REPO_DIR/README.md to add the repository to your system"
fi
