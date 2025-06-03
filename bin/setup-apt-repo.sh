#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - APT Repository Setup Script
# ==========================================================
# Complete utility for managing ROS-Hacks APT repository,
# building packages, and deploying to GitHub Pages

set -e

# Get script directory - handle symlinks properly
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do
    DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
    SOURCE="$(readlink "$SOURCE")"
    [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
SCRIPT_DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
ROSHACKS_DIR="$(dirname "$SCRIPT_DIR")"

# Source core configuration
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
fi

# Source repository functions
if [[ -f "${ROSHACKS_DIR}/src/apt/repository.sh" ]]; then
    source "${ROSHACKS_DIR}/src/apt/repository.sh"
fi

# Set default repository directory
REPO_DIR="${ROSHACKS_DIR}"
BUILD_DIR="${ROSHACKS_DIR}/build"

# Ensure build directory exists
mkdir -p "$BUILD_DIR"

# Function to increment version number
function increment_version() {
    printf "${BLUE_TXT}Incrementing version number...${NC}\n"

    # Read current version from VERSION file
    VERSION_FILE="${ROSHACKS_DIR}/VERSION"
    if [[ ! -f "$VERSION_FILE" ]]; then
        echo "1.0.0" > "$VERSION_FILE"
        printf "${YELLOW_TXT}Created initial VERSION file with 1.0.0${NC}\n"
    fi
    
    CURRENT_VERSION=$(cat "$VERSION_FILE")

    # Parse the version components
    MAJOR=$(echo "$CURRENT_VERSION" | cut -d. -f1)
    MINOR=$(echo "$CURRENT_VERSION" | cut -d. -f2)
    PATCH=$(echo "$CURRENT_VERSION" | cut -d. -f3)

    # Increment patch version
    PATCH=$((PATCH + 1))

    # Create new version string
    NEW_VERSION="${MAJOR}.${MINOR}.${PATCH}"
    # For native packages, do not use a Debian revision number
    NEW_DEBIAN_VERSION="${NEW_VERSION}"

    # Update VERSION file
    echo "$NEW_VERSION" >"$VERSION_FILE"

    # Update debian/changelog if it exists
    CHANGELOG_FILE="${ROSHACKS_DIR}/debian/changelog"
    if [[ -f "$CHANGELOG_FILE" ]]; then
        TIMESTAMP=$(date "+%a, %d %b %Y %H:%M:%S %z")

        # Create new changelog entry - write it properly with actual newlines
        cat >"$CHANGELOG_FILE.new" <<EOF
ros-hacks (${NEW_DEBIAN_VERSION}) unstable; urgency=medium

  * Auto-updated version to ${NEW_VERSION}

 -- ROS-Hacks <ros-hacks@example.com>  $TIMESTAMP

EOF

        # Append the existing changelog to the new entry
        cat "$CHANGELOG_FILE" >>"$CHANGELOG_FILE.new"

        # Replace the old changelog with the new one
        mv "$CHANGELOG_FILE.new" "$CHANGELOG_FILE"
    fi

    # Also update the DEBIAN/control file if it exists (generated during build)
    DEBIAN_CONTROL="${ROSHACKS_DIR}/debian/ros-hacks/DEBIAN/control"
    if [ -f "$DEBIAN_CONTROL" ]; then
        # Update the Version field in the DEBIAN/control file
        sed -i "s/^Version: .*/Version: ${NEW_DEBIAN_VERSION}/" "$DEBIAN_CONTROL"
        printf "${GREEN_TXT}Updated version in DEBIAN/control to ${NEW_DEBIAN_VERSION}${NC}\n"
    fi

    printf "${GREEN_TXT}Version updated from $CURRENT_VERSION to $NEW_VERSION${NC}\n"
}

# Create and initialize APT repository structure
function init_repository() {
    local repo_dir=${1:-$(pwd)}

    printf "${BLUE_TXT}Initializing APT repository structure at ${WHITE_TXT}${repo_dir}${NC}\n"

    # Create the necessary directory structure
    mkdir -p "${repo_dir}/dists/stable/main/binary-amd64"
    mkdir -p "${repo_dir}/dists/stable/main/binary-i386"
    mkdir -p "${repo_dir}/pool/main"

    # Create empty Packages files for both architectures to prevent warnings
    touch "${repo_dir}/dists/stable/main/binary-amd64/Packages"
    touch "${repo_dir}/dists/stable/main/binary-i386/Packages"

    # Compress the Packages files
    gzip -9 -c "${repo_dir}/dists/stable/main/binary-amd64/Packages" >"${repo_dir}/dists/stable/main/binary-amd64/Packages.gz"
    gzip -9 -c "${repo_dir}/dists/stable/main/binary-i386/Packages" >"${repo_dir}/dists/stable/main/binary-i386/Packages.gz"

    printf "${GREEN_TXT}Repository structure created successfully.${NC}\n"
}

# Generate the Release file with proper metadata
function generate_release_file() {
    local repo_dir=${1:-$(pwd)}

    printf "${BLUE_TXT}Generating Release file...${NC}\n"

    # Create Release file with proper metadata
    cat >"${repo_dir}/dists/stable/Release" <<EOF
Origin: ROS-Hacks
Label: ROS-Hacks
Suite: stable
Codename: stable
Version: 1.0
Architectures: amd64 i386
Components: main
Description: ROS-Hacks APT Repository
Date: $(date -R)
EOF

    # Generate hash entries for the Release file
    (
        cd "${repo_dir}/dists/stable"
        echo "MD5Sum:" >>Release
        for f in $(find . -type f | grep -v "Release"); do
            echo " $(md5sum $f | cut -d' ' -f1) $(stat -c%s $f) $f" >>Release
        done
        echo "SHA1:" >>Release
        for f in $(find . -type f | grep -v "Release"); do
            echo " $(sha1sum $f | cut -d' ' -f1) $(stat -c%s $f) $f" >>Release
        done
        echo "SHA256:" >>Release
        for f in $(find . -type f | grep -v "Release"); do
            echo " $(sha256sum $f | cut -d' ' -f1) $(stat -c%s $f) $f" >>Release
        done
    )

    printf "${GREEN_TXT}Release file generated successfully.${NC}\n"
}

# Generate GPG key for signing the repository
function generate_gpg_key() {
    local repo_dir=${1:-$(pwd)}

    printf "${BLUE_TXT}Generating GPG key for repository signing...${NC}\n"

    # Check if a key already exists
    if gpg --list-keys "ROS-Hacks" &>/dev/null; then
        printf "${YELLOW_TXT}GPG key for ROS-Hacks already exists. Using existing key.${NC}\n"
    else
        # Generate a new GPG key
        gpg --batch --gen-key <<EOF
Key-Type: RSA
Key-Length: 4096
Name-Real: ROS-Hacks
Name-Email: ros-hacks@example.com
Expire-Date: 0
%no-protection
%commit
EOF
        printf "${GREEN_TXT}GPG key generated successfully.${NC}\n"
    fi

    # Export public key
    mkdir -p "${repo_dir}/config/keys"
    gpg --armor --export "ROS-Hacks" >"${repo_dir}/config/keys/ros-hacks.key"
    printf "${GREEN_TXT}GPG key exported to ${WHITE_TXT}${repo_dir}/config/keys/ros-hacks.key${NC}\n"
}

# Sign the Release file
function sign_release_file() {
    local repo_dir=${1:-$(pwd)}

    printf "${BLUE_TXT}Signing Release file...${NC}\n"

    # Sign the Release file
    gpg --clearsign -o "${repo_dir}/dists/stable/InRelease" "${repo_dir}/dists/stable/Release"
    gpg -abs -o "${repo_dir}/dists/stable/Release.gpg" "${repo_dir}/dists/stable/Release"

    printf "${GREEN_TXT}Release file signed successfully.${NC}\n"
}

# Add a Debian package to the repository
function add_package() {
    local pkg_file=$1
    local repo_dir=${2:-$REPO_DIR}

    if [[ -z "$pkg_file" ]]; then
        printf "${RED_TXT}No package file specified!${NC}\n"
        return 1
    fi

    if [[ ! -f "$pkg_file" ]]; then
        printf "${RED_TXT}Package file not found: $pkg_file${NC}\n"
        return 1
    fi

    # Copy package to repo
    printf "${BLUE_TXT}Adding package $pkg_file to repository...${NC}\n"
    
    # Create pool/main directory if it doesn't exist
    mkdir -p "${repo_dir}/pool/main"
    
    # Copy the package
    cp "$pkg_file" "${repo_dir}/pool/main/"
    
    # Update repository index
    update_repo "$repo_dir"
    
    printf "${GREEN_TXT}Package added to repository successfully!${NC}\n"
    sign_release_file "${repo_dir}"

    printf "${GREEN_TXT}Package added successfully.${NC}\n"
}

# Print installation instructions
function print_instructions() {
    # Try to get GitHub username from git config
    local github_user=$(git config --get remote.origin.url | sed -n 's|.*github.com[:/]\([^/]*\)/.*|\1|p')
    if [[ -z "$github_user" ]]; then
        github_user="danlil240"
    fi
    
    local repo_url="https://${github_user}.github.io/ROS-Hacks"

    printf "\n${BLUE_TXT}APT Repository Installation Instructions:${NC}\n"
    printf "${WHITE_TXT}To use this repository, run the following commands:${NC}\n\n"
    printf "wget -O /tmp/ros-hacks.key ${repo_url}/config/keys/ros-hacks.key\n"
    printf "sudo mkdir -p /etc/apt/keyrings\n"
    printf "sudo gpg --dearmor -o /etc/apt/keyrings/ros-hacks.gpg /tmp/ros-hacks.key\n"
    printf "echo \"deb [signed-by=/etc/apt/keyrings/ros-hacks.gpg] ${repo_url} stable main\" | sudo tee /etc/apt/sources.list.d/ros-hacks.list\n"
    printf "sudo apt update\n"
    printf "sudo apt install ros-hacks\n"
    printf "\n"
    
    printf "${YELLOW_TXT}Note: Make sure GitHub Pages is enabled in your repository settings:${NC}\n"
    printf "${YELLOW_TXT}1. Go to https://github.com/$github_user/ROS-Hacks/settings/pages${NC}\n"
    printf "${YELLOW_TXT}2. Set source to 'main' branch and '/' folder${NC}\n"
    printf "${YELLOW_TXT}3. It may take a few minutes for the pages to be published${NC}\n"
}

# Build the Debian package
function build_package() {
    printf "${BLUE_TXT}Building ROS-Hacks Debian package...${NC}\n"
    
    # Check if debuild is installed
    if ! command -v debuild &> /dev/null; then
        printf "${YELLOW_TXT}debuild not found. Installing build dependencies...${NC}\n"
        sudo apt update && sudo apt install -y devscripts debhelper
    fi
    
    # Check if pbuilder is installed
    if ! command -v pbuilder &> /dev/null; then
        printf "${YELLOW_TXT}pbuilder not found. Installing...${NC}\n"
        sudo apt update && sudo apt install -y pbuilder
    fi
    
    # Increment version
    increment_version
    
    # Clean old files
    printf "${BLUE_TXT}Cleaning old build files...${NC}\n"
    
    # Remove old .deb and related files
    rm -f "${ROSHACKS_DIR}"/../ros-hacks_*.deb
    rm -f "${ROSHACKS_DIR}"/../ros-hacks_*.changes
    rm -f "${ROSHACKS_DIR}"/../ros-hacks_*.buildinfo
    rm -f "${ROSHACKS_DIR}"/../ros-hacks_*.dsc
    rm -f "${ROSHACKS_DIR}"/../ros-hacks_*.tar.xz
    
    # Build the source package
    printf "${BLUE_TXT}Building source package...${NC}\n"
    cd "${ROSHACKS_DIR}"
    debuild -S -us -uc -i
    
    if [ $? -ne 0 ]; then
        printf "${RED_TXT}Failed to build source package. Aborting.${NC}\n"
        return 1
    fi
    
    # Build the binary package using pbuilder
    printf "${BLUE_TXT}Building binary package with pbuilder...${NC}\n"
    sudo pbuilder build --buildresult "$BUILD_DIR" "${ROSHACKS_DIR}"/../ros-hacks_*.dsc
    
    if [ $? -ne 0 ]; then
        printf "${RED_TXT}Failed to build binary package. Aborting.${NC}\n"
        return 1
    fi
    
    printf "${GREEN_TXT}Package built successfully!${NC}\n"
    printf "${BLUE_TXT}Built package:${NC}\n"
    ls -la "$BUILD_DIR"/ros-hacks_*.deb
    
    # Ask if user wants to add the package to the repository
    printf "${BLUE_TXT}Do you want to add this package to the repository? [y/N] ${NC}"
    read -r ADD_TO_REPO
    
    if [[ "$ADD_TO_REPO" == "y" || "$ADD_TO_REPO" == "Y" ]]; then
        local pkg_file=$(ls -1 "$BUILD_DIR"/ros-hacks_*.deb | head -n 1)
        add_package "$pkg_file"
    fi
}

# Setup GitHub repository
function setup_github() {
    printf "${BLUE_TXT}Setting up GitHub repository...${NC}\n"
    
    # Check if git is installed
    if ! command -v git &> /dev/null; then
        printf "${YELLOW_TXT}git not found. Installing...${NC}\n"
        sudo apt update && sudo apt install -y git
    fi
    
    # Ask for GitHub username if not already configured
    local GITHUB_USER=$(git config --get remote.origin.url | sed -n 's|.*github.com[:/]\([^/]*\)/.*|\1|p')
    
    if [[ -z "$GITHUB_USER" ]]; then
        printf "${BLUE_TXT}Enter your GitHub username: ${NC}"
        read -r GITHUB_USER
        
        if [[ -z "$GITHUB_USER" ]]; then
            printf "${RED_TXT}GitHub username is required. Aborting.${NC}\n"
            return 1
        fi
    fi
    
    # Initialize git repository if needed
    cd "$REPO_DIR"
    
    if [[ ! -d ".git" ]]; then
        printf "${BLUE_TXT}Initializing git repository...${NC}\n"
        git init
        git branch -M main
        
        # Create .gitignore
        cat >".gitignore" <<EOF
*.deb
*.changes
*.buildinfo
*.dsc
*.tar.xz
*.o
*.so
old/
build/
dist/
__pycache__/
EOF
        
        printf "${GREEN_TXT}Local git repository initialized${NC}\n"
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
    
    printf "${YELLOW_TXT}Remote 'origin' configured to: https://github.com/$GITHUB_USER/ROS-Hacks.git${NC}\n"
    
    # Get the current branch name
    BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD)
    
    printf "${BLUE_TXT}Do you want to push to GitHub now? [y/N] ${NC}"
    read -r PUSH_NOW
    
    if [[ "$PUSH_NOW" = "y" || "$PUSH_NOW" = "Y" ]]; then
        printf "${BLUE_TXT}Pushing to GitHub...${NC}\n"
        git push -u origin $BRANCH_NAME
        if [ $? -eq 0 ]; then
            printf "${GREEN_TXT}Successfully pushed to GitHub${NC}\n"
            printf "${YELLOW_TXT}Remember to enable GitHub Pages in repository settings:${NC}\n"
            printf "${YELLOW_TXT}1. Go to https://github.com/$GITHUB_USER/ROS-Hacks/settings/pages${NC}\n"
            printf "${YELLOW_TXT}2. Set source to 'main' branch and '/' folder${NC}\n"
            printf "${YELLOW_TXT}3. Click Save${NC}\n"
        else
            printf "${RED_TXT}Failed to push to GitHub. You can push manually:${NC}\n"
            printf "cd $REPO_DIR\n"
            printf "git push -u origin main\n"
        fi
    else
        printf "${YELLOW_TXT}You can push manually with:${NC}\n"
        printf "cd $REPO_DIR\n"
        printf "git push -u origin main\n"
    fi
}

# Create instructions file in repository
function create_instructions() {
    printf "${BLUE_TXT}Creating installation instructions README...${NC}\n"
    
    # Try to get GitHub username from git config
    local github_user=$(git config --get remote.origin.url | sed -n 's|.*github.com[:/]\([^/]*\)/.*|\1|p')
    if [[ -z "$github_user" ]]; then
        github_user="danlil240"
    fi
    
    cat >"${REPO_DIR}/INSTALL.md" <<EOF
# Installing ROS-Hacks

This guide will help you install ROS-Hacks from the APT repository.

## Installation Steps

1. Add the GPG key:
\`\`\`bash
wget -O /tmp/ros-hacks.key https://${github_user}.github.io/ROS-Hacks/config/keys/ros-hacks.key
sudo mkdir -p /etc/apt/keyrings
sudo gpg --dearmor -o /etc/apt/keyrings/ros-hacks.gpg /tmp/ros-hacks.key
\`\`\`

2. Add the repository to your sources:
\`\`\`bash
echo "deb [signed-by=/etc/apt/keyrings/ros-hacks.gpg] https://${github_user}.github.io/ROS-Hacks stable main" | sudo tee /etc/apt/sources.list.d/ros-hacks.list
\`\`\`

3. Update and install:
\`\`\`bash
sudo apt update
sudo apt install ros-hacks
\`\`\`

## Verification

After installation, you can verify the installation by running:

\`\`\`bash
source /usr/share/ros-hacks/bin/ros-hacks.sh
\`\`\`

Or if you have sourced your .bashrc:

\`\`\`bash
source ~/.bashrc
\`\`\`

Then try some commands like:

\`\`\`bash
wslist    # List available workspaces
domain    # Show current ROS domain ID
\`\`\`
EOF
    
    printf "${GREEN_TXT}Created installation instructions at ${REPO_DIR}/INSTALL.md${NC}\n"
}

# Main function
function main() {
    local repo_dir=${1:-$(pwd)}

    printf "${LIGHT_BLUE_TXT}===============================${NC}\n"
    printf "${LIGHT_BLUE_TXT}ROS-Hacks APT Repository Setup${NC}\n"
    printf "${LIGHT_BLUE_TXT}===============================${NC}\n\n"

    # Initialize repository structure
    init_repository "${repo_dir}"

    # Generate Release file
    generate_release_file "${repo_dir}"

    # Generate GPG key
    generate_gpg_key "${repo_dir}"

    # Sign Release file
    sign_release_file "${repo_dir}"

    printf "\n${GREEN_TXT}APT repository setup complete!${NC}\n"

    # Print installation instructions
    print_instructions "https://danlil240.github.io/ROS-Hacks"
}

# Show usage
function show_usage() {
    printf "Usage: ${0} [OPTIONS]\n"
    printf "Options:\n"
    printf "  setup            Initialize the APT repository structure\n"
    printf "  add <pkg_file>   Add a Debian package to the repository\n"
    printf "  update           Update repository index files\n"
    printf "  build            Build the Debian package\n"
    printf "  github           Setup and push to GitHub repository\n"
    printf "  instructions     Print installation instructions\n"
    printf "  create-docs      Create installation documentation file\n"
    printf "  all              Run complete setup process (update, build, github)\n"
    printf "  help             Show this help message\n"
}

# Parse command line arguments
if [[ $# -eq 0 ]]; then
    main
else
    case "${1}" in
    setup)
        init_repository
        generate_release_file
        generate_gpg_key
        sign_release_file
        ;;
    add)
        if [[ $# -lt 2 ]]; then
            printf "${RED_TXT}Error: Package file not specified.${NC}\n"
            show_usage
            exit 1
        fi
        add_package "${2}"
        ;;
    update)
        update_repo "$REPO_DIR"
        ;;
    build)
        build_package
        ;;
    github)
        setup_github
        print_instructions
        ;;
    create-docs)
        create_instructions
        ;;
    all)
        printf "${BLUE_TXT}Running complete setup process...${NC}\n"
        update_repo "$REPO_DIR"
        create_instructions
        build_package
        setup_github
        print_instructions
        ;;
    instructions)
        print_instructions
        ;;
    help)
        show_usage
        ;;
    *)
        printf "${RED_TXT}Error: Unknown command ${1}${NC}\n"
        show_usage
        exit 1
        ;;
    esac
fi
