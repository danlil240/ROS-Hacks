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

# Function to increment version number
increment_version() {
    echo -e "${BLUE}Incrementing version number...${NC}"

    # Use the global SOURCE_DIR that was already set
    VERSION_FILE="${SOURCE_DIR}/VERSION"
    CURRENT_VERSION=$(cat "$VERSION_FILE")

    # Parse the version components
    MAJOR=$(echo "$CURRENT_VERSION" | cut -d. -f1)
    MINOR=$(echo "$CURRENT_VERSION" | cut -d. -f2)
    PATCH=$(echo "$CURRENT_VERSION" | cut -d. -f3)

    # Increment patch version
    PATCH=$((PATCH + 1))

    # Create new version string
    NEW_VERSION="${MAJOR}.${MINOR}.${PATCH}"
    NEW_DEBIAN_VERSION="${NEW_VERSION}-1"

    # Update VERSION file
    echo "$NEW_VERSION" >"$VERSION_FILE"

    # Update debian/changelog
    CHANGELOG_FILE="${SOURCE_DIR}/debian/changelog"
    TIMESTAMP=$(date "+%a, %d %b %Y %H:%M:%S %z")

    # Create new changelog entry - write it properly with actual newlines
    cat >"$CHANGELOG_FILE.new" <<EOF
ros-hacks (${NEW_DEBIAN_VERSION}) unstable; urgency=medium

  * Auto-updated version to ${NEW_VERSION}

 -- Daniel <danlil240@gmail.com>  $TIMESTAMP

EOF

    # Append the existing changelog to the new entry
    cat "$CHANGELOG_FILE" >>"$CHANGELOG_FILE.new"

    # Replace the old changelog with the new one
    mv "$CHANGELOG_FILE.new" "$CHANGELOG_FILE"

    # Also update the DEBIAN/control file if it exists (generated during build)
    DEBIAN_CONTROL="${SOURCE_DIR}/debian/ros-hacks/DEBIAN/control"
    if [ -f "$DEBIAN_CONTROL" ]; then
        # Update the Version field in the DEBIAN/control file
        sed -i "s/^Version: .*/Version: ${NEW_DEBIAN_VERSION}/" "$DEBIAN_CONTROL"
        echo -e "${GREEN}Updated version in DEBIAN/control to ${NEW_DEBIAN_VERSION}${NC}"
    fi

    echo -e "${GREEN}Version updated from $CURRENT_VERSION to $NEW_VERSION${NC}"
}

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
SOURCE_DIR="$(dirname "$(dirname "$(readlink -f "$0")")")"
APT_REPO_DIR="$HOME/ros-hacks-apt"
KEY_NAME="ros-hacks-key"

echo -e "${BLUE}Source directory: $SOURCE_DIR${NC}"
echo -e "${BLUE}APT Repository directory: $APT_REPO_DIR${NC}"

# Create and initialize APT repository if it doesn't exist
if [ ! -d "$APT_REPO_DIR" ]; then
    echo -e "${BLUE}Creating APT repository directory...${NC}"
    mkdir -p "$APT_REPO_DIR"
    cd "$APT_REPO_DIR"
    git init
    echo "# ROS-Hacks APT Repository" > README.md
    echo "This repository contains the APT repository files for ROS-Hacks package distribution." >> README.md
    git add README.md
    git commit -m "Initial APT repository setup"
    git remote add origin "https://github.com/danlil240/ros-hacks-apt.git"
fi

REPO_DIR="$APT_REPO_DIR"

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
if ! gpg --list-keys | grep -q "ROS-Hacks APT Repository"; then
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

    # Create amd64 Packages file
    dpkg-scanpackages --multiversion pool/ >dists/stable/main/binary-amd64/Packages
    gzip -k -f dists/stable/main/binary-amd64/Packages

    # Create empty i386 Packages file to prevent warnings
    mkdir -p "$REPO_DIR/dists/stable/main/binary-i386"
    touch "$REPO_DIR/dists/stable/main/binary-i386/Packages"
    gzip -k -f "$REPO_DIR/dists/stable/main/binary-i386/Packages"

    # Create Release file with proper date format and hashes
    cat >dists/stable/Release <<EOF
Origin: ROS-Hacks Repository
Label: ROS-Hacks
Suite: stable
Codename: stable
Architectures: amd64 i386
Components: main
Description: ROS-Hacks APT Repository
Date: $(date -R -u)
EOF

    # Generate hashes for the Packages files
    echo "SHA256:" >>dists/stable/Release
    cd "$REPO_DIR"
    # Include both amd64 and i386 package files in the hashes
    for f in dists/stable/main/binary-*/Packages*; do
        echo -n " "
        echo -n $(sha256sum $f | cut -d" " -f1)
        echo -n " "
        echo -n $(stat -c%s $f)
        echo " $(echo $f | sed 's|^dists/stable/||')"
    done >>dists/stable/Release

    echo "MD5Sum:" >>dists/stable/Release
    for f in dists/stable/main/binary-*/Packages*; do
        echo -n " "
        echo -n $(md5sum $f | cut -d" " -f1)
        echo -n " "
        echo -n $(stat -c%s $f)
        echo " $(echo $f | sed 's|^dists/stable/||')"
    done >>dists/stable/Release

    # Sign Release file with a deterministic key and export the matching public key
    rm -f dists/stable/Release.gpg dists/stable/InRelease
    
    # Always use the same key by storing/reading the fingerprint
    KEY_FPR_FILE="$REPO_DIR/.signing_key_fpr"
    
    if [ -f "$KEY_FPR_FILE" ]; then
        # Use existing key fingerprint
        SIGNING_KEY=$(cat "$KEY_FPR_FILE")
        echo -e "${BLUE}Using existing signing key: $SIGNING_KEY${NC}"
    else
        # Get the first (and should be only) key for ROS-Hacks APT Repository
        SIGNING_KEY=$(gpg --list-secret-keys --with-colons 'ROS-Hacks APT Repository' 2>/dev/null | awk -F: '/^fpr:/ {print $10; exit}')
        if [ -z "$SIGNING_KEY" ]; then
            echo -e "${RED}No GPG key found for 'ROS-Hacks APT Repository'${NC}"
            exit 1
        fi
        # Save the fingerprint for future use
        echo "$SIGNING_KEY" > "$KEY_FPR_FILE"
        echo -e "${GREEN}Saved signing key fingerprint: $SIGNING_KEY${NC}"
    fi
    
    # Verify the key still exists
    if ! gpg --list-secret-keys "$SIGNING_KEY" >/dev/null 2>&1; then
        echo -e "${RED}Signing key $SIGNING_KEY not found! Removing cached fingerprint.${NC}"
        rm -f "$KEY_FPR_FILE"
        exit 1
    fi
    
    gpg --batch --yes --default-key "$SIGNING_KEY" -abs -o dists/stable/Release.gpg dists/stable/Release
    gpg --batch --yes --default-key "$SIGNING_KEY" --clearsign -o dists/stable/InRelease dists/stable/Release
    # Export public key used for signing so clients fetch the correct key
    gpg --armor --export "$SIGNING_KEY" > "$REPO_DIR/ros-hacks.key"

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
    # We need to use a different heredoc delimiter to avoid issues with Markdown
    # 'EOT' is common for this purpose, and we'll use the '-EOT' form which prevents
    # tab expansion to preserve the exact formatting
    cat >"$REPO_DIR/README.md" <<-'MDCONTENT'
# ROS-Hacks

This is a repository for the ROS-Hacks package, a productivity toolkit that enhances the ROS2 development experience with workspace management tools, convenient aliases, keyboard shortcuts, and utility functions.

## Adding this repository to your system

1. Download and add the GPG key:
```bash
wget -qO /tmp/ros-hacks.key https://danlil240.github.io/ros-hacks-apt/ros-hacks.key
sudo mkdir -p /etc/apt/keyrings
cat /tmp/ros-hacks.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-hacks.gpg
```

2. Add the repository to your sources:
```bash
echo "deb [signed-by=/etc/apt/keyrings/ros-hacks.gpg] https://danlil240.github.io/ros-hacks-apt stable main" | sudo tee /etc/apt/sources.list.d/ros-hacks.list
```

3. Update and install:
```bash
sudo apt update
sudo apt install ros-hacks
```

4. After installation, run the setup script:
```bash
ros-hacks-setup
```

## Features and Usage

### ROS2 Workspace Management

- **Select workspace**: Press `F3` or run `select_ws`
- **Create new workspace**: Press `Shift+F3` or run `prompt_new_ws`
- **Rebuild current workspace**: Press `F9` or run `rebuild_curr_ws`
- **Force CMake reconfigure**: Press `Ctrl+F9` or run `rebuild_curr_ws --cmake-force-configure`
- **Build specific package**: Press `F8`, then enter the package name

### ROS2 Command Shortcuts

- **List topics**: `rtl` or `ros2 topic list` or press `Alt+Ctrl+t`
- **Echo topic**: `rte <topic>` or `ros2 topic echo <topic>`
- **List nodes**: `rnl` or `ros2 node list` or press `Alt+Ctrl+n`
- **List services**: `rsl` or `ros2 service list`
- **List parameters**: `rpl` or `ros2 param list`
- **List actions**: `ral` or `ros2 action list`

### Workspace Navigation

- **Go to workspace root**: `cw`
- **Go to workspace src directory**: `cs`
- **Go to a package directory**: `ros2cd <package_name>`

### Build Commands

- **Build workspace**: `cob` or `colcon build --symlink-install`
- **Build in Debug mode**: `cobd` 
- **Build in Release mode**: `cobr`
- **Build specific package**: `cobp <package_name>`
- **Build up to package**: `cobput <package_name>`
- **Clean workspace**: `coc`

### ROS_DOMAIN_ID Management

- **Set domain ID**: `set_ros_domain_id <id>`
- **Get current domain ID**: `print_ros_domain_id`
- **Change domain ID**: Available in the `select_ws` menu

### Utility Functions

- **Show ROS environment variables**: `pR`
- **Source ROS2 setup**: `sr`
- **Clean ROS2 environment**: `csr`
- **Source current workspace**: `sw`
- **Show colcon build errors**: `se` or `show_colcon_errors`
- **Monitor topic**: `ros2_topic_monitor <topic>`
- **Check topic frequency**: `ros2_topic_hz <topic>`
- **Check topic bandwidth**: `ros2_topic_bw <topic>`

### Keyboard Shortcuts (via inputrc)

- `F5`: Reload bash configuration
- `Shift+F9`: Rebuild workspace and exit terminal
- `Ctrl+g`: Add grep filter to command
- `Alt+Ctrl+i`: Start apt install command
- `Alt+Ctrl+p`: Start pip install command
- `Shift+F2`: Install ROS2 package

### Workspace Alias Management

ROS-Hacks automatically discovers and sources alias files in your workspace:

- **Automatic discovery**: When sourcing a workspace, ROS-Hacks searches for files ending with "aliases"
- **Smart caching**: Alias files are cached for fast terminal startup - only searches when switching workspaces
- **Manual refresh**: Run `cache_ws_aliases` to manually refresh the alias cache for current workspace
- **Workspace-specific**: Each workspace can have its own set of alias files anywhere in the directory tree

**How it works:**
1. When you select a new workspace (`F3` or `select_ws`), ROS-Hacks searches for all files ending with "aliases"
2. Found alias files are cached and automatically sourced
3. On subsequent terminal sessions, cached aliases are sourced instantly without searching
4. Cache refreshes automatically when switching to a different workspace

**Example alias file locations:**
- `~/my_ws/src/my_package/scripts/.my_aliases`
- `~/my_ws/src/custom_aliases`  
- `~/my_ws/src/tools/robot_aliases`

### Quick Commands

- **Set a quick command**: `set-quick-command "your command here"`
- **Execute saved command**: `exec-quick-command`
- **View saved command**: `print-quick-command`

The prompt shows the current workspace name and ROS_DOMAIN_ID for easy reference.
MDCONTENT

    # Replace the GitHub username placeholder with the actual username
    sed -i "s/your-github-username/danlil240/g" "$REPO_DIR/README.md"
    
    echo -e "${GREEN}Instructions created in $REPO_DIR/README.md${NC}"
}

# Build the package
build_package() {
    echo -e "${BLUE}Building ROS-Hacks package...${NC}"

    # Increment version before building
    if [[ -z "$1" ]]; then
        printf "Increment version? (y/N): "
        if [[ -n ${ZSH_VERSION:-} ]]; then
            read -r -k 1 REPLY
        else
            read -r -n 1 REPLY
        fi
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            increment_version
        fi
    else
        increment_version
    fi

    # Check if build dependencies are installed
    if ! command -v dpkg-buildpackage &>/dev/null; then
        echo -e "${YELLOW}dpkg-buildpackage is not installed. Installing...${NC}"
        sudo apt-get update
        sudo apt-get install -y debhelper debhelper-compat build-essential dh-make
    fi

    # Create build directory
    BUILD_DIR="${SOURCE_DIR}/build"
    mkdir -p "$BUILD_DIR"

    # Get absolute paths before changing directories
    SOURCE_DIR_ABS="$(cd "$SOURCE_DIR" && pwd)"
    PARENT_DIR_ABS="$(dirname "$SOURCE_DIR_ABS")"

    cd "$SOURCE_DIR"
    dpkg-buildpackage -us -uc -b -d
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Package built successfully${NC}"

        # Collect the built .deb into our build directory (handle different dpkg-buildpackage outputs)
        echo -e "${BLUE}Collecting .deb into build directory...${NC}"
        CANDIDATES=()
        
        while IFS= read -r p || [[ -n "$p" ]]; do
            [[ -z "$p" ]] && continue
            CANDIDATES+=("$p")
        done < <(find "$PARENT_DIR_ABS" -maxdepth 1 -type f -name 'ros-hacks_*.deb' 2>/dev/null)

        while IFS= read -r p || [[ -n "$p" ]]; do
            [[ -z "$p" ]] && continue
            CANDIDATES+=("$p")
        done < <(find "$SOURCE_DIR_ABS" -maxdepth 1 -type f -name 'ros-hacks_*.deb' 2>/dev/null)
        
        if [ ${#CANDIDATES[@]} -eq 0 ]; then
            echo -e "${RED}No .deb candidates found. Searched in:${NC}"
            echo -e "${RED}  - $PARENT_DIR_ABS/${NC}"
            echo -e "${RED}  - $SOURCE_DIR_ABS/${NC}"
            echo -e "${BLUE}Available files in parent dir:${NC}"
            ls -la "$PARENT_DIR_ABS"/*.deb 2>/dev/null || echo -e "${YELLOW}No .deb files found${NC}"
            exit 1
        fi
        # Pick most recent by mtime
        DEB_SRC=$(ls -t "${CANDIDATES[@]}" | head -n1)
        mkdir -p "$BUILD_DIR"
        cp -f "$DEB_SRC" "$BUILD_DIR/"
        DEB_FILE="$BUILD_DIR/$(basename "$DEB_SRC")"
        echo -e "${GREEN}Found package: $DEB_FILE${NC}"
        add_package "$DEB_FILE"
    else
        echo -e "${RED}Package build failed${NC}"
        exit 1
    fi
}

# Setup GitHub repository
setup_github() {
    echo -e "${BLUE}Setting up GitHub repository...${NC}"

    local GITHUB_USER="danlil240"

    if [ -z "$GITHUB_USER" ]; then
        echo -e "${RED}GitHub username cannot be empty${NC}"
        return 1
    fi

    # Update README with correct GitHub username
    sed -i "s/your-github-username/$GITHUB_USER/g" "$REPO_DIR/README.md"

    # Work in the APT repository directory
    cd "$REPO_DIR"
    
    # Add all APT repository files
    git add .
    
    # Check if there are changes to commit
    if git diff --staged --quiet; then
        echo -e "${YELLOW}No changes to commit in APT repository${NC}"
    else
        git commit -m "Update APT repository - $(date '+%Y-%m-%d %H:%M:%S')"
        echo -e "${GREEN}APT repository changes committed${NC}"
    fi

    # Check if remote exists and set to ros-hacks-apt
    if git remote | grep -q "origin"; then
        git remote set-url origin "https://github.com/$GITHUB_USER/ros-hacks-apt.git"
    else
        git remote add origin "https://github.com/$GITHUB_USER/ros-hacks-apt.git"
    fi

    echo -e "${YELLOW}Remote 'origin' configured to: https://github.com/$GITHUB_USER/ros-hacks-apt.git${NC}"

    # Get the current branch name
    BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD)

    PUSH_NOW="y"

    if [ "$PUSH_NOW" = "y" ] || [ "$PUSH_NOW" = "Y" ]; then
        echo -e "${YELLOW}3. Ensure GitHub Pages is correctly configured:${NC}"
        git push -u origin $BRANCH_NAME
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}Successfully pushed to GitHub${NC}"
            echo -e "${YELLOW}Remember to enable GitHub Pages in repository settings:${NC}"
            echo -e "${YELLOW}1. Go to https://github.com/$GITHUB_USER/ros-hacks-apt/settings/pages${NC}"
            echo -e "${YELLOW}2. Set source to 'main' branch and '/' folder${NC}"
            echo -e "${YELLOW}3. It may take a few minutes for the pages to be published${NC}"
            echo -e "${YELLOW}4. Click Save${NC}"
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
    local GITHUB_USER=$1
    if [ -z "$GITHUB_USER" ]; then
        GITHUB_USER=$(git config --get remote.origin.url | sed -n 's|.*github.com[:/]\([^/]*\)/.*|\1|p')
    fi

    echo -e "\n${GREEN}=== INSTALLATION INSTRUCTIONS FOR TARGET MACHINE ===${NC}"
    echo -e "${BLUE}Run these commands on the target machine:${NC}"
    echo -e "# Download the key file first, then process it"
    echo -e "wget -O /tmp/ros-hacks.key https://$GITHUB_USER.github.io/ros-hacks-apt/ros-hacks.key"
    echo -e "sudo mkdir -p /etc/apt/keyrings"
    echo -e "cat /tmp/ros-hacks.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-hacks.gpg"
    echo -e "echo \"deb [signed-by=/etc/apt/keyrings/ros-hacks.gpg] https://$GITHUB_USER.github.io/ros-hacks-apt stable main\" | sudo tee /etc/apt/sources.list.d/ros-hacks.list"
    echo -e "sudo apt update"
    echo -e "sudo apt install ros-hacks"

    echo -e "\n${YELLOW}Note: Make sure GitHub Pages is enabled in your repository settings:${NC}"
    echo -e "${YELLOW}1. Go to https://github.com/$GITHUB_USER/ros-hacks-apt/settings/pages${NC}"
    echo -e "${YELLOW}2. Set source to 'main' branch and '/' folder${NC}"
    echo -e "${YELLOW}3. It may take a few minutes for the pages to be published${NC}"
}

# Install package locally without adding to repo or pushing to git
install_local() {
    local PACKAGE=$1

    if [ ! -f "$PACKAGE" ]; then
        echo -e "${RED}Package file not found: $PACKAGE${NC}"
        return 1
    fi

    echo -e "${BLUE}Installing package locally: $(basename $PACKAGE)${NC}"

    # Check if the package is already installed
    PACKAGE_NAME=$(dpkg-deb -f "$PACKAGE" Package)
    CURRENT_VERSION=$(dpkg-query -W -f='${Version}' "$PACKAGE_NAME" 2>/dev/null || echo "not installed")

    if [ "$CURRENT_VERSION" != "not installed" ]; then
        echo -e "${YELLOW}Package $PACKAGE_NAME is already installed (version $CURRENT_VERSION)${NC}"
        echo -e "${YELLOW}Reinstalling...${NC}"
    else
        echo -e "${GREEN}Installing new package: $PACKAGE_NAME${NC}"
    fi

    # Install the package
    sudo dpkg -i "$PACKAGE"
    INSTALL_STATUS=$?

    # Handle potential dependency issues
    if [ $INSTALL_STATUS -ne 0 ]; then
        echo -e "${YELLOW}Fixing dependencies and retrying...${NC}"
        sudo apt-get install -f -y
        sudo dpkg -i "$PACKAGE"
        INSTALL_STATUS=$?
    fi

    if [ $INSTALL_STATUS -eq 0 ]; then
        echo -e "${GREEN}Package installed successfully${NC}"
        # Show version of installed package
        NEW_VERSION=$(dpkg-query -W -f='${Version}' "$PACKAGE_NAME")
        echo -e "${GREEN}Installed version: $NEW_VERSION${NC}"
    else
        echo -e "${RED}Failed to install package${NC}"
        return 1
    fi
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
    # Get the username for instructions
    GITHUB_USER=$(git config --get remote.origin.url | sed -n 's|.*github.com[:/]\([^/]*\)/.*|\1|p')
    show_instructions "$GITHUB_USER"
elif [ "$1" = "github" ]; then
    setup_github
    # Get the username for instructions
    GITHUB_USER=$(git config --get remote.origin.url | sed -n 's|.*github.com[:/]\([^/]*\)/.*|\1|p')
    show_instructions "$GITHUB_USER"
elif [ "$1" = "build" ]; then
    build_package
elif [ "$1" = "install-local" ] && [ -n "$2" ]; then
    install_local "$2"
elif [ "$1" = "build-install" ]; then
    # Build the package and install it locally in one step
    build_package
    # Find the most recent .deb file in build directory
    DEB_FILE=$(find "$BUILD_DIR" -maxdepth 1 -name "ros-hacks_*.deb" -type f -printf "%T@ %p\n" | sort -n | tail -1 | cut -d' ' -f2-)
    if [ -n "$DEB_FILE" ]; then
        echo -e "${GREEN}Found package: $DEB_FILE${NC}"
        install_local "$DEB_FILE"
    else
        echo -e "${RED}Could not find built .deb package${NC}"
        exit 1
    fi
else
    echo -e "${BLUE}ROS-Hacks APT Repository Setup Tool${NC}"
    echo -e "${YELLOW}Usage:${NC}"
    echo -e "  $(basename $0)                   - Initial repository setup"
    echo -e "  $(basename $0) build             - Build the package"
    echo -e "  $(basename $0) add <deb-file>    - Add a .deb package to the repository"
    echo -e "  $(basename $0) update            - Update repository metadata"
    echo -e "  $(basename $0) github            - Push to GitHub and show installation instructions"
    echo -e "  $(basename $0) all               - Do all steps: build, add to repo, push to GitHub"
    echo -e "  $(basename $0) install-local <deb-file> - Install a .deb package locally without adding to repo"
    echo -e "  $(basename $0) build-install     - Build the package and install it locally"

    if [ -z "$1" ]; then
        echo -e "\n${BLUE}Running initial repository setup...${NC}"
        update_repo
        create_instructions

        echo -e "${GREEN}Repository setup complete!${NC}"
        echo -e "${YELLOW}Next steps:${NC}"
        echo -e "1. Run '$(basename $0) build' to build the package"
        echo -e "2. Run '$(basename $0) github' to setup and push to GitHub"
        echo -e "   OR"
        echo -e "3. Run '$(basename $0) all' to do everything in one step"
        echo -e "   OR"
        echo -e "4. Run '$(basename $0) build-install' to build and install locally without publishing"
    fi
fi
