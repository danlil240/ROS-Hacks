# ROS-Hacks

ROS-Hacks is a utility package for Robot Operating System (ROS2) development that simplifies workspace management, builds, and daily operations.

## Table of Contents
- [Installation](#installation)
- [Getting Started](#getting-started)
- [ROS Workspace Management](#ros-workspace-management)
- [Building and Development](#building-and-development)
- [ROS2 Domain ID Management](#ros2-domain-id-management)
- [Quick Commands](#quick-commands)
- [Aliases and Shortcuts](#aliases-and-shortcuts)
- [Advanced Features](#advanced-features)

## Installation

### From APT Repository

1. Download the GPG key and set up the keyring:
```bash
wget -O /tmp/ros-hacks.key https://danlil240.github.io/ROS-Hacks/ros-hacks.key
sudo mkdir -p /etc/apt/keyrings
cat /tmp/ros-hacks.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-hacks.gpg
```

2. Add the repository to your sources (using the modern signed-by approach):
```bash
echo "deb [signed-by=/etc/apt/keyrings/ros-hacks.gpg] https://danlil240.github.io/ROS-Hacks stable main" | sudo tee /etc/apt/sources.list.d/ros-hacks.list
```

3. Update and install:
```bash
sudo apt update
sudo apt install ros-hacks
```

### Complete Installation

If the setup script did not run automatically during installation (or you want to run it again), simply execute:

```bash
ros-hacks-setup
```

This script will:

1. Check for and optionally install required dependencies (tmux and fzf)
2. Automatically configure your `~/.bashrc` to source ROS-Hacks
3. Set up keyboard shortcuts through an `~/.inputrc` file
4. Create initial configuration files (ROS domain ID and colcon defaults)

After running the setup, you'll need to refresh your terminal environment:

```bash
source ~/.bashrc
```

## Getting Started

ROS-Hacks automatically detects your OS and loads the appropriate ROS2 distribution (Foxy for Ubuntu 20.04, Humble for Ubuntu 22.04, Iron for Ubuntu 24.04). After installation, your terminal prompt will show your current workspace and domain ID.

Example prompt:
```
user my_robot_ws:1 ~/projects:
$
```

This shows you're in the `my_robot_ws` workspace with ROS domain ID 1.

## ROS Workspace Management

### Creating a New Workspace

```bash
prompt_new_ws  # Interactive prompt to create a new workspace
```

Or directly:

```bash
createWS my_robot  # Creates ~/my_robot_ws/src and initializes it
```

### Selecting a Workspace

```bash
select_ws  # Interactive selection from available workspaces
```

This will display a list of workspaces found in your home directory and let you choose one to work with.

### Switching Between Workspaces

```bash
sw  # Source the currently selected workspace
```

### Getting Workspace Information

```bash
get_current_ws_name  # Shows the current workspace name
```

## Building and Development

### Building Your Workspace

```bash
cob               # colcon build with symlink install
cobd              # Debug build
cobr              # Release build
cobp <pkg_name>   # Build specific package
cobput <pkg_name> # Build package and its dependencies
```

### Cleaning the Workspace

```bash
coc  # Clean the current workspace (build, install, log)
```

### Rebuilding the Current Workspace

```bash
rebuild_curr_ws  # Clean and rebuild the current workspace
```

### Navigation Shortcuts

```bash
cw   # Change to workspace root directory
cs   # Change to workspace src directory
```

## ROS2 Domain ID Management

Domain IDs isolate ROS2 communications on a network. ROS-Hacks makes this easy to manage:

```bash
set_ros_domain_id 5    # Set domain ID to 5
print_ros_domain_id    # Show current domain ID
```

You can also change domain ID through the workspace selector:

```bash
select_ws  # Choose "Change ROS_DOMAIN_ID" option
```

## Quick Commands

ROS-Hacks allows saving and executing frequently used commands in a workspace:

```bash
set-quick-command        # Save the last command as quick command
get-quick-command        # Display the saved quick command
print-quick-command      # Show the saved quick command with header
exec-quick-command       # Run the saved command in a tmux session
kill-tmux-quick-command  # Kill the tmux session and ROS2 nodes
```

## Aliases and Shortcuts

ROS-Hacks provides many aliases for common ROS2 operations:

### ROS2 Commands
```bash
rt   # ros2 topic
rtl  # ros2 topic list
rte  # ros2 topic echo
rti  # ros2 topic info
rn   # ros2 node
rnl  # ros2 node list
rni  # ros2 node info
rs   # ros2 service
rsl  # ros2 service list
rp   # ros2 param
rpl  # ros2 param list
```

### Build Commands
```bash
cob   # colcon build --symlink-install
cobd  # Debug build
cobr  # Release build
```

### Environment Management
```bash
pR    # Print all ROS-related environment variables
sr    # Source ROS2 setup.bash
csr   # Clean environment and source ROS2
```

## Advanced Features

### Building Release Packages

```bash
build_release  # Create a deployable package from your workspace
```

### Unsetting ROS Environment

```bash
unROS  # Clean ROS-related environment variables
```

### Finding Errors After Build

```bash
show_colcon_errors  # Show compilation errors after build
se                  # Short alias for show_colcon_errors
```

### Version Control Integration

The prompt shows Git branch information when in a Git repository, and utility functions help with package versioning and releases.

## Keyboard Shortcuts

ROS-Hacks provides many convenient keyboard shortcuts through its custom `inputrc` configuration:

### Navigation Shortcuts
- **F3**: Select ROS2 workspace interactively
- **Shift+F3**: Create a new ROS2 workspace
- **Ctrl+Right/Left**: Move cursor forward/backward one word

### ROS2 Commands
- **Alt+Ctrl+t**: List all ROS2 topics
- **Alt+Ctrl+n**: List all ROS2 nodes
- **Ctrl+g**: Add grep filter to any command

### Building Shortcuts
- **F9**: Rebuild current workspace
- **Shift+F9**: Rebuild workspace and exit terminal
- **Ctrl+F9**: Force CMake reconfigure and rebuild
- **F8**: Build specific package in current workspace

### Environment Management
- **F5**: Reload bash configuration
- **Ctrl/Alt+F5**: Watch current command with refresh
- **Shift+F5**: Run current command in detached tmux session

### Installation Shortcuts
- **Shift+F2**: Install ROS2 package
- **Alt+Ctrl+i**: Start apt install command
- **Alt+Ctrl+p**: Start pip install command


