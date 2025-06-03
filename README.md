# ROS-Hacks

A collection of utilities for ROS2 workspace management and development workflow optimization.

## Project Organization

The project has been reorganized into a modular structure for better maintainability:

```
ros-hacks/
├── bin/                      # Executable scripts
│   ├── ros-hacks.sh          # Main entry point
│   ├── setup-apt-repo.sh     # APT repository management 
│   └── diagnostic.sh         # System diagnostic tool
├── src/                      # Source code organized by function
│   ├── core/                 # Core functionality and configuration
│   ├── workspace/            # Workspace management functions
│   ├── ros/                  # ROS2-specific utilities
│   ├── apt/                  # APT repository functions
│   └── utils/                # General utilities
├── config/                   # Configuration files
│   ├── aliases.sh            # Command aliases
│   ├── inputrc               # Keyboard shortcuts
│   └── defaults.yaml         # Colcon defaults
├── install/                  # Installation scripts
│   ├── setup.sh              # Main setup script
│   └── check_dependencies.sh # Dependency verification
└── docs/                     # Documentation
```

## Features

- Easy ROS2 workspace management (create, select, build)
- Intelligent ROS environment handling
- Useful aliases for common ROS2 commands
- Keyboard shortcuts (F3 to select workspace, Shift+F3 to create new workspace)
- Domain ID management for ROS2 communication
- Quick command storage and execution for each workspace
- APT repository management for distribution

## Installation

### From Source

1. Clone the repository:
```bash
git clone https://github.com/danlil240/ROS-Hacks.git ~/.ros-hacks
```

2. Run the setup script:
```bash
cd ~/.ros-hacks
./install/setup.sh
```

3. Source your bashrc:
```bash
source ~/.bashrc
```

### From APT Repository

1. Add the GPG key:
```bash
wget -O /tmp/ros-hacks.key https://danlil240.github.io/ROS-Hacks/config/keys/ros-hacks.key
sudo mkdir -p /etc/apt/keyrings
sudo gpg --dearmor -o /etc/apt/keyrings/ros-hacks.gpg /tmp/ros-hacks.key
```

2. Add the repository to your sources:
```bash
echo "deb [signed-by=/etc/apt/keyrings/ros-hacks.gpg] https://danlil240.github.io/ROS-Hacks stable main" | sudo tee /etc/apt/sources.list.d/ros-hacks.list
```

3. Update and install:
```bash
sudo apt update
sudo apt install ros-hacks
```

## Usage

- `wsselect` - Select a ROS2 workspace
- `wsnew` - Create a new ROS2 workspace
- `wsrebuild` - Rebuild the current workspace
- `wsclean` - Clean the current workspace
- `wslist` - List available workspaces
- `domain` - Show current ROS_DOMAIN_ID
- `setdomain` - Set ROS_DOMAIN_ID

Run `diagnostic.sh` to check your ROS-Hacks installation and environment.

## Repository Management

To set up or update the APT repository:

```bash
# Initialize repository
./bin/setup-apt-repo.sh setup

# Add a package
./bin/setup-apt-repo.sh add /path/to/package.deb

# Show installation instructions
./bin/setup-apt-repo.sh instructions
```
