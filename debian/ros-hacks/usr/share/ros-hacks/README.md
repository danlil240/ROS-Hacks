# ROS-Hacks

This is a repository for the ROS-Hacks package, a productivity toolkit that enhances the ROS2 development experience with workspace management tools, convenient aliases, keyboard shortcuts, and utility functions.

## Adding this repository to your system

1. Download and add the GPG key:
```bash
wget -qO /tmp/ros-hacks.key https://danlil240.github.io/ROS-Hacks/ros-hacks.key
sudo mkdir -p /etc/apt/keyrings
cat /tmp/ros-hacks.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-hacks.gpg
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
