# ROS-Hacks Completions

This directory contains shell completion scripts for ROS-Hacks commands.

## Files

- `ros-hacks-completions.bash` - Bash completions for ROS-Hacks
- `ros-hacks-completions.zsh` - ZSH completions for ROS-Hacks
- `test-zsh-completions.zsh` - Test script for ZSH completions

## Supported Commands

### Package Management
- `cobp` - Build specific packages
- `cobput` - Build packages up to specific package
- `ros2cd` - Change directory to ROS package

### Topic Operations
- `ros2_topic_monitor` - Monitor ROS topic with echo
- `ros2_topic_hz` - Show ROS topic frequency
- `ros2_topic_bw` - Show ROS topic bandwidth
- `rte` - ROS topic echo
- `rti` - ROS topic info

### Workspace Management
- `select_ws` - Select ROS workspace
- `set_current_ws` - Set current ROS workspace
- `source_ws` - Source ROS workspace
- `createWS` - Create new ROS workspace
- `clean_ros2_ws` - Clean ROS workspace

### Build Tools
- `gencc` - Generate compile_commands.json
- `vscgen` - Generate VS Code configuration
- `vscgen_pkg` - Generate VS Code config for package
- `open_colcon_log` - Open colcon build log

### Development Tools
- `rir` - Run command in ROS environment
- `show_colcon_errors` - Show colcon build errors
- `show_colcon_warnings` - Show colcon build warnings

### ROS2 Aliases
- `rt` - ROS2 topic
- `rtl` - ROS2 topic list
- `rn` - ROS2 node
- `rnl` - ROS2 node list
- `rs` - ROS2 service
- `rsl` - ROS2 service list
- `rp` - ROS2 param
- `rpl` - ROS2 param list
- `ra` - ROS2 action
- `ral` - ROS2 action list
- `rb` - ROS2 bag
- `rl` - ROS2 launch

### Utility Aliases
- `pR` - Print ROS-related environment variables
- `sw` - Source current workspace
- `sr` - Source ROS2 setup
- `csr` - Clear ROS and source ROS2 setup
- `cob` - Colcon build with symlink install
- `cobd` - Colcon build debug
- `cobr` - Colcon build release
- `coc` - Clean current workspace
- `cw` - Change to workspace directory
- `cs` - Change to workspace src directory
- `cg` - Create COLCON_IGNORE
- `rcg` - Remove COLCON_IGNORE
- `whgit` - Get git remote URL
- `se` - Show colcon errors
- `launch` - Interactive launch selector

## Installation

### Bash
Bash completions are automatically loaded when sourcing `ROS-Hacks.sh` in an interactive bash session.

### ZSH
ZSH completions are automatically loaded when sourcing `ROS-Hacks.zsh` in an interactive zsh session.

## Testing

To test ZSH completions:

```bash
source completions/test-zsh-completions.zsh
```

This will check if all expected commands have completion functions registered.
