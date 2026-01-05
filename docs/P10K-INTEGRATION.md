# ROS-Hacks Powerlevel10k Integration

This integration adds custom segments to your Powerlevel10k prompt to show the current ROS workspace and domain ID.

## Installation

1. Ensure you have ROS-Hacks installed and working
2. Ensure you have Powerlevel10k installed and configured
3. The p10k integration is automatically loaded when you source ROS-Hacks.zsh

## Configuration

### Method 1: Add segments to your p10k configuration

Add the following segments to your `~/.p10k.zsh` file:

```zsh
# Add ROS workspace and domain to left prompt
POWERLEVEL9K_LEFT_PROMPT_ELEMENTS=(
  # ... your existing elements ...
  ros_workspace
  ros_domain
  # ... more elements ...
)
```

### Method 2: Use p10k configure command

You can also add the segments using the p10k configure tool:

1. Run `p10k configure`
2. When prompted for prompt segments, add `ros_workspace` and `ros_domain`

### Method 3: Manual configuration

Add these lines to your `~/.zshrc` (after sourcing p10k):

```zsh
# Add ROS-Hacks segments to p10k
POWERLEVEL9K_LEFT_PROMPT_ELEMENTS+=(ros_workspace ros_domain)
```

## Customization

You can customize the appearance of the ROS segments using these variables:

```zsh
# Workspace segment customization
POWERLEVEL9K_ROS_WORKSPACE_ICON='🤖'
POWERLEVEL9K_ROS_WORKSPACE_FOREGROUND='green'
POWERLEVEL9K_ROS_WORKSPACE_BACKGROUND='black'

## Features

- **ros_workspace**: Shows the current ROS workspace name (e.g., "demo_ws")
- **ros_domain**: Shows the current ROS_DOMAIN_ID (e.g., "DOM:42")
- **Auto-refresh**: Prompt automatically updates when you switch workspaces or change domains
- **Conditional display**: Segments only show when relevant (workspace selected, non-zero domain)

## Example Output

With both segments enabled, your prompt will show something like:

```
user@machine ~/ros2_ws 🤖 demo_ws 🌐 DOM:42 master
$ 
```

## Troubleshooting

If the segments don't appear:

1. Make sure ROS-Hacks is loaded: `echo $ROSHACKS_LOADED`
2. Check if the functions exist: `typeset -f prompt_ros_workspace`
3. Verify p10k is loaded: `typeset -f _p9k_prompt_segment`
4. Restart your shell or run `source ~/.zshrc`

## Manual Testing

You can test the segments manually:

```zsh
# Test workspace segment
prompt_ros_workspace

# Test domain segment  
prompt_ros_domain

# Force prompt refresh
_p9k_reset_prompt 1
```
