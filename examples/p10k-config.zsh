#!/usr/bin/env zsh

# ==========================================================
# Example Powerlevel10k Configuration with ROS-Hacks
# ==========================================================
# Save this as ~/.p10k.zsh or integrate into your existing config

# Left prompt segments - ROS segments added
POWERLEVEL9K_LEFT_PROMPT_ELEMENTS=(
  os_icon               # OS icon (optional)
  context               # user@hostname
  dir                   # current directory
  vcs                   # git status
  ros_workspace         # ROS workspace name (🤖 workspace_name)
  ros_domain            # ROS domain ID (🌐 DOM:42)
  command_execution_time
  status                # exit code
)

# Right prompt segments
POWERLEVEL9K_RIGHT_PROMPT_ELEMENTS=(
  time                  # current time
)

# ROS-Hacks segment customization
POWERLEVEL9K_ROS_WORKSPACE_ICON='🤖'
POWERLEVEL9K_ROS_WORKSPACE_FOREGROUND='green'
POWERLEVEL9K_ROS_WORKSPACE_BACKGROUND='black'
POWERLEVEL9K_ROS_WORKSPACE_VISUAL_IDENTIFIER_EXPANSION='🤖'

POWERLEVEL9K_ROS_DOMAIN_ICON='🌐'
POWERLEVEL9K_ROS_DOMAIN_FOREGROUND='cyan'
POWERLEVEL9K_ROS_DOMAIN_BACKGROUND='black'
POWERLEVEL9K_ROS_DOMAIN_VISUAL_IDENTIFIER_EXPANSION='🌐'

# Standard p10k settings
POWERLEVEL9K_PROMPT_ADD_NEWLINE=false
POWERLEVEL9K_PROMPT_ON_NEWLINE=true
POWERLEVEL9K_RPROMPT_ON_NEWLINE=true

POWERLEVEL9K_MULTILINE_FIRST_PROMPT_PREFIX=''
POWERLEVEL9K_MULTILINE_LAST_PROMPT_PREFIX='%K{white}❯%k'

POWERLEVEL9K_SHORTEN_DIR_LENGTH=3
POWERLEVEL9K_SHORTEN_STRATEGY='truncate_to_last'

POWERLEVEL9K_VCS_CLEAN_FOREGROUND='green'
POWERLEVEL9K_VCS_MODIFIED_FOREGROUND='yellow'
POWERLEVEL9K_VCS_UNTRACKED_FOREGROUND='red'

POWERLEVEL9K_STATUS_OK_FOREGROUND='green'
POWERLEVEL9K_STATUS_ERROR_FOREGROUND='red'

POWERLEVEL9K_COMMAND_EXECUTION_TIME_FOREGROUND='yellow'
POWERLEVEL9K_COMMAND_EXECUTION_TIME_THRESHOLD=3

POWERLEVEL9K_TIME_FOREGROUND='blue'
POWERLEVEL9K_TIME_FORMAT='%D{%H:%M}'

# Conditional display - hide ROS segments when not relevant
POWERLEVEL9K_ROS_WORKSPACE_CONTENT_EXPANSION='${$((get_current_ws_name != "none")):#$(get_current_ws_name)}'
POWERLEVEL9K_ROS_DOMAIN_CONTENT_EXPANSION='${$((ROS_DOMAIN_ID != 0)):#DOM:$ROS_DOMAIN_ID}'

# Icons (using nerd fonts if available)
POWERLEVEL9K_OS_ICON_FOREGROUND='white'
POWERLEVEL9K_DIR_FOREGROUND='blue'
POWERLEVEL9K_CONTEXT_FOREGROUND='magenta'

# Transient prompt (optional)
POWERLEVEL9K_TRANSIENT_PROMPT=on
POWERLEVEL9K_TRANSIENT_PROMPT_PREFIX='%K{white}❯%k '
POWERLEVEL9K_TRANSIENT_PROMPT_FOREGROUND='green'
POWERLEVEL9K_TRANSIENT_PROMPT_BACKGROUND='black'
