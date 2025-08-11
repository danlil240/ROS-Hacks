#!/usr/bin/env bash
if [[ ${SKIP} == 1 ]]; then
    return 1
fi

# Determine ROS2 distribution based on Ubuntu version
if [[ $(lsb_release -cs) == 'focal' ]]; then
    ROS2_NAME='foxy'
elif [[ $(lsb_release -cs) == 'jammy' ]]; then
    ROS2_NAME='humble'
elif [[ $(lsb_release -cs) == 'noble' ]]; then
    ROS2_NAME='jazzy'
else
    # Default to latest stable release if version can't be determined
    ROS2_NAME='humble'
fi

# Define colors:
NC='\033[0m'
GREEN_TXT='\e[0;32m'
GREEN_TXT2='\e[32m'
DARK_GREEN_TXT='\e[2;32m'
WHITE_TXT='\e[1;37m'
RED_TXT='\e[31m'
DIM_RED_TXT='\e[2;31m'
LIGHT_BLUE_TXT='\e[96m'
BLUE_TXT='\e[34m'
DIM_BLUE_TXT='\e[2;34m'
DARK_GREY_TXT='\e[90m'
YELLOW_TXT='\e[93m'
BOLDRED="\033[1m\033[31m"     # Bold Red
BOLDGREEN="\033[1m\033[32m"   # Bold Green
BOLDYELLOW="\033[1m\033[33m"  # Bold Yellow
BOLDBLUE="\033[1m\033[34m"    # Bold Blue
BOLDMAGENTA="\033[1m\033[35m" # Bold Magenta
BOLDCYAN="\033[1m\033[36m"    # Bold Cyan
BOLDWHITE="\033[1m\033[37m"   # Bold White

WS_FILE=$HOME/.cache/ros-hacks/ros_ws_selected
ROS_DOMAIN_ID_FILE=$HOME/.cache/ros-hacks/ros_domain_id
QUICK_COMMAND_FILE=$HOME/.cache/ros-hacks/quick_command

alias pR='printenv | grep -i -e ROS -e CMAKE -e RMW -e AMENT -e COLCON'
alias sw='source_ws $(cat $WS_FILE)'
alias sr='source /opt/ros/${ROS2_NAME}/setup.bash'
alias csr='unROS; source /opt/ros/${ROS2_NAME}/setup.bash'

# Build aliases
alias cob='colcon build --symlink-install'
alias cobd='colcon build --symlink-install --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Debug'
alias cobr='colcon build --symlink-install --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release'
alias cobp='colcon build --symlink-install --packages-select'
alias cobput='colcon build --symlink-install --packages-up-to'
alias coc='clean_ros2_ws $(cat $WS_FILE)'

# Navigation aliases
alias cw='cd $(cat $WS_FILE)'
alias cs='cd $(cat $WS_FILE)/src'
alias cg='touch COLCON_IGNORE'
alias rcg='rm COLCON_IGNORE'

# ROS2 command aliases
alias rt='ros2 topic'
alias rtl='ros2 topic list'
alias rte='ros2 topic echo'
alias rti='ros2 topic info'
alias rn='ros2 node'
alias rnl='ros2 node list'
alias rni='ros2 node info'
alias rs='ros2 service'
alias rsl='ros2 service list'
alias rp='ros2 param'
alias rpl='ros2 param list'
alias ra='ros2 action'
alias ral='ros2 action list'
alias rb='ros2 bag'
alias rl='ros2 launch'

# Utility aliases
alias whgit='git config --get remote.origin.url'
# ensure any previous alias is cleared so the function is used
unalias show_colcon_errors 2>/dev/null || true
alias se='show_colcon_errors'

# Shell options
shopt -s direxpand
shopt -s expand_aliases
shopt -s histappend
shopt -s cmdhist

# History settings
export PROMPT_COMMAND='history -a'
export HISTFILESIZE=10000
export HISTSIZE=10000
export HISTCONTROL=ignoreboth
export HISTIGNORE='t:f:l:ls:bg:fg:history:h:select_ws:kill-tmux-gz:rt:rtl:rte:rti:rn:rs:rp:ra:rb'
export HISTTIMEFORMAT='%F %T '

# PS1=' \[\e[1;32m\]\u\[\033[00m\] \[\e[32m\]$(get_current_ws_name):$ROS_DOMAIN_ID\[\033[00m\] \[\033[03;94m\]\w\[\033[00m\]\[\033[38;5;51m\]$(__git_ps1)\[\033[00m\]:\n\$ '
