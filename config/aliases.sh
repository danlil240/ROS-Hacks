#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - Aliases
# ==========================================================

# Skip if required
if [[ ${SKIP} == 1 ]]; then
    return 1
fi

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

# ROS Environment Aliases
alias pR='printenv | grep -i -e ROS -e CMAKE -e RMW -e AMENT -e COLCON'
alias sw='source_ws $(cat $WS_FILE)'
alias sr='source /opt/ros/${ROS2_NAME}/setup.bash'
alias csr='unROS; source /opt/ros/${ROS2_NAME}/setup.bash'

# Build Aliases
alias cob='colcon build --symlink-install'
alias cobd='colcon build --symlink-install --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Debug'
alias cobr='colcon build --symlink-install --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release'
alias cobp='colcon build --symlink-install --packages-select'
alias cobput='colcon build --symlink-install --packages-up-to'
alias coc='clean_ros2_ws $(cat $WS_FILE)'

# Navigation Aliases
alias cw='cd $(cat $WS_FILE)'
alias cs='cd $(cat $WS_FILE)/src'
alias cg='touch COLCON_IGNORE'
alias rcg='rm COLCON_IGNORE'

# ROS2 Command Aliases
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

# Workspace Management Aliases
alias wsselect='select_ws'
alias wsnew='prompt_new_ws'
alias wsrebuild='rebuild_curr_ws'
alias wsclean='clean_ros2_ws $(cat $WS_FILE)'
alias wslist='print_ws'
alias domain='print_ros_domain_id'
alias setdomain='set_ros_domain_id'

# Quick Command Aliases
alias qcset='set-quick-command'
alias qcget='get-quick-command'
alias qcprint='print-quick-command'
alias qcexec='exec-quick-command'

# Utility Aliases
alias whgit='git config --get remote.origin.url'
alias show_colcon_errors='for pkg in $curr_ws/log/latest_build/*/; do   pkg_name=$(basename "$pkg");   log_file="$pkg/stdout_stderr.log";   if grep -qiw "error" "$log_file"; then     echo -e "\n===== 🔴 Error in $pkg_name =====";     grep -A 20 -B 5 "error" "$log_file";   fi; done'
alias se='show_colcon_errors'
