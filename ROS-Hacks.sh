PS1=' \[\e[1;32m\]\u\[\033[00m\] \[\e[32m\]$(get_current_ws_name):$ROS_DOMAIN_ID\[\033[00m\] \[\033[03;94m\]\w\[\033[00m\]\[\033[38;5;51m\]$(__git_ps1)\[\033[00m\]:\n$ '
source /home/daniel/.ROS-Hacks/aliases.sh
source /home/daniel/.ROS-Hacks/functions.sh
get_current_ws
source_ws $curr_ws