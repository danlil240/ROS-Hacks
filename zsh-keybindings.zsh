if [[ -z ${ZSH_VERSION:-} ]]; then
  return 0
fi

if [[ $- != *i* ]]; then
  return 0
fi

__rh_zle_insert() {
  local s="$1"
  LBUFFER+="$s"
}

__rh_zle_set_and_accept() {
  local cmd="$1"
  BUFFER="$cmd"
  zle accept-line
}

__rh_zle_wrap_watch() {
  local cmd="$BUFFER"
  BUFFER="watch -n 0.1 \"${cmd}\" "
  zle accept-line
}

__rh_zle_wrap_tmux_detached() {
  local cmd="$BUFFER"
  BUFFER="tmux new -d \"${cmd}\" "
  zle accept-line
}

__rh_widget_insert_grep() { __rh_zle_insert " | grep -i "; }
__rh_widget_ros2_topic_list() { __rh_zle_set_and_accept "ros2 topic list"; }
__rh_widget_ros2_node_list() { __rh_zle_set_and_accept "ros2 node list"; }
__rh_widget_select_ws() { __rh_zle_set_and_accept "select_ws"; }
__rh_widget_prompt_new_ws() { __rh_zle_set_and_accept "prompt_new_ws"; }
__rh_widget_install_ros_pkg() { __rh_zle_set_and_accept "sudo apt install -y ros-${ROS2_NAME}-"; }
__rh_widget_reload_shell_rc() {
  if [[ -f "$HOME/.zshrc" ]]; then
    __rh_zle_set_and_accept "source ~/.zshrc"
  else
    __rh_zle_set_and_accept "source ~/.bashrc"
  fi
}
__rh_widget_rebuild_ws() { __rh_zle_set_and_accept "rebuild_curr_ws"; }
__rh_widget_rebuild_ws_and_exit() { __rh_zle_set_and_accept "rebuild_curr_ws && exit"; }
__rh_widget_rebuild_ws_force() { __rh_zle_set_and_accept "rebuild_curr_ws --cmake-force-configure"; }
__rh_widget_apt_install() { __rh_zle_insert "sudo apt install -y "; }
__rh_widget_pip_install() { __rh_zle_insert "python3 -m pip install "; }

zle -N __rh_widget_insert_grep
zle -N __rh_widget_ros2_topic_list
zle -N __rh_widget_ros2_node_list
zle -N __rh_widget_select_ws
zle -N __rh_widget_prompt_new_ws
zle -N __rh_widget_install_ros_pkg
zle -N __rh_widget_reload_shell_rc
zle -N __rh_widget_rebuild_ws
zle -N __rh_widget_rebuild_ws_and_exit
zle -N __rh_widget_rebuild_ws_force
zle -N __rh_widget_apt_install
zle -N __rh_widget_pip_install
zle -N __rh_zle_wrap_watch
zle -N __rh_zle_wrap_tmux_detached

bindkey -M emacs '^[^T' __rh_widget_ros2_topic_list
bindkey -M emacs '^[^N' __rh_widget_ros2_node_list
bindkey -M emacs '^G' __rh_widget_insert_grep

bindkey -M emacs '^[[15;5~' __rh_zle_wrap_watch
bindkey -M emacs '^[[15;3~' __rh_zle_wrap_watch
bindkey -M emacs '^[[15;2~' __rh_zle_wrap_tmux_detached

bindkey -M emacs '^[OR' __rh_widget_select_ws
bindkey -M emacs '^[[1;2R' __rh_widget_prompt_new_ws

bindkey -M emacs '^[[1;2Q' __rh_widget_install_ros_pkg

bindkey -M emacs '^[[15~' __rh_widget_reload_shell_rc

bindkey -M emacs '^[[20~' __rh_widget_rebuild_ws
bindkey -M emacs '^[[20;2~' __rh_widget_rebuild_ws_and_exit
bindkey -M emacs '^[[20;5~' __rh_widget_rebuild_ws_force

bindkey -M emacs '^[^I' __rh_widget_apt_install
bindkey -M emacs '^[^P' __rh_widget_pip_install

bindkey -M emacs '^[[1;5C' forward-word
bindkey -M emacs '^[[1;5D' backward-word
bindkey -M emacs '^[[5C' forward-word
bindkey -M emacs '^[[5D' backward-word
bindkey -M emacs '^[^[[C' forward-word
bindkey -M emacs '^[^[[D' backward-word
