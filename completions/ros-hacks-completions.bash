# bash completions for ROS-Hacks helpers

# Return workspace root, fallback to current dir
__rh_ws_root() {
    local ws_file="${WS_FILE:-$HOME/.cache/ros-hacks/ros_ws_selected}"
    if [[ -f "$ws_file" ]]; then
        cat "$ws_file"
        return
    fi
    echo "$PWD"
}

# Complete ROS packages (prefers colcon list, falls back to ros2 pkg list)
_rh_complete_pkg() {
    local cur
    COMPREPLY=()
    cur=${COMP_WORDS[COMP_CWORD]}
    local ws="$(__rh_ws_root)"
    local pkgs
    if command -v colcon >/dev/null 2>&1; then
        pkgs=$(cd "$ws" 2>/dev/null && colcon list --names-only 2>/dev/null)
    fi
    if [[ -z "$pkgs" ]] && command -v ros2 >/dev/null 2>&1; then
        pkgs=$(ros2 pkg list 2>/dev/null)
    fi
    COMPREPLY=( $(compgen -W "$pkgs" -- "$cur") )
}

# Complete ROS topics
_rh_complete_topic() {
    local cur
    COMPREPLY=()
    cur=${COMP_WORDS[COMP_CWORD]}
    local topics
    if command -v ros2 >/dev/null 2>&1; then
        topics=$(ros2 topic list 2>/dev/null)
    fi
    COMPREPLY=( $(compgen -W "$topics" -- "$cur") )
}

# Hook up completions
complete -F _rh_complete_pkg cobp cobput ros2cd open_colcon_log
complete -F _rh_complete_topic ros2_topic_monitor ros2_topic_hz ros2_topic_bw rte rti
