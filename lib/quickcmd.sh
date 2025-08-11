#!/usr/bin/env bash
# ==========================================================
# ROS-Hacks - Quick Command helpers (moved from functions.sh)
# ==========================================================

# Save last typed command into per-workspace quick command file
function set-quick-command() {
    local currline
    currline=$(fc -ln | tail -n2 | head -n1)
    if [[ -z "${currline}" ]]; then
        echo "No command available for saving"
        return 1
    fi
    local quick_file
    quick_file=$curr_ws/${QUICK_COMMAND_FILE}
    echo "# Auto-saved at $(date +'%Y-%m-%d %H:%M:%S')" >"${quick_file}"
    echo "${currline}" >>"${quick_file}"
    echo "Command saved!"
}

function get-quick-command() {
    local quick_file
    quick_file=$curr_ws/${QUICK_COMMAND_FILE}
    if [ ! -f "${quick_file}" ]; then
        echo "No quick command exists in this workspace"
        return 1
    fi
    tail -n1 "${quick_file}"
    return 0
}

function print-quick-command() {
    local quick_file
    quick_file=$curr_ws/${QUICK_COMMAND_FILE}
    if [ ! -f "${quick_file}" ]; then
        echo "No quick command exists in this workspace"
        return 1
    fi
    echo "Quick command in $curr_ws workspace:"
    cat "${quick_file}"
    return 0
}

function exec-quick-command() {
    local quick_file
    quick_file=$curr_ws/${QUICK_COMMAND_FILE}
    if [ ! -f "${quick_file}" ]; then
        echo "No quick command exists in this workspace"
        return 1
    fi
    local cmd_line ses_name
    cmd_line=$(tail -n1 "${quick_file}")
    ses_name=$(get_current_ws_name)
    printf "Executing in tmux session ${ses_name}: '%s'\n" "${cmd_line}"
    tmux new-session -d -s "${ses_name}" "cd ${curr_ws} && source install/setup.bash && ${cmd_line}; read"
    return 0
}

function kill-tmux-quick-command() {
    local ses_name
    ses_name=$(get_current_ws_name)
    printf "Killing tmux session %s and all ROS2 nodes...\n" "${ses_name}"
    tmux kill-session -t "${ses_name}" >/dev/null 2>&1 || true
    sleep 0.2

    # Kill all ROS2 nodes
    ros2 node list | xargs -r -L 1 -I % sh -c 'ros2 lifecycle set % shutdown || true; ros2 service call % shutdown || true' >/dev/null 2>&1 || true
    pkill -f ros2 || true

    # Kill Gazebo processes
    pkill -f gz || true
    pkill -f gazebo || true

    printf "All processes terminated.\n"
    return 0
}
