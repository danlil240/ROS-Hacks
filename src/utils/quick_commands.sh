#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - Quick Command Functions
# ==========================================================

# Set a quick command for the current workspace
# Usage: set-quick-command
function set-quick-command() {
    currline=$(fc -ln | tail -n2 | head -n1)
    if [[ -z "${currline}" ]]; then
        echo "No command available for saving"
        return 1
    fi

    get_current_ws
    if [[ -z "${curr_ws}" ]]; then
        echo "No workspace is currently selected"
        return 1
    fi

    quick_file=$curr_ws/${QUICK_COMMAND_FILE}
    echo "# Auto-saved at $(date +'%Y-%m-%d %H:%M:%S')" >${quick_file}
    echo "${currline}" >>${quick_file}
    echo "Command saved!"
}

# Get the quick command for the current workspace
# Usage: get-quick-command
function get-quick-command() {
    get_current_ws
    if [[ -z "${curr_ws}" ]]; then
        echo "No workspace is currently selected"
        return 1
    fi

    quick_file=$curr_ws/${QUICK_COMMAND_FILE}
    if [ ! -f ${quick_file} ]; then
        echo "No quick command exists in this workspace"
        return 1
    fi

    cat ${quick_file} | tail -n1
    return 0
}

# Print the quick command for the current workspace
# Usage: print-quick-command
function print-quick-command() {
    get_current_ws
    if [[ -z "${curr_ws}" ]]; then
        echo "No workspace is currently selected"
        return 1
    fi

    quick_file=$curr_ws/${QUICK_COMMAND_FILE}
    if [ ! -f ${quick_file} ]; then
        echo "No quick command exists in this workspace"
        return 1
    fi

    echo "Quick command in $curr_ws workspace:"
    cat ${quick_file}
    return 0
}

# Execute the quick command for the current workspace in a tmux session
# Usage: exec-quick-command
function exec-quick-command() {
    get_current_ws
    if [[ -z "${curr_ws}" ]]; then
        echo "No workspace is currently selected"
        return 1
    fi

    quick_file=$curr_ws/${QUICK_COMMAND_FILE}
    if [ ! -f ${quick_file} ]; then
        echo "No quick command exists in this workspace"
        return 1
    fi

    cmd_line=$(cat ${quick_file} | tail -n1)
    ses_name=$(get_current_ws_name)
    printf "Executing in tmux session ${ses_name}: '${cmd_line}'\n"
    tmux new-session -d -s ${ses_name} "cd ${curr_ws} && source install/setup.bash && ${cmd_line}; read"
    return 0
}
