#!/usr/bin/env bash
# ==========================================================
# ROS-Hacks - Topic tools (moved from functions.sh)
# ==========================================================

function ros2_topic_monitor() {
    local topic=${1:-""}
    if [[ -z "${topic}" ]]; then
        topic=$(ros2 topic list | fzf --prompt="Select topic: ")
        if [[ -z "${topic}" ]]; then
            printf "${RED_TXT}No topic selected.${NC}\n"
            return 1
        fi
    fi
    printf "${GREEN_TXT}Monitoring topic: ${topic}${NC}\n"
    printf "${YELLOW_TXT}Press Ctrl+C to stop${NC}\n"
    ros2 topic echo "$topic"
}

function ros2_topic_hz() {
    local topic=${1:-""}
    if [[ -z "${topic}" ]]; then
        topic=$(ros2 topic list | fzf --prompt="Select topic: ")
        if [[ -z "${topic}" ]]; then
            printf "${RED_TXT}No topic selected.${NC}\n"
            return 1
        fi
    fi
    printf "${GREEN_TXT}Checking topic frequency: ${topic}${NC}\n"
    printf "${YELLOW_TXT}Press Ctrl+C to stop${NC}\n"
    ros2 topic hz "$topic"
}

function ros2_topic_bw() {
    local topic=${1:-""}
    if [[ -z "${topic}" ]]; then
        topic=$(ros2 topic list | fzf --prompt="Select topic: ")
        if [[ -z "${topic}" ]]; then
            printf "${RED_TXT}No topic selected.${NC}\n"
            return 1
        fi
    fi
    printf "${GREEN_TXT}Checking topic bandwidth: ${topic}${NC}\n"
    printf "${YELLOW_TXT}Press Ctrl+C to stop${NC}\n"
    ros2 topic bw "$topic"
}

function ros2_pkg_list() {
    find "$curr_ws/install" -mindepth 1 -maxdepth 1 -type d -not -path "*/\.*" \
        -not -name "include" -not -name "lib" -not -name "share" \
        -not -name "bin" -not -name "etc" -exec basename {} \;
}
