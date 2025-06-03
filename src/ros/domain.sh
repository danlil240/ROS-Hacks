#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - ROS Domain ID Management Functions
# ==========================================================

# Gets the current ROS_DOMAIN_ID
# Usage: get_ros_domain_id
# Sets global variable: domain_id
function get_ros_domain_id() {
    if [[ -f "${ROS_DOMAIN_ID_FILE}" ]]; then
        domain_id=$(cat "${ROS_DOMAIN_ID_FILE}")
    else
        domain_id="0"
    fi
}

# Prints the current ROS_DOMAIN_ID
# Usage: print_ros_domain_id
function print_ros_domain_id() {
    get_ros_domain_id
    printf "${BLUE_TXT}Current ROS_DOMAIN_ID: ${WHITE_TXT}${domain_id}${NC}\n"
}

# Sets the ROS_DOMAIN_ID
# Usage: set_ros_domain_id <domain_id>
function set_ros_domain_id() {
    local new_id=${1:-""}
    if [[ -z "${new_id}" ]]; then
        printf "${RED_TXT}Domain ID not specified.${NC}\n"
        return 1
    fi

    # Check if the input is a valid number
    if ! [[ "$new_id" =~ ^[0-9]+$ ]]; then
        printf "${RED_TXT}Invalid domain ID. Please enter a number.${NC}\n"
        return 1
    fi

    # Save the domain ID
    echo "${new_id}" > "${ROS_DOMAIN_ID_FILE}"
    export ROS_DOMAIN_ID="${new_id}"
    printf "${GREEN_TXT}ROS_DOMAIN_ID set to: ${WHITE_TXT}${new_id}${NC}\n"
}

# Prints domain ID information
# Usage: print_domain_info
function print_domain_info() {
    get_ros_domain_id
    printf "${BLUE_TXT}Current ROS_DOMAIN_ID: ${WHITE_TXT}${domain_id}${NC}\n"
}
