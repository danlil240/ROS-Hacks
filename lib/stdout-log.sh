#!/usr/bin/env bash
# ==========================================================
# ROS-Hacks - Per-terminal stdout cache (/tmp/ros-hacks)
# ==========================================================

if ! { type rh_print | grep -q function; } 2>/dev/null; then
export ROSHACKS_STDOUT_LOG_LOADED=1

ROSHACKS_LOG_DIR="${ROSHACKS_LOG_DIR:-/tmp/ros-hacks}"

# Quiet by default: ROS-Hacks sourcing output goes to the log file only.
# Set ROSHACKS_VERBOSE=1 to also print to the terminal.
export ROSHACKS_VERBOSE="${ROSHACKS_VERBOSE:-0}"

__rh_sanitize_id() {
    local value="$1"
    value="${value//#/_}"
    value="${value//\//_}"
    value="${value// /_}"
    printf '%s' "${value}"
}

__rh_init_stdout_log() {
    mkdir -p "${ROSHACKS_LOG_DIR}" 2>/dev/null || return 1

    if [[ -z "${ROSHACKS_STDOUT_LOG:-}" ]]; then
        local tty_id shell_id
        tty_id="$(__rh_sanitize_id "${TTY:-no-tty}")"
        shell_id="${$}"
        export ROSHACKS_STDOUT_LOG="${ROSHACKS_LOG_DIR}/stdout-${tty_id}-${shell_id}.log"
    fi

    if [[ ! -f "${ROSHACKS_STDOUT_LOG}" ]]; then
        {
            printf '=== ROS-Hacks stdout log ===\n'
            printf 'shell pid: %s\n' "${$}"
            printf 'tty: %s\n' "${TTY:-unknown}"
            printf 'started: %s\n' "$(date '+%Y-%m-%d %H:%M:%S')"
            printf '===========================\n\n'
        } >>"${ROSHACKS_STDOUT_LOG}" 2>/dev/null
    fi
}

__rh_append_stdout_log() {
    __rh_init_stdout_log || return 1
    printf '%b\n' "$@" >>"${ROSHACKS_STDOUT_LOG}" 2>/dev/null
}

rh_print() {
    __rh_append_stdout_log "$@"
    if [[ "${ROSHACKS_VERBOSE}" == "1" ]]; then
        printf '%b\n' "$@"
    fi
}

__rh_log_section() {
    local label="$1"
    __rh_init_stdout_log || return 1
    {
        printf '\n--- %s: %s ---\n' "$(date '+%Y-%m-%d %H:%M:%S')" "${label}"
    } >>"${ROSHACKS_STDOUT_LOG}" 2>/dev/null
}

__rh_source_quiet() {
    local file="$1"
    if [[ ! -f "${file}" ]]; then
        rh_print "${YELLOW_TXT}Setup file not found (skipping): ${file}${NC}"
        return 1
    fi

    __rh_log_section "source ${file}"
    # shellcheck disable=SC1090
    source "${file}" >>"${ROSHACKS_STDOUT_LOG}" 2>&1
}

show_ros_hacks_stdout() {
    __rh_init_stdout_log || {
        printf '%s\n' "${RED_TXT}Failed to initialize ROS-Hacks stdout log.${NC}"
        return 1
    }

    if [[ ! -s "${ROSHACKS_STDOUT_LOG}" ]]; then
        printf 'No ROS-Hacks stdout logged yet.\n'
        printf 'Log file: %s\n' "${ROSHACKS_STDOUT_LOG}"
        return 0
    fi

    printf 'ROS-Hacks stdout log: %s\n' "${ROSHACKS_STDOUT_LOG}"
    if command -v less >/dev/null 2>&1; then
        less -R +G "${ROSHACKS_STDOUT_LOG}"
    else
        cat "${ROSHACKS_STDOUT_LOG}"
    fi
}

alias rhout='show_ros_hacks_stdout'

fi
