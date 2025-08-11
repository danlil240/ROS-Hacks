#!/usr/bin/env bash
# ==========================================================
# ROS-Hacks - Build helpers and better error surfacing
# This module augments colcon log inspection with summaries and interactive views.
# ==========================================================

# Resolve latest colcon log directory
_rh_log_dir() {
    local ws="${curr_ws:-}"
    if [[ -z "$ws" ]]; then
        # Try to load current ws from cache file
        local ws_file="${WS_FILE:-$HOME/.cache/ros-hacks/ros_ws_selected}"
        if [[ -f "$ws_file" ]]; then
            ws=$(cat "$ws_file")
        fi
    fi
    if [[ -z "$ws" ]]; then
        echo ""; return 1
    fi
    if [[ -d "$ws/log/latest_build" ]]; then
        echo "$ws/log/latest_build"
    else
        echo "$ws/log"
    fi
}

# Print a package error/warning summary line: "pkg|E:<num>|W:<num>|<log_path>"
_rh_pkg_summary_line() {
    local pkg_dir="$1"
    local log_file="$pkg_dir/stdout_stderr.log"
    local pkg_name
    pkg_name=$(basename "$pkg_dir")
    if [[ ! -f "$log_file" ]]; then
        return 1
    fi
    # Count errors and warnings (case-insensitive)
    local errs warns
    errs=$(grep -i "error" -c "$log_file" 2>/dev/null || echo 0)
    warns=$(grep -i "warning" -c "$log_file" 2>/dev/null || echo 0)
    echo "$pkg_name|E:$errs|W:$warns|$log_file"
}

# Print a summary table for all package logs
_rh_errors_summary() {
    local log_root="$(_rh_log_dir)"
    if [[ -z "$log_root" || ! -d "$log_root" ]]; then
        echo "No colcon log directory found. Build first (cob)."
        return 1
    fi
    local lines=()
    local d
    shopt -s nullglob
    for d in "$log_root"/*/; do
        _rh_pkg_summary_line "$d"
    done | awk -F'|' '{printf "%-32s  %-8s  %-8s  %s\n", $1, $2, $3, $4}' | sort -t '|' -k2,2nr -k3,3nr
}

# Preview a log around the first match of ERROR/WARNING
_rh_preview_log() {
    local log_file="$1"
    local pattern="$2"
    if [[ ! -f "$log_file" ]]; then
        echo "Log not found: $log_file"; return 1
    fi
    local ln
    ln=$(grep -inE "$pattern" "$log_file" | head -n1 | cut -d: -f1)
    if [[ -z "$ln" ]]; then
        ln=$(wc -l <"$log_file")
    fi
    local start=$(( ln>80 ? ln-80 : 1 ))
    local end=$(( ln+120 ))
    sed -n "${start},${end}p" "$log_file" | sed "${ln}q; d" >/dev/null 2>&1
    sed -n "${start},${end}p" "$log_file"
}

# Open a colcon log in pager (less) or cat fallback
open_colcon_log() {
    local pkg="${1:-}"
    local log_root="$(_rh_log_dir)"
    if [[ -z "$pkg" ]]; then
        echo "Usage: open_colcon_log <package>"; return 2
    fi
    local log_file="$log_root/$pkg/stdout_stderr.log"
    if [[ ! -f "$log_file" ]]; then
        echo "Log not found for package '$pkg' at $log_file"; return 1
    fi
    if command -v less >/dev/null 2>&1; then
        less +G "$log_file"
    else
        ${PAGER:-cat} "$log_file"
    fi
}

# Improved error surfacing: interactive if fzf exists, else summary
show_colcon_errors() {
    local log_root="$(_rh_log_dir)"
    if [[ -z "$log_root" || ! -d "$log_root" ]]; then
        echo "No colcon log directory found. Build first (cob)."
        return 1
    fi

    # If fzf exists, offer interactive picker
    if command -v fzf >/dev/null 2>&1; then
        local summary selected log_file
        summary=$(_rh_errors_summary)
        if [[ -z "$summary" ]]; then
            echo "No package logs found"; return 0
        fi
        selected=$(echo "$summary" | fzf --ansi \
            --prompt="Select pkg (enter=errors, alt-w=warnings, ctrl-o=open): " \
            --header="E/W counts per package. Preview shows first match context." \
            --with-nth=1,2,3 \
            --preview='awk "{print \$4}" <<< {} | xargs -r -I{} bash -lc "_pat=\"error|fatal\"; $(typeset -f _rh_preview_log); _rh_preview_log {} \"\${_pat}\""' \
            --bind 'alt-w:toggle-preview+reload(echo "$summary"),ctrl-o:execute-silent(awk "{print \$1}" <<< {} | xargs -r -I% bash -lc "open_colcon_log %")')
        [[ -z "$selected" ]] && return 0
        log_file=$(awk '{print $4}' <<< "$selected")
        # Show only errors with context
        echo "===== Errors in $(awk '{print $1}' <<< "$selected") ====="
        grep -inE "error|fatal" -n "$log_file" | head -n 1 >/dev/null 2>&1
        if [[ $? -ne 0 ]]; then
            echo "No errors found; showing warnings instead"
            grep -inE "warning" "$log_file" | sed -n '1,200p'
            return 0
        fi
        # Print first 200 matching lines with 5 lines context around each
        grep -inE "error|fatal" -n "$log_file" | cut -d: -f1 | head -n 20 | while read -r ln; do
            echo "--- context around line $ln ---"
            local start=$(( ln>5 ? ln-5 : 1 ))
            local end=$(( ln+5 ))
            sed -n "${start},${end}p" "$log_file"
        done
        return 0
    fi

    # Fallback: print summary and first error per package
    echo "Package                               Errors   Warnings"
    echo "--------------------------------------------------------"
    local d
    shopt -s nullglob
    for d in "$log_root"/*/; do
        local pkg_name log_file errs warns first
        pkg_name=$(basename "$d")
        log_file="$d/stdout_stderr.log"
        [[ -f "$log_file" ]] || continue
        errs=$(grep -i "error" -c "$log_file" 2>/dev/null || echo 0)
        warns=$(grep -i "warning" -c "$log_file" 2>/dev/null || echo 0)
        printf "%-36s %6s   %8s\n" "$pkg_name" "$errs" "$warns"
        first=$(grep -inE "error|fatal" "$log_file" | head -n1)
        if [[ -n "$first" ]]; then
            echo "  -> $first"
        fi
    done
}

show_colcon_warnings() {
    local log_root="$(_rh_log_dir)"
    if [[ -z "$log_root" || ! -d "$log_root" ]]; then
        echo "No colcon log directory found. Build first (cob)."
        return 1
    fi
    if command -v fzf >/dev/null 2>&1; then
        local items selected
        items=$(find "$log_root" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort)
        selected=$(echo "$items" | fzf --prompt="Select pkg (enter=show warnings, ctrl-o=open log): " \
            --preview='LOG="$(_rh_log_dir)/{}/stdout_stderr.log"; grep -in --color=always -E "warning" "$LOG" | sed -n "1,200p"' \
            --bind 'ctrl-o:execute-silent(bash -lc "open_colcon_log {}")')
        [[ -z "$selected" ]] && return 0
        local log_file="$log_root/$selected/stdout_stderr.log"
        grep -inE "warning" "$log_file" | sed -n '1,200p'
        return 0
    fi
    # Fallback: dump warnings
    local d log_file
    for d in "$log_root"/*/; do
        log_file="$d/stdout_stderr.log"
        [[ -f "$log_file" ]] || continue
        echo "===== Warnings in $(basename "$d") ====="
        grep -inE "warning" "$log_file" | sed -n '1,200p' || true
    done
}
