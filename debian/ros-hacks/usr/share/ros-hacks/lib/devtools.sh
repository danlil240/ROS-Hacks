#!/usr/bin/env bash
# Developer convenience helpers for ROS-Hacks

# gencc: Generate unified compile_commands.json for the current or provided workspace
function gencc() {
  local ws="${1:-}"
  if [[ -z "$ws" && -f "$WS_FILE" ]]; then
    ws="$(cat "$WS_FILE")"
  fi
  if [[ -z "$ws" ]]; then
    echo "Usage: gencc <workspace_root>" >&2
    return 2
  fi
  "${ROSHACKS_DIR:-/usr/share/ros-hacks}"/scripts/gen-compile-commands.sh "$ws"
}

# vscgen: Generate VS Code configuration in .vscode/ for the workspace
function vscgen() {
  local ws="${1:-}"
  if [[ -z "$ws" && -f "$WS_FILE" ]]; then
    ws="$(cat "$WS_FILE")"
  fi
  if [[ -z "$ws" ]]; then
    echo "Usage: vscgen <workspace_root>" >&2
    return 2
  fi
  "${ROSHACKS_DIR:-/usr/share/ros-hacks}"/scripts/gen-vscode-config.sh "$ws"
}

# vscgen_pkg: Generate VS Code configuration for a single package (from package dir or path)
function vscgen_pkg() {
  local pkg_dir="${1:-$PWD}"
  "${ROSHACKS_DIR:-/usr/share/ros-hacks}"/scripts/gen-vscode-config-pkg.sh "$pkg_dir"
}

# rir: Run a command in a ROS 2 environment (system + workspace)
function rir() {
  "${ROSHACKS_DIR:-/usr/share/ros-hacks}"/scripts/run-in-ros.sh "$@"
}
