#!/usr/bin/env bash
set -euo pipefail

# Generate VS Code configuration for a ROS 2 workspace.
# Usage: gen-vscode-config.sh [<workspace_root>]

WS_ROOT="${1:-}"
if [[ -z "$WS_ROOT" ]]; then
  if [[ -f "$HOME/.cache/ros-hacks/ros_ws_selected" ]]; then
    WS_ROOT=$(cat "$HOME/.cache/ros-hacks/ros_ws_selected")
  else
    echo "Usage: $0 <workspace_root>" >&2
    exit 2
  fi
fi

if [[ ! -d "$WS_ROOT" ]]; then
  echo "Workspace directory not found: $WS_ROOT" >&2
  exit 1
fi

SCRIPTS_DIR=$(cd -- "$(dirname -- "$0")" >/dev/null 2>&1 && pwd)
RUN_IN_ROS="$SCRIPTS_DIR/run-in-ros.sh"
GEN_CC="$SCRIPTS_DIR/gen-compile-commands.sh"
GCC='$gcc'

mkdir -p "$WS_ROOT/.vscode"

# c_cpp_properties.json
cat >"$WS_ROOT/.vscode/c_cpp_properties.json" <<JSON
{
  "version": 4,
  "configurations": [
    {
      "name": "ROS2",
      "intelliSenseMode": "gcc-x64",
      "compilerPath": "/usr/bin/g++",
      "compileCommands": "${WS_ROOT}/compile_commands.json",
      "cppStandard": "c++17",
      "cStandard": "c17"
    }
  ]
}
JSON

# settings.json
cat >"$WS_ROOT/.vscode/settings.json" <<JSON
{
  "cmake.configureOnOpen": false,
  "editor.formatOnSave": true,
  "C_Cpp.default.configurationProvider": "ms-vscode.cmake-tools"
}
JSON

# extensions.json
cat >"$WS_ROOT/.vscode/extensions.json" <<JSON
{
  "recommendations": [
    "ms-vscode.cpptools",
    "ms-vscode.cmake-tools",
    "ms-python.python",
    "ms-iot.vscode-ros"
  ]
}
JSON

# tasks.json
cat >"$WS_ROOT/.vscode/tasks.json" <<JSON
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "colcon: build",
      "type": "shell",
      "command": "${RUN_IN_ROS}",
      "args": ["--ws", "${WS_ROOT}", "colcon", "build", "--symlink-install"],
      "problemMatcher": ["${GCC}"],
      "group": {"kind": "build", "isDefault": true}
    },
    {
      "label": "colcon: build (Debug)",
      "type": "shell",
      "command": "${RUN_IN_ROS}",
      "args": ["--ws", "${WS_ROOT}", "colcon", "build", "--symlink-install", "--cmake-args", "-GNinja", "-DCMAKE_BUILD_TYPE=Debug"],
      "problemMatcher": ["${GCC}"]
    },
    {
      "label": "colcon: test",
      "type": "shell",
      "command": "${RUN_IN_ROS}",
      "args": ["--ws", "${WS_ROOT}", "colcon", "test"],
      "problemMatcher": []
    },
    {
      "label": "gen: compile_commands.json",
      "type": "shell",
      "command": "${GEN_CC}",
      "args": ["${WS_ROOT}"],
      "problemMatcher": []
    },
    {
      "label": "colcon: clean",
      "type": "shell",
      "command": "bash",
      "args": ["-lc", "rm -rf ${WS_ROOT}/build ${WS_ROOT}/install ${WS_ROOT}/log"],
      "problemMatcher": []
    }
  ]
}
JSON

# launch.json (templates)
cat >"$WS_ROOT/.vscode/launch.json" <<JSON
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "ROS2: Run Python node (template)",
      "type": "python",
      "request": "launch",
      "console": "integratedTerminal",
      "module": "ros2",
      "args": ["run", "<pkg>", "<entrypoint>"],
      "justMyCode": true
    },
    {
      "name": "ROS2: Debug C++ node (template)",
      "type": "cppdbg",
      "request": "launch",
      "program": "${WS_ROOT}/install/<pkg>/lib/<pkg>/<executable>",
      "args": [],
      "stopAtEntry": false,
      "cwd": "${WS_ROOT}",
      "environment": [],
      "externalConsole": false,
      "MIMode": "gdb",
      "miDebuggerPath": "/usr/bin/gdb"
    }
  ]
}
JSON

echo "Generated .vscode configuration in ${WS_ROOT}"
