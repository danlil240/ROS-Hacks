#!/usr/bin/env bash
set -eu
set -o pipefail 2>/dev/null || true
[[ -n ${ZSH_VERSION:-} ]] && setopt pipefail

# Generate VS Code configuration for a single ROS 2 package.
# Usage: gen-vscode-config-pkg.sh [<package_root>]
# If not provided, uses current directory.

PKG_ROOT="${1:-$PWD}"
if [[ ! -d "$PKG_ROOT" ]]; then
  echo "Package directory not found: $PKG_ROOT" >&2
  exit 1
fi
if [[ ! -f "$PKG_ROOT/package.xml" ]]; then
  echo "package.xml not found in: $PKG_ROOT" >&2
  exit 2
fi

# Extract package name from package.xml (simple grep-based parser)
PKG_NAME=$(grep -oP '<name>\K[^<]+' "$PKG_ROOT/package.xml" | head -n1 || true)
if [[ -z "$PKG_NAME" ]]; then
  echo "Failed to detect package name from package.xml" >&2
  exit 3
fi

SCRIPTS_DIR=$(cd -- "$(dirname -- "$0")" >/dev/null 2>&1 && pwd)
RUN_IN_ROS="$SCRIPTS_DIR/run-in-ros.sh"

# Detect selected workspace root (if available) and compute relative paths
WS_FILE="$HOME/.cache/ros-hacks/ros_ws_selected"
WS_ROOT=""
if [[ -f "$WS_FILE" ]]; then
  WS_ROOT=$(cat "$WS_FILE")
fi
ABS_WS=""
REL_PREFIX=""
if [[ -n "$WS_ROOT" && -d "$WS_ROOT" ]]; then
  ABS_WS=$(readlink -f "$WS_ROOT" 2>/dev/null || python3 -c 'import os,sys; print(os.path.realpath(sys.argv[1]))' "$WS_ROOT")
  REL_PREFIX=$(python3 - "$WS_ROOT" "$PKG_ROOT" <<'PY'
import os, sys
ws, pkg = sys.argv[1], sys.argv[2]
rel = os.path.relpath(ws, pkg)
print("" if rel == "." else rel + "/")
PY
)
fi

# Determine ROS 2 distro for include path hints
ROS2_NAME=""
case "$(lsb_release -cs 2>/dev/null || echo unknown)" in
  focal) ROS2_NAME=foxy;;
  jammy) ROS2_NAME=humble;;
  noble) ROS2_NAME=jazzy;;
  *)     ROS2_NAME=humble;;
esac

VSCODE_DIR="$PKG_ROOT/.vscode"
mkdir -p "$VSCODE_DIR"

# Paths for local package-oriented workspace
# Preserve VS Code variable expansion literally
WF='${workspaceFolder}'
if [[ -n "$REL_PREFIX" ]]; then
  BUILD_REL="${REL_PREFIX}build"
  INSTALL_REL="${REL_PREFIX}install"
  LOG_REL="${REL_PREFIX}log"
else
  BUILD_REL=".colcon/build"
  INSTALL_REL=".colcon/install"
  LOG_REL=".colcon/log"
fi
BUILD_BASE="${WF}/${BUILD_REL}"
INSTALL_BASE="${WF}/${INSTALL_REL}"
LOG_BASE="${WF}/${LOG_REL}"
PKG_BUILD_CC="${BUILD_BASE}/${PKG_NAME}/compile_commands.json"

# c_cpp_properties.json
cat >"$VSCODE_DIR/c_cpp_properties.json" <<JSON
{
  "version": 4,
  "configurations": [
    {
      "name": "ROS2-Package",
      "intelliSenseMode": "linux-gcc-x64",
      "compilerPath": "/usr/bin/g++",
      "compileCommands": "${PKG_BUILD_CC}",
      "configurationProvider": "ms-vscode.cmake-tools",
      "defines": [],
      "includePath": [
        "${WF}/**",
        "${WF}/include",
        "/opt/ros/${ROS2_NAME}/include/**",
        "/usr/include/eigen3"
      ],
      "browse": {
        "path": [
          "${WF}",
          "/opt/ros/${ROS2_NAME}/include",
          "/usr/include/eigen3",
          "${INSTALL_BASE}"
        ],
        "limitSymbolsToIncludedHeaders": true
      },
      "cppStandard": "c++17",
      "cStandard": "c17"
    }
  ]
}
JSON

# settings.json
cat >"$VSCODE_DIR/settings.json" <<JSON
{
  "cmake.configureOnOpen": false,
  "editor.formatOnSave": true,
  "C_Cpp.default.configurationProvider": "ms-vscode.cmake-tools",
  "C_Cpp.clang_format_style": "file",
  "C_Cpp.intelliSenseEngine": "default",
  "C_Cpp.errorSquiggles": "enabled",
  "C_Cpp.default.intelliSenseMode": "linux-gcc-x64",
  "C_Cpp.default.cppStandard": "c++17",
  "C_Cpp.default.includePath": [
    "${WF}/**",
    "${WF}/include"
  ],
  "C_Cpp.default.compileCommands": "${PKG_BUILD_CC}",
  "cmake.generator": "Ninja",
  "cmake.sourceDirectory": "${WF}",
  "cmake.buildDirectory": "${BUILD_BASE}/${PKG_NAME}/",
  "cmake.environment": {
    "PATH": "${env:PATH}",
    "SHELL": "/bin/bash",
    "BASH_ENV": "${INSTALL_BASE}/setup.bash"
  },
  "python.analysis.extraPaths": ["${WF}/src"],
  "files.associations": {"*.ipp": "cpp"}
}
JSON

# extensions.json
cat >"$VSCODE_DIR/extensions.json" <<JSON
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
GCC='$gcc'
cat >"$VSCODE_DIR/tasks.json" <<JSON
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "gen: env (.vscode/ros_env.env)",
      "type": "shell",
      "options": {"cwd": "${WF}"},
      "command": "${RUN_IN_ROS}",
      "args": [
        "bash", "-lc",
        "env | egrep -E '^(AMENT|COLCON|ROS|RMW|LD_LIBRARY_PATH|PYTHONPATH|CMAKE_PREFIX_PATH|PATH)=' > .vscode/ros_env.env"
      ],
      "problemMatcher": []
    },
    {
      "label": "colcon: build package (Debug)",
      "type": "shell",
      "options": {"cwd": "${WF}"},
      "command": "${RUN_IN_ROS}",
      "args": [
        "colcon", "build",
        "--symlink-install",
        "--base-paths", ".",
        "--packages-select", "${PKG_NAME}",
        "--build-base", "${BUILD_REL}",
        "--install-base", "${INSTALL_REL}",
        "--log-base", "${LOG_REL}",
        "--cmake-args", "-GNinja", "-DCMAKE_BUILD_TYPE=Debug", "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
      ],
      "problemMatcher": ["${GCC}"],
      "group": {"kind": "build", "isDefault": true}
    },
    {
      "label": "colcon: build package (Release)",
      "type": "shell",
      "options": {"cwd": "${WF}"},
      "command": "${RUN_IN_ROS}",
      "args": [
        "colcon", "build",
        "--symlink-install",
        "--base-paths", ".",
        "--packages-select", "${PKG_NAME}",
        "--build-base", "${BUILD_REL}",
        "--install-base", "${INSTALL_REL}",
        "--log-base", "${LOG_REL}",
        "--cmake-args", "-GNinja", "-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
      ],
      "problemMatcher": ["${GCC}"]
    },
    {
      "label": "colcon: test package",
      "type": "shell",
      "options": {"cwd": "${WF}"},
      "command": "${RUN_IN_ROS}",
      "args": [
        "colcon", "test",
        "--base-paths", ".",
        "--packages-select", "${PKG_NAME}",
        "--build-base", "${BUILD_REL}",
        "--install-base", "${INSTALL_REL}",
        "--log-base", "${LOG_REL}"
      ],
      "problemMatcher": []
    },
    {
      "label": "gen: link compile_commands.json",
      "type": "shell",
      "options": {"cwd": "${WF}"},
      "command": "bash",
      "args": ["-lc", "ln -sf ${BUILD_REL}/${PKG_NAME}/compile_commands.json compile_commands.json"],
      "problemMatcher": []
    },
    {
      "label": "colcon: clean package",
      "type": "shell",
      "options": {"cwd": "${WF}"},
      "command": "bash",
      "args": ["-lc", "rm -rf .colcon/build .colcon/install .colcon/log compile_commands.json"],
      "problemMatcher": []
    }
  ]
}
JSON

# launch.json (templates)
cat >"$VSCODE_DIR/launch.json" <<JSON
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "ROS2: Run Python node (${PKG_NAME})",
      "type": "python",
      "request": "launch",
      "console": "integratedTerminal",
      "module": "ros2",
      "args": ["run", "${PKG_NAME}", "<entrypoint>"],
      "envFile": "${WF}/.vscode/ros_env.env",
      "preLaunchTask": "gen: env (.vscode/ros_env.env)",
      "justMyCode": true
    },
    {
      "name": "ROS2: Debug C++ node (${PKG_NAME})",
      "type": "cppdbg",
      "request": "launch",
      "program": "${WF}/.colcon/install/${PKG_NAME}/lib/${PKG_NAME}/<executable>",
      "args": [],
      "stopAtEntry": false,
      "cwd": "${WF}",
      "environment": [],
      "externalConsole": false,
      "MIMode": "gdb",
      "miDebuggerPath": "/usr/bin/gdb"
    }
  ]
}
JSON

echo "Generated package .vscode configuration in ${PKG_ROOT} for package ${PKG_NAME}"
