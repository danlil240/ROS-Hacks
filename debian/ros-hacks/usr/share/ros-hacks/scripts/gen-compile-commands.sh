#!/usr/bin/env bash
set -euo pipefail

# Generate a unified compile_commands.json at the workspace root by merging
# all package-specific compile_commands.json files under build/.
# Usage: gen-compile-commands.sh [<workspace_root>]

WS_ROOT="${1:-}"
if [[ -z "$WS_ROOT" ]]; then
  if [[ -f "$HOME/.cache/ros-hacks/ros_ws_selected" ]]; then
    WS_ROOT=$(cat "$HOME/.cache/ros-hacks/ros_ws_selected")
  else
    echo "Usage: $0 <workspace_root>" >&2
    exit 2
  fi
fi

BUILD_DIR="$WS_ROOT/build"
OUT_FILE="$WS_ROOT/compile_commands.json"

if [[ ! -d "$BUILD_DIR" ]]; then
  echo "Build directory not found: $BUILD_DIR" >&2
  exit 1
fi

tmp=$(mktemp)
python3 - "$BUILD_DIR" "$tmp" <<'PY'
import json, os, sys
build_dir = sys.argv[1]
out_file = sys.argv[2]
entries = []
for root, dirs, files in os.walk(build_dir):
    if 'compile_commands.json' in files:
        path = os.path.join(root, 'compile_commands.json')
        try:
            with open(path) as f:
                data = json.load(f)
                if isinstance(data, list):
                    entries.extend(data)
        except Exception as e:
            print(f"Warning: failed to read {path}: {e}", file=sys.stderr)
            continue
# Deduplicate by file+directory+command
seen = set()
unique = []
for e in entries:
    key = (e.get('file'), e.get('directory'), e.get('command'))
    if key in seen:
        continue
    seen.add(key)
    unique.append(e)
with open(out_file, 'w') as f:
    json.dump(unique, f, indent=2)
print(f"Wrote {len(unique)} entries to {out_file}")
PY

mv "$tmp" "$OUT_FILE"
