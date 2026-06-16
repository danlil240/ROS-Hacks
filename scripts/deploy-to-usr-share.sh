#!/usr/bin/env bash
# Copy ROS-Hacks runtime files into /usr/share/ros-hacks (requires root).
set -euo pipefail

SRC="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." &>/dev/null && pwd)"
DEST="/usr/share/ros-hacks"

if [[ "$(id -u)" -ne 0 ]]; then
    echo "Re-running with sudo..."
    exec sudo "$0" "$@"
fi

install -d "$DEST" "$DEST/lib" "$DEST/completions" "$DEST/scripts"

install -m 755 "$SRC"/*.sh "$DEST/"
install -m 644 "$SRC"/*.yaml "$SRC"/*.zsh "$SRC/inputrc" "$SRC/VERSION" "$SRC/README.md" "$DEST/"
install -m 755 "$SRC/lib"/*.sh "$DEST/lib/"
install -m 644 "$SRC/completions"/* "$DEST/completions/"
install -m 755 "$SRC/scripts"/*.sh "$DEST/scripts/"
[[ -f "$SRC/scripts/setup-p10k.zsh" ]] && install -m 644 "$SRC/scripts/setup-p10k.zsh" "$DEST/scripts/"

echo "Deployed to $DEST"
grep -q 'function csr()' "$DEST/functions.sh" && echo "csr function: OK"
grep -q 'ensure_ros2_name' "$DEST/functions.sh" && echo "ensure_ros2_name: OK"
