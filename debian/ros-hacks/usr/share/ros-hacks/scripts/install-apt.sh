#!/usr/bin/env bash
set -eu
set -o pipefail 2>/dev/null || true
[[ -n ${ZSH_VERSION:-} ]] && setopt pipefail

# Install ROS-Hacks from the GitHub Pages APT repo
REPO_URL="https://danlil240.github.io/ROS-Hacks"
LIST_FILE="/etc/apt/sources.list.d/ros-hacks.list"
KEYRING_DIR="/etc/apt/keyrings"
KEY_FILE="$KEYRING_DIR/ros-hacks.gpg"

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root (e.g., sudo $0)"
  exit 1
fi

mkdir -p "$KEYRING_DIR"

# Download and install the GPG key using signed-by (avoids apt-key)
TMP_KEY=$(mktemp)
wget -qO "$TMP_KEY" "$REPO_URL/ros-hacks.key"
cat "$TMP_KEY" | gpg --dearmor -o "$KEY_FILE"
rm -f "$TMP_KEY"

echo "deb [signed-by=$KEY_FILE] $REPO_URL stable main" > "$LIST_FILE"

apt update
apt install -y ros-hacks

echo "Installed ros-hacks. Run: ros-hacks-setup"
