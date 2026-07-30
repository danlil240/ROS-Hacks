#!/bin/bash
# Trigger a full release through GitHub Actions CI.
#
# Usage:
#   ./scripts/release.sh           # patch bump (default)
#   ./scripts/release.sh patch     # patch bump
#   ./scripts/release.sh minor     # minor bump
#   ./scripts/release.sh major     # major bump
#   ./scripts/release.sh --dry-run # preview version bump without releasing
#
# Pipeline chain (all automated via CI):
#   1. prepare-release.yml  → bump version, tag, push
#   2. build-package.yml    → build .deb, create GitHub release
#   3. Update APT Repository (danlil240/ros-hacks-apt) → sign with GPG secret, publish

set -euo pipefail

SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUMP="${1:-patch}"
DRY_RUN=0

case "$BUMP" in
    patch|minor|major) ;;
    --dry-run) DRY_RUN=1; BUMP="patch" ;;
    -h|--help)
        echo "Usage: $0 [patch|minor|major|--dry-run]"
        exit 0
        ;;
    *)
        echo "ERROR: Unknown argument '$BUMP'" >&2
        echo "Usage: $0 [patch|minor|major|--dry-run]" >&2
        exit 1
        ;;
esac

if ! command -v gh &>/dev/null; then
    echo "ERROR: gh CLI not installed. Install from https://cli.github.com/" >&2
    exit 1
fi

if ! gh auth status &>/dev/null; then
    echo "ERROR: Not authenticated with gh CLI. Run: gh auth login" >&2
    exit 1
fi

CURRENT_VERSION="$(tr -d '[:space:]' <"$SOURCE_DIR/VERSION")"

if [ "$DRY_RUN" -eq 1 ]; then
    echo "=== Dry Run ==="
    echo "Current version: $CURRENT_VERSION"
    DRY_RUN=1 "$SOURCE_DIR/scripts/bump-version.sh" "$BUMP"
    exit 0
fi

echo "============================================"
echo "  ROS-Hacks Release"
echo "============================================"
echo "  Current version: $CURRENT_VERSION"
echo "  Bump type:       $BUMP"
echo "  Repository:      danlil240/ROS-Hacks"
echo "============================================"
echo ""

# Step 1: Trigger prepare-release workflow
echo ">>> Triggering prepare-release workflow..."
gh workflow run prepare-release.yml -f bump="$BUMP"
sleep 3

# Get the most recent run
RUN_ID="$(gh run list --workflow=prepare-release.yml --limit=1 --json databaseId --jq '.[0].databaseId')"
if [ -z "$RUN_ID" ]; then
    echo "ERROR: Could not find workflow run ID" >&2
    exit 1
fi

echo ">>> Workflow run: https://github.com/danlil240/ROS-Hacks/actions/runs/$RUN_ID"
echo ""

# Step 2: Monitor prepare-release
echo ">>> Monitoring prepare-release workflow..."
if gh run watch "$RUN_ID" --exit-status; then
    echo "✓ prepare-release completed successfully"
else
    echo "✗ prepare-release failed" >&2
    echo "  Check: https://github.com/danlil240/ROS-Hacks/actions/runs/$RUN_ID" >&2
    exit 1
fi

# Extract the new version from the tag
NEW_TAG="$(gh run view "$RUN_ID" --json jobs --jq '.jobs[].steps[] | select(.name=="Create and push tag") | .number' 2>/dev/null || true)"
# Fallback: compute it locally
if [ -z "$NEW_TAG" ]; then
    MAJOR="$(echo "$CURRENT_VERSION" | cut -d. -f1)"
    MINOR="$(echo "$CURRENT_VERSION" | cut -d. -f2)"
    PATCH="$(echo "$CURRENT_VERSION" | cut -d. -f3)"
    case "$BUMP" in
        patch) PATCH=$((PATCH + 1)) ;;
        minor) MINOR=$((MINOR + 1)); PATCH=0 ;;
        major) MAJOR=$((MAJOR + 1)); MINOR=0; PATCH=0 ;;
    esac
    NEW_TAG="v${MAJOR}.${MINOR}.${PATCH}"
fi

echo ""
echo ">>> Waiting for build-package workflow to start..."
sleep 5

# Step 3: Monitor build-package
BUILD_RUN_ID="$(gh run list --workflow=build-package.yml --limit=1 --json databaseId --jq '.[0].databaseId')"
if [ -n "$BUILD_RUN_ID" ]; then
    echo ">>> Monitoring build-package workflow..."
    echo "    Run: https://github.com/danlil240/ROS-Hacks/actions/runs/$BUILD_RUN_ID"
    if gh run watch "$BUILD_RUN_ID" --exit-status; then
        echo "✓ build-package completed successfully"
    else
        echo "⚠ build-package had issues (check the run for details)" >&2
        echo "  Check: https://github.com/danlil240/ROS-Hacks/actions/runs/$BUILD_RUN_ID" >&2
    fi
else
    echo "⚠ Could not find build-package run (it may still be starting)" >&2
fi

# Step 4: Monitor APT repo update
echo ""
echo ">>> Waiting for APT repository update workflow..."
sleep 5

APT_RUN_ID="$(gh run list --workflow="Update APT Repository" -R danlil240/ros-hacks-apt --limit=1 --json databaseId --jq '.[0].databaseId')"
if [ -n "$APT_RUN_ID" ]; then
    echo ">>> Monitoring APT repository update..."
    echo "    Run: https://github.com/danlil240/ros-hacks-apt/actions/runs/$APT_RUN_ID"
    if gh run watch "$APT_RUN_ID" -R danlil240/ros-hacks-apt --exit-status; then
        echo "✓ APT repository updated and published"
    else
        echo "⚠ APT repository update had issues" >&2
        echo "  Check: https://github.com/danlil240/ros-hacks-apt/actions/runs/$APT_RUN_ID" >&2
    fi
else
    echo "⚠ Could not find APT repository update run (it may still be starting)" >&2
    echo "  Check manually: https://github.com/danlil240/ros-hacks-apt/actions" >&2
fi

echo ""
echo "============================================"
echo "  Release pipeline complete!"
echo "============================================"
echo "  Package: https://github.com/danlil240/ROS-Hacks/releases"
echo "  APT repo: https://danlil240.github.io/ros-hacks-apt"
echo ""
echo "  Install on any machine:"
echo "    sudo apt update && sudo apt install ros-hacks"
echo "============================================"
