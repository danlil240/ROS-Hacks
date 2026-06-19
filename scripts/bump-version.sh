#!/bin/bash
# Bump VERSION and debian/changelog. Used by CI and release automation only.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VERSION_FILE="${SOURCE_DIR}/VERSION"
CHANGELOG_FILE="${SOURCE_DIR}/debian/changelog"

BUMP="${1:-patch}"
DRY_RUN="${DRY_RUN:-0}"

usage() {
    echo "Usage: $0 [patch|minor|major]" >&2
    exit 1
}

case "$BUMP" in
    patch|minor|major) ;;
    -h|--help) usage ;;
    *) usage ;;
esac

if [[ ! -f "$VERSION_FILE" ]]; then
    echo "ERROR: VERSION file not found at $VERSION_FILE" >&2
    exit 1
fi

CURRENT_VERSION="$(tr -d '[:space:]' <"$VERSION_FILE")"
MAJOR="$(echo "$CURRENT_VERSION" | cut -d. -f1)"
MINOR="$(echo "$CURRENT_VERSION" | cut -d. -f2)"
PATCH="$(echo "$CURRENT_VERSION" | cut -d. -f3)"

case "$BUMP" in
    patch) PATCH=$((PATCH + 1)) ;;
    minor)
        MINOR=$((MINOR + 1))
        PATCH=0
        ;;
    major)
        MAJOR=$((MAJOR + 1))
        MINOR=0
        PATCH=0
        ;;
esac

NEW_VERSION="${MAJOR}.${MINOR}.${PATCH}"
NEW_DEBIAN_VERSION="${NEW_VERSION}-1"
TAG="v${NEW_VERSION}"
TIMESTAMP="$(date "+%a, %d %b %Y %H:%M:%S %z")"

echo "Current version: $CURRENT_VERSION"
echo "New version:     $NEW_VERSION"
echo "Tag:             $TAG"

if [[ "$DRY_RUN" == "1" ]]; then
    exit 0
fi

echo "$NEW_VERSION" >"$VERSION_FILE"

cat >"${CHANGELOG_FILE}.new" <<EOF
ros-hacks (${NEW_DEBIAN_VERSION}) unstable; urgency=medium

  * Release ${NEW_VERSION}

 -- Daniel <danlil240@gmail.com>  ${TIMESTAMP}

EOF

cat "$CHANGELOG_FILE" >>"${CHANGELOG_FILE}.new"
mv "${CHANGELOG_FILE}.new" "$CHANGELOG_FILE"

echo "VERSION=$NEW_VERSION"
echo "TAG=$TAG"
