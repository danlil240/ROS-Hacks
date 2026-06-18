#!/bin/bash

# Backup or restore the GPG signing key for ROS-Hacks APT repository
# This ensures the same key is used across different machines/environments

set -e

APT_REPO_DIR="$HOME/ros-hacks-apt"
GPG_BACKUP_DIR="$APT_REPO_DIR/.gpg-backup"
GPG_PRIVATE_KEY="$GPG_BACKUP_DIR/private.key"
GPG_PUBLIC_KEY="$GPG_BACKUP_DIR/public.key"
CANONICAL_SIGNING_KEY_FPR="24F6F039ADD1E7A19FE61176172DC787BB63A69F"

NC='\033[0m'
GREEN='\e[0;32m'
YELLOW='\e[93m'
BLUE='\e[34m'
RED='\e[31m'

read_expected_fpr() {
    if [ -f "$APT_REPO_DIR/SIGNING_KEY_FPR" ]; then
        tr -d '[:space:]' <"$APT_REPO_DIR/SIGNING_KEY_FPR"
    else
        echo "$CANONICAL_SIGNING_KEY_FPR"
    fi
}

gpg_key_fingerprint() {
    local key_file="$1"
    gpg --with-colons --import-options show-only --import <"$key_file" 2>/dev/null | awk -F: '/^fpr:/ {print $10; exit}'
}

require_canonical_fpr() {
    local fpr="$1"
    local context="$2"
    if [ "$fpr" != "$CANONICAL_SIGNING_KEY_FPR" ]; then
        echo -e "${RED}ERROR: $context fingerprint ($fpr) does not match canonical key ($CANONICAL_SIGNING_KEY_FPR).${NC}"
        exit 1
    fi
}

EXPECTED_FPR="$(read_expected_fpr)"

case "$1" in
    backup)
        echo -e "${BLUE}Backing up GPG key...${NC}"
        if ! gpg --list-secret-keys "$EXPECTED_FPR" >/dev/null 2>&1; then
            echo -e "${RED}Canonical signing key $EXPECTED_FPR not found in keyring${NC}"
            exit 1
        fi

        mkdir -p "$GPG_BACKUP_DIR"
        gpg --armor --export-secret-keys "$EXPECTED_FPR" >"$GPG_PRIVATE_KEY"
        gpg --armor --export "$EXPECTED_FPR" >"$GPG_PUBLIC_KEY"
        echo "$EXPECTED_FPR" >"$APT_REPO_DIR/.signing_key_fpr"

        BACKUP_FPR="$(gpg_key_fingerprint "$GPG_PRIVATE_KEY")"
        require_canonical_fpr "$BACKUP_FPR" "Backed-up private key"

        echo -e "${GREEN}GPG key backed up to: $GPG_BACKUP_DIR${NC}"
        echo -e "${YELLOW}Keep this directory secure and backed up!${NC}"
        ;;

    restore)
        echo -e "${BLUE}Restoring GPG key from backup...${NC}"
        if [ ! -f "$GPG_PRIVATE_KEY" ]; then
            echo -e "${RED}No backup found at: $GPG_PRIVATE_KEY${NC}"
            exit 1
        fi

        BACKUP_FPR="$(gpg_key_fingerprint "$GPG_PRIVATE_KEY")"
        require_canonical_fpr "$BACKUP_FPR" "Backed-up private key"

        if gpg --list-secret-keys "$EXPECTED_FPR" >/dev/null 2>&1; then
            echo -e "${YELLOW}Canonical key already exists in keyring. Skipping import.${NC}"
        else
            gpg --batch --import "$GPG_PRIVATE_KEY"
            echo -e "${GREEN}GPG key restored from backup${NC}"
        fi
        echo "$EXPECTED_FPR" >"$APT_REPO_DIR/.signing_key_fpr"
        ;;

    status)
        echo -e "${BLUE}GPG Key Status:${NC}"
        echo -e "Canonical fingerprint: $EXPECTED_FPR"
        echo -e "Backup location: $GPG_BACKUP_DIR"

        if [ -f "$GPG_PRIVATE_KEY" ]; then
            echo -e "${GREEN}✓ Private key backup exists${NC}"
            BACKUP_FPR="$(gpg_key_fingerprint "$GPG_PRIVATE_KEY")"
            echo -e "  Fingerprint: $BACKUP_FPR"
            if [ "$BACKUP_FPR" = "$EXPECTED_FPR" ]; then
                echo -e "${GREEN}✓ Backup matches canonical key${NC}"
            else
                echo -e "${RED}✗ Backup DOES NOT match canonical key!${NC}"
            fi
        else
            echo -e "${RED}✗ Private key backup NOT found${NC}"
        fi

        if gpg --list-secret-keys "$EXPECTED_FPR" >/dev/null 2>&1; then
            echo -e "${GREEN}✓ Canonical key exists in GPG keyring${NC}"
        else
            echo -e "${RED}✗ Canonical key NOT found in GPG keyring${NC}"
        fi
        ;;

    *)
        echo -e "${BLUE}GPG Key Management for ROS-Hacks APT Repository${NC}"
        echo -e "${YELLOW}Usage:${NC}"
        echo -e "  $(basename $0) backup   - Backup the canonical GPG key"
        echo -e "  $(basename $0) restore  - Restore the canonical GPG key from backup"
        echo -e "  $(basename $0) status   - Show backup and keyring status"
        echo
        echo -e "${YELLOW}Important:${NC}"
        echo -e "- Canonical fingerprint: $CANONICAL_SIGNING_KEY_FPR"
        echo -e "- Keep $GPG_BACKUP_DIR secure and backed up"
        echo -e "- Never commit the .gpg-backup directory to git"
        ;;
esac
