#!/bin/bash

# Backup or restore the GPG signing key for ROS-Hacks APT repository
# This ensures the same key is used across different machines/environments

set -e

APT_REPO_DIR="$HOME/ros-hacks-apt"
GPG_BACKUP_DIR="$APT_REPO_DIR/.gpg-backup"
GPG_PRIVATE_KEY="$GPG_BACKUP_DIR/private.key"
GPG_PUBLIC_KEY="$GPG_BACKUP_DIR/public.key"

NC='\033[0m'
GREEN='\e[0;32m'
YELLOW='\e[93m'
BLUE='\e[34m'
RED='\e[31m'

case "$1" in
    backup)
        echo -e "${BLUE}Backing up GPG key...${NC}"
        if ! gpg --list-keys | grep -q "ROS-Hacks APT Repository"; then
            echo -e "${RED}No GPG key found for 'ROS-Hacks APT Repository'${NC}"
            exit 1
        fi
        
        mkdir -p "$GPG_BACKUP_DIR"
        gpg --armor --export-secret-keys "ROS-Hacks APT Repository" > "$GPG_PRIVATE_KEY"
        gpg --armor --export "ROS-Hacks APT Repository" > "$GPG_PUBLIC_KEY"
        
        echo -e "${GREEN}GPG key backed up to: $GPG_BACKUP_DIR${NC}"
        echo -e "${YELLOW}Keep this directory secure and backed up!${NC}"
        ;;
        
    restore)
        echo -e "${BLUE}Restoring GPG key from backup...${NC}"
        if [ ! -f "$GPG_PRIVATE_KEY" ]; then
            echo -e "${RED}No backup found at: $GPG_PRIVATE_KEY${NC}"
            exit 1
        fi
        
        if gpg --list-keys | grep -q "ROS-Hacks APT Repository"; then
            echo -e "${YELLOW}Key already exists in keyring. Skipping import.${NC}"
        else
            gpg --batch --import "$GPG_PRIVATE_KEY"
            echo -e "${GREEN}GPG key restored from backup${NC}"
        fi
        ;;
        
    status)
        echo -e "${BLUE}GPG Key Status:${NC}"
        echo -e "Backup location: $GPG_BACKUP_DIR"
        
        if [ -f "$GPG_PRIVATE_KEY" ]; then
            echo -e "${GREEN}✓ Private key backup exists${NC}"
            BACKUP_FPR=$(gpg --with-colons --import-options show-only --import < "$GPG_PRIVATE_KEY" 2>/dev/null | awk -F: '/^fpr:/ {print $10; exit}')
            echo -e "  Fingerprint: $BACKUP_FPR"
        else
            echo -e "${RED}✗ Private key backup NOT found${NC}"
        fi
        
        if gpg --list-keys | grep -q "ROS-Hacks APT Repository"; then
            echo -e "${GREEN}✓ Key exists in GPG keyring${NC}"
            KEYRING_FPR=$(gpg --list-secret-keys --with-colons 'ROS-Hacks APT Repository' 2>/dev/null | awk -F: '/^fpr:/ {print $10; exit}')
            echo -e "  Fingerprint: $KEYRING_FPR"
            
            if [ -n "$BACKUP_FPR" ] && [ "$BACKUP_FPR" = "$KEYRING_FPR" ]; then
                echo -e "${GREEN}✓ Backup and keyring match${NC}"
            elif [ -n "$BACKUP_FPR" ]; then
                echo -e "${RED}✗ Backup and keyring DO NOT match!${NC}"
            fi
        else
            echo -e "${RED}✗ Key NOT found in GPG keyring${NC}"
        fi
        ;;
        
    *)
        echo -e "${BLUE}GPG Key Management for ROS-Hacks APT Repository${NC}"
        echo -e "${YELLOW}Usage:${NC}"
        echo -e "  $(basename $0) backup   - Backup the GPG key"
        echo -e "  $(basename $0) restore  - Restore the GPG key from backup"
        echo -e "  $(basename $0) status   - Show backup and keyring status"
        echo
        echo -e "${YELLOW}Important:${NC}"
        echo -e "- Keep $GPG_BACKUP_DIR secure and backed up"
        echo -e "- Use the same key across all machines to avoid signature errors"
        echo -e "- Never commit the .gpg-backup directory to git"
        ;;
esac
