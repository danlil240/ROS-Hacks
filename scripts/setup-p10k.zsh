#!/usr/bin/env zsh

# ==========================================================
# ROS-Hacks P10K Setup Script
# ==========================================================

SCRIPT_DIR="$(cd -- "$(dirname -- "${(%):-%x}")" >/dev/null 2>&1 && pwd)"
ROSHACKS_DIR="$(dirname "$SCRIPT_DIR")"

# If running from symlink, get the actual directory
if [[ -L "${(%):-%x}" ]]; then
  ROSHACKS_DIR="$(dirname "$(readlink -f "${(%):-%x}")")/.."
fi

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[93m'
RED='\033[31m'
BLUE='\033[34m'
NC='\033[0m'

print_status() {
    local color=$1
    local message=$2
    printf "${color}${message}${NC}\n"
}

print_status "$BLUE" "=== ROS-Hacks Powerlevel10k Setup ==="

# Check if p10k is available
if ! typeset -f p10k >/dev/null 2>&1; then
  # Try to source p10k if it's installed but not loaded
  local p10k_dir=""
  if [[ -n "${ZSH_CUSTOM:-}" && -d "$ZSH_CUSTOM/themes/powerlevel10k" ]]; then
    p10k_dir="$ZSH_CUSTOM/themes/powerlevel10k"
  elif [[ -n "${ZSH:-}" && -d "$ZSH/custom/themes/powerlevel10k" ]]; then
    p10k_dir="$ZSH/custom/themes/powerlevel10k"
  elif [[ -d "$HOME/.oh-my-zsh/custom/themes/powerlevel10k" ]]; then
    p10k_dir="$HOME/.oh-my-zsh/custom/themes/powerlevel10k"
  fi
  
  if [[ -n "$p10k_dir" && -f "$p10k_dir/powerlevel10k.zsh-theme" ]]; then
    source "$p10k_dir/powerlevel10k.zsh-theme"
  else
    print_status "$RED" "Error: Powerlevel10k is not installed or not loaded"
    print_status "$YELLOW" "Please install Powerlevel10k first:"
    print_status "$YELLOW" "  git clone --depth=1 https://github.com/romkatv/powerlevel10k.git \$ZSH_CUSTOM/themes/powerlevel10k"
    print_status "$YELLOW" "Then set 'ZSH_THEME=\"powerlevel10k/powerlevel10k\"' in your ~/.zshrc"
    exit 1
  fi
  
  # Check again after sourcing
  if ! typeset -f p10k >/dev/null 2>&1; then
    print_status "$RED" "Error: Failed to load Powerlevel10k"
    exit 1
  fi
fi

print_status "$GREEN" "✓ Powerlevel10k found"

# Check if ROS-Hacks is available
if [[ ! -f "$ROSHACKS_DIR/functions.sh" ]]; then
    print_status "$RED" "Error: ROS-Hacks functions.sh not found at $ROSHACKS_DIR/functions.sh"
    exit 1
fi

print_status "$GREEN" "✓ ROS-Hacks found at $ROSHACKS_DIR"

# Source ROS-Hacks to test functions
source "$ROSHACKS_DIR/functions.sh"

# Check if required functions exist
if ! typeset -f get_current_ws_name >/dev/null 2>&1; then
    print_status "$RED" "Error: get_current_ws_name function not found"
    exit 1
fi

if ! typeset -f get_ros_domain_id >/dev/null 2>&1; then
    print_status "$RED" "Error: get_ros_domain_id function not found"
    exit 1
fi

print_status "$GREEN" "✓ ROS-Hacks functions available"

# Test the segments
print_status "$BLUE" "Testing ROS segments..."

ws_name=$(get_current_ws_name)
domain_id=""
if [[ -f "$HOME/.cache/ros-hacks/ros_domain_id" ]]; then
    domain_id=$(cat "$HOME/.cache/ros-hacks/ros_domain_id" 2>/dev/null || echo "0")
else
    domain_id="${ROS_DOMAIN_ID:-0}"
fi

print_status "$BLUE" "Current workspace: $ws_name"
print_status "$BLUE" "Current domain: $domain_id"

# Check if p10k config exists
P10K_CONFIG="$HOME/.p10k.zsh"
if [[ -f "$P10K_CONFIG" ]]; then
    print_status "$GREEN" "✓ Found existing p10k configuration at $P10K_CONFIG"
    
    # Check if ROS segments are already configured
    if grep -q "ros_workspace\|ros_domain" "$P10K_CONFIG"; then
        print_status "$YELLOW" "ROS segments appear to be already configured in p10k"
        print_status "$YELLOW" "You can test them by running: source ~/.zshrc"
    else
        print_status "$BLUE" "Adding ROS segments to existing p10k configuration..."
        
        # Backup existing config
        cp "$P10K_CONFIG" "$P10K_CONFIG.backup.$(date +%Y%m%d_%H%M%S)"
        
        # Add ROS segments to the left prompt elements
        # This is a simple approach - we'll add only the workspace segment since domain is now combined
        sed -i '/POWERLEVEL9K_LEFT_PROMPT_ELEMENTS=(/a\  ros_workspace' "$P10K_CONFIG"
        
        # Add ROS segment customization if not present
        if ! grep -q "POWERLEVEL9K_ROS_WORKSPACE_ICON\|POWERLEVEL9K_ROS_DOMAIN_ICON" "$P10K_CONFIG"; then
            cat >> "$P10K_CONFIG" << 'EOF'

# ROS-Hacks segment customization
POWERLEVEL9K_ROS_WORKSPACE_ICON=''
POWERLEVEL9K_ROS_WORKSPACE_FOREGROUND='green'
POWERLEVEL9K_ROS_DOMAIN_ICON=''  
POWERLEVEL9K_ROS_DOMAIN_FOREGROUND='cyan'
EOF
        fi
        
        print_status "$GREEN" "✓ Added ros_workspace and ros_domain segments to p10k configuration"
        print_status "$YELLOW" "Backup created: $P10K_CONFIG.backup.$(date +%Y%m%d_%H%M%S)"
    fi
else
    print_status "$YELLOW" "No p10k configuration found. Creating a basic one..."
    
    # Create a basic p10k configuration
    cat > "$P10K_CONFIG" << 'EOF'
# Powerlevel10k configuration with ROS-Hacks integration

# Left prompt segments
POWERLEVEL9K_LEFT_PROMPT_ELEMENTS=(
  context
  dir
  vcs
  ros_workspace
  command_execution_time
  status
)

# Right prompt segments  
POWERLEVEL9K_RIGHT_PROMPT_ELEMENTS=(
  time
)

# ROS-Hacks segment customization
POWERLEVEL9K_ROS_WORKSPACE_ICON=''
POWERLEVEL9K_ROS_WORKSPACE_FOREGROUND='green'
POWERLEVEL9K_ROS_DOMAIN_ICON=''  
POWERLEVEL9K_ROS_DOMAIN_FOREGROUND='cyan'

# Basic p10k settings
POWERLEVEL9K_PROMPT_ADD_NEWLINE=false
POWERLEVEL9K_SHORTEN_DIR_LENGTH=3
POWERLEVEL9K_VCS_CLEAN_FOREGROUND='green'
POWERLEVEL9K_VCS_MODIFIED_FOREGROUND='yellow'
POWERLEVEL9K_VCS_UNTRACKED_FOREGROUND='red'
EOF
    
    print_status "$GREEN" "✓ Created basic p10k configuration with ROS segments"
fi

# Inject ROS-Hacks integration into p10k configuration
print_status "$BLUE" "Injecting ROS-Hacks integration into p10k configuration..."

# Check if ROS-Hacks integration is already sourced
if ! grep -q "p10k-integration.zsh\|ROS-Hacks Powerlevel10k Integration" "$P10K_CONFIG"; then
    # Add the integration file sourcing at the end of the config
    cat >> "$P10K_CONFIG" << 'EOF'

# ==========================================================
# ROS-Hacks Powerlevel10k Integration
# ==========================================================
# Source ROS-Hacks p10k integration for custom segments
if [[ -f "${ROSHACKS_DIR:-}/p10k-integration.zsh" ]]; then
  source "${ROSHACKS_DIR}/p10k-integration.zsh"
elif [[ -f "$HOME/.ros-hacks/p10k-integration.zsh" ]]; then
  source "$HOME/.ros-hacks/p10k-integration.zsh"
elif [[ -f "/usr/share/ros-hacks/p10k-integration.zsh" ]]; then
  source "/usr/share/ros-hacks/p10k-integration.zsh"
fi
EOF
    
    print_status "$GREEN" "✓ Added ROS-Hacks integration sourcing to p10k configuration"
else
    print_status "$GREEN" "✓ ROS-Hacks integration already present in p10k configuration"
fi

# Test if integration file exists
P10K_INTEGRATION="$ROSHACKS_DIR/p10k-integration.zsh"
if [[ -f "$P10K_INTEGRATION" ]]; then
    print_status "$GREEN" "✓ P10K integration file found"
else
    print_status "$RED" "Error: P10K integration file not found at $P10K_INTEGRATION"
    exit 1
fi

print_status "$BLUE" "=== Setup Complete ==="
print_status "$GREEN" "ROS-Hacks Powerlevel10k integration has been configured!"
print_status ""
print_status "$BLUE" "Next steps:"
print_status "1. Restart your terminal or run: source ~/.zshrc"
print_status "2. If you don't see the ROS segments, run: p10k configure"
print_status "3. Add 'ros_workspace' and 'ros_domain' to your prompt segments"
print_status ""
print_status "$BLUE" "Customization options:"
print_status "- Add to ~/.zshrc: POWERLEVEL9K_ROS_WORKSPACE_ICON='🤖'"
print_status "- Add to ~/.zshrc: POWERLEVEL9K_ROS_DOMAIN_ICON='🌐'"
print_status ""
print_status "$BLUE" "For more info, see: $ROSHACKS_DIR/docs/P10K-INTEGRATION.md"
