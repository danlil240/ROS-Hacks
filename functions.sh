#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - ROS2 Workspace Management Functions
# Version: 1.0.0
# ==========================================================

# Color definitions for terminal output
NC='\033[0m'
GREEN_TXT='\e[0;32m'
RED_TXT='\e[31m'
YELLOW_TXT='\e[93m'
BLUE_TXT='\e[34m'
LIGHT_BLUE_TXT='\e[96m'
WHITE_TXT='\e[1;37m'
DIM_BLUE_TXT='\e[2;34m'

# Ensure required environment variables are set
WS_FILE=${WS_FILE:-"$HOME/.cache/ros-hacks/ros_ws_selected"}
ROS_DOMAIN_ID_FILE=${ROS_DOMAIN_ID_FILE:-"$HOME/.cache/ros-hacks/ros_domain_id"}
QUICK_COMMAND_FILE=${QUICK_COMMAND_FILE:-"$HOME/.cache/ros-hacks/quick_command"}
ROS2NAME=${ROS2_NAME:-"humble"}
WS_SEARCH_PATHS_FILE=${WS_SEARCH_PATHS_FILE:-"$HOME/.cache/ros-hacks/ws_paths"}
WS_ALIASES_FILE=${WS_ALIASES_FILE:-"$HOME/.cache/ros-hacks/ws_aliases"}

# Prompts the user to create a new ROS2 workspace
# Usage: prompt_new_ws
function prompt_new_ws() {
    printf "${LIGHT_BLUE_TXT}Creating new ROS2 workspace${NC}\n"
    printf "${LIGHT_BLUE_TXT}Please enter the workspace name, will create ~/####_ws/src:${NC}\n"
    printf ":  "
    read -r name
    if [[ -z "${name}" ]]; then
        printf "${RED_TXT}No workspace name provided, aborting.${NC}\n"
        return 1
    fi
    createWS "$name"
}

# Creates a new ROS2 workspace with the specified name
# Usage: createWS <workspace_name>
function createWS() {
    local name=${1:-""}
    if [[ -z "${name}" ]]; then
        printf "${RED_TXT}ROS2 workspace name not specified.${NC}\n"
        return 1
    fi

    # Check if workspace already exists
    if [[ -d "${HOME}/${name}_ws" ]]; then
        printf "${YELLOW_TXT}Workspace ~/${name}_ws already exists.${NC}\n"
        printf "Do you want to use it anyway? (y/n): "
        read -r choice
        case "$choice" in
        y | Y) printf "${YELLOW_TXT}Using existing workspace.${NC}\n" ;;
        *)
            printf "${RED_TXT}Aborting workspace creation.${NC}\n"
            return 1
            ;;
        esac
    else
        # Create workspace directory structure
        mkdir -p "${HOME}/${name}_ws/src"
        if [[ $? -ne 0 ]]; then
            printf "${RED_TXT}Failed to create workspace directory.${NC}\n"
            return 1
        fi
    fi

    # Set and initialize workspace
    set_current_ws "${HOME}/${name}_ws"
    get_current_ws
    cd "$curr_ws" || {
        printf "${RED_TXT}Failed to change to workspace directory.${NC}\n"
        return 1
    }

    # Clean up current ROS environment before setting new workspace
    unROS

    # Initialize as ROS2 workspace
    printf "${BLUE_TXT}Initializing ROS2 workspace...${NC}\n"
    local ros_setup="/opt/ros/${ROS2_NAME}/setup.bash"
    if [[ -n ${ZSH_VERSION:-} && -f "/opt/ros/${ROS2_NAME}/setup.zsh" ]]; then
        ros_setup="/opt/ros/${ROS2_NAME}/setup.zsh"
    fi
    source "${ros_setup}"
    colcon build --symlink-install
    local build_status=$?
    source_ws "${curr_ws}"

    if [[ $build_status -eq 0 ]]; then
        printf "${GREEN_TXT}ROS2 workspace created successfully at ${WHITE_TXT}${curr_ws}${NC}\n"
    else
        printf "${YELLOW_TXT}ROS2 workspace initialized with warnings at ${WHITE_TXT}${curr_ws}${NC}\n"
    fi
}

function unROS() {
    # Get all variables containing 'ROS'
    vars=$(env | egrep -i ROS | cut -d= -f1 | sort -u)
    # For everyone do:
    while IFS= read -r v || [[ -n "$v" ]]; do
        if [[ ! "$v" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
            continue
        fi
        # Get the value
        if [[ $v == *"PWD"* ]]; then
            continue
        fi
        str=$(printenv "$v")
        # Divide into array separated by colon
        if [[ -n ${ZSH_VERSION:-} ]]; then
            IFS=':' read -rA arrIN <<< "$str"
        else
            IFS=':' read -r -a arrIN <<< "$str"
        fi
        # If variable name contains 'ROS' unset it
        if [[ $v == *"ROS"* ]]; then
            unset "$v"
            continue
        else # Otherwise evaluate the fields
            local new_str=""
            # For every field check:
            for i in "${arrIN[@]}"; do
                # If contains 'ros' or '_ws/' - skip the field
                if [[ $i == *"ros"* ]]; then
                    continue
                fi
                if [[ $i =~ "_ws/" ]]; then
                    continue
                fi
                if [[ -z "$new_str" ]]; then
                    new_str="$i"
                else
                    new_str="${new_str}:$i"
                fi
            done
            # After all fields are processed - move the temp var to the original
            if [[ -z "$new_str" ]]; then
                unset "$v"
            else
                if [[ -n ${ZSH_VERSION:-} ]]; then
                    typeset -gx "${v}=${new_str}"
                else
                    export "${v}=${new_str}"
                fi
            fi
        fi
    done < <(printf "%s\n" "$vars")

    # Unset paths that are setup from sourcing ROS
    unset CMAKE_PREFIX_PATH
    unset AMENT_PREFIX_PATH
    unset COLCON_PREFIX_PATH
}

function select_ws() {

    ask_for_ws_and_domain

    c=0
    for i in "${arrIN[@]}"; do
        c=$(($c + 1))
        if [[ $c == $num ]]; then
            set_current_ws $i
            # Force cache refresh when selecting new workspace
            source_ws $i true
            return 0
        fi
    done
}

# Gets the current ROS2 workspace path
# Usage: get_current_ws
# Sets global variable: curr_ws
function get_current_ws() {
    unset curr_ws
    if [[ -f "$WS_FILE" ]]; then
        curr_ws=$(cat "$WS_FILE" 2>/dev/null || echo "")
    else
        curr_ws=""
    fi
}

# Gets the current ROS2 workspace name (not full path)
# Usage: get_current_ws_name
# Returns: Workspace name or "none" if no workspace is selected
function get_current_ws_name() {
    get_current_ws
    if [[ -z "$curr_ws" ]]; then
        echo "none"
        return
    fi
    basename "$curr_ws"
}

# Sets the current ROS2 workspace
# Usage: set_current_ws <workspace_path>
function set_current_ws() {
    local new_curr_ws=${1:-""}
    if [[ -z "${new_curr_ws}" ]]; then
        printf "${RED_TXT}ROS2 workspace path not specified.${NC}\n"
        return 1
    fi

    # Verify workspace path exists
    if [[ ! -d "${new_curr_ws}" ]]; then
        printf "${RED_TXT}Workspace directory does not exist: ${WHITE_TXT}${new_curr_ws}${NC}\n"
        return 1
    fi

    # Set the workspace path
    echo "$new_curr_ws" >"$WS_FILE"
    if [[ $? -ne 0 ]]; then
        printf "${RED_TXT}Failed to write to workspace file: ${WHITE_TXT}${WS_FILE}${NC}\n"
        return 1
    fi
    printf "${GREEN_TXT}Current workspace set to: ${WHITE_TXT}${new_curr_ws}${NC}\n"
    return 0
}

# Gets the current ROS_DOMAIN_ID
# Usage: get_ros_domain_id
# Sets global variable: domain_id
function get_ros_domain_id() {
    unset domain_id
    if [[ -f "$ROS_DOMAIN_ID_FILE" ]]; then
        domain_id=$(cat "$ROS_DOMAIN_ID_FILE" 2>/dev/null || echo "0")
    else
        domain_id="0" # Default to domain ID 0
    fi
}

function print_ros_domain_id() {
    get_ros_domain_id
    echo $domain_id
}

function set_ros_domain_id() {
    id=${1:-""}
    if [[ -z "${id}" ]]; then
        printf "${RED_TXT}ROS DOMAIN ID not specified.${NC}\n"
        return 1
    else
        printf "${BLUE_TXT}ROS DOMAIN ID${NC} set to: ${BLUE_TXT}$id${NC}\n"
        echo $id >$ROS_DOMAIN_ID_FILE
        export ROS_DOMAIN_ID=$id
    fi
}

function determine_ws_ros_version() {
    ws_name=${1:-""}
    if [[ -z "${ws_name}" ]]; then
        printf "${RED_TXT}ROS workspace path not specified.${NC}\n"
        return 1
    fi
    if [[ -d "$ws_name/install" ]]; then
        ros_type="ROS2"
    elif [[ -d "$ws_name/devel" ]]; then
        ros_type="ROS1"
    else
        printf "${RED_TXT}Not a valid ROS workspace: $ws_name${NC}\n"
        printf "${YELLOW_TXT}Make sure the workspace has been built with colcon.${NC}\n"
        return 1
    fi
}

# Searches for files ending with 'aliases' in the current workspace and caches their paths
# Usage: cache_ws_aliases
function cache_ws_aliases() {
    local ws_path="${1:-""}"
    if [[ -z "${ws_path}" ]]; then
        get_current_ws
        ws_path="${curr_ws}"
    fi
    if [[ -z "${ws_path}" ]]; then
        printf "${RED_TXT}No current workspace set for alias caching.${NC}\n"
        return 1
    fi

    if [[ ! -d "${ws_path}" ]]; then
        printf "${RED_TXT}Workspace directory does not exist: ${ws_path}${NC}\n"
        return 1
    fi

    local src_path="${ws_path}/src"
    if [[ ! -d "${src_path}" ]]; then
        printf "${YELLOW_TXT}Workspace has no src directory (skipping alias cache): ${src_path}${NC}\n"
        return 0
    fi

    local maxdepth="${ROSHACKS_ALIAS_FIND_MAXDEPTH:-8}"
    if [[ ! "${maxdepth}" =~ ^[0-9]+$ ]]; then
        maxdepth=8
    fi

    local timeout_sec="${ROSHACKS_ALIAS_FIND_TIMEOUT:-3}"
    if [[ ! "${timeout_sec}" =~ ^[0-9]+$ ]]; then
        timeout_sec=3
    fi

    # Ensure cache directory exists
    mkdir -p "$(dirname "${WS_ALIASES_FILE}")"
    
    # Search for files ending with 'aliases' in the workspace
    printf "${BLUE_TXT}Searching for alias files in workspace...${NC}\n"
    local alias_files=()

    # local candidates_file
    # candidates_file=$(mktemp 2>/dev/null || echo "")
    # if [[ -z "${candidates_file}" ]]; then
    #     printf "${YELLOW_TXT}Failed to create temp file for alias search (skipping).${NC}\n"
    #     return 0
    # fi

    # if command -v timeout >/dev/null 2>&1; then
    #     local find_rc=0
    #     timeout -k 1 "${timeout_sec}" find "${src_path}" \
    #         -maxdepth "${maxdepth}" \
    #         -type d \( \
    #             -name .git -o -name build -o -name install -o -name log \
    #             -o -name node_modules -o -name .venv -o -name venv -o -name .cache \
    #         \) -prune -o \
    #         -type f \( -name "*aliases" -o -name ".*aliases" \) -print \
    #         >"${candidates_file}" 2>/dev/null
    #     find_rc=$?
    #     if [[ ${find_rc} -eq 124 ]]; then
    #         printf "${YELLOW_TXT}Alias search timed out after ${timeout_sec}s (keeping existing cache).${NC}\n"
    #         rm -f "${candidates_file}"
    #         return 0
    #     fi
    # else
    #     fdfind "${src_path}" \
    #         -maxdepth "${maxdepth}" \
    #         -type d \( \
    #             -name .git -o -name build -o -name install -o -name log \
    #             -o -name node_modules -o -name .venv -o -name venv -o -name .cache \
    #         \) -prune -o \
    #         -type f \( -name "*aliases" -o -name ".*aliases" \) -print \
    #         >"${candidates_file}" 2>/dev/null
    # fi
    
    while IFS= read -r -d '' file
    do
        alias_files+=("${file}")
    done < <(fd -H -t f -g "*aliases" -0 "${src_path}" 2>/dev/null) \
    > "${WS_ALIASES_FILE}"

    # while IFS= read -r file; do
    #     [[ -z "$file" ]] && continue
    #     alias_files+=("${file}")
    # done <"${candidates_file}"
    # rm -f "${candidates_file}"
    
    # Clear the cache file and write new alias file paths
    if [[ ${#alias_files[@]} -gt 0 ]]; then
        printf "${GREEN_TXT}Found ${#alias_files[@]} alias file(s):${NC}\n"
        for alias_file in "${alias_files[@]}"; do
            echo "${alias_file}" >> "${WS_ALIASES_FILE}"
            printf "  ${WHITE_TXT}${alias_file}${NC}\n"
        done
    else
        printf "${YELLOW_TXT}No alias files found in workspace.${NC}\n"
    fi
    
}

# Sources all cached alias files
# Usage: source_cached_aliases
function source_cached_aliases() {
    if [[ ! -f "${WS_ALIASES_FILE}" ]]; then
        return 0  # No alias cache file, nothing to source
    fi
    
    local alias_count=0
    while IFS= read -r alias_file; do
        # Skip empty lines
        [[ -z "${alias_file}" ]] && continue
        
        if [[ -f "${alias_file}" ]]; then
            printf "Sourcing alias file:${BOLDWHITE} ${alias_file}${NC}\n"
            source "${alias_file}"
            alias_count=$((alias_count + 1))
        else
            printf "${YELLOW_TXT}Alias file not found (skipping): ${alias_file}${NC}\n"
        fi
    done < "${WS_ALIASES_FILE}"
}

function source_ws() {
    ws_name=${1:-""}
    force_cache=${2:-false}
    if [[ -z "${ws_name}" ]]; then
        printf "${RED_TXT}ROS workspace path not specified.${NC}\n"
    else
        determine_ws_ros_version $ws_name
        if [[ $ros_type == "ROS2" ]]; then # Catkin found in ws
            unROS
            printf "Sourcing ${WHITE_TXT}$ws_name ${NC}\n"
            local ws_setup="$ws_name/install/setup.bash"
            if [[ -n ${ZSH_VERSION:-} && -f "$ws_name/install/setup.zsh" ]]; then
                ws_setup="$ws_name/install/setup.zsh"
            fi
            source "$ws_setup"
            if [[ -n ${ZSH_VERSION:-} && -f "$ws_name/post_source.zsh" ]]; then
                source "$ws_name/post_source.zsh"
            elif [ -f $ws_name/post_source.bash ]; then
                source $ws_name/post_source.bash
            fi
            
            # Only cache when forced (workspace switch) or no cache exists
            if [[ "$force_cache" == "true" ]] || [[ ! -f "${WS_ALIASES_FILE}" ]]; then
                cache_ws_aliases "$ws_name"
            fi
            
            # Always source cached aliases (fast operation)
            source_cached_aliases
        else
            printf "${RED_TXT}ERROR in ROS WS $ws_name - Sourcing aborted.${NC}\n"
        fi
    fi
    get_ros_domain_id
    if [[ ! -z "${domain_id}" ]]; then
        export ROS_DOMAIN_ID=$domain_id
    fi
}

function ask_for_ws_and_domain() {
    get_current_ws
    find_ws
    print_domain_info

    # Create an array for fzf
    local options=("Change ROS_DOMAIN_ID")
    for i in "${arrIN[@]}"; do
        options+=("${i}")
    done

    local selection=$(printf "%s\n" "${options[@]}" | fzf --no-multi --height=50% --border --prompt="Select workspace or option: " \
        --header="Current WS: ${curr_ws:-None} | ROS_DOMAIN_ID: ${domain_id:-Not set}" \
        --preview="ls -la {} 2>/dev/null || echo 'Special option'")

    if [[ -z "$selection" ]]; then
        echo "Cancelling."
        return 1
    fi

    if [[ "$selection" == "Change ROS_DOMAIN_ID" ]]; then
        printf "Please enter new ${BLUE_TXT}ROS_DOMAIN_ID${NC} (empty to disable): "
        read new_domain
        if [[ -z "${new_domain}" ]]; then
            echo "Disabling domain"
            rm -f $ROS_DOMAIN_ID_FILE
            unset ROS_DOMAIN_ID
        else
            set_ros_domain_id $new_domain
        fi
        return 0
    else
        num=0
        for i in "${arrIN[@]}"; do
            num=$((num + 1))
            if [[ "${i}" = "$selection" ]]; then
                echo "Sourcing WS: $selection"
                break
            fi
        done
    fi
}

function rebuild_curr_ws() {
    get_current_ws
    if [[ -z "$curr_ws" ]]; then
        printf "${RED_TXT}No ROS workspace selected. Use ${BLUE_TXT}select_ws${RED_TXT} to select one.${NC}\n"
        return 1
    fi

    if [[ ! -d "$curr_ws" ]]; then
        printf "${RED_TXT}Selected workspace does not exist: ${WHITE_TXT}$curr_ws${NC}\n"
        return 1
    fi

    # Get current directory to return to it later
    pushd . >/dev/null || {
        printf "${RED_TXT}Failed to save current directory.${NC}\n"
        return 1
    }

    cd "$curr_ws" || {
        printf "${RED_TXT}Failed to change to workspace directory.${NC}\n"
        popd >/dev/null || exit
        return 1
    }

    printf "${BLUE_TXT}Rebuilding workspace:${WHITE_TXT} $curr_ws ${NC}\n"

    local build_status=0
    csr

    colcon build --symlink-install "$@"
    build_status=$?

    if [[ "${ROSHACKS_REBUILD_RECACHE_ALIASES:-0}" == "1" ]] || [[ ! -f "${WS_ALIASES_FILE}" ]]; then
        cache_ws_aliases "$curr_ws"
    fi

    # Source the workspace
    source_ws "$curr_ws"

    # Return to the original directory
    popd >/dev/null || exit

    if [[ $build_status -eq 0 ]]; then
        printf "${GREEN_TXT}Workspace rebuild completed successfully.${NC}\n"
    else
        printf "${RED_TXT}Workspace rebuild completed with errors. Check the build logs.${NC}\n"
        for pkg in $curr_ws/log/latest_build/*/; do
            pkg_name=$(basename "$pkg")
            log_file="$pkg/stdout_stderr.log"
            if grep -qiw "error" "$log_file"; then
                echo -e "\n===== 🔴 Error in $pkg_name ====="
                tail -n 50 "$log_file"
            fi
        done

    fi

    return $build_status
}

function get_version_stamp() {
    if [[ -z "$curr_ws" ]]; then
        printf "${RED_TXT}No workspace selected.${NC}\n"
        return 1
    fi

    repo_list=$(find $curr_ws -name .git -type d -prune)
    info_file="$curr_ws/install/release_info.txt"
    echo "Workspace: $(basename $curr_ws)" >$info_file
    echo "===========================" >>$info_file
    echo $(uname -a) >>$info_file
    echo $(date) >>$info_file
    echo "===========================" >>$info_file
    echo "ROS2 Distribution: $ROS2_NAME" >>$info_file
    echo "===========================" >>$info_file
    echo "Git Repositories:" >>$info_file

    for repo in $repo_list; do
        pushd $curr_ws >/dev/null
        cd $repo/..
        echo "$(basename $PWD) $(git rev-parse HEAD) ($(git describe --tags --always))" >>$info_file
        popd >/dev/null
    done

    cat $info_file
}

function build_release() {
    get_current_ws

    if [[ -z "$curr_ws" ]]; then
        printf "${RED_TXT}No workspace selected.${NC}\n"
        return 1
    fi

    printf "Building deployable package of ${WHITE_TXT}$curr_ws${NC}\n"
    unROS
    cd $curr_ws
    source /opt/ros/${ROS2_NAME}/setup.bash

    # Check for version scripts
    pkg_list=$(colcon list -p)
    for pkg_path in $pkg_list; do
        if [[ -f "$curr_ws/$pkg_path/scripts/version_release.py" ]]; then
            cd $curr_ws/$pkg_path
            python3 $curr_ws/$pkg_path/scripts/version_release.py
            cd $curr_ws
        fi
    done

    if [[ ! -d "$curr_ws/releases" ]]; then
        mkdir -p "$curr_ws/releases"
    fi

    version_name="$(basename $curr_ws)_release_$(date '+%Y-%m-%d_%H-%M-%S')"
    get_version_stamp

    # Clean and build release version
    clean_ros2_ws $curr_ws &&
        colcon build --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release &&
        tar czf $curr_ws/releases/${version_name}.tar.gz -C $curr_ws install

    if [[ $? -eq 0 ]]; then
        printf "${GREEN_TXT}Release package created: ${WHITE_TXT}$curr_ws/releases/${version_name}.tar.gz${NC}\n"
    else
        printf "${RED_TXT}Failed to create release package.${NC}\n"
    fi
}

function ros2cd() {
    pkgname=${1:-""}
    if [[ -z "${pkgname}" ]]; then
        printf "${RED_TXT}ROS2 package name not specified.${NC}\n"
        return 1
    fi

    get_current_ws
    cd $(ros2 pkg prefix $pkgname)
    return 0
}

# ==========================================================
# Quick Command Functions
# ==========================================================
 # Moved to lib/quickcmd.sh

# ==========================================================
# System Utility Functions
# ==========================================================
# Moved to lib/system.sh

# ==========================================================
# ROS2 Message Debugging Functions
# ==========================================================
# Moved to lib/topic_tools.sh

# ==========================================================
# Launch File Selection Functions
# ==========================================================

# Interactive launch file selector using fzf
# Usage: launch_select
function launch_select() {
    get_current_ws
    if [[ -z "${curr_ws}" ]]; then
        printf "${RED_TXT}No current workspace set.${NC}\n"
        return 1
    fi
    
    if [[ ! -d "${curr_ws}/src" ]]; then
        printf "${RED_TXT}No src directory found in workspace: ${curr_ws}${NC}\n"
        return 1
    fi
    
    # Check if fzf is available
    if ! command -v fzf &> /dev/null; then
        printf "${RED_TXT}fzf is not installed. Please install fzf to use this feature.${NC}\n"
        return 1
    fi
    
    # Find all launch files in the workspace
    local launch_files=()
    local display_entries=()
    
    printf "${BLUE_TXT}Searching for launch files...${NC}\n"
    
    while IFS= read -r launch_file || [[ -n "$launch_file" ]]; do
        [[ -z "$launch_file" ]] && continue
        # Get relative path from src directory
        local rel_path="${launch_file#${curr_ws}/src/}"
        
        # Find the actual package name by looking for package.xml
        local package_name=""
        local current_dir=$(dirname "$launch_file")
        
        # Traverse up the directory tree until we find package.xml
        while [[ "$current_dir" != "${curr_ws}/src" && "$current_dir" != "/" ]]; do
            if [[ -f "$current_dir/package.xml" ]]; then
                # Extract package name from package.xml
                package_name=$(grep -oP '<name>\K[^<]+' "$current_dir/package.xml" 2>/dev/null | head -1)
                break
            fi
            current_dir=$(dirname "$current_dir")
        done
        
        # Fallback to first directory if package.xml not found
        if [[ -z "$package_name" ]]; then
            package_name=$(echo "$rel_path" | cut -d'/' -f1)
        fi
        
        # Get just the launch file name
        local launch_name=$(basename "$launch_file")
        
        # Store both full path and display format
        launch_files+=("$launch_file")
        display_entries+=("${package_name}/${launch_name}"$'\t'"${launch_file}")
        
    done < <(find "${curr_ws}/src" -type f \( -name "*.launch.py" -o -name "*.launch" \) 2>/dev/null)
    
    if [[ ${#launch_files[@]} -eq 0 ]]; then
        printf "${YELLOW_TXT}No launch files found in workspace.${NC}\n"
        return 0
    fi
    
    printf "${GREEN_TXT}Found ${#launch_files[@]} launch file(s)${NC}\n"
    
    # Use fzf to select launch file
    local selected_display
    selected_display=$(printf '%s\n' "${display_entries[@]}" | \
        fzf --prompt="Select launch file: " \
            --height=40% \
            --border \
            --preview-window=right:50% \
            --preview='p=$(printf "%s" {} | cut -f2); echo "Launch file: $(basename "$p")"; echo "Path: $p"' \
            --with-nth=1 \
            --delimiter='\t')
    
    if [[ -z "$selected_display" ]]; then
        printf "${YELLOW_TXT}No launch file selected.${NC}\n"
        return 0
    fi
    
    selected_launch=$(printf '%s' "$selected_display" | cut -f2)
    selected_display=$(printf '%s' "$selected_display" | cut -f1)
    
    # Extract package name and launch file name
    local package_name=$(echo "$selected_display" | cut -d'/' -f1)
    local launch_name=$(echo "$selected_display" | cut -d'/' -f2)
    
    local launch_cmd="ros2 launch ${package_name} ${launch_name}"
    
    printf "${GREEN_TXT}Selected: ${WHITE_TXT}${package_name}/${launch_name}${NC}\n"
    
    user_command="$launch_cmd"
    if [[ -n ${ZSH_VERSION:-} ]]; then
        vared -p "$ " user_command
    else
        read -e -i "$launch_cmd" -p "$ " user_command
    fi
    
    # Execute the command (which might be modified by the user)
    if [[ -n "$user_command" ]]; then
        eval "$user_command"
        # Add the command to bash history
        history -s "$user_command"
    fi
}

# ==========================================================
# ROS2 Utility Functions
# ==========================================================

function clean_ros2_ws() {
    ws=${1:-""}
    if [[ -z "${ws}" ]]; then
        printf "${RED_TXT}ROS2 workspace path not specified.${NC}\n"
        return 1
    fi

    printf "${YELLOW_TXT}Clearing the workspace: ${ws} ${NC}\n"
    find ${ws}/log -mindepth 1 -maxdepth 1 -type d -print0 | xargs -0 rm -R >/dev/null 2>&1
    find ${ws}/install -mindepth 1 -maxdepth 1 -type d -print0 | xargs -0 rm -R >/dev/null 2>&1
    find ${ws}/build -mindepth 1 -maxdepth 1 -type d -print0 | xargs -0 rm -R >/dev/null 2>&1
    printf "${GREEN_TXT}Workspace cleaned successfully.${NC}\n"

    return 0
}

function find_ws() {
    local paths_file="${WS_SEARCH_PATHS_FILE}"

    # Ensure the paths file exists and contains at least the HOME directory
    if [[ ! -f "${paths_file}" ]]; then
        mkdir -p "$(dirname "${paths_file}")"
        echo "$HOME" >"${paths_file}"
    fi

    local ws_dirs=()

    # Read search paths line by line
    while IFS= read -r search_path || [[ -n "$search_path" ]]; do
        # Skip empty lines and comments
        [[ -z "${search_path}" || "${search_path}" =~ ^# ]] && continue
        if [[ -d "${search_path}" ]]; then
            # Find workspace-like directories directly under the search path
            while IFS= read -r d; do
                ws_dirs+=("$d")
            done < <(find "${search_path}" -maxdepth 1 -type d -name "*ws*" 2>/dev/null)
        fi
    done <"${paths_file}"

    # Remove duplicates, sort, and populate global array arrIN
    arrIN=()
    while IFS= read -r d || [[ -n "$d" ]]; do
        [[ -z "$d" ]] && continue
        arrIN+=("$d")
    done < <(printf "%s\n" "${ws_dirs[@]}" | sort -u)
    # print_ws
}

function print_domain_info() {
    get_ros_domain_id
    # printf "${BLUE_TXT}ROS_DOMAIN_ID${NC}: ${LIGHT_BLUE_TXT}$domain_id${NC}\n"
}

function ask_for_num() {
    max=${1:-"1"}
    printf "Select a number [1-$max] or cancel : "
    if [[ -n ${ZSH_VERSION:-} ]]; then
        if (( max < 10 )); then
            read -r -k 1 num
        else
            read -r -k 2 num
        fi
    else
        if (( max < 10 )); then
            read -r -n 1 num
        else
            read -r -n 2 num
        fi
    fi
    echo ""
    case $num in
    [123456789]*)
        echo ""
        ;;
    *)
        echo ""
        echo "Cancelling."
        return 1
        ;;
    esac

    return 0
}

# Adds a new directory to the workspace search paths
# Usage: add_ws_path <path>
function add_ws_path() {
    local new_path="${1:-""}"

    if [[ -z "${new_path}" ]]; then
        printf "${RED_TXT}No path specified.${NC}\n"
        return 1
    fi

    if [[ ! -d "${new_path}" ]]; then
        printf "${RED_TXT}Path '${new_path}' does not exist.${NC}\n"
        return 1
    fi

    mkdir -p "$(dirname "${WS_SEARCH_PATHS_FILE}")"
    # Create the file if it does not exist
    touch "${WS_SEARCH_PATHS_FILE}"

    # Check if the path is already in the file
    if grep -Fxq "${new_path}" "${WS_SEARCH_PATHS_FILE}"; then
        printf "${YELLOW_TXT}Path already exists in search list.${NC}\n"
        return 0
    fi

    echo "${new_path}" >>"${WS_SEARCH_PATHS_FILE}"
    printf "${GREEN_TXT}Added '${new_path}' to workspace search paths.${NC}\n"
}
