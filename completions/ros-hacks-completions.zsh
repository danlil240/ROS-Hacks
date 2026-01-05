#compdef ros-hacks

# ZSH completions for ROS-Hacks helpers

# Return workspace root, fallback to current dir
_rh_ws_root() {
    local ws_file="${WS_FILE:-$HOME/.cache/ros-hacks/ros_ws_selected}"
    if [[ -f "$ws_file" ]]; then
        cat "$ws_file"
        return
    fi
    echo "$PWD"
}

# Complete ROS packages (prefers colcon list, falls back to ros2 pkg list)
_rh_complete_pkg() {
    local ws="$(_rh_ws_root)"
    local pkgs
    
    if command -v colcon >/dev/null 2>&1; then
        pkgs=$(cd "$ws" 2>/dev/null && colcon list --names-only 2>/dev/null)
    fi
    
    if [[ -z "$pkgs" ]] && command -v ros2 >/dev/null 2>&1; then
        pkgs=$(ros2 pkg list 2>/dev/null | cut -d' ' -f1)
    fi
    
    _describe 'ROS packages' pkgs
}

# Complete ROS topics
_rh_complete_topic() {
    local topics
    if command -v ros2 >/dev/null 2>&1; then
        topics=$(ros2 topic list 2>/dev/null)
    fi
    _describe 'ROS topics' topics
}

# Complete workspaces
_rh_complete_ws() {
    local ws_file="${WS_SEARCH_PATHS_FILE:-$HOME/.cache/ros-hacks/ws_search_paths}"
    local workspaces
    
    if [[ -f "$ws_file" ]]; then
        workspaces=$(cat "$ws_file" 2>/dev/null)
    fi
    
    # Also add common workspace locations
    workspaces="$workspaces $HOME/*_ws"
    
    # Expand and filter to actual directories
    local expanded_ws
    for ws in $workspaces; do
        if [[ -d "$ws" ]]; then
            expanded_ws="$expanded_ws ${ws##*/}"
        fi
    done
    
    _describe 'ROS workspaces' expanded_ws
}

# Complete ROS domain IDs
_rh_complete_domain_id() {
    local domains="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15"
    _describe 'ROS domain IDs' domains
}

# Complete colcon log files
_rh_complete_colcon_log() {
    local ws="$(_rh_ws_root)"
    local log_dir="$ws/log"
    local packages
    
    if [[ -d "$log_dir" ]]; then
        packages=$(find "$log_dir" -mindepth 1 -maxdepth 1 -type d -exec basename {} \; 2>/dev/null)
    fi
    
    _describe 'colcon log packages' packages
}

# Complete utility commands
_rh_complete_utility() {
    local -a utilities
    utilities=(
        'pR:Print ROS-related environment variables'
        'sw:Source current workspace'
        'sr:Source ROS2 setup'
        'csr:Clear ROS and source ROS2 setup'
        'cob:Colcon build with symlink install'
        'cobd:Colcon build debug'
        'cobr:Colcon build release'
        'coc:Clean current workspace'
        'cw:Change to workspace directory'
        'cs:Change to workspace src directory'
        'cg:Create COLCON_IGNORE'
        'rcg:Remove COLCON_IGNORE'
        'rt:ROS2 topic'
        'rtl:ROS2 topic list'
        'rn:ROS2 node'
        'rnl:ROS2 node list'
        'rs:ROS2 service'
        'rsl:ROS2 service list'
        'rp:ROS2 param'
        'rpl:ROS2 param list'
        'ra:ROS2 action'
        'ral:ROS2 action list'
        'rb:ROS2 bag'
        'rl:ROS2 launch'
        'whgit:Get git remote URL'
        'se:Show colcon errors'
        'launch:Interactive launch selector'
    )
    _describe 'utility commands' utilities
}

# Complete ROS2 command subcommands
_rh_complete_ros2_cmd() {
    local cmd=$line[1]
    local subcmd=$line[2]
    
    case $cmd in
        rt|ros2\ topic)
            case $subcmd in
                list|echo|info|hz|bw|pub|type)
                    # For topic-specific commands, complete topics
                    _rh_complete_topic
                    ;;
                *)
                    # Complete topic subcommands
                    local -a subcmds
                    subcmds=(list echo info hz bw pub type)
                    _describe 'topic subcommands' subcmds
                    ;;
            esac
            ;;
        rn|ros2\ node)
            case $subcmd in
                list|info)
                    # Complete nodes
                    local nodes
                    if command -v ros2 >/dev/null 2>&1; then
                        nodes=$(ros2 node list 2>/dev/null)
                    fi
                    _describe 'ROS nodes' nodes
                    ;;
                *)
                    local -a subcmds
                    subcmds=(list info)
                    _describe 'node subcommands' subcmds
                    ;;
            esac
            ;;
        rs|ros2\ service)
            case $subcmd in
                list|type|call)
                    # Complete services
                    local services
                    if command -v ros2 >/dev/null 2>&1; then
                        services=$(ros2 service list 2>/dev/null)
                    fi
                    _describe 'ROS services' services
                    ;;
                *)
                    local -a subcmds
                    subcmds=(list type call)
                    _describe 'service subcommands' subcmds
                    ;;
            esac
            ;;
        rp|ros2\ param)
            case $subcmd in
                list|get|set|describe)
                    # Complete nodes first, then parameters
                    local nodes
                    if command -v ros2 >/dev/null 2>&1; then
                        nodes=$(ros2 node list 2>/dev/null)
                    fi
                    _describe 'ROS nodes' nodes
                    ;;
                *)
                    local -a subcmds
                    subcmds=(list get set describe)
                    _describe 'parameter subcommands' subcmds
                    ;;
            esac
            ;;
        *)
            # Default file completion
            _files
            ;;
    esac
}

# Main completion function for ros-hacks commands
_ros_hacks() {
    local context state state_descr line
    typeset -A opt_args

    _arguments -C \
        '1: :->command' \
        '*:: :->args' \
        && return 0

    case $state in
        command)
            local -a commands
            commands=(
                'cobp:Build specific packages with colcon'
                'cobput:Build packages up to specific package with colcon'
                'ros2cd:Change directory to ROS package'
                'open_colcon_log:Open colcon build log for package'
                'ros2_topic_monitor:Monitor ROS topic with echo'
                'ros2_topic_hz:Show ROS topic frequency'
                'ros2_topic_bw:Show ROS topic bandwidth'
                'rte:ROS topic echo'
                'rti:ROS topic info'
                'select_ws:Select ROS workspace'
                'set_current_ws:Set current ROS workspace'
                'set_ros_domain_id:Set ROS domain ID'
                'source_ws:Source ROS workspace'
                'createWS:Create new ROS workspace'
                'clean_ros2_ws:Clean ROS workspace'
                'rebuild_curr_ws:Rebuild current workspace'
                'build_release:Build release version'
                'launch_select:Interactive launch file selector'
                'gencc:Generate compile_commands.json'
                'vscgen:Generate VS Code configuration'
                'vscgen_pkg:Generate VS Code config for package'
                'rir:Run command in ROS environment'
                'show_colcon_errors:Show colcon build errors'
                'show_colcon_warnings:Show colcon build warnings'
                'fixJB:Fix JetBrains IDE desktop files'
                'unROS:Unset ROS environment variables'
                'print_ros_domain_id:Print current ROS domain ID'
                'get_current_ws:Get current workspace path'
                'get_current_ws_name:Get current workspace name'
                'get_version_stamp:Get workspace version stamp'
                'cache_ws_aliases:Cache workspace aliases'
                'source_cached_aliases:Source cached aliases'
                'ask_for_ws_and_domain:Interactive workspace/domain selector'
                'determine_ws_ros_version:Determine workspace ROS version'
                'add_ws_path:Add workspace search path'
                'set-quick-command:Set quick command'
                'get-quick-command:Get quick command'
                'print-quick-command:Print quick command'
                'exec-quick-command:Execute quick command'
                'kill-tmux-quick-command:Kill quick command tmux session'
                'ros2_pkg_list:List installed packages'
            )
            _describe 'command' commands
            ;;
        args)
            local cmd=$line[1]
            case $cmd in
                cobp|cobput|ros2cd)
                    _rh_complete_pkg
                    ;;
                open_colcon_log)
                    _rh_complete_colcon_log
                    ;;
                ros2_topic_monitor|ros2_topic_hz|ros2_topic_bw|rte|rti)
                    _rh_complete_topic
                    ;;
                select_ws|set_current_ws|source_ws|clean_ros2_ws|createWS|gencc|vscgen)
                    _rh_complete_ws
                    ;;
                set_ros_domain_id)
                    _rh_complete_domain_id
                    ;;
                vscgen_pkg)
                    _files -/
                    ;;
                rir)
                    _files
                    ;;
                *)
                    # Default completion - try to complete files
                    _files
                    ;;
            esac
            ;;
    esac
}

# Register completion functions for specific commands
compdef _rh_complete_pkg cobp cobput ros2cd open_colcon_log
compdef _rh_complete_topic ros2_topic_monitor ros2_topic_hz ros2_topic_bw rte rti
compdef _rh_complete_ws select_ws set_current_ws source_ws clean_ros2_ws createWS gencc vscgen
compdef _rh_complete_domain_id set_ros_domain_id
compdef _rh_complete_colcon_log open_colcon_log

# Register ROS2 command completions
compdef _rh_complete_ros2_cmd rt rtl rn rnl rs rsl rp rpl ra ral rb rl

# Register utility completions
compdef _rh_complete_utility pR sw sr csr cob cobd cobr coc cw cs cg rcg whgit se launch

# Main completion for the ros-hacks command itself
compdef _ros_hacks ros-hacks

# Also register for individual function completion
for func in cobp cobput ros2cd open_colcon_log ros2_topic_monitor ros2_topic_hz ros2_topic_bw rte rti select_ws set_current_ws set_ros_domain_id source_ws createWS clean_ros2_ws rebuild_curr_ws build_release launch_select gencc vscgen vscgen_pkg rir show_colcon_errors show_colcon_warnings; do
    if typeset -f "$func" >/dev/null 2>&1; then
        compdef _ros_hacks "$func"
    fi
done
