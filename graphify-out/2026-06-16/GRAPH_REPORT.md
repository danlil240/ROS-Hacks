# Graph Report - /home/daniel/projects/ROS-Hacks  (2026-06-16)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 130 nodes · 166 edges · 22 communities (7 shown, 15 thin omitted)
- Extraction: 100% EXTRACTED · 0% INFERRED · 0% AMBIGUOUS
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `49ff2f59`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- [[_COMMUNITY_Workspace Management|Workspace Management]]
- [[_COMMUNITY_ROS Command Shortcuts|ROS Command Shortcuts]]
- [[_COMMUNITY_System Diagnostics|System Diagnostics]]
- [[_COMMUNITY_Package Repository Setup|Package Repository Setup]]
- [[_COMMUNITY_Build Log Analysis|Build Log Analysis]]
- [[_COMMUNITY_Environment Setup|Environment Setup]]
- [[_COMMUNITY_Quick Command Execution|Quick Command Execution]]
- [[_COMMUNITY_Development Tools|Development Tools]]
- [[_COMMUNITY_ROS Topic Monitoring|ROS Topic Monitoring]]
- [[_COMMUNITY_ROS Environment Variables|ROS Environment Variables]]
- [[_COMMUNITY_ROS Command Completions|ROS Command Completions]]
- [[_COMMUNITY_Shell Aliases|Shell Aliases]]
- [[_COMMUNITY_System Fixes|System Fixes]]
- [[_COMMUNITY_GPG Key Backup|GPG Key Backup]]
- [[_COMMUNITY_Compile Commands Generation|Compile Commands Generation]]
- [[_COMMUNITY_VSCode Package Config|VSCode Package Config]]
- [[_COMMUNITY_VSCode Workspace Config|VSCode Workspace Config]]
- [[_COMMUNITY_APT Installation|APT Installation]]
- [[_COMMUNITY_ROS Execution Scripts|ROS Execution Scripts]]
- [[_COMMUNITY_Build and Release Workflow|Build and Release Workflow]]
- [[_COMMUNITY_Build Configuration Defaults|Build Configuration Defaults]]
- [[_COMMUNITY_Disabled APT Workflow|Disabled APT Workflow]]

## God Nodes (most connected - your core abstractions)
1. `ROS-Hacks` - 11 edges
2. `run_diagnostics()` - 10 edges
3. `get_current_ws()` - 9 edges
4. `source_ws()` - 9 edges
5. `setup-apt-repo.sh script` - 8 edges
6. `main()` - 7 edges
7. `createWS()` - 6 edges
8. `ask_for_ws_and_domain()` - 6 edges
9. `unROS()` - 5 edges
10. `select_ws()` - 5 edges

## Surprising Connections (you probably didn't know these)
- `ROS-Hacks Completions` --references--> `ROS-Hacks`  [EXTRACTED]
  completions/README.md → README.md
- `Powerlevel10k Integration` --references--> `ROS-Hacks Powerlevel10k Integration`  [EXTRACTED]
  README.md → docs/P10K-INTEGRATION.md

## Import Cycles
- None detected.

## Communities (22 total, 15 thin omitted)

### Community 0 - "Workspace Management"
Cohesion: 0.16
Nodes (24): ask_for_ws_and_domain(), build_release(), cache_ws_aliases(), clean_ros2_ws(), createWS(), determine_ws_ros_version(), find_ws(), get_current_ws() (+16 more)

### Community 1 - "ROS Command Shortcuts"
Cohesion: 0.15
Nodes (13): ROS-Hacks Completions, ROS-Hacks Powerlevel10k Integration, Build Commands, Keyboard Shortcuts, Powerlevel10k Integration, Quick Commands, ROS2 Command Shortcuts, ROS2 Workspace Management (+5 more)

### Community 2 - "System Diagnostics"
Cohesion: 0.30
Nodes (10): check_bash_config(), check_dependencies(), check_domain_id(), check_file(), check_workspace(), check_zsh_config(), fix_problems(), print_header() (+2 more)

### Community 3 - "Package Repository Setup"
Cohesion: 0.42
Nodes (9): setup-apt-repo.sh script, add_package(), build_package(), create_instructions(), increment_version(), install_local(), setup_github(), show_instructions() (+1 more)

### Community 4 - "Build Log Analysis"
Cohesion: 0.25
Nodes (3): build.sh script, _rh_errors_summary(), _rh_pkg_summary_line()

### Community 5 - "Environment Setup"
Cohesion: 0.46
Nodes (7): setup.sh script, check_dependencies(), configure_bashrc(), configure_inputrc(), configure_zshrc(), create_initial_configs(), main()

### Community 9 - "ROS Environment Variables"
Cohesion: 0.33
Nodes (4): ROS-Hacks.sh script, COLCON_DEFAULTS_FILE, COLCON_HOME, ROSHACKS_LOADED

## Knowledge Gaps
- **31 isolated node(s):** `ROS-Hacks.sh script`, `COLCON_HOME`, `COLCON_DEFAULTS_FILE`, `ROSHACKS_LOADED`, `aliases.sh script` (+26 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **15 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **What connects `ROS-Hacks.sh script`, `COLCON_HOME`, `COLCON_DEFAULTS_FILE` to the rest of the system?**
  _31 weakly-connected nodes found - possible documentation gaps or missing edges._