# Graph Report - ROS-Hacks  (2026-06-19)

## Corpus Check
- 29 files · ~15,773 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 214 nodes · 270 edges · 31 communities (13 shown, 18 thin omitted)
- Extraction: 100% EXTRACTED · 0% INFERRED · 0% AMBIGUOUS
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `f0f87e8f`
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
- [[_COMMUNITY_Community 22|Community 22]]
- [[_COMMUNITY_Community 23|Community 23]]
- [[_COMMUNITY_Community 24|Community 24]]
- [[_COMMUNITY_Community 25|Community 25]]
- [[_COMMUNITY_Community 26|Community 26]]
- [[_COMMUNITY_Community 27|Community 27]]
- [[_COMMUNITY_Community 28|Community 28]]
- [[_COMMUNITY_Community 29|Community 29]]
- [[_COMMUNITY_Community 30|Community 30]]

## God Nodes (most connected - your core abstractions)
1. `ROS-Hacks` - 15 edges
2. `Features and Usage` - 11 edges
3. `run_diagnostics()` - 10 edges
4. `setup-apt-repo.sh script` - 10 edges
5. `ROS-Hacks Release Specialist` - 10 edges
6. `get_current_ws()` - 9 edges
7. `source_ws()` - 9 edges
8. `Supported Commands` - 8 edges
9. `main()` - 7 edges
10. `createWS()` - 6 edges

## Surprising Connections (you probably didn't know these)
- None detected - all connections are within the same source files.

## Import Cycles
- None detected.

## Communities (31 total, 18 thin omitted)

### Community 0 - "Workspace Management"
Cohesion: 0.14
Nodes (29): ask_for_ws_and_domain(), build_release(), cache_ws_aliases(), clean_ros2_ws(), clean_source_ros_distro(), createWS(), csr(), determine_ws_ros_version() (+21 more)

### Community 1 - "ROS Command Shortcuts"
Cohesion: 0.20
Nodes (15): Adding this repository to your system, Build Commands, Features and Usage, Keyboard Shortcuts, Keyboard Shortcuts (via inputrc), Powerlevel10k Integration, Quick Commands, ROS2 Command Shortcuts (+7 more)

### Community 2 - "System Diagnostics"
Cohesion: 0.30
Nodes (10): check_bash_config(), check_dependencies(), check_domain_id(), check_file(), check_workspace(), check_zsh_config(), fix_problems(), print_header() (+2 more)

### Community 3 - "Package Repository Setup"
Cohesion: 0.26
Nodes (12): setup-apt-repo.sh script, add_package(), build_package(), clean_build_artifacts(), create_instructions(), export_signing_key_backup(), increment_version(), install_local() (+4 more)

### Community 4 - "Build Log Analysis"
Cohesion: 0.25
Nodes (3): build.sh script, _rh_errors_summary(), _rh_pkg_summary_line()

### Community 5 - "Environment Setup"
Cohesion: 0.46
Nodes (7): setup.sh script, check_dependencies(), configure_bashrc(), configure_inputrc(), configure_zshrc(), create_initial_configs(), main()

### Community 9 - "ROS Environment Variables"
Cohesion: 0.33
Nodes (4): ROS-Hacks.sh script, COLCON_DEFAULTS_FILE, COLCON_HOME, ROSHACKS_LOADED

### Community 22 - "Community 22"
Cohesion: 0.13
Nodes (14): Bash, Build Tools, Development Tools, Files, Installation, Package Management, ROS2 Aliases, ROS-Hacks Completions (+6 more)

### Community 23 - "Community 23"
Cohesion: 0.18
Nodes (10): Configuration, Customization, Force prompt refresh, Installation, Method 1: Add segments to your p10k configuration, Method 2: Use p10k configure command, Method 3: Manual configuration, ROS-Hacks Powerlevel10k Integration (+2 more)

### Community 24 - "Community 24"
Cohesion: 0.40
Nodes (4): Agent instructions — ROS-Hacks, APT signing key — DO NOT CHANGE, Never do these, Publishing APT packages

### Community 27 - "Community 27"
Cohesion: 0.39
Nodes (7): stdout-log.sh script, __rh_append_stdout_log(), __rh_init_stdout_log(), __rh_log_section(), rh_print(), __rh_source_quiet(), show_ros_hacks_stdout()

### Community 28 - "Community 28"
Cohesion: 0.13
Nodes (14): Additional resources, Automated release pipeline, Failure recovery, GPG signing key — DO NOT CHANGE, Never do these, One-time setup (only if dispatch fails), PR / main branch policy, Repositories (+6 more)

### Community 29 - "Community 29"
Cohesion: 0.33
Nodes (5): Canonical signing key recovery, CI monitoring shortcuts, Fine-grained PAT for `ROS_HACKS_APT_DISPATCH_TOKEN`, Manual APT retry (fallback), ROS-Hacks release reference

## Knowledge Gaps
- **61 isolated node(s):** `ROS-Hacks.sh script`, `COLCON_HOME`, `COLCON_DEFAULTS_FILE`, `ROSHACKS_LOADED`, `aliases.sh script` (+56 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **18 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `ROS-Hacks` connect `ROS Command Shortcuts` to `Community 22`?**
  _High betweenness centrality (0.024) - this node is a cross-community bridge._
- **Why does `Powerlevel10k Integration` connect `ROS Command Shortcuts` to `Community 23`?**
  _High betweenness centrality (0.015) - this node is a cross-community bridge._
- **What connects `ROS-Hacks.sh script`, `COLCON_HOME`, `COLCON_DEFAULTS_FILE` to the rest of the system?**
  _61 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Workspace Management` be split into smaller, more focused modules?**
  _Cohesion score 0.13709677419354838 - nodes in this community are weakly interconnected._
- **Should `Community 22` be split into smaller, more focused modules?**
  _Cohesion score 0.13333333333333333 - nodes in this community are weakly interconnected._
- **Should `Community 28` be split into smaller, more focused modules?**
  _Cohesion score 0.13333333333333333 - nodes in this community are weakly interconnected._