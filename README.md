![ROS](https://upload.wikimedia.org/wikipedia/commons/b/bb/Ros_logo.svg)
# ROS Hacks Repo

## Overview
The repository is designed to make ROS developer's life easier.
After the installation usefull aliases and functions will be added to the terminal.
Everything was tested on Ubuntu 22.04 with ROS2 humble.

## Installation

### Option 1: Install via APT (Recommended)

The easiest way to install ROS Hacks is via APT package manager:

```shell
# Add the GPG key for the ROS-Hacks repository
wget -qO - https://your-github-username.github.io/ros-hacks-apt-repo/ros-hacks-apt.key | sudo apt-key add -

# Add the repository to your sources
echo "deb https://your-github-username.github.io/ros-hacks-apt-repo stable main" | sudo tee /etc/apt/sources.list.d/ros-hacks.list

# Update package lists and install ROS-Hacks
sudo apt update
sudo apt install ros-hacks

# Run the setup script to complete installation
sudo ros-hacks-setup
```

### Option 2: Manual Installation

Make sure that `tmux` and `fzf` is installed:

```shell
sudo apt install tmux

git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf
~/.fzf/install
```

Clone the repo into some folder, for example:

```shell
git clone https://github.com/danlil240/ROS-Hacks.git ~/.ROS-Hacks
```

Navigate to the directory and execute the `setup.sh` script.

```shell
cd ~/.ROS-Hacks;  bash setup.sh
```

The `~/.inputrc` file is saved to  `~/.inputrc.bak` prior to being overwritten.

## Terminal shortcuts
The shortcuts can be run in a **new** terminal after the installation. _Note: re-sourcing `~/.bashrc` isn't enough._

Complete list of the shortcuts can be seen in  [`inputrc`](inputrc) file.
Few of them are presented below:

| Shortcut | Executed command | Description |
| ------ | ------ |  ------ |
| F3 | `select_ws` | Displays ROS workspace selection dialog. |
| Shift-F3 | `prompt_new_ws` | Displays new ROS workspace creation dialog. |
| F5 | `source ~/.bashrc` | You know what it does :) |
| F9 | `rebuild_curr_ws` | Re-builds the currently selected workspace and sources it. |
| Shift-F12 | `set-quick-command` | Saves currently typed-in command for quick-launch<sup>1</sup>. |
| F12 | `exec-quick-command` | Executes the saved command in detached tmux session. |
| ... | .... |....... |

<sup>1</sup> The function is primarily targeted for quick launching and killing of Gazebo worlds using various `roslaunch` commands. A quick execution commands are saved for each workspace separately.

Example of usage of Quick Command:
Type in terminal `roslaunch gazebo_ros empty_world.launch`, hit **`Shift-F12`**. The command will be saved for currently sourced workspace. Hit **`F12`** to execute the command in background tmux session. The Gazebo client GUI will show up. Hit  **`Ctrl-F12`** to stop the session, killing Gazebo along with all ROS nodes (currently only on ROS1).

## Aliases
Complete list of the aliases can be seen in  [`aliases.sh`](aliases.sh) file.
Few of them are presented below:

| Alias | Expanded command | Description |
| ------ | ------ |  ------ |
| `sc` | `source_ws $(cat $WS_FILE)` | Sources the currently selected workspace. |
| `o` | `sudo chown -R $USER:$USER` | Allows easy ownership change for selected files. |
| `x` | `chmod +x ` | Adds execution permissions for desired files. |
| ... | .... |....... |

## Functions

### Quick Launch 
| Function | Description |
| ------ | ------ |
| `set-quick-command` | Saves the command for further use. |
| `get-quick-command` | Loads the command. |
| `print-quick-command` | Displays the command. |
| `exec-quick-command` | Executes the command in background *tmux* session. |
| `kill-tmux-quick-command` | Ends the execution of the *tmux* session, along with *Gazebo* and **ALL** ROS nodes. |
| ... | .... |

### ROS Workspace 

A simple UI for ROS workspace creation and selection is defined in several functions.

| Function | Description |
| ------ | ------ |
| `select_ws` | Shows dialog for WS selections |
| `prompt_new_ws` | Shows dialog for new ROS workspace creation (ROS1/ROS2). |
| `rebuild_curr_ws` | Rebuilds ROS workspace (ROS1 - catkin/ROS2 - colcon). |
| ... | .... |

### Miscellaneous 
| Function | Description |
| ------ | ------ |
| `RM` | Sets `ROS_MASTER_URI` variable, optionally in local network. |
| `rt` | Executes `rostopic` with grep for desired argument. |
| `fixJB` | Updates JetBrains' shortcuts for CLion and PyCharm to be launched within ROS workspace. |
| `unROS` | Cleans all ROS-related environment variables. |
| ... | .... |


## Removal

### If installed via APT
```shell
sudo apt remove ros-hacks
```
This will remove the package but may leave configuration files. To remove everything:
```shell
sudo apt purge ros-hacks
```

### If installed manually
Clear `~/.bashrc` file from the added lines. Delete the repo. Voilà!

## GitHub Repository Management

This project uses GitHub Actions to automatically build Debian packages when changes are pushed or new tags are created. The workflow builds the package and creates a release when a tag with format `v*` (e.g., `v1.0.1`) is pushed.

### Creating a New Release

1. Update the VERSION file with the new version number
2. Update the debian/changelog file with the new version and changes
3. Commit your changes
4. Create and push a new tag:
   ```shell
   git tag v1.0.1
   git push origin v1.0.1
   ```
5. GitHub Actions will automatically build the package and create a release

### Setting Up a Personal APT Repository

To make the package available for installation via APT, you can use the included script to set up a personal APT repository on GitHub Pages:

1. Run the setup script:
   ```shell
   ./scripts/setup-apt-repo.sh
   ```
   
2. This creates a repository structure in `~/ros-hacks-apt-repo`

3. Initialize the directory as a Git repository and push to GitHub:
   ```shell
   cd ~/ros-hacks-apt-repo
   git init
   git add .
   git commit -m "Initial repository setup"
   git remote add origin https://github.com/your-github-username/ros-hacks-apt-repo.git
   git push -u origin main
   ```

4. Enable GitHub Pages for the repository (in repository settings)

5. When you have a new package to add to the repository:
   ```shell
   ./scripts/setup-apt-repo.sh add path/to/ros-hacks_*.deb
   ```
