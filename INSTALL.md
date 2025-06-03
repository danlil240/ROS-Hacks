# Installing ROS-Hacks

This guide will help you install ROS-Hacks from the APT repository.

## Installation Steps

1. Add the GPG key:
```bash
wget -O /tmp/ros-hacks.key https://danlil240.github.io/ROS-Hacks/config/keys/ros-hacks.key
sudo mkdir -p /etc/apt/keyrings
sudo gpg --dearmor -o /etc/apt/keyrings/ros-hacks.gpg /tmp/ros-hacks.key
```

2. Add the repository to your sources:
```bash
echo "deb [signed-by=/etc/apt/keyrings/ros-hacks.gpg] https://danlil240.github.io/ROS-Hacks stable main" | sudo tee /etc/apt/sources.list.d/ros-hacks.list
```

3. Update and install:
```bash
sudo apt update
sudo apt install ros-hacks
```

## Verification

After installation, you can verify the installation by running:

```bash
source /usr/share/ros-hacks/bin/ros-hacks.sh
```

Or if you have sourced your .bashrc:

```bash
source ~/.bashrc
```

Then try some commands like:

```bash
wslist    # List available workspaces
domain    # Show current ROS domain ID
```
