# ROS-Hacks APT Repository

This is a personal APT repository for the ROS-Hacks package.

## Adding this repository to your system

1. Download and add the GPG key:
```bash
wget -qO /tmp/ros-hacks.key https://your-github-username.github.io/ROS-Hacks/ros-hacks.key
sudo mkdir -p /etc/apt/keyrings
cat /tmp/ros-hacks.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-hacks.gpg
```

2. Add the repository to your sources:
```bash
echo "deb [signed-by=/etc/apt/keyrings/ros-hacks.gpg] https://your-github-username.github.io/ROS-Hacks stable main" | sudo tee /etc/apt/sources.list.d/ros-hacks.list
```

3. Update and install:
```bash
sudo apt update
sudo apt install ros-hacks
```


4. After installation, run the setup script:
```bash
ros-hacks-setup
```


## Publishing Instructions

1. Build the package:
```bash
cd ros-hacks
dpkg-buildpackage -us -uc -b
```

2. Add the built package to the repository:
```bash
scripts/setup-apt-repo.sh add ../ros-hacks_*.deb
```

3. Push the repository to GitHub:
```bash
cd ~/ROS-Hacks
git add .
git commit -m "Update repository"
git push
```
