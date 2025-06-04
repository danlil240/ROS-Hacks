# ROS-Hacks APT Repository

This is a personal APT repository for the ROS-Hacks package.

## Adding this repository to your system

1. Download the GPG key and set up the keyring:
```bash
wget -O /tmp/ros-hacks.key https://danlil240.github.io/ROS-Hacks/ros-hacks.key
sudo mkdir -p /etc/apt/keyrings
cat /tmp/ros-hacks.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-hacks.gpg
```

2. Add the repository to your sources (using the modern signed-by approach):
```bash
echo "deb [signed-by=/etc/apt/keyrings/ros-hacks.gpg] https://danlil240.github.io/ROS-Hacks stable main" | sudo tee /etc/apt/sources.list.d/ros-hacks.list
```

3. Update and install:
```bash
sudo apt update
sudo apt install ros-hacks
```

## Publishing Instructions

1. Build the package:
```bash
cd ROS-Hacks
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

4. Ensure GitHub Pages is configured in your repository settings to serve from the main branch.
