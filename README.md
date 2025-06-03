# ROS-Hacks APT Repository

This is a personal APT repository for the ROS-Hacks package.

## Adding this repository to your system

1. Add the GPG key:
```bash
wget -qO - https://danlil240.github.io/ros-hacks-repo/ros-hacks.key | sudo apt-key add -
```

2. Add the repository to your sources:
```bash
echo "deb https://danlil240.github.io/ros-hacks-repo stable main" | sudo tee /etc/apt/sources.list.d/ros-hacks.list
```

3. Update and install:
```bash
sudo apt update
sudo apt install ros-hacks
```

## Publishing Instructions

1. Build the package:
```bash
cd ros-hacks
# Create source package
debuild -S -us -uc -i
# Build in a clean environment with pbuilder
sudo pbuilder build --buildresult build ../*.dsc
```

2. Add the built package to the repository:
```bash
scripts/setup-apt-repo.sh add ../ros-hacks_*.deb
```

3. Push the repository to GitHub:
```bash
cd ~/ros-hacks-repo
git add .
git commit -m "Update repository"
git push
```
