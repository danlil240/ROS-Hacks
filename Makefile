# ROS-Hacks Makefile
# Provides easy installation and setup commands

# Variables
PREFIX ?= $(HOME)/.ros-hacks
INSTALL_DIR = $(PREFIX)
SYSTEM_INSTALL_DIR = /usr/share/ros-hacks

.PHONY: all install uninstall symlinks system-install help clean

# Default target
all: help

# Help information
help:
	@echo "ROS-Hacks Makefile"
	@echo "-----------------"
	@echo "Available targets:"
	@echo "  install         - Install ROS-Hacks to $(INSTALL_DIR)"
	@echo "  system-install  - Install ROS-Hacks system-wide (requires sudo)"
	@echo "  symlinks        - Create symlinks for quick access to scripts"
	@echo "  uninstall       - Remove ROS-Hacks installation"
	@echo "  clean           - Clean up build artifacts"
	@echo
	@echo "Variables:"
	@echo "  PREFIX          - Installation prefix (default: $(PREFIX))"

# Install to user's home directory
install:
	@echo "Installing ROS-Hacks to $(INSTALL_DIR)..."
	@mkdir -p $(INSTALL_DIR)
	@cp -r bin src config install $(INSTALL_DIR)/
	@cp -f README.md VERSION $(INSTALL_DIR)/ 2>/dev/null || true
	@chmod +x $(INSTALL_DIR)/bin/*.sh
	@chmod +x $(INSTALL_DIR)/install/*.sh
	@$(INSTALL_DIR)/install/setup.sh
	@echo "Installation complete. Please source your ~/.bashrc"
	@echo "  source ~/.bashrc"

# Install system-wide
system-install:
	@echo "Installing ROS-Hacks system-wide to $(SYSTEM_INSTALL_DIR)..."
	@sudo mkdir -p $(SYSTEM_INSTALL_DIR)
	@sudo cp -r bin src config install docs $(SYSTEM_INSTALL_DIR)/
	@sudo chmod +x $(SYSTEM_INSTALL_DIR)/bin/*.sh
	@sudo chmod +x $(SYSTEM_INSTALL_DIR)/install/*.sh
	@echo "System installation complete."
	@echo "Please run the setup script:"
	@echo "  $(SYSTEM_INSTALL_DIR)/install/setup.sh"

# Create symlinks for convenience
symlinks:
	@echo "Creating symlinks in /usr/local/bin..."
	@sudo ln -sf $(INSTALL_DIR)/bin/ros-hacks.sh /usr/local/bin/ros-hacks
	@sudo ln -sf $(INSTALL_DIR)/bin/diagnostic.sh /usr/local/bin/ros-diagnostic
	@sudo ln -sf $(INSTALL_DIR)/bin/setup-apt-repo.sh /usr/local/bin/ros-apt-repo
	@echo "Symlinks created."

# Uninstall
uninstall:
	@echo "Uninstalling ROS-Hacks..."
	@if [ -d "$(INSTALL_DIR)" ]; then \
		rm -rf $(INSTALL_DIR); \
		echo "Removed $(INSTALL_DIR)"; \
	fi
	@if [ -d "$(SYSTEM_INSTALL_DIR)" ]; then \
		sudo rm -rf $(SYSTEM_INSTALL_DIR); \
		echo "Removed $(SYSTEM_INSTALL_DIR)"; \
	fi
	@if [ -L "/usr/local/bin/ros-hacks" ]; then \
		sudo rm -f /usr/local/bin/ros-hacks; \
		echo "Removed symlink /usr/local/bin/ros-hacks"; \
	fi
	@if [ -L "/usr/local/bin/ros-diagnostic" ]; then \
		sudo rm -f /usr/local/bin/ros-diagnostic; \
		echo "Removed symlink /usr/local/bin/ros-diagnostic"; \
	fi
	@if [ -L "/usr/local/bin/ros-apt-repo" ]; then \
		sudo rm -f /usr/local/bin/ros-apt-repo; \
		echo "Removed symlink /usr/local/bin/ros-apt-repo"; \
	fi
	@echo "ROS-Hacks has been uninstalled."
	@echo "You may want to manually remove the ROS-Hacks entries from your ~/.bashrc file."

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	@find . -name "*.o" -type f -delete
	@find . -name "*.so" -type f -delete
	@find . -name "*.pyc" -type f -delete
	@find . -name "__pycache__" -type d -exec rm -rf {} +
	@echo "Clean complete."
