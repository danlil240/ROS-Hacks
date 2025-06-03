#!/usr/bin/env bash

# ==========================================================
# ROS-Hacks - APT Repository Management Functions
# ==========================================================

# Set up APT repository for ROS-Hacks packages
# Usage: setup_apt_repo <repo_dir>
function setup_apt_repo() {
    local repo_dir=${1:-$(pwd)}
    
    # Ensure required directories exist
    mkdir -p "${repo_dir}/dists/stable/main/binary-amd64"
    mkdir -p "${repo_dir}/dists/stable/main/binary-i386"
    mkdir -p "${repo_dir}/pool/main"
    
    # Create empty Packages files for both architectures
    touch "${repo_dir}/dists/stable/main/binary-amd64/Packages"
    touch "${repo_dir}/dists/stable/main/binary-i386/Packages"
    
    # Compress the Packages files
    gzip -9 -c "${repo_dir}/dists/stable/main/binary-amd64/Packages" > "${repo_dir}/dists/stable/main/binary-amd64/Packages.gz"
    gzip -9 -c "${repo_dir}/dists/stable/main/binary-i386/Packages" > "${repo_dir}/dists/stable/main/binary-i386/Packages.gz"
    
    # Create Release file with proper hash entries
    # Use a numeric date format (YYYYMMDD) that APT definitely accepts
    local formatted_date=$(LC_ALL=C TZ=UTC date +"%Y%m%d")
    
    cat > "${repo_dir}/dists/stable/Release" << EOF
Origin: ROS-Hacks
Label: ROS-Hacks
Suite: stable
Codename: stable
Version: 1.0
Architectures: amd64 i386
Components: main
Description: ROS-Hacks APT Repository
Date: ${formatted_date}
EOF
    
    # Generate hash entries for Release file
    (
        cd "${repo_dir}/dists/stable"
        echo "MD5Sum:" >> Release
        for f in $(find . -type f | grep -v "Release"); do
            echo " $(md5sum $f | cut -d' ' -f1) $(stat -c%s $f) $f" >> Release
        done
        echo "SHA1:" >> Release
        for f in $(find . -type f | grep -v "Release"); do
            echo " $(sha1sum $f | cut -d' ' -f1) $(stat -c%s $f) $f" >> Release
        done
        echo "SHA256:" >> Release
        for f in $(find . -type f | grep -v "Release"); do
            echo " $(sha256sum $f | cut -d' ' -f1) $(stat -c%s $f) $f" >> Release
        done
    )
    
    printf "${GREEN_TXT}APT repository structure created at ${WHITE_TXT}${repo_dir}${NC}\n"
    
    # Generate GPG key if it doesn't exist
    if [[ ! -f "${repo_dir}/ros-hacks.key" ]]; then
        printf "${BLUE_TXT}Generating GPG key for repository signing...${NC}\n"
        gpg --batch --gen-key <<EOF
Key-Type: RSA
Key-Length: 4096
Name-Real: ROS-Hacks
Name-Email: ros-hacks@example.com
Expire-Date: 0
%no-protection
%commit
EOF
        
        # Export public key
        gpg --armor --export "ROS-Hacks" > "${repo_dir}/ros-hacks.key"
        printf "${GREEN_TXT}GPG key generated and exported to ${WHITE_TXT}${repo_dir}/ros-hacks.key${NC}\n"
    fi
    
    # Sign the Release file
    gpg --clearsign -o "${repo_dir}/dists/stable/InRelease" "${repo_dir}/dists/stable/Release"
    gpg -abs -o "${repo_dir}/dists/stable/Release.gpg" "${repo_dir}/dists/stable/Release"
    
    printf "${GREEN_TXT}Repository files signed successfully.${NC}\n"
    printf "${BLUE_TXT}APT repository setup complete.${NC}\n"
    
    # Print installation instructions
    printf "\n${BLUE_TXT}Installation Instructions:${NC}\n"
    printf "${WHITE_TXT}To use this repository, run the following commands:${NC}\n\n"
    printf "wget -O /tmp/ros-hacks.key https://danlil240.github.io/ROS-Hacks/ros-hacks.key\n"
    printf "sudo mkdir -p /etc/apt/keyrings\n"
    printf "sudo gpg --dearmor -o /etc/apt/keyrings/ros-hacks.gpg /tmp/ros-hacks.key\n"
    printf "echo \"deb [signed-by=/etc/apt/keyrings/ros-hacks.gpg] https://danlil240.github.io/ROS-Hacks stable main\" | sudo tee /etc/apt/sources.list.d/ros-hacks.list\n"
    printf "sudo apt update\n"
    printf "\n"
}

# Add a Debian package to the repository
# Usage: add_package_to_repo <package_file> <repo_dir>
function add_package_to_repo() {
    local pkg_file=${1:-""}
    local repo_dir=${2:-$(pwd)}
    
    if [[ -z "${pkg_file}" ]]; then
        printf "${RED_TXT}Package file not specified.${NC}\n"
        return 1
    fi
    
    if [[ ! -f "${pkg_file}" ]]; then
        printf "${RED_TXT}Package file ${pkg_file} does not exist.${NC}\n"
        return 1
    fi
    
    # Copy package to pool directory
    local pkg_name=$(basename "${pkg_file}")
    cp "${pkg_file}" "${repo_dir}/pool/main/"
    
    # Update Packages file
    (
        cd "${repo_dir}"
        dpkg-scanpackages --multiversion "pool/main" > "dists/stable/main/binary-amd64/Packages"
        gzip -9 -c "dists/stable/main/binary-amd64/Packages" > "dists/stable/main/binary-amd64/Packages.gz"
    )
    
    # Update Release file with new hashes
    update_release_file "${repo_dir}"
    
    printf "${GREEN_TXT}Package ${WHITE_TXT}${pkg_name}${GREEN_TXT} added to repository.${NC}\n"
    printf "${BLUE_TXT}Repository updated successfully.${NC}\n"
}

# Update the Release file with current hashes
# Usage: update_release_file <repo_dir>
function update_release_file() {
    local repo_dir=${1:-$(pwd)}
    
    # Update timestamp with numeric YYYYMMDD format
    local formatted_date=$(LC_ALL=C TZ=UTC date +"%Y%m%d")
    sed -i "s/Date: .*/Date: ${formatted_date}/" "${repo_dir}/dists/stable/Release"
    
    # Remove old hash entries
    sed -i '/MD5Sum:/,$d' "${repo_dir}/dists/stable/Release"
    
    # Generate new hash entries
    (
        cd "${repo_dir}/dists/stable"
        echo "MD5Sum:" >> Release
        for f in $(find . -type f | grep -v "Release"); do
            echo " $(md5sum $f | cut -d' ' -f1) $(stat -c%s $f) $f" >> Release
        done
        echo "SHA1:" >> Release
        for f in $(find . -type f | grep -v "Release"); do
            echo " $(sha1sum $f | cut -d' ' -f1) $(stat -c%s $f) $f" >> Release
        done
        echo "SHA256:" >> Release
        for f in $(find . -type f | grep -v "Release"); do
            echo " $(sha256sum $f | cut -d' ' -f1) $(stat -c%s $f) $f" >> Release
        done
    )
    
    # Sign the Release file
    gpg --clearsign -o "${repo_dir}/dists/stable/InRelease" "${repo_dir}/dists/stable/Release"
    gpg -abs -o "${repo_dir}/dists/stable/Release.gpg" "${repo_dir}/dists/stable/Release"
}

# Update the entire repository
# Usage: update_repo <repo_dir>
function update_repo() {
    local repo_dir=${1:-$(pwd)}
    
    printf "${BLUE_TXT}Updating APT repository...${NC}\n"
    
    # Ensure directory structure exists
    mkdir -p "${repo_dir}/dists/stable/main/binary-amd64"
    mkdir -p "${repo_dir}/dists/stable/main/binary-i386"
    mkdir -p "${repo_dir}/pool/main"
    
    # Scan packages and update Packages files
    printf "${BLUE_TXT}Scanning packages...${NC}\n"
    (
        cd "${repo_dir}"
        dpkg-scanpackages --multiversion "pool/main" > "dists/stable/main/binary-amd64/Packages"
        # Create a copy for i386 to prevent architecture warnings
        touch "dists/stable/main/binary-i386/Packages"
        
        # Compress the Packages files
        gzip -9 -c "dists/stable/main/binary-amd64/Packages" > "dists/stable/main/binary-amd64/Packages.gz"
        gzip -9 -c "dists/stable/main/binary-i386/Packages" > "dists/stable/main/binary-i386/Packages.gz"
    )
    
    # Update the Release file with new hashes
    update_release_file "${repo_dir}"
    
    printf "${GREEN_TXT}Repository updated successfully.${NC}\n"
}
