#!/bin/bash

# This script installs dependencies and builds the project on Debian-based platforms.
# Tested on Ubuntu 20.04 LTS.

# Exit on errors.
set -o errexit

# Install dependencies.
sudo apt install build-essential cmake libgl1-mesa-dev mesa-utils libglfw3 libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev -y

# Build the project.
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4

echo ""
echo "Installation and buld completed. Run the script 'grsi-run.sh' to replicate a representative figure."

