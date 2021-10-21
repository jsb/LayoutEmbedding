#!/bin/bash
# This script is intended to install the required dependencies
# and automatically build the project.
# Tested on Ubuntu 20.04 LTS
# Dependency installs
sudo apt install build-essential cmake libgl1-mesa-dev mesa-utils libglfw3 libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev -y
# Actually build project
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
echo "The installation and build process in complete now. Run the script 'run_le.sh' now to reproduce some figures."
