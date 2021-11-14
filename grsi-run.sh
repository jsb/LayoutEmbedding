#!/bin/bash

# Run this script after the project has been built (using the grsi-install.sh script or by following the instructions in README.md).
# This script reproduces a representative figure 

# Exit on errors.
set -o errexit

# Run the program.
cd build
./pig_figure

echo ""
echo "Images replicating Fig. 1 from the paper can be found in $(pwd)/pig_figure"
