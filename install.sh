#!/usr/bin/env bash

# Main installer for ROSKY. Prompts for which components to install, then
# delegates to the individual scripts in install_script/.
# -------------------------------------------------------------------------
# Copyright © 2019 Wei-Chih Lin, kjoelovelife@gmail.com
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# -------------------------------------------------------------------------

set -euo pipefail

# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------

### ROS ###
ros1_distro="melodic"
ros1_install_script="ros_install_${ros1_distro}.sh"
install_source="install_script"

### Hardware detection ###
platform="tegra"
kernel=$(uname -a)

### Resolved paths ###
# Assume the repo lives at ~/ROSKY (the expected install location).
main_path="${HOME}/ROSKY"

### Install flags ###
install_ros1=false
install_rosky=false
setup_ssh=false

# ---------------------------------------------------------------------------
# Jetson-specific: set max performance mode (10W)
# ---------------------------------------------------------------------------
if [[ $kernel =~ $platform ]]; then
    sudo nvpmodel -m0  # 10W mode
    sudo nvpmodel -q
fi

# ---------------------------------------------------------------------------
# Prompt for components
# ---------------------------------------------------------------------------
echo ""
read -rp "Do you want to install ROS automatically? (y/N): " ros_answer
if [[ "$ros_answer" == "y" || "$ros_answer" == "Y" ]]; then
    install_ros1=true
else
    echo "Skipping ROS installation."
fi

read -rp "Do you use ROSKY-jetson_nano and want to install dependencies? (y/N): " rosky_answer
if [[ "$rosky_answer" == "y" || "$rosky_answer" == "Y" ]]; then
    echo "Ensure you have an internet connection — packages will be downloaded."
    install_rosky=true
else
    echo "Skipping ROSKY Jetson Nano dependencies."
fi

read -rp "Do you want to set up SSH? (y/N): " ssh_answer
if [[ "$ssh_answer" == "y" || "$ssh_answer" == "Y" ]]; then
    setup_ssh=true
else
    echo "Skipping SSH setup."
fi

echo ""
echo "Starting installation — do not leave your seat, some steps require interaction."
sleep 3

# ---------------------------------------------------------------------------
# Run selected install scripts
# ---------------------------------------------------------------------------
installed=()

if [[ "$install_ros1" == true ]]; then
    bash "${main_path}/${install_source}/${ros1_install_script}"
    installed+=("ROS ${ros1_distro}")
fi

if [[ "$install_rosky" == true ]]; then
    bash "${main_path}/${install_source}/rosky_jetson_nano_dependiences.sh"
    installed+=("ROSKY Jetson Nano dependencies")
fi

if [[ "$setup_ssh" == true ]]; then
    bash "${main_path}/${install_source}/ssh_setup.sh"
    installed+=("SSH")
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
if [[ ${#installed[@]} -eq 0 ]]; then
    echo "Nothing was installed."
else
    echo "Installed:"
    for item in "${installed[@]}"; do
        echo "  - $item"
    done
fi
