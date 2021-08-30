#!/usr/bin/env bash

# -------------------------------------------------------------------------
#Apache License
#
#Copyright (c) 2021 Lin Wei-Chih
#
#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.
#SOFTWARE.--------------------------------------------------------------
#Reference:
##bash echo color: https://misc.flogisoft.com/bash/tip_colors_and_formatting 
##bash if test   : http://linux.vbird.org/linux_basic/0340bashshell-scripts.php
# -------------------------------------------------------------------------

# read password
echo -n "Please enter your password: "
read -s PASSWORD
echo ""

# set parameter
ubuntu_distro=$(grep RELEASE /etc/lsb-release | awk -F '=' {'print $2'})
install_source="install_scripts"
main_path="home/$USER/ROSKY"
## ros information
if [ $ubuntu_distro == '20.04' ]; then
    ros1_distro=noetic
else
    ros1_distro=melodic
fi
 
## hardware 
arch=$(dpkg --print-architecture)
shell=$SHELL | awk -F '/' '{print $NF}'
L4T_VERSION_STRING=$(head -n 1 /etc/nv_tegra_release 2>> /dev/null)
L4T_RELEASE=$(echo $L4T_VERSION_STRING | cut -f 2 -d ' ' | grep -Po '(?<=R)[^;]+' 2>> /dev/null)
L4T_REVISION=$(echo $L$T_VERSION_STRING | cut -f 2 -d ',' | grep -Po '(?<=REVISION: )[^;]+' 2>> /dev/null)



# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel
