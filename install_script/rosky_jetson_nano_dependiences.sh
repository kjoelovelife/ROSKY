#!/usr/bin/env bash

# Shell script scripts to install useful tools , such as opencv , pytorch...
# -------------------------------------------------------------------------
#Copyright © 2019 Wei-Chih Lin , kjoelovelife@gmail.com 

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.
# -------------------------------------------------------------------------
# reference
# https://chtseng.wordpress.com/2019/05/01/nvida-jetson-nano-%E5%88%9D%E9%AB%94%E9%A9%97%EF%BC%9A%E5%AE%89%E8%A3%9D%E8%88%87%E6%B8%AC%E8%A9%A6/
#
# -------------------------------------------------------------------------


## set parameter

### ros information ###
ros1_distro=melodic
install_source="install_scripts"
main_path="ROSKY"

### hardware ###
platform="tegra"
kernel=$(uname -a)
shell=`echo $SHELL | awk -F '/' '{print $NF}'`

#=========step 1. setup python and python3 =========#
echo $PASSWORD | sudo -S apt-get install python-pip
echo $PASSWORD | sudo -S apt-get install python3-pip
#====================================================

#=========step 2. Grant your user access to the i2c bus =======================
# pip should be installed
echo $PASSWORD | sudo -S usermod -aG i2c $USER
#==============================================================================

#=========step 3. Download jetson-inference and jetbot_ros ===================================
# git and cmake should be installed
echo $PASSWORD | sudo -S apt-get install git cmake

# get workspace

if [[ $kernel =~ $platform ]] ; then
    # clone the repo and submodules
    git clone https://github.com/dusty-nv/jetson-inference ./jetson-inference
    cd jetson-inference
    git submodule update --init

    # build from source
    mkdir build
    cd build
    cmake ../
    make

    # install libraries
    echo $PASSWORD | sudo -S make install
    cd ~/$main_path/
    #git clone https://github.com/dusty-nv/jetbot_ros ~/$main_path/catkin_ws/src/jetbot_ros
fi
#===========================================================================

#=========step 4. install ros dependencies =================================
# install dependencies 
echo $PASSWORD | sudo -S apt install -y \
	python-frozendict \
	libxslt-dev \
	libxml2-dev \
	python-lxml \
	python-bs4 \
	python-tables \
        python-sklearn \
        python-rospkg \
        apt-file \
        iftop \
        atop \
        ntpdate \
        python-termcolor \
        python-sklearn \
        libatlas-base-dev \
        python-dev \
        ipython \
        python-sklearn \
        python-smbus \
        libmrpt-dev \
        mrpt-apps \
        ros-$ros1_distro-slam-gmapping \
        ros-$ros1_distro-map-server \
        ros-$ros1_distro-navigation \
        ros-$ros1_distro-vision-msgs \
        ros-$ros1_distro-image-transport \
        ros-$ros1_distro-image-publisher \
        ros-$ros1_distro-teleop-teist-keyboard \
        byobu
#===========================================================================

#======  step 5. Create the name "/dev/ydlidar" for YDLIDAR and "/dev/omnibot_car" for omnibot_car  ===============
# install dependencies 
echo "Setup YDLidar X4 , and it information in ~/${workspace}/catkin_ws/src/ydlidar/README.md "
cd ~/$main_path/catkin_ws/src/ydlidar/startup
echo $PASSWORD | sudo -S chmod 777 ./*
echo $PASSWORD | sudo -S sh initenv.sh

cd ~/$main_path/catkin_ws/src/omnibot_car/startup
echo $PASSWORD | sudo -S chmod 777 ./*
echo $PASSWORD | sudo -S sh initenv.sh

echo $PASSWORD | sudo -S udevadm control --reload-rules
echo $PASSWORD | sudo -S udevadm trigger
#===========================================================================


# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel
