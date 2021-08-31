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

# set command for bash
set -e

# read password
echo -n "Please enter your password: "
read -s PASSWORD


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

##set repository
#echo $PASSWORD | sudo add-apt-repository ppa:joseluisblancoc/mrpt-stable

# step 1. setup python3
## pip should be installed
echo $PASSWORD | sudo -S apt install python3-pip

# step 2. Grant your user access to the i2c bus
sudo apt install i2c-tools python3-smbus
echo $PASSWORD | sudo -S usermod -aG i2c $USER

# step 3. git and cmake should be installed 
echo $PASSWORD | sudo -S apt-get install git cmake

# step 4. Install Jetson-inference
## reference: https://github.com/dusty-nv/jetson-inference
### test downloda source file
if test -d /$main_path/setup/jetson-inference ;then
    echo -e "\e[93mjetson-inference exist. Do not download again.\e[0m"
else
    git clone https://github.com/dusty-nv/jetson-inference /$main_path/setup/jetson-inference
fi ###test downloda source file

## test $L4T_VERSION_STRING
if test -z "$L4T_VERSION_STRING" ;then
    echo -e "\e[93mThere is not jetpack on this computer. Skip install jetson-inference\e[0m"
else
    cd /$main_path/setup/jetson-inference 
    git submodule update --init

    ### test build
    if test -d /$main_path/setup/jetson-inference/build ;then
        echo -e "\e[93mPerhaps you had installed jetson-inference. If you want to re-install, please manual installing it.\e[0m"
    else
        ### build from source
        mkdir build
        cd build
        cmake ../
        make
        echo -d "\e[93mYou have to select somthing while installing jetson-inference. Pleas don't leave... \e[0m"
        read -p "Please press \"Enter\" to continue..."
        ### install libraries
        cd ..
        echo $PASSWORD | sudo -S make install 
    fi ### test build      
fi ## test $L4T_VERSION_STRING

#step 5. install ros dependencies	
echo $PASSWORD | sudo -S apt install -y \
        build-essential \
        cmake \
        libavcodec-dev \
        libavformat-dev \
        libavutil-dev \
        libeigen3-dev \
        libglew-dev \
        libgtk2.0-dev \
        libgtk-3-dev \
        libjpeg-dev \
        libpng-dev \
        libpostproc-dev \
        libswscale-dev \
        libtbb-dev \
        libtiff5-dev \
        libv4l-dev \
        libxvidcore-dev \
        libx264-dev \
        libgeographic-dev \
        gstreamer1.0-tools \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-good1.0-dev\
        qt5-default \
        zlib1g-dev \
        'pkg-config' \
	python3-frozendict \
	libxslt1-dev \
	libxml2-dev \
	python3-lxml \
	python3-bs4 \
	python3-tables \
        python3-sklearn \
        python3-rospkg \
        python3-catkin-tools \
        python3-pip \
        python3-dev \
        python3-numpy \
        apt-file \
        iftop \
        atop \
        ntpdate \
        python3-termcolor \
        python3-sklearn \
        libatlas-base-dev \
        python3-dev \
        python3-ipython \
        python3-sklearn \
        python3-smbus \
        libmrpt-dev \
        libopencv-dev \
        mrpt-apps \
        ros-$ros1_distro-slam-gmapping \
        ros-$ros1_distro-map-server \
        ros-$ros1_distro-navigation \
        ros-$ros1_distro-vision-msgs \
        ros-$ros1_distro-image-transport \
        ros-$ros1_distro-image-publisher \
        ros-$ros1_distro-teleop-twist-keyboard \
        ros-$ros1_distro-teb-local-planner \
        ros-$ros1_distro-image-transport \
        ros-$ros1_distro-vision-msgs \
        ros-$ros1_distro-image-publisher \
        ros-$ros1_distro-image-transport \
        ros-$ros1_distro-cv-bridge \
        ros-$ros1_distro-vision-opencv \
        ros-$ros1_distro-image-proc \
        ros-$ros1_distro-cv-bridge \
        ros-$ros1_distro-geographic-info \
        ros-$ros1_distro-mavros-msgs \
        ros-$ros1_distro-camera-info-manager\
        byobu \
        x11vnc



echo $PASSWORD | sudo -S pip3 install rospkg catkin_pkg

#step 6. Create the name "/dev/ydlidar" for YDLIDAR and "/dev/omnibot_car" for omnibot_car
echo "Setup YDLidar X4 , and it information in /$main_path/catkin_ws/src/ydlidar/README.md."
cd /$main_path/catkin_ws/src/ydlidar/startup
echo $PASSWORD | sudo -S chmod 777 ./*
echo $PASSWORD | sudo -S sh initenv.sh

cd /$main_path/catkin_ws/src/ominibot_car/startup
echo $PASSWORD | sudo -S chmod 777 ./*
echo $PASSWORD | sudo -S sh initenv.sh

echo $PASSWORD | sudo -S udevadm control --reload-rules
echo $PASSWORD | sudo -S udevadm trigger

#step 7. Install Jupyter Lab
sudo apt -y install curl dirmngr apt-transport-https lsb-release ca-certificates 'gcc' g++ 'make'
sudo pip3 install ipython pygments traitlets --upgrade
sudo -H pip3 install -U jupyter jupyterlab
jupyter lab --generate-config
passwd=$(python3 -c "from notebook.auth import passwd; print(passwd('$PASSWORD'))")
echo "c.ServerApp.password = '$passwd'" >> /home/$USER/.jupyter/jupyter_lab_config.py
cd /$main_path/setup && python3 /$main_path/setup/create_jupyter_service.py
echo $PASSWORD | sudo -S mv /$main_path/setup/jupyter.service /etc/systemd/system/jupyter.service

#step 8. Install python3 dependencies
sudo -H pip3 install -U jetson-stats ruamel.yaml


#step 9. Active services
echo $PASSWORD | sudo systemctl restart jetson_stats.service
echo $PASSWORD | sudo systemctl enable jupyter
echo $PASSWORD | sudo systemctl start jupyter

echo -e "\e[93mInstall done! Please reboot your machine to active services.\e[0m"

cd /$main_path/install_script

# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel
