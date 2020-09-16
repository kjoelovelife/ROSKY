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
ros1_install_script="ros_install_$ros1_disrto.sh"
install_source="install_scripts"

### hardware ###
platform="tegra"
kernel=$(uname -a)
shell=`echo $SHELL | awk -F '/' '{print $NF}'`

### install package ###
ros1=false
rosky=false
ssh_setup=false

## file path
main_path="ROSKY"

## Configure power mode
if [[ $kernel =~ $platform ]] ; then
    #echo $PASSWORD | sudo -S nvpmodel -m1 # 5W
    echo $PASSWORD | sudo -S nvpmodel -m0 # 10W
    echo $PASSWORD | sudo -S nvpmodel -q
fi

## install ros
echo -n "Do you want to install ROS automatically? (y/N): "
read ros_install
if [ "$ros_install" '==' "y" ] || [ "$ros_install" '==' "Y" ];
then
    # Install ROS 1 melodic
    ~/$main_path/$install_source/$ros1_install_script
    ros1=true
else
    echo "Skip installing ROS"
fi

## install ROSKY-jetson_nano dependencies
echo -n "Do you use ROSKY-jetson_nano and want to install dependenices? (y/N): "
read ROSKY_jetson_nano
if [ "$ROSKY_jetson_nano" '==' "y" ] || [ "$ROSKY_jetson_nano" '==' "Y" ];
then
    echo "Do not leave your seat!! Some package you need to chek...."
    sleep 5s
    # Install jetson-inference
    ~/$main_path/$install_source/rosky_jetson_nano_dependiences.sh
    rosky=true
else
    echo "Skip installing ROSKY-jetson_nano dependencies"
fi


## setup ssh
echo -n "Do you want to setup ssh? (y/N): "
read ssh_
if [ "$ssh_" '==' "y" ] || [ "$ssh_" '==' "Y" ];
then
    # setup ssh
    ~/$main_path/$install_source/ssh_setup.sh
    ssh_setup=true
else
    echo "Skip setup ssh."
fi

## install done
echo "install done. You install :"
echo ""

## Show which package install
if [ $ros1 == true ] ; then
    echo "ROS , version : $ros1_distro ."
fi

if [ $rosky == true ] ; then
    echo "ROSKY dependenices."
fi

if [ $ssh_setup == true ] ; then
    echo "SSH."
fi
