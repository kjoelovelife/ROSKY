#!/bin/bash

## setup parameter
current_ip=$(hostname -I | awk '{print $1}')
if [ -z $current_ip ]; then
    current_ip="127.0.0.1"
fi
ros_default_master_uri=$current_ip
main_path="ROSKY"
work_space="catkin_ws/src"

## clear ROS env var for resolving warning #
unset ros_option
unset ROS_DISTRO
unset ROS_HOSTNAME


## setup ros
echo "Activating ROS..."
source /opt/ros/melodic/setup.bash
echo "...done."

## setup python path
echo "Setting up PYTHONPATH."
export PYTHONPATH=~/$main_path/$work_space:$PYTHONPATH

## setup ros hostname and source seyup.bash 
echo "Setup ROS_HOSTNAME. Now your ip adress : $current_ip "
export ROS_HOSTNAME=$HOSTNAME.local
source ~/$main_path/catkin_ws/devel/setup.bash


# TODO: check that the time is >= 2015

# TODO: run a python script that checks all libraries are installed

exec "$@" #Passes arguments. Need this for ROS remote launching to work.
