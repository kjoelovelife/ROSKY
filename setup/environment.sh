#!/bin/bash

# setup parameter
current_ip=$(hostname -I | awk '{print $1}')
if [ -z $current_ip ]; then
    current_ip="127.0.0.1"
fi

# setup python path
echo "Setting up PYTHONPATH."
export PYTHONPATH="~/ROSKY/catkin_ws/src":$PYTHONPATH

# setup cv_bridge with python3 and opencv4
#source ~/$main_path/$cvbridge_path/install/setup.bash --extend

# setup ros hostname and source setup.bash 
source ~/ROSKY/catkin_ws/devel/setup.bash

# setup vehicle name
source ~/ROSKY/setup/set_vehicle_name.sh

#exec "$@" #Passes arguments. Need this for ROS remote launching to work.
