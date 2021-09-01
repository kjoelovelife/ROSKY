#!/bin/bash

cd ~/ROSKY/catkin_ws && catkin_make
python3 ~/ROSKY/setup/config_ros_menu.py
echo -n 1 | source ~/.bashrc
