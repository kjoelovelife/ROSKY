#!/usr/bin/env bash

git clone https://github.com/adlink-ros/ros_menu ~/ros_menu
cd ~/ros_menu && source ~/ros_menu/install.sh
sudo rosdep update
cd ~/ROSKY/install_script




# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel
