#!/bin/bash

main_path="ROSKY"
work_space="catkin_ws"

cd ~/$main_path/$work_space && catkin_make

echo "source ~/$main_path/setup/environment.sh" >> ~/.bashrc
source ~/.bashrc
