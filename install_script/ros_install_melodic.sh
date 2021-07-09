#!/usr/bin/env bash
echo -n "Do you want to install ROS-melodic? (y/N): "
read ros_install
echo -n "Please enter your password: "
read -s PASSWORD
echo ""
if [ "$ros_install" '==' "y" ] || [ "$ros_install" '==' "Y" ]; then
    ros1_distro=melodic
    #================= step 1. Install ROS melodic =================================
    # enable all Ubuntu packages:
    echo $PASSWORD | sudo -S apt-add-repository universe
    echo $PASSWORD | sudo -S apt-add-repository multiverse
    echo $PASSWORD | sudo -S apt-add-repository restricted
    # add ROS repository to apt sources
    sudo apt update && sudo apt install -y curl gnupg2 lsb-release
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    ## delete ROS old key : sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116 
    # install ROS desktop-full
    sudo apt-get update
    sudo apt install -y ros-$ros1_distro-desktop-full
    sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    sudo apt install python-rosdep
    sudo rosdep init
    rosdep update
    sudo apt-get install -y python-rosinstall \
                            python-rosinstall-generator \
                            python-wstool \
                            build-essential
    ##  configure variable about ROS in ~/.bashrc
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    echo $PASSWORD | sudo chown -R $USER ~/.ros
    echo "ROS $ros1_distro installed successfully." 
    #==============================================================================
else
    echo -n "Skip install ROS Melodic."
fi




# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel
