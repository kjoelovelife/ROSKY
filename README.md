# [ROSKY](https://www.icshop.com.tw/product-page.php?28182)

Four wheels and ROS1 robot with Jetson nano 4G Developer kit.

<p align="center">
  <img src="https://github.com/kjoelovelife/ROSKY/blob/noetic/rosky.jpg" width=280 />
</p>

## Information
----
- Single Computer Board(SBD): Jetson Nano 4G Developer Kit
- Operating System: [Xubuntu 20.04 Focal Fossa L4T R32.3.1](https://forums.developer.nvidia.com/t/xubuntu-20-04-focal-fossa-l4t-r32-3-1-custom-image-for-the-jetson-nano/121768)
- Framework:

-- Robot Operating System(ROS) - Noetic

-- Robot Operating System 2(ROS2) - Foxy

- Python: 3.8 

## Developer
----
- [Joe, Lin](weichih.lin@protonmail.com)

## Installation
----
Please use the command below to auto-install ROS1, ROS2 and ros_menu from [Adlink-ROS](https://github.com/Adlink-ROS/ros_menu):

```
source ~/ROSKY/install_script/ros_install.sh
```

After install ROS1, ROS2 and ros_menu, please use the command below to install dependencies and configure environment for using project ROSKY:

```
source ~/ROSKY/install_scripts/rosky_jetson_nano_dependiences.sh 
```

Then you can reboot machine to finish the installation.

## How to use ROS2 - ROS1 bridge
----
Open a new terminal and select option 1: ROS 1 noetic

<p align="center">
  <img src="https://github.com/kjoelovelife/ROSKY/blob/noetic/readme_resource/select_ROS1.png" width=280 />
</p>

And then, you can start the Master Node through launching a file or running ```roscore```.

<p align="center">
  <img src="https://github.com/kjoelovelife/ROSKY/blob/noetic/readme_resource/master_node.png" width=280 />
</p>

Open another terminal throught pressing [ctrl] + [shift] + [T] in the same terminal, then will open another terminal pagination.
In this pagination, select option 3 to auto-start package ROS2-ROS1 bridge.

<p align="center">
  <img src="https://github.com/kjoelovelife/ROSKY/blob/noetic/readme_resource/ROS2_ROS1_bridge.png" width=280 />
</p>

Awesome! Now you can use ROS2 to communicate ROS1 package!

There is a example on ROSKY: use ROS1 to run gmapping.launch, and then use ROS2 and rviz2 to get the map!

<p align="center">
  <img src="https://github.com/kjoelovelife/ROSKY/blob/noetic/readme_resource/ROS2_ROS1_bridge_example.png"/>
</p>
