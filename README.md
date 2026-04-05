# ROSKY

<p align="center">
  <img src="https://github.com/kjoelovelife/ROSKY/blob/master/rosky.jpg" width=280 />
</p>

ROSKY is an open-source educational autonomous robot platform based on the **Nvidia Jetson Nano 4GB**. It uses an omnibot car motor controller and a 160-degree fisheye camera, and is built on **ROS Melodic** to support self-driving development including lane following, obstacle detection, traffic light detection, SLAM, and deep learning inference.

> Inspired by [Duckietown](https://github.com/duckietown) and [NVIDIA JetBot](https://github.com/NVIDIA-AI-IOT/jetbot).

---

## Hardware Requirements

- Nvidia Jetson Nano 4GB Developer Kit
- Ominibot car motor controller
- Fisheye camera (160-degree FOV)
- YDLidar X4 (optional, for SLAM)

## Software Requirements

| Component | Version |
|-----------|---------|
| OS        | Ubuntu 18.04 (Jetson Nano image) |
| ROS       | Melodic (1.0) |
| Python    | 2.7 |
| OpenCV    | 4.3.0 |
| CUDA      | 10.2 |
| PyTorch   | 1.4.0 |

---

## Installation

Clone the repository into your home directory and run the installer:

```bash
git clone https://github.com/kjoelovelife/ROSKY ~/ROSKY
cd ~/ROSKY
bash install.sh
```

The installer will prompt you to optionally install:
- ROS Melodic
- ROSKY Jetson Nano dependencies (OpenCV, PyTorch, device rules)
- SSH key setup

After installation, build the ROS workspace:

```bash
bash ~/ROSKY/install_script/catkin_make.sh
```

Then set up per-robot configuration files (uses `$HOSTNAME` by default):

```bash
source ~/ROSKY/setup/set_package_param.sh
```

---

## Quick Start

```bash
# Source the ROSKY environment (also added to ~/.bashrc during install)
source ~/ROSKY/setup/environment.sh

# Lane following
roslaunch rosky_base lane_following.launch veh:=rosky01

# Manual keyboard control
roslaunch rosky_base teleop_keyboard.launch veh:=rosky01

# Camera intrinsic calibration
roslaunch rosky_base intrinsic_calibration.launch veh:=rosky01
```

---

## Deep Learning

The `deep_learning` package supports three workflows. See [`catkin_ws/src/deep_learning/readme.md`](catkin_ws/src/deep_learning/readme.md) for full instructions.

| Package           | Description |
|-------------------|-------------|
| `img_recognition` | AlexNet-based image classification (e.g. free/blocked) |
| `road_following`  | ResNet18-based regression for path following |
| `self_driving`    | Combines both models for full autonomous driving |

---

## Developer

- [Wei-Chih Lin](mailto:kjoelovelife@gmail.com)
- Contact for hardware: [iCShop](https://www.icshop.com.tw/)
