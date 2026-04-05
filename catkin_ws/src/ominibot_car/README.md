# ominibot_car

Motor driver and kinematics package for the omnibot car platform. Converts high-level `CarControl` commands into per-wheel velocity commands and communicates with the motor controller over a serial interface.

## Nodes

| Node | Script | Description |
|------|--------|-------------|
| `wheels_ominibot_driver_node` | `src/wheels_ominibot_driver_node.py` | Main driver; sends wheel commands to the hardware via serial |
| `wheels_driver_node` | `src/wheels_driver_node.py` | Alternative driver for Dagu-style hardware |
| `wheels_trimmer_node` | `src/wheels_trimmer_node.py` | Applies per-wheel trim offsets to compensate for motor asymmetry |
| `car_cmd_switch_node` | `src/car_cmd_switch_node.py` | Multiplexes between autonomous and manual control sources |
| `wheels_cmd_switch_node` | `src/wheels_cmd_switch_node.py` | Lower-level command multiplexer |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~car_control` | `rosky_msgs/CarControl` | Speed and steering from `lane_control` or teleop |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~wheels_cmd` | `rosky_msgs/WheelsCmdStamped` | Individual wheel velocity commands |

## Hardware Interface

The `include/ominibot_car/ominibot_car_com.py` library handles serial communication with the motor controller. The device is expected at `/dev/omnibot_car` (a udev symlink set up during installation by running `startup/initenv.sh`).

To set up the udev rule manually:

```bash
cd ~/ROSKY/catkin_ws/src/ominibot_car/startup
sudo chmod 777 ./*
sudo sh initenv.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Kinematics Calibration

Wheel radius, baseline, and trim are stored in `rosky_base/config/baseline/calibration/kinematics/<hostname>.yaml`. Adjust trim values if the robot drifts when commanded to drive straight.
